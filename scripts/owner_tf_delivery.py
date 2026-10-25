"""Owner TF observations and independently serviced localization ingestion.

Receipt means callback entry, NOT DDS transport receipt. No callback disk IO;
TF insertion retains the standard listener implementation.
"""
import copy
import time
from collections import deque
from functools import wraps
from threading import Event, RLock, Thread


def stamp_seconds(transform):
    stamp = transform.header.stamp
    return stamp.sec + stamp.nanosec * 1e-9


class TFDelivery:
    def __init__(self):
        self.lock = RLock()
        self.coherent = deque(maxlen=128)
        self.edges = {edge: dict(callbacks=0, received_stamp=None, callback_wall=None,
                                inserted_latest_stamp=None, insertion_wall=None)
                      for edge in ('map->odom', 'odom->base_link')}
        self.callback_count = 0
        self.spin_count = 0
        self.last_spin_start = self.last_spin_end = None
        self.last_between_spins_s = self.last_spin_duration_s = None
        self.max_between_spins_s = self.max_spin_duration_s = 0.

    def spin_start(self):
        now = time.monotonic()
        if self.last_spin_end is not None:
            self.last_between_spins_s = now-self.last_spin_end
            self.max_between_spins_s = max(self.max_between_spins_s, self.last_between_spins_s)
        self.last_spin_start = now
        self.spin_count += 1

    def spin_end(self):
        self.last_spin_end = time.monotonic()
        self.last_spin_duration_s = self.last_spin_end-self.last_spin_start
        self.max_spin_duration_s = max(self.max_spin_duration_s, self.last_spin_duration_s)

    def snapshot(self, buffer):
        with self.lock:
            return self._snapshot(buffer)

    def _snapshot(self, buffer):
        from rclpy.time import Time
        result = copy.deepcopy(dict(edges=self.edges, callback_count=self.callback_count,
            spin_count=self.spin_count, last_spin_start=self.last_spin_start,
            last_spin_end=self.last_spin_end, max_between_spins_s=self.max_between_spins_s,
            last_between_spins_s=self.last_between_spins_s, last_spin_duration_s=self.last_spin_duration_s,
            max_spin_duration_s=self.max_spin_duration_s,
            receipt_semantics='listener callback entry; DDS receipt unavailable',
            insertion_semantics='buffer latest after unchanged listener callback; completion time is an upper bound'))
        result['buffer_latest_at_fault'] = {}
        for edge in self.edges:
            parent, child = edge.split('->')
            try:
                result['buffer_latest_at_fault'][edge] = stamp_seconds(buffer.lookup_transform(parent, child, Time()))
            except Exception as exc:
                result['buffer_latest_at_fault'][edge] = dict(exception=repr(exc))
        try:
            result['latest_common_stamp'] = stamp_seconds(buffer.lookup_transform('map', 'base_link', Time()))
        except Exception as exc:
            result['latest_common_stamp'] = None
            result['lookup_exception'] = repr(exc)
        return result


def observed_listener_type():
    # Lazy import keeps the offline input checker independent of ROS initialization.
    from tf2_ros import TransformListener
    from rclpy.time import Time

    class ObservedListener(TransformListener):
        def __init__(self, buffer, node, delivery):
            self.delivery = delivery
            super().__init__(buffer, node)

        def callback(self, data):
            with self.delivery.lock:
                self._callback(data)

        def static_callback(self, data):
            with self.delivery.lock:
                super().static_callback(data)

        def _callback(self, data):
            entered = time.monotonic()
            self.delivery.callback_count += 1
            touched = set()
            for tr in data.transforms:
                edge = tr.header.frame_id + '->' + tr.child_frame_id
                if edge in self.delivery.edges:
                    touched.add(edge)
                    item = self.delivery.edges[edge]
                    item.update(received_stamp=stamp_seconds(tr), callback_wall=entered,
                                callbacks=item['callbacks']+1)
            # Exactly the original insertion path, authority, order and exceptions.
            super().callback(data)
            completed = time.monotonic()
            try:
                common = self.buffer.lookup_transform('map', 'base_link', Time())
            except Exception:
                common = None
            if common is not None and (not self.delivery.coherent or
                    stamp_seconds(common) != self.delivery.coherent[-1][0]):
                self.delivery.coherent.append((stamp_seconds(common), completed, common))
            for edge in touched:
                parent, child = edge.split('->')
                try:
                    latest = stamp_seconds(self.buffer.lookup_transform(parent, child, Time()))
                except Exception:
                    latest = None
                self.delivery.edges[edge].update(inserted_latest_stamp=latest, insertion_wall=completed)

    return ObservedListener


class LocalizationIngress:
    """One dedicated node, owned by exactly one SingleThreadedExecutor.

    Worker only mutates TF and a bounded odom mailbox. Monitor histories, failure
    decisions, commands and evidence remain on the original owner thread.
    """
    def __init__(self, *, context=None):
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from nav_msgs.msg import Odometry
        from tf2_ros import Buffer
        self.delivery = TFDelivery()
        self.buffer = Buffer()
        self.odometry = deque(maxlen=128)
        self.failure = None
        self.last_odom_stamp = None
        self.stop = Event()
        self.node = Node('warehouse_owner_localization', context=context,
                         enable_rosout=False, start_parameter_services=False)
        self.executor = SingleThreadedExecutor(context=self.node.context)
        try:
            self.listener = observed_listener_type()(self.buffer, self.node, self.delivery)
            self.node.create_subscription(Odometry, '/slam/odom', self.receive_odom,
                                          qos_profile_sensor_data)
            self.executor.add_node(self.node)
            self.thread = Thread(target=self.run, name='owner-localization', daemon=True)
            self.thread.start()
        except Exception:
            self.executor.shutdown(timeout_sec=2.)
            self.node.destroy_node()
            raise

    def receive_odom(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        with self.delivery.lock:
            if self.last_odom_stamp is not None and stamp < self.last_odom_stamp:
                self.failure = self.failure or 'lio_stamp_regression'
            # Equal stamps cannot renew the first receipt's freshness deadline.
            if stamp != self.last_odom_stamp:
                self.odometry.append((stamp, time.monotonic(), msg))
            self.last_odom_stamp = stamp

    def run(self):
        try:
            while not self.stop.is_set():
                self.executor.spin_once(timeout_sec=.02)
        except Exception as exc:
            with self.delivery.lock:
                self.failure = self.failure or 'localization_executor:' + repr(exc)

    def transfer(self, monitor):
        # Caller holds delivery.lock. Do not call any Monitor function on worker.
        if not self.thread.is_alive() and not self.stop.is_set():
            self.failure = self.failure or 'localization_executor_stopped'
        monitor.failure = monitor.failure or self.failure
        while self.odometry:
            _, receipt, msg = self.odometry.popleft()
            monitor.receive_odom(msg, receipt_wall=receipt)

    def close(self):
        self.stop.set()
        self.executor.wake()
        self.thread.join(timeout=2.)
        if self.thread.is_alive():
            raise RuntimeError('localization_executor_shutdown_timeout')
        self.executor.shutdown(timeout_sec=2.)
        self.executor.remove_node(self.node)
        self.listener.unregister()
        self.node.destroy_node()


def localization_decision(method):
    """Freeze TF insertion + odom transfer for one owner decision (reentrant)."""
    @wraps(method)
    def locked(monitor, *args, **kwargs):
        delivery = getattr(monitor, 'tf_delivery', None)
        if delivery is None:
            return method(monitor, *args, **kwargs)
        with delivery.lock:
            ingress = getattr(monitor, 'ingress', None)
            if ingress is not None:
                ingress.transfer(monitor)
            return method(monitor, *args, **kwargs)
    return locked
