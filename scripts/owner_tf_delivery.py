"""Bounded observations on the existing, single-executor Owner TF listener.

Receipt means callback entry, NOT DDS transport receipt. No disk IO or TF mutation.
"""
import copy
import time


def stamp_seconds(transform):
    stamp = transform.header.stamp
    return stamp.sec + stamp.nanosec * 1e-9


class TFDelivery:
    def __init__(self):
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
            for edge in touched:
                parent, child = edge.split('->')
                try:
                    latest = stamp_seconds(self.buffer.lookup_transform(parent, child, Time()))
                except Exception:
                    latest = None
                self.delivery.edges[edge].update(inserted_latest_stamp=latest, insertion_wall=completed)

    return ObservedListener
