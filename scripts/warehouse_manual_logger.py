#!/usr/bin/python3
"""Record public /map and map->base_link TF for a manual zero-motion viewer."""
import argparse
import csv
import hashlib
import json
import math
import signal
import time
from datetime import datetime, timezone
from pathlib import Path


def yaw_of(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def render_track(path, grid, rows):
    import numpy as np
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.colors import ListedColormap, BoundaryNorm
    from matplotlib.transforms import Affine2D

    figure = Figure(figsize=(10, 8)); FigureCanvasAgg(figure); axis = figure.subplots()
    if grid:
        width, height = grid['width'], grid['height']
        cells = np.asarray(grid['data'], dtype=np.int16).reshape((height, width))
        raster = np.where(cells < 0, 101, cells)
        colors = [(v, v, v, 1.) for v in np.linspace(1., .05, 101)] + [(.68, .72, .76, 1.)]
        cmap = ListedColormap(colors)
        ox, oy, yaw = grid['origin']
        axis.imshow(raster, origin='lower', interpolation='nearest', cmap=cmap,
                    norm=BoundaryNorm(np.arange(-.5, 102.5), cmap.N),
                    extent=[0, width*grid['resolution'], 0, height*grid['resolution']],
                    transform=Affine2D().rotate(yaw).translate(ox, oy) + axis.transData,
                    zorder=0)
    else:
        axis.text(.5, .5, 'Public /map unavailable', ha='center', va='center', transform=axis.transAxes)

    segments = []
    for row in rows:
        if (not segments or row['timestamp_ns'] <= segments[-1][-1]['timestamp_ns'] or
                row['timestamp_ns'] - segments[-1][-1]['timestamp_ns'] > 2_000_000_000):
            segments.append([])
        segments[-1].append(row)
    for index, segment in enumerate(segments):
        xy = np.asarray([[row['x'], row['y']] for row in segment])
        if len(xy) == 1:
            axis.scatter(xy[:, 0], xy[:, 1], color=f'C{index % 10}', s=18,
                         label='Actual TF trajectory' if index == 0 else None, zorder=4)
        else:
            axis.plot(xy[:, 0], xy[:, 1], '.-', color=f'C{index % 10}', linewidth=1.5,
                      markersize=2, label='Actual TF trajectory' if index == 0 else None, zorder=4)
    if rows:
        first, last = rows[0], rows[-1]
        axis.scatter(first['x'], first['y'], marker='*', s=150, color='#239b56',
                     edgecolor='black', label='Recorded start', zorder=5)
        axis.scatter(last['x'], last['y'], marker='X', s=80, color='#ff8c00',
                     edgecolor='black', label='Recorded end', zorder=6)
    axis.set(xlabel='map x (m)', ylabel='map y (m)', title='Warehouse map and measured trajectory')
    axis.set_aspect('equal'); axis.grid(alpha=.15)
    if rows:
        axis.legend(loc='best', fontsize=8)
    figure.tight_layout(); figure.savefig(path, dpi=150); figure.clear()


class ManualLogger:
    def __init__(self, output):
        import rclpy
        from rclpy.clock import Clock, ClockType
        from rclpy.parameter import Parameter
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rclpy.time import Time
        from nav_msgs.msg import OccupancyGrid, Path as PathMsg
        from geometry_msgs.msg import Point, PoseStamped
        from visualization_msgs.msg import Marker, MarkerArray
        from tf2_ros import Buffer, TransformListener, TransformException

        self.rclpy, self.Time, self.PathMsg = rclpy, Time, PathMsg
        self.Marker, self.MarkerArray = Marker, MarkerArray
        self.Point = Point
        self.TransformException = TransformException
        self.node = Node('warehouse_manual_track_logger', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map = None; self.map_error = None; self.rows = []; self.path_msg = PathMsg()
        self.map_stamp_ns = None
        self.path_msg.header.frame_id = 'map'
        self.epoch = 0; self.segment = 0; self.last_sim_ns = None
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self.node)
        self.path_pub = self.node.create_publisher(PathMsg, '/course_manual/trajectory', transient)
        self.markers_pub = self.node.create_publisher(MarkerArray, '/course_manual/trajectory_markers', transient)
        self.node.create_subscription(OccupancyGrid, '/map', self.on_map, transient)
        self.node.create_timer(.2, self.sample, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.output = Path(output); self.output.mkdir(parents=True, exist_ok=True)
        self.started = time.time(); self.stop_reason = 'shutdown'

    def on_map(self, msg):
        if msg.header.frame_id != 'map':
            self.map_error = 'map_frame_mismatch'; return
        if (msg.info.width <= 0 or msg.info.height <= 0 or
                len(msg.data) != msg.info.width * msg.info.height or
                not math.isfinite(msg.info.resolution) or msg.info.resolution <= 0 or
                any(value < -1 or value > 100 for value in msg.data)):
            self.map_error = 'invalid_occupancy_grid'; return
        origin_q = msg.info.origin.orientation
        q_norm = sum(value*value for value in (origin_q.x, origin_q.y, origin_q.z, origin_q.w))
        if (not all(math.isfinite(value) for value in
                    (msg.info.origin.position.x, msg.info.origin.position.y,
                     origin_q.x, origin_q.y, origin_q.z, origin_q.w)) or
                abs(q_norm - 1.) > 1e-3 or abs(origin_q.x) > 1e-3 or abs(origin_q.y) > 1e-3):
            self.map_error = 'invalid_map_origin'; return
        self.map = dict(width=msg.info.width, height=msg.info.height,
            resolution=msg.info.resolution,
            origin=[msg.info.origin.position.x, msg.info.origin.position.y, yaw_of(origin_q)],
            data=list(msg.data), frame_id=msg.header.frame_id)
        self.map_stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.map_error = None

    def sample(self):
        try:
            transform = self.tf.lookup_transform('map', 'base_link', self.Time())
        except self.TransformException:
            return
        if transform.header.frame_id != 'map' or transform.child_frame_id != 'base_link':
            return
        t = transform.transform.translation; stamp = transform.header.stamp
        timestamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        now_ns = self.node.get_clock().now().nanoseconds
        if now_ns <= 0 or timestamp_ns <= 0:
            return
        clock_reset = False
        if self.last_sim_ns is not None and now_ns < self.last_sim_ns:
            self.epoch += 1; self.segment += 1
            self.path_msg = self.PathMsg(); self.path_msg.header.frame_id = 'map'
            clock_reset = True
        self.last_sim_ns = now_ns
        age_ns = now_ns - timestamp_ns
        if not 0 <= age_ns <= 500_000_000:
            return
        if self.rows and timestamp_ns == self.rows[-1]['timestamp_ns']:
            return
        if self.rows and timestamp_ns < self.rows[-1]['timestamp_ns']:
            if not clock_reset:
                self.epoch += 1; self.segment += 1
            self.path_msg = self.PathMsg(); self.path_msg.header.frame_id = 'map'
        elif self.rows and (timestamp_ns <= self.rows[-1]['timestamp_ns'] or
                            timestamp_ns - self.rows[-1]['timestamp_ns'] > 2_000_000_000):
            self.segment += 1
        orientation = transform.transform.rotation
        orientation_norm = sum(value*value for value in
                               (orientation.x, orientation.y, orientation.z, orientation.w))
        if (not all(math.isfinite(value) for value in
                    (t.x, t.y, t.z, orientation.x, orientation.y, orientation.z, orientation.w)) or
                abs(orientation_norm - 1.) > 1e-3):
            return
        row = dict(timestamp_ns=timestamp_ns, x=float(t.x), y=float(t.y),
                   yaw_rad=yaw_of(orientation), segment=self.segment, epoch=self.epoch,
                   source='public map->base_link TF')
        self.rows.append(row)
        pose = __import__('geometry_msgs.msg', fromlist=['PoseStamped']).PoseStamped()
        pose.header = transform.header; pose.header.frame_id = 'map'
        pose.pose.position = self.Point(x=t.x, y=t.y, z=t.z)
        pose.pose.orientation = orientation
        self.path_msg.header = pose.header; self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)
        self.publish_markers()

    def publish_markers(self):
        message = self.MarkerArray(); segments = []
        for row in self.rows:
            if (not segments or row['timestamp_ns'] <= segments[-1][-1]['timestamp_ns'] or
                    row['timestamp_ns'] - segments[-1][-1]['timestamp_ns'] > 2_000_000_000):
                segments.append([])
            segments[-1].append(row)
        for index, points in enumerate(segments):
            marker = self.Marker(); marker.header.frame_id = 'map'; marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.ns = 'actual_trajectory'; marker.id = index; marker.type = self.Marker.LINE_STRIP
            marker.action = self.Marker.ADD; marker.pose.orientation.w = 1.
            marker.scale.x = .035; marker.color.a = 1.
            marker.color.r = float(index % 3 == 0); marker.color.g = float(index % 3 == 1); marker.color.b = float(index % 3 == 2)
            from geometry_msgs.msg import Point
            marker.points = [self.Point(x=p['x'], y=p['y'], z=.04) for p in points]
            if len(points) == 1:
                marker.type = self.Marker.SPHERE; marker.scale.x = marker.scale.y = marker.scale.z = .08
                marker.pose.position = self.Point(x=points[0]['x'], y=points[0]['y'], z=.04)
            message.markers.append(marker)
        self.markers_pub.publish(message)

    def export(self):
        csv_path = self.output / 'trajectory.csv'
        with csv_path.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=['timestamp_ns', 'x', 'y', 'yaw_rad', 'segment', 'epoch', 'source'])
            writer.writeheader(); writer.writerows(self.rows)
        render_track(self.output / 'trajectory.png', self.map, self.rows)
        result = dict(kind='manual_warehouse_view', outcome='stopped_by_user', success=None,
            stop_reason=self.stop_reason, motion_enabled=False, trajectory_samples=len(self.rows),
            trajectory_source='public map->base_link TF', map_source='/map nav_msgs/msg/OccupancyGrid' if self.map else None,
            map_sha256=(hashlib.sha256(json.dumps(self.map, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
                        if self.map else None),
            map_error=self.map_error, tf_epoch_count=self.epoch + 1,
            map_stamp_ns=self.map_stamp_ns,
            last_tf_age_s=((self.node.get_clock().now().nanoseconds - self.rows[-1]['timestamp_ns']) * 1e-9
                           if self.rows else None),
            final_position=({k: self.rows[-1][k] for k in ('x', 'y', 'yaw_rad')} if self.rows else None),
            artifacts=dict(csv=str(csv_path), image=str(self.output / 'trajectory.png')),
            started_utc=datetime.fromtimestamp(self.started, timezone.utc).isoformat(),
            ended_utc=datetime.now(timezone.utc).isoformat())
        (self.output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')

    def run(self):
        try:
            self.rclpy.spin(self.node)
        except (KeyboardInterrupt, self.rclpy.executors.ExternalShutdownException):
            self.stop_reason = 'interrupted'
        finally:
            self.export()
            self.node.destroy_node()
            if self.rclpy.ok(): self.rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    import rclpy
    rclpy.init()
    ManualLogger(args.output_dir).run()


if __name__ == '__main__':
    main()
