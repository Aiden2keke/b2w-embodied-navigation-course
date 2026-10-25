"""Exercise real ROS message callbacks without rclpy.init, DDS or motion."""
from types import SimpleNamespace as NS
import json

import numpy as np
import pytest
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from nav2_msgs.msg import Costmap
from rclpy.time import Time

from course_navigation.adaptive_speed import Config, Governor
from course_navigation.speed_governor import SpeedGovernor


class Publisher:
    def __init__(self): self.messages = []
    def publish(self, msg): self.messages.append(msg)


def node(monkeypatch):
    import course_navigation.speed_governor as module
    monkeypatch.setattr(module.time, 'monotonic', lambda: 20.)
    n = SpeedGovernor.__new__(SpeedGovernor)
    n.governor = Governor(Config(w_ref=1.5, release_hold_s=.1))
    n.footprint = [[.75, .43], [.75, -.43], [-.75, -.43], [-.75, .43]]
    n.map_timeout = n.odom_timeout = n.path_timeout = n.tf_timeout = n.cloud_timeout = .6
    n.grid = n.path = n.speed = n.map_frame = n.last_ros = None
    n.times = {}
    n.last_tick_wall = None
    n.get_clock = lambda: NS(now=lambda: Time(seconds=10.))
    n.limit_pub, n.status_pub = Publisher(), Publisher()
    tr = TransformStamped()
    tr.header.stamp.sec = 10
    tr.transform.rotation.w = 1.
    n.tf = NS(lookup_transform=lambda *args: tr)
    return n


def feed(n):
    m = Costmap()
    m.header.frame_id = 'odom'
    m.header.stamp.sec = m.metadata.update_time.sec = 10
    m.metadata.resolution = .05
    m.metadata.size_x = m.metadata.size_y = 120
    m.metadata.origin.position.x = m.metadata.origin.position.y = -3.
    m.metadata.origin.orientation.w = 1.
    m.data = [0]*14400
    n.map_received(m)
    p = Path()
    p.header.frame_id, p.header.stamp.sec = 'base_link', 10
    for x in (0., 1., 2.):
        point = PoseStamped()
        point.pose.position.x = x
        p.poses.append(point)
    n.path_received(p)
    o = Odometry()
    o.header.stamp.sec, o.child_frame_id = 10, 'base_link'
    o.pose.pose.orientation.w = 1.
    o.twist.twist.linear.x = .18
    n.odom_received(o)
    n.times['cloud'] = (10., 20.)
    return m, p, o


def test_callbacks_transform_path_and_publish_positive_narrow_then_open(monkeypatch):
    n = node(monkeypatch)
    feed(n)
    assert np.allclose(n.path, [(0., 0.), (1., 0.), (2., 0.)])
    n.tick()
    assert n.limit_pub.messages[-1].speed_limit == .11
    n.get_clock = lambda: NS(now=lambda: Time(seconds=10.2))
    n.tick()
    assert n.limit_pub.messages[-1].speed_limit == .43
    assert json.loads(n.status_pub.messages[-1].data)['mode'] == 'OPEN'


@pytest.mark.parametrize('source', ['map', 'odom', 'path', 'localization', 'cloud'])
def test_stale_source_never_opens_even_with_fresh_other_inputs(monkeypatch, source):
    n = node(monkeypatch)
    m, p, o = feed(n)
    if source == 'localization':
        tr = n.tf.lookup_transform()
        tr.header.stamp.sec = 8
    else:
        n.times[source] = (8., 20.)
    n.governor.mode = 'OPEN'
    n.tick()
    assert 0 < n.limit_pub.messages[-1].speed_limit <= .11
    assert json.loads(n.status_pub.messages[-1].data)['mode'] == 'NARROW'


def test_fresh_header_cannot_hide_stale_costmap_update(monkeypatch):
    n = node(monkeypatch)
    m, _, _ = feed(n)
    m.metadata.update_time.sec = 8
    n.map_received(m)
    n.tick()
    assert not json.loads(n.status_pub.messages[-1].data)['healthy']


def test_humble_topic_unset_update_time_uses_fresh_header_and_receipt(monkeypatch):
    n = node(monkeypatch)
    m, _, _ = feed(n)
    m.metadata.update_time.sec = 0  # Humble prepareCostmap() leaves this field unset.
    n.map_received(m)
    n.tick()
    assert json.loads(n.status_pub.messages[-1].data)['healthy']
    n.get_clock = lambda: NS(now=lambda: Time(seconds=10.2))
    n.tick()
    assert n.limit_pub.messages[-1].speed_limit == .43


def test_invalid_odom_and_clock_rewind_reset_all_inputs(monkeypatch):
    n = node(monkeypatch)
    _, _, o = feed(n)
    o.twist.twist.linear.x = float('nan')
    n.odom_received(o)
    n.tick()
    assert n.speed is None and n.limit_pub.messages[-1].speed_limit == .11
    n.get_clock = lambda: NS(now=lambda: Time(seconds=9.))
    n.tick()
    assert not n.times


def test_path_transform_uses_message_timestamp_and_translation(monkeypatch):
    n = node(monkeypatch)
    feed(n)
    tr = TransformStamped()
    tr.transform.rotation.w = 1.
    tr.transform.translation.x = 1.
    def lookup(target, source, stamp):
        assert target == 'odom' and source == 'base_link' and stamp.nanoseconds == 10_000_000_000
        return tr
    n.tf.lookup_transform = lookup
    p = Path()
    p.header.frame_id, p.header.stamp.sec = 'base_link', 10
    p.poses = [PoseStamped(), PoseStamped()]
    p.poses[-1].pose.position.x = 1.
    n.path_received(p)
    assert np.allclose(n.path, [(1., 0.), (2., 0.)])


def test_governor_execution_gap_requires_new_clearance_hold(monkeypatch):
    import course_navigation.speed_governor as module
    n = node(monkeypatch)
    feed(n)
    n.tick()
    n.governor.mode = 'OPEN'
    monkeypatch.setattr(module.time, 'monotonic', lambda: 21.)
    # All sources have recovered, but the governor heartbeat had a one-second gap.
    n.times = {name: (10., 21.) for name in n.times}
    n.tick()
    assert json.loads(n.status_pub.messages[-1].data)['mode'] == 'NARROW'
