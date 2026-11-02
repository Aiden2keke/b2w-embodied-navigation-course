"""Exercise the installed transport callbacks/cleanup without a ROS context or node."""
import json
from io import BytesIO
from pathlib import Path
from types import SimpleNamespace as NS
import sys
from unittest.mock import patch
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from stage4_route.ros_transport import RosTransport


def transport():
    t=RosTransport.__new__(RosTransport)
    t.messages={};t.receipts={};t.statuses={};t.failure=None;t.raw=BytesIO();t.bytes=t.seq=0
    t.owned_id=None;t.preflight_id=None;t.child_ids={};t.audit={'action_outcomes':[]};t.sim=lambda:1.
    return t


def status(ids):
    from action_msgs.msg import GoalStatusArray,GoalStatus
    msg=GoalStatusArray()
    for id in ids:
        s=GoalStatus();s.status=2;s.goal_info.goal_id.uuid=list(bytes.fromhex(id));msg.status_list.append(s)
    return msg


def test_owned_preflight_vs_foreign_and_behavior_callbacks():
    t=transport();owned='1'*32;foreign='2'*32;t.preflight_id=owned
    t.receive('/compute_path_through_poses/_action/status',status([owned]));assert t.failure is None
    t.receive('/compute_path_through_poses/_action/status',status([foreign]));assert t.failure=='foreign_preflight_action'
    for name in ('spin','backup','drive_on_heading','assisted_teleop','wait','navigate_to_pose','follow_waypoints'):
        t=transport();t.owned_id=owned;t.receive('/'+name+'/_action/status',status([foreign]));assert t.failure


def test_child_replacement_is_rejected():
    t=transport();t.owned_id='1'*32
    t.receive('/follow_path/_action/status',status(['1'*32]));assert not t.failure
    t.receive('/follow_path/_action/status',status(['3'*32]));assert t.failure=='foreign_follow_path'


def test_goal_callback_fails_even_without_motion():
    from geometry_msgs.msg import PoseStamped
    t=transport();t.receive('/goal_pose',PoseStamped());assert t.failure=='competing_goal'


def test_command_publisher_exclusivity():
    t=transport()
    expected={'/clock':['clock'],'/slam/odom':['pose_adapter'],'/slam/map_points':['map'],
              '/course_navigation/cmd_vel_raw':['controller_server'],
              '/course_navigation/cmd_vel':['behavior_server']*5+['velocity_smoother'],
              '/path_manager/path_manager_ros/nav_vel':['velocity_adapter','foreign']}
    t.node=NS(get_publishers_info_by_topic=lambda topic:[NS(node_name=n,endpoint_gid=[1]*24) for n in expected[topic]])
    with pytest.raises(ValueError,match='command publisher'):t.graph()


def test_duplicate_tf_does_not_reset_cleanup_stop(tmp_path):
    t=transport();t.directory=tmp_path;t.raw_path=tmp_path/'raw.jsonl';t.raw_path.write_bytes(b'');t.raw=t.raw_path.open('ab')
    t.send_future=None;t.handle=None;t.result_future=None;t.nav=NS(destroy=lambda:None)
    t.node=NS(destroy_node=lambda:None);t.rclpy=NS(shutdown=lambda:None)
    ticks=[0]
    def spin():ticks[0]+=1
    t.spin=spin
    def sample():
        sim=1.+(ticks[0]//2)*.1
        return dict(sim=sim,tf_stamp=sim,odom_stamp=sim,map_odom_stamp=sim,pose_age=0.,clock_age=0.,speed=0.,wz=0.,command_speed=0.,command_wz=0.)
    t.sample=sample
    assert t.close() and t.audit['cleanup']['stationary_confirmed']


def test_send_does_not_preflight_or_send_after_fault():
    t=transport();t.failure='competing_goal';t.preflight=lambda goals:pytest.fail('must not plan after fault')
    with pytest.raises(ValueError,match='competing_goal'):t.send([], 'unused')


def test_normal_behavior_publisher_multiplicity_matches_installed_contract():
    from rclpy.action import get_action_client_names_and_types_by_node
    t=transport()
    expected={'/clock':['clock'],'/slam/odom':['pose_adapter'],'/slam/map_points':['map'],
              '/course_navigation/cmd_vel_raw':['controller_server'],
              '/course_navigation/cmd_vel':['behavior_server']*5+['velocity_smoother'],
              '/path_manager/path_manager_ros/nav_vel':['velocity_adapter']}
    t.node=NS(get_publishers_info_by_topic=lambda topic:[NS(node_name=n,endpoint_gid=[i+1]*24) for i,n in enumerate(expected[topic])],
              get_node_names=lambda:[],count_publishers=lambda t:0,get_node_names_and_namespaces=lambda:[],get_name=lambda:'stage4_test')
    graph=t.graph();assert len(graph['/course_navigation/cmd_vel'])==6


def test_late_owned_planner_status_is_accepted():
    t=transport();t.preflight_id='1'*32;t.owned_id='2'*32
    t.receive('/compute_path_through_poses/_action/status',status(['1'*32]));assert t.failure is None
    t.receive('/compute_path_through_poses/_action/status',status(['3'*32]));assert t.failure=='foreign_preflight_action'
