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
    # Callback/action tests isolate recording; exact CDR is tested below with
    # the real worker and SQLite replay.
    t.journal=NS(append=lambda *args:None,failure=None,
                 close=lambda **kw:dict(complete=True,failure=None,accepted=0,written=0,bytes=0))
    t.localization_tf={}
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
        return dict(sim=sim,wall=sim,x=0.,y=0.,tf_stamp=sim,odom_stamp=sim,map_odom_stamp=sim,pose_age=0.,clock_age=0.,speed=0.,wz=0.,command_speed=0.,command_wz=0.)
    t.sample=sample
    assert t.close() and t.audit['cleanup']['stationary_confirmed']
    assert t.audit['cleanup']['stop_source_span_s']>=5.
    assert t.audit['cleanup']['stop_wall_span_s']>=3.


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


def test_transport_retains_exact_full_cloud_and_map_payloads(tmp_path):
    from stage4_route.journal import BinaryJournal,replay
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import OccupancyGrid
    t=transport();t.raw_path=tmp_path/'raw.sqlite3';t.journal=BinaryJournal(t.raw_path,reserve_bytes=0)
    cloud=PointCloud2();cloud.header.frame_id='lidar_link';cloud.data=list(range(256))*8192
    m=OccupancyGrid();m.data=[-1,0,100]
    expected=[]
    def observed_serialize(message):
        raw=serialize_message(message);expected.append(raw);return raw
    with patch('rclpy.serialization.serialize_message',side_effect=observed_serialize):
        t.receive('/pointcloud',cloud);t.receive('/map',m)
    assert t.journal.close()['complete']
    records=list(replay(t.raw_path))
    # DDS serialization padding need not be canonical across separate calls;
    # preserve exactly the bytes supplied by each actual callback.
    assert len(records)==len(expected)==2
    assert all(r['payload']==raw for r,raw in zip(records,expected))


def test_async_recording_fault_is_seen_before_another_goal_or_tick():
    t=transport();t.journal.failure='evidence_disk_limit'
    t.preflight=lambda goals:pytest.fail('must not plan after writer fault')
    with pytest.raises(ValueError,match='evidence_disk_limit'):t.send([], 'unused')


def test_duplicate_or_unrelated_tf_cannot_renew_localization_receipt():
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    t=transport();tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom';tf.header.stamp.sec=1
    with patch('stage4_route.ros_transport.time.monotonic',return_value=10.):t.receive('/tf',TFMessage(transforms=[tf]))
    with patch('stage4_route.ros_transport.time.monotonic',return_value=10.4):t.receive('/tf',TFMessage(transforms=[tf]))
    assert t.localization_tf[('map','odom')]['wall']==10.
    tf.header.frame_id='base_link';tf.child_frame_id='FR_foot';tf.header.stamp.sec=2
    t.receive('/tf',TFMessage(transforms=[tf]))
    assert ('odom','base_link') not in t.localization_tf


def test_recording_capacity_covers_measured_peak_trial_and_cleanup(tmp_path):
    t=transport();t.raw_path=tmp_path/'raw';t.raw_path.write_bytes(b'')
    t.recording_first_wall=10.;t.recorded_input_bytes=10000000;t.recording_peak_rate=2000000.
    t.recording_window_wall=19.9;t.recording_window_bytes=0
    t.journal.max_bytes=400000000;t.journal.reserve=100000000
    with patch('stage4_route.ros_transport.time.monotonic',return_value=20.),patch('shutil.disk_usage',return_value=NS(free=800000000)):
        with pytest.raises(ValueError,match='capacity'):t.verify_recording_capacity(60.)
        t.journal.max_bytes=600000000
        result=t.verify_recording_capacity(60.)
        assert result['required_remaining_bytes']==480000000
        assert result['headroom_factor']==2.
        t.recording_first_wall=19.
        with pytest.raises(ValueError,match='observation'):t.verify_recording_capacity(60.)


def test_recording_close_failure_invalidates_nested_cleanup_audit(tmp_path):
    t=transport();t.raw_path=tmp_path/'raw';t.raw_path.write_bytes(b'')
    t.send_future=None;t.handle=None;t.result_future=None;t.nav=NS(destroy=lambda:None)
    t.node=NS(destroy_node=lambda:None);t.rclpy=NS(shutdown=lambda:None)
    ticks=[0]
    def spin():ticks[0]+=1
    t.spin=spin
    def sample():
        sim=1.+ticks[0]*.1
        return dict(sim=sim,wall=sim,x=0.,y=0.,tf_stamp=sim,odom_stamp=sim,map_odom_stamp=sim,pose_age=0.,clock_age=0.,speed=0.,wz=0.,command_speed=0.,command_wz=0.)
    t.sample=sample
    t.journal.close=lambda **kw:dict(complete=False,failure='evidence_flush_timeout',bytes=0)
    assert not t.close()
    assert not t.audit['cleanup']['passed']
    assert not t.audit['cleanup']['stationary_confirmed']


@pytest.mark.parametrize('first_spin_advance',[.1,6.])
def test_late_owned_acceptance_is_still_cancelled_and_never_passes_deadline(tmp_path,first_spin_advance):
    t=transport();t.raw_path=tmp_path/'raw';t.raw_path.write_bytes(b'')
    t.owned_id='1'*32;t.handle=None;t.result_future=None
    t.guard_config=dict(session_id='cleanup-test',inhibit_path=str(tmp_path/'inhibit.json'))
    t.nav=NS(destroy=lambda:None);t.node=NS(destroy_node=lambda:None);t.rclpy=NS(shutdown=lambda:None)
    now=[0.];done=[False];cancels=[]
    result=NS(done=lambda:done[0])
    def cancel():
        cancels.append(t.owned_id);done[0]=True
        return NS(done=lambda:True,result=lambda:NS(goals_canceling=[NS(goal_id=NS(uuid=list(bytes.fromhex(t.owned_id))))]))
    handle=NS(accepted=True,get_result_async=lambda:result,cancel_goal_async=cancel)
    t.send_future=NS(done=lambda:now[0]>=6.,result=lambda:handle)
    def spin():now[0]+=first_spin_advance if now[0]==0. else .1
    t.spin=spin
    t.sim=lambda:now[0]
    t.sample=lambda:dict(sim=now[0],wall=now[0],x=0.,y=0.,tf_stamp=now[0]-.4,odom_stamp=now[0]-.4,map_odom_stamp=now[0]-.4,pose_age=0.,clock_age=0.,speed=0.,wz=0.,command_speed=0.,command_wz=0.)
    with patch('stage4_route.ros_transport.time.monotonic',side_effect=lambda:now[0]):
        assert not t.close()
    assert cancels==['1'*32]
    assert t.audit['cleanup']['owned_action_terminal']
    assert t.failure=='owned_action_terminal_timeout'
    assert now[0]>=11.,'Stop hold must follow owned action termination'
    assert t.audit['cleanup']['stop_source_start_s']>=t.audit['cleanup']['terminal_watermark_sim']
