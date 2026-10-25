"""Bounded raw capture contracts; these tests never initialize ROS or DDS."""
import base64
import hashlib
import importlib
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))


def module():
    assert importlib.util.find_spec('dwb_failure_window'), 'bounded DWB recorder is missing'
    return importlib.import_module('dwb_failure_window')


def evaluation(invalid=False):
    return dict(header=dict(stamp=dict(sec=119, nanosec=907500001), frame_id='odom'),
                best_index=0, worst_index=1,
                twists=[dict(traj=dict(velocity=dict(x=.1*i, y=0., theta=.2),
                    poses=[dict(x=1., y=2., theta=.3), dict(x=1.1+i, y=2.2, theta=.4)],
                    time_offsets=[dict(sec=0, nanosec=0), dict(sec=1, nanosec=700000000)]),
                    total=-1. if invalid else float(i),
                    scores=[dict(name='BaseObstacle', raw_score=-1. if invalid else 2., scale=.02)])
                    for i in range(2)])


def put(rec, t, topic='/evaluation', value=None, epoch=0, cdr=b'original-cdr'):
    return rec.ingest(topic, 'dwb_msgs/msg/LocalPlanEvaluation' if topic=='/evaluation' else 'test/Message',
                      evaluation() if value is None else value, cdr,
                      received_ns=int(t*1e9), utc_ns=1791968948734699055,
                      sim_stamp=dict(sec=119, nanosec=907500000), epoch=epoch,
                      publisher_gid='01')


def records(folder):
    return [json.loads(line) for line in (folder/'failure-window.jsonl').read_text().splitlines()]


def test_first_invalid_keeps_complete_rollouts_reasons_and_raw_cdr(tmp_path):
    rec=module().FailureWindow(tmp_path, before_s=2, after_s=1)
    put(rec, 1, '/clock', {'clock': {'sec': 1, 'nanosec': 1}})
    put(rec, 2)
    failed=evaluation(True)
    put(rec, 3, value=failed)
    put(rec, 3.01, '/rosout', dict(name='DWBLocalPlanner', msg='1.00: BaseObstacle/Trajectory Hits Obstacle.',
        stamp=dict(sec=1791968948, nanosec=734699055)))
    put(rec, 3.2, value=evaluation(True))
    rec.advance(4_000_000_001)
    report=rec.close()
    saved=records(tmp_path)
    assert report['trigger']['seq']==3 and report['stop_reason']=='post_window_complete'
    assert sum(r['derived'].get('all_invalid', False) for r in saved)==2
    source=next(r for r in saved if r['seq']==3)
    assert source['source_message']==failed
    assert source['source_message']['worst_index']==1
    assert len(source['source_message']['twists'][1]['traj']['poses'])==2
    assert base64.b64decode(source['cdr_base64'])==b'original-cdr'
    assert source['cdr_sha256']==hashlib.sha256(b'original-cdr').hexdigest()
    assert source['derived']['rejecting_critics']=={'BaseObstacle':2}
    assert source['derived']['rejection_reason'] is None
    reason=next(r for r in saved if r['topic']=='/rosout')
    assert reason['derived']['critic_reason']==dict(critic='BaseObstacle', reason='Trajectory Hits Obstacle.', reported_fraction=1.)
    assert report['controller_private_inputs_captured'] is False
    assert report['transport_completeness']=='unknown'


def test_wall_window_preserves_integer_stamps_and_clock_epochs(tmp_path):
    rec=module().FailureWindow(tmp_path, before_s=1, after_s=1)
    put(rec, 1, '/tf', {'transforms': []}, epoch=0)
    put(rec, 1.5, '/clock', {'clock': {'sec': 0, 'nanosec': 0}}, epoch=1)
    put(rec, 2, value=evaluation(True), epoch=1)
    rec.advance(3_000_000_001)
    rec.close()
    saved=records(tmp_path)
    assert [r['receipt']['epoch'] for r in saved]==[0,1,1]
    assert saved[-1]['source_message']['header']['stamp']['nanosec']==907500001
    assert saved[-1]['receipt']['utc_ns']==1791968948734699055
    assert saved[-1]['receipt']['sim_stamp']==dict(sec=119, nanosec=907500000)


def test_empty_or_partly_valid_evaluation_does_not_trigger(tmp_path):
    rec=module().FailureWindow(tmp_path)
    e=evaluation(True);e['twists'][0]['total']=0
    put(rec, 0, value=e)
    e['twists']=[];put(rec, 1, value=e)
    report=rec.close('duration')
    assert report['trigger'] is None and report['stop_reason']=='duration'
    assert not report['retention_complete']


def test_pre_window_eviction_and_old_context_are_explicit(tmp_path):
    rec=module().FailureWindow(tmp_path, before_s=1, after_s=.5)
    put(rec, 0, '/tf_static', {'transforms': [{'header': {'stamp': {'sec': 0, 'nanosec': 0}}}]})
    put(rec, .1, '/plan', {'poses': [1,2,3]})
    put(rec, .2, '/course_navigation/cmd_vel_raw', {'linear': {'x': .2}})
    put(rec, 2)
    put(rec, 3, value=evaluation(True))
    rec.advance(3_500_000_001);report=rec.close()
    saved=records(tmp_path)
    assert [r['seq'] for r in saved]==[1,2,4,5]
    assert report['context_before_window_seqs']==[1,2]
    assert report['retention_complete']


def test_memory_count_and_disk_caps_do_not_overwrite_existing_evidence(tmp_path):
    rec=module().FailureWindow(tmp_path, memory_bytes=10000, max_records=3,
        max_record_bytes=4000, max_raw_bytes=1000, disk_bytes=12000, reserve_bytes=4096)
    for i in range(50):
        put(rec, i/100, '/slam/odom', {'pose': 'x'*100})
        assert rec.buffer_bytes<=10000 and rec.buffer_count<=3
    put(rec, .6, value=evaluation(True))
    for i in range(20): put(rec, .7+i/100, '/tf', {'transforms': 'y'*500})
    report=rec.close()
    assert report['drops'] and not report['retention_complete']
    assert sum(p.stat().st_size for p in tmp_path.iterdir())<=12000
    before=(tmp_path/'failure-window.jsonl').read_bytes()
    with pytest.raises(FileExistsError): module().FailureWindow(tmp_path)
    assert (tmp_path/'failure-window.jsonl').read_bytes()==before


def test_oversized_records_are_flagged_without_unbounded_retention(tmp_path):
    rec=module().FailureWindow(tmp_path, max_raw_bytes=100, max_record_bytes=2000)
    put(rec, 0, '/tf', {}, cdr=b'x'*101)
    put(rec, .1, '/plan', {'poses': 'x'*3000})
    report=rec.close()
    assert report['drops']['raw_message_limit']==1
    assert report['drops']['record_limit']==1
    assert records(tmp_path)==[]


def test_post_window_does_not_slide_on_repeated_failures(tmp_path):
    rec=module().FailureWindow(tmp_path, before_s=1, after_s=1)
    put(rec, 0, value=evaluation(True))
    put(rec, .9, value=evaluation(True))
    put(rec, 1.1, value=evaluation(True))
    report=rec.close()
    assert report['trigger']['received_ns']==0
    assert len(records(tmp_path))==2
    assert report['stop_reason']=='post_window_complete'


def test_corrupt_numbers_remain_recoverable_from_raw_cdr(tmp_path):
    rec=module().FailureWindow(tmp_path)
    e=evaluation();e['twists'][0]['total']=float('nan')
    put(rec, 0, value=e, cdr=b'cdr-with-original-nan')
    rec.close()
    r=records(tmp_path)[0]
    assert r['source_message']['twists'][0]['total']=={'nonfinite_float':'nan'}
    assert r['derived']['all_invalid'] is False
    assert base64.b64decode(r['cdr_base64'])==b'cdr-with-original-nan'


def test_real_ros_message_cdr_roundtrip_without_ros_init(tmp_path):
    from dwb_msgs.msg import LocalPlanEvaluation, TrajectoryScore, CriticScore
    from geometry_msgs.msg import Pose2D
    from rclpy.serialization import serialize_message, deserialize_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    msg=LocalPlanEvaluation();msg.header.stamp.nanosec=907500001
    score=TrajectoryScore();score.total=-1.;score.traj.poses=[Pose2D(x=1.,y=2.,theta=.3)]
    score.scores=[CriticScore(name='Oscillation',raw_score=-1.,scale=0.)];msg.twists=[score]
    cdr=serialize_message(msg)
    rec=module().FailureWindow(tmp_path)
    put(rec, 0, value=message_to_ordereddict(msg), cdr=cdr);rec.close()
    saved=records(tmp_path)[0]
    restored=deserialize_message(base64.b64decode(saved['cdr_base64']),LocalPlanEvaluation)
    assert restored==msg


def test_cli_opt_in_is_separate_and_limits_are_validated_without_ros():
    import observe_warehouse_narrow_failure as observer
    args=observer.build_parser().parse_args(['--dwb-failure-window'])
    assert args.dwb_failure_window and not args.dwb_debug
    with pytest.raises(SystemExit): observer.build_parser().parse_args(['--dwb-debug','--dwb-failure-window'])
    for values in [dict(before_s=-1),dict(after_s=float('nan')),dict(max_records=0),
                   dict(disk_bytes=1024),dict(memory_bytes=0)]:
        with pytest.raises(ValueError): module().FailureWindow.validate_limits(**values)


def test_metadata_limit_keeps_last_accepted_snapshot(tmp_path):
    rec=module().FailureWindow(tmp_path,reserve_bytes=4096)
    metadata={'runtime':{'planner':'Smac2D'}}
    rec.set_metadata(metadata)
    metadata['runtime']['unbounded']='x'*10000
    rec.set_metadata(metadata)
    report=rec.close()
    assert report['metadata']=={'runtime':{'planner':'Smac2D'}}
    assert report['drops']=={'metadata_limit':1}


def test_existing_manifest_is_never_overwritten(tmp_path):
    path=tmp_path/'failure-window-manifest.json';path.write_text('original evidence')
    with pytest.raises(FileExistsError):module().FailureWindow(tmp_path)
    assert path.read_text()=='original evidence'
    assert not (tmp_path/'failure-window.jsonl').exists()


def test_full_419_candidate_cycle_and_public_context_survive(tmp_path):
    from copy import deepcopy
    rec=module().FailureWindow(tmp_path,before_s=1,after_s=1)
    context={
        '/local_costmap/costmap_raw':dict(header=dict(stamp=dict(sec=119,nanosec=817500000),frame_id='odom'),data=[0,253,254,255]),
        '/tf':dict(transforms=[dict(header=dict(stamp=dict(sec=119,nanosec=500000000),frame_id='odom'),child_frame_id='base_link')]),
        '/plan':dict(header=dict(frame_id='map'),poses=[dict(pose=dict(position=dict(x=-4.47874116897583,y=6.116940498352051)))]),
        '/course_navigation/cmd_vel_raw':dict(linear=dict(x=.3789473684,y=0.),angular=dict(z=0.)),
        '/course_navigation/cmd_vel':dict(linear=dict(x=.38,y=0.),angular=dict(z=0.)),
        '/path_manager/path_manager_ros/nav_vel':dict(linear=dict(x=.38,y=0.),angular=dict(z=0.)),
        '/slam/odom':dict(header=dict(frame_id='odom'),child_frame_id='base_link',twist=dict(twist=dict(linear=dict(x=.31,y=.01)))),
        '/sim/ground_truth/odom':dict(header=dict(frame_id='sim_world'),child_frame_id='base_link',pose=dict(pose=dict(position=dict(x=1.,y=2.))))}
    for i,(topic,msg) in enumerate(context.items()):put(rec,1+i*.01,topic,msg)
    failed=evaluation(True);failed['twists']=[deepcopy(failed['twists'][0]) for _ in range(419)]
    for i,candidate in enumerate(failed['twists']):
        candidate['traj']['velocity']['theta']=i/1000
        candidate['scores'][0]['name']='Oscillation' if i%2 else 'BaseObstacle'
    put(rec,2,value=failed)
    rec.advance(3_000_000_001);rec.close()
    saved=records(tmp_path)
    assert saved[-1]['source_message']==failed
    assert saved[-1]['derived']['rejecting_critics']=={'BaseObstacle':210,'Oscillation':209}
    for record in saved[:-1]:assert record['source_message']==context[record['topic']]


def test_callback_matches_installed_humble_executor_without_ros_init(tmp_path):
    import asyncio
    from types import SimpleNamespace
    from dwb_msgs.msg import LocalPlanEvaluation
    from rclpy.executors import Executor
    from rclpy.serialization import serialize_message,deserialize_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    rec=module().FailureWindow(tmp_path)
    msg=LocalPlanEvaluation();msg.header.stamp.nanosec=123456789
    def receive(topic,kind,raw):
        decoded=message_to_ordereddict(deserialize_message(raw,LocalPlanEvaluation))
        rec.ingest(topic,kind,decoded,raw,received_ns=1,utc_ns=2,sim_stamp=None,epoch=0)
    callback=module().make_subscription_callback('/evaluation','dwb_msgs/msg/LocalPlanEvaluation',receive)
    # Invoke the exact installed executor dispatch method; no Executor or Node is constructed.
    asyncio.run(Executor._execute_subscription(None,SimpleNamespace(callback=callback),serialize_message(msg)))
    rec.close()
    saved=records(tmp_path)[0]
    assert saved['source_message']['header']['stamp']['nanosec']==123456789
    assert saved['receipt']['publisher_gid'] is None


def test_nested_metadata_fits_final_manifest_reserve(tmp_path):
    rec=module().FailureWindow(tmp_path,reserve_bytes=4096,disk_bytes=12000)
    metadata={'values':[0]*600}
    rec.set_metadata(metadata)
    report=rec.close()
    assert report['metadata']==metadata
    assert (tmp_path/'failure-window-manifest.json').stat().st_size<=4096


def test_static_tf_sets_remain_separate_without_publisher_gid(tmp_path):
    rec=module().FailureWindow(tmp_path,before_s=1)
    for i,child in enumerate(['lidar_link','imu_link']):
        rec.ingest('/tf_static','tf2_msgs/msg/TFMessage',
            {'transforms':[{'header':{'frame_id':'base_link'},'child_frame_id':child}]},b'raw',
            received_ns=i*100000000,utc_ns=2,sim_stamp=None,epoch=0)
    put(rec,3,value=evaluation(True))
    rec.close()
    assert [r['source_message']['transforms'][0]['child_frame_id'] for r in records(tmp_path)[:2]]==['lidar_link','imu_link']


def test_manifest_pressure_keeps_raw_records_and_marks_omission(tmp_path):
    rec=module().FailureWindow(tmp_path,reserve_bytes=4096,disk_bytes=256*1024)
    for i in range(80):
        put(rec,i*.001,'/'+'topic'*32+str(i),{'value':i})
    report=rec.close()
    assert len(records(tmp_path))==80
    assert report['manifest_detail_omitted']
    assert report['received_total']==80
    assert report['drops']['manifest_detail_limit']==1
    assert sum(p.stat().st_size for p in tmp_path.iterdir())<=256*1024
