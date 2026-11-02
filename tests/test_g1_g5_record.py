import sys,json,math
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import g1_g5_media as media
import run_g1_g5_record as batch

def test_profile_b_only_halves_forward_speed_and_preserves_collision_contract():
    import yaml
    root=Path(__file__).resolve().parents[1]
    base=yaml.safe_load((root/'course_navigation/config/nav2_smac2d_diag.yaml').read_text())
    slow_path=root/'course_navigation/config/nav2_smac2d_dwb_slow_diag.yaml'
    assert slow_path.is_file()
    slow=yaml.safe_load(slow_path.read_text())
    base['controller_server']['ros__parameters']['FollowPath']['max_vel_x']=.2
    assert slow==base

def test_startup_camera_config_preserves_saved_inputs_and_only_replaces_pose(tmp_path):
    views=Path(__file__).resolve().parents[1]/'artifacts/video/manual_views'
    original=(views/'gazebo_gui.config').read_bytes();pose=(views/'gazebo_camera_pose_final.txt').read_bytes()
    dest=tmp_path/'startup.config';result=batch.startup_camera_config(views,dest)
    assert result['position']==pytest.approx([-1.7076944113,5.4659056664,8.4832897186])
    assert result['rpy']==pytest.approx([0,1.130683299,-1.113886579],abs=1e-6)
    changed=dest.read_text();assert changed.replace(' '.join(format(v,'.17g') for v in result['position']+result['rpy']),'-6 0 6 0 0.5 0')==original.decode()
    assert (views/'gazebo_gui.config').read_bytes()==original and (views/'gazebo_camera_pose_final.txt').read_bytes()==pose
    with pytest.raises(FileExistsError):batch.startup_camera_config(views,dest)

def test_screenshot_content_rejects_black_frames(tmp_path,monkeypatch):
    import types
    p=tmp_path/'capture.png';p.write_bytes(b'fixture')
    monkeypatch.setattr(batch.subprocess,'run',lambda *a,**kw:types.SimpleNamespace(stdout=bytes(96*64*3)))
    with pytest.raises(RuntimeError,match='black, empty or undecodable'):batch.screenshot_content('ffmpeg',p)

def test_optional_camera_feedback_unavailable_does_not_fail_setup(monkeypatch):
    import types,os
    class Listener:
        returncode=None
        def __init__(self,*args,**kwargs):
            self.r,self.w=os.pipe();self.stdout=os.fdopen(self.r,'rb',buffering=0)
        def poll(self):return self.returncode
        def terminate(self):self.returncode=-15;os.close(self.w)
        def wait(self,timeout):return self.returncode
    monkeypatch.setattr(batch.subprocess,'Popen',Listener)
    monkeypatch.setattr(batch.select,'select',lambda *args:([],[],[]))
    result=batch.optional_camera_feedback({},types.SimpleNamespace(poll=lambda:None),batch.time.monotonic()+2)
    assert result['available'] is False


def test_camera_evaluation_transform_does_not_assume_map_equals_world():
    points=[[0,0,0],[2,5,0],[-4,7,0],[6,2,0]]
    assert media.world_points(points,[3,-2,math.pi/2])[1]==pytest.approx([-2,0,0])


def test_retry_only_explicit_ordinary_navigation_failure_with_safe_rest_cleanup():
    r=dict(success=False,route=dict(stop_reason='staging_failed',segments=[dict(nav2_result=dict(status=6,end_reason='nav2_failed'))]),normal_rest=dict(measured_ground_truth_rest=True,measured_lio_rest=True,six_axis_final_zero=True,start_sim=20,end_sim=25,wall_s=5),normal_cleanup=dict(passed=True,remaining={}),watchdog=dict(reason='normal_shutdown',emergency=False),localization_cleanup=dict(passed=True))
    assert batch.classify(r)['retry']
    for field in ['contact_failure','truth_first_fault','readiness_first_fault','error']:
        bad={**r,field:'fault'};assert not batch.classify(bad)['retry']
    assert not batch.classify({**r,'watchdog':dict(emergency=True)})['retry']
    assert not batch.classify({**r,'normal_cleanup':dict(passed=False,remaining={'123':1})})['retry']
    assert not batch.classify({})['retry']


def test_bounded_loop_stops_on_first_success_and_repeated_cause():
    seen=[]
    def attempt(i):seen.append(i);return dict(success=i==2,retry=True,reason='nav2_abort' if i==1 else 'success')
    assert batch.run_batch(attempt,12,7200)['success'] and seen==[1,2]
    seen.clear()
    def same(i):seen.append(i);return dict(success=False,retry=True,reason='same_abort')
    r=batch.run_batch(same,12,7200);assert seen==[1,2] and r['stop_reason']=='repeated_failure'
    assert batch.run_batch(lambda i:dict(success=False,retry=False,reason='safety'),12,7200)['attempts']==1


def test_no_review_never_launches_simulator(tmp_path):
    r=batch.main(['--run','--output-root',str(tmp_path)])
    assert r==2
    assert not list(tmp_path.rglob('results_report.json'))
    assert not list(tmp_path.rglob('success_*.mp4'))


def test_limits_are_not_expandable():
    for args in [['--max-attempts','13'],['--max-total-minutes','121'],['--max-attempts','0']]:
        with pytest.raises(SystemExit):batch.parser().parse_args(args)


def test_five_published_goals_are_not_success():
    assert not batch.verify_route(dict(route=dict(success=True,sent_poses=[{}]*5)),{})['passed']


def test_encoding_uses_h264_and_common_time_interval(tmp_path):
    commands=media.export_commands('/ffmpeg',tmp_path,dict(radar=10.,warehouse=10.2),10.2,20.)
    assert len(commands)==3
    assert all('libx264' in c and '25' in c and '-n' in c for c in commands)
    assert 'hstack=inputs=2' in ' '.join(commands[-1])


def test_gui_is_client_only_and_server_pin_is_not_weakened():
    from warehouse_ab_trial import gui_client_command
    cmd=gui_client_command()
    assert cmd[:3]==['ign','gazebo','-g'] and '-r' not in cmd and '-s' not in cmd


def test_camera_correspondence_pairs_historical_truth_to_fresh_tf_stamp():
    from g1_g5_presentation import pair_correspondence
    from types import SimpleNamespace as N
    q=N(x=0.,y=0.,z=0.,w=1.)
    tf=N(header=N(stamp=N(sec=10,nanosec=0)),transform=N(translation=N(x=.3,y=.1,z=.6),rotation=q))
    def truth(stamp,x):return N(header=N(stamp=N(sec=stamp,nanosec=0)),pose=N(pose=N(position=N(x=x,y=2.,z=.6),orientation=q)))
    sample=pair_correspondence(tf,[(20.,truth(10,1.)),(20.1,truth(11,9.))],10.4,20.2)
    assert sample['world_from_map_se2']==pytest.approx([.7,1.9,0])
    assert sample['truth_stamp']==10
    assert pair_correspondence(tf,[(20.1,truth(11,9.))],10.4,20.2) is None
    assert pair_correspondence(tf,[(19.,truth(10,1.))],10.4,20.2) is None


def test_restore_manual_camera_keeps_exact_request_and_files(tmp_path,monkeypatch):
    import types,os
    views=tmp_path/'views';views.mkdir()
    session=tmp_path/'session';session.mkdir();(session/'warehouse-gui.log').write_text('Camera pose topic advertised on [/gui/camera/pose]')
    pose='position { x: -1.7 y: 5.4 z: 8.4 }\norientation { x: 0 y: 0 z: 0 w: 1 }\n'
    (views/'gazebo_camera_pose_final.txt').write_text(pose)
    (views/'gazebo_gui.config').write_bytes(b'<plugin><camera_pose>old default</camera_pose></plugin>')
    before={p.name:p.read_bytes() for p in views.iterdir()};calls=[]
    def run(argv,**kw):
        calls.append(argv)
        if argv[0:1]==['xwininfo']:
            return types.SimpleNamespace(stdout='Map State: IsViewable\nWidth: 1000\nHeight: 800\n')
        if argv[1:2]==['topic'] and '-i' in argv:
            raise AssertionError('topic-info CLI must not gate a working camera pose stream')
        if argv[1:2]==['service'] and '-i' in argv:
            return types.SimpleNamespace(stdout='Request msg type: ignition.msgs.GUICamera\nResponse msg type: ignition.msgs.Boolean\n')
        if argv[1:2]==['service']:
            Listener.restore_requested=True
        return types.SimpleNamespace(stdout='data: true\n')
    class Listener:
        returncode=None
        restore_requested=False
        restore_sent=False
        latest=None
        def __init__(self,argv,**kw):
            calls.append(argv);self.pid=123
            self.r,self.w=os.pipe();self.stdout=os.fdopen(self.r,'rb',buffering=0)
            pose=dict(position=dict(x=-1.7,y=5.4,z=8.4),orientation=dict(x=0,y=0,z=0,w=1))
            self.encoded=(json.dumps(pose)+'\n').encode();os.write(self.w,self.encoded);Listener.latest=self
        def poll(self):return self.returncode
        def terminate(self):self.returncode=-15;os.close(self.w)
        def wait(self,timeout):return self.returncode
    monkeypatch.setattr(media,'run',run);monkeypatch.setattr(batch.time,'sleep',lambda _:None)
    monkeypatch.setattr(batch.subprocess,'Popen',Listener)
    def fake_select(*args):
        if args[3]==0:return ([],[],[])
        if Listener.restore_requested and not Listener.restore_sent:
            os.write(Listener.latest.w,Listener.latest.encoded*3);Listener.restore_sent=True
        return ([args[0][0]],[],[])
    monkeypatch.setattr(batch.select,'select',fake_select)
    monkeypatch.setattr(batch,'camera_feedback_stable',lambda rows,target:len(rows)>=3)
    result=batch.restore_manual_camera(tmp_path,{},views,window={'id':1,'pid':2},owner=types.SimpleNamespace(poll=lambda:None),session=session,deadline=batch.time.monotonic()+10,display=':1')
    assert calls[0][0:2]==['xwininfo','-display'] and calls[1][:3]==['ign','topic','-e']
    assert not any(argv[1:2]==['topic'] and '-i' in argv for argv in calls)
    assert calls[3][-1]=='pose {\n'+pose+'\n}'
    assert result['manual_camera_restored'] and not result['visibility_verified']
    assert before=={p.name:p.read_bytes() for p in views.iterdir()}


def test_camera_pose_feedback_requires_finite_pose_structure():
    pose=dict(position=dict(x=1.,y=2.,z=3.),orientation=dict(x=0.,y=0.,z=0.,w=1.))
    assert batch.camera_pose_valid(pose)
    assert not batch.camera_pose_valid(dict(position=pose['position'],orientation=dict(x=float('nan'),y=0.,z=0.,w=1.)))
    assert not batch.camera_pose_valid(dict(position=pose['position'],orientation=dict(x=0.,y=0.,z=0.)))
    assert batch.gui_advertises_camera_pose('\x1b[1;32mCamera pose topic advertised on [/gui/camera/pose]\x1b[0m')


def test_camera_pose_completion_requires_repeated_stable_target_feedback():
    target=dict(position=[1.,2.,3.],orientation=[0.,0.,0.,1.])
    def sample(t,p=(1.,2.,3.)):return dict(receipt_wall=t,position=dict(zip('xyz',p)),orientation=dict(zip('xyzw',[0.,0.,0.,1.])))
    assert not batch.camera_feedback_stable([sample(1.)],target)
    assert not batch.camera_feedback_stable([sample(1.),sample(1.2),sample(1.4,[1.01,2,3])],target)
    assert batch.camera_feedback_stable([sample(1.),sample(1.1),sample(1.2)],target)
    assert batch.camera_feedback_stable([sample(2.),sample(2.04),sample(2.08),sample(2.12),sample(2.16)],target)
    assert batch.camera_feedback_stable([sample(3.+i*.02) for i in range(9)],target)
    assert not batch.camera_feedback_stable([sample(4.+i*.1,(1.02,2.,3.)) for i in range(4)],target)


def test_owner_process_cleanup_never_substitutes_for_measured_physical_rest():
    r=dict(normal_rest=dict(start_sim=0.,end_sim=6.,wall_s=3.,max_displacement_m=.001,max_yaw_rad=.001,
                            six_axis_final_zero=True,measured_ground_truth_rest=True,measured_lio_rest=True),
           normal_cleanup=dict(passed=True,remaining={}),watchdog=dict(reason='normal_shutdown',emergency=False))
    assert batch.owner_rest_is_qualified(r)
    emergency={**r,'watchdog':dict(reason='monitor:KeyboardInterrupt()',emergency=True,
                                   pause=dict(physical_stop_measured=False))}
    assert not batch.owner_rest_is_qualified(emergency)
    assert not batch.owner_rest_is_qualified({'normal_cleanup':dict(passed=True,remaining={})})


def test_missing_camera_feedback_closes_listener_without_restore_request(tmp_path,monkeypatch):
    import types,os
    views=tmp_path/'views';views.mkdir();session=tmp_path/'session';session.mkdir()
    (views/'gazebo_camera_pose_final.txt').write_text('position { x: 1 y: 2 z: 3 }\norientation { x: 0 y: 0 z: 0 w: 1 }\n')
    (views/'gazebo_gui.config').write_text('<gui/>')
    calls=[];listeners=[]
    def run(argv,**kw):
        calls.append(argv)
        if argv[0]=='xwininfo':return types.SimpleNamespace(stdout='Map State: IsViewable\nWidth: 800\nHeight: 600\n')
        return types.SimpleNamespace(stdout='No publishers on topic [/gui/camera/pose]\n')
    class Listener:
        returncode=None
        def __init__(self,*args,**kwargs):
            self.r,self.w=os.pipe();self.stdout=os.fdopen(self.r,'rb',buffering=0);listeners.append(self)
        def poll(self):return self.returncode
        def terminate(self):self.returncode=-15;os.close(self.w)
        def wait(self,timeout):return self.returncode
    monkeypatch.setattr(media,'run',run);monkeypatch.setattr(batch.subprocess,'Popen',Listener)
    monkeypatch.setattr(batch.time,'sleep',lambda _:None)
    with pytest.raises(batch.CameraSetupFailure,match='camera_pose_not_received_before_restore'):
        batch.restore_manual_camera(tmp_path,{},views,window={'id':1,'pid':2},owner=types.SimpleNamespace(poll=lambda:None),session=session,deadline=batch.time.monotonic()+.01,display=':1')
    assert listeners[0].returncode==-15
    assert not any(argv[1:2]==['topic'] and '-i' in argv for argv in calls)
    assert not any(argv[1:2]==['service'] and '-s' in argv for argv in calls)


def test_camera_failure_waits_for_owner_and_never_claims_emergency_rest(tmp_path,monkeypatch):
    import types,sys
    session=tmp_path/'session';session.mkdir()
    (session/'session-report.json').write_text(json.dumps(dict(normal_rest=None,
        normal_cleanup=dict(passed=True,remaining={}),watchdog=dict(reason='monitor:camera_failure',emergency=True,
        pause=dict(physical_stop_measured=False)))))
    class Owned:
        def __init__(self,token):pass
        def scan(self):return {'42':dict(pid=42)}
    monkeypatch.setitem(sys.modules,'narrow_processes',types.SimpleNamespace(OwnedProcesses=Owned))
    class Owner:
        returncode=0
        def poll(self):return 0
        def send_signal(self,*args):raise AssertionError('camera failure handler must not interrupt owner')
    result=batch.finish_owner_after_camera_failure(Owner(),1.,session,'token',[])
    assert result['owner_exited'] and not result['physical_rest_qualified']
    assert result['watchdog']['pause']['physical_stop_measured'] is False


def test_saved_topdown_view_is_reused_without_refitting(tmp_path):
    import yaml
    from g1_g5_presentation import rviz_config
    root=Path(__file__).resolve().parents[1];cfg=yaml.safe_load((root/'course_bringup/rviz/course.rviz').read_text());view=dict(Class='rviz_default_plugins/TopDownOrtho',Scale=37.,X=2.1,Y=3.2,Angle=.4)
    cfg['Visualization Manager']['Views']['Current']=view;source=tmp_path/'saved.rviz';source.write_text(yaml.safe_dump(cfg));before=source.read_bytes()
    rviz_config(tmp_path/'working.rviz',source=source)
    assert yaml.safe_load((tmp_path/'working.rviz').read_text())['Visualization Manager']['Views']['Current']==view
    assert source.read_bytes()==before
    assert all('SetGoal' not in t['Class'] for t in yaml.safe_load((tmp_path/'working.rviz').read_text())['Visualization Manager']['Tools'])


def test_stationary_modes_cannot_select_navigation():
    for flag in ['--stationary-setup','--record-only']:
        a=batch.parser().parse_args([flag]);assert not a.run
    with pytest.raises(SystemExit):batch.parser().parse_args(['--stationary-setup','--run'])


def test_manual_setup_never_moves_or_resizes_saved_windows(monkeypatch):
    monkeypatch.setattr(media,'arrange',lambda *a: (_ for _ in ()).throw(AssertionError('Manual windows must not be arranged')))
    assert batch.arrange_for_mode({},':1',True)['policy']=='saved/manual layouts preserved'


def test_splash_or_tool_windows_are_not_capture_targets():
    assert media.normal_window_kind('WM_CLASS rviz2 _NET_WM_WINDOW_TYPE_SPLASH') is None
    assert media.normal_window_kind('WM_CLASS rviz2 _NET_WM_WINDOW_TYPE_NORMAL')=='radar'
    assert media.normal_window_kind('WM_CLASS Gazebo GUI _NET_WM_WINDOW_TYPE_NORMAL')=='warehouse'


def test_real_waypoint_pose_and_metrics_proof_is_required(tmp_path):
    from run_warehouse_route import load_route,planned_goals
    goals=planned_goals(load_route(),'full',[0.,0.,0.]);segments=[];observed=[]
    for i,g in enumerate(goals):
        pose=g['pose'];p=pose['pose']['position'];q=pose['pose']['orientation'];yaw=math.atan2(2*q['w']*q['z'],1-2*q['z']**2)
        f=tmp_path/f'{i}.json';f.write_text(json.dumps(dict(goal_pose=pose,success=True,nav2_status=4)))
        segments.append(dict(name=g['name'],navigation_success=True,evaluation_passed=True,evidence_complete=True,nav2_result=dict(status=4),sent_pose=pose,metrics_file=str(f),metrics_sha256=media.digest(f)))
        observed.append(dict(name=g['name'],sent_pose=pose,sim=float(i+1),wall=10.+i,truth_stamp=float(i+1),truth_world_se2=[p['x'],p['y'],yaw],localized_se2=[p['x'],p['y'],yaw],contact_samples=1,contact_failure=None,readiness=dict(passed=True)))
    r=dict(_session_directory=str(tmp_path),route=dict(success=True,mode='full',segments=segments,planned_goals=goals,sent_poses=[g['pose'] for g in goals]),waypoint_observations=observed)
    view=dict(verified=True,world_from_map_se2=[0.,0.,0.])
    assert batch.verify_route(r,view)['passed']
    observed[-1]['truth_world_se2'][0]+=.06
    assert not batch.verify_route(r,view)['passed']
    observed[-1]['truth_world_se2'][0]-=.06
    observed[-1]['contact_failure']='collision'
    assert not batch.verify_route(r,view)['passed']
    observed[-1]['contact_failure']=None
    segments[-1]['navigation_success']=False;segments[-1]['nav2_result']['status']=6
    r['route']['success']=False
    assert [x['name'] for x in batch.verified_arrival_prefix(r,view)]==['G1','G2','G3','G4']
    observed[2]['truth_world_se2'][0]+=.06
    with pytest.raises(RuntimeError,match='physical arrival proof'):batch.verified_arrival_prefix(r,view)


def test_video_freeze_screen_uses_measured_motion_not_final_rest(tmp_path):
    rows=[]
    for wall,speed in [(10,.05),(10.4,.05),(10.8,.05),(11.2,.05),(11.6,.05),(12.,.05),(12.4,0.),(13.,0.),(20.,0.)]:
        rows.append(dict(kind='truth',wall=wall,message=dict(twist=dict(twist=dict(linear=dict(x=speed,y=0.),angular=dict(z=0.))))))
    (tmp_path/'monitor-events.jsonl').write_text('\n'.join(json.dumps(x) for x in rows)+'\n')
    assert batch.measured_motion_intervals(tmp_path)==[(10,12.)]


def test_ruby_single_title_server_is_not_missed_by_host_admission():
    assert batch.process_is_course_stack(['ign gazebo -r -v 3 -s /tmp/private.world'])
    assert batch.process_is_course_stack(['/usr/bin/ruby','/usr/bin/ign','gazebo','-g'])
    assert not batch.process_is_course_stack(['/usr/bin/python3','scripts/run_g1_g5_record.py'])


def test_batch_xlib_helper_is_bounded_and_token_owned(monkeypatch):
    from types import SimpleNamespace as N
    seen={}
    def run(argv,**kw):seen.update(kw);return N(stdout=json.dumps(dict(windows=dict(radar=dict(id=1)),screen=[1920,1080])))
    monkeypatch.setattr(media,'run',run)
    windows={};env={'B2W_NARROW_OWNER':'test'}
    assert batch.arrange_for_mode(windows,':1',False,env)==[1920,1080]
    assert seen['timeout']==5 and seen['env']==env and windows['radar']['id']==1


def test_collect_ten_continues_after_success_and_ordinary_failure_but_stops_on_safety():
    seen=[]
    def attempt(i):
        seen.append(i)
        if i==2:return dict(success=False,retry=True,reason='G2:6:nav2_failed')
        return dict(success=True,retry=False,reason='verified_success')
    result=batch.run_collect_batch(attempt,10,7200)
    assert seen==list(range(1,11)) and result['attempts']==10
    assert result['successes']==9 and result['success_rate']==.9
    seen.clear()
    def unsafe(i):
        seen.append(i)
        return dict(success=False,retry=False,reason='contact_safety_fault')
    assert batch.run_collect_batch(unsafe,10,7200)['stop_reason']=='contact_safety_fault'
    assert seen==[1]


def test_collect_repeated_ordinary_failure_stops_without_forcing_ten():
    result=batch.run_collect_batch(lambda i:dict(success=False,retry=True,reason='G1:6:nav2_failed'),10,7200)
    assert result['attempts']==2 and result['stop_reason']=='repeated_failure'
    sequence=[dict(success=False,retry=True,reason='G2:6:nav2_failed'),
              dict(success=True,retry=False,reason='verified_success'),
              dict(success=False,retry=True,reason='G2:6:nav2_failed')]
    result=batch.run_collect_batch(lambda i:sequence[i-1],10,7200)
    assert result['attempts']==3 and result['stop_reason']=='repeated_failure'


def test_unreviewed_collect_mode_stops_before_simulator_with_zero_attempts(tmp_path):
    assert batch.main(['--run','--collect-attempts','10','--output-root',str(tmp_path)])==2
    assert not list(tmp_path.rglob('attempt_*'))
    assert not list(tmp_path.rglob('*.mp4'))


def test_collect_mode_does_not_accept_stationary_or_unbounded_counts():
    for argv in [['--collect-attempts','10'],['--stationary-setup','--collect-attempts','10']]:
        assert batch.main(argv)==2
    with pytest.raises(SystemExit):batch.main(['--run','--collect-attempts','11'])


def test_collect_outputs_keep_separate_evidence_and_label_partial_recordings(tmp_path):
    source=tmp_path/'attempt_01';source.mkdir()
    for name in ['partial_radar.mp4','partial_warehouse.mp4','partial_combined.mp4']:(source/name).write_bytes(name.encode())
    row=dict(success=False,retry=True,reason='G2:6:nav2_failed',directory=str(source),
             videos=[dict(path=str(source/name)) for name in ['partial_radar.mp4','partial_warehouse.mp4','partial_combined.mp4']],
             waypoints=[dict(name='G1',truth_position_error_m=.01,truth_heading_error_rad=.02)])
    assert batch.project_attempt(tmp_path,1,row)=='failed'
    dest=tmp_path/'failed'/'attempt_01'
    assert (dest/'radar.mp4').read_bytes()==b'partial_radar.mp4'
    assert json.loads((dest/'report.json').read_text())['partial_video']
    assert (dest/'telemetry').resolve()==source.resolve()
    summary=batch.write_collect_summary(tmp_path,dict(requested=10,stop_reason='repeated_failure',results=[row]))
    assert summary['attempted']==1 and summary['successful']==0
    assert summary['failure_by_waypoint']=={'G2':1}
    assert 'G2' in (tmp_path/'summary.csv').read_text()


def test_real_video_review_requires_independent_acceptance_and_pinned_evidence(tmp_path):
    path=tmp_path/'recording_review.json'
    path.write_text(json.dumps(dict(accepted=True,reviewer='implementer',implementer='implementer',
                                    camera_inputs={},rviz_sha256='rviz',map_lidar_visible=True,
                                    route_visible=True,camera_pose_verified=True,videos_synchronized=True,
                                    evidence={})))
    with pytest.raises(RuntimeError,match='Independent real dual-video review'):
        batch.validate_recording_review(dict(path=str(path)),{},'rviz')
    row=json.loads(path.read_text());row['reviewer']='another_reviewer';path.write_text(json.dumps(row))
    with pytest.raises(RuntimeError,match='evidence incomplete'):
        batch.validate_recording_review(dict(path=str(path)),{},'rviz')
