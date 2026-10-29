"""Bind dispatch to immutable revisions and actual external loaded deployment."""
import copy
import hashlib
import json
import os
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))


def test_dispatch_rejects_post_preflight_grid_path_registration_or_frame_change():
    from stage4_route.dispatch import DispatchCertificate
    bindings=dict(path='a'*64,grids={'global':'b'*64,'local':'c'*64,'live':'d'*64},
                  registration='e'*64,start_pose=[0.,0.,0.],start_source=10.,frame='map')
    cert=DispatchCertificate.issue(bindings,wall=20.,sim=10.)
    cert.validate(bindings,wall=20.1,sim=10.1)
    for change in [dict(path='f'*64),dict(grids={'global':'f'*64}),dict(registration='f'*64),dict(frame='odom')]:
        with pytest.raises(ValueError):cert.validate({**bindings,**change},wall=20.1,sim=10.1)
    for wall,sim in [(20.6,10.1),(20.1,10.6),(19.9,10.)]:
        with pytest.raises(ValueError):cert.validate(bindings,wall=wall,sim=sim)


def test_dispatch_certificate_copies_callback_owned_bindings():
    from stage4_route.dispatch import DispatchCertificate
    value={'path':'a'*64,'grids':{'global':'b'*64}}
    cert=DispatchCertificate.issue(value,wall=20.,sim=10.)
    value['grids']['global']='c'*64
    with pytest.raises(ValueError):cert.validate(value,wall=20.1,sim=10.1)


def deployment(tmp_path):
    prefix=tmp_path/'external-copy-install/nav2_controller';exe=prefix/'lib/nav2_controller/controller_server'
    exe.parent.mkdir(parents=True);exe.write_bytes(b'pinned executable')
    lib=prefix/'lib/libcontroller_server_core.so';lib.write_bytes(b'pinned library')
    policy=dict(exe_sha256=hashlib.sha256(exe.read_bytes()).hexdigest(),library_sha256=hashlib.sha256(lib.read_bytes()).hexdigest())
    proc=tmp_path/'proc/123';proc.mkdir(parents=True);(proc/'exe').symlink_to(exe)
    fields=['S','1','123']+['0']*16+['999']+['0']*8
    (proc/'stat').write_text('123 (controller) '+' '.join(fields))
    st=lib.stat();device=f'{os.major(st.st_dev):02x}:{os.minor(st.st_dev):02x}'
    (proc/'maps').write_text(f'1000-2000 r-xp 00000000 {device} {st.st_ino} {lib}\n')
    (proc/'environ').write_bytes(b'B2W_NARROW_OWNER=owned\0')
    value=dict(kind='stage4_session_deployment',version=1,session_id='one',guard_manifest='/tmp/session/guard.json',
               controller=dict(prefix=str(prefix),executable=str(exe),library=str(lib),
                   process=dict(pid=123,start_ticks='999',pgid=123,uid=proc.stat().st_uid),owner_token='owned',**policy))
    return value,policy,tmp_path/'proc',proc,lib


def test_external_deployment_without_workspace_deps_and_exact_loaded_library(tmp_path):
    from stage4_route.deployment import verify_controller
    value,policy,procroot,proc,lib=deployment(tmp_path)
    report=verify_controller(value,policy,proc_root=procroot)
    assert report['pid']==123 and report['library']==str(lib)
    assert not (tmp_path/'.deps').exists()
    (proc/'maps').write_text((proc/'maps').read_text().replace(str(lib),'/foreign/libcontroller_server_core.so'))
    with pytest.raises(ValueError):verify_controller(value,policy,proc_root=procroot)


def test_pid_reuse_foreign_owner_or_changed_binary_is_rejected(tmp_path):
    from stage4_route.deployment import verify_controller
    value,policy,procroot,proc,lib=deployment(tmp_path)
    bad=copy.deepcopy(value);bad['controller']['process']['start_ticks']='998'
    with pytest.raises(ValueError):verify_controller(bad,policy,proc_root=procroot)
    (proc/'environ').write_bytes(b'B2W_NARROW_OWNER=foreign\0')
    with pytest.raises(ValueError):verify_controller(value,policy,proc_root=procroot)
    (proc/'environ').write_bytes(b'B2W_NARROW_OWNER=owned\0');lib.write_bytes(b'changed')
    with pytest.raises(ValueError):verify_controller(value,policy,proc_root=procroot)


def test_real_send_rejects_changed_costmap_after_preflight_before_action_submission():
    from stage4_route.ros_transport import RosTransport
    from stage4_route.dispatch import DispatchCertificate
    from types import SimpleNamespace as NS
    from unittest.mock import patch
    t=RosTransport.__new__(RosTransport);t.failure=None;t.journal=NS(failure=None)
    t.registration_graph={};t.graph=lambda:{};t.sim=lambda:10.;t.audit={}
    revision={'global':'before'};t.dispatch_bindings=lambda:dict(revision)
    current=dict(x=0.,y=0.,yaw=0.,tf_stamp=10.,safety='clear',speed=0.,wz=0.,command_speed=0.,command_wz=0.)
    t.sample=lambda:current
    def preflight(goals):
        t.certificate_start=current;t.checked_path=None
        import time
        t.dispatch_certificate=DispatchCertificate.issue(t.dispatch_bindings(),wall=time.monotonic(),sim=10.)
    t.preflight=preflight;t.ingest_dispatch_updates=lambda:None;calls=[]
    def capacity(*a):
        calls.append(1)
        if len(calls)==2:revision['global']='changed-after-plan'
    t.verify_recording_capacity=capacity
    t.nav=NS(send_goal_async=lambda *a,**k:pytest.fail('must not send invalidated path'))
    # Exercise dispatch wiring only; production interlock remains independently tested.
    with patch('stage4_route.motion_qualification.require_motion_qualification',lambda:None):
        with pytest.raises(ValueError,match='changed|expired'):t.send([], {})


def test_abort_uses_pinned_manifest_even_if_file_was_replaced(tmp_path):
    from stage4_route.ros_transport import RosTransport
    from course_navigation.stage4_control import read_control,write_control
    t=object.__new__(RosTransport);t.audit={};t.sim=lambda:10.
    original=tmp_path/'original-stop';other=tmp_path/'other-stop'
    manifest=tmp_path/'guard.json';write_control(manifest,dict(session_id='new',inhibit_path=str(other)))
    t.guard_config=dict(session_id='old',inhibit_path=str(original))
    t.request_independent_abort('fault')
    assert read_control(original)['reason']=='fault' and not other.exists()


def test_identical_map_geometry_advances_freshness_without_invalidating_sweep():
    from test_stage4_transport import transport
    from nav_msgs.msg import OccupancyGrid
    t=transport();m=OccupancyGrid();m.header.frame_id='map';m.header.stamp.sec=1
    m.info.width=m.info.height=2;m.info.resolution=.1;m.info.origin.orientation.w=1.;m.data=[0]*4
    t.receive('/map',m);before=copy.deepcopy(t.grid_revisions);old=t.grid_observations['/map']['cdr_sha256']
    m.header.stamp.sec=2;t.receive('/map',m)
    assert t.grid_revisions==before and t.grid_observations['/map']['source']==2.
    assert t.grid_observations['/map']['cdr_sha256']!=old
    m.data=[0,0,0,-1];t.receive('/map',m)
    assert t.grid_revisions!=before
