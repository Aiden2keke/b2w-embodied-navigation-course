#!/usr/bin/python3
"""Explicit owned Stage4 zero-only session; no motion stage or safety bypass."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'course_navigation'))
from stage4_route.contract import digest,write_new
from stage4_route.session import SessionOwner
from course_navigation.stage4_control import read_control,write_control,process_identity
from narrow_processes import OwnedProcesses
ROOT=Path(__file__).resolve().parents[1]


def observation_guard(guard):
    """Pin source inventory; observation carries no braking/registration assumptions."""
    if set(guard)!={'session_id','collision_inventory_path','collision_inventory_sha256'} or not guard['session_id']:
        raise ValueError('Zero-only guard requires only session ID and pinned source collision inventory')
    inv=Path(guard['collision_inventory_path'])
    if not inv.is_absolute() or digest(inv)!=guard['collision_inventory_sha256']:raise ValueError('Inventory identity changed')
    inventory=json.loads(inv.read_text());source=Path(inventory['converted_sdf'])
    if not source.is_absolute() or digest(source)!=inventory['converted_sdf_sha256']:raise ValueError('Source collision SDF changed')
    import xml.etree.ElementTree as ET
    actual=sorted(link.get('name')+'::'+collision.get('name')
        for link in ET.parse(source).getroot().find('model').findall('link') for collision in link.findall('collision'))
    if not actual or actual!=inventory['collisions']:raise ValueError('Collision inventory differs from source SDF')
    return dict(guard,zero_only=True)


def validate_inputs(path):
    domain=os.environ.get('ROS_DOMAIN_ID','')
    if not domain.isdigit() or not 0<=int(domain)<=232 or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise ValueError('Explicit ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1 required')
    value=read_control(path)
    if value['kind']!='stage4_zero_session_inputs' or value['version']!=2:raise ValueError('Explicit versioned zero-only inputs required')
    value['guard']=observation_guard(value['guard'])
    file=Path(value['world'])
    if not file.is_absolute() or digest(file)!=value['world_sha256']:raise ValueError('Absolute hash-bound world required')
    policy=Path(value['pretrained_directory'])/'POLICY_SOURCE.json'
    if not policy.is_absolute() or digest(policy)!=value['policy_source_sha256']:raise ValueError('Pinned unchanged policy source required')
    from prepare_rl_sar import REVISION,ASSET_REVISION,HASHES
    metadata=json.loads(policy.read_text());bundle=policy.parent
    if metadata['revision']!=REVISION or metadata['robot_assets']['revision']!=ASSET_REVISION:
        raise ValueError('Unchanged pinned RL-SAR revisions required')
    for name,expected in HASHES.items():
        if digest(bundle/name)!=expected:raise ValueError('RL-SAR source bytes changed: '+name)
    onnx='policy/b2w/robot_lab/policy.onnx'
    if digest(bundle/onnx)!=metadata['files'][onnx]['sha256']:raise ValueError('RL-SAR ONNX changed')
    # No fallback install, download or dependency link. The user sources the copy overlay.
    from ament_index_python.packages import get_package_prefix
    prefix=Path(get_package_prefix('course_navigation'))
    for source in (ROOT/'course_navigation/course_navigation').glob('*.py'):
        installed=prefix/'lib/python3.10/site-packages/course_navigation'/source.name
        if not installed.is_file() or digest(installed)!=digest(source):raise ValueError('Rebuild matching copy-installed source: '+source.name)
    for package in ('course_bringup','course_navigation'):
        share=Path(get_package_prefix(package))/'share'/package
        for source in (ROOT/package/'launch').glob('*.py'):
            if digest(share/'launch'/source.name)!=digest(source):raise ValueError('Copy-installed launch differs: '+str(source))
    prefix=Path(get_package_prefix('nav2_controller'));pinned=json.loads((ROOT/'course_navigation/config/stage4_runtime_identity.json').read_text())
    for file,key in ((prefix/'lib/nav2_controller/controller_server','exe_sha256'),(prefix/'lib/libcontroller_server_core.so','library_sha256')):
        if digest(file)!=pinned[key]:raise ValueError('Reviewed controller overlay required')
    value['controller_prefix']=str(prefix)
    return value


def evaluation_bridge_argv():
    """Stage4-only evaluation delivery; one-way, no command or truth inputs."""
    return ['/usr/bin/python3','/opt/ros/humble/bin/ros2','run','ros_gz_bridge','parameter_bridge',
        '/b2w/evaluation/full_contacts@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
        '/b2w/evaluation/collision_inventory@std_msgs/msg/String[ignition.msgs.StringMsg',
        '--ros-args','-r','__node:=stage4_evaluation_bridge','-p','use_sim_time:=true']


def execute(value,output):
    owner=SessionOwner(output,value['guard']);children=[];files=[];cleanup=None;error=None;probe_code=None;watchdog=None
    processes=OwnedProcesses(owner.token);started=time.monotonic();last_wait=None
    env=dict(os.environ,B2W_NARROW_OWNER=owner.token,IGN_PARTITION='b2w-stage4-'+owner.token,
        ROS_LOG_DIR=str(owner.directory/'ros-logs'),PYTHONDONTWRITEBYTECODE='1')
    def launch(argv,name):
        log=(owner.directory/(name+'.log')).open('xb');files.append(log)
        child=subprocess.Popen(argv,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,cwd=ROOT)
        children.append(child);processes.observe();return child
    try:
        # No RViz goal producer, goal manager, adaptive governor or alternate profile.
        argv=['/usr/bin/python3','/opt/ros/humble/bin/ros2','launch','course_bringup','course.launch.py',
            'mode:=navigation','scene:=warehouse','gui:=false','rviz:=false','goal_manager_enabled:=false',
            'adaptive_speed:=false','map_mode:=online','locomotion_backend:=rl_sar',
            'nav_params_file:='+str(ROOT/'course_navigation/config/nav2_smac2d_diag.yaml'),
            'world:='+value['world'],'pretrained_directory:='+value['pretrained_directory'],
            'stage4_guard_manifest:='+str(owner.directory/'guard.json')]
        write_new(owner.directory/'invocation.json',dict(argv=argv,inputs=value,owner=owner.config['owner'],token=owner.token,
            motion_authorized=False))
        stack=launch(argv,'stack')
        evaluation_bridge=launch(evaluation_bridge_argv(),'evaluation-bridge')
        while time.monotonic()-started<90.:
            now=time.monotonic();owner.heartbeat(now);processes.observe()
            if stack.poll() is not None:raise ValueError('Owned stack exited during bootstrap')
            if evaluation_bridge.poll() is not None:raise ValueError('Owned evaluation bridge exited during bootstrap')
            if Path(owner.config['inhibit_path']).exists():raise ValueError('Adapter inhibited during bootstrap')
            try:
                health=read_control(owner.config['adapter_heartbeat_path']);manifest=owner.accept_adapter(health,wall=now)
                from stage4_route.deployment import verify_controller
                prefix=Path(value['controller_prefix']);exe=prefix/'lib/nav2_controller/controller_server';lib=prefix/'lib/libcontroller_server_core.so'
                candidates=[]
                for pid in processes.scan():
                    try:
                        if Path(f'/proc/{pid}/exe').resolve()==exe:candidates.append(process_identity(pid))
                    except (OSError,ValueError):continue
                if len(candidates)!=1:raise ValueError('Need unique owned reviewed controller')
                pinned=json.loads((ROOT/'course_navigation/config/stage4_runtime_identity.json').read_text())
                deployment=dict(kind='stage4_session_deployment',version=1,session_id=owner.config['session_id'],
                    guard_manifest=str(owner.directory/'guard.json'),guard_manifest_sha256=digest(owner.directory/'guard.json'),
                    adapter=manifest['adapter'],controller=dict(prefix=str(prefix),executable=str(exe),library=str(lib),
                        process=candidates[0],owner_token=owner.token,**pinned))
                verify_controller(deployment,pinned)
            except (OSError,ValueError,KeyError,TypeError) as exc:last_wait=str(exc);time.sleep(.02);continue
            write_control(owner.directory/'watchdog.json',manifest);write_new(owner.directory/'deployment.json',deployment)
            watchdog=launch(['/usr/bin/python3',str(ROOT/'scripts/stage4_safety_watchdog.py'),
                '--manifest',str(owner.directory/'watchdog.json'),'--wall-limit','150'],'watchdog')
            break
        if watchdog is None:raise ValueError('Bootstrap timeout: '+str(last_wait))
        probe=launch(['/usr/bin/python3',str(ROOT/'scripts/stage4_zero_probe.py'),'--deployment',str(owner.directory/'deployment.json'),'--output',str(owner.directory/'probe')],'probe')
        while time.monotonic()-started<220.:
            owner.heartbeat(time.monotonic());processes.observe()
            if watchdog.poll() is not None or stack.poll() is not None or evaluation_bridge.poll() is not None:
                raise ValueError('Owned watchdog/stack/evaluation bridge exited')
            if Path(owner.config['inhibit_path']).exists():raise ValueError('Independent inhibit latched')
            if probe.poll() is not None:probe_code=probe.returncode;break
            time.sleep(.02)
        if probe_code is None:raise ValueError('Zero-only session wall timeout')
    except (Exception,KeyboardInterrupt) as exc:error=repr(exc)
    finally:
        try:owner.stop(error or 'zero_session_complete')
        except OSError as exc:error=(error or '')+' inhibit_io: '+repr(exc)
        finally:
            if children:
                cleanup=processes.cleanup(children[0])
                for child in children:
                    try:child.wait(timeout=1.)
                    except subprocess.TimeoutExpired:pass
            for file in files:file.close()
    result=dict(kind='stage4_zero_session',zero_motion_smoke_pass=probe_code==0 and not error and bool(cleanup and cleanup['passed']),
        probe_exit_code=probe_code,error=error,cleanup=cleanup,live_stack_wall_s=time.monotonic()-started,
        motion_attempts=0,motion_authorized=False,safety_complete=False,full_route_pass=False,segment_pass=False)
    write_new(owner.directory/'session-result.json',result)
    return 0 if result['zero_motion_smoke_pass'] else 1


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--inputs',type=Path,required=True)
    p.add_argument('--output',type=Path)
    modes=p.add_mutually_exclusive_group()
    modes.add_argument('--execute-zero-smoke',action='store_true')
    modes.add_argument('--check-e1-request',action='store_true',
        help='Read-only request/accounting check; confers no motion authority')
    modes.add_argument('--execute-e1',action='store_true',
        help='Reserved future interface; always rejected as unqualified')
    p.add_argument('--qualification',type=Path,help='Reserved future evidence path; cannot authorize motion')
    a=p.parse_args(argv)
    try:
        from course_navigation.stage4_qualification import require_motion_qualification,validate_e1_pulse_request
        # Reject before reading artifacts, creating an owner or launching anything.
        if a.execute_e1:require_motion_qualification()
        if a.qualification is not None:raise ValueError('Qualification artifacts cannot authorize this zero-only owner')
        if a.check_e1_request:
            validate_e1_pulse_request(read_control(a.inputs))
            print('E1 request shape/content identities verified; NONZERO remains NO-GO')
            return 0
        if a.output is None:raise ValueError('--output required for zero-only inputs/session')
        value=validate_inputs(a.inputs)
        if not a.execute_zero_smoke:print('Inputs verified; no session launched');return 0
        return execute(value,a.output)
    except (ValueError,KeyError,TypeError,OSError,RuntimeError) as exc:
        print('Stage4 session rejected: '+str(exc),file=sys.stderr);return 2

if __name__=='__main__':raise SystemExit(main())
