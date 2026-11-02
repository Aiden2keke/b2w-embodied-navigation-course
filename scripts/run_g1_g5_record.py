#!/usr/bin/env python3
"""Retired automatic video workflow; use run_warehouse_navigation.py for manual recording."""
import argparse,csv,fcntl,json,math,os,re,shlex,signal,subprocess,sys,time,uuid
import select
from datetime import datetime,timezone
from pathlib import Path
import g1_g5_media as media
ROOT=Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT=ROOT/'course_results/tracks'
MANUAL_VIEWS=ROOT/'artifacts/video/manual_views'


def bounded_int(high):
    def value(text):
        n=int(text)
        if not 1<=n<=high:raise argparse.ArgumentTypeError(f'Must be1..{high}')
        return n
    return value

def parser():
    p=argparse.ArgumentParser(description=__doc__);m=p.add_mutually_exclusive_group()
    m.add_argument('--stationary-setup',action='store_true',help='Retired')
    m.add_argument('--record-only',action='store_true',help='Retired')
    m.add_argument('--prepare-only',action='store_true',help='Retired')
    m.add_argument('--dry-run',action='store_true',help='Offline requirements review; no ROS or GUI launch')
    m.add_argument('--run',action='store_true',help='Retired; use scripts/run_warehouse_navigation.py --run')
    p.add_argument('--stationary-seconds',type=bounded_int(90),default=60,help='Stationary setup viewing time,1..90wall seconds; original owner watchdog remains150s')
    p.add_argument('--max-attempts',type=bounded_int(12),default=12);p.add_argument('--max-total-minutes',type=bounded_int(120),default=120)
    p.add_argument('--collect-attempts',type=bounded_int(10),help='Collect at most ten independent results; requires --run')
    p.add_argument('--output-root',type=Path,default=DEFAULT_OUTPUT)
    p.add_argument('--review-config',type=Path,default=DEFAULT_OUTPUT/'reviewed_run.json')
    return p

def classify(r):
    for key,reason in [('contact_failure','contact_safety_fault'),('truth_first_fault','physical_truth_fault'),
                       ('readiness_first_fault','localization_or_tf_fault'),('watchdog_pending','watchdog_pending'),
                       ('error','owner_error')]:
        if r.get(key):return dict(success=False,retry=False,reason=reason)
    rest=r.get('normal_rest',{});cleanup=r.get('normal_cleanup',{});watch=r.get('watchdog',{})
    if watch.get('emergency') or watch.get('reason')!='normal_shutdown':return dict(success=False,retry=False,reason='watchdog_emergency_or_abnormal_shutdown')
    if cleanup.get('passed') is not True or cleanup.get('remaining')!={} or r.get('localization_cleanup',{}).get('passed') is not True:
        return dict(success=False,retry=False,reason='incomplete_owned_cleanup')
    safe=(cleanup.get('passed') is True and cleanup.get('remaining')=={} and
          r.get('localization_cleanup',{}).get('passed') is True and watch==dict(reason='normal_shutdown',emergency=False) and
          all(rest.get(k) is True for k in ['measured_ground_truth_rest','measured_lio_rest','six_axis_final_zero']) and
          rest.get('end_sim',0)-rest.get('start_sim',0)>=5 and rest.get('wall_s',0)>=3)
    if not safe:return dict(success=False,retry=False,reason='unqualified_final_rest')
    route=r.get('route',{});segments=route.get('segments',[])
    if r.get('success') and route.get('success'):return dict(success=False,retry=False,reason='requires_physical_and_video_verification')
    if segments:
        last=segments[-1];nav=last.get('nav2_result',{})
        if nav.get('status')==6 and nav.get('end_reason') in ['aborted','nav2_failed','navigation_failed','timeout','navigation_timeout']:
            return dict(success=False,retry=True,reason=f"{last.get('name','unknown')}:{nav['status']}:{nav['end_reason']}")
    return dict(success=False,retry=False,reason=route.get('stop_reason','unclassified_failure'))

def run_batch(attempt,max_attempts,total_seconds,clock=time.monotonic):
    if not 1<=max_attempts<=12 or not 0<=total_seconds<=7200:raise ValueError('Batch exceeds authorized bounds')
    end=clock()+total_seconds;rows=[];previous=None;stop='attempt_limit'
    for index in range(1,max_attempts+1):
        if clock()>=end:stop='time_limit';break
        row=attempt(index);rows.append(row)
        if row.get('success'):stop='verified_success';break
        if not row.get('retry'):stop=row.get('reason','hard_stop');break
        if row['reason']==previous:stop='repeated_failure';break
        previous=row['reason']
    return dict(success=bool(rows and rows[-1].get('success')),attempts=len(rows),stop_reason=stop,results=rows)

def run_collect_batch(attempt,count,total_seconds,clock=time.monotonic):
    if not 1<=count<=10 or not 0<=total_seconds<=7200:raise ValueError('Collection exceeds authorized bounds')
    end=clock()+total_seconds;rows=[];failure_causes=set();stop='collected_requested_attempts'
    for index in range(1,count+1):
        if clock()>=end:stop='time_limit';break
        row=attempt(index);rows.append(row)
        if row.get('success'):
            continue
        if not row.get('retry'):
            stop=row.get('reason','hard_stop');break
        if row.get('reason') in failure_causes:
            stop='repeated_failure';break
        failure_causes.add(row.get('reason'))
    successes=sum(bool(r.get('success')) for r in rows)
    return dict(attempts=len(rows),successful=successes,successes=successes,
                success_rate=successes/len(rows) if rows else None,
                completed=len(rows)==count and stop=='collected_requested_attempts',
                stop_reason=stop,results=rows)

def project_attempt(base,index,row):
    """Keep immutable owner paths valid; expose files through a category directory."""
    category='success' if row.get('success') else ('failed' if row.get('retry') else 'blocked')
    dest=base/category/f'attempt_{index:02d}';dest.mkdir(parents=True)
    source=Path(row['directory']).resolve()
    if not source.is_relative_to(base.resolve()):raise RuntimeError('Attempt evidence is outside this batch')
    (dest/'telemetry').symlink_to(source,target_is_directory=True)
    for item in row.get('videos',[]):
        path=Path(item['path']).resolve()
        if not path.is_relative_to(source) or not path.is_file():raise RuntimeError('Video does not belong to attempt')
        name=path.name
        for prefix in ('success_','partial_'):
            if name.startswith(prefix):name=name[len(prefix):];break
        os.link(path,dest/name)
    media.write(dest/'report.json',dict(row,classification=category,attempt=index,
                                      video_complete=bool(row.get('success')),
                                      partial_video=bool(row.get('videos') and not row.get('success'))))
    return category

def write_collect_summary(base,result):
    rows=[];failure_by_reason={};failure_by_waypoint={}
    for index,row in enumerate(result.get('results',[]),1):
        category='success' if row.get('success') else ('failed' if row.get('retry') else 'blocked')
        waypoints=row.get('waypoints',[])
        route_actions=[dict(name=s.get('name'),result=s.get('nav2_result')) for s in (row.get('route') or {}).get('segments',[])]
        rows.append(dict(attempt=index,classification=category,success=bool(row.get('success')),
                         reason=row.get('reason',''),furthest_verified_waypoint=waypoints[-1]['name'] if waypoints else '',
                         waypoint_results=json.dumps(waypoints,separators=(',',':')),
                         route_actions=json.dumps(route_actions,separators=(',',':')),
                         elapsed_wall_s=row.get('elapsed_wall_s'),
                         video_complete=bool(row.get('success')),
                         partial_video=bool(row.get('videos') and not row.get('success')),
                         contact_fault=bool(row.get('contact_failure')),
                         watchdog_result=(row.get('watchdog') or {}).get('reason',''),
                         cleanup_passed=(row.get('normal_cleanup') or {}).get('passed') is True,
                         owner_report=row.get('owner_report',''),directory=row.get('directory','')))
        if category!='success':
            reason=row.get('reason','unknown');failure_by_reason[reason]=failure_by_reason.get(reason,0)+1
            reached=rows[-1]['furthest_verified_waypoint'];names=['G1','G2','G3','G4','G5']
            next_goal=names[names.index(reached)+1] if reached in names[:-1] else ('pre_route_or_G1' if not reached else 'post_G5')
            failure_by_waypoint[next_goal]=failure_by_waypoint.get(next_goal,0)+1
    complete=[r for r in rows if r['success']]
    best=min(complete,key=lambda r:r['elapsed_wall_s']) if complete else None
    summary=dict(requested=result.get('requested'),attempted=len(rows),successful=sum(r['success'] for r in rows),
                 success_rate=sum(r['success'] for r in rows)/len(rows) if rows else None,
                 stop_reason=result.get('stop_reason'),failure_by_reason=failure_by_reason,
                 failure_by_waypoint=failure_by_waypoint,
                 best_verified_complete_combined=str(base/'success'/f"attempt_{best['attempt']:02d}"/'combined.mp4') if best else None,
                 best_selection='shortest verified successful elapsed wall time' if best else None,
                 attempts=rows)
    media.write(base/'summary.json',summary)
    with (base/'summary.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=['attempt','classification','success','reason','furthest_verified_waypoint','waypoint_results','route_actions','elapsed_wall_s','video_complete','partial_video','contact_fault','watchdog_result','cleanup_passed','owner_report','directory']);writer.writeheader();writer.writerows(rows)
    return summary

def verify_route(r, view, count=5):

    def require(condition):
        if not condition:
            raise ValueError('Physical waypoint proof failed')
    try:
        from run_warehouse_route import load_route, planned_goals
        route = r['route']
        segments = route['segments']
        observed = r['waypoint_observations']
        goals = route['planned_goals']
        require((route['success'] or count<5) and route['mode'] == 'full' and len(route['sent_poses']) == count)
        require([x['name'] for x in segments] == ['G1', 'G2', 'G3', 'G4', 'G5'][:count] and len(observed) == count and view['verified'])
        canonical = load_route()['goals'][:count]
        metrics = []
        last = -math.inf
        t = view['world_from_map_se2']
        angle = lambda x: math.atan2(math.sin(x), math.cos(x))
        for (i, (g, s, o)) in enumerate(zip(goals, segments, observed)):
            require(s['navigation_success'] and s['evaluation_passed'] and s['evidence_complete'] and (s['nav2_result']['status'] == 4))
            require(g['name'] == canonical[i]['name'] == o['name'])
            pose = g['pose']['pose']
            goal = [pose['position'][k] for k in ['x', 'y']]
            require(goal == canonical[i]['position'] and o['sent_pose'] == s['sent_pose'])
            metric_path = Path(s['metrics_file']).resolve()
            require(metric_path.is_relative_to(Path(r['_session_directory']).resolve()))
            payload = json.loads(metric_path.read_text())
            require(payload['goal_pose'] == s['sent_pose'] and payload['success'] is True and (payload['nav2_status'] == 4))
            require(media.digest(s['metrics_file']) == s['metrics_sha256'] and o['sim'] > last and (not o['contact_failure']) and (o['contact_samples'] > 0))
            require(0 <= o['sim'] - o['truth_stamp'] <= 0.5 and o['readiness']['passed'])
            q = pose['orientation']
            yaw = math.atan2(2 * (q['w'] * q['z'] + q['x'] * q['y']), 1 - 2 * (q['y'] ** 2 + q['z'] ** 2))
            target = media.world_points([goal + [0]], t)[0]
            truth = o['truth_world_se2']
            est = o['localized_se2']
            te = math.dist(truth[:2], target[:2])
            ee = math.dist(est[:2], goal)
            th = abs(angle(truth[2] - yaw - t[2]))
            eh = abs(angle(est[2] - yaw))
            require(te <= 0.05 and ee <= 0.05 and (th <= 0.3) and (eh <= 0.3))
            metrics.append(dict(name=g['name'], action_status=4, truth_world=truth, localized_map=est, truth_position_error_m=te, estimated_position_error_m=ee, truth_heading_error_rad=th, estimated_heading_error_rad=eh, arrival_sim=o['sim'], arrival_wall=o['wall'], contact_failure=None, metrics_sha256=s['metrics_sha256']))
            last = o['sim']
        return dict(passed=True, waypoints=metrics)
    except (KeyError, AssertionError, ValueError, OSError, TypeError) as e:
        return dict(passed=False, reason='physical_waypoint_verification_failed', detail=repr(e))

def verified_arrival_prefix(r,view):
    route=r['route'];segments=route['segments'];expected=['G1','G2','G3','G4','G5']
    if len(segments)>5 or [s.get('name') for s in segments]!=expected[:len(segments)]:
        raise RuntimeError('Route segment order changed')
    count=0
    for segment in segments:
        if not (segment.get('navigation_success') and segment.get('evaluation_passed') and
                segment.get('evidence_complete') and segment.get('nav2_result',{}).get('status')==4):break
        count+=1
    if count==0:return []
    candidate=dict(r,route=dict(route,segments=segments[:count],sent_poses=route['sent_poses'][:count],
                                planned_goals=route['planned_goals'][:count]),
                   waypoint_observations=r.get('waypoint_observations',[])[:count])
    proof=verify_route(candidate,view,count=count)
    if not proof['passed']:raise RuntimeError('Prior waypoint lacks independent physical arrival proof: '+proof['detail'])
    return proof['waypoints']

def measured_motion_intervals(session):
    intervals=[];start=last=None
    for line in (Path(session)/'monitor-events.jsonl').open():
        e=json.loads(line)
        if e.get('kind')!='truth':continue
        wall=e['wall'];twist=e['message']['twist']['twist']
        moving=math.hypot(twist['linear']['x'],twist['linear']['y'])>.015 or abs(twist['angular']['z'])>.02
        if start is not None and (not moving or wall-last>.5):intervals.append((start,last));start=last=None
        if moving:
            if start is None:start=wall
            last=wall
    if start is not None:intervals.append((start,last))
    return intervals

def read_review(path):
    """Read externally reviewed inputs. Never manufacture qualification or envelope."""
    side=path.with_suffix(path.suffix+'.sha256')
    if not path.is_file() or not side.is_file():
        def local_name(p):
            try:return p.resolve().relative_to(ROOT).as_posix()
            except ValueError:return str(p)
        raise RuntimeError('Missing independently reviewed '+local_name(path)+' and '+local_name(side)+'; retained heading/Profile-B envelope remain unqualified')
    expected=side.read_text().split()[0]
    if media.digest(path)!=expected:raise RuntimeError('Reviewed run configuration SHA256 changed')
    cfg=json.loads(path.read_text())
    for key in ['deployment_manifest','route_envelope','control_review']:
        if key not in cfg:raise RuntimeError('Missing independently reviewed '+key)
        item=cfg[key];p=Path(item['path'])
        if media.digest(p)!=item['sha256']:raise RuntimeError(key+' pin mismatch')
    files=['scripts/run_warehouse_navigation.py','scripts/warehouse_track_artifacts.py',
           'scripts/run_g1_g5_record.py',
           'scripts/warehouse_ab_trial.py','scripts/run_warehouse_route.py','scripts/warehouse_route_admission.py']
    if set(cfg.get('source_pins',{}))!=set(files) or any(media.digest(ROOT/p)!=cfg['source_pins'][p] for p in files):raise RuntimeError('Independently reviewed automation source pins changed/incomplete')
    control=json.loads(Path(cfg['control_review']['path']).read_text())
    required=['retained_heading','bidirectional_turning','small_command_tracking','profile_B_directional_stopping','balance_and_stop','independent_review']
    if not all(control.get(k) is True for k in required):raise RuntimeError('Controller/route qualification not accepted')
    if not control.get('reviewer') or control.get('reviewer')==control.get('implementer'):raise RuntimeError('Independent scientific review absent')
    if control.get('deployment_manifest_sha256')!=cfg['deployment_manifest']['sha256']:raise RuntimeError('Control qualification belongs to another deployment')
    for e in control['evidence']:
        if media.digest(e['path'])!=e['sha256']:raise RuntimeError('Control raw evidence changed')
    if not control['evidence']:raise RuntimeError('No empirical controller qualification')
    y=cfg['continuation_yaws']
    if len(y)!=3 or not all(type(v) in (int,float) and math.isfinite(v) for v in y):raise RuntimeError('G3-G5 reviewed arrival headings absent')
    from warehouse_route_admission import load_envelope
    cfg['envelope']=load_envelope(cfg['route_envelope']['path'],cfg['route_envelope']['sha256'],media.digest(ROOT/'artifacts/recovery/robot.urdf'),media.digest(ROOT/'course_navigation/config/nav2_smac2d_dwb_slow_diag.yaml'),[.2,.6],cfg['deployment_manifest']['sha256'])
    cfg['_review_path']=str(path);cfg['_review_sha256']=expected
    return cfg

def validate_recording_review(item,camera_pins,rviz_sha256):
    """Require a separately reviewed real stationary dual-video check for this exact view."""
    review=json.loads(Path(item['path']).read_text())
    if (review.get('accepted') is not True or not review.get('reviewer') or
            review.get('reviewer')==review.get('implementer') or
            review.get('camera_inputs')!=camera_pins or review.get('rviz_sha256')!=rviz_sha256 or
            not all(review.get(k) is True for k in ['map_lidar_visible','route_visible','camera_pose_verified','videos_synchronized'])):
        raise RuntimeError('Independent real dual-video review absent or stale')
    evidence=review.get('evidence',{})
    if set(evidence)!={'attempt_report','camera_restore','framing','owner_report'}:
        raise RuntimeError('Real dual-video review evidence incomplete')
    for pin in evidence.values():
        if media.digest(pin['path'])!=pin['sha256']:raise RuntimeError('Real dual-video evidence hash changed')
    attempt=json.loads(Path(evidence['attempt_report']['path']).read_text())
    owner=json.loads(Path(evidence['owner_report']['path']).read_text())
    camera=json.loads(Path(evidence['camera_restore']['path']).read_text())
    framing=json.loads(Path(evidence['framing']['path']).read_text())
    if (attempt.get('prepared') is not True or len(attempt.get('video',[]))!=3 or
            owner.get('success') is not True or owner.get('stationary_view',{}).get('no_navigation') is not True or
            owner.get('stationary_commands',{}).get('nonzero')!=0 or
            owner.get('normal_cleanup',{}).get('passed') is not True or
            owner.get('normal_cleanup',{}).get('remaining')!={} or
            not all(owner.get('normal_rest',{}).get(k) is True for k in
                    ['measured_ground_truth_rest','measured_lio_rest','six_axis_final_zero']) or
            owner.get('watchdog')!=dict(reason='normal_shutdown',emergency=False) or
            camera.get('manual_camera_restored') is not True or camera.get('saved_inputs')!=camera_pins or
            framing.get('manual_camera_restored') is not True or not framing.get('verified')):
        raise RuntimeError('Real stationary recording did not pass original owner and view gates')
    names={'prepare_radar.mp4','prepare_warehouse.mp4','prepare_combined.mp4'}
    if {Path(v['path']).name for v in attempt['video']}!=names:
        raise RuntimeError('Real dual-video set incomplete')
    binary=media.ffmpeg()
    for v in attempt['video']:
        if (media.digest(v['path'])!=v['sha256'] or v.get('codec')!='h264' or v.get('fps')!=25 or
                v.get('black_empty') is not False or v.get('duration_s',0)<5):
            raise RuntimeError('Real stationary video decode/identity failed')
        current=media.inspect_video(binary,v['path'])
        if current['sha256']!=v['sha256'] or abs(current['duration_s']-v['duration_s'])>.05:
            raise RuntimeError('Real stationary video no longer decodes as reviewed')
    durations=[v['duration_s'] for v in attempt['video']]
    if max(durations)-min(durations)>.15:raise RuntimeError('Real stationary dual videos are not synchronized')

def ros_env(domain):
    script='source /opt/ros/humble/setup.bash && source install/setup.bash && env -0'
    raw=subprocess.check_output(['/bin/bash','-c',script],cwd=ROOT);env=dict(x.split('=',1) for x in raw.decode().split('\0') if '=' in x)
    env.update(ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1',FASTRTPS_DEFAULT_PROFILES_FILE=str(ROOT/'course_sim/config/fastdds_local.xml'),QT_QPA_PLATFORM='xcb')
    env['LD_LIBRARY_PATH']=str(ROOT/'.deps/lib')+':'+str(ROOT/'b2w_sim/b2w_controllers/third_party/onnxruntime/lib64')+':'+env.get('LD_LIBRARY_PATH','')
    return env

def process_is_course_stack(argv):
    try:parts=shlex.split(' '.join(argv))
    except ValueError:return any('ign gazebo' in x for x in argv)
    names={Path(x).name for x in parts[:2]}
    return bool(names & {'b2w_rl_sar_controller','lio_sam_mapOptimization','warehouse_ab_trial.py'} or any(Path(x).name in ['ign','gz'] and parts[i+1:i+2]==['gazebo'] for i,x in enumerate(parts)))

def host_admission():
    gpu=media.run(['nvidia-smi','--query-compute-apps=pid,process_name,used_memory','--format=csv,noheader']).stdout.strip()
    if gpu:raise RuntimeError('Insufficient GPU isolation: existing compute process; no job will be modified')
    competing=[]
    for p in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            a=[x.decode(errors='replace') for x in p.read_bytes().split(b'\0') if x]
            if not a:continue
            if process_is_course_stack(a):competing.append(str(p.parent))
        except OSError:pass
    if competing:raise RuntimeError('Existing simulator/controller ownership: '+','.join(competing))
    return dict(gpu_compute_empty=True,competing=[],gpu=media.run(['nvidia-smi','--query-gpu=name,uuid,utilization.gpu,memory.used','--format=csv']).stdout)

def fresh_domain():
    from course_sim.session import SessionLease
    for domain in range(120,233):
        lock=ROOT/'course_results'/f'.b2w-course-{os.getuid()}-domain-{domain}.lock'
        if lock.exists():continue
        occupied=False
        for e in Path('/proc').glob('[0-9]*/environ'):
            try:
                if f'ROS_DOMAIN_ID={domain}'.encode() in e.read_bytes().split(b'\0'):occupied=True;break
            except OSError:pass
        if not occupied:
            lease=SessionLease(domain);lease.close();return domain
    raise RuntimeError('No fresh unused ROS domain')

def prepare_manifest(directory):
    # Zero-only scope; strict old native/path set is preserved. Never route evidence.
    m=json.loads((ROOT/'artifacts/recovery/root-installed-manifest.json').read_text());ctrl=str(ROOT/'install/b2w_controllers/lib/b2w_controllers/b2w_rl_sar_controller');old=m['native'][ctrl]['sha256']
    current=media.digest(ctrl)
    if current!='aa2e03e68e777cea84aea8e5a4e4f6d82477f5b9d3265df8a657bc7eb1ccacc5':raise RuntimeError('No previously qualified stationary native available for GUI-only preparation')
    for i in m['source_install']:
        i['source_sha256']=media.digest(i['source']);i['installed_sha256']=media.digest(i['installed']);i['match']=i['source_sha256']==i['installed_sha256']
        if not i['match']:raise RuntimeError('Source/install mismatch')
    for p,v in m['native'].items():
        if p==ctrl:v['sha256']=current
        elif media.digest(p)!=v['sha256']:raise RuntimeError('Other native pin changed')
    files=['b2w_sim/b2w_controllers/src/rl_sar_controller.cpp','b2w_sim/b2w_controllers/src/differential_stop_brake.hpp','course_sim/config/rl_sar_stop_brake.yaml','scripts/warehouse_ab_trial.py','scripts/observe_warehouse_narrow_failure.py']
    m['controller_revision']=dict(baseline_controller_sha256=old,source_pins={str(ROOT/p):media.digest(ROOT/p) for p in files})
    p=directory/'zero-only-deployment.json';media.write(p,m);return dict(path=str(p),sha256=media.digest(p))

def arrange_for_mode(windows,display,manual,env=None):
    # Manual setup never rearranges saved windows. Batch arrangement uses a
    # bounded token-owned helper so Xlib cannot kill or hang the coordinator.
    if manual:return {'policy':'saved/manual layouts preserved'}
    program="import json,sys;sys.path.insert(0,'scripts');import g1_g5_media as m;w=json.load(sys.stdin);s=m.arrange(w,sys.argv[1]);print(json.dumps(dict(windows=w,screen=s)))"
    result=media.run(['/usr/bin/python3','-c',program,display],input=json.dumps(windows),env=env,cwd=ROOT,timeout=5)
    data=json.loads(result.stdout);windows.update(data['windows']);return data['screen']

class CameraSetupFailure(RuntimeError):
    def __init__(self,reason,detail=None):
        self.reason=reason;self.detail=detail
        super().__init__(reason+((': '+str(detail)) if detail else ''))

def camera_pose_valid(pose):
    """Accept only a complete finite Fortress Pose from the dedicated topic stream."""
    try:
        xyz=[float(pose['position'][k]) for k in 'xyz']
        q=[float(pose['orientation'][k]) for k in 'xyzw']
        norm=sum(v*v for v in q)
        return all(math.isfinite(v) for v in xyz+q) and abs(norm-1.0)<=1e-3
    except (KeyError,TypeError,ValueError):return False

def gui_advertises_camera_pose(log_text):
    clean=re.sub(r'\x1b\[[0-?]*[ -/]*[@-~]','',log_text)
    return 'Camera pose topic advertised on [/gui/camera/pose]' in clean

def startup_camera_config(views, destination):
    """Write a new config with only MinimalScene's camera pose changed."""
    raw=(views/'gazebo_camera_pose_final.txt').read_text()
    def values(name, axes):
        match=re.search(name+r'\s*\{([^}]+)\}',raw)
        if not match:raise ValueError('Saved camera lacks '+name)
        data={k:float(v) for k,v in re.findall(r'([xyzw]):\s*([-+0-9.eE]+)',match[1])}
        result=[data[k] for k in axes]
        if not all(math.isfinite(v) for v in result):raise ValueError('Invalid camera pose')
        return result
    xyz=values('position','xyz');x,y,z,w=values('orientation','xyzw')
    if abs(x*x+y*y+z*z+w*w-1)>1e-4:raise ValueError('Invalid camera quaternion')
    roll=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    pitch=math.asin(max(-1.,min(1.,2*(w*y-z*x))))
    yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    config=(views/'gazebo_gui.config').read_text()
    pattern=r'(<plugin\s+filename="MinimalScene"[^>]*>.*?<camera_pose>)([^<]*)(</camera_pose>)'
    matches=list(re.finditer(pattern,config,re.S))
    if len(matches)!=1:raise ValueError('Expected exactly one MinimalScene camera_pose')
    m=matches[0];pose=' '.join(format(v,'.17g') for v in xyz+[roll,pitch,yaw])
    result=config[:m.start(2)]+pose+config[m.end(2):]
    with Path(destination).open('x') as out:out.write(result)
    return dict(position=xyz,rpy=[roll,pitch,yaw],config_sha256=media.digest(destination),
                saved_inputs={str(p):media.digest(p) for p in [views/'gazebo_gui.config',views/'gazebo_camera_pose_final.txt']})

def screenshot_content(binary,path):
    result=subprocess.run([binary,'-nostdin','-v','error','-i',str(path),'-frames:v','1',
                           '-vf','scale=96:64','-pix_fmt','rgb24','-f','rawvideo','pipe:1'],
                          capture_output=True,check=True,timeout=10)
    pixels=media.np.frombuffer(result.stdout,media.np.uint8)
    if len(pixels)!=96*64*3 or pixels.mean()<2 or pixels.std()<2:
        raise RuntimeError('Screenshot is black, empty or undecodable: '+str(path))
    return dict(path=str(path),sha256=media.digest(path),mean=float(pixels.mean()),std=float(pixels.std()))

def optional_camera_feedback(env,owner,deadline):
    listener=None;buffer=''
    try:
        listener=subprocess.Popen(['ign','topic','-e','-t','/gui/camera/pose','--json-output'],
                                  env=env,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,bufsize=0,start_new_session=True)
        end=min(deadline,time.monotonic()+1.)
        while time.monotonic()<end and owner.poll() is None and listener.poll() is None:
            ready,_,_=select.select([listener.stdout],[],[],max(0.,end-time.monotonic()))
            if not ready:break
            chunk=os.read(listener.stdout.fileno(),65536)
            if not chunk:break
            buffer+=chunk.decode(errors='replace');poses,buffer=extract_json_messages(buffer)
            for pose in poses:
                if camera_pose_valid(pose):return dict(available=True,topic='/gui/camera/pose',receipt_wall=time.monotonic(),pose=pose)
        return dict(available=False,reason='no valid camera Pose received in diagnostic window')
    except (OSError,ValueError) as error:
        return dict(available=False,reason=repr(error))
    finally:
        if listener is not None and listener.poll() is None:
            listener.terminate()
            try:listener.wait(timeout=2)
            except subprocess.TimeoutExpired:listener.kill();listener.wait(timeout=2)

def pose_matches(pose,position,quaternion,tolerance=5e-5):
    try:
        xyz=[float(pose['position'][k]) for k in 'xyz'];q=[float(pose['orientation'][k]) for k in 'xyzw']
        return (max(abs(a-b) for a,b in zip(xyz,position))<=tolerance and
                min(max(abs(a-b) for a,b in zip(q,quaternion)),max(abs(a+b) for a,b in zip(q,quaternion)))<=tolerance)
    except (KeyError,TypeError,ValueError):return False

def camera_feedback_stable(samples,target):
    if len(samples)<3:return False
    recent=[]
    for sample in reversed(samples):
        if not pose_matches(sample,target['position'],target['orientation']):break
        recent.append(sample)
    return len(recent)>=3 and recent[0]['receipt_wall']-recent[-1]['receipt_wall']>=.15

def extract_json_messages(buffer):
    decoder=json.JSONDecoder();rows=[];offset=0
    while True:
        start=buffer.find('{',offset)
        if start<0:return rows,''
        try:row,end=decoder.raw_decode(buffer,start)
        except json.JSONDecodeError:
            return rows,buffer[start:]
        rows.append(row);offset=end

def owner_rest_is_qualified(report):
    if not isinstance(report,dict):return False
    from warehouse_ab_supervisor import rest_proof
    return (rest_proof(report.get('normal_rest')) and
            (report.get('normal_cleanup') or {}).get('passed') is True and
            (report.get('normal_cleanup') or {}).get('remaining')=={} and
            report.get('watchdog')==dict(reason='normal_shutdown',emergency=False))

def assert_window_viewable(window,display,env):
    try:
        info=media.run(['xwininfo','-display',display,'-id',hex(window['id'])],env=env,timeout=2).stdout
    except Exception as error:raise CameraSetupFailure('gazebo_window_query_failed',repr(error))
    state=re.search(r'^\s*Map State:\s*(.+)$',info,re.M)
    width=re.search(r'^\s*Width:\s*(\d+)',info,re.M);height=re.search(r'^\s*Height:\s*(\d+)',info,re.M)
    if not state or state[1].strip()!='IsViewable' or not width or not height or min(int(width[1]),int(height[1]))<200:
        raise CameraSetupFailure('gazebo_window_not_viewable',info)
    return dict(map_state=state[1].strip(),width=int(width[1]),height=int(height[1]))

def wait_for(test,deadline,owner=None):
    while time.monotonic()<deadline:
        if owner is not None and owner.poll() is not None:raise RuntimeError('Owner exited before presentation readiness')
        try:
            result=test()
            if result:return result
        except (FileNotFoundError,json.JSONDecodeError):pass
        time.sleep(.1)
    raise TimeoutError('Presentation readiness deadline')

def restore_manual_camera(directory,env,views=MANUAL_VIEWS,*,window,owner,session,deadline,display):
    pose_path=views/'gazebo_camera_pose_final.txt';gui_path=views/'gazebo_gui.config'
    raw=pose_path.read_text();before={str(p):media.digest(p) for p in [pose_path,gui_path]}
    def section(name,axes):
        match=re.search(name+r'\s*\{([^}]+)\}',raw)
        if not match:raise ValueError('Saved camera lacks '+name)
        data={k:float(v) for k,v in re.findall(r'([xyzw]):\s*([-+0-9.eE]+)',match[1])}
        values=[data[k] for k in axes]
        if not all(math.isfinite(v) for v in values):raise ValueError('Invalid saved camera')
        return values
    position=section('position','xyz');quaternion=section('orientation','xyzw')
    if abs(sum(v*v for v in quaternion)-1)>1e-4:raise ValueError('Saved camera quaternion is not unit length')
    request='pose {\n'+raw+'\n}' # EXACT saved world pose; never fit, transform, or rewrite it.
    window_state=assert_window_viewable(window,display,env)
    gui_log=session/'warehouse-gui.log'
    gui_text=gui_log.read_text(errors='replace') if gui_log.is_file() else ''
    listener=subprocess.Popen(['ign','topic','-e','-t','/gui/camera/pose','--json-output'],
                              env=env,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,bufsize=0,start_new_session=True)
    transport_info=dict(topic='/gui/camera/pose',message_type='ignition.msgs.Pose',
                        listener_command=['ign','topic','-e','-t','/gui/camera/pose','--json-output'])
    service_info='';samples=[];buffer='';accepted=False
    try:
        # The long-lived typed listener is the data-path admission. `ign topic
        # -i` is deliberately excluded: the installed Fortress CLI can stall
        # even while CameraTracking is publishing on this topic.
        while time.monotonic()<deadline and owner.poll() is None:
            if listener.poll() is not None:
                raise CameraSetupFailure('camera_feedback_process_exited',listener.returncode)
            ready,_,_=select.select([listener.stdout],[],[],min(.1,max(0.,deadline-time.monotonic())))
            if not ready:continue
            chunk=os.read(listener.stdout.fileno(),65536)
            if not chunk:raise CameraSetupFailure('camera_feedback_stream_closed')
            buffer+=chunk.decode(errors='replace')
            poses,buffer=extract_json_messages(buffer)
            now=time.monotonic()
            samples.extend(dict(receipt_wall=now,position=pose.get('position',{}),
                                orientation=pose.get('orientation',{}),header=pose.get('header'))
                           for pose in poses if camera_pose_valid(pose))
            if samples:break
        else:
            reason='camera_pose_not_received_before_restore' if owner.poll() is None else 'owner_exited_before_camera_feedback'
            raise CameraSetupFailure(reason,transport_info)
        if not samples:raise CameraSetupFailure('camera_pose_not_received_before_restore',transport_info)
        service_info=media.run(['ign','service','-i','-s','/gui/move_to/pose'],env=env,timeout=2).stdout
        if 'GUICamera' not in service_info or 'Boolean' not in service_info:
            raise CameraSetupFailure('camera_restore_service_type_mismatch',service_info)
        reply=media.run(['ign','service','-s','/gui/move_to/pose','--reqtype','ignition.msgs.GUICamera','--reptype','ignition.msgs.Boolean','--timeout','3000','--req',request],env=env).stdout
        if 'data: true' not in reply:raise CameraSetupFailure('camera_restore_rpc_rejected',reply)
        accepted=True
        # Do not count queued pre-request poses as completion feedback.
        while select.select([listener.stdout],[],[],0)[0]:
            if not os.read(listener.stdout.fileno(),65536):break
        samples=[];buffer=''
        while time.monotonic()<deadline:
            if owner.poll() is not None:raise CameraSetupFailure('owner_exited_during_camera_animation')
            if listener.poll() is not None:raise CameraSetupFailure('camera_feedback_process_exited',listener.returncode)
            ready,_,_=select.select([listener.stdout],[],[],.1)
            if not ready:continue
            chunk=os.read(listener.stdout.fileno(),65536)
            if not chunk:raise CameraSetupFailure('camera_feedback_stream_closed')
            buffer+=chunk.decode(errors='replace')
            poses,buffer=extract_json_messages(buffer)
            for pose in poses:
                if not camera_pose_valid(pose):continue
                sample=dict(receipt_wall=time.monotonic(),position=pose['position'],
                            orientation=pose['orientation'],header=pose.get('header'))
                # Fortress CameraTracking publishes at 50 Hz while connected;
                # retain enough real frames to prove a 150 ms settled interval.
                samples.append(sample);samples=samples[-32:]
                if camera_feedback_stable(samples,dict(position=position,orientation=quaternion)):
                    break
            if camera_feedback_stable(samples,dict(position=position,orientation=quaternion)):break
        else:raise CameraSetupFailure('camera_target_not_stable_before_owner_deadline',dict(samples=len(samples),last=samples[-1] if samples else None,transport_info=transport_info))
    finally:
        if listener.poll() is None:
            listener.terminate()
            try:listener.wait(timeout=2)
            except subprocess.TimeoutExpired:listener.kill();listener.wait(timeout=2)
    if not accepted or len(samples)<3:raise CameraSetupFailure('camera_completion_unqualified')
    if any(media.digest(p)!=h for p,h in before.items()):raise RuntimeError('Original saved camera files changed')
    result=dict(manual_camera_restored=True,saved_inputs=before,requested_position=position,requested_quaternion=quaternion,
                actual_pose=dict(position=samples[-1]['position'],orientation=samples[-1]['orientation']),
                feedback_samples=samples[-3:],transport_info=transport_info,service_info=service_info,
                feedback_received=True,feedback_sample_count=len(samples),listener_pid=listener.pid,
                topic_identity=dict(name='/gui/camera/pose',message_type='ignition.msgs.Pose',window_id=window['id'],window_pid=window['pid']),
                gui_camera_pose_advertised=gui_advertises_camera_pose(gui_text),
                window=window_state,visibility_verified=False,colors=[],
                camera_policy='exact user-saved WORLD pose; received typed camera Pose feedback; three stable measured target poses; no angle search/refit')
    media.write(Path(directory)/'manual-camera-restore.json',result);return result

def finish_owner_after_camera_failure(owner,owner_started,session,token,children):
    """Leave zero-only cleanup to the qualified owner/watchdog path; never SIGINT it here."""
    from narrow_processes import OwnedProcesses
    end=owner_started+160. # Existing owner/watchdog limit is150s;10s for cleanup evidence.
    while owner.poll() is None and time.monotonic()<end:time.sleep(.1)
    if owner.poll() is None:
        return dict(owner_exited=False,owner_pending=True,watchdog_pending=True,remaining=None,
                    reason='owner_did_not_finish_inside_existing_watchdog_bound')
    path=session/'session-report.json'
    report=json.loads(path.read_text()) if path.is_file() else None
    live=OwnedProcesses(token).scan()
    for child in children:
        if child is owner:continue
        try:child.wait(timeout=2)
        except subprocess.TimeoutExpired:pass
    if not live:
        from course_sim.session import SessionLease
        lease=SessionLease(int(json.loads((session/'preflight.json').read_text())['domain']));lease.close()
    return dict(owner_exited=True,owner_returncode=owner.returncode,owner_report_data=report,
                normal_rest=(report or {}).get('normal_rest'),normal_cleanup=(report or {}).get('normal_cleanup'),
                watchdog=(report or {}).get('watchdog'),remaining=live,
                physical_rest_qualified=owner_rest_is_qualified(report) and not live)

def perform_attempt(index,base,cfg,deadline,prepare=False,setup_seconds=0,record_only=False):
    from g1_g5_presentation import rviz_config
    from narrow_processes import OwnedProcesses
    directory=base/f'attempt_{index:02d}';directory.mkdir();owner=None;children=[];logs=[];session=None;token=None;owner_started=None
    try:
        if not prepare:
            if media.digest(cfg['_review_path'])!=cfg['_review_sha256']:raise RuntimeError('Review changed during batch')
            read_review(Path(cfg['_review_path']))
        media.write(directory/'host.json',host_admission());domain=fresh_domain();env=ros_env(domain);binary=media.ffmpeg();display=env.get('DISPLAY')
        if not display:raise RuntimeError('No X11/XWayland display; native Wayland windows cannot be captured with existing tool')
        for name in ['gazebo_gui.config','gazebo_camera_pose_final.txt','g1_g5.rviz']:
            if not (MANUAL_VIEWS/name).is_file():raise RuntimeError('Saved manual view missing: '+name)
        gui_copy=directory/'gazebo.saved.gui.config'
        startup=startup_camera_config(MANUAL_VIEWS,gui_copy) if record_only else None
        if not record_only:gui_copy.write_bytes((MANUAL_VIEWS/'gazebo_gui.config').read_bytes())
        deployment=prepare_manifest(directory) if prepare else cfg['deployment_manifest']
        from warehouse_ab_supervisor import identity
        caller=identity(os.getpid())
        args=['/usr/bin/python3',str(ROOT/'scripts/warehouse_ab_trial.py'),'--profile','B','--recording-parent-pid',str(caller['pid']),'--recording-parent-start-ticks',str(caller['start_ticks']),'--gui','--gui-config',str(gui_copy),'--recording-end-hold-s','5' if prepare else '3','--deployment-manifest',deployment['path'],'--output-root',str(directory/'owner')]
        manifest=json.loads(Path(deployment['path']).read_text())
        if 'controller_revision' in manifest:args+=['--controller-revision-sha256',deployment['sha256']]
        if prepare:args+=['--stationary-containment','--stationary-normal-cleanup','--stationary-view-only','--stationary-view-s',str(setup_seconds or 15),'--precheck-spawn-x','0','--precheck-spawn-y','0']
        else:args+=['--route-mode','full','--authorize-motion','route-full','--continuation-yaws',*map(str,cfg['continuation_yaws']),'--route-envelope',cfg['route_envelope']['path'],'--route-envelope-sha256',cfg['route_envelope']['sha256'],'--video-ready-file',str(directory/'dual.ready.json')]
        media.run(args+['--check'],env=env,timeout=20)
        def launch(argv,name,child_env=env):
            stream=(directory/(name+'.log')).open('x');logs.append(stream);p=subprocess.Popen(list(map(str,argv)),cwd=ROOT,env=child_env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True);children.append(p);return p
        owner_started=time.monotonic();owner=launch(args+['--execute'],'owner');session=wait_for(lambda:next(iter((directory/'owner').glob('*-*/watchdog.json')),None),min(deadline,time.monotonic()+12),owner).parent
        token=json.loads((session/'watchdog.json').read_text())['token']
        pin=wait_for(lambda:json.loads((session/'server-pin.json').read_text()),min(deadline,time.monotonic()+20),owner)
        from warehouse_ab_supervisor import same_process
        if not same_process(pin,token):raise RuntimeError('Verified server identity exited before GUI attach')
        server_env=dict(x.split('=',1) for x in Path(f"/proc/{pin['pid']}/environ").read_bytes().decode().split('\0') if '=' in x)
        partition=server_env.get('IGN_PARTITION')
        if not partition:raise RuntimeError('Verified server has no private Ignition partition')
        env=dict(env,B2W_NARROW_OWNER=token,IGN_PARTITION=partition)
        route=rviz_config(directory/'radar.rviz',source=MANUAL_VIEWS/'g1_g5.rviz');media.write(directory/'route.json',route)
        launch(['/opt/ros/humble/lib/rviz2/rviz2','-d',directory/'radar.rviz','--ros-args','-p','use_sim_time:=true'],'radar',env)
        launch(['/usr/bin/python3',ROOT/'scripts/g1_g5_presentation.py','--output',directory],'presentation',env)
        seen_windows=None;seen_since=0.
        def windows_ready():
            nonlocal seen_windows,seen_since
            try:found=media.owned_windows(token,display)
            except RuntimeError:return None
            signature=tuple((k,v['id'],v['pid']) for k,v in sorted(found.items()))
            if signature!=seen_windows:seen_windows=signature;seen_since=time.monotonic();return None
            return found if time.monotonic()-seen_since>=1. else None
        windows=wait_for(windows_ready,min(deadline,time.monotonic()+20),owner);screen=arrange_for_mode(windows,display,prepare,env);media.write(directory/'windows.json',dict(windows=windows,screen=screen,display=display,token=token))
        # Reuse the existing15s camera-service admission bound. Do not burn
        # the owner's150s watchdog budget waiting on a GUI that is not ready.
        camera_deadline=min(deadline,time.monotonic()+15.,owner_started+140.)
        try:
            if record_only:
                assert_window_viewable(windows['warehouse'],display,env)
                restored=dict(startup_config=startup, saved_inputs=startup['saved_inputs'],
                              manual_camera_restored=False,visibility_verified=False,colors=[],
                              camera_feedback=optional_camera_feedback(env,owner,camera_deadline))
            else:
                wait_for(lambda:'/gui/move_to/pose' in media.run(['ign','service','-l'],env=env,timeout=3).stdout,camera_deadline,owner)
                restored=restore_manual_camera(directory,env,window=windows['warehouse'],owner=owner,
                                               session=session,deadline=camera_deadline,display=display)
        except Exception as error:
            failure=(error if isinstance(error,CameraSetupFailure) else
                     CameraSetupFailure('camera_gui_readiness_failed',repr(error)))
            media.write(directory/'camera-restore-failure.json',dict(reason=failure.reason,detail=failure.detail,
                        at_wall=time.monotonic(),gui_log=str(session/'warehouse-gui.log'),
                        gui_camera_pose_advertised=gui_advertises_camera_pose(
                            (session/'warehouse-gui.log').read_text(errors='replace') if (session/'warehouse-gui.log').is_file() else ''),
                        windows=windows))
            containment=finish_owner_after_camera_failure(owner,owner_started,session,token,children)
            row=dict(success=False,retry=False,reason='camera_setup_failed',error=str(failure),
                     camera_diagnostics=str(directory/'camera-restore-failure.json'),directory=str(directory),
                     owner_report=str(session/'session-report.json'),**containment)
            media.write(directory/'attempt_report.json',row);return row
        wait_for(lambda:(directory/'correspondence.json').is_file(),min(deadline,time.monotonic()+45),owner)
        view=dict(json.loads((directory/'correspondence.json').read_text()),**restored)
        screenshots={}
        for kind in windows:
            media.screenshot(windows[kind]['id'],display,directory/(kind+'.png'))
            screenshots[kind]=screenshot_content(binary,directory/(kind+'.png'))
        view['screenshots']=screenshots
        if not prepare:
            approved=cfg.get('camera_review',{})
            if approved.get('route_visibility_accepted') is not True or approved.get('saved_inputs')!=view['saved_inputs']:raise RuntimeError('Exact saved camera restored; route-wide visibility review still required')
            view['visibility_verified']=True
        media.write(directory/'framing.json',view)
        if setup_seconds:
            media.write(directory/'setup_ready.json',dict(stationary=True,no_navigation=True,map_correspondence=True,rviz_save_as=str(MANUAL_VIEWS/'g1_g5.rviz'),view_seconds=setup_seconds,camera_inputs=view['saved_inputs']))
            print('STATIONARY VIEWS READY — RViz File → Save Config As → '+str(MANUAL_VIEWS/'g1_g5.rviz')+'; bounded automatic stop, no navigation.',flush=True)
        recorders={}
        for kind in ['radar','warehouse']:
            recorders[kind]=launch(['/usr/bin/python3',ROOT/'scripts/record_warehouse_video.py','--execute','--fps','25','--duration','600','--window-id',str(windows[kind]['id']),'--owned-window-pid',str(windows[kind]['pid']),'--display',display,'--output',directory/(kind+'.webm')],kind+'-capture',env)
        ledgers={k:wait_for(lambda k=k:json.loads((directory/(k+'.ready.json')).read_text()),min(deadline,time.monotonic()+12),owner) for k in recorders}
        time.sleep(3.) # Both captures ready before opening the goal gate.
        combined=dict(ledgers['warehouse'],views=ledgers,framing_file=str(directory/'framing.json'),framing_sha256=media.digest(directory/'framing.json'),dual_view=True)
        media.write(directory/'dual.ready.json',combined)
        last_gpu_check=0.
        while owner.poll() is None:
            if time.monotonic()-last_gpu_check>2.:
                gpu=media.run(['nvidia-smi','--query-compute-apps=pid','--format=csv,noheader']).stdout
                for pid in gpu.split():
                    try:owned=('B2W_NARROW_OWNER='+token).encode() in Path('/proc',pid,'environ').read_bytes().split(b'\0')
                    except FileNotFoundError:continue
                    if not owned:raise RuntimeError('GPU isolation lost to another process; no job will be modified')
                last_gpu_check=time.monotonic()
            if time.monotonic()>=deadline:raise TimeoutError('Batch total deadline')
            hb=json.loads((session/'heartbeat.json').read_text())
            if hb.get('phase')!='shutdown' and any(p.poll() is not None for p in recorders.values()):raise RuntimeError('A recorder failed before qualified shutdown')
            time.sleep(.1)
        r=json.loads((session/'session-report.json').read_text());r['_session_directory']=str(session);live=OwnedProcesses(token).scan()
        if live:raise RuntimeError('Post-owner token-owned processes remain')
        from course_sim.session import SessionLease
        reclaimed=SessionLease(domain);reclaimed.close()
        if any(media.digest(p)!=h for p,h in restored['saved_inputs'].items()):raise RuntimeError('Original saved camera inputs were modified')
        if r.get('token')!=token:raise RuntimeError('Owner report belongs to a different attempt')
        media.write(directory/'postflight.json',dict(remaining=live,normal_cleanup=r.get('normal_cleanup'),watchdog=r.get('watchdog')))
        for p in children:
            try:p.wait(timeout=3)
            except subprocess.TimeoutExpired:raise RuntimeError('Presentation child failed cleanup')
        if prepare:
            if (not r.get('success') or r.get('stationary_commands',{}).get('nonzero')!=0 or
                not r.get('stationary_commands',{}).get('samples') or not owner_rest_is_qualified(r)):
                raise RuntimeError('Zero-only preparation did not qualify')
            starts={k:v['started_wall'] for k,v in ledgers.items()};end=min(starts[k]+media.inspect_video(binary,directory/(k+'.webm'),deadline=deadline)['duration_s'] for k in starts)
            for cmd in media.export_commands(binary,directory,starts,max(starts.values()),end,prefix='prepare'):media.run(cmd,timeout=120)
            videos=[media.inspect_video(binary,directory/('prepare_'+k+'.mp4'),deadline=deadline) for k in ['radar','warehouse','combined']]
            if max(v['duration_s'] for v in videos)-min(v['duration_s'] for v in videos)>.15:raise RuntimeError('Export duration/synchronization failure')
            result=dict(success=False,prepared=not record_only,retry=False,reason='visual_framing_review_required' if record_only else 'stationary_manual_views_recording_check_passed',directory=str(directory),domain=domain,video=videos,owner_report=str(session/'session-report.json'))
        else:
            outcome=classify(r)
            if not r.get('success'):
                result={**outcome,'directory':str(directory),'owner_report':str(session/'session-report.json'),
                        'route':r.get('route'),'watchdog':r.get('watchdog'),'normal_cleanup':r.get('normal_cleanup'),
                        'contact_failure':r.get('contact_failure'),'readiness_first_fault':r.get('readiness_first_fault')}
                if outcome['retry']:
                    result['waypoints']=verified_arrival_prefix(r,view)
                    starts={k:v['started_wall'] for k,v in ledgers.items()};begin=max(starts.values())
                    end=r['recording_end_hold']['end_wall']
                    raw={k:media.inspect_video(binary,directory/(k+'.webm'),deadline=deadline) for k in starts}
                    if end<=begin or any(starts[k]+raw[k]['duration_s']<end-.5 for k in starts):
                        raise RuntimeError('Ordinary failure lacks complete attempt recording')
                    for cmd in media.export_commands(binary,directory,starts,begin,end,prefix='partial'):
                        media.run(cmd,timeout=max(1,min(120,deadline-time.monotonic())))
                    result['videos']=[media.inspect_video(binary,directory/('partial_'+k+'.mp4'),deadline=deadline)
                                      for k in ['radar','warehouse','combined']]
                    result['raw_videos']=raw;result['video_complete']=False;result['partial_video']=True
                media.write(directory/'attempt_report.json',result);return result
            if outcome['reason']!='requires_physical_and_video_verification':raise RuntimeError('Route success did not pass safety/rest/cleanup verification')
            verified=verify_route(r,view)
            if not verified['passed']:raise RuntimeError(verified['reason'])
            starts={k:v['started_wall'] for k,v in ledgers.items()};begin=max(starts.values());end=r['recording_end_hold']['end_wall'];first=r['recording_journey_start_wall']
            if first-begin<3:raise RuntimeError('Continuous dual pre-roll absent')
            motion=[(max(a,first),min(b,end)) for a,b in measured_motion_intervals(session) if b>first and a<end]
            if not any(b-a>=1 for a,b in motion):raise RuntimeError('Successful route lacks measured body-motion intervals')
            raw={k:media.inspect_video(binary,directory/(k+'.webm'),[(a-starts[k],b-starts[k]) for a,b in motion],deadline=deadline,required_colors=view['colors'] if k=='warehouse' else ()) for k in starts}
            if any(starts[k]+v['duration_s']<end-.5 for k,v in raw.items()):raise RuntimeError('Capture does not cover whole route/rest/end hold')
            for cmd in media.export_commands(binary,directory,starts,begin,end,prefix='pending'):media.run(cmd,timeout=max(1,min(120,deadline-time.monotonic())))
            videos=[media.inspect_video(binary,directory/('pending_'+k+'.mp4'),[(a-begin,b-begin) for a,b in motion],deadline=deadline,required_colors=view['colors'] if k=='warehouse' else ()) for k in ['radar','warehouse','combined']]
            if max(v['duration_s'] for v in videos)-min(v['duration_s'] for v in videos)>.15 or min(v['duration_s'] for v in videos)<end-begin-.5:raise RuntimeError('Export duration/synchronization failure')
            for v in videos:
                old=Path(v['path']);new=old.with_name(old.name.replace('pending_','success_',1));old.rename(new);v['path']=str(new)
            result=dict(success=True,retry=False,reason='verified_success',directory=str(directory),domain=domain,waypoints=verified['waypoints'],videos=videos,owner_report=str(session/'session-report.json'),raw_videos=raw,framing=view,continuous_same_attempt=True,
                        route=r.get('route'),normal_rest=r.get('normal_rest'),normal_cleanup=r.get('normal_cleanup'),
                        watchdog=r.get('watchdog'),contact_failure=r.get('contact_failure'),readiness_first_fault=r.get('readiness_first_fault'))
            media.write(directory/'success_report.json',result)
        media.write(directory/'attempt_report.json',result);return result
    except BaseException as error:
        if record_only and owner is not None and owner.poll() is None and session is not None and token:
            containment=finish_owner_after_camera_failure(owner,owner_started,session,token,children)
            result=dict(success=False,retry=False,reason='hard_stop',error=repr(error),directory=str(directory),**containment)
            media.write(directory/'attempt_report.json',result);return result
        if owner is not None and owner.poll() is None:
            owner.send_signal(signal.SIGINT)
            try:owner.wait(timeout=20)
            except subprocess.TimeoutExpired:pass # independent existing watchdog remains authoritative
        remaining=None
        if token:
            remaining=OwnedProcesses(token).scan()
            # Cleanup is exact-token only; never touches unrelated GUI/GPU jobs.
            if remaining:
                for p in children:
                    if p is not owner and p.poll() is None:p.send_signal(signal.SIGINT)
        if token and remaining:
            end=time.monotonic()+8.
            while time.monotonic()<end and OwnedProcesses(token).scan():time.sleep(.1)
            remaining=OwnedProcesses(token).scan()
        result=dict(success=False,retry=False,reason='hard_stop',error=repr(error),directory=str(directory),remaining=remaining)
        media.write(directory/'attempt_report.json',result);return result
    finally:
        for stream in logs:stream.close()

def main(argv=None):
    p=parser();args=p.parse_args(argv)
    if args.run or args.stationary_setup or args.record_only or args.prepare_only or args.collect_attempts:
        print('The automatic video workflow is retired. Run scripts/run_warehouse_navigation.py --run; screen recording is manual.', file=sys.stderr)
        return 2
    if args.collect_attempts and not args.run:p.error('--collect-attempts requires --run')
    args.output_root.mkdir(parents=True,exist_ok=True)
    collect=bool(args.collect_attempts)
    output=args.output_root/(f'G1_G5_batch_{args.collect_attempts}' if collect else datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8]);output.mkdir()
    if collect:
        for category in ['success','failed','blocked']:(output/category).mkdir()
    mode=next(name for name,enabled in [('collect',collect),('run',args.run),('stationary_setup',args.stationary_setup),('record_only',args.record_only),('prepare',args.prepare_only),('dry_run',True)] if enabled)
    report=dict(success=False,attempts=0,mode=mode,output=str(output),max_attempts=args.collect_attempts or args.max_attempts,max_total_minutes=args.max_total_minutes)
    prepare=args.prepare_only or args.record_only or args.stationary_setup
    try:
        cfg=read_review(args.review_config) if args.run else None
        if not args.run and not prepare:
            from run_warehouse_route import load_route
            report.update(stop_reason='dry_run_no_launch',canonical_route=load_route(),route_blocker='No accepted retained-heading/Profile-B route review/envelope currently available',requirements=['reviewed_run.json + externally reviewed SHA256 sidecar','accepted empirical controller review and deployment-matching envelope','G3-G5 arrival headings','independent source review before motion','fresh GPU/domain/map/TF/contact/collision/watchdog/cleanup'])
        else:
            lock=ROOT/'course_results'/f'.b2w-g1-g5-batch-{os.getuid()}.lock'
            with lock.open('a+') as lease:
                fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
                # Import installed course helpers under system ROS without changing caller's shell.
                env=ros_env(0)
                for path in env.get('PYTHONPATH','').split(':'):
                    if path and path not in sys.path:sys.path.append(path)
                deadline=time.monotonic()+args.max_total_minutes*60-20. # reserve owned containment/cleanup time
                if prepare:
                    row=perform_attempt(1,output,None,deadline,True,args.stationary_seconds if args.stationary_setup else 60,record_only=args.record_only);report.update(attempts=1,preparation=row,stop_reason=row['reason'])
                else:
                    if collect:
                        def collected_attempt(index):
                            started=time.monotonic();row=perform_attempt(index,output,cfg,deadline)
                            row['elapsed_wall_s']=time.monotonic()-started
                            try:project_attempt(output,index,row)
                            except Exception as error:
                                row.update(success=False,retry=False,reason='evidence_projection_failed',error=repr(error))
                                blocked=output/'blocked'/f'attempt_{index:02d}';blocked.mkdir(exist_ok=True)
                                media.write(blocked/'report.json',row)
                            return row
                        result=run_collect_batch(collected_attempt,args.collect_attempts,max(0,args.max_total_minutes*60-20.))
                        report.update(result,requested=args.collect_attempts,
                                      success=result['completed'] and result['successful']>0)
                    else:
                        result=run_batch(lambda i:perform_attempt(i,output,cfg,deadline),args.max_attempts,max(0,args.max_total_minutes*60-20.));report.update(result)
                    if report['success'] and not collect:
                        source=Path(result['results'][-1]['directory'])
                        for name in ['success_radar.mp4','success_warehouse.mp4','success_combined.mp4','success_report.json']:os.link(source/name,output/name)
    except (Exception,KeyboardInterrupt) as error:
        report.update(success=False,stop_reason='blocked_prerequisites',error=repr(error))
        if collect:
            admission=output/'blocked'/'admission';admission.mkdir()
            media.write(admission/'report.json',dict(attempted=report.get('attempts',0),reason=report['stop_reason'],error=report['error']))
    if collect:write_collect_summary(output,dict(report,requested=args.collect_attempts))
    media.write(output/'results_report.json',report);print(json.dumps(report,indent=2))
    return 0 if report['success'] or (not args.run and (not prepare or report.get('preparation',{}).get('prepared'))) else 2
if __name__=='__main__':raise SystemExit(main())
