#!/usr/bin/env bash
# One owned, bounded zero-only Warehouse capture. Never issues a motion lease.
set -eo pipefail
course_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$course_root"
source /opt/ros/humble/setup.bash
source .deps/nav2-controller-1.1.20/install/setup.bash
source install/local_setup.bash
export PYTHONPATH="$course_root/scripts:$course_root/course_navigation:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$course_root/.deps/lib:$course_root/b2w_sim/b2w_controllers/third_party/onnxruntime/lib64:${LD_LIBRARY_PATH:-}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 ROS_LOCALHOST_ONLY=1
exec /usr/bin/python3 - "$@" <<'PY'
import hashlib,json,os,signal,subprocess,sys,time,uuid
import xml.etree.ElementTree as ET
from pathlib import Path
import run_g1_g5_record as batch
from stage4_session import validate_inputs

root=Path.cwd()
if len(sys.argv)>2:raise SystemExit('Usage: capture_warehouse_registration.sh [new-output-directory]')
out=(Path(sys.argv[1]) if len(sys.argv)==2 else root/'course_results'/('warehouse-registration-'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())+'-'+uuid.uuid4().hex[:8])).resolve()
if not out.is_relative_to(root/'course_results'):raise SystemExit('Output must be inside repository course_results/')
host=batch.host_admission();domain=batch.fresh_domain()
out.mkdir(parents=True,exist_ok=False)
env=dict(os.environ,ROS_DOMAIN_ID=str(domain),ROS_LOG_DIR=str(out/'ros-log'))
os.environ.update(ROS_DOMAIN_ID=str(domain),ROS_LOG_DIR=str(out/'ros-log'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
source=root/'third_party/aws_warehouse/warehouse.world'
tree=ET.parse(source);world=tree.getroot().find('world');before=ET.tostring(world)
plugin=ET.SubElement(world,'plugin',filename=str(root/'artifacts/recovery/contact-build/libb2w_contact_observer.so'),name='b2w_evaluation::Observer')
tree.write(out/'observer.world',encoding='unicode');world.remove(plugin)
assert ET.tostring(world)==before,'World geometry changed'
inventory=json.loads((root/'artifacts/recovery/collision-inventory.json').read_text())
inventory['converted_sdf']=str(root/'artifacts/recovery/robot.sdf')
assert sha(inventory['converted_sdf'])==inventory['converted_sdf_sha256']
inv=out/'collision-inventory.json';inv.write_text(json.dumps(inventory,indent=2))
bundle=root/'third_party/b2w_rl_sar'
value=dict(kind='stage4_zero_session_inputs',version=2,world=str(out/'observer.world'),world_sha256=sha(out/'observer.world'),pretrained_directory=str(bundle),policy_source_sha256=sha(bundle/'POLICY_SOURCE.json'),guard=dict(session_id=uuid.uuid4().hex,collision_inventory_path=str(inv),collision_inventory_sha256=sha(inv)))
inputs=out/'inputs.json';inputs.write_text(json.dumps(value));inputs.chmod(0o600)
validate_inputs(inputs) # Check exact installed sources and reviewed Nav2 native before launch.
(out/'host.json').write_text(json.dumps(dict(host=host,domain=domain),indent=2))
owner=collector=None;logs=[];code=None
try:
    for name in ('presentation','owner'):logs.append((out/(name+'.log')).open('x'))
    collector=subprocess.Popen(['/usr/bin/python3','scripts/g1_g5_presentation.py','--output',str(out)],env=env,stdout=logs[0],stderr=subprocess.STDOUT)
    owner=subprocess.Popen(['/usr/bin/python3','scripts/stage4_session.py','--inputs',str(inputs),'--output',str(out/'session'),'--execute-zero-smoke'],env=env,stdout=logs[1],stderr=subprocess.STDOUT)
    print('Zero-only host capture: '+str(out),flush=True)
    try:code=owner.wait(timeout=260)
    except (subprocess.TimeoutExpired,KeyboardInterrupt):
        owner.send_signal(signal.SIGINT)
        try:code=owner.wait(timeout=30)
        except subprocess.TimeoutExpired:code=None # Existing owner/watchdog retains cleanup authority.
finally:
    if collector is not None and collector.poll() is None:
        collector.send_signal(signal.SIGINT)
        try:collector.wait(timeout=10)
        except subprocess.TimeoutExpired:collector.terminate();collector.wait(timeout=5)
    for log in logs:log.close()
result=dict(owner_returncode=code,owner_pending=owner is not None and owner.poll() is None,motion_authorized=False,saved_map_registered=False,correspondence_available=(out/'correspondence.json').is_file(),output=str(out.relative_to(root)))
(out/'host-execution.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
raise SystemExit(0 if code==0 and result['correspondence_available'] else 1)
PY
