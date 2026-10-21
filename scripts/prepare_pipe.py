#!/usr/bin/python3
"""Generate/check the local pipe without ROS, Fuel, downloads or installed robot assets."""
import argparse
import hashlib
import json
import os
import sysconfig
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'course_sim'))
from course_sim.pipe_environment import PipeConfig, make_pipe_world, clearance


def resolve_plugin(name):
    """Fortress accepts versionless aliases; inspect local multiarch and configured paths."""
    directories=[Path(p) for p in os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH','').split(os.pathsep) if p]
    directories += [Path(p) for p in os.environ.get('LD_LIBRARY_PATH','').split(os.pathsep) if p]
    multiarch=sysconfig.get_config_var('MULTIARCH')
    directories += [Path('/usr/lib')/multiarch] if multiarch else []
    directories += [Path('/usr/local/lib'),Path('/usr/lib')]
    for directory in directories:
        for filename in ['lib'+name+'.so','lib'+name.replace('ignition-gazebo-', 'ignition-gazebo6-')+'.so']:
            path=directory/filename
            if path.is_file():return str(path)
    return None


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'course_sim/config/pipe_v0.yaml')
    p.add_argument('--world',type=Path,default=ROOT/'course_sim/worlds/pipe_v0.world')
    p.add_argument('--check',action='store_true',help='read-only reproducibility, resource, SDF and collision checks')
    p.add_argument('--output',type=Path,help='write JSON validation/manifest evidence')
    args=p.parse_args()
    cfg=PipeConfig(**yaml.safe_load(args.config.read_text()))
    text=make_pipe_world(cfg)
    result=cfg.manifest()
    result['world_path']=str(args.world.resolve())
    result['world_sha256']=hashlib.sha256(text.encode()).hexdigest()
    result['collision_clearance']=clearance(cfg,ROOT/'b2w_sim/b2w_description_ros2/urdf/b2w_description_rsl.urdf')
    result['passed']=result['collision_clearance']['passed']
    if not args.check:
        if not result['passed']: raise ValueError('unsafe spawn; refusing to generate world')
        args.world.parent.mkdir(parents=True,exist_ok=True); args.world.write_text(text)
    else:
        actual=args.world.read_text(); root=ET.fromstring(actual)
        result['reproducible']=actual==text
        result['external_uris']=[e.text for e in root.findall('.//uri')]
        result['includes']=len(root.findall('.//include'))
        result['collision_count']=len(root.findall('.//collision'))
        plugins=[e.get('filename') for e in root.findall('.//plugin')]
        result['plugins']={name:resolve_plugin(name) for name in plugins}
        result['plugins_resolved']=all(result['plugins'].values())
        if shutil.which('ign'):
            proc=subprocess.run(['ign','sdf','-k',str(args.world)],capture_output=True,text=True,timeout=20)
            result['sdf_check']=dict(returncode=proc.returncode,stdout=proc.stdout,stderr=proc.stderr)
        else: result['sdf_check']=dict(returncode=None,stderr='ign unavailable')
        result['passed'] &= (result['reproducible'] and not result['external_uris'] and not result['includes']
            and result['plugins_resolved'] and result['sdf_check']['returncode']==0)
    data=json.dumps(result,indent=2)+'\n'
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(data)
    print(data)
    return 0 if result['passed'] else 1


if __name__=='__main__': raise SystemExit(main())
