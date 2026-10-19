#!/usr/bin/python3
"""Serve the independent offline waypoint editor. Does not import or contact ROS."""
import argparse
from pathlib import Path
from waypoint_editor.core import Editor, ROOT
from waypoint_editor.server import create_server


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--map',type=Path,default=Path('/tmp/b2w-waypoint-map/warehouse.yaml'))
    p.add_argument('--nav-config',type=Path,default=ROOT/'course_navigation/config/nav2.yaml')
    p.add_argument('--route',type=Path,default=ROOT/'course_navigation/config/warehouse_route.yaml')
    p.add_argument('--output',type=Path,default=Path('/tmp/b2w-waypoint-candidates'))
    p.add_argument('--local-costmap',type=Path,help='Registered nav2_uint8 JSON; see docs/WAYPOINT_EDITOR.md')
    p.add_argument('--global-costmap',type=Path,help='Registered nav2_uint8 JSON; see docs/WAYPOINT_EDITOR.md')
    p.add_argument('--candidate',type=Path,help='Opt-in reload of a verified v1/v2 offline candidate YAML; Reset returns to this state')
    p.add_argument('--port',type=int,default=8765)
    args=p.parse_args()
    try:
        editor=Editor(args.map,args.nav_config,args.route)
        for name,path in [('local',args.local_costmap),('global',args.global_costmap)]:
            if path: editor.import_costmap(path,name)
        if args.candidate: editor.import_candidate(args.candidate)
    except (ValueError,OSError,KeyError,TypeError) as error:
        p.error(str(error))
    with create_server(editor,args.output,args.port) as server:
        print(f'Offline editor: http://127.0.0.1:{server.server_port}',flush=True)
        print(f'Map: {editor.map.meta}; image SHA256: {editor.map.identity["image_sha256"]}',flush=True)
        print('Ctrl+C to stop. Exports are new candidates only; no deployment.',flush=True)
        try: server.serve_forever()
        except KeyboardInterrupt: pass


if __name__=='__main__': main()
