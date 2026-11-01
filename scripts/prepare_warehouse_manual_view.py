#!/usr/bin/python3
"""Create a per-session Fortress GUI config from the reviewed manual camera pose."""
import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VIEWS = ROOT / 'artifacts/video/manual_views'


def prepare(destination):
    raw = (VIEWS / 'gazebo_camera_pose_final.txt').read_text()

    def values(name, axes):
        match = re.search(name + r'\s*\{([^}]+)\}', raw)
        if not match:
            raise ValueError('Saved camera lacks ' + name)
        data = {key: float(value) for key, value in
                re.findall(r'([xyzw]):\s*([-+0-9.eE]+)', match[1])}
        result = [data[key] for key in axes]
        if not all(math.isfinite(value) for value in result):
            raise ValueError('Invalid camera pose')
        return result

    xyz = values('position', 'xyz')
    x, y, z, w = values('orientation', 'xyzw')
    if abs(x*x + y*y + z*z + w*w - 1.) > 1e-4:
        raise ValueError('Camera quaternion is not normalized')
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(max(-1., min(1., 2*(w*y - z*x))))
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    config = (VIEWS / 'gazebo_gui.config').read_text()
    pattern = r'(<plugin\s+filename="MinimalScene"[^>]*>.*?<camera_pose>)([^<]*)(</camera_pose>)'
    matches = list(re.finditer(pattern, config, re.S))
    if len(matches) != 1:
        raise ValueError('Expected one MinimalScene camera_pose in approved GUI config')
    match = matches[0]
    pose = ' '.join(format(value, '.17g') for value in xyz + [roll, pitch, yaw])
    rendered = config[:match.start(2)] + pose + config[match.end(2):]
    target = Path(destination)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(rendered)
    return pose


if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise SystemExit('usage: prepare_warehouse_manual_view.py OUTPUT_CONFIG')
    print(prepare(sys.argv[1]))
