from pathlib import Path
import sys
root = Path(__file__).resolve().parents[1]
for name in ['course_sim','course_slam','course_navigation','course_bringup']:
    sys.path.insert(0, str(root/name))
