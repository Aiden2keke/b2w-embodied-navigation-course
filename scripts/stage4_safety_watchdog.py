#!/usr/bin/python3
"""Action-independent local lease watchdog; publishes no ROS velocity or goals."""
import argparse
import json
import time
from course_navigation.stage4_control import Watchdog,read_control,latch_inhibit


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',required=True);p.add_argument('--wall-limit',type=float,required=True)
    a=p.parse_args()
    if not 0<a.wall_limit<=960:p.error('Explicit watchdog wall limit must be <=960 s')
    config=read_control(a.manifest);watchdog=Watchdog(config);started=time.monotonic()
    try:
        while time.monotonic()-started<a.wall_limit:
            if not watchdog.step():
                print(json.dumps(dict(event='stage4_watchdog_first_fault',**watchdog.first_fault),
                    sort_keys=True,allow_nan=False),flush=True)
                return 2
            time.sleep(.05)
        return 0
    finally:latch_inhibit(config['inhibit_path'],'watchdog_terminated')


if __name__=='__main__':raise SystemExit(main())
