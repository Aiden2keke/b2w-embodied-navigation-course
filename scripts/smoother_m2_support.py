#!/usr/bin/env python3
"""OFFLINE nominal OPEN_LOOP smoother/M2 support proof from current YAML.

This is command arithmetic, not measured robot motion or an exact-route replay.
Only the unscaled, zero-x-deadband open-loop case is justified here; stop on
configurations requiring other dynamics instead of guessing their behavior.
"""
import argparse
import hashlib
import json
import math
import sys
from decimal import Decimal
from pathlib import Path

import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'course_navigation'))
from course_navigation import m2_inverse as m2
NAV_CONFIG=ROOT/'course_navigation/config/nav2.yaml'


def bucket(v):
    if v==0.:return 'ZERO'
    if v<0:return 'NEGATIVE'
    if v<m2.V_MIN:return 'LOW'
    if v<=m2.V_MAX:return 'SUPPORTED'
    return 'HIGH'


def ramp(start,target,delta,until_supported=False):
    """Nominal rate-limited recurrence with Decimal, no binary64 tiny tail."""
    value=Decimal(str(start));target=Decimal(str(target));step=Decimal(str(delta));out=[]
    if step<=0:raise ValueError('positive nominal tick increment required')
    for _ in range(100000):
        if value==target:return out
        value=min(target,value+step) if value<target else max(target,value-step)
        v=float(value);out.append({'vx':v,'bucket':bucket(v)})
        if until_supported and m2.V_MIN<=v<=m2.V_MAX:return out
    raise ValueError('nominal ramp exceeds audit step bound')


def audit(config_path=NAV_CONFIG):
    path=Path(config_path);cfg=yaml.safe_load(path.read_text());p=cfg['velocity_smoother']['ros__parameters']
    f=float(p['smoothing_frequency']);up=float(p['max_accel'][0]);down=float(p['max_decel'][0])
    lo=float(p['min_velocity'][0]);hi=float(p['max_velocity'][0]);deadband=float(p['deadband_velocity'][0])
    if not all(math.isfinite(x) for x in (f,up,down,lo,hi,deadband)) or not (f>0 and up>0 and down<0 and lo<=0<hi):
        raise ValueError('YAML cannot support finite positive-rate forward/stop calculation')
    if p['feedback']!='OPEN_LOOP' or p['scale_velocities'] is not False or deadband!=0.:
        raise ValueError('nominal proof requires OPEN_LOOP, scale_velocities=false and zero x deadband')
    target=.15
    if not lo<=0<=target<=hi or not m2.V_MIN<=target<=m2.V_MAX:
        raise ValueError('representative .15m/s target not allowed by current YAML and M2 support')
    up_step=float(Decimal(str(up))/Decimal(str(f)));down_step=float(abs(Decimal(str(down)))/Decimal(str(f)))
    startup=ramp(0.,target,up_step,until_supported=True)
    deceleration=[dict(startup[-1])]+ramp(startup[-1]['vx'],0.,down_step)
    blocker=any(point['bucket']=='LOW' for point in startup+deceleration)
    return {'offline_only':True,'physical_motion_data':False,'source':str(path.resolve()),
            'source_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
            'configuration':{'smoothing_frequency':f,'max_accel_x':up,'max_decel_x':down,
                'min_velocity_x':lo,'max_velocity_x':hi,'feedback':p['feedback'],
                'scale_velocities':p['scale_velocities'],'deadband_velocity_x':deadband},
            'delta_v_up':up_step,'delta_v_down':down_step,'representative_target_vx':target,
            'startup':startup,'deceleration':deceleration,
            'classification':'LOW_SPEED_SUPPORT_BLOCKER' if blocker else 'NO_NOMINAL_LOW_SPEED_GAP_OBSERVED',
            'NORMAL_NAV2_M2_READY':False,
            'reason':'Strict M2 adapter follows smoother and checks every command before legacy limiting. Positive LOW ramp values reject; route timing evidence is unnecessary for this structural blocker.',
            'validated_condition':'pure_forward_vy0_wz0',
            'high_speed_risk':{'configured_smoother_max_vx':hi,
                'configured_DWB_max_vel_x':cfg['controller_server']['ros__parameters']['FollowPath']['max_vel_x'],
                'M2_desired_v_max':m2.V_MAX,'allowed_above_M2_support':hi>m2.V_MAX,
                'frozen_route_fraction':None,'every_route_reaches_max':False}}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,default=NAV_CONFIG);p.add_argument('--output',type=Path)
    a=p.parse_args();r=audit(a.config);text=json.dumps(r,indent=2,allow_nan=False)+'\n'
    if a.output:a.output.write_text(text)
    else:print(text,end='')


if __name__=='__main__':main()
