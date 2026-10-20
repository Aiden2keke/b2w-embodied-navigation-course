"""Single versioned topological restriction, independent of physical clearance."""
import json
import math
from pathlib import Path

POLICY=Path(__file__).resolve().parents[2]/'course_navigation/config/stage4_route_constraints.json'


def constraints():
    return json.loads(POLICY.read_text())


def segment_band(phase):
    if type(phase) is not int or not 0<=phase<5:raise ValueError('Original segment phase required')
    return constraints()['segment_center_band_m'][phase]


def scope(mode,segment=None):
    names=('start-g1','g1-g2','g2-g3','g3-g4','g4-g5')
    if mode=='segment' and segment in names:return names.index(segment),1
    if segment is not None:raise ValueError('--segment requires --mode segment')
    if mode=='critical':return 0,2
    if mode=='full':return 0,5
    raise ValueError('Select an original independent segment')


def deadline_for(length_m,*,measured_speed,rotation_rad,measured_yaw_rate,mode):
    values=(length_m,measured_speed,rotation_rad,measured_yaw_rate)
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in values) or min(length_m,rotation_rad)<0 or min(measured_speed,measured_yaw_rate)<=0:
        raise ValueError('Measured positive achieved rates and finite path geometry required')
    # Conservative rates must come from a same-profile, sensor-measured lower
    # bound, never a requested Twist. Global campaign caps remain hard limits.
    sim=1.5*(length_m/measured_speed+rotation_rad/measured_yaw_rate)+20.
    wall=1.5*sim+20.
    caps={'e1':(90.,180.),'segment':(180.,300.),'critical':(180.,300.),'full':(600.,900.)}
    if mode not in caps or sim>caps[mode][0] or wall>caps[mode][1]:raise ValueError('Derived deadline exceeds authorized stage cap')
    return dict(sim_s=sim,wall_s=wall)
