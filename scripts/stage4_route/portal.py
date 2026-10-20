"""Sensor-bound portal evidence. This observer cannot steer or grant motion."""
import copy
import math


class CorridorPortal:
    def __init__(self,certificate):
        c=copy.deepcopy(certificate);self.certificate=c;self.events={};self.last=None;self.failure=None
        if c['kind']!='stage4_sensor_corridor_portal' or c['version']!=1 or not c['session_id']:
            raise ValueError('Versioned sensor portal required')
        for key in ('sensor_sha256','registration_sha256','map_sha256'):
            if len(c[key])!=64 or any(v not in '0123456789abcdef' for v in c[key]):raise ValueError('Sensor/registration/map content identities required')
        values=[*c['origin'],*c['tangent'],c['entry_m'],c['exit_m'],*c['lateral_bounds_m'],c['uncertainty_m'],c['max_yaw_rad']]
        if (len(c['origin'])!=2 or len(c['tangent'])!=2 or len(c['lateral_bounds_m'])!=2 or
                any(type(v) not in (int,float) or not math.isfinite(v) for v in values) or
                abs(math.hypot(*c['tangent'])-1)>1e-8 or c['exit_m']<=c['entry_m'] or
                c['lateral_bounds_m'][0]>=c['lateral_bounds_m'][1] or c['uncertainty_m']<=0 or not 0<c['max_yaw_rad']<=.2):
            raise ValueError('Finite measured portal geometry and conservative uncertainty required')

    @property
    def passed(self):return not self.failure and 'tail_exit' in self.events

    def observe(self,s):
        if self.failure or self.passed:return
        try:
            if any(type(s[k]) not in (int,float) or not math.isfinite(s[k]) for k in ('x','y','yaw','sim','wall')):raise ValueError('invalid_portal_pose')
            c=self.certificate;tx,ty=c['tangent'];dx,dy=s['x']-c['origin'][0],s['y']-c['origin'][1]
            along=dx*tx+dy*ty;lateral=-dx*ty+dy*tx
            yaw=math.remainder(s['yaw']-math.atan2(ty,tx),math.tau)
            # Conservative full padded body including orientation support.
            longitudinal=.75*abs(math.cos(yaw))+.43*abs(math.sin(yaw))+c['uncertainty_m']
            cross=.75*abs(math.sin(yaw))+.43*abs(math.cos(yaw))+c['uncertainty_m']
            front,tail=along+longitudinal,along-longitudinal
            if self.last:
                if not (0<s['sim']-self.last['sim']<=.3 and 0<s['wall']-self.last['wall']<=.5):raise ValueError('portal_pose_gap_or_clock')
                if math.dist([s['x'],s['y']],[self.last['x'],self.last['y']])>.2:raise ValueError('portal_pose_jump')
                if 'front_entry' in self.events and along<self.last['along']-1e-4:raise ValueError('portal_reverse_motion')
            elif front>=c['entry_m']:raise ValueError('portal_entry_not_observed')
            if front>=c['entry_m'] and tail<=c['exit_m']:
                if abs(yaw)>c['max_yaw_rad']:raise ValueError('portal_heading_violation')
                if lateral-cross<c['lateral_bounds_m'][0] or lateral+cross>c['lateral_bounds_m'][1]:raise ValueError('portal_full_body_lateral_violation')
            for name,crossed in (('front_entry',front>=c['entry_m']),('tail_entry',tail>=c['entry_m']),
                                 ('front_exit',front>c['exit_m']),('tail_exit',tail>c['exit_m'])):
                if crossed and name not in self.events:self.events[name]=copy.deepcopy(s)
            if 'tail_exit' in self.events and not all(k in self.events for k in ('front_entry','tail_entry','front_exit')):raise ValueError('portal_events_incomplete')
            self.last=dict(s,along=along)
        except (KeyError,TypeError,ValueError) as exc:self.failure=str(exc)
