"""Measured independent-inhibit stopping; does not depend on action status."""
import math


class IndependentStop:
    def __init__(self,watermark):
        self.watermark=dict(watermark);self.first=self.last=None;self.confirmed=False

    def observe(self,s):
        try:
            keys=('sim','wall','x','y','tf_stamp','odom_stamp','map_odom_stamp','pose_age','clock_age',
                  'speed','wz','command_speed','command_wz')
            if any(type(s[k]) not in (int,float) or not math.isfinite(s[k]) for k in keys):raise ValueError()
            if (s['tf_stamp']<self.watermark['sim'] or s['wall']<self.watermark['wall'] or
                    any(not 0<=s['sim']-s[k]<=.5 for k in ('tf_stamp','odom_stamp','map_odom_stamp')) or
                    any(not 0<=s[k]<=.5 for k in ('pose_age','clock_age')) or
                    not 0<=s['speed']<=.03 or abs(s['wz'])>.03 or
                    not 0<=s['command_speed']<=.01 or abs(s['command_wz'])>.01):raise ValueError()
            if self.last:
                dt,dw=s['tf_stamp']-self.last['tf_stamp'],s['wall']-self.last['wall']
                if dt==0:return  # Repeated source cannot renew or extend the hold.
                if not 0<dt<=.3 or not 0<dw<=.5:raise ValueError()
            if self.first is None:self.first=dict(s)
            if math.dist([s['x'],s['y']],[self.first['x'],self.first['y']])>.03:raise ValueError()
            self.last=dict(s)
            self.confirmed=(s['tf_stamp']-self.first['tf_stamp']>=5. and s['wall']-self.first['wall']>=3.)
        except (KeyError,TypeError,ValueError):self.first=self.last=None;self.confirmed=False

    def report(self):
        return dict(confirmed=self.confirmed,inhibit_watermark=self.watermark,
            hold_start_tf=self.first['tf_stamp'] if self.first else None,hold_end_tf=self.last['tf_stamp'] if self.last else None,
            hold_start_wall=self.first['wall'] if self.first else None,hold_end_wall=self.last['wall'] if self.last else None,
            source_span_s=self.last['tf_stamp']-self.first['tf_stamp'] if self.first and self.last else 0.,
            wall_span_s=self.last['wall']-self.first['wall'] if self.first and self.last else 0.,
            action_independent=True)
