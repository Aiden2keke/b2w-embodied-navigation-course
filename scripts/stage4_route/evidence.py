"""Independent ordered measured-passage evidence, never an action advancement driver."""
import math


def distance(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1];q=dx*dx+dy*dy
    t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/q)) if q else 0.
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)


class RouteEvidence:
    def __init__(self,goals,*,start,count=5):
        if count not in (2,5):raise ValueError('Only critical or full execution')
        self.goals=goals[:count];self.start=start;self.events=[];self.path=[];self.failure=None
        self.minimum=[None]*count;self.inside_since=None;self.last=None;self.stop_since=None
        self.status=None;self.terminal_sim=None;self.narrow_samples=0;self.path_length=0.

    def fail(self,reason):
        if self.failure is None:self.failure=reason

    def observe(self,s):
        if self.failure:return
        try:
            keys=['sim','wall','x','y','yaw','tf_stamp','odom_stamp','map_odom_stamp','slam_map_age',
                  'slam_map_wall_age','slam_map_wall_budget',
                  'clock_age','pose_age','tf_receipt_age','speed','wz','command_speed','command_wz']
            if any(type(s[k]) not in (int,float) or not math.isfinite(s[k]) for k in keys):raise ValueError()
            if (any(not 0<=s['sim']-s[k]<=.5 for k in ('tf_stamp','odom_stamp','map_odom_stamp')) or
                any(not 0<=s[k]<=.5 for k in ('clock_age','pose_age','tf_receipt_age')) or
                not 0<=s['slam_map_age']<=7.5 or
                not 7.5<=s['slam_map_wall_budget']<=20. or
                not 0<=s['slam_map_wall_age']<=s['slam_map_wall_budget']):raise ValueError()
        except (KeyError,TypeError,ValueError):self.fail('stale_or_missing_localization');return
        if self.last:
            if any(s[k]<=self.last[k] for k in ('sim','wall','tf_stamp','odom_stamp')):self.fail('clock_reset_or_duplicate');return
            if s['sim']-self.last['sim']>.3 or s['wall']-self.last['wall']>.5:self.fail('pose_gap');return
            delta=math.hypot(s['x']-self.last['x'],s['y']-self.last['y'])
            if delta>.2:self.fail('pose_jump');return
            self.path_length+=delta
        self.last=s;self.path.append(dict(s))
        if s.get('safety')!='clear':self.fail('unsafe_or_missing_costmap_collision_evidence');return
        errors=[math.hypot(s['x']-g['position'][0],s['y']-g['position'][1]) for g in self.goals]
        for i,d in enumerate(errors):self.minimum[i]=d if self.minimum[i] is None else min(d,self.minimum[i])
        i=len(self.events)
        if i<len(self.goals):
            if any(d<=.25 for d in errors[i+1:]):
                self.fail('out_of_order_'+self.goals[next(k for k in range(i+1,len(errors)) if errors[k]<=.25)]['name']);return
            a=self.start if i==0 else self.goals[i-1]['position'];b=self.goals[i]['position']
            band=.35 if i==1 else 3.0
            if distance((s['x'],s['y']),a,b)>band:self.fail('disallowed_detour');return
            if i==1:self.narrow_samples+=1
            if errors[i]<=.25:
                if self.inside_since is None:self.inside_since=s
                if s['sim']-self.inside_since['sim']>=.1-1e-9:
                    self.events.append(dict(goal=self.goals[i]['name'],entry=self.inside_since,
                                            confirmed=s,xy_error_m=errors[i],radius_m=.25))
                    self.inside_since=None
            else:self.inside_since=None
        if self.terminal_sim is not None and s['sim']>=self.terminal_sim:
            yaw_error=abs(math.remainder(s['yaw']-self.goals[-1]['yaw_rad'],math.tau))
            stopped=(errors[-1]<=.25 and yaw_error<=.3 and s['speed']<=.03 and abs(s['wz'])<=.03 and
                     s['command_speed']<=.01 and abs(s['command_wz'])<=.01)
            self.stop_since=(s['sim'] if self.stop_since is None else self.stop_since) if stopped else None

    def terminal(self,status,*,sim):
        self.status=status;self.terminal_sim=sim;self.stop_since=None
        if status!=4:self.fail('navigation_not_succeeded')

    def finish(self,*,cleanup,safety_complete):
        stop=(self.last is not None and self.stop_since is not None and
              self.last['sim']-self.stop_since>=1.)
        narrow=len(self.events)>=2 and self.narrow_samples>=2
        practical=(not self.failure and len(self.events)==len(self.goals) and narrow and self.status==4 and stop and cleanup)
        return dict(result=('FULL_ROUTE_PASS' if practical and safety_complete and len(self.goals)==5 else
                            'PRACTICAL_ROUTE_PASS_SAFETY_UNCERTAIN' if practical else 'ROUTE_FAIL'),
                    full_route_pass=bool(practical and safety_complete and len(self.goals)==5),
                    practical_route_pass=bool(practical),historical_005m_benchmark='not_run_unmodified',
                    failure=self.failure,region_events=self.events,narrow_passage_confirmed=narrow,
                    action_status=self.status,terminal_stop_confirmed=bool(stop),cleanup_passed=cleanup,
                    collision_coverage_complete=safety_complete,path_length_m=self.path_length,
                    per_goal_minimum_xy_error_m=dict(zip([g['name'] for g in self.goals],self.minimum)),
                    terminal_xy_error_m=(math.hypot(self.last['x']-self.goals[-1]['position'][0],self.last['y']-self.goals[-1]['position'][1]) if self.last else None),
                    path=self.path,uncertainty='Sampled padded footprint/costmap and plate coverage; unobserved full-body contact remains unknown')
