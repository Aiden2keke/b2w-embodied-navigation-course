"""Independent ordered measured-passage evidence, never an action advancement driver."""
import math
import copy


def distance(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1];q=dx*dx+dy*dy
    t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/q)) if q else 0.
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)


class RouteEvidence:
    def __init__(self,goals,*,start,count=5,start_index=0,portal=None):
        if (type(count) is not int or type(start_index) is not int or len(goals)!=5 or
                count<1 or start_index<0 or start_index+count>5):
            raise ValueError('Select a contiguous segment of the original five goals')
        if len(start)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) for v in start):
            raise ValueError('Start must contain two finite numeric coordinates')
        from .portal import CorridorPortal
        self.portal_provenance=copy.deepcopy(portal)
        self.portal=CorridorPortal(portal) if portal is not None and start_index<=1<start_index+count else None
        self.original_goals=copy.deepcopy(goals);self.start_index=start_index
        self.goals=self.original_goals[start_index:start_index+count];self.start=list(start);self.events=[];self.path=[];self.failure=None
        if start_index and math.dist(start,goals[start_index-1]['position'])>.25:
            raise ValueError('Segment Start must be in the preceding original goal region')
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
        if self.portal is not None:
            self.portal.observe(s)
            if self.portal.failure:self.fail(self.portal.failure);return
        if s.get('safety')!='clear':self.fail('unsafe_or_missing_costmap_collision_evidence');return
        errors=[math.hypot(s['x']-g['position'][0],s['y']-g['position'][1]) for g in self.goals]
        for i,d in enumerate(errors):self.minimum[i]=d if self.minimum[i] is None else min(d,self.minimum[i])
        i=len(self.events)
        phase=self.start_index+min(i,len(self.goals)-1)
        for future in self.original_goals[phase+1:]:
            if math.hypot(s['x']-future['position'][0],s['y']-future['position'][1])<=.25:
                self.fail('out_of_order_'+future['name']);return
        if i<len(self.goals):
            a=self.start if i==0 else self.goals[i-1]['position'];b=self.goals[i]['position']
            from .constraints import segment_band
            band=segment_band(phase)
            if distance((s['x'],s['y']),a,b)>band:self.fail('disallowed_detour');return
            if self.start_index and len(self.path)==1 and math.dist((s['x'],s['y']),self.start)>.03:
                self.fail('segment_start_pose_mismatch');return
            if (self.start_index and len(self.path)==1 and
                    math.dist((s['x'],s['y']),self.original_goals[self.start_index-1]['position'])>.25):
                self.fail('segment_start_outside_preceding_region');return
            if phase==1:self.narrow_samples+=1
            if errors[i]<=.25:
                if self.inside_since is None:self.inside_since=s
                if s['sim']-self.inside_since['sim']>=.1-1e-9:
                    self.events.append(dict(goal=self.goals[i]['name'],entry=self.inside_since,
                                            confirmed=s,xy_error_m=errors[i],radius_m=.25))
                    self.inside_since=None
            else:self.inside_since=None
        if self.terminal_sim is not None and s['tf_stamp']>=self.terminal_sim:
            yaw_error=abs(math.remainder(s['yaw']-self.goals[-1]['yaw_rad'],math.tau))
            stopped=(errors[-1]<=.25 and yaw_error<=.3 and s['speed']<=.03 and abs(s['wz'])<=.03 and
                     s['command_speed']<=.01 and abs(s['command_wz'])<=.01)
            if stopped:
                if self.stop_since is None:self.stop_since=dict(s)
                if math.hypot(s['x']-self.stop_since['x'],s['y']-self.stop_since['y'])>.03:
                    self.fail('terminal_hold_excursion')
            elif self.stop_since is not None:
                self.fail('terminal_hold_broken');self.stop_since=None

    def terminal(self,status,*,sim):
        self.status=status;self.terminal_sim=sim;self.stop_since=None
        if status!=4:self.fail('navigation_not_succeeded')

    def finish(self,*,cleanup,safety_complete):
        stop=(self.last is not None and self.stop_since is not None and
              not self.failure and
              self.last['tf_stamp']-self.stop_since['tf_stamp']>=5. and
              self.last['wall']-self.stop_since['wall']>=3.)
        includes_narrow=self.start_index<=1<self.start_index+len(self.goals)
        ordered_narrow=includes_narrow and len(self.events)>1-self.start_index and self.narrow_samples>=2
        narrow=bool(ordered_narrow and self.portal is not None and self.portal.passed)
        practical=(not self.failure and len(self.events)==len(self.goals) and
                   (narrow or not includes_narrow) and self.status==4 and stop and cleanup)
        full_scope=self.start_index==0 and len(self.goals)==5
        segment_pass=bool(practical and safety_complete and not full_scope)
        return dict(result=('FULL_ROUTE_PASS' if practical and safety_complete and full_scope else
                            'SEGMENT_PASS' if segment_pass else
                            'PRACTICAL_ROUTE_PASS_SAFETY_UNCERTAIN' if practical else 'ROUTE_FAIL'),
                    full_route_pass=bool(practical and safety_complete and full_scope),
                    segment_pass=segment_pass,original_goal_start_index=self.start_index,
                    practical_route_pass=bool(practical),historical_005m_benchmark='not_run_unmodified',
                    failure=self.failure,region_events=self.events,narrow_passage_confirmed=narrow,
                    ordered_narrow_regions_diagnostic=ordered_narrow,
                    portal_events=self.portal.events if self.portal is not None else {},
                    corridor_evidence_missing=bool(includes_narrow and self.portal is None),
                    action_status=self.status,terminal_stop_confirmed=bool(stop),cleanup_passed=cleanup,
                    collision_coverage_complete=safety_complete,path_length_m=self.path_length,
                    per_goal_minimum_xy_error_m=dict(zip([g['name'] for g in self.goals],self.minimum)),
                    terminal_xy_error_m=(math.hypot(self.last['x']-self.goals[-1]['position'][0],self.last['y']-self.goals[-1]['position'][1]) if self.last else None),
                    path=self.path,uncertainty='Sampled padded footprint/costmap and plate coverage; unobserved full-body contact remains unknown')
