"""Pure offline evidence helpers. No ROS imports or navigation side effects."""
import math


class MeasurementClock:
    """Received clock span, summed within epochs; reset jumps add no duration."""
    def __init__(self):
        self.last = None
        self.duration_s = None

    def update(self, t):
        if self.last is not None and t >= self.last:
            self.duration_s = (self.duration_s or 0.) + t-self.last
        self.last = t


def observed_age(events, t, epoch):
    """Uncensored causal age for health only; never makes stale values fresh."""
    matched = align(events, t, epoch, math.inf)
    return t-matched['t'] if matched else None


def age_statistics(values):
    """Linear interpolated quantiles; invalid/negative/missing ages are unknown."""
    values = sorted(v for v in values if v is not None and math.isfinite(v) and v >= 0)
    def quantile(p):
        index = (len(values)-1)*p; lo = math.floor(index)
        return values[lo] + (values[min(lo+1, len(values)-1)]-values[lo])*(index-lo)
    return dict(count=len(values), p50=quantile(.5) if values else None,
                p95=quantile(.95) if values else None, max=values[-1] if values else None)


def measurement_health(rows, counts, topics, wall_duration, simulation_duration, mode):
    """Observer measurements only, not publisher/control rates or causal verdicts."""
    def rate(count, duration):
        return count/duration if duration is not None and math.isfinite(duration) and duration > 0 else None
    fresh = {}
    for key in ['raw', 'smooth', 'final', 'slam', 'truth', 'force_n']:
        limit = .3 if key in ['raw', 'smooth', 'final'] else .5
        def usable(r):
            value = r.get(key); age = r.get('ages', {}).get(key)
            if value is None or age is None or not math.isfinite(age) or not 0 <= age <= limit:
                return False
            if key in ['raw', 'smooth', 'final']:
                return command_state(value) != 'missing'
            return all(math.isfinite(v) for v in value) if key in ['slam', 'truth'] else math.isfinite(value)
        count = sum(usable(r) for r in rows)
        fresh['wrench' if key == 'force_n' else key] = dict(count=count, fraction=count/len(rows) if rows else None)
    return dict(mode=mode, wall_duration_s=wall_duration, simulation_duration_s=simulation_duration,
                sim_wall_ratio=rate(simulation_duration, wall_duration) if simulation_duration is not None else None,
                rate_basis='received count / attach wall duration; NOT publisher or controller rate; includes idle time',
                simulation_duration_basis='sum of received clock advances within epochs; reset jumps excluded',
                topics={topic: dict(received_messages=counts.get(topic, 0),
                                    observed_receive_hz_wall=rate(counts[topic], wall_duration) if counts.get(topic) else None)
                        for topic in topics},
                aligned_samples=len(rows), aligned_sample_observed_hz_wall=rate(len(rows), wall_duration),
                aligned_sample_observed_hz_sim=rate(len(rows), simulation_duration),
                tf_age_s={name: age_statistics((r.get(name+'_tf') or {}).get('age_s') for r in rows)
                          for name in ['map_base', 'odom_base']},
                command_age_s={key: age_statistics(r.get('observed_ages', {}).get(key) for r in rows)
                               for key in ['raw', 'smooth', 'final']},
                command_age_basis='simulation receive-clock age; unstamped Twist transport/backlog latency is unknown',
                age_basis='aligned sample ages including stale causal history; finite nonnegative only; linear quantiles',
                fresh_samples=fresh, freshness_limits_sim_s=dict(commands=.3, odometry_wrench=.5))


class ClockEpoch:
    def __init__(self):
        self.last = None
        self.epoch = 0

    def update(self, t):
        if self.last is not None and t < self.last:
            self.epoch += 1
        self.last = t
        return self.epoch


def align(events, t, epoch, max_age):
    """Latest causal sample, never interpolate unstamped commands or cross resets."""
    usable = [e for e in events if e['epoch'] == epoch and 0 <= t-e['t'] <= max_age]
    return max(usable, key=lambda e: e['t']) if usable else None


def command_state(command, linear=.02, angular=.05):
    if command is None or len(command) != 3 or not all(math.isfinite(v) for v in command):
        return 'missing'
    return 'motion' if math.hypot(*command[:2]) > linear or abs(command[2]) > angular else 'zero'


def angle_distance(a, b):
    return abs(math.atan2(math.sin(a-b), math.cos(a-b)))


def pose_extent(poses):
    if not poses or any(p is None for p in poses):
        return None
    # Bounding-box diagonal bounds all pairwise translation; angular excursion
    # from first pose prevents rotations from being mistaken for immobility.
    return (math.hypot(max(p[0] for p in poses)-min(p[0] for p in poses),
                       max(p[1] for p in poses)-min(p[1] for p in poses)),
            max(angle_distance(p[2], poses[0][2]) for p in poses))


def stationary_intervals(rows, duration=3., translation=.04, rotation=.1, max_gap=.35):
    """Conservative active-goal stationary candidates, not proof of failure.

    Prefer evaluation truth; use SLAM only when truth absent for a whole segment.
    Break on gaps, source/frame changes, goal replacement, resets and missing pose.
    """
    intervals = []; segment = []; source = None
    def finish():
        if segment and segment[-1]['t']-segment[0]['t'] >= duration:
            intervals.append(dict(start=segment[0]['t'], end=segment[-1]['t'],
                                  epoch=segment[0]['epoch'], source=source, rows=list(segment)))
    for r in rows:
        src = 'truth' if r.get('truth') is not None else 'slam'
        p = r.get(src)
        valid = r.get('active') and p is not None and all(math.isfinite(v) for v in p)
        continuous = (not segment or (r['epoch'] == segment[-1]['epoch'] and
                      0 < r['t']-segment[-1]['t'] <= max_gap and source == src and
                      r.get(src+'_frame') == segment[-1].get(src+'_frame') and
                      r.get('goal_ids') == segment[-1].get('goal_ids')))
        if not valid or not continuous:
            finish(); segment = []; source = None
        if not valid: continue
        if segment:
            extent = pose_extent([s[src] for s in segment]+[p])
            if extent[0] > translation or extent[1] > rotation:
                finish(); segment = []
        source = src; segment.append(r)
    finish()
    return intervals


def classify_interval(rows, before=(), logs=(), force_threshold=50.):
    """Time-ordered support labels; ambiguous cases never assign a causal layer."""
    start, end, epoch = rows[0]['t'], rows[-1]['t'], rows[0]['epoch']
    prior = [r for r in before if r['epoch']==epoch and start-1. <= r['t'] < start]
    def states(key):
        return [command_state(r.get(key)) for r in rows]
    def uniformly(key, state):
        return all(s == state for s in states(key))
    truth = pose_extent([r.get('truth') for r in rows])
    slam = pose_extent([r.get('slam') for r in rows])
    stationary = lambda e: e is not None and e[0] <= .04 and e[1] <= .1
    cases = []; reasons = []
    if uniformly('raw', 'zero'):
        # The displacement must occur AFTER an observed fresh zero, not
        # anywhere in the preceding second. Same pose source/frame only.
        zero_suffix = []
        for r in reversed(prior):
            if command_state(r.get('raw')) != 'zero': break
            zero_suffix.append(r)
        zero_suffix.reverse()
        source = 'truth' if all(r.get('truth') is not None for r in zero_suffix+rows[:1]) else 'slam'
        zero_then_moved = False
        for i, r in enumerate(zero_suffix):
            p = r.get(source)
            if p is None: continue
            for later in zero_suffix[i+1:]+rows[:1]:
                q = later.get(source)
                if (q is not None and r.get(source+'_frame')==later.get(source+'_frame') and
                    math.hypot(q[0]-p[0], q[1]-p[1]) > .04): zero_then_moved = True
        if zero_then_moved:
            cases.append('A')
            reasons.append('Raw command near zero while preceding pose samples still displaced; verify ordering in raw events.')
        else:
            reasons.append('Raw zero during stationarity; command-before-stop chronology unavailable.')
    if uniformly('final', 'motion') and stationary(truth):
        cases.append('B')
        reasons.append('Final command materially nonzero with nearly stationary evaluation truth; contact or tracking failure candidate.')
    elif uniformly('final', 'motion') and stationary(slam):
        reasons.append('SLAM-only motion mismatch; physical stationarity unconfirmed.')
    boundary = ('raw_motion_final_zero' if uniformly('raw','motion') and uniformly('final','zero')
                else 'mixed_or_missing' if any('missing' in states(k) or len(set(states(k)))>1 for k in ['raw','smooth','final'])
                else 'no_constant_stage_mismatch')
    if boundary == 'raw_motion_final_zero':
        reasons.append('Investigate smoother/adapter freshness and TF gates before assigning DWB or RL cause.')
    quiet = [r for r in prior if r.get('force_n') is not None and r['force_n'] <= force_threshold]
    rises = [r['t'] for r in rows if r['t'] <= start+1. and r.get('force_n') is not None and r['force_n'] > force_threshold]
    if quiet and rises:
        cases.append('C'); reasons.append('Plate wrench threshold rise near stall onset; limited sensor coverage and inertial loads apply.')
    relevant = [l for l in logs if l['epoch']==epoch and l.get('t') is not None and start-1. <= l['t'] <= end+1.]
    dwb = [l for l in relevant if 'no valid trajectories' in l['text'].lower()]
    exceptions = [l for l in relevant if any(s in l['text'].lower() for s in ['controller patience exceeded','trajectory hits','plannerexception'])]
    if dwb or exceptions:
        cases.append('D'); reasons.append('Controller feasibility/exception log observed; inspect exact text and evaluation scores.')
    progress = [l for l in relevant if 'failed to make progress' in l['text'].lower()]
    if any(l.get('chronology_valid',True) and l['t']-start >= 3. for l in progress):
        cases.append('E'); reasons.append('Progress failure follows an earlier stationary interval; downstream temporal evidence.')
    if progress and not any(l.get('chronology_valid',True) for l in progress):
        reasons.append('Progress log received but occurrence timing is unknown; receipt order cannot prove downstream E.')
    disagreement = truth is not None and slam is not None and stationary(truth) != stationary(slam)
    return dict(start=start, end=end, epoch=epoch, cases=cases, reasons=reasons,
                command_boundary=boundary, truth_extent=truth, slam_extent=slam,
                localization_disagreement=disagreement, first_causal_layer=None,
                physical_collision_proven=False, contact_threshold_n=force_threshold,
                feasibility_logs=dwb+exceptions, progress_logs=progress,
                command_states={k: {s: states(k).count(s) for s in set(states(k))} for k in ['raw','smooth','final']})


def precursor_chronology(rows, events, window_s=6.):
    """Bounded receipt chronology, separate from A–E and causal attribution.

    Six simulation seconds covers the observed 4.435s feasibility-to-stationary
    gap including retry/braking. It is a diagnostic search bound, not a causal
    threshold. Never cross clock epochs or the current NavigateToPose activation.
    Header/receipt clocks remain explicit; missing raw stays unknown.
    """
    if not math.isfinite(window_s) or window_s <= 0:
        raise ValueError('Precursor window must be finite and positive')
    start, epoch = rows[0]['t'], rows[0]['epoch']
    prior = sorted((e for e in events if e['epoch']==epoch and
                    e.get('sim_receive_s') is not None and e['sim_receive_s']<=start),
                   key=lambda e:(e['sim_receive_s'],e['seq']))
    goal_ids=set(rows[0].get('goal_ids',[])); activation=None
    seen=set(); transitions=[]
    for e in prior:
        if e['topic'] not in ['/navigate_to_pose/_action/status','/follow_path/_action/status']:
            continue
        for status in e['value'].get('status_list',[]):
            uid=bytes(status['goal_info']['goal_id']['uuid']).hex()
            key=(e['topic'],uid,status['status'])
            if key in seen: continue
            seen.add(key)
            if e['topic']=='/navigate_to_pose/_action/status' and uid in goal_ids and status['status'] in [1,2]:
                if activation is None: activation=e['sim_receive_s']
            transitions.append((e,dict(goal_id=uid,status=status['status'],
                                       goal_stamp=status['goal_info'].get('stamp'))))
    lower=max(start-window_s,activation if activation is not None else start-window_s)
    def record(e, kind, **extra):
        t=e['sim_receive_s']; stamp=e.get('stamp_s')
        return dict(kind=kind,seq=e['seq'],topic=e['topic'],t=t,epoch=epoch,
                    source_stamp_s=stamp,wall_monotonic_s=e.get('wall_monotonic_s'),
                    source_header_stamp=e['value'].get('stamp',e['value'].get('header',{}).get('stamp')),
                    timestamp_basis='simulation_receive_only',
                    offset_from_stationary_s=t-start,**extra)
    selected=[]; raw=None
    for e in prior:
        if e['sim_receive_s']<lower: continue
        if e['topic']=='/rosout':
            text=e['value'].get('msg',''); low=text.lower()
            if any(k in low for k in ['no valid trajectories','controller patience exceeded',
                                     'starting point in lethal space','failed to make progress',
                                     'aborting handle','received a goal']):
                selected.append(record(e,'log',text=text,name=e['value'].get('name')))
        elif e['topic']=='/course_navigation/cmd_vel_raw':
            v=e['value']; command=[v['linear']['x'],v['linear']['y'],v['angular']['z']]
            if command_state(command)!='missing':
                raw=record(e,'last_raw_command',command=command,state=command_state(command))
    if raw is not None: selected.append(raw)
    selected.extend(record(e,'action_status',**s) for e,s in transitions if e['sim_receive_s']>=lower)
    selected.sort(key=lambda e:(e['t'],e['seq']))
    return dict(window_s=window_s,window_start=lower,window_end=start,
                current_goal_activation_receive_s=activation,events=selected,last_raw_command=raw,
                attribution='Receipt chronology only; does not alter A–E or assign first_causal_layer')


def topic_health(expected, graph, counts):
    return {name: dict(expected_type=kind, discovered_types=graph.get(name, []),
                       messages=counts.get(name, 0),
                       state=('absent' if name not in graph else 'type_mismatch' if kind not in graph[name]
                              else 'recorded' if counts.get(name, 0) else 'no_messages'))
            for name, kind in expected.items()}


def navigation_state(status, publisher_present):
    if status is None or not publisher_present:
        return dict(active=None, goal_ids=[], evidence='unknown')
    active = [s for s in status.get('status_list',[]) if s['status'] in [1,2]]
    return dict(active=bool(active), goal_ids=[bytes(s['goal_info']['goal_id']['uuid']).hex() for s in active],
                evidence='observed_active' if active else 'observed_terminal_or_empty')


def relative_footprint(world_polygon, pose):
    """Invert published footprint pose; input is already padded by costmap ROS."""
    x,y,theta=pose; c,s=math.cos(theta),math.sin(theta)
    return [[c*(a-x)+s*(b-y),-s*(a-x)+c*(b-y)] for a,b in world_polygon]


def pad_footprint(poly, padding):
    if padding < 0 or not math.isfinite(padding): raise ValueError('Invalid padding')
    return [[x + (padding if x>0 else -padding if x<0 else 0),
             y + (padding if y>0 else -padding if y<0 else 0)] for x,y in poly]


def transform_polygon(poly, pose):
    x,y,yaw = pose; c,s = math.cos(yaw),math.sin(yaw)
    return [[x+c*a-s*b, y+s*a+c*b] for a,b in poly]


def edges(poly):
    return list(zip(poly, poly[1:]+poly[:1]))


def overlap(a, b):
    """Closed convex polygon intersection via separating axes (cell areas)."""
    for p,q in edges(a)+edges(b):
        axis = [q[1]-p[1], p[0]-q[0]]
        pa=[v[0]*axis[0]+v[1]*axis[1] for v in a]
        pb=[v[0]*axis[0]+v[1]*axis[1] for v in b]
        if max(pa)<min(pb)-1e-12 or max(pb)<min(pa)-1e-12: return False
    return True


def point_segment_distance(p, a, b):
    dx,dy=b[0]-a[0],b[1]-a[1]; den=dx*dx+dy*dy
    t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/den)) if den else 0.
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)


def polygon_distance(a,b):
    if overlap(a,b): return 0.
    return min([point_segment_distance(p,x,y) for p in a for x,y in edges(b)]+
               [point_segment_distance(p,x,y) for p in b for x,y in edges(a)])


def footprint_geometry(grid, pose, footprint):
    """Filled padded polygon against raw cell areas, NOT Nav2 critic replication.

    Lethal=254 is represented obstacle overlap; inscribed=253 is inflated-zone
    overlap, not physical contact. Unknown/out-of-map forbids a clear verdict.
    Clearance is in this finite 2D grid, excludes unknown and 3D body dynamics.
    """
    w,h,res=grid['width'],grid['height'],grid['resolution']; data=grid['data']
    if w<=0 or h<=0 or res<=0 or not math.isfinite(res) or len(data)!=w*h:
        raise ValueError('Malformed raw costmap')
    if len(footprint)<3: raise ValueError('Missing footprint')
    if not all(math.isfinite(v) for p in footprint for v in p): raise ValueError('Nonfinite footprint')
    signs=[]
    for i in range(len(footprint)):
        a,b,c=footprint[i-2],footprint[i-1],footprint[i]
        cross=(b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0])
        if abs(cross)>1e-12: signs.append(cross>0)
    if not signs or len(set(signs))>1: raise ValueError('Only convex footprints supported')
    ox,oy,oyaw=grid['origin']; c,s=math.cos(oyaw),math.sin(oyaw)
    world=transform_polygon(footprint,pose)
    poly=[[(c*(x-ox)+s*(y-oy))/res,(-s*(x-ox)+c*(y-oy))/res] for x,y in world]
    cx=(c*(pose[0]-ox)+s*(pose[1]-oy))/res; cy=(-s*(pose[0]-ox)+c*(pose[1]-oy))/res
    ix,iy=math.floor(cx),math.floor(cy)
    center=data[iy*w+ix] if 0<=ix<w and 0<=iy<h else None
    outside=any(x<0 or y<0 or x>=w or y>=h for x,y in poly)
    lethal=inscribed=unknown=False; clear_lethal=clear_inscribed=None
    # First bound the footprint cells; unknown far outside does not affect overlap.
    lo_x=max(0,math.floor(min(x for x,y in poly)-1e-10)); hi_x=min(w-1,math.floor(max(x for x,y in poly)))
    lo_y=max(0,math.floor(min(y for x,y in poly)-1e-10)); hi_y=min(h-1,math.floor(max(y for x,y in poly)))
    def cell(x,y): return [[x,y],[x+1,y],[x+1,y+1],[x,y+1]]
    for y in range(lo_y,hi_y+1):
        for x in range(lo_x,hi_x+1):
            cost=data[y*w+x]
            if cost>=253 and overlap(poly,cell(x,y)):
                lethal |= cost==254; inscribed |= cost==253; unknown |= cost==255
    for i,cost in enumerate(data):
        if cost not in (253,254): continue
        distance=polygon_distance(poly,cell(i%w,i//w))*res
        if cost==254: clear_lethal=distance if clear_lethal is None else min(clear_lethal,distance)
        clear_inscribed=distance if clear_inscribed is None else min(clear_inscribed,distance)
    complete=not outside and not unknown
    return dict(center_cost=int(center) if center is not None else None,
                center_safe=center is not None and center<253,
                lethal_overlap=lethal, inscribed_overlap=inscribed, unknown_overlap=unknown,
                outside_map=outside, complete=complete,
                center_safe_footprint_colliding=bool(center is not None and center<253 and lethal),
                clear_of_represented_lethal=(not lethal) if complete else None,
                clearance_to_lethal_m=clear_lethal, clearance_to_lethal_or_inscribed_m=clear_inscribed,
                footprint_world_xy=world, physical_collision_measurement=False)
