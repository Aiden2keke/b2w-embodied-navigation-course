"""Route startup only. No ROS imports, publishers, goals or replacement local planner.

The caller must supply a fresh planner-produced path and an independently
validated physical reaction+stopping displacement bound for its operating range.
None means unavailable, never zero. Nav2 acceleration settings and one observed
coast are not such a bound. A circular articulated containment footprint covers
all initial rotations, including rotation before translation.
"""
import math
import xml.etree.ElementTree as ET


def collision_radius(urdf):
    """Conservative all-joint/all-attitude root sphere from collision primitives.

    Triangle inequality along each joint chain; collision-origin rotations and
    joint angles cannot increase the sum. This deliberately does not infer a
    gait envelope from nominal standing joints or from the smaller Nav2 polygon.
    Unsupported geometry, missing joints and disconnected/cyclic trees reject.
    """
    root = ET.parse(urdf).getroot()
    links = {link.get('name'): link for link in root.findall('link')}
    parents = {}
    def norm(text):
        v = [float(x) for x in text.split()]
        if len(v)!=3 or not all(math.isfinite(x) for x in v): raise ValueError('Invalid xyz')
        return math.sqrt(sum(x*x for x in v))
    for joint in root.findall('joint'):
        if joint.get('type') not in ('fixed','revolute','continuous'):
            raise ValueError('Unsupported joint motion')
        child = joint.find('child').get('link')
        if child in parents: raise ValueError('Multiple parents')
        origin = joint.find('origin')
        parents[child] = (joint.find('parent').get('link'),norm(origin.get('xyz','0 0 0')) if origin is not None else 0.)
    def offset(name, seen=()):
        if name in seen or name not in links: raise ValueError('Invalid tree')
        if name == 'base_link': return 0.
        parent, distance = parents[name]
        return distance+offset(parent,seen+(name,))
    radius = 0.; count=0
    for name, link in links.items():
        distance=offset(name)
        for collision in link.findall('collision'):
            origin=collision.find('origin'); geometry=collision.find('geometry')
            if len(geometry)!=1: raise ValueError('Invalid collision')
            shape=geometry[0]
            if shape.tag=='box':
                extent=norm(shape.get('size'))/2
            elif shape.tag=='sphere': extent=float(shape.get('radius'))
            elif shape.tag=='cylinder': extent=math.hypot(float(shape.get('radius')),float(shape.get('length'))/2)
            else: raise ValueError('Unsupported collision geometry')
            if not math.isfinite(extent) or extent<=0: raise ValueError('Invalid collision extent')
            local=norm(origin.get('xyz','0 0 0')) if origin is not None else 0.
            radius=max(radius,distance+local+extent);count+=1
    if not count or not math.isfinite(radius) or radius<=0: raise ValueError('Missing collision footprint')
    return radius


def _point_segment(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy))) if dx or dy else 0.
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)


def _cell_distance(a,b,x0,y0,x1,y1):
    # Segment/AABB clipping detects crossing even with both endpoints outside.
    lo,hi=0.,1.
    for origin,delta,lower,upper in ((a[0],b[0]-a[0],x0,x1),(a[1],b[1]-a[1],y0,y1)):
        if delta==0:
            if not lower<=origin<=upper: break
        else:
            t0,t1=sorted(((lower-origin)/delta,(upper-origin)/delta))
            lo,hi=max(lo,t0),min(hi,t1)
            if lo>hi: break
    else: return 0.
    def to_box(p): return math.hypot(max(x0-p[0],0.,p[0]-x1),max(y0-p[1],0.,p[1]-y1))
    return min(to_box(a),to_box(b),*(_point_segment(p,a,b) for p in ((x0,y0),(x0,y1),(x1,y0),(x1,y1))))


def _conflict(grid, a, b, radius):
    r=grid['resolution'];ox,oy=grid['origin'];w,h=grid['width'],grid['height']
    if (type(w)!=int or type(h)!=int or w<=0 or h<=0 or len(grid['data'])!=w*h
            or not all(math.isfinite(v) for v in (r,ox,oy)) or r<=0):
        raise ValueError('Invalid grid')
    for y in range(math.floor((min(a[1],b[1])-radius-oy)/r)-1,math.floor((max(a[1],b[1])+radius-oy)/r)+1):
        for x in range(math.floor((min(a[0],b[0])-radius-ox)/r)-1,math.floor((max(a[0],b[0])+radius-ox)/r)+1):
            if _cell_distance(a,b,ox+x*r,oy+y*r,ox+(x+1)*r,oy+(y+1)*r)>radius+1e-12: continue
            value=grid['data'][y*w+x] if 0<=x<w and 0<=y<h else None
            if value!=0:
                return dict(x=x,y=y,value=value,cell_xy=[ox+x*r,oy+y*r])
    return None


def admit(grid, *, pose, path, radius, padding, reaction_stop_m, ready):
    """Check exact cell rectangles against continuous swept disks, fail closed.

    padding is a declared pose-error allowance plus footprint padding; cell
    discretization is handled by whole-cell intersection, without center tests.
    Path reliability/freshness/goal identity are the live caller's responsibility.
    This predicate never grants motion from current-footprint clearance alone.
    """
    result=dict(passed=False,current_footprint_passed=False,reason='invalid_inputs')
    try:
        if ready is not True:
            result['reason']='tf_odom_contacts_or_ownership';return result
        if (len(pose)!=3 or not all(math.isfinite(v) for v in pose)
                or not math.isfinite(radius) or radius<=0 or not math.isfinite(padding) or padding<0): return result
        conflict=_conflict(grid,pose,pose,radius+padding)
        if conflict:
            result.update(reason='current_footprint',conflict=conflict);return result
        result['current_footprint_passed']=True
        if path is None or len(path)<2:
            result['reason']='missing_validated_path';return result
        if any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in path): return result
        if math.dist(pose[:2],path[0][:2])>.05 or abs(math.remainder(pose[2]-path[0][2],2*math.pi))>.05:
            result['reason']='path_start_mismatch';return result
        for a,b in zip([pose]+list(path),path):
            conflict=_conflict(grid,a,b,radius+padding)
            if conflict:
                result.update(reason='swept_footprint',conflict=conflict);return result
        if reaction_stop_m is None:
            result['reason']='missing_validated_reaction_stopping_bound';return result
        if not math.isfinite(reaction_stop_m) or reaction_stop_m<0: return result
        # Include robot-to-first-path offset and the stop/reaction envelope at
        # every point, not just at the final prefix pose. Disk covers yaw sweep.
        for a,b in zip([pose]+list(path),path):
            conflict=_conflict(grid,a,b,radius+padding+reaction_stop_m)
            if conflict:
                result.update(reason='swept_or_stopping_footprint',conflict=conflict);return result
        result.update(passed=True,reason='passed',radius_m=radius,padding_m=padding,reaction_stop_m=reaction_stop_m)
    except (KeyError,ValueError,TypeError,OverflowError,IndexError):
        pass
    return result


def initial_prefix(packet, pose, goal, clock, length=.5):
    """Validate planning-only Smac result, retain an exact initial path prefix.

    The initial current pose is included so an initial change of yaw is covered.
    This is geometric admission, not a claim that DWB will execute this polyline.
    """
    points=packet['poses']
    if (packet['status']!=4 or packet['frame']!='map' or not 0<=clock-packet['stamp']<=.5
            or len(points)<2 or not 0<length<=.5
            or any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in points)
            or math.dist(points[0][:2],pose[:2])>.05
            or math.dist(points[-1][:2],goal)>.3):
        raise ValueError('Nonvalidated, stale or mismatched initial path')
    prefix=[tuple(pose),tuple(points[0])];remaining=length
    for a,b in zip(points,points[1:]):
        distance=math.dist(a[:2],b[:2])
        if distance>remaining:
            t=remaining/distance
            prefix.append((a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]),b[2]))
            break
        prefix.append(tuple(b));remaining-=distance
    if sum(math.dist(a[:2],b[:2]) for a,b in zip(prefix,prefix[1:]))<min(.1,length):
        raise ValueError('Degenerate initial path')
    return prefix


def _rectangle_conflict(grid,pose,front,rear,side):
    """SAT overlap of a rotated body rectangle and complete axis-aligned cells."""
    r=grid['resolution'];ox,oy=grid['origin'];w,h=grid['width'],grid['height']
    if (type(w)!=int or type(h)!=int or min(w,h)<=0 or len(grid['data'])!=w*h
            or not all(math.isfinite(v) for v in (r,ox,oy)) or r<=0):raise ValueError('Invalid grid')
    c,s=math.cos(pose[2]),math.sin(pose[2]);hx=(front+rear)/2;shift=(front-rear)/2
    cx,cy=pose[0]+c*shift,pose[1]+s*shift
    ex,ey=abs(c)*hx+abs(s)*side,abs(s)*hx+abs(c)*side
    for y in range(math.floor((cy-ey-oy)/r)-1,math.floor((cy+ey-oy)/r)+1):
        for x in range(math.floor((cx-ex-ox)/r)-1,math.floor((cx+ex-ox)/r)+1):
            dx,dy=ox+(x+.5)*r-cx,oy+(y+.5)*r-cy
            cell_radius=r/2*(abs(c)+abs(s))
            if (abs(dx)>ex+r/2+1e-12 or abs(dy)>ey+r/2+1e-12
                    or abs(c*dx+s*dy)>hx+cell_radius+1e-12
                    or abs(-s*dx+c*dy)>side+cell_radius+1e-12):continue
            value=grid['data'][y*w+x] if 0<=x<w and 0<=y<h else None
            if value!=0:return dict(x=x,y=y,value=value,cell_xy=[ox+x*r,oy+y*r])
    return None


def admit_directional(grid,*,pose,path,half_extents,padding,stop,ready):
    """Operational rectangle with directional reaction/braking/tracking bounds.

    Caller supplies qualified footprint and displacement/yaw bounds for the
    operating range. Inflation is only padding and continuous-sweep error;
    braking translation is separated into forward, reverse and lateral bounds.
    No positive result is possible with absent stop evidence.
    """
    result=dict(passed=False,footprint_passed=False,reason='invalid_operational_inputs')
    try:
        hx,hy=half_extents
        if (ready is not True or len(pose)!=3 or not all(math.isfinite(v) for v in (*pose,hx,hy,padding))
                or min(hx,hy)<=0 or padding<0):return result
        conflict=_rectangle_conflict(grid,pose,hx+padding,hx+padding,hy+padding)
        if conflict:result.update(reason='current_footprint_collision',conflict=conflict);return result
        result['footprint_passed']=True
        if path is None or len(path)<2:result['reason']='missing_validated_path';return result
        if any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in path):return result
        if math.dist(path[0][:2],pose[:2])>.05:result['reason']='path_start_mismatch';return result
        rho=math.hypot(hx,hy);samples=[]
        # Smac2D does not prescribe DWB's initial turn sense or winding.
        # Cover every initial orientation at the current center rather than
        # inventing a shortest commanded rotation from two path quaternions.
        rotations=[(pose[0],pose[1],pose[2]+i*math.pi/2) for i in range(5)]
        sweep=rotations+list(path)
        for segment,(a,b) in enumerate(zip([pose]+sweep,sweep)):
            yaw=math.remainder(b[2]-a[2],2*math.pi);distance=math.dist(a[:2],b[:2])
            n=max(1,math.ceil((distance+rho*abs(yaw))/(grid['resolution']/2)))
            error=(distance+rho*abs(yaw))/(2*n)
            for i in range(n+1):
                t=i/n;p=(a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]),a[2]+t*yaw)
                conflict=_rectangle_conflict(grid,p,hx+padding+error,hx+padding+error,hy+padding+error)
                if conflict:result.update(reason='initial_sweep_collision',conflict=conflict);return result
                samples.append((p,error,segment<len(rotations)))
        if stop is None:result['reason']='missing_qualified_stopping_bounds';return result
        f,b,l,yaw,drift=(stop[k] for k in ('forward_m','reverse_m','lateral_m','yaw_rad','rotation_drift_m'))
        if not all(math.isfinite(v) and v>=0 for v in (f,b,l,yaw,drift)):return result
        rotation_margin=rho*min(yaw,2.)
        for p,error,rotating in samples:
            margin=padding+error+rotation_margin
            # In-place turn drift is measured independently; forward braking
            # reach must not be projected sideways at hypothetical turn angles.
            front,back,side=(drift,drift,drift) if rotating else (f,b,l)
            conflict=_rectangle_conflict(grid,p,hx+front+margin,hx+back+margin,hy+side+margin)
            if conflict:result.update(reason='directional_stopping_collision',conflict=conflict);return result
        result.update(passed=True,reason='passed',half_extents=list(half_extents),padding_m=padding,stop=stop)
    except (ValueError,KeyError,TypeError,OverflowError,IndexError):pass
    return result


def load_envelope(file,expected_sha,urdf_sha,profile_sha,command_caps,deployment_sha=None):
    """Load operator-reviewed empirical bounds pinned to deployment and raw data.

    Hashes establish identity, not scientific sufficiency. The separate review
    and trial authorization must accept the measured operating-range coverage.
    No default qualified envelope is shipped or synthesized from a single run.
    """
    import hashlib,json
    from pathlib import Path
    file=Path(file);data=file.read_bytes()
    if expected_sha is None or hashlib.sha256(data).hexdigest()!=expected_sha:raise ValueError('Reviewed envelope hash mismatch')
    e=json.loads(data)
    if (e['schema_version']!=1 or e['urdf_sha256']!=urdf_sha or e['profile_sha256']!=profile_sha
            or e['command_caps']!=command_caps or len(e['half_extents'])!=2
            or e['half_extents'][0]<.7 or e['half_extents'][1]<.38
            or e['padding_m']<.05 or e['localization_error_m']<=0
            or e['tracking_error_m']<0 or not 0<e['body_tilt_limit_rad']<=.4
            or not e['evidence']):raise ValueError('Envelope operating range/deployment invalid')
    if deployment_sha is not None and e.get('deployment_manifest_sha256')!=deployment_sha:
        raise ValueError('Envelope deployment manifest mismatch')
    values=[*e['half_extents'],e['padding_m'],e['localization_error_m'],e['tracking_error_m'],e['body_tilt_limit_rad'],e['physical_speed_cap_mps'],e['physical_yaw_cap_rad_s']]
    if e['physical_speed_cap_mps']<=0 or e['physical_yaw_cap_rad_s']<=0:
        raise ValueError('Physical operating caps must be positive')
    stop=[e['stop'][k] for k in ('forward_m','reverse_m','lateral_m','yaw_rad','rotation_drift_m')]
    if not all(type(v) in (int,float) and math.isfinite(v) and v>=0 for v in values+stop) or stop[0]<=0 or stop[3]<=0:
        raise ValueError('Invalid physical bounds')
    for row in e['evidence']:
        p=Path(row['path']);p=p if p.is_absolute() else file.parent/p
        if hashlib.sha256(p.read_bytes()).hexdigest()!=row['sha256']:raise ValueError('Raw qualification evidence changed')
    return e


def arrival_yaw(packet,goal,grid,half_extents,padding,clock):
    """Derive a continuation yaw only from a fresh, collision-checked incoming path.

    Geometry validation includes the exact waypoint and arrival rotation. This
    does not qualify stopping or authorize a goal. Feed its result to the existing
    sequential runner's explicit --continuation-yaws only after evidence review.
    """
    initial_prefix(packet,packet['request_pose'],goal,clock)
    points=packet['poses']
    if math.dist(points[-1][:2],goal)>.05:raise ValueError('Incoming endpoint outside explicit .05m route evaluation')
    yaw=None
    for a,b in reversed(list(zip(points,points[1:]))):
        if math.dist(a[:2],b[:2])>1e-9:
            yaw=math.atan2(b[1]-a[1],b[0]-a[0]);break
    if yaw is None:raise ValueError('No incoming tangent')
    result=admit_directional(grid,pose=points[0],path=list(points)+[(goal[0],goal[1],yaw)],
        half_extents=half_extents,padding=padding,stop=None,ready=True)
    if result['reason']!='missing_qualified_stopping_bounds':raise ValueError('Incoming path/arrival footprint is not known-free')
    return yaw
