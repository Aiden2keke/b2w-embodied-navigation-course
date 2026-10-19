"""Pure offline geometry and immutable-source candidate export. No ROS dependency."""
import ast
import copy
import csv
import hashlib
import json
from types import SimpleNamespace
import math
from pathlib import Path
import tempfile

from PIL import Image, ImageDraw
import yaml

ROOT = Path(__file__).resolve().parents[2]
DISCLAIMER = ('Offline filled-footprint geometry only; no physical collision-free feasibility proof. '
              'Unknown, occupied, outside and restricted regions block. Sweeps are discrete samples. '
              'Fresh SLAM frames require independently verified registration to this saved map.')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def pixel_to_map(m, u, v):
    """Integer u,v denote image pixel centers; origin denotes lower-left cell corner."""
    x, y = (u + .5)*m['resolution'], (m['height'] - v - .5)*m['resolution']
    ox, oy, a = m['origin']
    c, s = math.cos(a), math.sin(a)
    return [ox + c*x - s*y, oy + s*x + c*y]


def map_to_pixel(m, x, y):
    ox, oy, a = m['origin']
    c, s = math.cos(a), math.sin(a)
    x, y = x-ox, y-oy
    return [(c*x+s*y)/m['resolution']-.5,
            m['height']-(-s*x+c*y)/m['resolution']-.5]


def transform_polygon(points, pose):
    x, y, a = pose
    c, s = math.cos(a), math.sin(a)
    return [[x+c*u-s*v, y+s*u+c*v] for u,v in points]


def cross(a,b,c):
    return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])


def on_segment(a,b,p):
    return abs(cross(a,b,p))<1e-10 and all(min(a[i],b[i])-1e-10<=p[i]<=max(a[i],b[i])+1e-10 for i in (0,1))


def segments_intersect(a,b,c,d):
    return ((cross(a,b,c)*cross(a,b,d)<0 and cross(c,d,a)*cross(c,d,b)<0)
            or any((on_segment(a,b,c),on_segment(a,b,d),on_segment(c,d,a),on_segment(c,d,b))))


def inside(p, poly):
    result=False
    for a,b in zip(poly,poly[1:]+poly[:1]):
        if on_segment(a,b,p): return True
        if (a[1]>p[1]) != (b[1]>p[1]) and p[0] < (b[0]-a[0])*(p[1]-a[1])/(b[1]-a[1])+a[0]:
            result=not result
    return result


def polygon_valid(p):
    try:
        if not 3<=len(p)<=256 or any(len(v)!=2 or not all(type(x) in (int,float) and math.isfinite(x) for x in v) for v in p): return False
        if len({tuple(v) for v in p})!=len(p): return False
        if abs(sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(p,p[1:]+p[:1])))<1e-10: return False
        n=len(p)
        for i in range(n):
            for j in range(i+1,n):
                if j==i+1 or (i==0 and j==n-1): continue
                if segments_intersect(p[i],p[(i+1)%n],p[j],p[(j+1)%n]): return False
        return True
    except (TypeError,ValueError): return False


def polygons_overlap(a,b):
    return (any(inside(p,b) for p in a) or any(inside(p,a) for p in b)
            or any(segments_intersect(p,q,r,s) for p,q in zip(a,a[1:]+a[:1]) for r,s in zip(b,b[1:]+b[:1])))


class Map:
    def __init__(self, path):
        self.path=Path(path).resolve()
        self.raw=yaml.safe_load(self.path.read_text())
        m=self.raw
        if (m.get('mode','trinary')!='trinary' or m.get('negate') not in (0,1)
                or not 0<m['resolution']<100 or not math.isfinite(m['resolution'])
                or len(m['origin'])!=3 or not all(math.isfinite(v) for v in m['origin'])
                or not 0<=m['free_thresh']<m['occupied_thresh']<=1):
            raise ValueError('Unsupported or invalid map metadata; require trinary planar map')
        self.image_path=(self.path.parent/m['image']).resolve()
        with Image.open(self.image_path) as im:
            if im.mode!='L': raise ValueError('Require grayscale map without alpha')
            self.image=im.copy()
        w,h=self.image.size
        if w*h>4_000_000: raise ValueError('Map exceeds offline size bound')
        self.meta=dict(m,width=w,height=h)
        self.identity=dict(yaml_path=str(self.path), image_path=str(self.image_path),
            yaml_sha256=sha(self.path), image_sha256=sha(self.image_path), metadata=copy.deepcopy(self.meta))
        self.occupancy_policy='Trinary YAML thresholds; grayscale 205 with negate=0 is conservatively unknown even if below free_thresh (saved-map ambiguity). Closed-cell contact blocks.'
        values=list(self.image.getdata())
        def classify(g):
            if g==205 and m['negate']==0: return 'unknown'
            occ=(g if m['negate'] else 255-g)/255.
            return 'occupied' if occ>m['occupied_thresh'] else 'free' if occ<m['free_thresh'] else 'unknown'
        self.cells=[[classify(g) for g in values[y*w:(y+1)*w]] for y in range(h)]


class Editor:
    def __init__(self,map_path,config_path,route_path):
        self.map=Map(map_path)
        self.config_path=Path(config_path).resolve()
        self.route_path=Path(route_path).resolve()
        cfg=yaml.safe_load(self.config_path.read_text())
        footprints=[]
        for key in ('local_costmap','global_costmap'):
            p=cfg[key][key]['ros__parameters']
            points=ast.literal_eval(p['footprint']) if isinstance(p['footprint'],str) else p['footprint']
            pad=p['footprint_padding']
            if not math.isfinite(pad) or pad<0 or not polygon_valid(points): raise ValueError('Invalid configured footprint')
            footprints.append([[x+(pad if x>0 else -pad if x<0 else 0),y+(pad if y>0 else -pad if y<0 else 0)] for x,y in points])
        if footprints[0]!=footprints[1]: raise ValueError('Local/global padded footprints differ')
        self.footprint=footprints[0]
        self.route=yaml.safe_load(self.route_path.read_text())
        # Inspect contract without importing ROS or the route transport runner.
        g=self.route['goals']
        if (self.route['schema_version']!=1 or self.route['frame_id']!='map'
                or [x['name'] for x in g]!=['G1','G2','G3','G4','G5']
                or [x['orientation'] for x in g]!=['corridor_heading']*2+[None]*3):
            raise ValueError('Unsupported canonical route schema')
        def heading(a,b): return math.atan2(b[1]-a[1],b[0]-a[0])
        goals=[]
        for i,goal in enumerate(g):
            a,b=(g[0],g[1]) if i<2 else (g[i],g[i+1]) if i<4 else (g[3],g[4])
            goals.append(dict(name=goal['name'],position=copy.deepcopy(goal['position']),
                              yaw_rad=heading(a['position'],b['position'])))
        self.sources={str(p):sha(p) for p in (self.map.path,self.map.image_path,self.config_path,self.route_path)}
        self.costmaps={}
        self.initial=dict(map_identity=copy.deepcopy(self.map.identity),goals=goals,polygons=[],markers={})

    def import_costmap(self,path,name):
        path=Path(path).resolve()
        if name not in ('local','global') or path.stat().st_size>32_000_000: raise ValueError('Invalid costmap input')
        r=json.loads(path.read_text())
        if (r['frame_id']!='map' or r['encoding']!='nav2_uint8'
                or r['source_map_image_sha256']!=self.map.identity['image_sha256']
                or r['source_map_yaml_sha256']!=self.map.identity['yaml_sha256']):
            raise ValueError('Costmap frame/map identity not explicitly matched')
        w,h=r['width'],r['height']
        if (type(w)!=int or type(h)!=int or not 0<w*h<=4_000_000 or w<=0 or h<=0
                or not math.isfinite(r['resolution']) or r['resolution']<=0
                or len(r['origin'])!=3 or not all(math.isfinite(v) for v in r['origin'])
                or len(r['data'])!=w*h or any(type(v)!=int or not 0<=v<=255 for v in r['data'])):
            raise ValueError('Invalid costmap geometry/data')
        values=r['data']
        rows=[values[(h-1-y)*w:(h-y)*w] for y in range(h)]
        cells=[['unknown' if v==255 else 'occupied' if v>=253 else 'free' for v in row] for row in rows]
        image=Image.new('RGBA',(w,h)); image.putdata([(255,70,100,150) if v>=253 else (255,190,50,min(110,v//2)) for row in rows for v in row])
        self.costmaps[name]=SimpleNamespace(meta={k:r[k] for k in ('width','height','origin','resolution')},image=image,cells=cells)
        self.sources[str(path)]=sha(path)

    def assert_sources(self):
        if any(sha(p)!=h for p,h in self.sources.items()): raise ValueError('Source files changed; restart and inspect identity')

    def validate(self,state):
        if state.get('map_identity')!=self.map.identity: raise ValueError('Map identity mismatch')
        if [g['name'] for g in state['goals']]!=['G1','G2','G3','G4','G5']: raise ValueError('Require G1–G5')
        for g in state['goals']:
            if len(g['position'])!=2: raise ValueError('Invalid XY')
            self.pose_valid([*g['position'],g['yaw_rad']])
        if len(state['polygons'])>100: raise ValueError('Too many polygons')
        for p in state['polygons']:
            if p['kind'] not in ('corridor','restricted') or not polygon_valid(p['points']): raise ValueError('Invalid polygon')
        if set(state['markers'])-{'entrance','exit'}: raise ValueError('Invalid marker')
        for pose in state['markers'].values(): self.pose_valid(pose)

    @staticmethod
    def pose_valid(pose):
        if len(pose)!=3 or any(type(v) not in (float,int) or not math.isfinite(v) or abs(v)>1e6 for v in pose): raise ValueError('Invalid finite pose')

    def check_pose(self,pose,polygons=(),grid=None):
        self.pose_valid(pose)
        target=grid or self.map
        world=transform_polygon(self.footprint,pose)
        poly=[map_to_pixel(target.meta,*p) for p in world]
        w,h=target.image.size
        outside=any(x<-.5 or x>w-.5 or y<-.5 or y>h-.5 for x,y in poly)
        counts=dict(unknown=0,occupied=0)
        x0=max(0,math.ceil(min(p[0] for p in poly)-.5-1e-10)); x1=min(w-1,math.floor(max(p[0] for p in poly)+.5+1e-10))
        y0=max(0,math.ceil(min(p[1] for p in poly)-.5-1e-10)); y1=min(h-1,math.floor(max(p[1] for p in poly)+.5+1e-10))
        for y in range(y0,y1+1):
            for x in range(x0,x1+1):
                kind=target.cells[y][x]
                if kind!='free' and polygons_overlap(poly,[[x-.5,y-.5],[x+.5,y-.5],[x+.5,y+.5],[x-.5,y+.5]]): counts[kind]+=1
        restricted=any(p['kind']=='restricted' and polygons_overlap(world,p['points']) for p in polygons)
        costmaps={name:self.check_pose(pose,polygons,grid=cm) for name,cm in self.costmaps.items()} if grid is None else {}
        return dict(**counts,outside=outside,restricted=restricted,costmaps=costmaps,
                    clear=not(outside or restricted or any(counts.values()) or any(not c['clear'] for c in costmaps.values())),physical_feasibility_proven=False)

    def check_state(self,state):
        self.validate(state)
        poses={g['name']:[*g['position'],g['yaw_rad']] for g in state['goals']}
        poses.update(state['markers'])
        return {name:self.check_pose(pose,state['polygons']) for name,pose in poses.items()}

    def sweep(self,pose,polygons=()):
        samples=[dict(yaw_rad=pose[2]+i*math.tau/72,**self.check_pose([*pose[:2],pose[2]+i*math.tau/72],polygons)) for i in range(73)]
        return dict(sample_count=len(samples),step_degrees=5,samples=samples,physical_feasibility_proven=False)

    def export(self,state,output_root):
        self.validate(state); self.assert_sources()
        output_root=Path(output_root).resolve()
        # Export only into /tmp, never repo, canonical configuration, or arbitrary source dirs.
        if not output_root.is_relative_to(Path('/tmp')) or output_root==Path('/tmp'): raise ValueError('Exports require a dedicated directory below /tmp')
        if any(Path(p).is_relative_to(output_root) for p in self.sources): raise ValueError('Export root contains source files')
        output_root.mkdir(parents=True,exist_ok=True)
        folder=Path(tempfile.mkdtemp(prefix='candidate-',dir=output_root))
        goals=[]
        for g in state['goals']:
            a=g['yaw_rad']
            goals.append(dict(g,quaternion=dict(x=0.,y=0.,z=math.sin(a/2),w=math.cos(a/2))))
        result=dict(kind='offline_waypoint_candidate',candidate_schema_version=1,frame_id='map',
            deployable=False,occupancy_policy=self.map.occupancy_policy,source_map=self.map.identity,source_hashes=self.sources,
            costmap_metadata={name:cm.meta for name,cm in self.costmaps.items()},
            canonical_route_snapshot=self.route,goals=goals,markers=state['markers'],polygons=state['polygons'],
            geometry=self.check_state(state),padded_footprint=self.footprint,limitations=DISCLAIMER)
        (folder/'candidate.yaml').write_text(yaml.safe_dump(result,sort_keys=False))
        (folder/'annotations.yaml').write_text(yaml.safe_dump(dict(polygons=state['polygons'],markers=state['markers']),sort_keys=False))
        with (folder/'waypoints.csv').open('w') as f:
            writer=csv.writer(f); writer.writerow(['name','x_m','y_m','yaw_rad'])
            writer.writerows([g['name'],*g['position'],g['yaw_rad']] for g in goals)
            writer.writerows([k,*v] for k,v in state['markers'].items())
        self.render(state,folder/'annotated.png')
        manifest=dict(occupancy_policy=self.map.occupancy_policy,sources=self.sources,map_identity=self.map.identity,outputs={p.name:sha(p) for p in folder.iterdir()},limitations=DISCLAIMER)
        (folder/'manifest.yaml').write_text(yaml.safe_dump(manifest,sort_keys=False))
        return folder

    def render(self,state,path):
        # Draw at 3x native resolution, preserving full map extent and aspect ratio.
        scale=3
        im=self.map.image.convert('RGB').resize((self.map.image.width*scale,self.map.image.height*scale),Image.NEAREST)
        draw=ImageDraw.Draw(im)
        def px(p): return tuple((v+.5)*scale for v in map_to_pixel(self.map.meta,*p))
        for p in state['polygons']:
            points=[px(v) for v in p['points']]; draw.line(points+points[:1],fill='red' if p['kind']=='restricted' else 'purple',width=3)
        for goals,color in ((self.initial['goals'],'orange'),(state['goals'],'cyan')):
            for g in goals:
                pose=[*g['position'],g['yaw_rad']]; center=px(pose[:2])
                poly=[px(v) for v in transform_polygon(self.footprint,pose)]
                draw.line(poly+poly[:1],fill=color,width=2)
                end=px([pose[0]+math.cos(pose[2]),pose[1]+math.sin(pose[2])])
                draw.line([center,end],fill=color,width=3); draw.text(center,g['name'],fill=color)
        for name,pose in state['markers'].items():
            x,y=px(pose[:2]); draw.ellipse([x-7,y-7,x+7,y+7],outline='magenta',width=3); draw.text((x+8,y),name,fill='magenta')
        draw.rectangle([0,0,im.width,35],fill='white'); draw.text((5,5),'OFFLINE CANDIDATE | orange original / cyan candidate | geometry only; fresh SLAM registration unverified',fill='black')
        im.save(path)
