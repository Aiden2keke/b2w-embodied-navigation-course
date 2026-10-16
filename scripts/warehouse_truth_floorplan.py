"""Canonical WORLD floorplan from authored warehouse collision triangles, offline only."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DIRECTORY = ROOT / 'course_navigation/maps'
METADATA = DIRECTORY / 'warehouse_ground_truth_floorplan.json'
IMAGE = DIRECTORY / 'warehouse_ground_truth_floorplan.png'
REVISION = 'ee0af733315e78432408c3cd98d378ecee5f767c'


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def world_geometry_sha256(path):
    """Relocatable SDF geometry identity; filesystem prefix is not geometry."""
    tree = ET.parse(path).getroot()
    for node in tree.iter():
        if node.tag == 'uri' and '/models/' in (node.text or ''):
            node.text = 'models/' + node.text.split('/models/', 1)[1]
        if node.text is not None:
            node.text = node.text.strip() or None
        node.tail = None
    return hashlib.sha256(ET.tostring(tree)).hexdigest()


def world_to_pixel(points, metadata):
    p = np.asarray(points, dtype=float)
    return np.stack(((p[..., 0]-metadata['xmin'])/metadata['resolution']-.5,
                     (metadata['ymax']-p[..., 1])/metadata['resolution']-.5), axis=-1)


def pixel_to_world(points, metadata):
    p = np.asarray(points, dtype=float)
    return np.stack((metadata['xmin']+(p[..., 0]+.5)*metadata['resolution'],
                     metadata['ymax']-(p[..., 1]+.5)*metadata['resolution']), axis=-1)


def rasterize(geometry, metadata):
    """Pixel-center coverage of actual projected triangles, no hulls/AABBs as geometry."""
    w, h, r = metadata['width'], metadata['height'], metadata['resolution']
    result = np.full((h, w, 3), 205, dtype=np.uint8)
    for model in sorted(geometry, key=lambda m: not m['support']):
        for tri in model['triangles_xy']:
            tri = np.asarray(tri)
            area = np.cross(tri[1]-tri[0], tri[2]-tri[0])
            if area == 0:  # Zero-area projection contains no pixel-center area.
                continue
            pix = world_to_pixel(tri, metadata)
            low = np.maximum(np.ceil(pix.min(axis=0)).astype(int), [0, 0])
            high = np.minimum(np.floor(pix.max(axis=0)).astype(int), [w-1, h-1])
            if np.any(high < low): continue
            cols, rows = np.meshgrid(np.arange(low[0], high[0]+1), np.arange(low[1], high[1]+1))
            points = pixel_to_world(np.stack((cols, rows), axis=-1), metadata)
            signs = [np.cross(tri[(i+1)%3]-tri[i], points-tri[i]) for i in range(3)]
            inside = np.logical_and.reduce([s >= 0 for s in signs] if area > 0 else [s <= 0 for s in signs])
            region = result[low[1]:high[1]+1, low[0]:high[0]+1]
            region[inside] = (248, 248, 242) if model['support'] else (20, 20, 20)
    return result


def load_floorplan(path=METADATA):
    from PIL import Image
    path = Path(path)
    metadata = json.loads(path.read_text())
    image = path.parent / metadata['image']
    if not image.is_file() or sha256(image) != metadata['image_sha256']:
        raise ValueError('Canonical floorplan image SHA mismatch')
    geometry = path.parent / metadata['geometry']
    if not geometry.is_file() or sha256(geometry) != metadata['geometry_sha256']:
        raise ValueError('Canonical floorplan geometry SHA mismatch')
    r = metadata['resolution']
    if (metadata['frame_id'] != 'world' or not math.isfinite(r) or r <= 0
            or not math.isclose(metadata['xmax']-metadata['xmin'], metadata['width']*r, abs_tol=1e-12)
            or not math.isclose(metadata['ymax']-metadata['ymin'], metadata['height']*r, abs_tol=1e-12)):
        raise ValueError('Invalid exact WORLD/image contract')
    data = np.asarray(Image.open(image).convert('RGB'))
    if data.shape != (metadata['height'], metadata['width'], 3):
        raise ValueError('Floorplan image size mismatch')
    return metadata, data


def pose_matrix(node):
    from scipy.spatial.transform import Rotation
    pose = node.find('pose')
    if pose is not None and any(k not in ('frame', 'relative_to') or v for k, v in pose.attrib.items()):
        raise ValueError('Unsupported relative/non-radian SDF pose; no guess')
    values = [float(v) for v in node.findtext('pose', '0 0 0 0 0 0').split()]
    if len(values) != 6 or not np.isfinite(values).all(): raise ValueError('Invalid SDF pose')
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler('xyz', values[3:]).as_matrix()
    matrix[:3, 3] = values[:3]
    return matrix


def build(mesh_exporter, output=DIRECTORY):
    from PIL import Image
    assets = ROOT / 'third_party/aws_warehouse'
    if (assets / 'REVISION').read_text().strip() != REVISION:
        raise ValueError('Pinned warehouse revision mismatch')
    world_file = assets / 'warehouse.world'
    world = ET.parse(world_file).getroot().find('world')
    geometry, sources, cache = [], set(), {}
    for model in world.findall('model'):
        if model.findtext('static') not in ('1', 'true') or model.findall('model') or model.findall('include'):
            raise ValueError('Nonstatic/nested scenery cannot be silently flattened')
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                mesh = collision.find('geometry/mesh')
                if mesh is None or len(collision.find('geometry')) != 1 or mesh.find('submesh') is not None:
                    raise ValueError('Unsupported exact collision shape')
                mesh_file = Path(mesh.findtext('uri').removeprefix('file://')).resolve()
                if not mesh_file.is_relative_to(assets / 'models'): raise ValueError('Mesh outside pinned assets')
                sources.update([mesh_file, mesh_file.parents[1] / 'model.sdf'])
                if mesh_file not in cache:
                    cache[mesh_file] = np.array(json.loads(subprocess.check_output([str(mesh_exporter), str(mesh_file)], text=True)))
                triangles = cache[mesh_file]
                if triangles.ndim != 3 or triangles.shape[1:] != (3, 3) or not np.isfinite(triangles).all():
                    raise ValueError('Mesh loader did not return exact finite triangles')
                scale = np.array([float(v) for v in mesh.findtext('scale', '1 1 1').split()])
                if scale.shape != (3,) or not np.isfinite(scale).all() or np.any(scale <= 0): raise ValueError('Invalid mesh scale')
                transform = pose_matrix(model) @ pose_matrix(link) @ pose_matrix(collision)
                transformed = (triangles*scale) @ transform[:3, :3].T + transform[:3, 3]
                support = mesh_file.name == 'aws_robomaker_warehouse_GroundB_01_collision.DAE'
                geometry.append(dict(name=model.get('name'), support=support,
                    triangles_xy=transformed[..., :2].tolist(), z_bounds=[float(transformed[..., 2].min()), float(transformed[..., 2].max())]))
    points = np.concatenate([np.asarray(m['triangles_xy']).reshape(-1, 2) for m in geometry])
    low, high = points.min(axis=0), points.max(axis=0)
    resolution = .02
    w, h = np.ceil((high-low)/resolution).astype(int)
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    image = output / IMAGE.name
    geometry_file = output / 'warehouse_ground_truth_floorplan.geometry.json'
    geometry_file.write_text(json.dumps(geometry, separators=(',', ':'), allow_nan=False)+'\n')
    sources.update([world_file, assets / 'REVISION', assets / 'worlds/no_roof_small_warehouse/no_roof_small_warehouse.world',
                    ROOT / 'scripts/prepare_warehouse.py', Path(__file__).resolve(), ROOT / 'scripts/warehouse_collision_mesh.cc'])
    metadata = dict(schema_version=1, frame_id='world', image=image.name, geometry=geometry_file.name,
        projection='all-height orthographic authored static collision triangles; GroundB is support',
        resolution=resolution, width=int(w), height=int(h), xmin=float(low[0]), xmax=float(low[0]+w*resolution),
        ymin=float(low[1]), ymax=float(low[1]+h*resolution), source_xy_bounds=[low.tolist(), high.tolist()],
        pixel_contract='top-left pixel center: x=xmin+(column+.5)*resolution; y=ymax-(row+.5)*resolution',
        rasterization='pixel-center triangle coverage, closed edges; ground light, non-ground dark, outside support gray',
        upstream_revision=REVISION, upstream='https://github.com/aws-robotics/aws-robomaker-small-warehouse-world',
        model_count=len(world.findall('model')), collision_count=len(geometry),
        world_geometry_sha256=world_geometry_sha256(world_file),
        source_sha256={str(p.relative_to(ROOT)): sha256(p) for p in sorted(sources)},
        mesh_loader='installed ignition-common4-graphics MeshManager::Load (offline)',
        mesh_loader_version=subprocess.check_output(['pkg-config', '--modversion', 'ignition-common4-graphics'], text=True).strip(),
        geometry_sha256=sha256(geometry_file), limitations='All-height collision projection, including overhead objects; not inflated clearance or a height-sliced traversability map.')
    Image.fromarray(rasterize(geometry, metadata)).save(image)
    metadata['image_sha256'] = sha256(image)
    (output / METADATA.name).write_text(json.dumps(metadata, indent=2, allow_nan=False)+'\n')
    return metadata


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mesh-exporter', type=Path, required=True)
    parser.add_argument('--output-directory', type=Path, default=DIRECTORY)
    args = parser.parse_args()
    print(json.dumps(build(args.mesh_exporter.resolve(), args.output_directory), indent=2))
