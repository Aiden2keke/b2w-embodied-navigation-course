#!/usr/bin/python3
"""Fetch pinned MIT-0 warehouse assets and generate a local Fortress world."""
import copy
import json
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REVISION = 'ee0af733315e78432408c3cd98d378ecee5f767c'
URL = 'https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git'


def main():
    assets = ROOT / 'third_party/aws_warehouse'
    if not (assets / 'REVISION').exists():
        if assets.exists():
            raise RuntimeError('Asset directory exists without a revision; preserve it and inspect manually')
        with tempfile.TemporaryDirectory(prefix='course-warehouse-') as tmp:
            checkout = Path(tmp) / 'upstream'
            subprocess.run(['git', 'clone', '--branch', 'ros2', '--single-branch', URL, str(checkout)], check=True)
            subprocess.run(['git', '-C', str(checkout), 'checkout', REVISION], check=True)
            assets.mkdir(parents=True)
            for name in ['models', 'worlds']:
                shutil.copytree(checkout / name, assets / name)
            shutil.copy2(checkout / 'LICENSE', assets / 'LICENSE')
            (assets / 'REVISION').write_text(REVISION + '\n')
            (assets / 'COLCON_IGNORE').touch()
    if (assets / 'REVISION').read_text().strip() != REVISION:
        raise RuntimeError('Warehouse asset revision mismatch')
    upstream = ET.parse(assets / 'worlds/no_roof_small_warehouse/no_roof_small_warehouse.world').getroot().find('world')
    template = ET.parse(ROOT / 'course_sim/worlds/confined.world').getroot().find('world')
    sdf = ET.Element('sdf', version='1.8')
    world = ET.SubElement(sdf, 'world', name='course_world')
    for node in template:
        if node.tag in ['physics', 'plugin', 'light']:
            world.append(copy.deepcopy(node))
    # Preserve upstream geometry and poses; flatten Classic nested includes.
    for wrapper in upstream.findall('model'):
        uri = wrapper.findtext('include/uri')
        if not uri or not uri.startswith('model://'):
            raise ValueError(f'Unsupported upstream model: {wrapper.attrib}')
        model = ET.parse(assets / 'models' / uri[8:] / 'model.sdf').getroot().find('model')
        if model.findtext('static') not in ['1', 'true']:
            raise ValueError('This asset adapter supports static scenery only')
        # Classic assets contain nonphysical inertia tensors; static scenery needs none.
        for link in model.findall('link'):
            for inertial in list(link.findall('inertial')):
                link.remove(inertial)
        if uri[8:] == 'aws_robomaker_warehouse_GroundB_01':
            # Use the existing course floor friction for the B2-W locomotion policy.
            for friction in model.findall('.//collision/surface/friction/ode'):
                friction.find('mu').text = '1.0'
                friction.find('mu2').text = '1.0'
        model.set('name', wrapper.get('name'))
        for old_pose in list(model.findall('pose')):
            model.remove(old_pose)
        ET.SubElement(model, 'pose').text = wrapper.findtext('pose', '0 0 0 0 0 0')
        for resource in model.findall('.//uri'):
            prefix = 'file://models/'
            if not resource.text.startswith(prefix):
                raise ValueError(f'Unsupported asset URI: {resource.text}')
            file = assets / 'models' / resource.text[len(prefix):]
            if not file.is_file():
                raise FileNotFoundError(file)
            resource.text = file.as_uri()
        world.append(model)
    ET.indent(sdf)
    destination = assets / 'warehouse.world'
    ET.ElementTree(sdf).write(destination, encoding='utf-8', xml_declaration=True)
    (assets / 'adaptation.json').write_text(json.dumps({
        'upstream': URL, 'revision': REVISION, 'license': 'MIT-0',
        'variant': 'no_roof_small_warehouse', 'models': len(world.findall('model')),
        'changes': ['Fortress systems and 400 Hz physics', 'flatten includes', 'absolute file URIs',
                    'remove unused invalid static-body inertias'],
        'ground_friction_mu': 1.0,
        'spawn': [0., 0., 1.], 'map_size_m': [40., 40.],
    }, indent=2) + '\n')
    print(destination)


if __name__ == '__main__':
    main()
