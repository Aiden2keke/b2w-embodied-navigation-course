#!/usr/bin/python3
"""Fetch pinned B2-W weights, deployment configuration and robot assets."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile
import tempfile
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
REVISION = '376d42c9b128f963ab08579762d5a216a976ce39'
ASSET_REVISION = '7dd30bdc7806898950b354260655d5a7f0ce844e'
HASHES = {
    'policy/b2w/base.yaml': 'a883606bdbd5450ed04227b82cde54d9a3fc0eeedb55d61b36bdf43889f8d9f8',
    'policy/b2w/robot_lab/config.yaml': 'd44bb3c36043f7216f7699a23d904fd01b5fc29ab7c35aabe56dc024d3f1cc74',
    'policy/b2w/robot_lab/policy.pt': '38155076408e8eccb22690c6c5be14bd1dcb9149245ca5e493308a9f6ff93b34',
}


def archive(repository, revision, destination):
    tar = destination / 'source.tar.gz'
    urllib.request.urlretrieve(f'https://codeload.github.com/fan-ziqi/{repository}/tar.gz/{revision}', tar)
    with tarfile.open(tar) as package:
        for entry in package.getmembers():
            target = (destination / entry.name).resolve()
            if not target.is_relative_to(destination.resolve()) or entry.issym() or entry.islnk():
                raise ValueError(f'Unsafe archive entry: {entry.name}')
        package.extractall(destination)
    return destination / f'{repository}-{revision}'


def robot_assets(destination):
    # Download only B2-W files, rather than all robots in the model zoo.
    request = urllib.request.Request(
        f'https://api.github.com/repos/fan-ziqi/rl_sar_zoo/git/trees/{ASSET_REVISION}?recursive=1',
        headers={'User-Agent': 'b2w-course-asset-fetcher'})
    with urllib.request.urlopen(request, timeout=60) as response:
        tree = json.load(response)
    if tree.get('truncated'):
        raise ValueError('Incomplete upstream asset tree')
    entries = [item for item in tree['tree']
               if item['type'] == 'blob' and item['path'].startswith('b2w_description/')]
    if not entries:
        raise ValueError('No B2-W assets found in pinned revision')
    def fetch(item):
        target = destination / item['path']
        target.parent.mkdir(parents=True, exist_ok=True)
        url = f'https://raw.githubusercontent.com/fan-ziqi/rl_sar_zoo/{ASSET_REVISION}/{item["path"]}'
        with urllib.request.urlopen(url, timeout=120) as response:
            data = response.read()
        identity = hashlib.sha1(f'blob {len(data)}\0'.encode()+data).hexdigest()
        if identity != item['sha']:
            raise ValueError(f'Asset checksum mismatch: {item["path"]}')
        target.write_bytes(data)
    with ThreadPoolExecutor(max_workers=8) as pool:
        list(pool.map(fetch, entries))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, default=ROOT / 'third_party/b2w_rl_sar')
    parser.add_argument('--export-python', help='Python with torch, onnx and onnxruntime installed')
    args = parser.parse_args()
    bundle = args.bundle.resolve()
    if not bundle.exists():
        bundle.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='b2w-policy-', dir=bundle.parent) as temporary:
            staging = Path(temporary)
            upstream = archive('rl_sar', REVISION, staging)
            # Keep upstream source and license with the exported policy.
            (upstream / 'robot_assets').mkdir(exist_ok=True)
            robot_assets(upstream / 'robot_assets')
            (upstream / 'COLCON_IGNORE').touch()
            metadata = {'repository': 'https://github.com/fan-ziqi/rl_sar', 'revision': REVISION,
                        'license': 'Apache-2.0', 'selected_robot': 'b2w', 'selected_policy': 'robot_lab',
                        'deployment_status': 'downloaded; physical validation required', 'files': {},
                        'robot_assets': {'repository': 'https://github.com/fan-ziqi/rl_sar_zoo',
                                         'revision': ASSET_REVISION, 'version': '1.0.2',
                                         'license_note': 'Preserve upstream attribution; asset license not determined by rl_sar code license.'}}
            for name, expected in HASHES.items():
                data = (upstream / name).read_bytes()
                if hashlib.sha256(data).hexdigest() != expected:
                    raise ValueError(f'Checksum mismatch: {name}')
                metadata['files'][name] = {'bytes': len(data), 'sha256': expected}
            (upstream / 'POLICY_SOURCE.json').write_text(json.dumps(metadata, indent=2)+'\n')
            upstream.rename(bundle)
    metadata = json.loads((bundle / 'POLICY_SOURCE.json').read_text())
    if metadata['revision'] != REVISION or metadata['robot_assets']['revision'] != ASSET_REVISION:
        raise ValueError('Existing bundle revision mismatch; refusing to overwrite it')
    for name, expected in HASHES.items():
        if hashlib.sha256((bundle / name).read_bytes()).hexdigest() != expected:
            raise ValueError(f'Checksum mismatch: {name}')
    if not (bundle / 'robot_assets/b2w_description/urdf/b2w_description.urdf').is_file():
        raise ValueError('Missing B2-W robot assets')
    if args.export_python:
        subprocess.run([args.export_python, str(ROOT / 'scripts/export_rl_sar.py'), str(bundle)], check=True)
    elif not (bundle / 'policy/b2w/robot_lab/policy.onnx').is_file():
        raise SystemExit('Downloaded successfully. Run again with --export-python; see docs/RL_SAR_SETUP.md.')
    else:
        data = (bundle / 'policy/b2w/robot_lab/policy.onnx').read_bytes()
        if hashlib.sha256(data).hexdigest() != metadata['files']['policy/b2w/robot_lab/policy.onnx']['sha256']:
            raise ValueError('Exported ONNX checksum mismatch')
    print(f'B2-W bundle verified: {bundle}')


if __name__ == '__main__':
    main()
