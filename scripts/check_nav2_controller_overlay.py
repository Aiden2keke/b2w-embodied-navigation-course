#!/usr/bin/python3
"""Offline launch/ELF resolution proof. Run with with_nav2_controller_overlay.sh."""
import os
import ctypes
import hashlib
import json
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_prefix
from launch import LaunchContext
from launch_ros.substitutions import ExecutableInPackage

root = Path(__file__).resolve().parents[1]
prefix = Path(os.environ.get('B2W_CONTROLLER_PREFIX', str(root / '.deps/nav2-controller-1.1.20/install/nav2_controller'))).resolve()
assert Path(get_package_prefix('nav2_controller')) == prefix
executable = Path(ExecutableInPackage(
    package='nav2_controller', executable='controller_server').perform(LaunchContext()))
assert executable == prefix / 'lib/nav2_controller/controller_server'
library = prefix / 'lib/libcontroller_server_core.so'
# Use the executable's actual interpreter; --list loads no ROS nodes/main().
elf = subprocess.check_output(['readelf', '-l', str(executable)], text=True)
interpreter = elf.split('Requesting program interpreter: ', 1)[1].split(']', 1)[0]
loader = subprocess.check_output([interpreter, '--list', str(executable)], text=True)
assert f'libcontroller_server_core.so => {library}' in loader, loader
assert 'not found' not in loader
# RTLD_NOW also checks symbol resolution and proves the library mapped in a process.
ctypes.CDLL(str(library), mode=2)
maps = Path('/proc/self/maps').read_text()
assert str(library) in maps
assert '/opt/ros/humble/lib/libcontroller_server_core.so' not in maps

# Public headers and plugin metadata must be byte-identical to this underlay.
headers = list((prefix / 'include/nav2_controller').rglob('*.hpp'))
for header in headers:
    assert header.read_bytes() == (Path('/opt/ros/humble') / header.relative_to(prefix)).read_bytes()
for name in ['package.xml', 'plugins.xml']:
    assert (prefix / 'share/nav2_controller' / name).read_bytes() == (
        Path('/opt/ros/humble/share/nav2_controller') / name).read_bytes()

def dynamic_contract(path):
    output = subprocess.check_output(['readelf', '-d', str(path)], text=True)
    return sorted(line.split('[', 1)[1].split(']', 1)[0]
                  for line in output.splitlines() if '(NEEDED)' in line or '(SONAME)' in line)


def symbols(path):
    output = subprocess.check_output(['nm', '-D', '--defined-only', str(path)], text=True)
    # Compare strong public functions/data and RTTI/vtables belonging to this package.
    # Weak STL/template/compiler instantiations are not the package's ABI contract.
    return {line.split()[-1] for line in output.splitlines()
            if line.split()[1] in {'T', 'D', 'B', 'V'}
            and (line.split()[-1].startswith('_ZN15nav2_controller')
                 or line.split()[-1].startswith('_ZNK15nav2_controller')
                 or line.split()[-1].startswith(('_ZTIN15nav2_controller', '_ZTVN15nav2_controller',
                                                '_ZTSN15nav2_controller')))}
abi = {}
for path in sorted((prefix / 'lib').glob('*.so')):
    original = Path('/opt/ros/humble/lib') / path.name
    assert dynamic_contract(original) == dynamic_contract(path), path.name
    old, new = symbols(original), symbols(path)
    assert old <= new, (path.name, sorted(old-new))
    result = subprocess.check_output(['ldd', '-r', str(path)], text=True, stderr=subprocess.STDOUT)
    assert 'not found' not in result and 'undefined symbol' not in result, result
    abi[path.name] = {'preserved_public_symbols': len(old), 'additional_symbols': sorted(new-old),
                      'relocations': 'resolved', 'SONAME_and_NEEDED': dynamic_contract(path)}
report = {'package_prefix': str(prefix), 'launch_executable': str(executable),
          'loader_resolution': loader, 'mapped_core': str(library),
          'sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in [executable, library]},
          'matching_headers': len(headers), 'package_and_plugins_xml': 'identical', 'ABI': abi}
print(json.dumps(report, indent=2))
