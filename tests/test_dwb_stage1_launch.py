"""Evaluate real launch descriptions and ROS parameter files without executing nodes."""
import importlib.util
from functools import lru_cache
import json
from pathlib import Path
import subprocess
import tempfile

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

ROOT=Path(__file__).resolve().parents[1]
CONFIG=ROOT/'course_navigation/config'


@lru_cache(None)
def probe_binary():
    source=ROOT/'scripts/nav2_stage1_probe'
    assert (source/'probe.cpp').is_file(), 'installed-library parameter/plugin probe is missing'
    build=Path(tempfile.mkdtemp(prefix='b2w-stage1-probe-'))
    subprocess.run(['cmake','-S',str(source),'-B',str(build),'-DCMAKE_PREFIX_PATH=/opt/ros/humble'],
                   check=True,capture_output=True)
    subprocess.run(['cmake','--build',str(build),'-j2'],check=True,capture_output=True)
    return build/'nav2_stage1_probe'


def probe(*args):
    result=subprocess.run([str(probe_binary()),*map(str,args)],check=True,text=True,capture_output=True)
    return json.loads(result.stdout.splitlines()[-1])


def flatten(value,prefix=''):
    result={}
    for key,v in value.items():
        name=prefix+key
        if isinstance(v,dict):result.update(flatten(v,name+'.'))
        else:result[name]=v
    return result


def load(path):
    spec=importlib.util.spec_from_file_location('stage1_launch',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def context_for(module, **overrides):
    context=LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action,DeclareLaunchArgument):
            context.launch_configurations[action.name]=perform_substitutions(context,action.default_value)
    context.launch_configurations.update(overrides)
    return context


def expand(**overrides):
    module=load(ROOT/'course_navigation/launch/navigation.launch.py')
    context=context_for(module, **overrides)
    effective={};nodes=module.setup(context)
    for node in nodes:
        if not isinstance(node,Node):continue
        executable=node._Node__node_executable
        node._perform_substitutions(context)  # Only expansion; never Node.execute().
        files=[path for path,is_file in node._Node__expanded_parameter_arguments if is_file]
        effective[executable]=probe('--parameters',executable,*files)
    return effective,context,nodes


def test_original_smac_and_independent_footprint_profile_remain_isolated():
    baseline=yaml.safe_load((CONFIG/'nav2.yaml').read_text())
    smac=yaml.safe_load((CONFIG/'nav2_smac2d_diag.yaml').read_text())
    gate=yaml.safe_load((CONFIG/'nav2_smac2d_footprint_gate_diag.yaml').read_text())
    expected=yaml.safe_load((CONFIG/'nav2.yaml').read_text())
    expected['planner_server']['ros__parameters']['GridBased']['plugin']='nav2_smac_planner/SmacPlanner2D'
    assert smac==expected
    assert smac['controller_server']==baseline['controller_server']
    assert 'ObstacleFootprint' not in smac['controller_server']['ros__parameters']['FollowPath']['critics']
    assert 'ObstacleFootprint' in gate['controller_server']['ros__parameters']['FollowPath']['critics']


@pytest.mark.parametrize('filename',['nav2.yaml','nav2_smac2d_diag.yaml','nav2_smac2d_footprint_gate_diag.yaml'])
def test_actual_expansion_changes_only_three_observability_parameters(filename):
    raw=yaml.safe_load((CONFIG/filename).read_text())
    plain,_,_=expand(nav_params_file=str(CONFIG/filename))
    effective,_,_=expand(nav_params_file=str(CONFIG/filename),dwb_observability='true')
    assert plain['controller_server']==flatten(raw['controller_server']['ros__parameters'])
    expected=yaml.safe_load(yaml.safe_dump(plain))
    expected['controller_server'].update({'FollowPath.debug_trajectory_details':True,
        'FollowPath.publish_evaluation':True,'FollowPath.publish_trajectories':False})
    assert effective==expected
    assert effective['controller_server']['FollowPath.plugin']=='dwb_core::DWBLocalPlanner'
    assert 'speed_governor' not in effective
    assert 'speed_limit_topic' not in effective['velocity_adapter']


def test_observability_rejects_rpp_and_invalid_flags():
    with pytest.raises(ValueError):expand(adaptive_speed='true',w_ref='1.0',dwb_observability='true')
    with pytest.raises(ValueError):expand(dwb_observability='sometimes')


def test_bringup_forwards_flag_and_reversible_profile_without_launching():
    module=load(ROOT/'course_bringup/launch/course.launch.py')
    context=context_for(module,mode='navigation',scene='warehouse',dwb_observability='true',
                        nav_params_file=str(CONFIG/'nav2_smac2d_diag.yaml'))
    includes=[a for a in module.setup(context) if isinstance(a,IncludeLaunchDescription)]
    args=dict(includes[-1].launch_arguments)
    assert args['dwb_observability']=='true'
    assert args['nav_params_file']==str(CONFIG/'nav2_smac2d_diag.yaml')
    assert args['adaptive_speed']=='false'
    assert args['map_width']==args['map_height']=='40.0'


def test_installed_plugins_load_without_ros_init():
    result=probe('--plugins')
    assert result['planners']==['nav2_navfn_planner/NavfnPlanner','nav2_smac_planner/SmacPlanner2D']
    assert result['controller']=='dwb_core::DWBLocalPlanner'
    assert result['footprint_critic']=='dwb_critics::ObstacleFootprintCritic'
    assert result['rejection_reason']=='Trajectory Hits Obstacle.'
