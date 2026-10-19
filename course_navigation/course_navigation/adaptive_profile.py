"""Opt-in Humble Smac2D + RPP profile derived from unchanged NavFn + DWB YAML."""
from copy import deepcopy

SPEED_TOPIC = '/course_navigation/speed_limit'
RPP = {
    'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
    'desired_linear_vel': .43,
    'lookahead_dist': .6,
    'min_lookahead_dist': .4,
    'max_lookahead_dist': .9,
    'lookahead_time': 1.5,
    'use_velocity_scaled_lookahead_dist': True,
    'transform_tolerance': .5,
    'min_approach_linear_velocity': .03,
    'approach_velocity_scaling_dist': .6,
    'use_collision_detection': True,
    'max_allowed_time_to_collision_up_to_carrot': 1.5,
    'use_regulated_linear_velocity_scaling': True,
    'use_cost_regulated_linear_velocity_scaling': True,
    'cost_scaling_dist': .6,
    'cost_scaling_gain': 1.,
    'inflation_cost_scaling_factor': 3.,
    'regulated_linear_scaling_min_radius': 1.25,
    'regulated_linear_scaling_min_speed': .06,
    'use_rotate_to_heading': True,
    'rotate_to_heading_min_angle': .6,
    'rotate_to_heading_angular_vel': .6,
    'max_angular_accel': .6,
    'allow_reversing': False,
    'use_interpolation': True,
}


def build_profile(baseline, config=None):
    """Select Smac2D/RPP and the opt-in smoother ceiling; preserve costmaps."""
    result = deepcopy(baseline)
    planner = result['planner_server']['ros__parameters']['GridBased']
    planner['plugin'] = 'nav2_smac_planner/SmacPlanner2D'
    planner.pop('use_astar', None)  # NavFn-only switch; Smac2D uses its own search.
    controller = result['controller_server']['ros__parameters']
    controller['FollowPath'] = deepcopy(RPP)
    controller['speed_limit_topic'] = SPEED_TOPIC
    if config is not None:
        controller['FollowPath']['desired_linear_vel'] = config.open_command
    result['velocity_smoother']['ros__parameters']['max_velocity'][0] = controller['FollowPath']['desired_linear_vel']
    return result
