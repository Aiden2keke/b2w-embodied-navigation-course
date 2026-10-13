"""Deterministic box fixtures and geometry derived from the unchanged Nav2 YAML."""
import ast
import copy
import math
import xml.etree.ElementTree as ET
import yaml


def geometry(config):
    params = yaml.safe_load(config.read_text())
    layers = [params[k][k]['ros__parameters'] for k in ('local_costmap', 'global_costmap')]
    keys = ('footprint', 'footprint_padding', 'resolution', 'inflation_layer')
    if any(layers[0][k] != layers[1][k] for k in keys):
        raise ValueError('Different local/global geometry requires a separate width derivation')
    p = layers[0]
    points = ast.literal_eval(p['footprint'])
    # Nav2 padFootprint adds padding to each signed coordinate (not rounded corners).
    padded = [(x + math.copysign(p['footprint_padding'], x),
               y + math.copysign(p['footprint_padding'], y)) for x, y in points]
    length = max(x for x,y in padded) - min(x for x,y in padded)
    width = max(y for x,y in padded) - min(y for x,y in padded)
    r = p['resolution']; inflation = p['inflation_layer']['inflation_radius']
    def snap(v): return round(round(v/r)*r, 6)
    # Probe just below/above inscribed threshold, then 2/6 cells of margin,
    # inflation overlap cessation and one-cell margin on either wall.
    widths = sorted(set([round(math.floor(width/r)*r-r,6), round(math.floor(width/r)*r,6),
                         round(math.ceil(width/r)*r,6), snap(width+3*r), snap(width+7*r),
                         snap(2*inflation), snap(2*inflation+2*r)]))
    return dict(footprint=points, padding_m=p['footprint_padding'], padded_length_m=length,
                padded_width_m=width, inscribed_radius_m=min(length,width)/2,
                circumscribed_radius_m=math.hypot(length/2,width/2), resolution_m=r,
                inflation_radius_m=inflation, cost_scaling_factor=p['inflation_layer']['cost_scaling_factor'],
                widths_m=widths, wide_route_width_m=round(max(length, 2*inflation)+4*r,6))


def fixture(root, directory, scenario, width, g):
    if scenario not in ('A','B') or not math.isfinite(width) or not .2 < width < 3:
        raise ValueError('Scenario A/B and finite width in (0.2,3) required')
    # Two metre choke length, six metre straight start-goal separation;
    # enclosed room prevents any external bypass. Same start/goal for A and B.
    ymin = -width/2 - 2.
    island_top = width/2 + 1.5
    ymax = island_top + g['wide_route_width_m']
    boxes = []
    def box(name, xmin, xmax, lo, hi):
        boxes.append(dict(id=name, xmin=xmin, xmax=xmax, ymin=lo, ymax=hi))
    box('lower',2,4,ymin,-width/2)
    box('upper',2,4,width/2, island_top if scenario=='A' else ymax)
    box('south',-2.2,8.2,ymin-.2,ymin)
    box('north',-2.2,8.2,ymax,ymax+.2)
    box('west',-2.2,-2,ymin,ymax)
    box('east',8,8.2,ymin,ymax)
    template = ET.parse(root/'course_sim/worlds/confined.world').getroot().find('world')
    sdf = ET.Element('sdf', version='1.8'); world = ET.SubElement(sdf,'world',name='course_world')
    for child in template:
        if child.tag in ('physics','plugin','light') or (child.tag=='model' and child.get('name')=='floor'):
            world.append(copy.deepcopy(child))
    model_template = template.find("model[@name='north']")
    for b in boxes:
        model = copy.deepcopy(model_template); model.set('name',b['id'])
        model.find('pose').text = f"{(b['xmin']+b['xmax'])/2} {(b['ymin']+b['ymax'])/2} 1 0 0 0"
        for size in model.findall('.//box/size'):
            size.text = f"{b['xmax']-b['xmin']} {b['ymax']-b['ymin']} 2"
        world.append(model)
    ET.indent(sdf); ET.ElementTree(sdf).write(directory/'fixture.world',encoding='utf-8',xml_declaration=True)
    return dict(scenario_id=scenario, corridor_width_m=width, bounds=[-2,8,ymin,ymax],
                boxes=boxes, start_world_xy=[0,0], goal_relative_start_xy=[6,0],
                wide_route_width_m=g['wide_route_width_m'], gate_x_m=3., island_top_m=island_top)


def chosen_route(points, width):
    crossings = []
    for (x,y),(xx,yy) in zip(points,points[1:]):
        if min(x,xx) <= 3 <= max(x,xx) and x != xx:
            crossings.append(y+(yy-y)*(3-x)/(xx-x))
    labels = {'narrow' if abs(y)<width/2 else 'wide' if y>width/2+1.5 else 'unclassified'
              for y in crossings}
    return next(iter(labels)) if len(labels)==1 else 'unclassified'


def classify_failure(reason, generated, controller_abort_observed, collisions):
    # Status arrays can retain unrelated/recovered ABORTED goals. Without a
    # navigation -> FollowPath UUID link AND terminal cause, this is evidence
    # only; it cannot establish primary controller failure.
    if collisions: return 'collision'
    if reason.startswith('planner_'): return 'planning_failure'
    if reason=='succeeded': return 'none'
    return {'timeout':'navigation_timeout', 'startup_timeout':'startup_not_ready',
            'runtime_exit':'runtime_exit', 'localization_stale':'localization_stale',
            'aborted':'navigation_abort_unattributed', 'canceled':'navigation_canceled',
            'navigation_rejected':'navigation_rejected', 'clock_reset':'clock_reset',
            'interrupted':'interrupted', 'goal_error_or_missing_pose':'goal_error_or_missing_pose',
            'startup_not_active':'startup_not_ready'}.get(reason, 'harness_error')
