#!/usr/bin/python3
"""Record measured ground-truth poses and export registered Warehouse maps."""
import argparse
import csv
import hashlib
import json
import math
import signal
import shutil
import time
from datetime import datetime, timezone
from pathlib import Path
import warehouse_truth_floorplan as floor

ROOT = floor.ROOT
SAVED_MAP_DIR = ROOT / 'experiments/g1_g5/waypoints/20261019-v1'
SAVED_PGM = SAVED_MAP_DIR / 'warehouse.pgm'
SAVED_YAML = SAVED_MAP_DIR / 'warehouse.yaml'
SAVED_REGISTRATION = ROOT / 'course_navigation/maps/warehouse_saved_map_registration.json'
MOTION_DISPLACEMENT_THRESHOLD_M = 0.15


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_fixed_map(world):
    metadata, image = floor.load_floorplan()
    if floor.world_geometry_sha256(world) != metadata['world_geometry_sha256']:
        raise ValueError('Warehouse world geometry does not match fixed map')
    for name, digest in metadata['source_sha256'].items():
        if name.startswith('third_party/aws_warehouse/models/') and floor.sha256(floor.ROOT / name) != digest:
            raise ValueError('Warehouse world model differs from fixed map: ' + name)
    return metadata, image


def _segments(rows):
    result = []
    for row in rows:
        if (not result or row['epoch'] != result[-1][-1]['epoch'] or
                row['segment'] != result[-1][-1]['segment']):
            result.append([])
        result[-1].append(row)
    return result


def _render_title_and_legend(fig, ax, title, subtitle, map_label, stationary):
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    fig.suptitle(title, x=.5, y=.965, fontsize=17, weight='semibold', color='#243447')
    ax.set_title(subtitle, fontsize=9, color='#586779', pad=5)
    handles = [Patch(facecolor='#77889a', edgecolor='#34495e', label=map_label)]
    if stationary:
        handles.append(Line2D([0], [0], color='#768390', lw=1.2,
                              label='Stationary session · no travel line'))
    else:
        handles.append(Line2D([0], [0], color='#1677ff', lw=2.2, label='Measured ground truth'))
    handles.extend([
        Line2D([0], [0], marker='o', linestyle='None', markerfacecolor='#22a447',
               markeredgecolor='white', markersize=7, label='Start'),
        Line2D([0], [0], marker='o', linestyle='None', markerfacecolor='#ff8a24',
               markeredgecolor='white', markersize=7, label='End'),
    ])
    ax.legend(handles=handles, loc='lower center', bbox_to_anchor=(.5, -.11),
              ncol=len(handles), frameon=True, framealpha=.96, fontsize=8,
              handlelength=1.7, columnspacing=1.2)


def _draw_rows(ax, rows, stationary):
    if not rows:
        return
    if not stationary:
        for segment in _segments(rows):
            if len(segment) > 1:
                ax.plot([r['x'] for r in segment], [r['y'] for r in segment],
                        color='#1677ff', lw=1.8, zorder=4, solid_capstyle='round')
    # Markers are centered on the measured samples. Small markers retain useful
    # positional meaning when a standing robot's start/end samples nearly overlap.
    ax.scatter(rows[0]['x'], rows[0]['y'], marker='o', s=10, color='#22a447',
               edgecolor='white', linewidth=.45, zorder=6)
    ax.scatter(rows[-1]['x'], rows[-1]['y'], marker='D', s=10, color='#ff8a24',
               edgecolor='white', linewidth=.45, zorder=7)
    if stationary and math.hypot(rows[-1]['x']-rows[0]['x'],rows[-1]['y']-rows[0]['y']) < .25:
        ax.annotate('START',(rows[0]['x'],rows[0]['y']),xytext=(-30,14),textcoords='offset points',
            color='#187d37',fontsize=7,weight='bold',
            bbox=dict(boxstyle='round,pad=.15',facecolor='white',edgecolor='#22a447',alpha=.94),zorder=8)
        ax.annotate('END',(rows[-1]['x'],rows[-1]['y']),xytext=(13,-18),textcoords='offset points',
            color='#b85e12',fontsize=7,weight='bold',
            bbox=dict(boxstyle='round,pad=.15',facecolor='white',edgecolor='#ff8a24',alpha=.94),zorder=8)


def _styled_world_raster(background):
    import numpy as np
    from scipy.ndimage import binary_erosion

    # Recolor only the canonical raster's existing support, outside and obstacle
    # classes. The pixel grid and geometry are unchanged.
    free = np.all(background >= 240, axis=2)
    outside = np.all((background >= 195) & (background <= 215), axis=2)
    obstacle = ~(free | outside)
    styled = np.empty_like(background)
    styled[:] = (230, 233, 235)       # outside/support boundary
    styled[free] = (247, 244, 235)    # warm warehouse floor
    styled[obstacle] = (91, 111, 132)  # Gazebo-inspired muted slate structures
    edge = obstacle & ~binary_erosion(obstacle, structure=np.ones((3, 3)), border_value=0)
    styled[edge] = (42, 57, 73)      # crisp darker obstacle outlines
    return styled


def render_fixed_track(path, rows, metadata, background, stationary=False):
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(10, 8), dpi=260)
    styled = _styled_world_raster(background)
    ax.imshow(styled, extent=[metadata['xmin'], metadata['xmax'], metadata['ymin'], metadata['ymax']],
              origin='upper', interpolation='nearest')
    ax.set_xlim(metadata['xmin'], metadata['xmax']); ax.set_ylim(metadata['ymin'], metadata['ymax'])
    ax.set_aspect('equal'); ax.axis('off')
    _draw_rows(ax, rows, stationary)
    _render_title_and_legend(fig, ax, 'B2-W Warehouse',
        'Ground-truth map · Gazebo-inspired geometry · world frame · 0.02 m/pixel',
        'Ground-truth geometry', stationary)
    fig.subplots_adjust(left=.025, right=.975, top=.89, bottom=.16)
    fig.savefig(path, dpi=260, bbox_inches='tight', pad_inches=.08, facecolor='white')
    plt.close(fig)


def load_saved_registration(path=SAVED_REGISTRATION):
    path=Path(path)
    if not path.is_file(): return None, 'No accepted paired-landmark calibration exists.'
    value=json.loads(path.read_text())
    expected=dict(world_geometry=sha256(floor.IMAGE),
        world_metadata=sha256(floor.METADATA),
        saved_pgm=sha256(SAVED_PGM),saved_yaml=sha256(SAVED_YAML))
    if (value.get('kind') != 'warehouse_saved_map_registration' or value.get('version') != 1 or
            value.get('accepted') is not True or value.get('image_hashes')!=expected):
        return None, 'Registration rejected: calibration or map asset identity mismatch.'
    transform=value.get('transform',{})
    conventions=value.get('coordinate_conventions',{})
    if (transform.get('source_frame')!='world' or transform.get('target_frame')!='saved_slam_map' or
            conventions.get('world_frame')!='world' or conventions.get('saved_frame')!='saved_slam_map' or
            conventions.get('saved_image_row0')!='highest map y; image origin upper'):
        return None, 'Registration rejected: coordinate frame contract mismatch.'
    numeric=[transform.get(k) for k in ('x_m','y_m','yaw_rad')]
    errors=value.get('residuals_m',[])
    if (value.get('fit_pair_count',0)<3 or value.get('holdout_pair_count')!=1
            or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in numeric)
            or value.get('fit_max_error_m',math.inf)>.15 or value.get('holdout_error_m',math.inf)>.15
            or not errors or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in errors)):
        return None, 'Registration rejected: fit/holdout validation fields are incomplete or exceed limits.'
    return value,None


def _read_truth_csv(path):
    required={'timestamp_ns','x','y','yaw_rad','segment','epoch','source'}
    rows=[]
    with Path(path).open(newline='') as stream:
        reader=csv.DictReader(stream)
        if not required.issubset(reader.fieldnames or ()):
            raise ValueError('Recorded trajectory CSV is missing required ground-truth columns')
        for raw in reader:
            row=dict(timestamp_ns=int(raw['timestamp_ns']),x=float(raw['x']),y=float(raw['y']),
                     yaw_rad=float(raw['yaw_rad']),segment=int(raw['segment']),epoch=int(raw['epoch']),
                     source=raw['source'])
            if (row['timestamp_ns'] <= 0 or not all(math.isfinite(row[k]) for k in ('x','y','yaw_rad')) or
                    row['source'] != '/sim/ground_truth/odom'):
                raise ValueError('Recorded trajectory contains an invalid or non-ground-truth pose')
            if rows and row['timestamp_ns'] <= rows[-1]['timestamp_ns'] and row['epoch']==rows[-1]['epoch']:
                raise ValueError('Recorded ground-truth timestamps are not increasing within an epoch')
            rows.append(row)
    if not rows:
        raise ValueError('Recorded ground-truth trajectory is empty')
    return rows


def _map_xy_to_pixel(xy, origin, resolution, width, height):
    ox,oy,_=origin
    return [((x-ox)/resolution-.5,
             (oy+height*resolution-y)/resolution-.5) for x,y in xy]


def export_map_images(output, rows, metadata, background, registration, stationary):
    """Render the legacy floorplan and both presentation maps from the same poses."""
    if registration is None:
        raise ValueError('Dual-map export requires the accepted saved-map registration')
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    render_fixed_track(output/'trajectory.png',rows,metadata,background,stationary=stationary)
    render_fixed_track(output/'trajectory_ground_truth.png',rows,metadata,background,stationary=stationary)
    render_saved_slam_track(output/'trajectory_slam.png',rows,registration,stationary=stationary)
    return transform_truth_rows_to_saved_map(rows,registration)


def has_measured_motion(rows, threshold_m=MOTION_DISPLACEMENT_THRESHOLD_M):
    """Classify motion from actual ground-truth displacement, ignoring small pose noise."""
    if len(rows) < 2 or not math.isfinite(threshold_m) or threshold_m <= 0:
        return False
    try:
        x0, y0 = float(rows[0]['x']), float(rows[0]['y'])
        if not math.isfinite(x0) or not math.isfinite(y0):
            return False
        return any(math.isfinite(float(row['x'])) and math.isfinite(float(row['y'])) and
                   math.hypot(float(row['x']) - x0, float(row['y']) - y0) >= threshold_m
                   for row in rows[1:])
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def export_recorded_session(session_directory, output_directory):
    """Offline-render an existing capture without modifying its evidence."""
    session=Path(session_directory).resolve();output=Path(output_directory).resolve()
    csv_source=session/'trajectory.csv';result_source=session/'result.json'
    if not csv_source.is_file() or not result_source.is_file():
        raise ValueError('Recorded session must contain trajectory.csv and result.json')
    source_result=json.loads(result_source.read_text())
    if source_result.get('trajectory_source')!='/sim/ground_truth/odom':
        raise ValueError('Recorded session result does not identify ground-truth odometry')
    source_gt=source_result.get('images',{}).get('ground_truth',{})
    source_frame=source_gt.get('source_frame')
    if source_frame is None and 'sim_world' in str(source_gt.get('coordinate_frame','')):
        source_frame='sim_world'
    if source_frame!='sim_world':
        raise ValueError('Recorded result does not identify the /sim/ground_truth/odom sim_world frame')
    rows=_read_truth_csv(csv_source)
    if source_result.get('trajectory_samples') != len(rows):
        raise ValueError('Recorded result sample count does not match trajectory.csv')
    registration,error=load_saved_registration()
    if registration is None:
        raise ValueError('Accepted saved-map registration is unavailable: '+str(error))
    metadata,background=load_fixed_map(floor.ROOT/'third_party/aws_warehouse/warehouse.world')
    output.mkdir(parents=True,exist_ok=True)
    targets=['trajectory.csv','trajectory.png','trajectory_ground_truth.png','trajectory_slam.png','result.json']
    existing=[name for name in targets if (output/name).exists()]
    if existing:
        raise FileExistsError('Refusing to overwrite preview artifacts: '+', '.join(existing))
    shutil.copy2(csv_source,output/'trajectory.csv')
    stationary=source_result.get('motion_enabled') is False
    saved_xy=export_map_images(output,rows,metadata,background,registration,stationary)
    image,saved_yaml,crop=load_saved_slam_map(registration)
    origin=saved_yaml['origin'];resolution=float(saved_yaml['resolution']);height,width=image.shape
    start_world=[rows[0]['x'],rows[0]['y']];end_world=[rows[-1]['x'],rows[-1]['y']]
    start_saved,end_saved=saved_xy[0],saved_xy[-1]
    start_world_pixel,end_world_pixel=floor.world_to_pixel([start_world,end_world],metadata).tolist()
    start_pixel,end_pixel=_map_xy_to_pixel([start_saved,end_saved],origin,resolution,width,height)
    start_display=[start_pixel[0]-crop['col0'],start_pixel[1]-crop['row0']]
    end_display=[end_pixel[0]-crop['col0'],end_pixel[1]-crop['row0']]
    reg_path=Path(SAVED_REGISTRATION).resolve()
    report=dict(kind='manual_warehouse_map_export',outcome='offline_preview',success=True,
        motion_enabled=source_result.get('motion_enabled'),
        motion_classification='stationary' if stationary else 'measured_motion',
        motion_classification_reason=('source session records motion_enabled=false; endpoint/balance variation is not drawn as travel'
            if stationary else 'source session records motion_enabled=true'),
        trajectory_samples=len(rows),trajectory_source='/sim/ground_truth/odom',
        trajectory_source_frame=source_frame,trajectory_frame='world',
        source_to_world_relation='sim_world and world are identity-aligned by the SDF ground-truth odometry publisher',
        source_session=str(session),source_csv_sha256=hashlib.sha256(csv_source.read_bytes()).hexdigest(),
        artifacts={name:str(output/name) for name in targets},
        images={
            'ground_truth':dict(path=str(output/'trajectory_ground_truth.png'),source_topic='/sim/ground_truth/odom',
                source_frame=source_frame,coordinate_frame='world',map_asset=str(floor.IMAGE),map_sha256=metadata['image_sha256'],
                resolution_m_per_pixel=metadata['resolution'],pixel_contract=metadata['pixel_contract'],
                style='Gazebo-inspired colors applied to canonical collision geometry',stationary=stationary,
                trajectory_drawn=not stationary,start_world_xy=start_world,end_world_xy=end_world,
                start_image_pixel_xy=start_world_pixel,end_image_pixel_xy=end_world_pixel),
            'slam':dict(path=str(output/'trajectory_slam.png'),source_topic='/sim/ground_truth/odom',
                trajectory_source_frame=source_frame,trajectory_frame='world',
                trajectory_label='ground truth; not SLAM localization',
                coordinate_frame='saved_slam_map',map_asset=str(SAVED_PGM),
                available=True,trajectory_drawn=not stationary,
                map_sha256=sha256(SAVED_PGM),yaml_asset=str(SAVED_YAML),
                yaml_sha256=sha256(SAVED_YAML),resolution_m_per_pixel=resolution,
                origin=list(origin),registration_path=str(reg_path),
                registration_sha256=hashlib.sha256(reg_path.read_bytes()).hexdigest(),
                registration=registration['transform'],registration_fit_rmse_m=registration['fit_rmse_m'],
                registration_holdout_error_m=registration['holdout_error_m'],start_world_xy=start_world,
                end_world_xy=end_world,start_saved_map_xy=start_saved,end_saved_map_xy=end_saved,
                pgm_size_px=[width,height],
                pixel_contract='top-left pixel center: x=origin_x+(column+0.5)*resolution; y=origin_y+(height-row-0.5)*resolution',
                crop_world_extent_m=[origin[0]+crop['col0']*resolution,
                    origin[1]+(height-crop['row1'])*resolution,
                    origin[0]+crop['col1']*resolution,
                    origin[1]+(height-crop['row0'])*resolution],
                start_image_pixel_xy=start_pixel,end_image_pixel_xy=end_pixel,stationary=stationary,
                display_crop_pixels=crop,start_display_pixel_xy=start_display,
                end_display_pixel_xy=end_display)})
    (output/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return dict(trajectory_samples=len(rows),motion_classification=report['motion_classification'],
        start_world_xy=start_world,end_world_xy=end_world,start_saved_map_xy=start_saved,
        end_saved_map_xy=end_saved,start_world_pixel_xy=start_world_pixel,
        end_world_pixel_xy=end_world_pixel,registration_sha256=report['images']['slam']['registration_sha256'])


def load_saved_slam_map(registration):
    import yaml
    import numpy as np
    from PIL import Image
    if registration.get('accepted') is not True:
        raise ValueError('Saved SLAM map requires an accepted registration')
    meta=yaml.safe_load(SAVED_YAML.read_text())
    if meta.get('image') != SAVED_PGM.name:
        raise ValueError('Saved-map YAML image does not identify the historical PGM')
    resolution=float(meta['resolution']);origin=meta['origin']
    if (not math.isfinite(resolution) or resolution <= 0 or len(origin) != 3 or
            not all(math.isfinite(float(v)) for v in origin) or abs(float(origin[2])) > 1e-9):
        raise ValueError('Unsupported saved-map resolution/origin')
    if (not math.isclose(resolution, registration['coordinate_conventions']['saved_resolution_m_per_pixel'], abs_tol=1e-12) or
            list(origin) != registration['coordinate_conventions']['saved_origin']):
        raise ValueError('Registration and saved-map YAML coordinate metadata disagree')
    image=np.asarray(Image.open(SAVED_PGM).convert('L'))
    if image.ndim != 2 or image.size == 0:
        raise ValueError('Historical saved PGM is not a grayscale raster')
    occupied=image < 128
    occupied_rows,occupied_cols=np.where(occupied)
    if len(occupied_rows)==0:
        raise ValueError('Historical saved PGM has no occupied map geometry to frame')
    padding_px=int(math.ceil(.8/float(meta['resolution'])))
    row0=max(0,int(occupied_rows.min())-padding_px)
    row1=min(image.shape[0],int(occupied_rows.max())+1+padding_px)
    col0=max(0,int(occupied_cols.min())-padding_px)
    col1=min(image.shape[1],int(occupied_cols.max())+1+padding_px)
    return image, meta, dict(row0=row0,row1=row1,col0=col0,col1=col1)


def transform_truth_rows_to_saved_map(rows, registration):
    transform=registration['transform'];c=math.cos(transform['yaw_rad']);s=math.sin(transform['yaw_rad'])
    return [(c*r['x']-s*r['y']+transform['x_m'],
             s*r['x']+c*r['y']+transform['y_m']) for r in rows]


def render_saved_slam_track(path, rows, registration, stationary=False):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    image,meta,crop=load_saved_slam_map(registration)
    ox,oy,_=meta['origin'];resolution=float(meta['resolution']);height,width=image.shape
    mapped=transform_truth_rows_to_saved_map(rows,registration)
    map_rows=[dict(row, x=xy[0], y=xy[1]) for row,xy in zip(rows,mapped)]
    fig,ax=plt.subplots(figsize=(10,8),dpi=260)
    # Preserve the original grayscale PGM cells and all accumulated map noise.
    crop_image=image[crop['row0']:crop['row1'],crop['col0']:crop['col1']]
    xmin=ox+crop['col0']*resolution;xmax=ox+crop['col1']*resolution
    ymax=oy+(height-crop['row0'])*resolution;ymin=oy+(height-crop['row1'])*resolution
    ax.imshow(crop_image,cmap='gray',vmin=0,vmax=255,origin='upper',interpolation='nearest',
        extent=[xmin,xmax,ymin,ymax])
    ax.set_xlim(xmin,xmax);ax.set_ylim(ymin,ymax)
    ax.set_aspect('equal');ax.axis('off')
    _draw_rows(ax,map_rows,stationary)
    _render_title_and_legend(fig,ax,'B2-W Warehouse',
        'SLAM map with ground-truth trajectory · saved_slam_map frame · /sim/ground_truth/odom',
        'Historical SLAM PGM',stationary)
    fig.subplots_adjust(left=.025,right=.975,top=.89,bottom=.16)
    fig.savefig(path,dpi=260,bbox_inches='tight',pad_inches=.08,facecolor='white');plt.close(fig)


def yaw_of(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def render_track(path, grid, rows):
    import numpy as np
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.colors import ListedColormap, BoundaryNorm
    from matplotlib.transforms import Affine2D

    figure = Figure(figsize=(10, 8)); FigureCanvasAgg(figure); axis = figure.subplots()
    if grid:
        width, height = grid['width'], grid['height']
        cells = np.asarray(grid['data'], dtype=np.int16).reshape((height, width))
        raster = np.where(cells < 0, 101, cells)
        colors = [(v, v, v, 1.) for v in np.linspace(1., .05, 101)] + [(.68, .72, .76, 1.)]
        cmap = ListedColormap(colors)
        ox, oy, yaw = grid['origin']
        axis.imshow(raster, origin='lower', interpolation='nearest', cmap=cmap,
                    norm=BoundaryNorm(np.arange(-.5, 102.5), cmap.N),
                    extent=[0, width*grid['resolution'], 0, height*grid['resolution']],
                    transform=Affine2D().rotate(yaw).translate(ox, oy) + axis.transData,
                    zorder=0)
    else:
        axis.text(.5, .5, 'Public /map unavailable', ha='center', va='center', transform=axis.transAxes)

    segments = []
    for row in rows:
        if (not segments or row['timestamp_ns'] <= segments[-1][-1]['timestamp_ns'] or
                row['timestamp_ns'] - segments[-1][-1]['timestamp_ns'] > 2_000_000_000):
            segments.append([])
        segments[-1].append(row)
    for index, segment in enumerate(segments):
        xy = np.asarray([[row['x'], row['y']] for row in segment])
        if len(xy) == 1:
            axis.scatter(xy[:, 0], xy[:, 1], color=f'C{index % 10}', s=18,
                         label='Actual TF trajectory' if index == 0 else None, zorder=4)
        else:
            axis.plot(xy[:, 0], xy[:, 1], '.-', color=f'C{index % 10}', linewidth=1.5,
                      markersize=2, label='Actual TF trajectory' if index == 0 else None, zorder=4)
    if rows:
        first, last = rows[0], rows[-1]
        axis.scatter(first['x'], first['y'], marker='*', s=150, color='#239b56',
                     edgecolor='black', label='Recorded start', zorder=5)
        axis.scatter(last['x'], last['y'], marker='X', s=80, color='#ff8c00',
                     edgecolor='black', label='Recorded end', zorder=6)
    axis.set(xlabel='map x (m)', ylabel='map y (m)', title='Warehouse map and measured trajectory')
    axis.set_aspect('equal'); axis.grid(alpha=.15)
    if rows:
        axis.legend(loc='best', fontsize=8)
    figure.tight_layout(); figure.savefig(path, dpi=150); figure.clear()


class ManualLogger:
    def __init__(self, output):
        import rclpy
        from rclpy.clock import Clock, ClockType
        from rclpy.parameter import Parameter
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rclpy.time import Time
        from nav_msgs.msg import OccupancyGrid, Path as PathMsg
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from rclpy.qos import qos_profile_sensor_data
        from geometry_msgs.msg import Point, PoseStamped
        from visualization_msgs.msg import Marker, MarkerArray
        from tf2_ros import Buffer, TransformListener, TransformException

        self.rclpy, self.Time, self.PathMsg = rclpy, Time, PathMsg
        self.Marker, self.MarkerArray = Marker, MarkerArray
        self.Point = Point
        self.TransformException = TransformException
        self.node = Node('warehouse_manual_track_logger', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map = None; self.map_error = None; self.rows = []; self.path_msg = PathMsg()
        self.truth_rows = []
        self.slam_rows = []
        self.saved_registration,self.saved_registration_error=load_saved_registration()
        self.fixed_metadata, self.fixed_image = load_fixed_map(floor.ROOT / 'third_party/aws_warehouse/warehouse.world')
        self.map_stamp_ns = None
        self.path_msg.header.frame_id = 'map'
        self.epoch = 0; self.segment = 0; self.last_sim_ns = None
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self.node)
        self.path_pub = self.node.create_publisher(PathMsg, '/course_manual/trajectory', transient)
        self.markers_pub = self.node.create_publisher(MarkerArray, '/course_manual/trajectory_markers', transient)
        self.node.create_subscription(OccupancyGrid, '/map', self.on_map, transient)
        self.node.create_subscription(Odometry, '/sim/ground_truth/odom', self.on_truth, qos_profile_sensor_data)
        self.node.create_subscription(Odometry, '/slam/odom', self.on_slam, qos_profile_sensor_data)
        self.node.create_subscription(PoseStamped, '/goal_pose', self.on_goal, qos_profile_sensor_data)
        self.node.create_timer(.2, self.sample, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.output = Path(output); self.output.mkdir(parents=True, exist_ok=True)
        self.started = time.time(); self.stop_reason = 'shutdown'
        self.motion_enabled = False
        self.goal_start_xy = None
        self.goal_pending = False
        self.goal_truth_rows = []

    def on_goal(self, _msg):
        self.goal_pending = True
        self.goal_start_xy = ([self.truth_rows[-1]['x'], self.truth_rows[-1]['y']]
                              if self.truth_rows else None)
        self.goal_truth_rows = ([dict(x=self.goal_start_xy[0], y=self.goal_start_xy[1])]
                                if self.goal_start_xy is not None else [])

    def on_slam(self, msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link': return
        stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        if stamp<=0 or not all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w)) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.001:return
        previous=self.slam_rows[-1] if self.slam_rows else None
        if previous and stamp<=previous['timestamp_ns']:return
        self.slam_rows.append(dict(timestamp_ns=stamp,x=float(p.x),y=float(p.y),yaw_rad=yaw_of(q),segment=0,epoch=0,source='/slam/odom',frame_id='odom'))

    def on_truth(self, msg):
        if msg.header.frame_id != 'sim_world' or msg.child_frame_id != 'base_link':
            return
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        now = self.node.get_clock().now().nanoseconds
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if (stamp <= 0 or not 0 <= now-stamp <= 500_000_000 or
                not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) or
                abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > .001):
            return
        previous = self.truth_rows[-1] if self.truth_rows else None
        if previous and stamp == previous['timestamp_ns']:
            return
        epoch = previous['epoch'] if previous else 0
        segment = previous['segment'] if previous else 0
        if previous and stamp < previous['timestamp_ns']:
            epoch += 1; segment += 1
        elif previous and stamp-previous['timestamp_ns'] > 2_000_000_000:
            segment += 1
        self.truth_rows.append(dict(timestamp_ns=stamp, x=float(p.x), y=float(p.y),
                                    yaw_rad=yaw_of(q), segment=segment, epoch=epoch,
                                    source='/sim/ground_truth/odom'))
        row = self.truth_rows[-1]
        if self.goal_pending:
            if self.goal_start_xy is None:
                self.goal_start_xy = [row['x'], row['y']]
            self.goal_truth_rows.append(row)
        if self.goal_pending and has_measured_motion(self.goal_truth_rows):
            self.motion_enabled = True
            self.goal_pending = False

    def on_map(self, msg):
        if msg.header.frame_id != 'map':
            self.map_error = 'map_frame_mismatch'; return
        if (msg.info.width <= 0 or msg.info.height <= 0 or
                len(msg.data) != msg.info.width * msg.info.height or
                not math.isfinite(msg.info.resolution) or msg.info.resolution <= 0 or
                any(value < -1 or value > 100 for value in msg.data)):
            self.map_error = 'invalid_occupancy_grid'; return
        origin_q = msg.info.origin.orientation
        q_norm = sum(value*value for value in (origin_q.x, origin_q.y, origin_q.z, origin_q.w))
        if (not all(math.isfinite(value) for value in
                    (msg.info.origin.position.x, msg.info.origin.position.y,
                     origin_q.x, origin_q.y, origin_q.z, origin_q.w)) or
                abs(q_norm - 1.) > 1e-3 or abs(origin_q.x) > 1e-3 or abs(origin_q.y) > 1e-3):
            self.map_error = 'invalid_map_origin'; return
        self.map = dict(width=msg.info.width, height=msg.info.height,
            resolution=msg.info.resolution,
            origin=[msg.info.origin.position.x, msg.info.origin.position.y, yaw_of(origin_q)],
            data=list(msg.data), frame_id=msg.header.frame_id)
        self.map_stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.map_error = None

    def sample(self):
        try:
            transform = self.tf.lookup_transform('map', 'base_link', self.Time())
        except self.TransformException:
            return
        if transform.header.frame_id != 'map' or transform.child_frame_id != 'base_link':
            return
        t = transform.transform.translation; stamp = transform.header.stamp
        timestamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        now_ns = self.node.get_clock().now().nanoseconds
        if now_ns <= 0 or timestamp_ns <= 0:
            return
        clock_reset = False
        if self.last_sim_ns is not None and now_ns < self.last_sim_ns:
            self.epoch += 1; self.segment += 1
            self.path_msg = self.PathMsg(); self.path_msg.header.frame_id = 'map'
            clock_reset = True
        self.last_sim_ns = now_ns
        age_ns = now_ns - timestamp_ns
        if not 0 <= age_ns <= 500_000_000:
            return
        if self.rows and timestamp_ns == self.rows[-1]['timestamp_ns']:
            return
        if self.rows and timestamp_ns < self.rows[-1]['timestamp_ns']:
            if not clock_reset:
                self.epoch += 1; self.segment += 1
            self.path_msg = self.PathMsg(); self.path_msg.header.frame_id = 'map'
        elif self.rows and (timestamp_ns <= self.rows[-1]['timestamp_ns'] or
                            timestamp_ns - self.rows[-1]['timestamp_ns'] > 2_000_000_000):
            self.segment += 1
        orientation = transform.transform.rotation
        orientation_norm = sum(value*value for value in
                               (orientation.x, orientation.y, orientation.z, orientation.w))
        if (not all(math.isfinite(value) for value in
                    (t.x, t.y, t.z, orientation.x, orientation.y, orientation.z, orientation.w)) or
                abs(orientation_norm - 1.) > 1e-3):
            return
        row = dict(timestamp_ns=timestamp_ns, x=float(t.x), y=float(t.y),
                   yaw_rad=yaw_of(orientation), segment=self.segment, epoch=self.epoch,
                   source='public map->base_link TF')
        self.rows.append(row)
        pose = __import__('geometry_msgs.msg', fromlist=['PoseStamped']).PoseStamped()
        pose.header = transform.header; pose.header.frame_id = 'map'
        pose.pose.position = self.Point(x=t.x, y=t.y, z=t.z)
        pose.pose.orientation = orientation
        self.path_msg.header = pose.header; self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)
        self.publish_markers()

    def publish_markers(self):
        message = self.MarkerArray(); segments = []
        for row in self.rows:
            if (not segments or row['timestamp_ns'] <= segments[-1][-1]['timestamp_ns'] or
                    row['timestamp_ns'] - segments[-1][-1]['timestamp_ns'] > 2_000_000_000):
                segments.append([])
            segments[-1].append(row)
        for index, points in enumerate(segments):
            marker = self.Marker(); marker.header.frame_id = 'map'; marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.ns = 'actual_trajectory'; marker.id = index; marker.type = self.Marker.LINE_STRIP
            marker.action = self.Marker.ADD; marker.pose.orientation.w = 1.
            marker.scale.x = .035; marker.color.a = 1.
            marker.color.r = float(index % 3 == 0); marker.color.g = float(index % 3 == 1); marker.color.b = float(index % 3 == 2)
            from geometry_msgs.msg import Point
            marker.points = [self.Point(x=p['x'], y=p['y'], z=.04) for p in points]
            if len(points) == 1:
                marker.type = self.Marker.SPHERE; marker.scale.x = marker.scale.y = marker.scale.z = .08
                marker.pose.position = self.Point(x=points[0]['x'], y=points[0]['y'], z=.04)
            message.markers.append(marker)
        self.markers_pub.publish(message)

    def export(self):
        csv_path = self.output / 'trajectory.csv'
        with csv_path.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=['timestamp_ns', 'x', 'y', 'yaw_rad', 'segment', 'epoch', 'source'])
            writer.writeheader(); writer.writerows(self.truth_rows)
        saved_xy=export_map_images(self.output,self.truth_rows,self.fixed_metadata,self.fixed_image,
                                   self.saved_registration,stationary=not self.motion_enabled)
        saved_image,saved_yaml,saved_crop=load_saved_slam_map(self.saved_registration)
        saved_height,saved_width=saved_image.shape
        saved_origin=saved_yaml['origin'];saved_resolution=float(saved_yaml['resolution'])
        saved_pixels=_map_xy_to_pixel(saved_xy,saved_origin,saved_resolution,saved_width,saved_height)
        saved_display_pixels=[[p[0]-saved_crop['col0'],p[1]-saved_crop['row0']] for p in saved_pixels]
        world_xy=[[r['x'],r['y']] for r in self.truth_rows]
        world_pixels=floor.world_to_pixel(world_xy,self.fixed_metadata).tolist() if world_xy else []
        slam_path = self.output / 'trajectory_slam.png'
        result = dict(kind='manual_warehouse_view', outcome='stopped_by_user', success=None,
            stop_reason=self.stop_reason, motion_enabled=self.motion_enabled,
            motion_classification='stationary' if not self.motion_enabled else 'measured_motion',
            trajectory_samples=len(self.truth_rows),
            trajectory_source='/sim/ground_truth/odom',trajectory_source_frame='sim_world',
            trajectory_frame='world',
            source_to_world_relation='sim_world and world are identity-aligned by the SDF ground-truth odometry publisher',
            map_source=str(floor.IMAGE),
            map_sha256=self.fixed_metadata['image_sha256'],
            map_error=None, truth_epoch_count=(self.truth_rows[-1]['epoch'] + 1 if self.truth_rows else 0),
            last_truth_age_s=((self.node.get_clock().now().nanoseconds - self.truth_rows[-1]['timestamp_ns']) * 1e-9
                           if self.truth_rows else None),
            final_position=({k: self.truth_rows[-1][k] for k in ('x', 'y', 'yaw_rad')} if self.truth_rows else None),
            artifacts=dict(csv=str(csv_path), image=str(self.output / 'trajectory.png')),
            images=dict(ground_truth=dict(path=str(self.output/'trajectory_ground_truth.png'),source_topic='/sim/ground_truth/odom',source_frame='sim_world',coordinate_frame='world',map_asset=str(floor.IMAGE),map_sha256=self.fixed_metadata['image_sha256'],resolution_m_per_pixel=self.fixed_metadata['resolution'],pixel_contract=self.fixed_metadata['pixel_contract'],start_world_xy=world_xy[0] if world_xy else None,end_world_xy=world_xy[-1] if world_xy else None,start_image_pixel_xy=world_pixels[0] if world_pixels else None,end_image_pixel_xy=world_pixels[-1] if world_pixels else None,trajectory_drawn=self.motion_enabled),
                        slam=dict(path=str(slam_path),source_topic='/sim/ground_truth/odom',trajectory_source_frame='sim_world',trajectory_frame='world',
                            trajectory_label='ground truth; not SLAM localization',map_asset=str(SAVED_PGM),
                            map_sha256=sha256(SAVED_PGM),available=True,
                            yaml_asset=str(SAVED_YAML),yaml_sha256=sha256(SAVED_YAML),
                            resolution_m_per_pixel=saved_resolution,origin=list(saved_origin),
                            pgm_size_px=[saved_width,saved_height],
                            pixel_contract='top-left pixel center: x=origin_x+(column+0.5)*resolution; y=origin_y+(height-row-0.5)*resolution',
                            display_crop_pixels=saved_crop,
                            registration_path=str(SAVED_REGISTRATION),
                            registration_sha256=sha256(SAVED_REGISTRATION),
                            registration=self.saved_registration['transform'],
                            registration_fit_rmse_m=self.saved_registration['fit_rmse_m'],
                            registration_holdout_error_m=self.saved_registration['holdout_error_m'],
                            coordinate_frame='saved_slam_map',recorded_truth_poses=len(self.truth_rows),
                            trajectory_drawn=self.motion_enabled,
                            start_saved_map_xy=saved_xy[0] if saved_xy else None,
                            end_saved_map_xy=saved_xy[-1] if saved_xy else None,
                            start_image_pixel_xy=saved_pixels[0] if saved_pixels else None,
                            end_image_pixel_xy=saved_pixels[-1] if saved_pixels else None,
                            start_display_pixel_xy=saved_display_pixels[0] if saved_display_pixels else None,
                            end_display_pixel_xy=saved_display_pixels[-1] if saved_display_pixels else None,
                            stationary=not self.motion_enabled)),
            started_utc=datetime.fromtimestamp(self.started, timezone.utc).isoformat(),
            ended_utc=datetime.now(timezone.utc).isoformat())
        (self.output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')

    def run(self):
        try:
            self.rclpy.spin(self.node)
        except (KeyboardInterrupt, self.rclpy.executors.ExternalShutdownException):
            self.stop_reason = 'interrupted'
        finally:
            self.export()
            self.node.destroy_node()
            if self.rclpy.ok(): self.rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--render-recorded-session', type=Path,
                        help='Offline-render an existing trajectory.csv/result.json into output-dir')
    args = parser.parse_args()
    if args.render_recorded_session:
        print(json.dumps(export_recorded_session(args.render_recorded_session,args.output_dir),indent=2))
        return
    import rclpy
    rclpy.init()
    ManualLogger(args.output_dir).run()


if __name__ == '__main__':
    main()
