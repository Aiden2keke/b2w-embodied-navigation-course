"""Real map callbacks/cloud decoding/TF Buffer, no ROS node or participant."""
from collections import deque
from types import SimpleNamespace as NS
import math
from pathlib import Path
import sys

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from tf2_ros import Buffer
from course_sim.cloud_format import OUTPUT, encode
from course_navigation.map_node import MapProjector
from course_navigation.map_projection import Grid

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import warehouse_ab_trial as trial


def cloud(points,frame='lidar_link',stamp=18):
    points=np.asarray(points)
    data=np.zeros(len(points),dtype=OUTPUT)
    for i,k in enumerate('xyz'):data[k]=points[:,i]
    return encode(data,Header(frame_id=frame,stamp=Time(sec=stamp)))


@pytest.fixture
def projector():
    m=MapProjector.__new__(MapProjector)
    m.grid=Grid(.05,(-20.,-20.),(40.,40.))
    m.min_z=.15;m.max_z=2.;m.frame='map';m.buffer=Buffer()
    m.pending=deque(maxlen=30);m.have_map=False
    m.published=[];m.publisher=NS(publish=m.published.append)
    m.now=18
    m.get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:Time(sec=m.now)))
    m.get_logger=lambda:NS(error=lambda *a,**k:None)
    return m


def transform(m,stamp=18,xy=(0.,0.),yaw=0.):
    t=TransformStamped();t.header.frame_id='map';t.child_frame_id='lidar_link'
    t.header.stamp=Time(sec=stamp);t.transform.translation.x=float(xy[0])
    t.transform.translation.y=float(xy[1]);t.transform.rotation.z=math.sin(yaw/2)
    t.transform.rotation.w=math.cos(yaw/2);m.buffer.set_transform(t,'offline-map-test')


def ingest(msg):
    m=trial.Monitor.__new__(trial.Monitor);m.latest={};m.failure=None
    m.receive_map(msg)
    return m


def test_obstacle_map_can_publish_before_any_scan_transform(projector):
    m=projector
    m.map(cloud([[4.,4.,.5]],frame='map'))
    m.scan(cloud([[3.,0.,.5]]))
    m.process_scans();m.publish()
    assert len(m.pending)==1
    grid=m.published[-1].data
    assert grid.count(100)==1 and grid.count(0)==0
    assert not ingest(m.published[-1]).grid['observed_free_present']


def test_delayed_exact_tf_then_real_rays_clear_full_pocket(projector):
    m=projector
    m.map(cloud([[4.,4.,.5]],frame='map'))
    angles=np.linspace(0,2*np.pi,720,endpoint=False)
    m.scan(cloud(np.column_stack((3*np.cos(angles),3*np.sin(angles),np.full(720,.5)))))
    transform(m,stamp=17)  # Latest TF is too old for scan stamp18: no extrapolation.
    m.process_scans();m.publish()
    assert len(m.pending)==1 and m.published[-1].data.count(0)==0
    transform(m,stamp=18)
    m.process_scans();m.publish()
    owner=ingest(m.published[-1])
    assert not m.pending and owner.grid['observed_free_present']
    # Finite rays can leave unknown cells; accumulation, not synthetic filling,
    # must cover those cells before the conservative full disk can pass.
    assert not trial.open_pocket(owner.grid,(-.342,0.))
    angles+=np.deg2rad(.25)
    m.now=19;transform(m,stamp=19)
    m.scan(cloud(np.column_stack((3*np.cos(angles),3*np.sin(angles),np.full(720,.5))),stamp=19))
    m.process_scans();m.publish();owner=ingest(m.published[-1])
    assert trial.open_pocket(owner.grid,(-.342,0.))
    assert m.published[-1].data.count(100)==1


def test_one_free_ray_never_substitutes_for_disk_clearance(projector):
    m=projector;m.map(cloud([[4.,4.,.5]],frame='map'));transform(m)
    m.scan(cloud([[3.,0.,.5]]));m.process_scans();m.publish()
    owner=ingest(m.published[-1])
    assert owner.grid['observed_free_present']
    with pytest.raises(RuntimeError,match='1.5m open pocket absent'):
        trial.motion_pocket_admission(owner.grid,(0.,0.),stationary_containment=False)


def test_scan_projection_applies_recorded_frame_transform_direction(projector):
    m=projector;transform(m,xy=(1.,2.),yaw=math.pi/2)
    assert m.project_scan(cloud([[2.,0.,.5]]))
    data=m.grid.data()
    x,y=m.grid.cells([1.,3.]);assert data[y,x]==0
    x,y=m.grid.cells([3.,2.]);assert data[y,x]==-1


def test_self_only_returns_do_not_create_free_space(projector):
    m=projector;transform(m)
    assert m.project_scan(cloud([[.1,0.,0.],[0.,.1,0.]]))
    assert not m.grid.free.any()
