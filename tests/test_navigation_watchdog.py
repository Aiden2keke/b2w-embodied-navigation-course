"""Final command-path watchdog; real adapter callbacks, no ROS node/context."""
from types import SimpleNamespace as NS
from unittest.mock import patch
import math
import pytest
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from course_navigation.velocity_adapter import VelocityAdapter, VelocityGate


def stamped(msg, t):
    msg.header.stamp.sec=int(t);msg.header.stamp.nanosec=round((t-int(t))*1e9)
    return msg


def fixture():
    a=object.__new__(VelocityAdapter)
    a.gate=VelocityGate();a.last_stamp=None;a.require_tf=True;a.m2_enabled=False;a.speed_limit=None
    a.localization_fault=False;a.clock_stamp=None;a.clock_wall=None;a.last_wall=None;a.tf_state={}
    clock=NS(sim=10.,wall=20.)
    a.get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(clock.sim*1e9)))
    messages=[];a.pub=NS(publish=lambda m:messages.append((m.linear.x,m.linear.y,m.angular.z)))
    links={}
    for parent,child in [('map','odom'),('odom','base_link'),('map','base_link')]:
        tf=stamped(TransformStamped(),10.);tf.header.frame_id=parent;tf.child_frame_id=child;tf.transform.rotation.w=1.
        links[parent,child]=tf
    a.tf=NS(lookup_transform=lambda p,c,t:links[p,c])
    return a,clock,links,messages


def state(t=10.):
    msg=stamped(Odometry(),t);msg.header.frame_id='odom';msg.child_frame_id='base_link';msg.pose.pose.orientation.w=1.
    return msg


def command(a):
    msg=Twist();msg.linear.x=.1;a.command(msg)


def test_valid_odom_and_each_link_allow_command():
    a,c,links,out=fixture()
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
        a.state(state());command(a);a.publish()
    assert out==[(.1,0.,0.)]


@pytest.mark.parametrize('link',[('map','odom'),('odom','base_link')])
def test_stale_direct_link_blocks_despite_fresh_composite(link):
    a,c,links,out=fixture();stamped(links[link],9.4)
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
        a.state(state());command(a);a.publish()
    assert out[-1]==(0.,0.,0.)


def test_duplicate_odom_cannot_renew_wall_freshness():
    a,c,links,out=fixture()
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
        a.state(state());c.wall+=.4;a.state(state());command(a)
        c.wall+=.11;a.publish()
    assert out[-1]==(0.,0.,0.)


def test_clock_suspension_with_new_commands_and_duplicate_odom_stops():
    a,c,links,out=fixture()
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
        a.state(state());command(a);a.publish()
        c.wall+=.6;a.state(state());command(a);a.publish()
    assert out[-1]==(0.,0.,0.)


def test_clock_rollback_latches_stop_until_restart():
    a,c,links,out=fixture()
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
        a.state(state());command(a);a.publish()
        c.sim=9.9;c.wall+=.1;a.state(state(9.9));command(a);a.publish()
        c.sim=10.1;c.wall+=.1;a.state(state(10.1));command(a)
        for tf in links.values():stamped(tf,10.1)
        a.publish()
    assert out[-2:]==[(0.,0.,0.),(0.,0.,0.)]


@pytest.mark.parametrize('fault',['frame','child','zero_q','nonunit_q','nan_twist','future','stale','nan_position'])
def test_invalid_odom_revokes_prior_validity_immediately(fault):
    a,c,links,out=fixture();msg=state()
    if fault=='frame':msg.header.frame_id='map'
    if fault=='child':msg.child_frame_id='laser'
    if fault=='zero_q':msg.pose.pose.orientation.w=0.
    if fault=='nonunit_q':msg.pose.pose.orientation.w=2.
    if fault=='nan_twist':msg.twist.twist.linear.x=math.nan
    if fault=='nan_position':msg.pose.pose.position.x=math.nan
    if fault=='future':stamped(msg,10.1)
    if fault=='stale':stamped(msg,9.4)
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
        a.state(state());command(a);a.state(msg);a.publish()
    assert out[-1]==(0.,0.,0.)


@pytest.mark.parametrize('fault',['nan_translation','nonunit_q','wrong_frame','future'])
def test_invalid_tf_revokes_command(fault):
    a,c,links,out=fixture();tf=links['map','odom']
    if fault=='nan_translation':tf.transform.translation.x=math.nan
    if fault=='nonunit_q':tf.transform.rotation.w=2.
    if fault=='wrong_frame':tf.header.frame_id='world'
    if fault=='future':stamped(tf,10.1)
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
        a.state(state());command(a);a.publish()
    assert out[-1]==(0.,0.,0.)


def test_existing_unlocalized_simulation_body_state_contract_is_preserved():
    a,c,links,out=fixture();a.require_tf=False;msg=state();msg.header.frame_id='sim_world'
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
        a.state(msg);command(a);a.publish()
    assert out[-1]==(.1,0.,0.)


def test_stalled_tf_receipt_stops_even_while_source_age_is_small():
    a,c,links,out=fixture()
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
        a.state(state());command(a);a.publish()
        c.wall+=.51;c.sim+=.2;a.state(state(10.2));command(a);a.publish()
    assert out[-1]==(0.,0.,0.)


def test_odom_source_rollback_latches_after_recovery():
    a,c,links,out=fixture()
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
        a.state(state());command(a);a.publish()
        c.sim+=.1;c.wall+=.1;a.state(state(9.9))
        c.sim+=.1;c.wall+=.1;a.state(state(10.2));command(a)
        for tf in links.values():stamped(tf,10.2)
        a.publish()
    assert out[-1]==(0.,0.,0.)


@pytest.mark.parametrize('kind',['odom','tf'])
def test_malformed_nanoseconds_cannot_disguise_a_fresh_source(kind):
    a,c,links,out=fixture();msg=state()
    if kind=='odom':msg.header.stamp.sec=8;msg.header.stamp.nanosec=2000000000
    else:
        links['map','odom'].header.stamp.sec=8;links['map','odom'].header.stamp.nanosec=2000000000
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
        a.state(state());command(a)
        if kind=='odom':a.state(msg)
        a.publish()
    assert out[-1]==(0.,0.,0.)


def test_tf_source_rollback_latches_even_when_old_stamp_is_stale():
    a,c,links,out=fixture()
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
        a.state(state());command(a);a.publish()
        stamped(links['map','odom'],9.);a.publish()
        c.sim+=.1;c.wall+=.1;a.state(state(10.1));command(a)
        for tf in links.values():stamped(tf,10.1)
        a.publish()
    assert out[-1]==(0.,0.,0.)
