"""Five-second structural cadence cannot expire healthy fast localization/grids."""
import time
from types import SimpleNamespace as NS
import pytest
from test_stage4_safety_guard import configured,decide,grid


def advance(g,cfg,sim,wall,seq,*,update_fast=True):
    from course_navigation.stage4_control import write_control
    write_control(cfg['lease_path'],dict(session_id='trial',sequence=seq,wall=wall,owner=cfg['owner'],adapter=g.adapter_identity))
    g.contacts.inventory(dict(sequence=seq,interval_start=sim-.05,stamp=sim,sampled_steps=10,
        collisions=['body::collision'],ever_forbidden_contact=False,evaluation_only=True),wall=wall)
    sec=int(sim);stamp=NS(sec=sec,nanosec=round((sim-sec)*1e9))
    g.contacts.contact_message(NS(header=NS(frame_id='b2w_evaluation_seq_'+str(seq),stamp=stamp),contacts=[]),wall=wall)
    if update_fast:
        for name in ('local','global'):g.install(name,grid(),frame='odom' if name=='local' else 'map',source=sim,receipt=wall,encoding='costmap',epoch='trial')
    return decide(g,sim=sim,wall=wall,pose_stamp=sim,pose_receipt=wall)


def test_five_second_structural_stream_and_twenty_hz_fast_inputs(tmp_path):
    g,cfg=configured(tmp_path)
    for i in range(1,101):
        sim=10.+i*.05;wall=20.+i*.05
        if i==100:
            g.observe_structure(sim,sim=sim,wall=wall)
            g.install('live',grid(),frame='map',source=sim,receipt=wall,encoding='costmap',epoch='trial')
        assert advance(g,cfg,sim,wall,i+1)==(.05,0.,0.),g.failure
    assert not g.failure


def test_structural_duplicate_cannot_renew_wall_or_source_expiry(tmp_path):
    g,cfg=configured(tmp_path)
    for i in range(1,152):
        sim=10.+i*.05;wall=20.+i*.05
        g.install('live',grid(),frame='map',source=10.,receipt=wall,encoding='costmap',epoch='trial')
        out=advance(g,cfg,sim,wall,i+1)
    assert out==(0.,0.,0.) and g.failure


def test_dynamic_costmaps_keep_two_second_expiry(tmp_path):
    g,cfg=configured(tmp_path)
    for i in range(1,43):out=advance(g,cfg,10.+i*.05,20.+i*.05,i+1,update_fast=False)
    assert out==(0.,0.,0.) and g.failure


def test_identical_grid_keeps_active_geometry_changed_grid_denies_immediately(tmp_path):
    from course_navigation.stage4_guard import GridPreparation
    import threading
    g,cfg=configured(tmp_path);worker=GridPreparation(g)
    held=threading.Event();release=threading.Event();original=g.prepare
    def paused(*args,**kw):
        held.set();assert release.wait(2.);return original(*args,**kw)
    try:
        worker.submit('local',grid(),frame='odom',source=10.1,receipt=20.1,encoding='costmap',epoch='trial')
        deadline=time.monotonic()+2.
        while time.monotonic()<deadline:
            if g.prepared_status()['grids'].get('local',{}).get('source')==10.1:break
            time.sleep(.002)
        # Worker is held: an identical accepted payload should need no rebuild.
        g.prepare=paused
        worker.submit('local',grid(),frame='odom',source=10.2,receipt=20.2,encoding='costmap',epoch='trial')
        assert 'local' in g.prepared_status()['grids']
        changed=grid();changed['data'][0]=254
        worker.submit('local',changed,frame='odom',source=10.3,receipt=20.3,encoding='costmap',epoch='trial')
        assert held.wait(1.)
        assert 'local' not in g.prepared_status()['grids']
    finally:release.set();assert worker.close()


def test_map_timer_republication_cannot_mask_expired_structural_source(tmp_path):
    g,cfg=configured(tmp_path)
    for i in range(1,152):
        sim=10.+i*.05;wall=20.+i*.05
        if i%20==0:g.install('live',grid(),frame='map',source=sim,receipt=wall,encoding='costmap',epoch='trial')
        out=advance(g,cfg,sim,wall,i+1)
    assert out==(0.,0.,0.) and g.failure


def test_adapter_structural_callback_checks_real_source_frame(tmp_path):
    from test_navigation_watchdog import fixture
    from unittest.mock import patch
    a,c,links,out=fixture();g,cfg=configured(tmp_path);a.stage4_guard=g
    message=NS(header=NS(frame_id='map',stamp=NS(sec=10,nanosec=0)))
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
        a.stage4_structure(message)
        assert not g.failure
        message.header.frame_id='world';a.stage4_structure(message)
    assert g.failure=='structural_source_frame_mismatch'
