"""Actual child isolation and fail-closed lifecycle; no ROS or motion authority."""
import os
import signal
import time
from pathlib import Path
import pytest
from test_stage4_safety_guard import configured,grid
from course_navigation.stage4_guard import GridPreparation


def until(predicate,seconds=3.):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        if predicate():return
        time.sleep(.005)
    assert predicate()


def submit(w,name='local',source=10.1,payload=None):
    w.submit(name,payload or grid(),frame='odom' if name=='local' else 'map',source=source,
             receipt=source+10.,encoding='costmap',epoch='trial')


def test_grid_computation_is_in_owned_process_and_preserves_revision(tmp_path):
    g,cfg=configured(tmp_path);w=GridPreparation(g)
    try:
        assert hasattr(w,'process'), 'expensive preparation still shares adapter GIL'
        assert w.process.pid!=os.getpid()
        submit(w)
        until(lambda:g.prepared_status()['grids'].get('local',{}).get('source')==10.1)
        checker,source,receipt,revision=g.grids['local']
        from course_navigation.stage4_geometry import grid_revision,ClearanceGrid
        frozen,expected=grid_revision(grid(),'costmap')
        assert revision==expected
        assert checker.clearance([2.,2.,0.])==ClearanceGrid(frozen).clearance([2.,2.,0.])
        assert not checker.data.flags.writeable and not checker.points.flags.writeable
        submit(w,source=10.2)
        assert g.grids['local'][1:3]==(10.2,20.2)
        submit(w,source=10.2)
        assert g.grids['local'][2]==20.2
    finally:assert w.close()
    assert w.process.poll() is not None and not w.worker.is_alive()


@pytest.mark.parametrize('fault',['death','timeout','malformed'])
def test_worker_fault_latches_and_cannot_restart_within_session(tmp_path,fault):
    g,cfg=configured(tmp_path);w=GridPreparation(g)
    try:
        assert hasattr(w,'process'), 'no independently killable grid process'
        submit(w);until(lambda:g.prepared_status()['grids'].get('local',{}).get('source')==10.1)
        if fault=='death':w.process.kill()
        elif fault=='timeout':
            os.kill(w.process.pid,signal.SIGSTOP)
            changed=grid();changed['data'][0]=254;submit(w,source=10.2,payload=changed)
        else:
            bad=grid();bad['data'][0]=256;submit(w,source=10.2,payload=bad)
        until(lambda:g.failure is not None)
        assert Path(cfg['inhibit_path']).exists()
        pid=w.process.pid;submit(w,source=10.3)
        assert g.failure and w.process.pid==pid
        assert not g.prepared_status()['grids']
    finally:assert w.close()
    assert w.process.poll() is not None


def test_changed_geometry_invalidates_immediately_and_obsolete_result_is_rejected(tmp_path):
    g,cfg=configured(tmp_path);w=GridPreparation(g)
    try:
        assert hasattr(w,'process')
        submit(w);until(lambda:g.prepared_status()['grids'].get('local',{}).get('source')==10.1)
        os.kill(w.process.pid,signal.SIGSTOP)
        one=grid();one['data'][0]=254
        submit(w,source=10.2,payload=one)
        assert 'local' not in g.prepared_status()['grids']
        # Ensure the old job is actually in flight, rather than coalesced unsent.
        until(lambda:'local' not in w.pending,seconds=.2)
        two=grid();two['data'][1]=254
        for i in range(20):submit(w,source=10.3+i*.001,payload=two)
        assert len(w.pending)<=4
        os.kill(w.process.pid,signal.SIGCONT)
        until(lambda:g.prepared_status()['grids'].get('local',{}).get('source')==10.319)
        from course_navigation.stage4_geometry import grid_revision
        assert g.grids['local'][3]==grid_revision(two,'costmap')[1]
        assert not g.failure
    finally:assert w.close()


def test_ipc_drip_cannot_extend_total_deadline():
    import pickle,struct
    from course_navigation import stage4_grid_worker as ipc
    class Drip:
        def __init__(self):
            payload=pickle.dumps(('ready',));self.data=bytearray(struct.pack('!I',len(payload))+payload)
        def settimeout(self,value):self.timeout=value
        def recv_into(self,buffer):
            time.sleep(.01);buffer[0]=self.data.pop(0);return 1
    with pytest.raises(TimeoutError):ipc.receive(Drip(),deadline=time.monotonic()+.025)


@pytest.mark.parametrize('fault',['generation','epoch','entry'])
def test_malformed_worker_reply_invalidates_all_clearance(tmp_path,monkeypatch,fault):
    import subprocess
    script=tmp_path/'bad_worker.py'
    script.write_text('''import sys,socket\nfrom course_navigation.stage4_grid_worker import send,receive\ns=socket.socket(fileno=int(sys.argv[1]));receive(s);send(s,('ready',));name,generation,grid,meta=receive(s)\nfault=sys.argv[2]\nif fault=='generation':generation+=1\nif fault=='epoch':meta['epoch']='other-session'\nsend(s,('result',name,generation,meta,None,None))\n''')
    original=subprocess.Popen
    def fake(argv,**kwargs):return original([argv[0],str(script),argv[-1],fault],**kwargs)
    monkeypatch.setattr(subprocess,'Popen',fake)
    g,cfg=configured(tmp_path);w=GridPreparation(g)
    try:
        submit(w);until(lambda:g.failure is not None)
        assert 'grid_worker_' in g.failure and not g.prepared_status()['grids']
        assert Path(cfg['inhibit_path']).exists()
    finally:assert w.close()
