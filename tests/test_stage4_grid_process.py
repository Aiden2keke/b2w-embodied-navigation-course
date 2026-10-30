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


def test_clearance_grid_pickle_uses_one_compact_map_buffer_and_keeps_geometry():
    import hashlib,json,pickle
    from course_navigation.stage4_geometry import grid_revision,ClearanceGrid
    payload=grid();payload['width']=100;payload['height']=100;payload['data']=[0]*10000
    payload['data'][50*100+60]=254
    frozen,revision=grid_revision(payload,'costmap')
    legacy=dict(width=payload['width'],height=payload['height'],resolution=payload['resolution'],
        origin=tuple(payload['origin']),data=tuple(payload['data']))
    assert revision==hashlib.sha256(json.dumps(legacy,sort_keys=True,allow_nan=False).encode()).hexdigest()
    assert type(frozen['data']) is bytes and len(frozen['data'])==10000
    checker=ClearanceGrid(frozen);checker.data.flags.writeable=False;checker.points.flags.writeable=False
    poses=([ -1.,-.75,0.],[-1.5,-.75,0.],[-1.,-.75,.3])
    before=[checker.clearance(p) for p in poses]
    restored=pickle.loads(pickle.dumps(checker,protocol=5))
    after=[restored.clearance(p) for p in poses]
    assert after==before
    assert restored.points is restored.tree.data
    assert not restored.data.flags.writeable and not restored.points.flags.writeable
    assert restored.blocking_cell(1.5,[1.1,1.4,-.1,.1],extra=0.) is not None


def test_grid_worker_failure_diagnostic_names_phase_and_elapsed_times():
    from course_navigation.stage4_guard import GridPreparation
    diagnostic=GridPreparation.failure_diagnostic(TimeoutError('grid_worker_deadline'),
        phase='response_receive_decode',elapsed_s=.503,phase_times={'request_pickle_send_s':.061})
    assert 'phase=response_receive_decode' in diagnostic
    assert 'elapsed_s=0.503' in diagnostic
    assert 'request_pickle_send_s=0.061' in diagnostic


def test_clearance_grid_pickle_rejects_values_that_do_not_fit_byte_storage():
    import pickle
    from course_navigation.stage4_geometry import ClearanceGrid
    payload=grid();payload['data'][10]=256
    with pytest.raises(ValueError,match='byte range'):
        pickle.dumps(ClearanceGrid(payload),protocol=5)


def test_grid_submit_compacts_only_after_exact_occupancy_validation():
    from course_navigation.stage4_guard import GridPreparation
    payload=grid();payload['data']=[0, -1, 100, 50]
    payload.update(width=2,height=2)
    frozen=GridPreparation.compact_payload(payload,'occupancy')
    assert frozen['data']==bytes([0,254,254,254])
    assert frozen['_compact_encoding']=='occupancy'
    for bad in (True, -2, 101, 1.5):
        malformed=dict(payload,data=[0,bad,100,50])
        with pytest.raises(ValueError,match='invalid_grid'):
            GridPreparation.compact_payload(malformed,'occupancy')


def test_typed_ros_grid_buffer_uses_vectorized_validation_and_canonicalization():
    from array import array
    from course_navigation.stage4_guard import GridPreparation
    occupancy=dict(width=4,height=1,resolution=.05,origin=[0.,0.,0.],data=array('b',[0,-1,50,100]))
    compact=GridPreparation.compact_payload(occupancy,'occupancy')
    assert compact['data']==bytes([0,254,254,254])
    costmap=dict(occupancy,data=array('B',[0,1,254,255]))
    compact=GridPreparation.compact_payload(costmap,'costmap')
    assert compact['data']==bytes([0,1,254,255])
    costmap['data']=array('b',[0,-1,0,0])
    with pytest.raises(ValueError,match='invalid_grid'):
        GridPreparation.compact_payload(costmap,'costmap')


def test_compact_grid_revision_matches_legacy_canonical_json():
    import hashlib,json
    from course_navigation.stage4_geometry import grid_revision
    payload=grid();payload.update(width=4,height=1,data=[0,-1,31,100])
    legacy=dict(width=4,height=1,resolution=payload['resolution'],origin=tuple(payload['origin']),
                data=(0,254,254,254))
    expected=hashlib.sha256(json.dumps(legacy,sort_keys=True,allow_nan=False).encode()).hexdigest()
    compact=dict(legacy,data=bytes([0,254,254,254]),_compact_encoding='occupancy')
    assert grid_revision(compact,'occupancy')[1]==expected


def test_grid_submit_rejects_non_integer_costmap_values_before_byte_conversion():
    from course_navigation.stage4_guard import GridPreparation
    payload=grid();payload.update(width=2,height=2,data=[0,1,255,True])
    with pytest.raises(ValueError,match='invalid_grid'):
        GridPreparation.compact_payload(payload,'costmap')


def test_malformed_compact_submit_clears_pending_accepted_and_prepared_grids(tmp_path):
    import threading
    g,cfg=configured(tmp_path)
    w=GridPreparation.__new__(GridPreparation)
    from course_navigation.stage4_guard import REQUIRED
    w.guard=g;w.condition=threading.Condition()
    w.pending={name:('pending',) for name in REQUIRED}
    w.accepted={name:('accepted',) for name in REQUIRED}
    with g.lock:
        g.grids.update({name:(object(),10.1,20.1,'a'*64) for name in REQUIRED})
    bad=grid();bad['data'][0]=256

    w.submit('local',bad,frame='odom',source=10.2,receipt=20.2,
             encoding='costmap',epoch='trial')

    assert g.failure=='invalid_grid'
    assert Path(cfg['inhibit_path']).exists()
    assert not w.pending and not w.accepted and not g.prepared_status()['grids']


def test_worker_rejects_tampered_compact_occupancy_values(tmp_path):
    g,_=configured(tmp_path)
    payload=grid();payload.update(width=2,height=2,data=bytes([0,254,255,0]),_compact_encoding='occupancy')
    assert g.prepare('live',payload,frame='map',source=10.,receipt=20.,encoding='occupancy',epoch='trial') is None
    assert g.failure=='invalid_grid'


def test_grid_computation_is_in_owned_process_and_preserves_revision(tmp_path):
    g,cfg=configured(tmp_path);w=GridPreparation(g)
    try:
        assert hasattr(w,'process'), 'expensive preparation still shares adapter GIL'
        assert w.process.pid!=os.getpid()
        submit(w)
        until(lambda:g.prepared_status()['grids'].get('local',{}).get('source')==10.1)
        assert w.last_job_phase_times['worker_prepare_s']>=0
        assert w.last_job_phase_times['response_read_s']>=0
        assert w.last_job_phase_times['response_decode_s']>=0
        public_phases=g.prepared_status()['grid_worker_phases']
        assert public_phases['worker_prepare_s']>=0
        assert public_phases['response_read_s']>=0
        assert public_phases['response_decode_s']>=0
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


def test_expired_result_is_not_installed_after_validation(tmp_path,monkeypatch):
    g,_=configured(tmp_path);w=GridPreparation(g)
    original=g.live_cadence.observe
    def slow_install_cadence(*args):
        time.sleep(.51)
        return original(*args)
    monkeypatch.setattr(g.live_cadence,'observe',slow_install_cadence)
    try:
        submit(w,'live',source=10.1)
        until(lambda:g.failure is not None or 'live' in g.prepared_status()['grids'])
        assert g.failure and 'grid_worker_deadline' in g.failure
        assert 'live' not in g.prepared_status()['grids']
    finally:assert w.close()


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
        if fault=='timeout':
            assert 'phase=response_receive_decode' in g.failure
            assert 'elapsed_s=' in g.failure and 'request_pickle_send_s=' in g.failure
            assert g.prepared_status()['grid_worker_phases']['response_read_s']>=0
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
    import pickle
    from course_navigation import stage4_grid_worker as ipc
    class Drip:
        def __init__(self):
            payload=pickle.dumps(('ready',));self.data=bytearray(ipc._FRAME.pack(len(payload),0,0,0)+payload)
        def settimeout(self,value):self.timeout=value
        def recv_into(self,buffer):
            time.sleep(.01);buffer[0]=self.data.pop(0);return 1
    with pytest.raises(TimeoutError):ipc.receive(Drip(),deadline=time.monotonic()+.025)


def test_ipc_protocol5_keeps_numpy_payload_out_of_pickle_stream():
    import numpy as np
    from course_navigation import stage4_grid_worker as ipc
    payload=np.arange(250000,dtype=np.uint8)
    packed,buffers=ipc._serialize({'map':payload})
    assert len(packed)<1024
    assert sum(memoryview(item).nbytes for item in buffers)==payload.nbytes
    class MemorySocket:
        def __init__(self):self.data=bytearray()
        def settimeout(self,_):pass
        def sendall(self,data):self.data.extend(data)
        def recv_into(self,destination):
            count=min(len(destination),len(self.data))
            destination[:count]=self.data[:count]
            del self.data[:count]
            return count
    wire=MemorySocket()
    ipc.send(wire,{'map':payload})
    restored=ipc.receive(wire,deadline=time.monotonic()+1.)
    assert np.array_equal(restored['map'],payload)


def test_ipc_shared_buffer_response_preserves_geometry_and_closes_fds():
    import gc,socket
    from course_navigation.stage4_geometry import ClearanceGrid,grid_revision
    from course_navigation import stage4_grid_worker as ipc
    payload=grid();payload['width']=100;payload['height']=100;payload['data']=[0]*10000
    payload['data'][5050]=254
    frozen,_=grid_revision(payload,'costmap');checker=ClearanceGrid(frozen)
    before=checker.clearance([-1.,-.75,0.])
    left,right=socket.socketpair()
    try:
        ipc.send(left,('result',('entry',checker)),shared_buffers=True)
        timings={};restored=ipc.receive(right,deadline=time.monotonic()+1.,phase_times=timings)
        received=restored[1][1]
        assert received.clearance([-1.,-.75,0.])==before
        assert received.points.shape==checker.points.shape
        assert received.points is received.tree.data
        assert timings['shared_buffer_stage_s']>=0
        shared_fd_targets=[os.readlink(f'/proc/self/fd/{fd}') for fd in os.listdir('/proc/self/fd')
                           if fd.isdigit() and os.path.exists(f'/proc/self/fd/{fd}')]
        assert any(target.startswith('/memfd:b2w-stage4-grid') for target in shared_fd_targets)
        del received,restored
        gc.collect()
        remaining_targets=[]
        for fd in os.listdir('/proc/self/fd'):
            try:remaining_targets.append(os.readlink(f'/proc/self/fd/{fd}'))
            except FileNotFoundError:pass
        assert not any(target.startswith('/memfd:b2w-stage4-grid') for target in remaining_targets)
    finally:
        left.close();right.close()


def test_ipc_receive_reports_read_and_decode_phases():
    from course_navigation import stage4_grid_worker as ipc
    class MemorySocket:
        def __init__(self):self.data=bytearray()
        def settimeout(self,_):pass
        def sendall(self,data):self.data.extend(data)
        def recv_into(self,destination):
            count=min(len(destination),len(self.data));destination[:count]=self.data[:count]
            del self.data[:count];return count
    wire=MemorySocket();ipc.send(wire,('ready',));timings={}
    assert ipc.receive(wire,deadline=time.monotonic()+1.,phase_times=timings)==('ready',)
    assert timings['read_s']>=0 and timings['decode_s']>=0


def test_ipc_shared_transport_handles_zero_length_numpy_buffer():
    import numpy as np
    from course_navigation import stage4_grid_worker as ipc
    class MemorySocket:
        def __init__(self):self.data=bytearray()
        def settimeout(self,_):pass
        def sendall(self,data):self.data.extend(data)
        def recv_into(self,destination):
            count=min(len(destination),len(self.data));destination[:count]=self.data[:count]
            del self.data[:count];return count
    wire=MemorySocket();source=np.empty(0,dtype=np.uint8)
    ipc.send(wire,source,shared_buffers=True)
    restored=ipc.receive(wire,deadline=time.monotonic()+1.)
    assert restored.size==0


@pytest.mark.parametrize('failure',['truncated_control','short_header'])
def test_ipc_closes_received_descriptor_when_header_fails(failure):
    import array,socket
    from course_navigation import stage4_grid_worker as ipc
    fd=os.memfd_create('ipc-test-fd',os.MFD_CLOEXEC)
    class BrokenHeader:
        def settimeout(self,_):pass
        def recvmsg(self,_size,_control_size):
            rights=array.array('i',[fd]).tobytes()
            flags=getattr(socket,'MSG_CTRUNC',0) if failure=='truncated_control' else 0
            header=ipc._FRAME.pack(0,0,ipc._FLAG_SHARED,0)
            if failure=='short_header':header=header[:3]
            return header,[(socket.SOL_SOCKET,socket.SCM_RIGHTS,rights)],flags,None
        def recv_into(self,_):return 0
    with pytest.raises((EOFError,ValueError)):
        ipc.receive(BrokenHeader(),deadline=time.monotonic()+1.)
    with pytest.raises(OSError):os.fstat(fd)


def test_ipc_deadline_is_rechecked_between_out_of_band_buffer_sends():
    import numpy as np
    from course_navigation import stage4_grid_worker as ipc
    class SlowSocket:
        def settimeout(self,value):self.timeout=value
        def sendall(self,_):time.sleep(.015)
    with pytest.raises(TimeoutError,match='grid_worker_deadline'):
        ipc.send(SlowSocket(),np.arange(128,dtype=np.uint8),deadline=time.monotonic()+.025)


def test_ipc_rejects_excessive_oob_buffer_count_before_reading_payload():
    import struct
    from course_navigation import stage4_grid_worker as ipc
    class HeaderOnly:
        def __init__(self):self.header=bytearray(ipc._FRAME.pack(0,65,0,0))
        def settimeout(self,_):pass
        def recv_into(self,destination):
            count=min(len(destination),len(self.header))
            destination[:count]=self.header[:count];del self.header[:count]
            return count
    with pytest.raises(ValueError,match='grid_ipc_oversized'):
        ipc.receive(HeaderOnly(),deadline=time.monotonic()+1.)


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
        submit(w);until(lambda:g.failure is not None and not g.prepared_status()['grids'])
        assert 'grid_worker_' in g.failure and not g.prepared_status()['grids']
        assert Path(cfg['inhibit_path']).exists()
    finally:assert w.close()
