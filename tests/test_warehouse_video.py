from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from record_warehouse_video import command


def test_capture_is_one_continuous_window_pipeline_without_robot_commands():
    cmd=command(Path('/tmp/uncut.webm'),':0',123)
    assert cmd[0]=='gst-launch-1.0' and cmd.count('webmmux')==1 and 'xid=123' in cmd
    assert 'video/x-raw,framerate=10/1' in cmd and 'location=/tmp/uncut.webm' in cmd
    assert not any('ros2' in v or 'gazebo' in v or 'splitmux' in v for v in cmd)


@pytest.mark.parametrize('display,window,suffix', [('',1,'.webm'),(':0',0,'.webm'),(':0',1,'.mp4')])
def test_capture_rejects_unselected_window_and_wrong_container(display,window,suffix):
    with pytest.raises(ValueError):command(Path('/tmp/video'+suffix),display,window)


def test_stale_or_dead_capture_cannot_open_first_goal_gate(tmp_path,monkeypatch):
    import json,time
    import warehouse_ab_supervisor
    from record_warehouse_video import is_ready
    video=tmp_path/'uncut.webm';video.write_bytes(b'x'*4097)
    ready=tmp_path/'uncut.ready.json';now=time.monotonic()
    ready.write_text(json.dumps(dict(output=str(video),started_wall=now-2,ready_wall=now-1,capture_process={'pid':123,'start_ticks':1})))
    monkeypatch.setattr(warehouse_ab_supervisor,'same_process',lambda pin:True)
    assert is_ready(ready,now-3) and not is_ready(ready,now-.5)
    monkeypatch.setattr(warehouse_ab_supervisor,'same_process',lambda pin:False)
    assert not is_ready(ready,now-3)


def test_capture_can_request_real25fps_without_changing_legacy_default():
    assert 'video/x-raw,framerate=25/1' in command(Path('/tmp/dual.webm'),':1',123,25)
    with pytest.raises(ValueError):command(Path('/tmp/dual.webm'),':1',123,120)


def test_dual_gate_requires_both_live_recorders_and_pinned_framing(tmp_path,monkeypatch):
    import json,time
    import warehouse_ab_supervisor
    from record_warehouse_video import is_ready
    import g1_g5_media as media
    now=time.monotonic();views={}
    for k in ['radar','warehouse']:
        video=tmp_path/(k+'.webm');video.write_bytes(b'x'*4097);views[k]=dict(output=str(video),started_wall=now-3,ready_wall=now-2,capture_process={'pid':123 if k=='radar' else 124,'start_ticks':1})
    frame=tmp_path/'framing.json';frame.write_text(json.dumps(dict(visibility_verified=True)))
    ready=tmp_path/'dual.json';row=dict(views['warehouse'],dual_view=True,views=views,framing_file=str(frame),framing_sha256=media.digest(frame));ready.write_text(json.dumps(row))
    monkeypatch.setattr(warehouse_ab_supervisor,'same_process',lambda pin:True)
    assert is_ready(ready,now-4)
    monkeypatch.setattr(warehouse_ab_supervisor,'same_process',lambda pin:pin['pid']!=123)
    assert not is_ready(ready,now-4)
    monkeypatch.setattr(warehouse_ab_supervisor,'same_process',lambda pin:True)
    frame.write_text(json.dumps(dict(visibility_verified=False)))
    assert not is_ready(ready,now-4)
