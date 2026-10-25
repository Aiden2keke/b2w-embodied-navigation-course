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
