"""Real FFmpeg regression: valid VFR WebM vs genuine truncated media."""
from pathlib import Path
import subprocess,sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import g1_g5_media as media

@pytest.fixture
def jitter_video(tmp_path):
    binary=media.ffmpeg();p=tmp_path/'fixture-only.webm'
    subprocess.run([binary,'-nostdin','-hide_banner','-v','error','-f','lavfi','-i',
      'testsrc2=size=96x64:rate=25','-t','2','-vf','settb=1/1000,setpts=PTS+24*sin(N)',
      '-c:v','libvpx','-deadline','realtime','-enc_time_base:v','1:1000','-fps_mode','passthrough',str(p)],
      check=True,capture_output=True,timeout=20)
    return binary,p

def test_valid_capture_is_not_rejected_by_decode_sink_timestamp_rounding(jitter_video):
    binary,p=jitter_video
    result=media.inspect_video(binary,p)
    assert result['decoded_frames']>=48 and result['duration_s']>=1.9 and result['black_empty'] is False

def test_genuine_corruption_still_rejects(jitter_video,tmp_path):
    binary,p=jitter_video;bad=tmp_path/'truncated.webm';bad.write_bytes(p.read_bytes()[:128])
    with pytest.raises((RuntimeError,subprocess.CalledProcessError)):
        media.inspect_video(binary,bad)


def test_retained_radar_decodes_every_frame_despite_sink_rounding():
    p=Path(__file__).resolve().parents[1]/'artifacts/video/20261028T103930Z-ac6d8126/attempt_01/radar.webm'
    if not p.exists():pytest.skip('local retained rendered capture is not distributed')
    result=media.inspect_video(media.ffmpeg(),p)
    assert result['decoded_frames']==1962 and 78.4<result['duration_s']<78.5
