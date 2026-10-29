"""Window-only presentation/media helpers; never publish robot commands."""
import ctypes,ctypes.util,hashlib,json,math,os,re,shutil,subprocess
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
def run(argv,**kw):return subprocess.run(list(map(str,argv)),check=True,capture_output=True,text=True,timeout=kw.pop('timeout',15),**kw)
def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,value):
    p=Path(p);tmp=p.with_suffix(p.suffix+'.tmp');tmp.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n');os.replace(tmp,p)
def ffmpeg():
    candidates=[shutil.which('ffmpeg')]+list(Path.home().glob('miniconda3/envs/*/lib/python*/site-packages/imageio_ffmpeg/binaries/ffmpeg-*'))
    for p in candidates:
        if p and Path(p).is_file() and os.access(p,os.X_OK):
            if 'libx264' in run([p,'-hide_banner','-encoders']).stdout:return str(p)
    raise RuntimeError('No existing FFmpeg with libx264; no automatic install')
def world_points(points,transform):
    x,y,a=transform;c,s=math.cos(a),math.sin(a)
    return [[x+c*p[0]-s*p[1],y+s*p[0]+c*p[1],p[2] if len(p)>2 else 0.] for p in points]
def normal_window_kind(props):
    if '_NET_WM_WINDOW_TYPE_NORMAL' not in props:return None
    return 'radar' if re.search(r'rviz',props,re.I) else ('warehouse' if re.search(r'gazebo|ignition',props,re.I) else None)

def owned_windows(token,display):
    env=dict(os.environ,DISPLAY=display);ids=re.findall(r'0x[0-9a-fA-F]+',run(['xprop','-root','_NET_CLIENT_LIST'],env=env).stdout);found={}
    for wid in ids:
        try:
            props=run(['xprop','-id',wid,'_NET_WM_PID','WM_CLASS','_NET_WM_NAME','_NET_WM_WINDOW_TYPE'],env=env).stdout
            match=re.search(r'_NET_WM_PID\(CARDINAL\) = (\d+)',props)
            if not match:continue
            pid=int(match[1])
            if ('B2W_NARROW_OWNER='+token).encode() not in Path(f'/proc/{pid}/environ').read_bytes().split(b'\0'):continue
            kind=normal_window_kind(props)
            if kind:
                if kind in found:raise RuntimeError('Ambiguous owned '+kind+' windows')
                found[kind]=dict(id=int(wid,16),pid=pid,properties=props)
        except (OSError,subprocess.SubprocessError):continue
    if set(found)!={'radar','warehouse'}:raise RuntimeError('Both token-owned X11/XWayland windows not available')
    return found

def arrange(windows,display):
    lib=ctypes.CDLL(ctypes.util.find_library('X11'));lib.XOpenDisplay.argtypes=[ctypes.c_char_p];lib.XOpenDisplay.restype=ctypes.c_void_p
    d=lib.XOpenDisplay(display.encode())
    if not d:raise RuntimeError('No X11/XWayland display; native Wayland capture unsupported')
    for name in ['XDefaultScreen','XDisplayWidth','XDisplayHeight','XCloseDisplay','XFlush']:getattr(lib,name).argtypes=[ctypes.c_void_p]+([ctypes.c_int] if name in ['XDisplayWidth','XDisplayHeight'] else [])
    screen=lib.XDefaultScreen(d);w=lib.XDisplayWidth(d,screen);h=lib.XDisplayHeight(d,screen)
    lib.XMoveResizeWindow.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_int,ctypes.c_int,ctypes.c_uint,ctypes.c_uint];lib.XRaiseWindow.argtypes=[ctypes.c_void_p,ctypes.c_ulong]
    if w<1280 or h<720:lib.XCloseDisplay(d);raise RuntimeError('Display too small for two readable route windows')
    origin_x=origin_y=0
    try:
        lines=run(['xrandr','--listmonitors'],env=dict(os.environ,DISPLAY=display)).stdout.splitlines()[1:]
        chosen=next((line for line in lines if '*' in line),lines[0])
        match=re.search(r'(\d+)/\d+x(\d+)/\d+\+(\d+)\+(\d+)',chosen)
        if match:w,h,origin_x,origin_y=map(int,match.groups())
    except (OSError,subprocess.SubprocessError,IndexError):pass
    ww=min(1200,w//2-20);hh=min(900,h-100)
    if ww<600 or hh<600:lib.XCloseDisplay(d);raise RuntimeError('Primary monitor cannot fit dual readable windows')
    # Preserve the user-saved Gazebo viewport dimensions as well as its pose.
    # Only reposition that window; fit RViz beside it.
    geometry=run(['xwininfo','-display',display,'-id',hex(windows['warehouse']['id'])]).stdout
    gw=int(re.search(r'^\s*Width:\s*(\d+)',geometry,re.M)[1]);gh=int(re.search(r'^\s*Height:\s*(\d+)',geometry,re.M)[1])
    if ww+gw+40>w or gh>h-60:lib.XCloseDisplay(d);raise RuntimeError('Saved Gazebo window cannot fit beside RViz on this monitor; no automatic camera/viewport resize')
    lib.XMoveWindow.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_int,ctypes.c_int]
    # Xlib's default error handler exits the whole process on a vanished XID.
    # Keep the cleanup path reachable, even if a window closes during a request.
    errors=[];callback_type=ctypes.CFUNCTYPE(ctypes.c_int,ctypes.c_void_p,ctypes.c_void_p)
    @callback_type
    def handler(_display,_event):errors.append('window_changed');return 0
    lib.XSetErrorHandler.argtypes=[ctypes.c_void_p];lib.XSetErrorHandler.restype=ctypes.c_void_p
    lib.XSync.argtypes=[ctypes.c_void_p,ctypes.c_int]
    previous=lib.XSetErrorHandler(ctypes.cast(handler,ctypes.c_void_p))
    try:
        lib.XMoveResizeWindow(d,windows['radar']['id'],origin_x+5,origin_y+35,ww,hh)
        lib.XMoveWindow(d,windows['warehouse']['id'],origin_x+ww+25,origin_y+35)
        for kind in ['radar','warehouse']:lib.XRaiseWindow(d,windows[kind]['id'])
        windows['warehouse']['saved_viewport_size']=[gw,gh]
        lib.XSync(d,0)
    finally:
        lib.XCloseDisplay(d);lib.XSetErrorHandler(previous)
    if errors:raise RuntimeError('X11 window changed during arrangement')
    return [w,h]

def screenshot(window,display,path):
    run(['gst-launch-1.0','-q','ximagesrc',f'display-name={display}',f'xid={window}','num-buffers=1','use-damage=false','!','videoconvert','!','pngenc','!','filesink',f'location={path}'])
def export_commands(binary,directory,starts,begin,end,prefix="success"):
    if prefix not in ['success','prepare','pending','partial']:raise ValueError('Invalid export prefix')
    d=Path(directory);duration=end-begin
    if duration<=0 or any(begin<t for t in starts.values()):raise ValueError('No common capture interval')
    result=[]
    for kind in ['radar','warehouse']:
        result.append([binary,'-nostdin','-hide_banner','-loglevel','error','-n','-ss',str(begin-starts[kind]),'-i',str(d/(kind+'.webm')),'-t',str(duration),'-an','-vf','scale=960:-2,fps=25','-r','25','-c:v','libx264','-preset','ultrafast','-threads','2','-crf','20','-movflags','+faststart',str(d/(prefix+'_'+kind+'.mp4'))])
    result.append([binary,'-nostdin','-hide_banner','-loglevel','error','-n','-i',str(d/(prefix+'_radar.mp4')),'-i',str(d/(prefix+'_warehouse.mp4')),'-filter_complex','[0:v]scale=960:720:force_original_aspect_ratio=decrease,pad=960:720:(ow-iw)/2:(oh-ih)/2,setsar=1[a];[1:v]scale=960:720:force_original_aspect_ratio=decrease,pad=960:720:(ow-iw)/2:(oh-ih)/2,setsar=1[b];[a][b]hstack=inputs=2[v]','-map','[v]','-r','25','-c:v','libx264','-preset','ultrafast','-threads','2','-crf','20','-movflags','+faststart',str(d/(prefix+'_combined.mp4'))])
    return result

def inspect_video(binary,path,moving_intervals=(),deadline=None,required_colors=()):
    # Full decode, followed by one-frame/sec content screening. No ffprobe dependency.
    def budget():
        remaining=120. if deadline is None else min(120.,deadline-__import__('time').monotonic())
        if remaining<=0:raise TimeoutError('Video verification exceeded batch deadline')
        return remaining
    # A coarse null-output encoder clock rounds valid VFR capture timestamps
    # onto duplicate DTS ticks. Preserve their microsecond resolution at the
    # verification sink; do not rewrite the capture or ignore decoder errors.
    p=run([binary,'-nostdin','-v','error','-xerror','-err_detect','explode','-i',path,
           '-map','0:v:0','-enc_time_base:v','1:1000000','-f','null','-',
           '-progress','pipe:1'],timeout=budget())
    times=re.findall(r'out_time_us=(\d+)',p.stdout);frames=re.findall(r'frame=(\d+)',p.stdout)
    if p.stderr.strip() or not times or not frames or 'progress=end' not in p.stdout:raise RuntimeError('Video decode incomplete: '+str(path))
    duration=int(times[-1])/1e6
    size=(320,240) if required_colors else (96,64)
    raw=subprocess.run([binary,'-nostdin','-v','error','-i',str(path),'-vf',f'fps=1,scale={size[0]}:{size[1]}','-pix_fmt','rgb24','-f','rawvideo','pipe:1'],check=True,capture_output=True,timeout=budget()).stdout
    a=np.frombuffer(raw,np.uint8)
    if len(a)<size[0]*size[1]*3:raise RuntimeError('Empty recording')
    a=a.reshape(-1,size[1],size[0],3);means=a.mean((1,2,3));stds=a.std((1,2,3));hashes=[hashlib.sha256(x.tobytes()).hexdigest() for x in a]
    if np.any((means<2)|(stds<2)):raise RuntimeError('Black or empty video frames')
    for start,end in moving_intervals:
        hs=hashes[max(0,math.ceil(start)):min(len(hashes),math.floor(end)+1)]
        if len(hs)>=3 and len(set(hs))<2:raise RuntimeError('Frozen view during measured robot motion')
    visibility=[]
    for frame in a:
        pixels=frame.reshape(-1,3).astype(float)
        counts=[int((np.linalg.norm(pixels-np.array(color)*255,axis=1)<65).sum()) for color in required_colors]
        if counts and min(counts)<1:raise RuntimeError('Recorded overview loses waypoint visibility')
        if counts:visibility.append(counts)
    info=run([binary,'-nostdin','-hide_banner','-i',path,'-t','0.01','-f','null','-'],timeout=budget()).stderr
    if Path(path).suffix=='.mp4' and ('Video: h264' not in info or '25 fps' not in info):raise RuntimeError('MP4 is not verified25FPS H.264')
    return dict(codec='h264' if 'Video: h264' in info else 'vp8',fps=25 if '25 fps' in info else None,path=str(path),sha256=digest(path),bytes=Path(path).stat().st_size,duration_s=duration,decoded_frames=int(frames[-1]),content_samples=len(a),black_empty=False,motion_freeze_screen_passed=True,waypoint_visibility_samples=len(visibility))
