#!/usr/bin/python3
"""Bounded uncut X11 Gazebo-window recording. No ROS, simulator or motion launch."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def command(output,display,window_id,fps=10):
    if window_id<=0 or not display or output.suffix!='.webm':raise ValueError('Select an X11 window, DISPLAY and .webm output')
    if fps not in (10,25,30):raise ValueError('Capture FPS must be10,25 or30')
    return ['gst-launch-1.0','-e','ximagesrc',f'display-name={display}',f'xid={window_id}',
        'use-damage=false','show-pointer=false','!',f'video/x-raw,framerate={fps}/1','!',
        'videoconvert','!','vp8enc','deadline=1','cpu-used=8','threads=2',
        'target-bitrate=1500000','!','webmmux','!','filesink',f'location={output}']


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--window-id',type=lambda x:int(x,0),required=True)
    p.add_argument('--display',default=os.environ.get('DISPLAY'))
    p.add_argument('--duration',type=float,default=600.)
    p.add_argument('--owned-window-pid',type=int,help='Automation: enforce exact token-owned window PID/start identity')
    p.add_argument('--fps',type=int,choices=(10,25,30),default=10)
    p.add_argument('--execute',action='store_true')
    args=p.parse_args(argv)
    if not 1<=args.duration<=900:p.error('Duration must be in [1,900] wall seconds')
    cmd=command(args.output.resolve(),args.display,args.window_id,args.fps)
    for element in ('ximagesrc','videoconvert','vp8enc','webmmux','filesink'):
        subprocess.run(['gst-inspect-1.0',element],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
    if not args.execute:
        print(json.dumps(dict(command=cmd,recording_started=False,duration_s=args.duration)));return 0
    def valid_owned_window():
        if args.owned_window_pid is None:return
        from warehouse_ab_supervisor import identity,same_process
        nonlocal window_pin
        if window_pin is None:window_pin=identity(args.owned_window_pid)
        token=os.environ.get('B2W_NARROW_OWNER')
        if not token or not same_process(window_pin,token):raise RuntimeError('Owned window process changed/exited')
        props=subprocess.check_output(['xprop','-display',args.display,'-id',hex(args.window_id),'_NET_WM_PID'],text=True,timeout=2)
        import re
        match=re.search(r'= (\d+)',props)
        if not match or int(match[1])!=window_pin['pid']:raise RuntimeError('Capture XID ownership changed')
    window_pin=None
    valid_owned_window()
    output=args.output.resolve();output.parent.mkdir(parents=True,exist_ok=True)
    # Refuse to overwrite an old video or recording ledger.
    if output.with_suffix('.ready.json').exists() or output.with_suffix('.capture.log').exists():
        raise FileExistsError('Existing capture ledger/log')
    fd=os.open(output,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600);os.close(fd)
    log=output.with_suffix('.capture.log').open('x')
    process=None;started=time.monotonic();ready=False;status=1
    def interrupt(*_):raise KeyboardInterrupt
    previous_term=signal.signal(signal.SIGTERM,interrupt)
    try:
        process=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True)
        last_owner_check=0.
        while process.poll() is None and time.monotonic()-started<args.duration:
            if time.monotonic()-last_owner_check>.5:
                valid_owned_window();last_owner_check=time.monotonic()
            if not ready and output.stat().st_size>4096:
                ready=True
                from warehouse_ab_supervisor import identity
                output.with_suffix('.ready.json').write_text(json.dumps(dict(output=str(output),capture_process=identity(process.pid),
                    started_wall=started,ready_wall=time.monotonic(),uncut=True,fps=args.fps,window_id=args.window_id,window_process=window_pin))+'\n')
                print('VIDEO READY — start the separately authorized route only after this message',flush=True)
            if not ready and time.monotonic()-started>10:raise RuntimeError('No recorded frames within 10 wall seconds')
            time.sleep(.1)
        if process.poll() is not None and process.returncode!=0:raise RuntimeError('Capture pipeline failed; inspect '+str(log.name))
        status=0 if ready else 1
    except KeyboardInterrupt:
        status=0 if ready else 1
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)  # -e finalizes the same uncut WebM.
            try:process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                process.kill();process.wait();status=1
        log.close()
        signal.signal(signal.SIGTERM,previous_term)
    return status



def is_ready(path,since):
    """Read the owned capture's readiness ledger without signaling that process."""
    from warehouse_ab_supervisor import same_process
    try:
        row=json.loads(Path(path).read_text())
        if row.get('dual_view'):
            from run_narrow_baseline import digest
            if digest(Path(row['framing_file']))!=row['framing_sha256'] or not json.loads(Path(row['framing_file']).read_text()).get('visibility_verified'):return False
            if set(row.get('views',{}))!={'radar','warehouse'}:return False
            for item in row['views'].values():
                if not (since<=item['started_wall']<=item['ready_wall']<=time.monotonic() and Path(item['output']).stat().st_size>4096 and same_process(item['capture_process'])):return False
        return (since<=row['started_wall']<=row['ready_wall']<=time.monotonic()
            and Path(row['output']).stat().st_size>4096 and same_process(row['capture_process']))
    except (OSError,ValueError,KeyError,TypeError):return False


if __name__=='__main__':raise SystemExit(main())
