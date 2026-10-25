#!/usr/bin/python3
"""Bounded uncut X11 Gazebo-window recording. No ROS, simulator or motion launch."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def command(output,display,window_id):
    if window_id<=0 or not display or output.suffix!='.webm':raise ValueError('Select an X11 window, DISPLAY and .webm output')
    return ['gst-launch-1.0','-e','ximagesrc',f'display-name={display}',f'xid={window_id}',
        'use-damage=false','show-pointer=false','!','video/x-raw,framerate=10/1','!',
        'videoconvert','!','vp8enc','deadline=1','cpu-used=8','threads=2',
        'target-bitrate=1500000','!','webmmux','!','filesink',f'location={output}']


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--window-id',type=lambda x:int(x,0),required=True)
    p.add_argument('--display',default=os.environ.get('DISPLAY'))
    p.add_argument('--duration',type=float,default=600.)
    p.add_argument('--execute',action='store_true')
    args=p.parse_args(argv)
    if not 1<=args.duration<=900:p.error('Duration must be in [1,900] wall seconds')
    cmd=command(args.output.resolve(),args.display,args.window_id)
    for element in ('ximagesrc','videoconvert','vp8enc','webmmux','filesink'):
        subprocess.run(['gst-inspect-1.0',element],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
    if not args.execute:
        print(json.dumps(dict(command=cmd,recording_started=False,duration_s=args.duration)));return 0
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
        while process.poll() is None and time.monotonic()-started<args.duration:
            if not ready and output.stat().st_size>4096:
                ready=True
                from warehouse_ab_supervisor import identity
                output.with_suffix('.ready.json').write_text(json.dumps(dict(output=str(output),capture_process=identity(process.pid),
                    started_wall=started,ready_wall=time.monotonic(),uncut=True))+'\n')
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
        return (since<=row['started_wall']<=row['ready_wall']<=time.monotonic()
            and Path(row['output']).stat().st_size>4096 and same_process(row['capture_process']))
    except (OSError,ValueError,KeyError,TypeError):return False


if __name__=='__main__':raise SystemExit(main())
