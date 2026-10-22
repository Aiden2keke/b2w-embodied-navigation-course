#!/usr/bin/python3
"""Bounded offline heartbeat I/O probe, no ROS/Gazebo or control permissions.

Compare fsync versus transient atomic replacement on the same filesystem.
Recorder mode reproduces file polling, /proc scanning and approximate journal
traffic only; it does NOT reproduce ROS callbacks or full live contention.
"""
import argparse
import json
import multiprocessing as mp
import os
from pathlib import Path
import resource
import tempfile
import time


def recorder(directory, stop, ready, results):
    started = time.monotonic(); cpu = time.process_time(); scans = []
    with (directory/'recorder.jsonl').open('x') as journal:
        ready.set()
        while not stop.wait(.1):
            begin = time.perf_counter()
            for p in Path('/proc').glob('[0-9]*'):
                try:
                    (p/'environ').read_bytes(); (p/'stat').read_text()
                except OSError:
                    pass
            for p in directory.glob('*.json'):
                try:
                    json.loads(p.read_bytes())
                except (OSError, ValueError):
                    pass
            # Original recorder produced ~140 KiB/s. Buffered, no fsync.
            journal.write(json.dumps({'wall': time.monotonic(), 'payload': 'x'*14336})+'\n')
            journal.flush()
            scans.append(time.perf_counter()-begin)
    results.send(dict(scans=len(scans),max_scan_s=max(scans,default=0),
                      cpu_s=time.process_time()-cpu,wall_s=time.monotonic()-started))
    results.close()


def write_sample(path, payload, durable):
    sections = {}; start = time.perf_counter(); cpu = time.thread_time()
    def mark(name, fn):
        wall0 = time.perf_counter(); cpu0 = time.thread_time()
        value = fn()
        sections[name] = dict(wall_s=time.perf_counter()-wall0,
                              thread_cpu_s=time.thread_time()-cpu0)
        return value
    data = mark('json', lambda: json.dumps(payload,sort_keys=True,allow_nan=False).encode())
    assert len(data)<=8192
    f = mark('temporary_file', lambda: tempfile.NamedTemporaryFile(
        dir=path.parent,prefix='.probe-',delete=False))
    temporary = Path(f.name)
    try:
        mark('write', lambda: f.write(data))
        mark('flush', f.flush)
        if durable:
            mark('fsync', lambda: os.fsync(f.fileno()))
        mark('close', f.close)
        mark('rename', lambda: os.replace(temporary,path))
    finally:
        f.close(); temporary.unlink(missing_ok=True)
    return dict(sections=sections,wall_s=time.perf_counter()-start,
                thread_cpu_s=time.thread_time()-cpu)


def stats(samples):
    def distribution(values):
        v=sorted(values)
        return dict(median_s=v[len(v)//2],p95_s=v[int((len(v)-1)*.95)],max_s=v[-1])
    result={k:distribution([s[k] for s in samples]) for k in ('wall_s','thread_cpu_s')}
    result['sections']={section:{k:distribution([s['sections'][section][k] for s in samples])
        for k in ('wall_s','thread_cpu_s')} for section in samples[0]['sections']}
    result['over_50ms']=sum(s['wall_s']>=.05 for s in samples)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--payload',type=Path,required=True)
    parser.add_argument('--samples',type=int,default=60)
    args=parser.parse_args()
    if not 10<=args.samples<=80: parser.error('samples must be10..80')
    args.output.mkdir(mode=0o700,parents=False,exist_ok=False)
    payload=json.loads(args.payload.read_text()); all_results=[]
    # ABBA ordering exposes some run-order drift; one bounded benchmark invocation.
    for index, enabled in enumerate((False,True,True,False)):
        directory=args.output/str(index);directory.mkdir(mode=0o700)
        stop=mp.Event();ready=mp.Event();receiver,sender=mp.Pipe(duplex=False)
        process=None; recorder_result=None; usage0=resource.getrusage(resource.RUSAGE_SELF)
        samples={True:[],False:[]}
        try:
            if enabled:
                process=mp.Process(target=recorder,args=(directory,stop,ready,sender));process.start()
                if not ready.wait(2):raise RuntimeError('Recorder surrogate not ready')
            deadline=time.monotonic()+12
            for sequence in range(args.samples):
                if time.monotonic()>deadline:raise RuntimeError('Phase12s cap reached')
                begin=time.monotonic()
                payload['sequence']=sequence+1;payload['wall']=begin
                for durable in ((True,False) if sequence%2==0 else (False,True)):
                    samples[durable].append(write_sample(directory/('durable.json' if durable else 'transient.json'),payload,durable))
                time.sleep(max(0,.02-(time.monotonic()-begin)))
        finally:
            stop.set()
            if process is not None:
                process.join(2)
                if process.is_alive():process.terminate();process.join(2)
                assert not process.is_alive()
                if receiver.poll(1):recorder_result=receiver.recv()
            receiver.close();sender.close()
        usage1=resource.getrusage(resource.RUSAGE_SELF)
        all_results.append(dict(phase=index,recorder_enabled=enabled,recorder=recorder_result,
            recorder_pid=process.pid if process else None,recorder_exitcode=process.exitcode if process else None,
            context_switches=dict(voluntary=usage1.ru_nvcsw-usage0.ru_nvcsw,involuntary=usage1.ru_nivcsw-usage0.ru_nivcsw),
            summary={str(k):stats(v) for k,v in samples.items()},samples={str(k):v for k,v in samples.items()}))
    result=dict(kind='bounded_offline_io_probe',payload=str(args.payload),samples_per_variant_per_phase=args.samples,
                filesystem=str(args.output.resolve()),phases=all_results,
                limitation='Recorder surrogate only: file/proc polling and journal traffic, no ROS callbacks, physics or calibrated scheduler load')
    (args.output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    for phase in all_results:
        print('phase',phase['phase'],'recorder',phase['recorder_enabled'],
              json.dumps({k:v['wall_s'] for k,v in phase['summary'].items()}),'recorder',phase['recorder'])


if __name__=='__main__':main()
