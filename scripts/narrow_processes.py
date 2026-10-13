"""Clean only processes with our unique inherited token and unchanged PID identity."""
import os
from pathlib import Path
import signal
import time

class OwnedProcesses:
    def __init__(self, token):
        self.marker = ('B2W_NARROW_OWNER='+token).encode()
        self.seen = {}
        self.last_scan = -float('inf')

    def scan(self):
        live = {}
        for path in Path('/proc').glob('[0-9]*'):
            try:
                stat = (path/'stat').read_text().rsplit(')',1)[1].split()
                if self.marker not in (path/'environ').read_bytes().split(b'\0'):
                    continue
                pid = int(path.name); identity = stat[19]
                self.seen[pid] = identity
                if stat[0] != 'Z': live[pid] = identity
            except (OSError, ValueError, IndexError):
                continue
        return live

    def observe(self):
        # /proc scans take ~12 ms on the host: never put one before every TF callback.
        if time.monotonic()-self.last_scan >= 1:
            self.last_scan=time.monotonic()
            self.scan()

    def send(self, sig):
        for pid, identity in self.scan().items():
            try:
                current = Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[19]
                if current == identity: os.kill(pid,sig)
            except (ProcessLookupError, FileNotFoundError): pass

    def cleanup(self, process):
        self.scan()
        if process.poll() is None: process.send_signal(signal.SIGINT)
        for sig, duration in [(None,10),(signal.SIGTERM,5),(signal.SIGKILL,3)]:
            if sig is not None: self.send(sig)
            until = time.monotonic()+duration
            while time.monotonic()<until:
                process.poll()
                if not self.scan(): break
                time.sleep(.1)
            if not self.scan(): break
        process.poll()
        return dict(owned_pid_start_ticks=self.seen, remaining=self.scan(),
                    passed=not self.scan(), launch_returncode=process.poll())
