"""Keep one simulation/control owner per ROS domain on this host."""
import fcntl
import os
from pathlib import Path


class SessionLease:
    def __init__(self, domain, directory='/tmp'):
        self.path = Path(directory) / f'b2w-course-{os.getuid()}-domain-{int(domain)}.lock'
        self.file = self.path.open('a+')
        try:
            fcntl.flock(self.file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.file.seek(0)
            owner = self.file.read().strip()
            self.file.close()
            raise RuntimeError(f'B2-W course already running in ROS_DOMAIN_ID={domain} '
                               f'(launch PID {owner}); stop that session with Ctrl+C first') from None
        self.file.seek(0)
        self.file.truncate()
        self.file.write(str(os.getpid()))
        self.file.flush()

    def close(self):
        self.file.close()
