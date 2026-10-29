"""Bounded asynchronous CDR evidence with deduplicated compressed payloads.

No ROS dependency. A successful close means every accepted record is committed.
Missing/corrupt/unfinished evidence can be read diagnostically but cannot pass.
"""
import hashlib
import json
from pathlib import Path
import queue
import shutil
import sqlite3
import threading
import time
import zlib


class BinaryJournal:
    def __init__(self, path, *, max_bytes=4 * 1024**3, queue_bytes=64 * 1024**2,
                 reserve_bytes=512 * 1024**2, compression_level=0):
        if min(max_bytes, queue_bytes) <= 0 or reserve_bytes < 0:
            raise ValueError('Invalid evidence budgets')
        if compression_level not in (0, 1, 2, 3):
            raise ValueError('Invalid evidence compression level')
        self.compression_level = compression_level
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.open('xb').close()
        self.seal = self.path.with_name(self.path.name + '.complete')
        if self.seal.exists():
            raise FileExistsError(self.seal)
        self.max_bytes, self.queue_limit, self.reserve = max_bytes, queue_bytes, reserve_bytes
        self.failure = None
        self.accepted = self.written = self.pending_bytes = 0
        self.closed = False
        self.lock = threading.Lock()
        self.close_lock = threading.Lock()
        self.queue = queue.Queue()
        self.ready = threading.Event()
        self.worker = threading.Thread(target=self._write, name='b2w-evidence-writer', daemon=True)
        self.worker.start()
        if not self.ready.wait(5):
            self._fail('evidence_writer_start_timeout')
            raise ValueError(self.failure)
        if self.failure:
            raise ValueError(self.failure)

    def _fail(self, reason):
        with self.lock:
            if self.failure is None:
                self.failure = reason

    def append(self, topic, type_name, payload, metadata):
        try:
            metadata_json = json.dumps(metadata, sort_keys=True, allow_nan=False, separators=(',', ':'))
            if not isinstance(payload, bytes) or not topic.startswith('/') or not type_name:
                raise ValueError('Invalid evidence record')
        except (ValueError, TypeError, AttributeError):
            self._fail('invalid_evidence_record')
            raise ValueError(self.failure)
        size = len(payload) + len(metadata_json.encode())
        with self.lock:
            if self.closed or self.failure:
                raise ValueError(self.failure or 'evidence_closed')
            if self.pending_bytes + size > self.queue_limit:
                self.failure = 'evidence_queue_limit'
                raise ValueError(self.failure)
            self.accepted += 1
            self.pending_bytes += size
            self.queue.put((self.accepted, topic, type_name, payload, metadata_json, size))

    def _write(self):
        conn = None
        try:
            conn = sqlite3.connect(self.path)
            # Memory rollback journals keep transactional writes off the reserved
            # disk space. A process crash remains incomplete (no close seal).
            conn.execute('PRAGMA journal_mode=MEMORY')
            page_size = conn.execute('PRAGMA page_size').fetchone()[0]
            initial_cap = min(self.max_bytes, shutil.disk_usage(self.path.parent).free-self.reserve-4096)
            if initial_cap < 32768:
                self._fail('evidence_disk_limit')
                return
            conn.execute('PRAGMA max_page_count=' + str(initial_cap // page_size))
            conn.executescript('''
                CREATE TABLE payloads(sha TEXT PRIMARY KEY, bytes INTEGER, data BLOB);
                CREATE TABLE events(sequence INTEGER PRIMARY KEY, topic TEXT, type TEXT,
                                    sha TEXT REFERENCES payloads(sha), metadata TEXT);
                CREATE TABLE state(key TEXT PRIMARY KEY, value TEXT);
                INSERT INTO state VALUES('complete', 'false');
                INSERT INTO state VALUES('format', '2');
                INSERT INTO state VALUES('failure', 'null');
                INSERT INTO state VALUES('records', '0');
            ''')
            conn.commit()
            self.ready.set()
            last_commit = time.monotonic()
            batch_size = 0
            while True:
                item = self.queue.get()
                if item is None:
                    break
                seq, topic, typ, payload, meta, size = item
                try:
                    if self.failure:
                        continue
                    free = shutil.disk_usage(self.path.parent).free
                    if free < self.reserve:
                        self._fail('evidence_disk_limit')
                        continue
                    # SQLite enforces the actual page allocation, including blob
                    # overflow pages, rather than estimating fixed row overhead.
                    cap = min(self.max_bytes // page_size,
                              (self.path.stat().st_blocks * 512 + free-self.reserve-4096) // page_size)
                    if cap < conn.execute('PRAGMA page_count').fetchone()[0]:
                        self._fail('evidence_disk_limit')
                        continue
                    conn.execute('PRAGMA max_page_count=' + str(cap))
                    sha = hashlib.sha256(payload).hexdigest()
                    if not conn.execute('SELECT 1 FROM payloads WHERE sha=?', (sha,)).fetchone():
                        # Full sensor streams prioritize bounded callback/flush
                        # latency. Compression is optional and benchmarked before
                        # use; even level 1 consumed ~20 ms per 1.5 MB live scan.
                        blob = zlib.compress(payload, level=self.compression_level)
                        conn.execute('INSERT INTO payloads VALUES(?,?,?)', (sha, len(payload), blob))
                    conn.execute('INSERT INTO events VALUES(?,?,?,?,?)', (seq, topic, typ, sha, meta))
                    batch_size += 1
                    if batch_size >= 100 or time.monotonic() - last_commit >= .05:
                        conn.commit()
                        self.written += batch_size
                        batch_size = 0
                        last_commit = time.monotonic()
                except sqlite3.OperationalError as exc:
                    conn.rollback()
                    batch_size = 0
                    self._fail('evidence_disk_limit' if 'full' in str(exc).lower()
                               else 'evidence_writer_error:OperationalError')
                finally:
                    with self.lock:
                        self.pending_bytes -= size
            if self.failure:
                conn.rollback()
                batch_size = 0
            else:
                conn.commit()
                self.written += batch_size
            complete = self.failure is None and self.written == self.accepted
            conn.execute('UPDATE state SET value=? WHERE key=?', (json.dumps(complete), 'complete'))
            conn.execute('UPDATE state SET value=? WHERE key=?', (json.dumps(self.failure), 'failure'))
            conn.execute('UPDATE state SET value=? WHERE key=?', (str(self.written), 'records'))
            conn.commit()
        except Exception as exc:
            self._fail('evidence_writer_error:' + type(exc).__name__)
        finally:
            self.ready.set()
            if conn is not None:
                conn.close()

    def close(self, timeout=5.):
        with self.close_lock:
            return self._close(timeout)

    def _close(self, timeout):
        with self.lock:
            if not self.closed:
                self.closed = True
                self.queue.put(None)
        self.worker.join(timeout)
        if self.worker.is_alive():
            self._fail('evidence_flush_timeout')
        elif self.failure is None and self.written == self.accepted and not self.seal.exists():
            # Only a close that observed worker termination can publish the seal.
            # A late final commit after a timeout can never become replayable.
            try:
                with self.seal.open('x') as stream:
                    json.dump({'format': 2, 'records': self.written}, stream)
            except OSError:
                self._fail('evidence_seal_error')
        return dict(complete=self.failure is None and not self.worker.is_alive()
                    and self.written == self.accepted, failure=self.failure,
                    accepted=self.accepted, written=self.written,
                    path=str(self.path), bytes=self.path.stat().st_size)


def replay(path, *, allow_incomplete=False):
    """Yield verified exact serialized messages; closed journal required by default."""
    path = Path(path).resolve()
    conn = sqlite3.connect(path.as_uri() + '?mode=ro', uri=True)
    try:
        complete = conn.execute("SELECT value FROM state WHERE key='complete'").fetchone()
        if not allow_incomplete and (not complete or complete[0] != 'true'):
            raise ValueError('incomplete evidence journal')
        version = conn.execute("SELECT value FROM state WHERE key='format'").fetchone()
        records = conn.execute("SELECT value FROM state WHERE key='records'").fetchone()
        if not allow_incomplete and version and version[0] == '2':
            try:
                seal = json.loads(path.with_name(path.name + '.complete').read_text())
                if seal != {'format': 2, 'records': int(records[0])}:
                    raise ValueError('invalid evidence close seal')
            except (OSError, TypeError, ValueError) as exc:
                raise ValueError('incomplete evidence close seal') from exc
        actual_count = conn.execute('SELECT count(*) FROM events').fetchone()[0]
        if not allow_incomplete and (not records or actual_count != int(records[0])):
            raise ValueError('evidence record count mismatch')
        previous = 0
        for seq, topic, typ, sha, meta, length, blob in conn.execute(
                'SELECT e.sequence,e.topic,e.type,e.sha,e.metadata,p.bytes,p.data '
                'FROM events e LEFT JOIN payloads p ON p.sha=e.sha ORDER BY e.sequence'):
            if seq != previous + 1:
                raise ValueError('evidence sequence gap')
            try:
                payload = zlib.decompress(blob)
            except (zlib.error, TypeError) as exc:
                raise ValueError('corrupt evidence payload') from exc
            if len(payload) != length or hashlib.sha256(payload).hexdigest() != sha:
                raise ValueError('corrupt evidence payload hash/length')
            yield dict(sequence=seq, topic=topic, type=typ, sha256=sha,
                       metadata=json.loads(meta), payload=payload)
            previous = seq
    finally:
        conn.close()
