"""Real disk/worker regressions for complete, replayable bounded evidence."""
import json
from pathlib import Path
import sqlite3
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))


def test_cloud_payloads_and_duplicate_maps_replay_exactly(tmp_path):
    from stage4_route.journal import BinaryJournal, replay
    path = tmp_path / 'raw.sqlite3'
    journal = BinaryJournal(path, reserve_bytes=0)
    cloud = bytes(range(256)) * 8192
    for topic, data in [('/pointcloud', cloud), ('/map', b'map'), ('/map', b'map')]:
        journal.append(topic, 'sensor_msgs/msg/PointCloud2', data, {'sim': 12.5})
    result = journal.close()
    assert result['complete'] is True
    # Payload metrics count every uncompressed event, even when the payload
    # table deduplicates repeated bytes; they are not on-disk file sizes.
    assert result['accepted_payload_bytes'] == len(cloud) + 2 * len(b'map')
    assert result['committed_payload_bytes'] == len(cloud) + 2 * len(b'map')
    rows = list(replay(path))
    assert [r['payload'] for r in rows] == [cloud, b'map', b'map']
    assert [r['sequence'] for r in rows] == [1, 2, 3]
    assert rows[0]['metadata']['sim'] == 12.5
    with sqlite3.connect(path) as conn:
        assert conn.execute('select count(*) from payloads').fetchone()[0] == 2


def test_oversized_message_latches_failure_and_prevents_success(tmp_path):
    from stage4_route.journal import BinaryJournal
    journal = BinaryJournal(tmp_path / 'raw.sqlite3', queue_bytes=64, reserve_bytes=0)
    with pytest.raises(ValueError, match='queue'):
        journal.append('/cloud', 'sensor_msgs/msg/PointCloud2', b'x' * 65, {})
    assert journal.close()['complete'] is False
    assert journal.failure == 'evidence_queue_limit'


def test_disk_budget_failure_is_reported_by_close(tmp_path):
    from stage4_route.journal import BinaryJournal, replay
    path = tmp_path / 'raw.sqlite3'
    journal = BinaryJournal(path, max_bytes=32768, reserve_bytes=0)
    journal.append('/map', 'nav_msgs/msg/OccupancyGrid', b'payload' * 10000, {})
    result = journal.close()
    assert result['complete'] is False
    assert result['failure'] == 'evidence_disk_limit'
    with pytest.raises(ValueError, match='incomplete'):
        list(replay(path))


def test_tampered_payload_and_nonfinite_metadata_rejected(tmp_path):
    from stage4_route.journal import BinaryJournal, replay
    path = tmp_path / 'raw.sqlite3'
    journal = BinaryJournal(path, reserve_bytes=0)
    journal.append('/map', 'nav_msgs/msg/OccupancyGrid', b'original', {})
    assert journal.close()['complete']
    with sqlite3.connect(path) as conn:
        conn.execute("update payloads set data=x'000102'")
    with pytest.raises(ValueError, match='payload'):
        list(replay(path))
    bad = BinaryJournal(tmp_path / 'invalid.sqlite3', reserve_bytes=0)
    with pytest.raises(ValueError):
        bad.append('/map', 'nav_msgs/msg/OccupancyGrid', b'x', {'sim': float('nan')})
    assert not bad.close()['complete']


def test_existing_journal_is_never_overwritten(tmp_path):
    from stage4_route.journal import BinaryJournal
    path = tmp_path / 'raw.sqlite3'
    path.write_bytes(b'preserved')
    with pytest.raises(FileExistsError):
        BinaryJournal(path, reserve_bytes=0)
    assert path.read_bytes() == b'preserved'


def test_realistic_clock_burst_flushes_without_per_message_fsync_backlog(tmp_path):
    from stage4_route.journal import BinaryJournal, replay
    journal = BinaryJournal(tmp_path / 'raw.sqlite3', reserve_bytes=0)
    for i in range(2400):
        journal.append('/clock', 'rosgraph_msgs/msg/Clock', i.to_bytes(8, 'little'), {'sim': i*.0025})
    assert journal.close(timeout=2.)['complete']
    assert len(list(replay(journal.path))) == 2400


def test_capture_sized_cloud_burst_closes_and_replays_losslessly(tmp_path):
    from stage4_route.journal import BinaryJournal, replay
    path = tmp_path / 'raw.sqlite3'
    journal = BinaryJournal(path, reserve_bytes=0)
    # The host capture's PointCloud2 payloads are 1,536,193 bytes each.
    base = bytes(range(256)) * 6000 + bytes(range(193))
    clouds = [base[:-1] + bytes((i,)) for i in range(6)]
    for i, cloud in enumerate(clouds):
        journal.append('/pointcloud', 'sensor_msgs/msg/PointCloud2', cloud, {'sample': i})
    result = journal.close(timeout=10.)
    assert result['complete'] is True
    rows = list(replay(path))
    assert [row['payload'] for row in rows] == clouds


def test_capture_timing_stress_is_bounded_and_replays_exact_records(tmp_path):
    import time
    from stage4_route.journal import BinaryJournal, replay

    path = tmp_path / 'raw.sqlite3'
    journal = BinaryJournal(path, reserve_bytes=0)
    cloud_size = 1_536_193
    # The capture had 2,327 records and 27 clouds in 3.48 host seconds.
    # Replay that measured rate and mix twice against absolute monotonic
    # deadlines. This prevents a slow producer from stretching the capture.
    record_count = 2327
    cloud_indices = {round(i * (record_count - 1) / 26) for i in range(27)}
    base = bytes(range(256)) * (cloud_size // 256) + bytes(cloud_size % 256)
    started = time.monotonic()
    producer_started = None
    producer_finished = None
    expected = []
    cloud_order = sorted(cloud_indices)
    for cycle in range(2):
        cycle_start = started + cycle * 3.48
        for i in range(record_count):
            due = cycle_start + i * 3.48 / (record_count - 1)
            time.sleep(max(0, due - time.monotonic()))
            if producer_started is None:
                producer_started = time.monotonic()
            if i in cloud_indices:
                cloud_number = cycle * 27 + cloud_order.index(i)
                payload = base[:-1] + bytes((cloud_number,))
                topic, typ = '/pointcloud', 'sensor_msgs/msg/PointCloud2'
            else:
                payload = (i + cycle * record_count).to_bytes(8, 'little')
                topic, typ = '/clock', 'rosgraph_msgs/msg/Clock'
            metadata = {'cycle': cycle, 'index': i}
            journal.append(topic, typ, payload, metadata, scheduled_at=due)
            expected.append((topic, typ, payload, metadata))
    producer_finished = time.monotonic()

    result = journal.close(timeout=4.)
    expected_input_bytes = 54 * cloud_size + (2 * (record_count - 27) * 8)
    enqueue_seconds = producer_finished - producer_started
    assert result['complete'] is True
    assert result['accepted'] == 2 * record_count
    assert result['written'] == 2 * record_count
    assert result['accepted_payload_bytes'] == expected_input_bytes
    assert result['committed_payload_bytes'] == expected_input_bytes
    assert result['max_pending_bytes'] < 16 * 1024**2
    assert result['record_processing_seconds'] > 0
    assert result['max_producer_lateness_seconds'] < 0.25
    assert 6.8 <= enqueue_seconds < 7.5
    # Require at least 25% worker headroom against the measured ~7 s producer
    # schedule, so the bounded run is not merely catching up at close time.
    assert result['record_processing_seconds'] <= 0.75 * enqueue_seconds
    assert result['close_seconds'] < 4.
    assert expected_input_bytes > 64 * 1024**2
    rows = list(replay(path))
    assert len(rows) == len(expected)
    assert [(row['topic'], row['type'], row['payload'], row['metadata'])
            for row in rows] == expected


def test_removing_last_event_invalidates_complete_journal(tmp_path):
    from stage4_route.journal import BinaryJournal, replay
    path = tmp_path / 'raw.sqlite3'
    journal = BinaryJournal(path, reserve_bytes=0)
    journal.append('/clock', 'rosgraph_msgs/msg/Clock', b'a', {})
    journal.append('/clock', 'rosgraph_msgs/msg/Clock', b'b', {})
    assert journal.close()['complete']
    with sqlite3.connect(path) as conn:
        conn.execute('delete from events where sequence=2')
    with pytest.raises(ValueError, match='count'):
        list(replay(path))


def test_sqlite_overflow_pages_cannot_exceed_hard_budget(tmp_path):
    from stage4_route.journal import BinaryJournal
    journal = BinaryJournal(tmp_path / 'raw.sqlite3', max_bytes=50376461, reserve_bytes=0)
    journal.append('/cloud', 'sensor_msgs/msg/PointCloud2', b'x' * (48 * 1024**2), {})
    result = journal.close()
    assert not result['complete']
    assert result['failure'] == 'evidence_disk_limit'
    assert result['bytes'] <= journal.max_bytes


def test_pending_database_allocation_preserves_disk_reserve(tmp_path, monkeypatch):
    from stage4_route import journal as module
    from collections import namedtuple
    usage = namedtuple('usage', 'total used free')
    monkeypatch.setattr(module.shutil, 'disk_usage', lambda _: usage(1000000, 800000, 200000))
    journal = module.BinaryJournal(tmp_path / 'raw.sqlite3', reserve_bytes=100000)
    journal.append('/cloud', 'sensor_msgs/msg/PointCloud2', b'x' * 150000, {})
    assert journal.close()['failure'] == 'evidence_disk_limit'


def test_flush_timeout_cannot_leave_replayable_success(tmp_path, monkeypatch):
    from stage4_route import journal as module
    import threading
    real_connect = sqlite3.connect
    entered, release = threading.Event(), threading.Event()

    class BlockFinal(sqlite3.Connection):
        def commit(self):
            try:
                state = self.execute("SELECT value FROM state WHERE key='complete'").fetchone()
            except sqlite3.OperationalError:
                state = None
            if state and state[0] == 'true':
                entered.set()
                assert release.wait(3.)
            return super().commit()

    monkeypatch.setattr(module.sqlite3, 'connect', lambda *a, **kw: real_connect(*a, factory=BlockFinal, **kw))
    path = tmp_path / 'raw.sqlite3'
    journal = module.BinaryJournal(path, reserve_bytes=0)
    journal.append('/clock', 'rosgraph_msgs/msg/Clock', b'a', {})
    assert not journal.close(timeout=.05)['complete']
    assert entered.wait(1.)
    release.set()
    assert not journal.close(timeout=2.)['complete']
    with pytest.raises(ValueError, match='incomplete'):
        list(module.replay(path))
