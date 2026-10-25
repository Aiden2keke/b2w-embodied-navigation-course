"""Bounded public-message failure capture. Pure recorder; ROS imports are delayed.

Raw CDR is authoritative. JSON preserves the full decoded message; nonfinite
floats have explicit tags. Rejection reasons are native log records, never
invented fields in LocalPlanEvaluation. Windows use monotonic receipt time.
"""
import base64
from collections import Counter, deque
from datetime import datetime, timezone
from functools import partial
import hashlib
import json
import math
from pathlib import Path
import re
import resource
import time

MIB = 1024 * 1024
CONTEXT_TOPICS = {'/tf_static', '/plan', '/goal_pose', '/received_global_plan',
                 '/transformed_global_plan', '/local_costmap/costmap_raw',
                 '/global_costmap/costmap_raw', '/local_costmap/published_footprint',
                 '/global_costmap/published_footprint'}
LIMITS = dict(before_s=3., after_s=2., memory_bytes=64*MIB, disk_bytes=256*MIB,
              max_record_bytes=16*MIB, max_raw_bytes=4*MIB, max_records=4096,
              reserve_bytes=MIB)


def portable(value):
    if isinstance(value, float) and not math.isfinite(value):
        return {'nonfinite_float': repr(value)}
    if isinstance(value, dict):
        return {k: portable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, bytes)):
        return [portable(v) for v in value]
    return value


def evaluation_summary(message):
    twists = message.get('twists', [])
    invalid = [t for t in twists if isinstance(t.get('total'), (int, float)) and t['total'] < 0]
    rejected = Counter(s['name'] for t in invalid for s in t.get('scores', []) if s.get('raw_score', 0) < 0)
    return dict(all_invalid=bool(twists) and len(invalid) == len(twists), candidates=len(twists),
                invalid=len(invalid), rejecting_critics=dict(rejected), rejection_reason=None)


def derived_fields(topic, message):
    if topic == '/evaluation':
        return evaluation_summary(message)
    if topic == '/rosout' and message.get('name') == 'DWBLocalPlanner':
        match = re.fullmatch(r'\s*([0-9]+(?:\.[0-9]+)?):\s*([^/]+)/(.+)', message.get('msg', ''))
        if match:
            return {'critic_reason': dict(critic=match[2].strip(), reason=match[3], reported_fraction=float(match[1]))}
    return {}


class FailureWindow:
    @staticmethod
    def validate_limits(**overrides):
        cfg = {**LIMITS, **overrides}
        for k in ('before_s', 'after_s'):
            if not math.isfinite(cfg[k]) or not 0 <= cfg[k] <= 10:
                raise ValueError(k + ' must be finite and within 0..10 seconds')
        for k, maximum in [('memory_bytes', 256*MIB), ('disk_bytes', 1024*MIB),
                           ('max_record_bytes', 32*MIB), ('max_raw_bytes', 8*MIB),
                           ('max_records', 8192), ('reserve_bytes', MIB)]:
            if type(cfg[k]) is not int or not 0 < cfg[k] <= maximum:
                raise ValueError(k + ' is outside its positive bounded range')
        if cfg['reserve_bytes'] < 4096 or cfg['disk_bytes'] <= cfg['reserve_bytes']:
            raise ValueError('disk budget must include at least 4096 reserved metadata bytes')
        return cfg

    def __init__(self, output, **limits):
        self.cfg = self.validate_limits(**limits)
        self.output = Path(output)
        if any(self.output.iterdir()):
            raise FileExistsError('Failure capture requires an empty new output directory')
        self.file = (self.output/'failure-window.jsonl').open('xb')
        self.ring = deque()
        self.context = {}
        self.seq = self.buffer_bytes = self.disk_bytes = self.written_records = 0
        self.counts = Counter(); self.drops = Counter(); self.context_seqs = []
        self.trigger = None; self.done = False; self.stop_reason = None
        self.first_ns = self.last_ns = self.epoch = None
        self.metadata = {}; self.closed_report = None

    @property
    def buffer_count(self):
        return len(self.ring) + len(self.context)

    def drop(self, reason):
        self.drops[reason] += 1

    def set_metadata(self, value):
        # Metadata is bounded separately from the record ring, inside disk reserve.
        encoded = json.dumps(portable(value), allow_nan=False).encode()
        if len(encoded) > self.cfg['reserve_bytes']//2:
            self.drop('metadata_limit')
            return
        self.metadata = json.loads(encoded)

    def _write(self, item):
        payload = item[2]
        if self.disk_bytes + len(payload) > self.cfg['disk_bytes'] - self.cfg['reserve_bytes']:
            self.drop('disk_limit'); self.done = True; self.stop_reason = 'disk_limit'
            return
        self.file.write(payload)
        self.disk_bytes += len(payload); self.written_records += 1

    def _flush_buffer(self, now_ns):
        cutoff = now_ns - int(self.cfg['before_s']*1e9)
        ring_ids = {item[0] for item in self.ring}
        self.context_seqs = [item[0] for item in self.context.values()
                             if item[0] not in ring_ids and item[1] < cutoff]
        unique = {item[0]: item for item in [*self.context.values(), *self.ring]}
        for item in sorted(unique.values(), key=lambda x: x[0]):
            if self.done: break
            self._write(item)
        self.ring.clear(); self.context.clear(); self.buffer_bytes = 0

    def advance(self, now_ns):
        if self.done: return
        if self.trigger is not None:
            if now_ns > self.trigger['received_ns'] + int(self.cfg['after_s']*1e9):
                self.done = True; self.stop_reason = 'post_window_complete'
        else:
            cutoff = now_ns - int(self.cfg['before_s']*1e9)
            while self.ring and self.ring[0][1] < cutoff:
                self.buffer_bytes -= len(self.ring.popleft()[2])

    def ingest(self, topic, msg_type, message, cdr, *, received_ns, utc_ns,
               sim_stamp, epoch, publisher_gid=None):
        self.advance(received_ns)
        if self.done: return
        if self.last_ns is not None and received_ns < self.last_ns:
            raise ValueError('monotonic receipt time regressed')
        self.last_ns = received_ns
        if self.first_ns is None: self.first_ns = received_ns
        if self.epoch is not None and epoch != self.epoch:
            self.buffer_bytes -= sum(len(item[2]) for item in self.context.values())
            self.context.clear()
        self.epoch = epoch
        self.seq += 1; self.counts[topic] += 1
        if len(cdr) > self.cfg['max_raw_bytes']:
            self.drop('raw_message_limit'); return
        derived = derived_fields(topic, message)
        record = dict(schema_version=1, seq=self.seq, topic=topic, type=msg_type,
                      receipt=dict(monotonic_ns=received_ns, utc_ns=utc_ns,
                                   sim_stamp=sim_stamp, epoch=epoch, publisher_gid=publisher_gid),
                      source_message=message, cdr_base64=base64.b64encode(cdr).decode('ascii'),
                      cdr_sha256=hashlib.sha256(cdr).hexdigest(), derived=derived)
        payload = (json.dumps(portable(record), allow_nan=False, separators=(',', ':'))+'\n').encode()
        if len(payload) > self.cfg['max_record_bytes']:
            self.drop('record_limit'); return
        item = (self.seq, received_ns, payload)
        if self.trigger is None and derived.get('all_invalid'):
            self.trigger = dict(seq=self.seq, received_ns=received_ns, epoch=epoch,
                                source_stamp=message.get('header', {}).get('stamp'))
            self._flush_buffer(received_ns)
        if self.trigger is not None:
            if not self.done: self._write(item)
            return
        self.ring.append(item); self.buffer_bytes += len(payload)
        if topic in CONTEXT_TOPICS:
            # Humble Python dispatch drops MessageInfo. Static transform sets
            # are separated by frame pairs, without inventing publisher IDs.
            identity = tuple(sorted((t.get('header', {}).get('frame_id', ''), t.get('child_frame_id', ''))
                                    for t in message.get('transforms', []))) if topic == '/tf_static' else publisher_gid
            key = (topic, identity)
            old = self.context.pop(key, None)
            if old: self.buffer_bytes -= len(old[2])
            self.context[key] = item; self.buffer_bytes += len(payload)
        while self.buffer_count > self.cfg['max_records'] or self.buffer_bytes > self.cfg['memory_bytes']:
            # Evict oldest retained reference; duplicated ring/context references
            # are deliberately charged twice, giving a conservative byte bound.
            oldest = min(self.context, key=lambda k: self.context[k][0]) if self.context else None
            if self.ring and (oldest is None or self.ring[0][0] <= self.context[oldest][0]):
                removed = self.ring.popleft()
            else:
                removed = self.context.pop(oldest)
            self.buffer_bytes -= len(removed[2]); self.drop('memory_or_count_limit')

    def close(self, reason='observer_stopped'):
        if self.closed_report is not None: return self.closed_report
        if self.trigger is None and self.last_ns is not None:
            self._flush_buffer(self.last_ns)
        self.file.close()
        stop = self.stop_reason or reason
        prehistory = ((self.trigger['received_ns']-self.first_ns)/1e9 if self.trigger else 0.)
        report = dict(schema_version=1, limits=self.cfg, trigger=self.trigger, stop_reason=stop,
                      received_counts=dict(self.counts), drops=dict(self.drops), written_records=self.written_records,
                      context_before_window_seqs=self.context_seqs, prehistory_available_s=prehistory,
                      retention_complete=bool(self.trigger and stop=='post_window_complete' and not self.drops
                                              and prehistory >= self.cfg['before_s']),
                      controller_private_inputs_captured=False, transport_completeness='unknown',
                      timestamp_basis='monotonic receipt window; original integer ROS stamps and clock epochs retained',
                      reason_association='native rosout records in receipt window; no per-candidate reason linkage',
                      metadata=portable(self.metadata), raw_file='failure-window.jsonl', raw_bytes=self.disk_bytes,
                      raw_sha256=file_digest(self.output/'failure-window.jsonl'))
        encoded = (json.dumps(report, allow_nan=False, separators=(',', ':'))+'\n').encode()
        if len(encoded) > self.cfg['reserve_bytes']:
            # Preserve raw records and always leave a bounded, explicitly
            # incomplete manifest even for unusually large context/count maps.
            self.drop('manifest_detail_limit')
            report.update(drops=dict(self.drops), retention_complete=False,
                          manifest_detail_omitted=True, metadata={},
                          context_before_window_count=len(self.context_seqs),
                          context_before_window_seqs=[], received_total=sum(self.counts.values()), received_counts={})
            encoded = (json.dumps(report, allow_nan=False, separators=(',', ':'))+'\n').encode()
        if len(encoded) > self.cfg['reserve_bytes']:
            raise RuntimeError('manifest exceeds reserved bytes')
        with (self.output/'failure-window-manifest.json').open('xb') as stream:
            stream.write(encoded)
        self.closed_report = report
        return report


def file_digest(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(65536), b''):
            digest.update(chunk)
    return digest.hexdigest()


def make_subscription_callback(topic, kind, receive):
    # Installed Humble rclpy Executor passes only the raw message, not MessageInfo.
    return partial(receive, topic, kind)


def record_window(args, parser, topics, parameter_nodes):
    """Attach only. No launch, command, action client or parameter mutation."""
    limits = dict(before_s=args.window_before, after_s=args.window_after,
                  memory_bytes=args.window_memory_mib*MIB, disk_bytes=args.window_disk_mib*MIB)
    try:
        FailureWindow.validate_limits(**limits)
    except ValueError as error:
        parser.error(str(error))
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rcl_interfaces.srv import ListParameters, GetParameters

    label = ''.join(c if c.isalnum() or c in '-_' else '_' for c in args.label)
    output = args.output_root/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')+'-'+label+'-dwb-window')
    output.mkdir(parents=True, exist_ok=False)
    recorder = FailureWindow(output, **limits)
    wall_started = time.monotonic_ns(); cpu_started = time.process_time()
    duration = min(args.duration or 120., 120.)  # Independent hard wall bound, including paused /clock.
    state = dict(sim_stamp=None, epoch=0)
    metadata = dict(label=args.label, read_only=True, child_processes=[], mode='dwb-failure-window',
                    duration_wall_limit_s=duration, qos='BEST_EFFORT; depth 1; transient-local only when all offers are',
                    per_message_publisher_gid='unavailable: installed Humble rclpy discards MessageInfo; graph endpoints are separate',
                    raw_message_limit_bytes=recorder.cfg['max_raw_bytes'], graph={}, runtime_parameters={}, errors=[])
    subs = {}; clients = []; futures = []; attempted = set()
    node = None; reason = 'duration'; initialized = False

    def error(text):
        if len(metadata['errors']) < 16:
            metadata['errors'].append(str(text)[:512])

    def received(topic, kind, raw):
        received_ns = time.monotonic_ns(); utc_ns = time.time_ns()
        if len(raw) > recorder.cfg['max_raw_bytes']:
            recorder.drop('raw_message_limit'); return
        message = deserialize_message(raw, get_message(kind))
        value = message_to_ordereddict(message)
        if topic == '/clock':
            stamp = dict(value['clock'])
            old = state['sim_stamp']
            if old is not None and (stamp['sec'], stamp['nanosec']) < (old['sec'], old['nanosec']):
                state['epoch'] += 1
            state['sim_stamp'] = stamp
        recorder.ingest(topic, kind, value, raw, received_ns=received_ns, utc_ns=utc_ns,
                        sim_stamp=state['sim_stamp'], epoch=state['epoch'])

    def parameters(prefix, services):
        list_name = '/'+prefix+'/list_parameters'; get_name = '/'+prefix+'/get_parameters'
        if prefix in attempted or 'rcl_interfaces/srv/ListParameters' not in services.get(list_name, []) or 'rcl_interfaces/srv/GetParameters' not in services.get(get_name, []):
            return
        attempted.add(prefix)
        lc = node.create_client(ListParameters, list_name)
        gc = node.create_client(GetParameters, get_name)
        clients.extend([lc, gc])
        future = lc.call_async(ListParameters.Request()); futures.append(future)

        def listed(result):
            try:
                names = result.result().result.names
                request = GetParameters.Request(); request.names = names[:512]
                pending = gc.call_async(request); futures.append(pending)

                def got(done):
                    try:
                        metadata['runtime_parameters'][prefix] = dict(
                            names_truncated=len(names)>512, receipt_monotonic_ns=time.monotonic_ns(),
                            values={k:message_to_ordereddict(v) for k,v in zip(request.names, done.result().values)})
                        recorder.set_metadata(metadata)
                    except Exception as exc:
                        error(exc)
                pending.add_done_callback(got)
            except Exception as exc:
                error(exc)
        future.add_done_callback(listed)

    def discover():
        graph = dict(node.get_topic_names_and_types(no_demangle=False))
        # Keep only selected graph endpoints; the wider domain graph can be huge.
        for topic, kind in topics.items():
            offered = node.get_publishers_info_by_topic(topic) if kind in graph.get(topic, []) else []
            metadata['graph'][topic] = dict(types=graph.get(topic, []),
                publishers=[dict(node=p.node_name, namespace=p.node_namespace,
                                 gid=bytes(p.endpoint_gid).hex()) for p in offered[:16]],
                publishers_truncated=len(offered)>16)
            if not offered: continue
            durability = (DurabilityPolicy.TRANSIENT_LOCAL if all(
                p.qos_profile.durability==DurabilityPolicy.TRANSIENT_LOCAL for p in offered)
                else DurabilityPolicy.VOLATILE)
            if topic in subs and subs[topic][1] == durability: continue
            if topic in subs: node.destroy_subscription(subs[topic][0])
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=durability)
            sub = node.create_subscription(get_message(kind), topic,
                make_subscription_callback(topic, kind, received), qos, raw=True)
            subs[topic] = (sub, durability)
        services = dict(node.get_service_names_and_types())
        for prefix in parameter_nodes: parameters(prefix, services)
        recorder.set_metadata(metadata)

    try:
        rclpy.init(); initialized = True
        node = Node('warehouse_dwb_window_observer', enable_rosout=False, start_parameter_services=False)
        print(f'Attach-only bounded DWB capture: {output}', flush=True)
        next_discovery = 0
        while rclpy.ok() and not recorder.done and (time.monotonic_ns()-wall_started)/1e9 < duration:
            now = time.monotonic_ns()
            if now >= next_discovery:
                discover(); next_discovery = now+2_000_000_000
            rclpy.spin_once(node, timeout_sec=.02)
            recorder.advance(time.monotonic_ns())
        if recorder.done: reason = recorder.stop_reason
    except (KeyboardInterrupt, ExternalShutdownException):
        reason = 'observer_interrupted'
    except Exception as exc:
        reason = 'observer_error'; error(repr(exc))
    finally:
        for future in futures:
            if not future.done(): future.cancel()
        if node is not None: node.destroy_node()
        if initialized and rclpy.ok(): rclpy.shutdown()
        metadata.update(wall_duration_s=(time.monotonic_ns()-wall_started)/1e9,
                        process_cpu_s=time.process_time()-cpu_started,
                        process_max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                        cleanup='observer resources destroyed; no external process signals')
        recorder.set_metadata(metadata)
        report = recorder.close(reason)
    print(f'Observer stopped: {report["stop_reason"]}; trigger={report["trigger"]}; drops={report["drops"]}', flush=True)
    return 1 if reason == 'observer_error' or report['drops'] else 0
