from pathlib import Path
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))

from run_warehouse_route import RosGoalTransport


def test_shared_transport_context_runs_owner_progress_and_never_shuts_it_down():
    advanced = []
    transport = RosGoalTransport(reuse_existing_context=True,
                                 progress_callback=lambda: advanced.append(True))

    class Context:
        shutdowns = 0
        def ok(self): return True
        def shutdown(self): self.shutdowns += 1
        def spin_once(self, node, timeout_sec): pass

    class Node:
        destroyed = False
        def destroy_node(self): self.destroyed = True

    transport.rclpy = Context()
    transport.node = Node()
    transport.initialized = False
    transport.pending = transport.foreign_goal = False
    transport.evidence = None
    transport.speed_recorder = None
    transport.spin(10**20)
    cleanup = transport.close()

    assert advanced == [True]
    assert transport.node.destroyed
    assert transport.rclpy.shutdowns == 0
    assert cleanup['passed'] is True
