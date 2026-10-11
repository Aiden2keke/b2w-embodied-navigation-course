"""An action endpoint exists before its Nav2 lifecycle node is active."""
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState


class LifecycleReadiness:
    def __init__(self, clients):
        self.clients = clients
        self.states = [None] * len(clients)
        self.pending = [None] * len(clients)
        self.next_poll = 0.

    def active(self, wall_time):
        for i, client in enumerate(self.clients):
            if not client.service_is_ready():
                self.states[i] = None
            future = self.pending[i]
            if future is not None and future.done():
                try:
                    self.states[i] = future.result().current_state.id
                except Exception:
                    self.states[i] = None
                self.pending[i] = None
        if wall_time >= self.next_poll:
            self.next_poll = wall_time + .5
            for i, client in enumerate(self.clients):
                if client.service_is_ready() and self.pending[i] is None:
                    self.pending[i] = client.call_async(GetState.Request())
        return all(state == State.PRIMARY_STATE_ACTIVE for state in self.states)
