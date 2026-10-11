from concurrent.futures import Future
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State
from course_navigation.lifecycle_readiness import LifecycleReadiness


class Client:
    def __init__(self):
        self.future = None

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.future = Future()
        return self.future

    def respond(self, state):
        response = GetState.Response()
        response.current_state.id = state
        self.future.set_result(response)


def test_action_endpoints_present_but_inactive_must_wait():
    clients = [Client(), Client(), Client()]
    ready = LifecycleReadiness(clients)
    assert not ready.active(0.)
    for client in clients:
        client.respond(State.PRIMARY_STATE_INACTIVE)
    assert not ready.active(.1)
    assert not ready.active(.6)
    for client in clients:
        client.respond(State.PRIMARY_STATE_ACTIVE)
    assert ready.active(.7)
    ready.active(1.2)
    clients[0].respond(State.PRIMARY_STATE_INACTIVE)
    for client in clients[1:]:
        client.respond(State.PRIMARY_STATE_ACTIVE)
    assert not ready.active(1.3)
