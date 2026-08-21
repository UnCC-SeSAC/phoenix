import json
from types import SimpleNamespace

from std_msgs.msg import String

from uncc_example.vla_spray_bridge_node import VLASprayBridge


class FakeLogger:
    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(json.loads(message.data))


def bridge_stub():
    bridge = SimpleNamespace(
        _active_action_id=None,
        _active_fire_id=None,
        _goal_handle=None,
        _client=SimpleNamespace(wait_for_server=lambda timeout_sec: False),
        _result_pub=FakePublisher(),
        get_logger=lambda: FakeLogger(),
    )
    bridge._publish_result = lambda action_id, fire_id, status, message: (
        VLASprayBridge._publish_result(bridge, action_id, fire_id, status, message)
    )
    return bridge


def test_unavailable_action_server_returns_correlated_failure():
    bridge = bridge_stub()
    msg = String()
    msg.data = json.dumps(
        {
            'action_id': 'action_0001',
            'fire_id': 'fire_0001',
            'command': 'SPRAY',
        }
    )

    VLASprayBridge._on_command(bridge, msg)

    assert bridge._result_pub.messages == [
        {
            'action_id': 'action_0001',
            'fire_id': 'fire_0001',
            'status': 'FAILED',
            'message': 'suppress_fire action unavailable',
        }
    ]
    assert bridge._active_action_id is None


def test_invalid_command_never_reaches_action_server():
    calls = []
    logger = FakeLogger()
    bridge = bridge_stub()
    bridge._client = SimpleNamespace(
        wait_for_server=lambda timeout_sec: calls.append(timeout_sec)
    )
    bridge.get_logger = lambda: logger
    msg = String()
    msg.data = json.dumps(
        {
            'action_id': 'action_0001',
            'fire_id': 'fire_0001',
            'command': 'MOVE',
        }
    )

    VLASprayBridge._on_command(bridge, msg)

    assert calls == []
    assert bridge._result_pub.messages == []
    assert logger.warnings
