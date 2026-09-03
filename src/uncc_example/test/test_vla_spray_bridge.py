from pathlib import Path

import json
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
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
        _control_mode='VLA',
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


def test_completed_physical_attempt_is_forwarded_for_world_model_verification():
    bridge = bridge_stub()
    bridge._active_action_id = 'action_0001'
    bridge._active_fire_id = 'fire_0001'
    bridge._finish = lambda status, message: VLASprayBridge._finish(
        bridge, status, message
    )
    wrapped = SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(
            success=False,
            message='관찰 구간 내 YOLO 감지 기록 없음',
        ),
    )

    VLASprayBridge._on_result(bridge, SimpleNamespace(result=lambda: wrapped))

    assert bridge._result_pub.messages == [{
        'action_id': 'action_0001',
        'fire_id': 'fire_0001',
        'status': 'SUCCEEDED',
        'message': '관찰 구간 내 YOLO 감지 기록 없음',
    }]


def test_suppression_servo_starts_detached():
    source = (
        Path(__file__).parents[1]
        / "uncc_example"
        / "fire_suppression_node.py"
    ).read_text(encoding="utf-8")

    assert "initial_angle=None" in source
    assert "self.servo.angle = angle" in source
    assert "self.servo.detach()" in source


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


def test_none_mode_blocks_new_suppression_goal():
    calls = []
    bridge = bridge_stub()
    bridge._control_mode = 'NONE'
    bridge._client = SimpleNamespace(
        wait_for_server=lambda timeout_sec: calls.append(timeout_sec)
    )
    msg = String()
    msg.data = json.dumps({
        'action_id': 'action_0002', 'fire_id': 'fire_0001', 'command': 'SPRAY'
    })

    VLASprayBridge._on_command(bridge, msg)

    assert calls == []
    assert bridge._result_pub.messages[-1]['message'] == 'CONTROL_MODE_MISMATCH'
