import json
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus

from uncc_example.rule_based_ui_adapter import summarize_goal_status
from uncc_example.rule_based_ui_contract import (
    RuleBasedStatus,
    parse_mission_command,
)


def test_status_snapshot_exposes_rule_based_sections():
    status = RuleBasedStatus(
        mission_state='FIRE_DETECTED',
        target_type='fire',
        current_target={'frame_id': 'map', 'x': 1.2, 'y': -0.3},
        found_targets=[
            {'type': 'person_confirmed', 'x': 0.5, 'y': 0.0},
            {'type': 'fire_unvisited', 'x': 1.2, 'y': -0.3},
        ],
        battery_raw=7200,
        navigation_status='RUNNING',
        exploration_status='IDLE',
        suppression_status='IDLE',
    )

    payload = status.snapshot()

    assert payload['schema_version'] == 1
    assert payload['mode'] == 'RULE_BASED'
    assert payload['mission']['state'] == 'FIRE_DETECTED'
    assert payload['robot']['navigation_status'] == 'RUNNING'
    assert payload['detections']['counts'] == {'person': 1, 'fire': 1}
    assert payload['suppression']['status'] == 'IDLE'


@pytest.mark.parametrize('text, command', [('START', 'START'), (' stop ', 'STOP')])
def test_mission_command_reuses_ui_envelope(text, command):
    result = parse_mission_command(json.dumps({
        'mission_id': 'mission_ui_001',
        'text': text,
    }))
    assert result == {
        'mission_id': 'mission_ui_001',
        'command': command,
    }


@pytest.mark.parametrize(
    'payload',
    [
        'not-json',
        json.dumps({'mission_id': 'm1'}),
        json.dumps({'mission_id': 'm1', 'text': 'NAVIGATE'}),
        json.dumps({'mission_id': '', 'text': 'START'}),
    ],
)
def test_invalid_mission_command_is_rejected(payload):
    with pytest.raises(ValueError):
        parse_mission_command(payload)


def test_goal_status_prefers_active_goal():
    msg = SimpleNamespace(status_list=[
        SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED),
        SimpleNamespace(status=GoalStatus.STATUS_EXECUTING),
    ])
    assert summarize_goal_status(msg) == 'RUNNING'
