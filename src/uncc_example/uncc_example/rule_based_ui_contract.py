"""Stable JSON contract between the Rule-based runtime and Firefighter UI."""

from __future__ import annotations

import copy
import json
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any


SCHEMA_VERSION = 1
MODE = 'RULE_BASED'
ALLOWED_COMMANDS = {'START', 'STOP'}


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def parse_mission_command(raw: str) -> dict[str, str]:
    """Parse the shared UI mission envelope with a Rule-based command text."""
    try:
        payload = json.loads(raw)
    except (TypeError, json.JSONDecodeError) as exc:
        raise ValueError('mission은 JSON object여야 합니다.') from exc

    if not isinstance(payload, dict):
        raise ValueError('mission은 JSON object여야 합니다.')
    if set(payload) != {'mission_id', 'text'}:
        raise ValueError('mission_id와 text 두 필드가 필요합니다.')

    mission_id = payload['mission_id']
    text = payload['text']
    if not isinstance(mission_id, str) or not mission_id.strip():
        raise ValueError('mission_id는 비어 있지 않은 문자열이어야 합니다.')
    if not isinstance(text, str):
        raise ValueError('text는 문자열이어야 합니다.')

    command = text.strip().upper()
    if command not in ALLOWED_COMMANDS:
        raise ValueError('Rule-based mission text는 START 또는 STOP이어야 합니다.')
    return {'mission_id': mission_id.strip(), 'command': command}


@dataclass
class RuleBasedStatus:
    """Mutable adapter-side cache serialized as one UI snapshot."""

    mission_state: str = 'UNKNOWN'
    manual_stop: bool = False
    # start_mission/stop_mission 서비스가 아직 discovery 전이면 False —
    # UI가 이걸 보고 버튼을 눌러도 되는 시점을 판단한다.
    mission_ready: bool = False
    target_type: str = 'idle'
    current_target: dict[str, Any] | None = None
    found_targets: list[dict[str, Any]] = field(default_factory=list)
    battery_raw: int | None = None
    navigation_status: str = 'UNKNOWN'
    exploration_status: str = 'UNKNOWN'
    suppression_status: str = 'IDLE'
    last_command: dict[str, Any] | None = None
    blocked_reason: str = ''

    def snapshot(self) -> dict[str, Any]:
        # RETURNING_TO_CHARGE 는 배터리 부족과 stop_mission(수동 정지) 둘 다에서
        # 쓰는 내부 상태값이라 그대로 두고, UI에 보여줄 라벨만 여기서 구분한다.
        display_state = (
            'STOP_EXPLORING'
            if self.mission_state == 'RETURNING_TO_CHARGE' and self.manual_stop
            else self.mission_state
        )
        targets = copy.deepcopy(self.found_targets)
        counts = {
            'person': sum(
                target.get('type', '').startswith('person')
                for target in targets
            ),
            'fire': sum(
                target.get('type', '').startswith('fire')
                for target in targets
            ),
        }
        return {
            'schema_version': SCHEMA_VERSION,
            'mode': MODE,
            'timestamp': utc_now_iso(),
            'mission': {
                'state': display_state,
                'ready': self.mission_ready,
                'target_type': self.target_type,
                'current_target': copy.deepcopy(self.current_target),
                'last_command': copy.deepcopy(self.last_command),
            },
            'robot': {
                'battery_raw': self.battery_raw,
                'navigation_status': self.navigation_status,
            },
            'exploration': {
                'status': self.exploration_status,
            },
            'detections': {
                'targets': targets,
                'counts': counts,
            },
            'suppression': {
                'status': self.suppression_status,
            },
            'blocked_reason': self.blocked_reason,
        }
