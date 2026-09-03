"""firefighter_ui의 시맨틱 맵 마커(모양/상태별 색)를 눈으로 확인하기 위한 목업.

★ VLA 의사결정 파이프라인을 흉내 내지 않습니다 — vla_orchestrator를 대체해
  /vla/status에 완성된 world_model 스냅샷을 그대로 반복 발행할 뿐입니다.
  person/fire 상태(REPORTED, EXTINGUISHED 등)는 실제로는 report/extinguish
  액션이 성공해야 바뀌는데, 이 목업은 그 전이를 거치지 않고 처음부터 7개
  상태를 동시에 박아 넣습니다 — 마커 렌더링만 확인하는 용도입니다.
"""

from __future__ import annotations

import json
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _pose(x: float, y: float, yaw: float = 0.0) -> dict:
    return {"x": x, "y": y, "yaw": yaw}


def _person(entity_id: str, x: float, y: float, state: str) -> dict:
    return {
        "id": entity_id,
        "position": _pose(x, y),
        "confidence": 0.9,
        "state": state,
        "reported": state == "REPORTED",
        "first_seen": "",
        "last_seen": "",
    }


def _fire(entity_id: str, x: float, y: float, state: str) -> dict:
    return {
        "id": entity_id,
        "position": _pose(x, y),
        "confidence": 0.9,
        "size": "SMALL",
        "state": state,
        "blocks_route_to": None,
        "spray_count": 0,
        "robot_within_spray_range": False,
        "first_seen": "",
        "last_seen": "",
        "verification_started_at": None,
        "verification_valid_observations": 0,
    }


def build_demo_world() -> dict:
    return {
        "mission": {"id": "map_marker_demo", "text": "맵 마커 확인용 목데이터", "status": "RUNNING"},
        "exploration_status": "COMPLETED",
        "perception_ready": True,
        "robot": {
            "pose": _pose(1.0, 1.0, 0.4),
            "pose_updated_at": "",
            "navigation_status": "IDLE",
            "home_pose": _pose(0.0, 0.0, 0.0),
        },
        "people": [
            _person("person_01", 2.0, 3.0, "DETECTED"),
            _person("person_02", -2.0, 3.0, "REPORTED"),
            _person("person_03", -2.0, -2.0, "LOST"),
        ],
        "fires": [
            _fire("fire_01", 3.0, 0.5, "ACTIVE"),
            _fire("fire_02", 3.0, -2.0, "PENDING_VERIFICATION"),
            _fire("fire_03", 0.0, -3.0, "EXTINGUISHED"),
            _fire("fire_04", -3.5, 0.5, "INACCESSIBLE"),
        ],
        "current_action": None,
        "last_action": None,
        "pending_action_ids": [],
        "unexplored_zones": [],
        "recent_events": [],
    }


class VLAStatusDemoNode(Node):
    def __init__(self) -> None:
        super().__init__("vla_status_demo")
        self.declare_parameter("status_topic", "/vla/status")
        self.declare_parameter("publish_period_sec", 1.0)
        self._pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self._world = build_demo_world()
        self.create_timer(
            max(0.2, float(self.get_parameter("publish_period_sec").value)),
            self._tick,
        )
        self.get_logger().info(
            "[맵 마커 목업] person/fire 7가지 상태를 고정 발행합니다 — "
            "VLA 의사결정을 흉내 내지 않습니다"
        )

    def _tick(self) -> None:
        payload = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "world_model": self._world,
            "decision": None,
            "validation": None,
            "submission": None,
            "blocked_reason": "",
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VLAStatusDemoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
