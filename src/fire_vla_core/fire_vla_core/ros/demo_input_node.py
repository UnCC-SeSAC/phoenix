from __future__ import annotations

import json
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VLADemoInputNode(Node):
    """Publishes one deterministic Mission and semantic scene for smoke tests."""

    def __init__(self) -> None:
        super().__init__("vla_demo_input")
        self.declare_parameter("person_x", 2.0)
        self.declare_parameter("person_y", 0.0)
        self.declare_parameter("fire_x", 1.0)
        self.declare_parameter("fire_y", 0.0)
        self.declare_parameter("publish_period_sec", 0.5)
        self.declare_parameter("repeat_observation", True)

        self.mission_pub = self.create_publisher(String, "/vla/mission", 10)
        self.observation_pub = self.create_publisher(
            String,
            "/vla/perception_observation",
            10,
        )
        self.mission_sent = False
        self.timer = self.create_timer(
            max(0.1, float(self.get_parameter("publish_period_sec").value)),
            self._tick,
        )

    def _tick(self) -> None:
        if not self.mission_sent:
            mission = String()
            mission.data = json.dumps(
                {
                    "mission_id": "demo_mission",
                    "text": "인명을 우선 확인하되, 접근 경로를 막는 소형 화점은 먼저 제거해.",
                },
                ensure_ascii=False,
            )
            self.mission_pub.publish(mission)
            self.mission_sent = True

        observed_at = datetime.now(timezone.utc).isoformat()
        observation = String()
        observation.data = json.dumps(
            {
                "timestamp": observed_at,
                "frame_id": "map",
                "frame_valid": True,
                "detector_healthy": True,
                "detections": [
                    {
                        "entity_id": "person_01",
                        "class_name": "person",
                        "confidence": 0.95,
                        "map_position": {
                            "x": float(self.get_parameter("person_x").value),
                            "y": float(self.get_parameter("person_y").value),
                            "yaw": 0.0,
                        },
                    },
                    {
                        "entity_id": "fire_01",
                        "class_name": "fire",
                        "confidence": 0.92,
                        "map_position": {
                            "x": float(self.get_parameter("fire_x").value),
                            "y": float(self.get_parameter("fire_y").value),
                            "yaw": 0.0,
                        },
                        "size": "SMALL",
                        "blocks_route_to": "person_01",
                    },
                ],
            },
            ensure_ascii=False,
        )
        self.observation_pub.publish(observation)

        if not bool(self.get_parameter("repeat_observation").value):
            self.timer.cancel()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VLADemoInputNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
