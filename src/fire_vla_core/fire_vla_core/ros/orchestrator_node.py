from __future__ import annotations

import json
from datetime import datetime, timezone

try:
    import rclpy
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:
    rclpy = None
    Node = object
    MutuallyExclusiveCallbackGroup = None
    MultiThreadedExecutor = None

from ..adapters.mock_adapters import (
    MockNavigationAdapter,
    MockReportAdapter,
    MockResultQueue,
    MockSprayAdapter,
    MockWaitAdapter,
)
from ..dispatcher import ActionDispatcher
from ..domain import Pose2D
from ..llm import (
    MockVLABrain,
    OllamaLLMClient,
    RemoteQwenBackend,
    TransformersQwenAdapter,
)
from ..orchestrator import VLAOrchestrator
from ..resolver import TargetResolver
from ..status import VLAStatusTracker
from ..validator import ActionValidator
from ..world_model import WorldModel, WorldModelConfig
from .perception_normalizer import CanonicalPerceptionNormalizer
from .topic_bridge_navigation_adapter import TopicBridgeNavigationAdapter
from .topic_bridge_person_report_adapter import TopicBridgePersonReportAdapter
from .topic_bridge_spray_adapter import TopicBridgeSprayAdapter


def _unix_seconds_to_iso(value: float) -> str:
    return datetime.fromtimestamp(value, tz=timezone.utc).isoformat()


def create_llm_backend(
    backend: str,
    *,
    ollama_model: str,
    ollama_base_url: str,
    transformers_model_id: str,
    transformers_device: str,
    transformers_max_new_tokens: int,
    remote_qwen_endpoint: str = "http://127.0.0.1:8088/infer",
    remote_qwen_timeout_sec: float = 3.0,
):
    backend = backend.strip().lower()
    if backend == "mock":
        return MockVLABrain()
    if backend == "ollama":
        return OllamaLLMClient(
            model=ollama_model,
            base_url=ollama_base_url,
        )
    if backend == "transformers":
        if transformers_max_new_tokens <= 0:
            raise ValueError(
                "transformers_max_new_tokens는 양수여야 합니다."
            )
        return TransformersQwenAdapter(
            model_id=transformers_model_id,
            device=transformers_device,
            max_new_tokens=transformers_max_new_tokens,
        )
    if backend == "remote_qwen":
        return RemoteQwenBackend(
            endpoint=remote_qwen_endpoint,
            timeout_sec=remote_qwen_timeout_sec,
        )
    raise ValueError(
        "llm_backend는 mock, ollama, transformers, remote_qwen 중 "
        "하나여야 합니다: "
        f"{backend}"
    )


class VLAOrchestratorNode(Node):
    """Jazzy-side VLA Brain node.

    Perception and robot pose currently use distro-neutral JSON/String topics.
    Navigation can run in MOCK mode or TOPIC_BRIDGE mode against the Humble
    ``uncc_example/vla_navigation_bridge`` node.
    """

    def __init__(self) -> None:
        super().__init__("vla_orchestrator")
        self.declare_parameter("llm_backend", "mock")
        self.declare_parameter("decision_period_sec", 1.0)
        self.declare_parameter(
            "llm_model",
            "Qwen2-1.5B-Instruct-Function-Calling-v1",
        )
        self.declare_parameter("llm_base_url", "http://127.0.0.1:11434")
        self.declare_parameter(
            "transformers_model_id",
            "Qwen/Qwen2.5-1.5B-Instruct",
        )
        self.declare_parameter("transformers_device", "xpu:0")
        self.declare_parameter("transformers_max_new_tokens", 128)
        self.declare_parameter("navigation_mode", "MOCK")
        self.declare_parameter(
            "remote_qwen_endpoint",
            "http://127.0.0.1:8088/infer",
        )
        self.declare_parameter("remote_qwen_timeout_sec", 3.0)
        self.declare_parameter("report_mode", "MOCK")
        self.declare_parameter("spray_mode", "MOCK")
        self.declare_parameter("mission_topic", "/vla/mission")
        self.declare_parameter("status_topic", "/vla/status")
        self.declare_parameter(
            "perception_topic",
            "/vla/perception_observation",
        )
        self.declare_parameter("robot_pose_topic", "/vla/robot_pose_json")
        self.declare_parameter("navigation_goal_topic", "/vla/navigation_goal")
        self.declare_parameter(
            "navigation_result_topic",
            "/vla/navigation_result",
        )
        self.declare_parameter(
            "navigation_cancel_topic",
            "/vla/navigation_cancel",
        )
        self.declare_parameter("spray_command_topic", "/vla/spray_command")
        self.declare_parameter("spray_result_topic", "/vla/spray_result")
        self.declare_parameter("spray_cancel_topic", "/vla/spray_cancel")
        self.declare_parameter("person_report_topic", "/vla/person_report")
        self.declare_parameter(
            "person_report_result_topic",
            "/vla/person_report_result",
        )

        self.world = WorldModel(WorldModelConfig())
        # Remote inference is synchronous. Keep robot pose processing separate
        # so Validator freshness advances while the timer waits for HTTP.
        self.pose_callback_group = MutuallyExclusiveCallbackGroup()
        self.perception_normalizer = CanonicalPerceptionNormalizer(self.world)
        self.mock_results = MockResultQueue()
        self.status_tracker = VLAStatusTracker()

        llm_backend = str(self.get_parameter("llm_backend").value)
        llm = create_llm_backend(
            llm_backend,
            ollama_model=str(self.get_parameter("llm_model").value),
            ollama_base_url=str(self.get_parameter("llm_base_url").value),
            transformers_model_id=str(
                self.get_parameter("transformers_model_id").value
            ),
            transformers_device=str(
                self.get_parameter("transformers_device").value
            ),
            transformers_max_new_tokens=int(
                self.get_parameter(
                    "transformers_max_new_tokens"
                ).value
            ),
            remote_qwen_endpoint=str(
                self.get_parameter("remote_qwen_endpoint").value
            ),
            remote_qwen_timeout_sec=float(
                self.get_parameter("remote_qwen_timeout_sec").value
            ),
        )

        navigation_mode = str(
            self.get_parameter("navigation_mode").value
        ).upper()
        if navigation_mode == "TOPIC_BRIDGE":
            self.navigation = TopicBridgeNavigationAdapter(
                self,
                goal_topic=str(
                    self.get_parameter("navigation_goal_topic").value
                ),
                result_topic=str(
                    self.get_parameter("navigation_result_topic").value
                ),
                cancel_topic=str(
                    self.get_parameter("navigation_cancel_topic").value
                ),
            )
        elif navigation_mode == "MOCK":
            self.navigation = MockNavigationAdapter(self.mock_results)
        else:
            raise ValueError(
                "navigation_mode은 MOCK 또는 TOPIC_BRIDGE여야 합니다."
            )

        validator = ActionValidator()
        spray_mode = str(self.get_parameter("spray_mode").value).upper()
        if spray_mode == "TOPIC_BRIDGE":
            self.spray = TopicBridgeSprayAdapter(
                self,
                self.world,
                command_topic=str(
                    self.get_parameter("spray_command_topic").value
                ),
                result_topic=str(
                    self.get_parameter("spray_result_topic").value
                ),
                cancel_topic=str(
                    self.get_parameter("spray_cancel_topic").value
                ),
                max_spray_attempts=validator.max_spray_attempts,
            )
        elif spray_mode == "MOCK":
            self.spray = MockSprayAdapter(self.mock_results)
        else:
            raise ValueError(
                "spray_mode은 MOCK 또는 TOPIC_BRIDGE여야 합니다."
            )

        report_mode = str(self.get_parameter("report_mode").value).upper()
        if report_mode == "TOPIC_BRIDGE":
            self.report = TopicBridgePersonReportAdapter(
                self,
                self.world,
                report_topic=str(
                    self.get_parameter("person_report_topic").value
                ),
                result_topic=str(
                    self.get_parameter("person_report_result_topic").value
                ),
            )
        elif report_mode == "MOCK":
            self.report = MockReportAdapter(self.mock_results)
        else:
            raise ValueError(
                "report_mode은 MOCK 또는 TOPIC_BRIDGE여야 합니다."
            )

        dispatcher = ActionDispatcher(
            self.navigation,
            self.spray,
            self.report,
            MockWaitAdapter(self.mock_results),
        )
        self.orchestrator = VLAOrchestrator(
            self.world,
            llm,
            TargetResolver(),
            validator,
            dispatcher,
        )

        self.state_pub = self.create_publisher(
            String,
            "/vla/world_model",
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.action_pub = self.create_publisher(
            String,
            "/vla/action_validated",
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("mission_topic").value),
            self._mission_cb,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("perception_topic").value),
            self._observation_cb,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("robot_pose_topic").value),
            self._pose_cb,
            10,
            callback_group=self.pose_callback_group,
        )
        self.create_timer(
            float(self.get_parameter("decision_period_sec").value),
            self._tick,
        )
        self.get_logger().info(
            "VLA orchestrator started: "
            f"llm_backend={llm_backend}, navigation_mode={navigation_mode}, "
            f"report_mode={report_mode}, spray_mode={spray_mode}"
        )

    def _mission_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self.world.set_mission(
                str(data.get("mission_id", "mission_001")),
                str(data["text"]),
            )
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Mission parsing failed: {exc}")

    def _observation_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self.world.update_observation_batch(
                self.perception_normalizer.normalize(data)
            )
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Observation parsing failed: {exc}")

    def _pose_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            pose = data["pose"]
            timestamp = data.get("timestamp")
            updated_at = (
                _unix_seconds_to_iso(float(timestamp))
                if timestamp is not None
                else None
            )
            self.world.update_robot_pose(
                Pose2D(
                    float(pose["x"]),
                    float(pose["y"]),
                    float(pose.get("yaw", 0.0)),
                ),
                updated_at=updated_at,
            )
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Robot pose parsing failed: {exc}")

    def _tick(self) -> None:
        try:
            self.orchestrator.process_results(self.mock_results)
            if isinstance(self.navigation, TopicBridgeNavigationAdapter):
                self.orchestrator.process_results(self.navigation)
            if isinstance(self.report, TopicBridgePersonReportAdapter):
                self.orchestrator.process_results(self.report)
            if isinstance(self.spray, TopicBridgeSprayAdapter):
                self.orchestrator.process_results(self.spray)

            cycle = self.orchestrator.decide_once()
            self.status_tracker.update(cycle)
            if cycle.validation and cycle.validation.action:
                msg = String()
                msg.data = json.dumps(
                    {
                        "approved": cycle.validation.approved,
                        "reason": cycle.validation.reason,
                        "action": cycle.validation.action.to_dict(),
                        "submission": (
                            cycle.submission.status.value
                            if cycle.submission
                            else None
                        ),
                        "blocked_reason": cycle.blocked_reason,
                    },
                    ensure_ascii=False,
                )
                self.action_pub.publish(msg)
            self._publish_state()
        except (ValueError, RuntimeError) as exc:
            self.get_logger().error(f"VLA orchestration failed: {exc}")
        except Exception:
            self.get_logger().exception("Unexpected VLA orchestration failure")

    def _publish_state(self) -> None:
        snapshot = self.world.create_snapshot()
        msg = String()
        msg.data = json.dumps(snapshot, ensure_ascii=False)
        self.state_pub.publish(msg)

        status_msg = String()
        status_msg.data = json.dumps(
            self.status_tracker.create_payload(snapshot),
            ensure_ascii=False,
        )
        self.status_pub.publish(status_msg)


def main(args=None) -> None:
    if rclpy is None:
        raise RuntimeError("ROS2 환경에서 실행해야 합니다.")
    rclpy.init(args=args)
    node = VLAOrchestratorNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
