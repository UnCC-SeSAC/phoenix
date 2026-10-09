import inspect
from pathlib import Path
from types import SimpleNamespace

import pytest

from fire_vla_core.llm import (
    MockVLABrain,
    OllamaLLMClient,
    TransformersQwenAdapter,
)
from fire_vla_core import qwen_inference_server
from fire_vla_core.domain import ExplorationStatus
from fire_vla_core.ros import orchestrator_node
from fire_vla_core.world_model import WorldModel


def backend_kwargs():
    return {
        "ollama_model": "ollama-model",
        "ollama_base_url": "http://ollama.test",
        "transformers_model_id": "test/qwen",
        "transformers_device": "xpu:0",
        "transformers_max_new_tokens": 64,
    }


def test_transformers_adapter_default_output_budget_is_96_tokens():
    assert (
        TransformersQwenAdapter.__dataclass_fields__["max_new_tokens"].default
        == 96
    )


def test_qwen_server_default_output_budget_is_96_tokens(monkeypatch):
    captured = {}

    class FakeServer:
        def serve_forever(self):
            return None

        def server_close(self):
            return None

    def fake_build_backend(args):
        captured["max_new_tokens"] = args.max_new_tokens
        return object()

    monkeypatch.setattr(qwen_inference_server, "build_backend", fake_build_backend)
    monkeypatch.setattr(
        qwen_inference_server,
        "create_server",
        lambda host, port, backend: FakeServer(),
    )

    qwen_inference_server.main([])

    assert captured["max_new_tokens"] == 96


def test_mock_backend_is_lazy(monkeypatch):
    monkeypatch.setattr(
        "fire_vla_core.llm._load_transformers_runtime",
        lambda: (_ for _ in ()).throw(
            AssertionError("Transformers runtime must stay lazy")
        ),
    )

    llm = orchestrator_node.create_llm_backend(
        "mock",
        **backend_kwargs(),
    )

    assert isinstance(llm, MockVLABrain)


def test_ollama_backend_preserves_existing_configuration():
    llm = orchestrator_node.create_llm_backend(
        "ollama",
        **backend_kwargs(),
    )

    assert isinstance(llm, OllamaLLMClient)
    assert llm.model == "ollama-model"
    assert llm.base_url == "http://ollama.test"


def test_transformers_backend_uses_configured_adapter(monkeypatch):
    captured = {}

    class FakeTransformersQwenAdapter:
        def __init__(self, **kwargs):
            captured.update(kwargs)

    monkeypatch.setattr(
        orchestrator_node,
        "TransformersQwenAdapter",
        FakeTransformersQwenAdapter,
    )

    llm = orchestrator_node.create_llm_backend(
        "transformers",
        **backend_kwargs(),
    )

    assert isinstance(llm, FakeTransformersQwenAdapter)
    assert captured == {
        "model_id": "test/qwen",
        "device": "xpu:0",
        "max_new_tokens": 64,
    }


def test_invalid_backend_fails_explicitly():
    with pytest.raises(ValueError, match="llm_backend"):
        orchestrator_node.create_llm_backend(
            "typo",
            **backend_kwargs(),
        )


def test_invalid_transformers_token_limit_fails_explicitly(monkeypatch):
    monkeypatch.setattr(
        orchestrator_node,
        "TransformersQwenAdapter",
        lambda **kwargs: pytest.fail("adapter must not be constructed"),
    )
    kwargs = backend_kwargs()
    kwargs["transformers_max_new_tokens"] = 0

    with pytest.raises(ValueError, match="양수"):
        orchestrator_node.create_llm_backend(
            "transformers",
            **kwargs,
        )


def test_pose_callback_can_run_while_remote_inference_blocks_timer():
    init_source = inspect.getsource(
        orchestrator_node.VLAOrchestratorNode.__init__
    )
    main_source = inspect.getsource(orchestrator_node.main)

    assert (
        "self.pose_callback_group = MutuallyExclusiveCallbackGroup()"
        in init_source
    )
    assert "callback_group=self.pose_callback_group" in init_source
    assert "MultiThreadedExecutor(num_threads=2)" in main_source
    assert "executor.spin()" in main_source


def test_orchestrator_applies_fire_and_person_confidence_defaults():
    init_source = inspect.getsource(
        orchestrator_node.VLAOrchestratorNode.__init__
    )
    assert 'self.declare_parameter("fire_confidence_threshold", 0.25)' in init_source
    assert 'self.declare_parameter("person_confidence_threshold", 0.50)' in init_source
    assert 'fire_confidence_threshold=float(' in init_source
    assert 'self.get_parameter("fire_confidence_threshold").value' in init_source
    assert 'person_confidence_threshold=float(' in init_source
    assert 'self.get_parameter("person_confidence_threshold").value' in init_source


def test_orchestrator_applies_person_fire_risk_distance_default():
    init_source = inspect.getsource(
        orchestrator_node.VLAOrchestratorNode.__init__
    )
    assert 'self.declare_parameter("person_fire_risk_distance_m", 0.20)' in init_source
    assert 'self.get_parameter("person_fire_risk_distance_m").value' in init_source


def test_orchestrator_applies_entity_merge_distance_default():
    init_source = inspect.getsource(
        orchestrator_node.VLAOrchestratorNode.__init__
    )
    assert 'self.declare_parameter("entity_merge_distance_m", 0.15)' in init_source
    assert 'self.get_parameter("entity_merge_distance_m").value' in init_source


def test_frontier_candidate_wiring_keeps_vla_as_navigation_owner():
    repository = Path(__file__).resolve().parents[3]
    wrapper = (repository / "scripts/vla_hardware_e2e.sh").read_text(
        encoding="utf-8"
    )
    launch = (
        repository / "src/uncc_example/launch/uncc_frontier.launch.py"
    ).read_text(encoding="utf-8")
    node_source = inspect.getsource(orchestrator_node.VLAOrchestratorNode)

    assert "start_frontier:=true frontier_candidate_only:=true" in wrapper
    assert "frontier_candidate_only = LaunchConfiguration" in launch
    assert "'candidate_only': ParameterValue(" in launch
    assert '"/explore/selected_frontier"' in node_source
    assert '"/exploration_complete"' in node_source
    assert 'f"frontier_{round(x / 0.1)}_{round(y / 0.1)}"' in node_source


def test_frontier_candidate_and_completion_update_world_without_dispatch():
    node = object.__new__(orchestrator_node.VLAOrchestratorNode)
    node.world = WorldModel()
    node._frontier_complete = False
    candidate = SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=2.0, y=-1.0),
        orientation=SimpleNamespace(z=0.0, w=1.0),
    ))

    node._frontier_candidate_cb(candidate)

    assert node.world.exploration_status == ExplorationStatus.RUNNING
    assert node.world.unexplored_zones == [{
        "id": "frontier_20_-10",
        "pose": {"x": 2.0, "y": -1.0, "yaw": 0.0},
    }]

    node._exploration_complete_cb(object())

    assert node._frontier_complete is True
    assert node.world.exploration_status == ExplorationStatus.COMPLETED
    assert node.world.unexplored_zones == []


def test_topic_bridge_and_vla_config_match_confidence_contract():
    repository = Path(__file__).resolve().parents[3]
    launch_source = (
        repository
        / "src/fire_vla_bringup/launch/topic_bridge_vla.launch.py"
    ).read_text(encoding="utf-8")
    config_source = (
        repository / "src/fire_vla_bringup/config/vla.yaml"
    ).read_text(encoding="utf-8")

    assert 'LaunchConfiguration("fire_confidence_threshold")' in launch_source
    assert 'default_value="0.25"' in launch_source
    assert 'LaunchConfiguration("person_confidence_threshold")' in launch_source
    assert 'default_value="0.50"' in launch_source
    assert 'LaunchConfiguration("entity_merge_distance_m")' in launch_source
    assert 'default_value="0.15"' in launch_source
    assert 'LaunchConfiguration("person_fire_risk_distance_m")' in launch_source
    assert 'default_value="0.20"' in launch_source
    assert "fire_confidence_threshold: 0.25" in config_source
    assert "person_confidence_threshold: 0.5" in config_source
    assert "entity_merge_distance_m: 0.15" in config_source
    assert "person_fire_risk_distance_m: 0.20" in config_source
