import json
import threading
import urllib.error
import urllib.request

import pytest

from fire_vla_core.adapters.mock_adapters import (
    MockNavigationAdapter,
    MockReportAdapter,
    MockResultQueue,
    MockSprayAdapter,
    MockWaitAdapter,
)
from fire_vla_core.dispatcher import ActionDispatcher
from fire_vla_core.domain import (
    ActionType,
    ObservationBatch,
    Pose2D,
    SemanticObservation,
    utc_now_iso,
)
from fire_vla_core.llm import (
    ALLOWED_ACTIONS,
    RemoteQwenBackend,
)
from fire_vla_core.orchestrator import VLAOrchestrator
from fire_vla_core.qwen_inference_server import create_server
from fire_vla_core.resolver import TargetResolver
from fire_vla_core.validator import ActionValidator
from fire_vla_core.world_model import WorldModel


class FakeResponse:
    def __init__(self, payload):
        self.payload = payload

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False

    def read(self):
        return json.dumps(self.payload, ensure_ascii=False).encode("utf-8")


def make_pipeline(backend):
    world = WorldModel()
    world.update_robot_pose(Pose2D(0.0, 0.0))
    world.set_mission("remote_test", "인명을 우선 확인해")
    now = utc_now_iso()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation(
            "person_0001", "person", 0.95, Pose2D(0.95, 0.0), now
        ),
    )))
    queue = MockResultQueue()
    navigation = MockNavigationAdapter(queue)
    dispatcher = ActionDispatcher(
        navigation,
        MockSprayAdapter(queue),
        MockReportAdapter(queue),
        MockWaitAdapter(queue),
    )
    orchestrator = VLAOrchestrator(
        world,
        backend,
        TargetResolver(),
        ActionValidator(),
        dispatcher,
    )
    return world, navigation, orchestrator


def test_remote_backend_success_uses_existing_pipeline(monkeypatch):
    captured = {}

    def fake_urlopen(request, timeout):
        captured["timeout"] = timeout
        captured["payload"] = json.loads(request.data.decode("utf-8"))
        return FakeResponse({
            "action": "NAVIGATE_TO",
            "target": "person_0001",
            "reason": "미보고 인명을 우선 확인한다",
        })

    monkeypatch.setattr("urllib.request.urlopen", fake_urlopen)
    backend = RemoteQwenBackend("http://pc.test:8088/infer", timeout_sec=3.0)
    world, navigation, orchestrator = make_pipeline(backend)
    snapshot = world.create_snapshot()
    snapshot["raw_sensor"] = {"scan": [1, 2, 3]}

    decision = backend.decide("인명을 우선 확인해", snapshot)
    assert decision.action == ActionType.NAVIGATE_TO
    assert captured["timeout"] == 3.0
    assert "raw_sensor" not in captured["payload"]["world_model"]

    cycle = orchestrator.decide_once()
    assert cycle.validation.approved
    assert cycle.submission is not None
    assert len(navigation.calls) == 1


def test_remote_backend_timeout_is_safe_non_dispatch(monkeypatch):
    monkeypatch.setattr(
        "urllib.request.urlopen",
        lambda *args, **kwargs: (_ for _ in ()).throw(
            urllib.error.URLError("unavailable")
        ),
    )
    _, navigation, orchestrator = make_pipeline(
        RemoteQwenBackend("http://pc.test:8088/infer")
    )

    cycle = orchestrator.decide_once()

    assert cycle.decision is None
    assert cycle.submission is None
    assert cycle.blocked_reason.startswith("LLM_INFERENCE_FAILED:")
    assert navigation.calls == []


def test_remote_backend_malformed_response_is_safe_non_dispatch(monkeypatch):
    monkeypatch.setattr(
        "urllib.request.urlopen",
        lambda *args, **kwargs: FakeResponse({
            "action": "NAVIGATE_TO",
            "target": "person_0001",
        }),
    )
    _, navigation, orchestrator = make_pipeline(
        RemoteQwenBackend("http://pc.test:8088/infer")
    )

    cycle = orchestrator.decide_once()

    assert cycle.decision is None
    assert cycle.submission is None
    assert cycle.blocked_reason.startswith("LLM_OUTPUT_INVALID:")
    assert navigation.calls == []


def test_remote_backend_rejects_invalid_configuration():
    with pytest.raises(ValueError, match="HTTP"):
        RemoteQwenBackend("pc.test:8088/infer")
    with pytest.raises(ValueError, match="양수"):
        RemoteQwenBackend("http://pc.test:8088/infer", timeout_sec=0)


def test_inference_server_health_and_synthetic_decision():
    class FixedBackend:
        def decide(self, mission, world_model):
            from fire_vla_core.domain import ActionDecision
            return ActionDecision(
                ActionType.NAVIGATE_TO,
                "미보고 인명을 우선 확인한다",
                "person_0001",
            )

    server = create_server("127.0.0.1", 0, FixedBackend())
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_port}"
    try:
        with urllib.request.urlopen(base_url + "/health", timeout=2) as response:
            assert json.loads(response.read()) == {"status": "ok"}
        payload = {
            "mission": "인명을 우선 확인해",
            "world_model": {"people": [{"id": "person_0001"}]},
            "allowed_actions": ALLOWED_ACTIONS,
        }
        request = urllib.request.Request(
            base_url + "/infer",
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=2) as response:
            result = json.loads(response.read())
        assert result == {
            "action": "NAVIGATE_TO",
            "target": "person_0001",
            "reason": "미보고 인명을 우선 확인한다",
        }
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
