import json
import urllib.error
from contextlib import nullcontext

import pytest

from fire_vla_core.domain import ActionType
from fire_vla_core.llm import (
    LLMError,
    LLMInferenceError,
    LLMOutputError,
    OllamaLLMClient,
    TransformersQwenAdapter,
    extract_valid_targets,
    parse_action_decision,
)


class FakeResponse:
    def __enter__(self): return self
    def __exit__(self, *args): return False
    def read(self): return json.dumps({"message": {"content": json.dumps({"action": "WAIT", "target": None, "reason": "대기"})}}).encode()


def test_ollama_client_parses_decision(monkeypatch):
    monkeypatch.setattr("urllib.request.urlopen", lambda *args, **kwargs: FakeResponse())
    decision = OllamaLLMClient(max_retries=0).decide("m", {})
    assert decision.action == ActionType.WAIT


def test_ollama_client_raises_after_retry(monkeypatch):
    monkeypatch.setattr("urllib.request.urlopen", lambda *args, **kwargs: (_ for _ in ()).throw(urllib.error.URLError("down")))
    with pytest.raises(LLMError):
        OllamaLLMClient(max_retries=0).decide("m", {})


def test_strict_parser_accepts_normal_json():
    decision = parse_action_decision(
        '{"action":"SEARCH","target":"zone_01","reason":"미탐색 구역을 탐색한다."}'
    )
    assert decision.action == ActionType.SEARCH
    assert decision.target == "zone_01"


@pytest.mark.parametrize(
    "content",
    [
        '{"action":"FLY","target":"zone_01","reason":"r"}',
        '{"action":"WAIT","target":null,"reason":"r","extra":1}',
        '\x60\x60\x60json\n{"action":"WAIT","target":null,"reason":"r"}\n\x60\x60\x60',
        '판단 결과: {"action":"WAIT","target":null,"reason":"r"}',
        '{"action":"WAIT","target":null,"reason":"   "}',
        '{"action":"WAIT","target":"null","reason":"r"}',
        '{"action":"SEARCH","target":null,"reason":"r"}',
    ],
)
def test_strict_parser_rejects_invalid_output(content):
    with pytest.raises(LLMOutputError):
        parse_action_decision(content)


def test_strict_parser_accepts_wait_with_json_null():
    decision = parse_action_decision(
        '{"action":"WAIT","target":null,"reason":"현재 위치에서 대기한다."}'
    )
    assert decision.action == ActionType.WAIT
    assert decision.target is None


def test_valid_targets_are_deduplicated_in_first_seen_order():
    snapshot = {
        "people": [{"id": "person_01"}, {"id": "shared"}],
        "fires": [{"id": "fire_01"}, {"id": "shared"}],
        "unexplored_zones": [{"id": "zone_01"}, {"id": "fire_01"}],
    }
    assert extract_valid_targets(snapshot) == [
        "person_01", "shared", "fire_01", "zone_01",
    ]


class FakeXPU:
    available = True

    @classmethod
    def is_available(cls):
        return cls.available

    @staticmethod
    def device_count():
        return 1


class FakeTorch:
    xpu = FakeXPU
    float32 = object()

    @staticmethod
    def inference_mode():
        return nullcontext()


class FakeInputIds:
    shape = (1, 2)


class FakeBatch(dict):
    def __init__(self):
        super().__init__(input_ids=FakeInputIds())

    def to(self, device):
        self.device = device
        return self


class FakeTokenizer:
    load_count = 0
    eos_token_id = 99
    response = '{"action":"WAIT","target":null,"reason":"대기한다."}'
    last_messages = None

    @classmethod
    def from_pretrained(cls, model_id):
        cls.load_count += 1
        return cls()

    def apply_chat_template(self, messages, **kwargs):
        type(self).last_messages = messages
        return "rendered"

    def __call__(self, rendered, return_tensors):
        return FakeBatch()

    def decode(self, generated, skip_special_tokens):
        return self.response


class FakeModel:
    load_count = 0

    @classmethod
    def from_pretrained(cls, model_id, torch_dtype):
        cls.load_count += 1
        return cls()

    def to(self, device):
        self.device = device
        return self

    def eval(self):
        self.eval_called = True
        return self

    def generate(self, **kwargs):
        return [[1, 2, 3]]


def test_transformers_adapter_loads_once_reuses_model_and_grounds_targets(
    monkeypatch,
):
    FakeTokenizer.load_count = 0
    FakeModel.load_count = 0
    monkeypatch.setattr(
        "fire_vla_core.llm._load_transformers_runtime",
        lambda: (FakeTorch, FakeTokenizer, FakeModel),
    )
    adapter = TransformersQwenAdapter()
    snapshot = {
        "people": [{"id": "person_01"}],
        "fires": [],
        "unexplored_zones": [{"id": "zone_01"}],
    }
    assert adapter.decide("대기해.", snapshot).action == ActionType.WAIT
    assert adapter.decide("대기해.", snapshot).action == ActionType.WAIT
    assert FakeTokenizer.load_count == 1
    assert FakeModel.load_count == 1
    user_payload = json.loads(FakeTokenizer.last_messages[1]["content"])
    assert user_payload["valid_targets"] == ["person_01", "zone_01"]


def test_transformers_adapter_does_not_fall_back_to_cpu(monkeypatch):
    FakeXPU.available = False
    monkeypatch.setattr(
        "fire_vla_core.llm._load_transformers_runtime",
        lambda: (FakeTorch, FakeTokenizer, FakeModel),
    )
    try:
        with pytest.raises(LLMInferenceError):
            TransformersQwenAdapter()
    finally:
        FakeXPU.available = True
