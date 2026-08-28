from __future__ import annotations

import json
import logging
import time
import urllib.error
import urllib.request
from copy import deepcopy
from dataclasses import dataclass
from math import hypot
from typing import Any

from .domain import ActionDecision, ActionType
from .ports import LLMPort


ALLOWED_ACTIONS = [
    action.value for action in ActionType
    if action != ActionType.REPORT_PERSON
]
LOGGER = logging.getLogger(__name__)


class LLMError(RuntimeError):
    pass


class LLMInferenceError(LLMError):
    pass


class LLMOutputError(LLMError):
    pass


def parse_action_decision(content: str) -> ActionDecision:
    """Parse the complete model response without repair or extraction."""
    try:
        data = json.loads(content)
    except (json.JSONDecodeError, TypeError) as exc:
        raise LLMOutputError(f"LLM 출력이 단일 JSON 객체가 아닙니다: {exc}") from exc
    if not isinstance(data, dict):
        raise LLMOutputError("LLM 출력은 JSON 객체여야 합니다.")
    if set(data) != {"action", "target", "reason"}:
        raise LLMOutputError("action, target, reason 세 필드만 허용됩니다.")
    try:
        action = ActionType(data["action"])
    except (TypeError, ValueError) as exc:
        raise LLMOutputError("action이 허용된 ActionType이 아닙니다.") from exc
    reason = data["reason"]
    if not isinstance(reason, str) or not reason.strip():
        raise LLMOutputError("reason은 비어 있지 않은 문자열이어야 합니다.")
    target = data["target"]
    if target == "null":
        raise LLMOutputError('문자열 "null"은 target으로 허용되지 않습니다.')
    if target is not None and not isinstance(target, str):
        raise LLMOutputError("target은 문자열 또는 JSON null이어야 합니다.")
    target_required = {
        ActionType.NAVIGATE_TO,
        ActionType.REPORT_PERSON,
        ActionType.EXTINGUISH,
        ActionType.SEARCH,
    }
    if action in target_required and not target:
        raise LLMOutputError(f"{action.value}에는 문자열 target이 필요합니다.")
    if action in {ActionType.WAIT, ActionType.RETURN_HOME} and target is not None:
        raise LLMOutputError(f"{action.value}의 target은 JSON null이어야 합니다.")
    return ActionDecision(action=action, target=target, reason=reason.strip())


def extract_valid_targets(world_model: dict[str, Any]) -> list[str]:
    targets: list[str] = []
    seen: set[str] = set()
    for collection in ("people", "fires", "unexplored_zones"):
        for entity in world_model.get(collection) or []:
            if (collection == "people" and isinstance(entity, dict)
                    and entity.get("reported") is True):
                continue
            entity_id = entity.get("id") if isinstance(entity, dict) else None
            if isinstance(entity_id, str) and entity_id and entity_id not in seen:
                seen.add(entity_id)
                targets.append(entity_id)
    return targets


def build_compact_world_model(world_model: dict[str, Any]) -> dict[str, Any]:
    """Keep remote inference limited to semantic decision state."""
    compact = {
        "exploration_status": world_model.get("exploration_status"),
        "perception_ready": world_model.get("perception_ready"),
        "robot": _select_fields(
            world_model.get("robot"),
            ("pose", "navigation_status", "home_pose"),
        ),
        "people": [
            _select_fields(item, ("id", "position", "state", "reported"))
            for item in world_model.get("people") or []
            if isinstance(item, dict) and item.get("reported") is not True
        ],
        "fires": [
            _select_fields(
                item,
                (
                    "id", "position", "size", "state",
                    "robot_within_spray_range", "spray_count",
                    "threatens_person", "threatened_person_id",
                ),
            )
            for item in world_model.get("fires") or []
            if isinstance(item, dict)
        ],
        "unexplored_zones": [
            _select_fields(item, ("id", "pose"))
            for item in world_model.get("unexplored_zones") or []
            if isinstance(item, dict)
        ],
        "current_action": _select_fields(
            world_model.get("current_action"),
            ("action", "target", "status"),
        ),
    }
    source_fires = [
        item for item in world_model.get("fires") or []
        if isinstance(item, dict)
    ]
    for entity, source in zip(compact["fires"], source_fires):
        entity["blocks_person_route"] = bool(
            source.get(
                "blocks_person_route",
                source.get("blocks_route_to"),
            )
        )
    robot_pose = (compact.get("robot") or {}).get("pose")
    if not isinstance(robot_pose, dict):
        return compact
    try:
        robot_x = float(robot_pose["x"])
        robot_y = float(robot_pose["y"])
    except (KeyError, TypeError, ValueError):
        return compact
    for collection in ("people", "fires"):
        for entity in compact.get(collection) or []:
            position = entity.get("position") if isinstance(entity, dict) else None
            if not isinstance(position, dict):
                continue
            try:
                distance = round(hypot(
                    float(position["x"]) - robot_x,
                    float(position["y"]) - robot_y,
                ), 3)
                entity["distance_from_robot_m"] = distance
                if collection == "people":
                    entity["within_report_range"] = distance <= 0.8
            except (KeyError, TypeError, ValueError):
                continue
    return compact


def _select_fields(
    value: Any,
    fields: tuple[str, ...],
) -> dict[str, Any] | None:
    if not isinstance(value, dict):
        return None
    return deepcopy({field: value[field] for field in fields if field in value})


@dataclass(slots=True)
class RemoteQwenBackend(LLMPort):
    endpoint: str
    timeout_sec: float = 3.0

    def __post_init__(self) -> None:
        if not self.endpoint.startswith(("http://", "https://")):
            raise ValueError("remote Qwen endpoint는 HTTP(S) URL이어야 합니다.")
        if self.timeout_sec <= 0:
            raise ValueError("remote Qwen timeout은 양수여야 합니다.")

    def decide(self, mission: str, world_model: dict[str, Any]) -> ActionDecision:
        payload = {
            "mission": mission,
            "world_model": build_compact_world_model(world_model),
            "allowed_actions": ALLOWED_ACTIONS,
        }
        request = urllib.request.Request(
            self.endpoint,
            data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        started_at = time.monotonic()
        LOGGER.info(
            "qwen_client event=request_start bytes=%d",
            len(request.data or b""),
        )
        try:
            with urllib.request.urlopen(
                request, timeout=self.timeout_sec
            ) as response:
                content = response.read().decode("utf-8")
        except urllib.error.HTTPError as exc:
            error_body = exc.read().decode("utf-8", errors="replace")
            LOGGER.warning(
                "qwen_client event=http_error elapsed_sec=%.6f status=%d body=%s",
                time.monotonic() - started_at,
                exc.code,
                error_body,
            )
            raise LLMInferenceError(
                "Remote Qwen 호출에 실패했습니다: "
                f"HTTP {exc.code}: {error_body}"
            ) from exc
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            LOGGER.warning(
                "qwen_client event=request_failed elapsed_sec=%.6f error=%s",
                time.monotonic() - started_at,
                exc,
            )
            raise LLMInferenceError(
                f"Remote Qwen 호출에 실패했습니다: {exc}"
            ) from exc
        LOGGER.info(
            "qwen_client event=response_received elapsed_sec=%.6f",
            time.monotonic() - started_at,
        )
        return parse_action_decision(content)


def build_qwen_system_prompt() -> str:
    return """You are a fire-response robot decision engine.
Prioritize human safety. Apply the FIRST matching policy, then reassess after
every terminal action result.
1. current_action is not null: WAIT with target null. Do not overlap actions.
2. An ACTIVE fire has blocks_person_route=true:
   choose that fire. EXTINGUISH only when robot_within_spray_range is true;
   otherwise NAVIGATE_TO that fire.
3. An ACTIVE fire remains: EXTINGUISH only when robot_within_spray_range is true;
   otherwise NAVIGATE_TO that fire.
4. unexplored_zones is non-empty: SEARCH an existing zone id.
5. Otherwise, including empty people/fires/zones: RETURN_HOME with target null.
People are reported automatically outside Qwen. When several valid fires
remain at the same priority, reason from explicit human-risk relations, action
feasibility, and current robot state; do not invent risk.

Output exactly one compact single-line JSON object with action, target, reason.
Keep reason at 12 words or fewer.
action must be in allowed_actions.
NAVIGATE_TO targets an existing fires id.
EXTINGUISH targets an existing fires id.
SEARCH targets an existing unexplored_zones id.
WAIT and RETURN_HOME use JSON null.
Every non-null target must exactly match one of valid_targets. If valid_targets
is empty, choose RETURN_HOME with target null.
Never invent an id, coordinate, class name, or the string "null".
Do not claim a missing or invalid map position. Never propose a stale or invalid
physical action; deterministic validation remains authoritative.
Do not output Markdown or any text outside the JSON object."""


def _load_transformers_runtime():
    import torch
    from transformers import AutoModelForCausalLM, AutoTokenizer
    return torch, AutoTokenizer, AutoModelForCausalLM


@dataclass(slots=True)
class TransformersQwenAdapter(LLMPort):
    model_id: str = "Qwen/Qwen3-1.7B"
    device: str = "xpu:0"
    dtype: str = "float32"
    max_new_tokens: int = 64
    _torch: Any = None
    _tokenizer: Any = None
    _model: Any = None

    def __post_init__(self) -> None:
        try:
            torch, tokenizer_class, model_class = _load_transformers_runtime()
            if not self.device.startswith("xpu:"):
                raise LLMInferenceError("명시적인 XPU device가 필요합니다.")
            if not torch.xpu.is_available():
                raise LLMInferenceError("PyTorch XPU를 사용할 수 없습니다.")
            try:
                device_index = int(self.device.split(":", 1)[1])
            except (IndexError, ValueError) as exc:
                raise LLMInferenceError(
                    f"유효하지 않은 XPU device입니다: {self.device}"
                ) from exc
            if device_index < 0 or device_index >= torch.xpu.device_count():
                raise LLMInferenceError(
                    f"존재하지 않는 XPU device입니다: {self.device}"
                )
            torch_dtype = getattr(torch, self.dtype, None)
            if torch_dtype is None:
                raise LLMInferenceError(f"지원하지 않는 dtype입니다: {self.dtype}")
            self._torch = torch
            self._tokenizer = tokenizer_class.from_pretrained(self.model_id)
            self._model = model_class.from_pretrained(
                self.model_id, torch_dtype=torch_dtype
            )
            self._model = self._model.to(self.device)
            self._model.eval()
        except LLMError:
            raise
        except Exception as exc:
            raise LLMInferenceError(
                f"Qwen runtime 초기화에 실패했습니다: {exc}"
            ) from exc

    def decide(self, mission: str, world_model: dict[str, Any]) -> ActionDecision:
        inference_world = build_compact_world_model(world_model)
        user_input = {
            "mission": mission,
            "world_model": inference_world,
            "allowed_actions": ALLOWED_ACTIONS,
            "valid_targets": extract_valid_targets(inference_world),
        }
        messages = [
            {"role": "system", "content": build_qwen_system_prompt()},
            {"role": "user", "content": json.dumps(user_input, ensure_ascii=False)},
        ]
        try:
            rendered = self._tokenizer.apply_chat_template(
                messages, tokenize=False, add_generation_prompt=True,
                enable_thinking=False,
            )
            inputs = self._tokenizer(rendered, return_tensors="pt").to(self.device)
            input_length = inputs["input_ids"].shape[-1]
            with self._torch.inference_mode():
                outputs = self._model.generate(
                    **inputs,
                    do_sample=False,
                    max_new_tokens=self.max_new_tokens,
                    pad_token_id=self._tokenizer.eos_token_id,
                )
            generated = outputs[0][input_length:]
            content = self._tokenizer.decode(
                generated, skip_special_tokens=True
            ).strip()
        except Exception as exc:
            raise LLMInferenceError(f"Qwen 추론에 실패했습니다: {exc}") from exc
        return parse_action_decision(content)


@dataclass(slots=True)
class OllamaLLMClient(LLMPort):
    model: str = "Qwen2-1.5B-Instruct-Function-Calling-v1"
    base_url: str = "http://127.0.0.1:11434"
    timeout_sec: float = 10.0
    temperature: float = 0.1
    max_retries: int = 2
    retry_backoff_sec: float = 0.5

    def decide(self, mission: str, world_model: dict[str, Any]) -> ActionDecision:
        payload = {
            "model": self.model,
            "stream": False,
            "format": "json",
            "options": {"temperature": self.temperature, "num_ctx": 2048},
            "messages": [
                {"role": "system", "content": build_system_prompt()},
                {"role": "user", "content": json.dumps({
                    "mission": mission,
                    "world_model": world_model,
                    "allowed_actions": ALLOWED_ACTIONS,
                }, ensure_ascii=False)},
            ],
        }
        last_error: Exception | None = None
        for attempt in range(self.max_retries + 1):
            try:
                request = urllib.request.Request(
                    self.base_url.rstrip("/") + "/api/chat",
                    data=json.dumps(payload).encode("utf-8"),
                    headers={"Content-Type": "application/json"},
                    method="POST",
                )
                with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                    body = json.loads(response.read().decode("utf-8"))
                content = body.get("message", {}).get("content")
                if not content:
                    raise LLMError("LLM 응답에 message.content가 없습니다.")
                return ActionDecision.from_dict(json.loads(content))
            except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, ValueError, LLMError) as exc:
                last_error = exc
                if attempt < self.max_retries:
                    time.sleep(self.retry_backoff_sec * (attempt + 1))
        raise LLMError(f"LLM 호출에 실패했습니다: {last_error}")


def build_system_prompt() -> str:
    return """당신은 화재 대응 로봇의 VLA Brain이다.
자연어 Mission과 검증된 WorldModel 사실만 사용해 다음 행동 하나를 제안하라.
오직 JSON 객체 하나만 출력하라. Markdown과 추가 설명은 금지한다.
허용 Action: NAVIGATE_TO, EXTINGUISH, SEARCH, WAIT, RETURN_HOME.
WorldModel에 없는 entity를 만들지 마라.
좌표와 action_id를 생성하지 마라. 좌표는 Application이 WorldModel에서 해결한다.
EXTINGUISH는 대상 화점의 robot_within_spray_range가 true이고 state가 ACTIVE일 때만 선택한다.
current_action이 존재하면 WAIT를 선택한다.
필수 키: action, target, reason.
target이 필요 없는 WAIT와 RETURN_HOME은 target을 null로 출력할 수 있다.
reason은 현재 상태에 근거한 한국어 한 문장으로 작성한다."""


class MockVLABrain(LLMPort):
    """Deterministic wiring mock, not a natural-language policy engine."""

    def decide(self, mission: str, world_model: dict[str, Any]) -> ActionDecision:
        if world_model.get("current_action") is not None:
            return ActionDecision(ActionType.WAIT, "현재 물리 행동의 완료 결과를 기다린다")

        people = world_model.get("people") or []
        fires = world_model.get("fires") or []
        unresolved_people = [p for p in people if not p.get("reported", False)]
        active_fires = [f for f in fires if f.get("state") == "ACTIVE"]
        blocking = next((f for f in active_fires if f.get("blocks_route_to") and any(p.get("id") == f.get("blocks_route_to") for p in unresolved_people)), None)

        if blocking:
            if blocking.get("robot_within_spray_range"):
                return ActionDecision(ActionType.EXTINGUISH, "인명 접근 경로를 차단하는 화점이 분사 가능 범위에 있다", blocking["id"])
            return ActionDecision(ActionType.NAVIGATE_TO, "인명 접근 경로를 차단하는 화점에 먼저 접근한다", blocking["id"])

        if unresolved_people:
            person = unresolved_people[0]
            robot_pose = (world_model.get("robot") or {}).get("pose")
            if self._is_near(robot_pose, person.get("position"), 0.8):
                return ActionDecision(ActionType.REPORT_PERSON, "인명 위치가 확인되어 소방관에게 보고한다", person["id"])
            return ActionDecision(ActionType.NAVIGATE_TO, "미보고 인명을 우선 확인한다", person["id"])

        if active_fires:
            fire = active_fires[0]
            if fire.get("robot_within_spray_range"):
                return ActionDecision(ActionType.EXTINGUISH, "활성 화점이 분사 가능 범위에 있다", fire["id"])
            return ActionDecision(ActionType.NAVIGATE_TO, "접근 가능한 활성 화점으로 이동한다", fire["id"])

        zones = world_model.get("unexplored_zones") or []
        if zones:
            return ActionDecision(ActionType.SEARCH, "미탐색 구역을 계속 탐색한다", zones[0].get("id"))
        return ActionDecision(ActionType.RETURN_HOME, "미해결 대상이 없어 시작 위치로 복귀한다")

    @staticmethod
    def _is_near(a: dict[str, Any] | None, b: dict[str, Any] | None, threshold: float) -> bool:
        if not a or not b:
            return False
        return ((float(a["x"]) - float(b["x"])) ** 2 + (float(a["y"]) - float(b["y"])) ** 2) ** 0.5 <= threshold
