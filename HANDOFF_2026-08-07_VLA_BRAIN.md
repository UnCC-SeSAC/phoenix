# VLA Brain 개발 세션 Handoff

- 문서 기준일: 2026-08-10 (Asia/Seoul)
- Repository: <https://github.com/UnCC-SeSAC/phoenix>
- Branch: `feature/vla-brain`
- 문서 최신화 전 기준 commit: `dc56f6e627942326eab4aa0a363ef34f46d7fe07`

이 문서는 현재 코드와 테스트를 source of truth로 작성한다. 구현 상태, 현재 checkout
재검증 결과, 과거 장비 integration 이력을 구분한다.

## 1. Architecture와 책임 경계

```text
Mission → Semantic WorldModel → LLMPort → ActionDecision
→ TargetResolver → ActionValidator → ActionDispatcher
→ Navigation / Spray / Report / Wait Port
```

지원 Action은 `NAVIGATE_TO`, `REPORT_PERSON`, `EXTINGUISH`, `SEARCH`, `WAIT`,
`RETURN_HOME`이다. 모든 실행 Port는 `submit(Action) → ActionSubmission` 계약을
사용하며 제출 상태와 최종 실행 결과를 분리한다.

LLM은 좌표, `action_id`, Nav2 goal 또는 Pump 명령을 직접 생성하지 않는다. target
pose는 Resolver가 WorldModel의 신뢰된 좌표로 구성한다. `reason`은 untrusted
diagnostic text이며 Validator의 물리 안전 판단 근거가 아니다.

팀의 YOLO/Depth/TF, SLAM, Frontier, Nav2/local avoidance, Pump driver 및 상태 관리
내부 구현은 VLA 팀 책임 밖이다. VLA 책임은 팀 출력을 `SemanticObservation`으로
수용하고 검증된 Action을 각 Port Adapter로 전달하는 Application/Integration 계층이다.

## 2. 현재 구현 완료 상태

- Mission, Robot, Person, Fire, Exploration 및 Action lifecycle WorldModel
- Action 제출 상태 `ACCEPTED/REJECTED/DUPLICATE`
- 최종 결과 `SUCCEEDED/FAILED/ABORTED/CANCELED/TIMED_OUT`
- `LLM → Resolver → Validator → Dispatcher` safety pipeline
- `MockVLABrain`, `OllamaLLMClient`, `TransformersQwenAdapter`
- strict Qwen JSON parser와 `valid_targets` grounding
- failure별 non-dispatch semantics
- decision-input signature deduplication
- ROS2 backend 및 XPU interpreter launch wiring
- `TopicBridgeNavigationAdapter` 코드와 launch wiring
- `fire_vla_core`, `fire_vla_bringup` packaging

Topic Bridge의 Qwen 기반 navigation goal 발행 검증은 아직 완료 상태가 아니며
VLA-01 범위다.

## 3. Qwen2.5와 XPU Runtime

```text
model: Qwen/Qwen2.5-1.5B-Instruct
device: xpu:0
dtype: float32
max_new_tokens: 128
validated hardware history: Intel Arc B580
```

`TransformersQwenAdapter`는 tokenizer/model을 생성 시 한 번만 로드하고 동일 객체를
재사용한다. 명시적인 `xpu:*` device만 허용하며 XPU unavailable/device 오류 시
`LLMInferenceError`를 발생시킨다. retry, JSON repair, target 보정 및 CPU fallback은
하지 않는다.

strict parser는 정확히 `action`, `target`, `reason` 세 필드만 허용한다.
`NAVIGATE_TO/REPORT_PERSON/EXTINGUISH/SEARCH`는 문자열 target이 필요하고
`WAIT/RETURN_HOME`은 JSON `null`만 허용한다. 문자열 `"null"`, Markdown fence,
reasoning prefix 및 extra field는 거부한다.

## 4. ROS2 backend와 XPU interpreter wiring

`VLAOrchestratorNode`의 `llm_backend` 허용값은 `mock`, `ollama`, `transformers`이고
기본값은 `mock`이다. Transformers 기본 parameter는 다음과 같다.

```text
transformers_model_id=Qwen/Qwen2.5-1.5B-Instruct
transformers_device=xpu:0
transformers_max_new_tokens=128
```

ROS2 Jazzy system Python과 XPU venv를 분리한다. `vla_python_executable`을 지정하면
launch prefix를 통해 VLAOrchestrator Node만 XPU Python으로 실행한다.

```bash
ros2 launch fire_vla_bringup topic_bridge_vla.launch.py \
  llm_backend:=transformers \
  transformers_device:=xpu:0 \
  vla_python_executable:=<workspace>/.venv-xpu/bin/python
```

## 5. Safety와 fallback semantics

```text
LLM inference failure → LLM_INFERENCE_FAILED → submission=None
LLM output/schema failure → LLM_OUTPUT_INVALID → submission=None
Target resolution failure → TARGET_RESOLUTION_FAILED → submission=None
Validation rejection → ACTION_VALIDATION_REJECTED → submission=None
```

Inference failure는 signature를 소비하지 않아 다음 timer에서 재시도할 수 있다. 실패
fallback으로 가짜 WAIT를 Dispatcher에 제출하지 않는다. 정상적인 LLM WAIT만 Resolver
→ Validator → Dispatcher → WaitPort 경로를 사용한다. Validator는 이동 pose freshness와
map bounds, target 존재 여부, 화점 ACTIVE 상태, spray range 및 시도 횟수를 검사한다.

## 6. Decision input deduplication

동일한 Mission/WorldModel 의미 입력에서는 LLM을 다시 호출하지 않는다.

- pose resolution: position `0.1 m`, yaw `0.1 rad`
- Mission, semantic entity, 의미 있는 pose, spray range 변화 시 재판단
- 물리 ActionResult 처리 후 재판단
- timestamp, event log, WAIT lifecycle 변화는 무시
- inference failure는 signature를 소비하지 않음
- output/resolution/validation blocked cycle은 동일 입력에서 반복 호출하지 않음

## 7. 검증 상태

현재 checkout에서 2026-08-10에 재검증한 결과:

```text
python3 -m pytest -q: 58 passed
colcon build --packages-select fire_vla_core fire_vla_bringup: PASS
코드 및 launch 구조 확인
```

현재 세션에서 실제 Qwen XPU inference, ROS2 + Qwen runtime, Nav2/robot 및 physical
hardware는 재실행하지 않았다.

이전 integration verification 이력에서는 Intel Arc B580 XPU smoke와 다음 경로가
PASS했다.

```text
Mission "대기해."
→ VLAOrchestratorNode
→ TransformersQwenAdapter / Qwen2.5 / xpu:0
→ ActionDecision(WAIT, target=null)
→ strict parser → TargetResolver → ActionValidator
→ ActionDispatcher → MockWaitAdapter ACCEPTED
```

이 이력은 현재 checkout에서 재실행한 결과와 구분한다.

## 8. 남은 Integration 작업

- VLA-01: Qwen Action → Topic Bridge `/vla/navigation_goal` 발행 검증
- VLA-02: Nav2 Navigation Result → VLA WorldModel E2E 연결
- VLA-03: Perception 출력 → SemanticObservation → WorldModel 연결
- VLA-04: Person Report Adapter 구현
- VLA-05: Pump/Spray Port 실제 interface 연결
- VLA-06: 소방관 Mission UI 및 Semantic Status MVP
- VLA-07: 실제 Robot VLA End-to-End 통합 테스트
- VLA-08: 실패·timeout·cancel 실물 안전성 검증

## 9. 다음 작업: VLA-01

```text
Qwen ActionDecision → TargetResolver → ActionValidator
→ TopicBridgeNavigationAdapter.submit() → /vla/navigation_goal JSON publish
```

검증 항목은 person/fire ID의 WorldModel pose resolution, 정상 NAVIGATE_TO payload,
invalid target 및 validation reject의 미발행, duplicate submission 방어다. 실제 Nav2와
실물 로봇 실행은 VLA-01 범위 밖이다.

현재 필수 blocker는 없다. `TopicBridgeNavigationAdapter` 직접 단위 테스트와 Qwen
backend + `TOPIC_BRIDGE` 발행 검증이 다음 구현 대상이다.
