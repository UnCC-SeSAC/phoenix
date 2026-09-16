# VLA 입문 가이드 (Phoenix 코드 기준)

이 문서는 VLA를 처음 보는 팀원이 "VLA가 뭔지 → 왜 쓰는지 → Phoenix에서
어떻게 구현했는지 → 어디를 읽으면 되는지"를 순서대로 이해하기 위한 학습용
문서다. 설계 계약을 정의하는 문서는
[CURRENT_VLA_DATA_ARCHITECTURE.md](CURRENT_VLA_DATA_ARCHITECTURE.md)이고,
이 문서는 그 위에 얹는 해설이다.

---

## 1. VLA가 뭔가

VLA = **V**ision **L**anguage **A**ction.
"보고(Vision) → 말로 이해하고(Language) → 행동한다(Action)"를 한 덩어리로
묶은 로봇 제어 방식이다.

세 가지를 한 문장으로 쓰면 이렇다.

> 카메라로 본 장면과 자연어 명령을 함께 입력받아, 로봇이 다음에 할 행동을
> 출력하는 시스템.

전통적인 로봇 제어와 비교하면 차이가 분명해진다.

| | 전통적 방식 (Rule-based / FSM) | VLA |
|---|---|---|
| 입력 | 센서 값, 미리 정해진 상태 | 센서 값 + **자연어 명령** |
| 판단 | 사람이 짠 if/else, 상태 기계 | **모델이 추론** |
| 새 상황 | 코드를 고쳐야 함 | 명령 문장만 바꾸면 됨 |
| 설명 | 로그를 읽어 유추 | 모델이 `reason`을 같이 내놓음 |

Phoenix에도 두 모드가 다 있다. Firefighter UI의 `RULE_BASED` 모드가 전통
방식(Frontier 탐색 + MissionExecutor)이고, `VLA` 모드가 이 문서가 설명하는
쪽이다. 같은 하드웨어를 두 가지 두뇌로 굴려보는 구조라고 보면 된다.

### 1.1 "진짜 VLA"와 Phoenix의 VLA는 조금 다르다 (중요)

논문에서 말하는 원조 VLA(RT-2, OpenVLA 등)는 **end-to-end**다. 카메라
이미지 픽셀과 명령 문장을 하나의 거대 모델에 넣으면, 모델이 곧바로 관절
토크나 그리퍼 좌표 같은 저수준 동작을 뱉는다.

Phoenix는 그렇게 하지 않는다. **모듈형(modular) VLA**다.

```text
[원조 VLA]     이미지 픽셀 + 문장 ──> 거대모델 ──> 관절/좌표 직접 출력

[Phoenix VLA]  이미지 ──> YOLO ──> 의미 있는 사실(WorldModel)
                                        +
               자연어 Mission ───────────┴──> Qwen ──> "행동 이름 + 대상 ID"
                                                          ↓
                                              Resolver/Validator (결정론적)
                                                          ↓
                                                   Nav2 / 펌프
```

왜 이렇게 나눴는지가 이 프로젝트 설계의 핵심이다.

1. **하드웨어 제약.** Raspberry Pi에서 수십억 파라미터 VLA 모델을 실시간
   추론할 수 없다. Phoenix는 Qwen3-1.7B를 별도 PC에 올리고 HTTP로 부른다.
2. **안전.** 화재 대응 로봇이다. 모델이 환각으로 좌표를 하나 잘못 뱉으면
   실제 모터가 돌고 펌프가 분사된다. 그래서 **모델에게 좌표를 만들 권한을
   주지 않는다.** 모델은 "이미 존재하는 대상 중에서 하나를 고르는" 일만 한다.
3. **검증 가능성.** 모델 출력이 JSON 4필드로 고정되어 있어서, 이후 단계를
   전부 결정론적 코드로 검사할 수 있다. 테스트 22개가 이 경계를 지킨다.

즉 Phoenix에서 LLM은 "운전대"가 아니라 **"다음에 뭘 할지 고르는 정책
결정자"**다. 실제 운전은 Nav2가 한다.

---

## 2. 어디에 필요한가

VLA가 유용해지는 지점은 **"상황의 조합이 너무 많아서 규칙으로 다 못 쓰는
경우"**다. Phoenix의 미션을 예로 들면:

- "불부터 꺼" / "사람 먼저 찾아" / "출구 막는 불만 꺼"
- 사람 1명 + 불 2개인데 그중 하나가 사람 근처에 있음
- 불에 접근했는데 분사 거리 안에 못 들어감
- 분사했는데 불이 안 꺼짐

Rule-based로 하면 이 조합마다 분기를 짜야 한다. VLA는 같은 코드에
**문장만 바꿔서** 다른 임무를 시킨다. Phoenix UI에서 소방관이
"인명을 우선 확인해"라고 타이핑하면 그게 그대로 Qwen 입력이 된다.

일반적으로 VLA가 쓰이는 분야는 이렇다.

- 로봇 팔 조작 ("빨간 컵을 집어서 싱크대에 놔") — 원조 VLA의 주 무대
- 모바일 로봇 탐사/수색 — Phoenix가 여기
- 가정용 서비스 로봇, 창고 물류
- 자율주행의 상위 의사결정 레이어

---

## 3. Phoenix VLA 한 사이클 전체 흐름

한 번의 결정 주기(기본 1초, `decision_period_sec`)에 일어나는 일이다.

```text
① Vision   카메라 → Hailo YOLO → bbox + score
                 → Depth + CameraInfo 역투영 → camera frame (X,Y,Z)
                 → source-time TF → map(x, y)
                 → /vla/perception_observation (JSON)

② World    perception_normalizer → entity ID 부여/연결
                 → WorldModel 갱신 (사람/화점/로봇 위치/상태)

③ Language Mission 문장 + compact WorldModel
                 → HTTP POST /infer (Pi → PC)
                 → Qwen3-1.7B
                 → {"mission_scope","action","target","reason"}

④ Action   TargetResolver   : target ID → 실제 map 좌표
           ActionValidator  : 안전 검사 (여기서 거부되면 끝)
           ActionDispatcher : 해당 포트로 전달

⑤ 실행     /vla/navigation_goal → Nav2 NavigateToPose → cmd_vel → 모터
           /vla/spray_command   → SuppressFire action → 펌프/서보
           /vla/person_report   → 소방관 보고

⑥ 결과     SUCCEEDED / FAILED / ABORTED / CANCELED / TIMED_OUT
                 → WorldModel 반영 → 다음 사이클 입력이 됨
```

`①②`는 Pi에서, `③`의 모델 추론만 PC에서, `④⑤⑥`은 다시 Pi에서 돈다.
raw 이미지·Depth·TF·Nav2·`cmd_vel`은 **절대 PC로 넘어가지 않는다.**

---

## 4. 단계별 구현 뜯어보기

### 4.1 Vision — 픽셀을 지도 좌표로

파일: `src/image_pipeline/`, `src/fire_vla_core/fire_vla_core/ros/fire_detection_adapter.py`,
`.../perception_bridge_node.py`

YOLO가 주는 것은 이미지 안의 **픽셀 박스**다. 그런데 로봇이 이동하려면
**지도 위의 좌표**가 필요하다. 그 변환이 이 단계다.

1. `image_pipeline`이 `/fire/detections`로 JSON을 낸다.
   `{class_name, score, x, y, depth, depth_status, frame_size, stamp}`
2. `fire_detection_adapter.backproject()`가 CameraInfo의 K 행렬로
   픽셀 + 깊이 → 카메라 광학 좌표계 `(X,Y,Z)`를 만든다.
3. `perception_bridge_node._transform_point()`가 **검출된 그 순간의
   타임스탬프**로 TF를 조회해 `map` 프레임으로 옮긴다.
   (지금 시각이 아니라 source time인 게 포인트다. 로봇이 움직이는 중이면
   0.2초 차이가 수십 cm 오차가 된다.)
4. 결과를 `/vla/perception_observation`으로 낸다.

**fail-closed 원칙**: 값이 의심스러우면 버린다. `depth_status="unknown"`,
잘못된 CameraInfo, 무한대/NaN 좌표, TF 조회 실패는 전부 그 검출을 **폐기**한다.
추측해서 좌표를 만들어내지 않는다. `fire_detection_adapter.py:99` 근처의
`if depth_status == "unknown": continue`가 그 자리다.

### 4.2 WorldModel — 로봇의 "기억"

파일: `src/fire_vla_core/fire_vla_core/world_model.py` (458줄, **가장 중요**)

LLM은 상태를 기억하지 못한다. 매 호출이 백지다. 그래서 "지금까지 뭘 봤고,
뭘 했고, 뭐가 남았는지"를 들고 있는 객체가 따로 필요하다. 그게 WorldModel이다.

들어 있는 것:

- `mission` : 현재 임무와 상태(READY/RUNNING/COMPLETED/…)
- `robot` : 위치, home 위치, navigation 상태
- `people` : `PersonEntity` — 위치, DETECTED/REPORTED
- `fires` : `FireEntity` — 위치, 크기, ACTIVE/PENDING_VERIFICATION/EXTINGUISHED/INACCESSIBLE
- `current_action` / `last_action` / `pending_actions`
- `event_log` : 최근 이벤트 (UI가 타임라인으로 보여줌)

핵심 동작 세 가지만 기억하면 된다.

**(a) entity ID 부여** — `ros/perception_normalizer.py`
YOLO는 매 프레임 "여기 불이 있다"고만 말한다. 이게 아까 그 불인지 새 불인지는
말해주지 않는다. Normalizer가 **같은 클래스 + 0.5 m 반경 + 2초 TTL**로
직전 관측과 짝지어 `fire_0001` 같은 ID를 유지한다.
정식 tracker가 아니므로 긴 가림이나 프로세스 재시작 뒤 영구 ID는 보장하지
않는다 — 문서에 그렇게 명시돼 있다.

**(b) 공간 관계 계산** — `_refresh_spatial_flags()`
매번 로봇-화점 거리를 재서 `robot_within_spray_range`(≤ 0.30 m)를 갱신하고,
화점-사람 거리가 `person_fire_risk_distance_m`(demo 기본 0.10 m) 이내면
`threatens_person=true`를 붙인다. **LLM이 거리를 계산하지 않는다.** 계산은
코드가 하고, LLM은 그 boolean을 읽기만 한다.

**(c) 진압 검증 lifecycle** — `_update_fire_verification()`
분사 성공(`SUCCEEDED`)은 "물을 뿌렸다"일 뿐 "껐다"가 아니다. 그래서
`PENDING_VERIFICATION`으로 보내고, **연속 3회 관측에서 불이 안 보여야**
`EXTINGUISHED`가 된다. 다시 보이면 `ACTIVE`로 복귀, 5초 넘으면 타임아웃으로
`ACTIVE` 복귀, 분사 3회 실패하면 `INACCESSIBLE`. 이 상태 기계가
WorldModel 안에 있고 LLM 밖에 있다는 게 설계의 요점이다.

### 4.3 Language — Qwen을 부르는 방법

파일: `fire_vla_core/llm.py`, `fire_vla_core/qwen_inference_server.py`

**보내는 것**(`build_compact_world_model()`): WorldModel 전체가 아니라
의사결정에 필요한 필드만 추린다. 토큰을 아끼고, 모델이 헷갈릴 여지를 줄인다.
로봇-대상 거리(`distance_from_robot_m`)도 미리 계산해서 넣어준다.

```json
{
  "mission": "인명을 우선 확인해",
  "world_model": {
    "robot": {"pose": {...}, "navigation_status": "IDLE"},
    "people": [{"id": "person_0001", "position": {...}, "within_report_range": true}],
    "fires":  [{"id": "fire_0001", "state": "ACTIVE",
                "robot_within_spray_range": false,
                "distance_from_robot_m": 1.42,
                "threatens_person": true}],
    "unexplored_zones": [],
    "current_action": null
  },
  "allowed_actions": ["NAVIGATE_TO", "EXTINGUISH", "SEARCH", "WAIT", "RETURN_HOME"]
}
```

**받는 것**: 정확히 4개 필드짜리 JSON 한 줄.

```json
{"mission_scope":"FIRE_ONLY","action":"NAVIGATE_TO","target":"fire_0001","reason":"approach active fire out of spray range"}
```

**시스템 프롬프트**(`build_qwen_system_prompt()`)는 우선순위 규칙을 번호로
못박았다.

1. `current_action`이 있으면 무조건 `WAIT` — 행동을 겹치지 않는다
2. 사람 경로를 막는 ACTIVE 화점 우선
3. 남은 ACTIVE 화점
4. 미탐색 구역이 있으면 `SEARCH`
5. 그 외 `RETURN_HOME`

그리고 이런 금지 조항들이 붙는다: "ID·좌표·클래스명을 지어내지 마라",
"문자열 `"null"`을 쓰지 마라", "Markdown 금지", "reason은 12단어 이내".

**파싱은 관대하지 않다** — `parse_action_decision()`은 고쳐 쓰지 않는다.
필드 집합이 정확히 `{mission_scope, action, target, reason}`이 아니면 예외,
`WAIT`인데 target이 있어도 예외, target이 필요한데 없어도 예외. 잘못된
응답은 그 사이클을 **blocked**로 만들고 로봇은 움직이지 않는다. 자동 fallback
모션은 없다.

참고로 `REPORT_PERSON`은 `ALLOWED_ACTIONS`에서 **빠져 있다**. 사람 보고는
모델 판단 없이 새 사람이 검출되는 즉시 자동으로 나간다
(`topic_bridge_person_report_adapter.publish_new_people()`). 인명 보고는
모델의 추론에 맡기기엔 너무 중요하다는 판단이다.

**백엔드는 4가지**로 갈아끼울 수 있다 (`create_llm_backend()`):
`mock`(결정론적 스텁, 테스트용) / `ollama` / `transformers`(같은 기기에서
직접 로드) / `remote_qwen`(HTTP로 PC 호출, 실전 구성).

### 4.4 Action — LLM과 모터 사이의 3중 안전벽

여기가 이 프로젝트에서 제일 배울 게 많은 부분이다. LLM 출력은
**절대 바로 실행되지 않는다.** 세 단계를 통과해야 한다.

**① TargetResolver** (`resolver.py`)
"`fire_0001`로 가라"를 "map 좌표 (2.31, 1.07, yaw 0.42)로 가라"로 바꾼다.
좌표의 출처는 **오직 WorldModel**이다. 덤으로 `navigation_standoff_m`(0.15 m)
만큼 떨어진 지점을 목표로 잡아서 불에 정면으로 박지 않게 한다.

**② ActionValidator** (`validator.py`)
결정론적 검사. 하나라도 걸리면 거부한다.

- 다른 물리 행동이 실행 중인가 → 거부
- 로봇 위치가 신선한가 (`robot_pose_max_age_sec = 0.5초`) → 아니면 거부
- 목표 좌표가 유한한가, 지도 범위 안인가
- `EXTINGUISH`면: 대상이 존재하나 / `ACTIVE`인가 / `robot_within_spray_range`인가 /
  분사 3회를 넘지 않았나
- `REPORT_PERSON`이면: 이미 보고한 사람인가

주석 한 줄이 철학을 요약한다 — *"LLM output은 물리 상태의 source of truth가 아니다."*

**③ ActionDispatcher** (`dispatcher.py`)
행동 종류에 맞는 포트로 보내고, 같은 `action_id`가 두 번 실행되지 않게
막는다(idempotency cache).

여기에 Orchestrator 레벨의 보호 장치가 더 붙는다
(`orchestrator.py`):

- `_correct_out_of_range_extinguish()` : 모델이 사거리 밖에서 `EXTINGUISH`를
  고르면 조용히 `NAVIGATE_TO`로 바꾼다
- `_decision_input_signature()` : WorldModel이 하나도 안 변했으면 LLM을
  **다시 부르지 않는다**. 1초마다 같은 질문을 던지는 낭비를 막는다
- `_non_retryable_semantic_keys` : 같은 미션에서 이미 성공한 행동을 반복 금지
- `_build_extinguish_continuation()` : 불로 이동이 성공하면, 조건이 그대로일
  때 LLM을 다시 부르지 않고 바로 `EXTINGUISH`로 이어간다 (반응 속도 확보)

### 4.5 실행 — ROS 2 경계

VLA Core는 **ROS 메시지도 Nav2도 모른다.** 전부 `std_msgs/String` JSON
토픽으로만 대화한다. 그래서 ROS 배포판(Jazzy/Humble)이 달라도 붙는다.

| 용도 | 토픽 |
|---|---|
| 미션 / 상태 | `/vla/mission`, `/vla/status` |
| 로봇 위치 | `/vla/robot_pose_json` |
| 이동 | `/vla/navigation_goal` / `_result` / `_cancel` |
| 인명 보고 | `/vla/person_report` / `_result` |
| 진압 | `/vla/spray_command` / `_result` / `_cancel` |

건너편에서 실제 하드웨어를 만지는 노드:

- `uncc_example/vla_navigation_bridge_node.py` → Nav2 `NavigateToPose`
- `uncc_example/vla_spray_bridge_node.py` → `SuppressFire` 액션 (펌프/서보)

두 bridge 모두 시작하자마자 `/vla/control_mode`를 확인하고, 값이 `VLA`가
아니면 `CONTROL_MODE_MISMATCH`로 즉시 거부한다. **Nav2 goal 소유자는 언제나
하나**여야 하기 때문이다. Rule-based 모드가 돌고 있는데 VLA가 goal을 던지면
로봇이 두 명령 사이에서 찢어진다. 모드 소유권은 Firefighter UI가 쥔다
(`firefighter_ui_node.py`의 `ControlModeOwner`).

### 4.6 결과가 다시 WorldModel로

`orchestrator.process_results()`가 매 사이클 각 어댑터의 결과 큐를 비운다.
`apply_action_result()`가 `action_id`로 correlation해서:

- 이동 결과 → `robot.navigation_status` 갱신
- 보고 `SUCCEEDED` → 그 사람만 `REPORTED`
- 분사 결과 → `spray_count += 1`, 성공이면 `PENDING_VERIFICATION`

중복 결과는 무시하고(`DUPLICATE_RESULT_IGNORED`), 모르는 `action_id`도
무시한다(`UNRELATED_RESULT_IGNORED`). 이렇게 갱신된 WorldModel이 다음
사이클의 LLM 입력이 된다 — 루프가 닫힌다.

---

## 5. 직접 굴려보기

가장 빠른 길은 하드웨어 없이 mock으로 한 사이클을 눈으로 보는 것이다.

```bash
# 하드웨어·ROS 없이 순수 파이썬으로 의사결정 루프만 확인
python -m fire_vla_core.mock_demo

# ROS 2 + 웹 UI까지 (127.0.0.1:8080)
colcon build --packages-select fire_vla_interfaces fire_vla_core fire_vla_bringup
source install/setup.bash
ros2 launch fire_vla_bringup firefighter_ui_mock.launch.py

# 테스트로 계약 확인 (22개 파일)
pytest src/fire_vla_core/tests
```

실제 Qwen을 붙일 때는 PC에서 추론 서버를 띄우고,

```bash
python -m fire_vla_core.qwen_inference_server --backend transformers --port 8088
```

Pi 쪽은 `llm_backend:=remote_qwen`, `remote_qwen_endpoint:=http://<PC>:8088/infer`로
`topic_bridge_vla.launch.py`를 띄운다. 전체 하드웨어 구성은
`vla_robot.launch.py`가 묶어준다.

### 코드 읽는 추천 순서

1. `fire_vla_core/domain.py` — 용어와 상태값이 전부 여기 있다 (247줄)
2. `fire_vla_core/world_model.py` — 로봇의 기억 (458줄)
3. `fire_vla_core/llm.py` — 프롬프트와 JSON 계약 (461줄)
4. `fire_vla_core/orchestrator.py` — 한 사이클 (420줄)
5. `resolver.py` → `validator.py` → `dispatcher.py` — 짧다, 합쳐서 210줄
6. `ros/orchestrator_node.py` — 위 전부를 ROS에 꽂는 배선
7. `tests/test_orchestrator.py`, `tests/test_world_model.py` — 의도가 가장
   선명하게 드러나는 곳

---

## 6. 이 구현에서 가져갈 교훈

VLA 시스템을 직접 짜게 된다면 Phoenix가 내린 판단들을 그대로 빌려도 된다.

1. **LLM에게 좌표를 만들게 하지 마라.** 존재하는 ID 중 고르게만 하고, ID→좌표
   변환은 결정론적 코드가 한다.
2. **LLM 출력을 신뢰하지 말고 검증하라.** Validator가 물리 상태의 source of
   truth다.
3. **거리·임계값 계산은 코드가 하고 모델은 boolean만 읽게 하라.** 모델에게
   산수를 시키면 틀린다.
4. **모델 응답 파싱은 엄격하게.** 고쳐 쓰거나 일부만 추출하지 마라. 이상하면
   그 사이클을 버리고 멈춘다.
5. **"성공"과 "목표 달성"을 구분하라.** 분사 성공 ≠ 진화 완료. 관측으로
   검증하는 단계를 따로 둔다.
6. **제어권 소유자는 항상 한 명.** VLA와 Rule-based가 동시에 goal을 내면 안 된다.
7. **실패 시 기본 동작은 "멈춤"이다.** 자동 fallback 모션은 없다.

---

## 7. 용어 빠른 참조

| 용어 | 뜻 |
|---|---|
| VLA | Vision-Language-Action, 시각+언어 입력으로 행동을 결정하는 방식 |
| WorldModel | 지금까지 인식한 사실과 진행 상태를 담은 로봇의 기억 |
| SemanticObservation | 정규화된 검출 1건 (ID, 클래스, 신뢰도, map 좌표, 시각) |
| ActionDecision | 모델이 낸 `{mission_scope, action, target, reason}` |
| Action | Resolver가 좌표까지 채운 실행 가능한 행동 |
| mission_scope | 임무 범위. FIRE_ONLY / PERSON_FIRE / FULL_EXPLORATION |
| Resolver | target ID → map 좌표 변환기 |
| Validator | 실행 전 결정론적 안전 검사기 |
| Dispatcher | 검증된 행동을 올바른 포트로 보내는 라우터 |
| Port / Adapter | 코어를 ROS·하드웨어와 분리하는 인터페이스 경계 |
| fail-closed | 값이 의심스러우면 추측하지 않고 버리고 멈추는 원칙 |
| PENDING_VERIFICATION | 분사는 했으나 진화가 아직 확인되지 않은 화점 상태 |
| source-time TF | 지금이 아니라 "관측된 그 순간"의 좌표 변환 |
