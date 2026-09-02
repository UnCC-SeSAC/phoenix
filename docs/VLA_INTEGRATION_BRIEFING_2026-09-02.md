# VLA Integration Briefing — 2026-09-02

## 기준선

- Branch: `integration/vla-robot-e2e`
- HEAD: `114d71f2c1b119c0df43dd2acceede8ba721b8c6`
- Fire VLA core regression: `276 PASS`
- image_pipeline regression: `355 PASS`
- Firefighter UI 포함 SW-only E2E: `PASS`
- 실제 fire/person, Nav2, suppression을 함께 쓰는 최종 Hardware E2E: 미완료

## 현재까지 완료한 범위

### VLA Mission과 WorldModel

- Mission 시작 시 이전 person/fire observation과 association을 제거하고 Robot pose,
  map, home pose는 유지한다.
- 새 person은 Qwen `REPORT_PERSON` action 없이 기존 topic/schema로 자동 보고한다.
  UI ACK가 늦어도 판단을 막지 않으며 같은 Mission의 동일 person은 한 번만 보고한다.
- ID 없는 reported person 관측은 association TTL을 넘더라도 같은 Mission, 같은 class,
  `0.50 m` radius 안이면 기존 ID를 유지한다. Mission boundary, unreported person TTL과
  fire association lifecycle은 유지한다.
- Demo 관계값 `person_fire_risk_distance_m=0.10 m` 안의 가장 가까운 person/fire를
  `threatens_person`과 `threatened_person_id`로 WorldModel, Qwen context, UI에 제공한다.

### Qwen과 physical decision

- 첫 구조화 응답에서 action, target, Mission scope를 함께 확정한다.
- Scope는 `FIRE_ONLY`, `PERSON_FIRE`, `FULL_EXPLORATION`만 허용하고 Mission 중 변경을
  막는다.
- 분사거리 밖 ACTIVE fire에 잘못 나온 `EXTINGUISH`는 유효한 map target일 때만 같은
  target의 `NAVIGATE_TO`로 한 번 교정한다.
- Nav2 성공 뒤 동일 ACTIVE fire, fresh/valid pose와 target, `0.8 m` 이내, 의미 있는
  WorldModel 변화 없음이 모두 성립하면 Qwen 재호출 없이 `EXTINGUISH`를 한 번 연결한다.
- 중복 Mission, Nav2 goal, suppression과 Pump action을 차단하는 기존 계약을 유지한다.

### 소화 완료와 UI

- Suppression success 직후 즉시 소화 완료로 처리하지 않는다.
- `PENDING_VERIFICATION`, delay `0.5 s`, 유효한 fire 미검출 3회, timeout `5 s` 후
  `ACTIVE` 복귀 계약을 유지한다. Invalid, future, stale observation은 증거로 쓰지 않는다.
- UI SW-only E2E에서 Mission 입력, person 자동 보고, 위험 관계, Qwen 1회, Mock Nav2,
  deterministic extinguish, Mock suppression, verification, Mission/UI `COMPLETED`, 중복
  action 0을 확인했다.

### 실제 Hardware에서 확인한 범위

- Camera, split HEF + companion ONNX, Depth, source-time TF/map, LiDAR, Nav2 server,
  Qwen HTTP와 VLA 연결을 개별 확인했다.
- Fire floor ROI는 실측 `0.45~0.46 m`에서 `0.456~0.457 m`를 선택했다. 최대 오차는
  약 `0.007 m`, 세 건 모두 `fallback_below`였으며 현재 배치에서는 과거 `1.45 m`
  과대 측정이 재현되지 않았다.
- Person association 반복 생성 원인을 수정하고 회귀를 통과했다.
- 실제 Nav2 접근 이후 Servo/Pump 분사와 화염 제거, terminal success는 아직 완료되지
  않았다.

## 현재 안전 blocker

Suppression은 현재 `DISABLED`다. 현장 관찰은 Servo physical pin 7 (`BCM4`), Pump
physical pin 8 (`BCM14`)이었으나 Hardware team의 최종 pin/polarity 계약은 아직 없다.
Suppression node startup만으로 Pump가 작동한 incident도 있었다. 실패한
`active_high=True` 실험은 commit하지 않았다.

Hardware team이 BCM pin, physical pin, active level, 전원과 공통 GND를 확정하고 불
OFF에서 startup 무동작 단독 시험을 통과하기 전에는 `fire_suppression_node`,
`fire_extinguisher`, `vla_spray_bridge`, GPIO 접근과 전체 Hardware wrapper start를
실행하지 않는다.

## 최신 팀 코드 비교

2026-09-02에 모든 remote branch를 fetch/prune하고 실제 diff와 PR을 비교했다. Main은
비교 이력일 뿐 VLA production source로 자동 사용하지 않았다.

| 범위 | 최신 근거 | 실제 내용 | VLA integration 판정 |
|---|---|---|---|
| FSM 소화 | PR #112, `fc060a5`, `1867ba1` | StateManager/MissionExecutor가 Nav2와 suppression을 소유하고 fire status service와 GPIO action server를 실행 | 보류. VLA Navigation Bridge/suppression owner와 동시 사용 시 이중 owner이며 실제 배선 계약도 불일치 |
| person/fire SLAM masking | PR #114, `ffdf1be`, `56788d6`, 최신 `acdcd41` 계열 | `/mission/found_targets`를 Nav2 keepout mask로 변환. 최신 branch는 즉시 등록, keepout 이탈용 MissionExecutor 연동 추가 | 보류. 현재 VLA는 `/vla/world_model` 계약이며 `/mission/found_targets` producer와 MissionExecutor를 실행하지 않음 |
| Nav2 tuning | `37cd5ad`, `da94688`, `fix/fire_keepout_person_mask_range@fb9aa71` | goal yaw, tolerance, inflation, keepout 이탈과 controller/odometry 설정 변경 | 보류. keepout/FSM 전체 runtime에 맞춘 값이며 VLA Hardware 재현 증거 없이 선별 적용 불가 |
| Hailo 최적화 | `albitro/image_detection_opt@c629fc0` | detection 연산 CPU 최적화 | 보류. 동일 HEF/postprocess 출력과 Pi latency/CPU 실측 후 검토 |
| class별 confidence | `yolo_conf_diff@b70796c`, main PR #121 | fire/person threshold를 launch parameter로 분리 | 보류. 현재 VLA production threshold와 WorldModel threshold 계약을 함께 검증해야 함 |

이번 비교에서 즉시 병합할 commit은 없다. 특히 branch 전체 merge는 현재 Mission scope,
Qwen 정상 1회, VLA goal owner, suppression verification과 Hardware safety 계약을
깨뜨릴 가능성이 크다. 팀 FSM의 실물 성공은 팀 실행 모드의 유효한 성과지만 VLA 모드의
소유권과 terminal contract가 검증됐다는 뜻은 아니다.

## 통합 방향

SLAM masking은 기능 가치가 있다. 다만 팀 node를 그대로 시작하지 않고 다음 조건이
충족될 때 별도 선별 작업으로 다룬다.

1. VLA Adapter가 `/vla/world_model`의 ACTIVE fire와 person map position을 keepout 입력
   계약으로 변환한다.
2. Nav2 goal owner는 VLA Navigation Bridge 하나만 유지한다.
3. Mask radius, goal tolerance와 spray stand-off가 서로 모순되지 않는지 Hardware에서
   확인한다.
4. Mask가 target 접근 자체를 막거나 로봇을 자기 keepout 안에 가두지 않는지 확인한다.
5. 팀 commit의 node/config/test 중 필요한 최소 범위만 선별하고 FSM MissionExecutor와
   suppression owner는 가져오지 않는다.

FSM 소화 구현은 팀/rule-based 모드의 source of truth로 유지한다. VLA 모드에는 핀,
극성, startup 무동작과 실제 action result가 확정된 suppression driver 계약만 선별
공유하며 StateManager/MissionExecutor의 goal ownership은 가져오지 않는다.

## 앞으로의 마일스톤

### M1. Suppression Hardware 계약 확정

- Hardware team이 Servo/Pump BCM pin, physical pin, active-high/low, 전원과 공통 GND를
  확정한다.
- 불 OFF, Mission/Nav2 0에서 suppression node startup 무동작을 확인한다.
- Servo 단독, Pump 단독, Pump OFF와 cancel/SIGTERM cleanup을 순서대로 확인한다.

완료 기준: startup actuator command 0, 명시적 단일 goal에서만 동작, 종료 후 Pump OFF.

### M2. Person+fire pre-spray Hardware E2E

- 새 Mission boundary 뒤 fresh person/fire를 생성한다.
- person 보고 1회, 0.10m 위험 관계, Qwen 1회와 Nav2 goal 1회를 확인한다.
- Robot stop과 deterministic `EXTINGUISH`까지 확인하되 M1 전에는 물리 dispatch를 막는다.

완료 기준: 중복 report/goal/spray 0, target과 path 방향 일치, Robot stop 확인.

### M3. Fire-only 실제 소화 E2E — Issue #89

- Fresh ACTIVE fire → Mission 1회 → Qwen → Nav2 접근 → 완전 정지 → suppression 1회
  → Pump OFF → 유효한 미검출 3회 → fire `EXTINGUISHED`를 연속 실행한다.

완료 기준: 실제 화염 제거와 terminal success. Software/개별 readiness만으로 대체하지
않는다.

### M4. Natural-language Mission/UI acceptance — Issue #91

- 소방관 UI 자연어 Mission부터 실제 Robot 실행, WorldModel과 UI `COMPLETED`까지 한 번의
  Hardware run으로 확인한다.
- 같은 실행 증거로 #89 기술 E2E와 #91 Mission/UI acceptance를 구분해 기록한다.

### M5. Semantic keepout 선별 통합

- M3의 실제 접근 경로에서 fire/person 저상 장애물 문제가 재현되거나 masking 필요성이
  확인되면 `/vla/world_model` adapter 기반 keepout을 별도 Issue로 구현한다.
- 팀 최신 keepout lineage를 기준으로 SW costmap test 후 Hardware path를 검증한다.

### M6. 성능과 운영 안정화

- 실제 Pi에서 Hailo 최적화 전후 detection 결과 동일성, latency와 CPU를 측정한다.
- Camera RGB stream 지속성, PC–Pi network와 battery/undervoltage를 운영 checkpoint로
  정리하되 단일 timeout을 production failure로 오판하지 않는다.

## 다음 실행의 시작점

지금 당장 Hardware를 재시도하지 않는다. 먼저 M1의 배선 계약을 확정한다. 그다음 현재
integration SHA를 isolated workspace에 배포하고, 기존 canonical Camera-first runtime을
정확히 한 번 시작한다. 과거 Mission/entity/goal은 재사용하지 않으며 M2 → M3 → M4
순서로 실제 증거를 쌓는다.
