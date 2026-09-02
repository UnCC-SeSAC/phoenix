# VLA Hardware E2E Happy Path

이 문서는 `integration/vla-robot-e2e` Hardware E2E의 authoritative 1-page
runbook이다. 기준은 `414abc6c57f61f6bdf44b5212f512d3937afc811`이다. 과거
실행 기록은 증거일 뿐 현재 절차를 대체하지 않는다.

## 고정 계약

- Pi workspace: `/ros2_ws/phoenix_vla`; container: `IntelPi`; runtime user:
  `root`; working directory: `/`.
- Wrapper: `scripts/vla_hardware_e2e.sh`; Camera를 먼저 한 번 시작하고 8초 뒤
  나머지 stack을 한 번 시작한다.
- 각 start는 `/tmp/phoenix_vla_e2e/<boot_id>_<UTC timestamp>_<pid>/`를 만들고
  모든 ROS 로그를 그 아래 `ros/`에 저장한다.
- PC Qwen은 Codex sandbox가 아니라 Intel GPU와 `/dev/dri`가 보이는 실제 PC
  host에서 canonical Python으로 시작한다. 이미 `/health`가 HTTP 200이면 재시작하지
  않는다.
- Pump는 BCM14, Servo는 BCM13이다. `spray_range_m=0.25 m`, fire stand-off는
  `0.20 m`, 실제 Nav2 XY goal tolerance는 `0.05 m`다. 최악 도착 fire 거리는
  `0.20 + 0.05 = 0.25 m`다.

## 실행 순서

1. 불 OFF, 충분한 배터리, Pump OFF를 확인한다. 로봇을 먼저 안전한 바닥 출발
   위치에 놓고 이후 Mission 종료까지 손으로 옮기지 않는다.
2. Pi HEAD와 tracked worktree를 확인한다. SHA가 같은 검증 install은 다시 build하지
   않는다. 이전 runtime이 있으면 wrapper `stop`으로 종료하고 zombie를 제외한 active
   production process 0개를 확인한다.
3. 실제 PC host에서 Qwen `/health` HTTP 200과 Pi→Qwen HTTP 200을 확인한다.
4. 현재 PC 주소로 `VLA_QWEN_ENDPOINT=http://<PC_IP>:8088/infer`를 설정하고 wrapper
   `start`를 정확히 한 번 실행한다. 중복 start를 보내지 않는다.
5. wrapper `status`를 한 번만 사용해 실제 RGB, Depth, CameraInfo,
   `/image_enhanced`, `/yolo_result`, Detection3D, map→base TF, Nav2,
   Suppression, Qwen을 확인한다. 통합 observer timeout은
   `DIAGNOSTIC_UNKNOWN`이며 그 자체로 production FAIL이 아니다.
6. Nav2 PASS에는 composable components load, `bt_navigator` lifecycle ACTIVE,
   현재 `/navigate_to_pose` server endpoint가 모두 필요하다. stale DDS endpoint나
   process 존재만으로 PASS로 판정하지 않는다.
7. Pump/Servo startup 무동작과 Mission/action 없음 확인 후 control mode를 VLA로 한
   번 설정한다. 정지한 map→base pose를 5초 관찰해 큰 위치/yaw jump가 없을 때만
   진행한다.
8. Observer를 먼저 준비한다. 넓은 무광 비가연성 받침 위의 불을 켜고 작업자는 즉시
   화각과 이동 경로에서 빠진다. Floor ROI valid pixel이 0인 `unknown` Depth에는
   Mission을 발행하지 않는다. 40~45 cm 배치의 실제 성공값은 약 `0.471 m`였다.
9. 유효 Depth/map의 fresh ACTIVE fire를 확인한 뒤 새 `FIRE_ONLY` Mission ID를
   정확히 한 번 발행한다. Qwen→NAVIGATE_TO→Nav2 도착→Robot stop→EXTINGUISH
   →Servo/Pump→미검출 3회→fire EXTINGUISHED→Mission COMPLETED를 중간 재발행 없이
   연속 관찰한다.
10. 종료 시 wrapper `stop`으로 Robot stop과 Pump OFF 후 전체 runtime을 종료한다.

## 중단 기준

실제 위험 이동, Robot stop 실패, Pump 비정상 작동, Nav2 terminal failure,
suppression failure 또는 물리 위험에서만 즉시 중단한다. Mission/Nav2/Spray는 timeout을
이유로 추정 재발행하지 않는다. 배터리 부족이나 수동 이동 후에는 이전 localization,
fire, Mission, goal을 모두 폐기하고 바닥-start 절차를 처음부터 적용한다.

현재 미완료 상태는 수정된 바닥-start 절차의 최종 실제 소화, 화염 제거 및 Mission
`COMPLETED` 확인이다. VLA Mission 단독 STOP API와 통합 status observer timeout의
원인은 아직 미확정이다.
