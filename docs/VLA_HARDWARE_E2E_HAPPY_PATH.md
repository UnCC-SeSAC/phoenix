# VLA Hardware E2E Happy Path

이 문서는 `integration/vla-robot-e2e` Hardware E2E의 authoritative 1-page
runbook이다. 기준은 `76251b3f16ceffc6f680b628a6a6f6ce399e2d8f`이다. 과거
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
- Pump는 BCM14, Servo는 BCM13이다. 최초 물리 소화 성공 기준인
  `spray_range_m=0.30 m`, fire stand-off `0.15 m`, 실제 Nav2 XY goal tolerance
  `0.05 m`를 동결한다. 후속 terminal 수정에서 이 geometry를 변경하지 않는다.

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

## 최초 물리 소화 성공 기준

`76251b3` Hardware 실행에서 Nav2는 약 `4.28 s`에 `SUCCEEDED`했고 도착 후
WorldModel의 base→fire 거리는 약 `0.198 m`였다. Robot stop 뒤 suppression은 정확히
한 번 실행됐으며 Servo/Pump가 실제 작동했다. 물줄기는 불꽃보다 살짝 뒤에
착탄했지만 실제 화염 제거는 `PASS`였다.

소프트웨어 완료는 별개로 `FAIL`이다. `fire_status_service`가 소화 후
`관찰 구간 내 YOLO 감지 기록 없음`을 성공 증거로 처리하지 못해 fire는 `ACTIVE`,
Mission은 `RUNNING`으로 남았다. 다음 변경 범위는 verification→`EXTINGUISHED`→Mission
`COMPLETED` 경계뿐이며, 위 성공 geometry와 Pump/Servo 설정은 변경하지 않는다.
