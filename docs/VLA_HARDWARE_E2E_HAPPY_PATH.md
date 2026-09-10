# VLA Hardware E2E Happy Path

이 문서는 `integration/vla-robot-e2e` Hardware E2E의 authoritative 1-page
runbook이다. 현재 software 기준은
`0a79c462e770eac4bf2765b701f796d1447d8b12`이다. 과거 실행 기록은 증거이며
현재 절차나 시험값을 대체하지 않는다.

## 고정 실행 계약

- Pi workspace는 `/ros2_ws/phoenix_vla`, container는 `IntelPi`, runtime user는
  `root`, working directory는 `/`이다. 팀 workspace `/ros2_ws/phoenix`는 건드리지
  않는다.
- Wrapper는 `scripts/vla_hardware_e2e.sh`이다. Camera를 정확히 한 번 먼저 시작하고
  8초 뒤 나머지 stack을 시작한다.
- 각 start는 `/tmp/phoenix_vla_e2e/<boot_id>_<UTC timestamp>_<pid>/` 아래에
  component 로그, PID/PGID와 `ROS_LOG_DIR`을 남긴다.
- Pump는 BCM14, Servo는 BCM13, Nav2 XY goal tolerance는 `0.05 m`이다.
- Production 기본값은 `navigation_standoff_m=0.15 m`, `spray_range_m=0.30 m`다.
  다음 시험의 `0.35/0.45 m`는 새 노즐용 **Hardware 미검증 후보**이며 성공 전
  production 기본값으로 동결하지 않는다.
- PC Qwen은 Codex sandbox가 아니라 Intel GPU와 `/dev/dri`가 보이는 실제 Ubuntu
  host에서 실행한다. 이미 `/health`가 HTTP 200이면 재시작하지 않는다.

## 새 SD카드 1회 준비

- `interfaces`를 먼저 build해 `from interfaces.action import SuppressFire`가 되는지
  확인한 뒤 관련 package를 build한다.
- Current team model is the Git-untracked `/shared/yolo26s/best_neural.hef`; keep
  `best_sim_postprocess.onnx` and `config_onnx_best_sim.json` in the same directory.
  Verify all three against the team-provided artifacts rather than substituting the
  historical workspace-local HEF.
- canonical root 환경에서 `gpiozero`와 `lgpio` import를 확인한다. 임의 pip/apt
  대체나 팀 workspace의 build/install 공유는 하지 않는다.

## 발표용 명령

Ubuntu PC 실제 GPU host에서 Qwen을 시작한다.

```bash
cd /home/chopper/Downloads/fire_vla_ros2_integration
PYTHONPATH=src/fire_vla_core \
  /home/chopper/Downloads/fire_vla_ros2_integration/.venv-xpu-qwen3/bin/python \
  -m fire_vla_core.qwen_inference_server \
  --host 0.0.0.0 --port 8088 --backend transformers \
  --model-id Qwen/Qwen3-1.7B --device xpu:0 --max-new-tokens 64
curl -fsS http://127.0.0.1:8088/health
```

Mac 또는 운영 PC에서 Pi에 접속해 wrapper를 실행한다. `<PC_IP>`와 `<PI_IP>`는
당일 주소를 사용한다.

```bash
ssh -CY uncc@<PI_IP>
cd /ros2_ws/phoenix_vla
export VLA_QWEN_ENDPOINT=http://<PC_IP>:8088/infer
export VLA_NAVIGATION_STANDOFF_M=0.35
export VLA_SPRAY_RANGE_M=0.45
scripts/vla_hardware_e2e.sh start
scripts/vla_hardware_e2e.sh status
```

Browser에서 `http://<PI_IP>:8080`을 열어 control mode를 `VLA`로 선택하고 Mission을
정확히 한 번 실행한다. 종료는 Pi SSH에서 수행한다.

```bash
cd /ros2_ws/phoenix_vla
scripts/vla_hardware_e2e.sh stop
```

## 최소 운영 순서

1. 불 OFF, 충분한 배터리, Pump OFF를 확인하고 로봇을 평평한 바닥에 먼저 배치한다.
   충전선·전원선을 분리한 뒤 runtime 중 손으로 옮기지 않는다.
2. HEAD, clean tracked worktree와 install 일치를 확인한다. 같은 SHA의 검증 install은
   다시 build하지 않는다.
3. PC Qwen과 Pi→Qwen `/health` HTTP 200을 확인한다.
4. 위 override를 명시하고 wrapper `start`를 정확히 한 번 실행한다. start/status의
   `VLA_PARAMETERS`가 `0.35/0.45`인지 확인한다.
5. wrapper `status`를 한 번 사용해 RGB, Depth, CameraInfo, `/image_enhanced`,
   `/yolo_result`, Detection3D, map→base TF, Nav2, Suppression, Qwen을 확인한다.
   통합 observer timeout은 `DIAGNOSTIC_UNKNOWN`이지 production FAIL이 아니다.
6. Nav2 PASS는 composable components loaded, `bt_navigator` ACTIVE와 현재
   `/navigate_to_pose` server가 모두 있어야 한다. stale DDS endpoint나 process
   존재만으로 PASS 판정하지 않는다.
7. Pump/Servo startup 무동작, Mission/action 없음, control mode `VLA`, 정지 pose
   5초 안정을 확인한다.
8. 넓은 무광 비가연성 받침 위의 불을 켜고 작업자는 화각과 이동 경로에서 빠진다.
   Floor ROI valid pixel 0의 `unknown` Depth에는 Mission을 발행하지 않는다.
9. Fresh ACTIVE fire와 유효 Depth/map을 확인한 뒤 새 `FIRE_ONLY` Mission ID를 한 번
   사용한다. Qwen→NAVIGATE_TO→Nav2→Robot stop→EXTINGUISH→Servo/Pump→유효
   미검출 3회→EXTINGUISHED→Mission COMPLETED를 재발행 없이 연속 관찰한다.
10. wrapper `stop` 후 wrapper-owned active process와 orphan Nav2가 0인지, zombie가
    active로 집계되지 않는지, Pump와 바퀴가 실제 정지했는지 확인한다.

## 시험 근거와 다음 판정

기존 노즐의 최초 물리 소화는 `0.15/0.30 m`에서 PASS했다. 새 노즐 실측은
앞바퀴 축–불꽃 `40 cm`, 렌즈–불꽃 `38 cm`, 노즐 끝–불꽃 `36 cm`이며 노즐 거리
`36 cm`에서 중앙 착탄했다. 이 때문에 다음 한 번의 Hardware E2E만 `0.35/0.45 m`
override로 수행한다. Camera/SLAM Map 실제 UI 표시와 terminal SUCCESS까지 PASS한
뒤에만 이를 production 기본값으로 동결한다.

## 최신 모델의 P0/P1 선행 검증

`0a79c46` 기준 팀 모델은 `/shared/yolo26s/best_neural.hef`, class order는
`fire, person`이다. 2026-09-10 정지 관측에서 production `conf=0.75`는 staged
fire/person을 발행하지 않았지만 runtime-only `0.01` 진단에서는 person 약 `0.704`,
fire 약 `0.500`까지 후보가 확인됐다. UI Camera overlay로 bbox가 실제 대상을 가리키는지
확인할 수 있다. `0.01`은 다수 저신뢰 후보도 통과시키므로 Mission/Nav2/Suppression용
확정값이 아니다.

따라서 다음 Hardware 순서는 #89의 P0(의도한 bbox의 Depth/map 및 base-frame 실측
비교) → P1(반복 score 분포와 threshold 평가)이다. 최신 모델에서 P0/P1이 끝나기 전에는
과거 물리 소화 성공을 최신 Hardware E2E PASS로 확장하지 않는다.

배터리 부족으로 Pi가 종료되거나 SSH가 timeout되면 해당 cycle의 runtime,
localization, Mission과 결과를 무효로 처리한다. 종료 성공을 추측하거나 같은
side-effect 명령을 재발행하지 않는다.
