# VLA Hardware Resume Runbook

기준 architecture는 Pi-local Robot control plane과 PC Qwen HTTP inference다.
PC↔Pi ROS 2 DDS 진단은 이 절차에 포함하지 않는다.

1. Robot 전원과 새 Wi-Fi NIC 연결을 확인한다.
2. PC NIC Power Save를 OFF로 전환한다.
3. `ssh lemma@10.42.0.1`로 Pi 접속을 확인한다.
4. Pi repository에서 `git fetch origin --prune` 후
   `integration/vla-robot-e2e` 최신 commit을 checkout한다.
5. Git 배포가 불가능하면 PC에서
   `scripts/create_pi_vla_bundle.sh /tmp/phoenix-pi-vla.tar.gz`를 실행해
   bundle을 만들고 USB 또는 동작하는 TCP 전송 경로로 Pi에 전달한다.
6. Pi에서 기존 verified underlay를 유지하고 current integration overlay의
   `fire_vla_core`, `fire_vla_bringup`, `uncc_example`만 build/source한다.
7. Hardware → LiDAR → SLAM → Nav2 → TF → VLA Navigation Bridge를 기동한다.
8. Pi에서 VLA Orchestrator를 `llm_backend=remote_qwen`으로 기동한다.
9. PC에서 `qwen_inference_server --host 0.0.0.0 --port 8088`을 기동한다.
10. Pi에서 PC의 `/health`와 synthetic `/infer` HTTP 요청 각 1건을 확인한다.
11. fresh Robot pose 기준 약 0.95 m 전방에 fresh `person_0001`을 1회 주입한다.
12. Mission `인명을 우선 확인해`를 정확히 1회 제출하고 Validator PASS를 확인한다.
13. `/vla/navigation_goal`과 NavigateToPose가 각각 정확히 1건인지 확인한다.
14. non-zero velocity, 실제 이동 방향, 자동 정지와 `/vla/navigation_result`를 확인한다.
15. runtime을 안전 종료하고 PC NIC Power Save를 원래 설정으로 복원한다.

Unexpected motion이면 active goal을 cancel하고 velocity stop을 확인한 뒤 검증된
halt path로 motor zero를 보낸다. Pump/Servo는 별도 명시적 작업 없이는 사용하지 않는다.
