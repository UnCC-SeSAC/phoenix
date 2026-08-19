# VLA Hardware Perception Resume Runbook

기준 architecture는 Pi-local Robot control plane과 PC Qwen HTTP inference다.
PC↔Pi ROS 2 DDS 진단은 이 절차에 포함하지 않는다.

1. `lspci`에서 Hailo accelerator를 확인한다.
2. `/dev/hailo*` device node를 확인한다.
3. `hailortcli scan`으로 HailoRT device 연결을 확인한다.
4. `best_filtered_hailo10h.hef` offline inference와 output shape를 확인한다.
5. ASCAMERA RGB와 CameraInfo topic/rate를 확인한다.
6. preprocess를 passthrough mode로 기동한다.
7. Hailo YOLO `/yolo_result`의 class/FPS/latency를 확인한다.
8. RGB/depth source timestamp synchronization을 확인한다.
9. production `/fire/detections` pixel/depth JSON을 확인한다.
10. Pi-local WorldModel의 `person_0001`/`fire_0001` map pose를 확인한다.
11. PC actual Qwen server health를 확인한다.
12. Mission `인명을 우선 확인해`를 정확히 1회 제출한다.
13. Resolver와 production Validator PASS를 확인한다.
14. `/vla/navigation_goal`과 Nav2 goal이 각각 정확히 1건인지 확인한다.
15. 실제 이동, 자동 정지, navigation result와 WorldModel 반영을 확인한다.

class contract는 `class_id 0 = fire`, `class_id 1 = person`이다. Unexpected motion이면
active goal을 cancel하고 velocity stop을 확인한 뒤 검증된 halt path로 motor zero를
보낸다. Pump/Servo는 별도 명시적 작업 없이는 사용하지 않는다.
