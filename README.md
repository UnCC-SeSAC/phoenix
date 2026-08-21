# Phoenix

Phoenix는 ROS 2 기반 화재 탐사 로봇 프로젝트다. Camera와 Hailo HEF YOLO로
사람과 화점을 탐지하고, Depth·CameraInfo·TF로 2D `map(x,y)` 위치를 만든다.
VLA Brain은 자연어 Mission과 Semantic WorldModel을 바탕으로 행동을 선택하며,
Nav2와 Robot controller가 실제 이동을 담당한다.

## 시스템 구조

```text
Raspberry Pi
Camera / HEF YOLO / Depth / TF / SLAM / Nav2 / VLA runtime
        │
        ├─ Perception → Semantic WorldModel
        │
        └─ compact WorldModel
                 ↓ HTTP/JSON
Ubuntu Local PC
Qwen3 inference
                 ↓ action / target / reason
Raspberry Pi
Resolver → Validator → Dispatcher → ROS 2 action
        ↓
Nav2 / Report / Pump·Servo boundary
        ↓
WorldModel / Firefighter UI
```

PC는 Qwen model loading과 inference만 담당한다. raw image, Depth, TF, Nav2,
`cmd_vel`은 Pi의 Robot runtime에 남는다. 시스템의 authoritative navigation과
semantic 좌표는 2D `map` frame이다.

## 주요 기능

- person/fire detection과 source-time `map(x,y)` 위치화
- Semantic WorldModel과 stable entity ID fallback
- Qwen 기반 `ActionDecision(action, target, reason)`
- Resolver, Validator, Dispatcher 안전 실행 경계
- Nav2 navigation, person report, suppression lifecycle
- VLA/Rule-based 2모드 Firefighter UI
- 실패·취소·timeout 결과의 WorldModel 반영

## 기술 스택

- ROS 2 Humble/Jazzy
- Raspberry Pi, Hailo, YOLO
- Depth Camera, CameraInfo, TF, SLAM Toolbox
- Nav2, Robot controller
- Qwen3, PyTorch XPU, HTTP/JSON
- Python, pytest, colcon

## 문서

- [공개 문서 안내](docs/README.md)
- [현재 VLA 데이터 아키텍처](docs/CURRENT_VLA_DATA_ARCHITECTURE.md)
- [Rule-based UI 계약](docs/RULE_BASED_UI_CONTRACT.md)

## 최소 실행

ROS 2와 package dependency가 준비된 개발 환경에서 software demo를 실행한다.

```bash
colcon build --packages-select   fire_vla_interfaces fire_vla_core fire_vla_bringup
source install/setup.bash
ros2 launch fire_vla_bringup firefighter_ui_mock.launch.py
```

기본 UI는 `http://127.0.0.1:8080`에서 열린다. 실제 Robot, Nav2 motion,
Pump/Servo 실행은 Hardware 환경과 별도 안전 승인이 필요하다.
