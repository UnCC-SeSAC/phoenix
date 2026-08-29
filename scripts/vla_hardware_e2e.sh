#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${VLA_CONTAINER:-IntelPi}"
WORKSPACE="/ros2_ws/phoenix_vla"
LOG_DIR="${VLA_E2E_LOG_DIR:-/tmp}"
DRY_RUN="${VLA_E2E_DRY_RUN:-0}"
CAMERA_WAIT_SEC=8
STATUS_WAIT_SEC=15
LOCK_FILE="/tmp/vla_hardware_e2e.lock"
HEF_PATH="/shared/Hailo/models/baseline_yolo26_neural_norm.hef"
ONNX_PATH="/shared/Hailo/models/best_sim_postprocess.onnx"
JSON_PATH="/shared/Hailo/models/config_onnx_best_sim.json"

ENVIRONMENT='source /opt/ros/humble/setup.bash
source /home/ubuntu/third_party_ros2/third_party_ws/install/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
source /ros2_ws/phoenix_vla/install/setup.bash
export MACHINE_TYPE=MentorPi_Mecanum
export need_compile=True
export DEPTH_CAMERA_TYPE=ascamera
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONPATH=/usr/local/lib/python3.10/dist-packages:${PYTHONPATH:-}:/home/ubuntu/.local/lib/python3.10/site-packages
cd /'

PRODUCTION_PATTERN='depth_camera.launch.py|uncc_frontier.launch.py|preprocess_node|yolo.launch.py|yolo_node|detection_3d.launch.py|detection_3d_node|topic_bridge_vla.launch.py|vla_navigation_bridge.launch.py|fire_extinguisher.launch.py|fire_suppression_node|vla_spray_bridge'
EXPECTED_LAUNCH_PATTERNS=(
    'depth_camera.launch.py'
    'uncc_frontier.launch.py'
    'ros2 run image_pipeline preprocess_node'
    'yolo.launch.py'
    'detection_3d.launch.py'
    'topic_bridge_vla.launch.py'
    'vla_navigation_bridge.launch.py'
    'fire_extinguisher.launch.py'
)

usage() {
    echo "사용법: $0 {start|status|mission|stop}"
    echo "start 전 VLA_QWEN_ENDPOINT=http://<CURRENT_PC_IP>:8088/infer 설정 필요"
}

run() {
    if [[ "$DRY_RUN" == "1" ]]; then
        printf 'DRY-RUN:'
        printf ' %q' "$@"
        printf '\n'
        return 0
    fi
    "$@"
}

container_shell() {
    local command="$1"
    run docker exec -u root -w / "$CONTAINER" bash -lc "$ENVIRONMENT
$command"
}

production_processes() {
    docker exec "$CONTAINER" bash -lc \
        "ps -eo stat=,args= | awk '\$1 !~ /^Z/' | grep -E '$PRODUCTION_PATTERN' | grep -v -E 'grep -E|vla_hardware_e2e.sh' || true"
}

runtime_state() {
    local processes pattern count complete=1
    processes="$(production_processes)"
    if [[ -z "$processes" ]]; then
        echo "NONE"
        return
    fi
    for pattern in "${EXPECTED_LAUNCH_PATTERNS[@]}"; do
        count="$(grep -Ec "$pattern" <<<"$processes" || true)"
        if [[ "$count" -ne 1 ]]; then
            complete=0
        fi
    done
    if [[ "$complete" -eq 1 ]]; then
        echo "COMPLETE"
    else
        echo "PARTIAL_OR_DUPLICATE"
    fi
}

launch_component() {
    local name="$1"
    local command="$2"
    local full="$ENVIRONMENT
nohup setsid $command >'$LOG_DIR/e2e_${name}.log' 2>&1 </dev/null &"
    run docker exec -d -u root -w / "$CONTAINER" bash -lc "$full"
    echo "$name: 시작 요청 (root, cwd=/)"
}

start_preflight() {
    echo "boot_id: $(cat /proc/sys/kernel/random/boot_id 2>/dev/null || echo UNKNOWN)"
    if [[ "$DRY_RUN" == "1" ]]; then
        run lsusb
        run docker exec "$CONTAINER" lsusb
        run test -e /dev/ldlidar
        run docker exec "$CONTAINER" test -e /dev/ldlidar
        run docker exec "$CONTAINER" test -f "$HEF_PATH" -a -f "$ONNX_PATH" -a -f "$JSON_PATH"
        return 0
    fi
    lsusb | grep -qi '3482:6723' || { echo "Camera device: FAIL" >&2; return 4; }
    docker exec "$CONTAINER" lsusb | grep -qi '3482:6723' || {
        echo "Container Camera device: FAIL" >&2; return 4;
    }
    [[ -e /dev/ldlidar ]] || { echo "LD19 device: FAIL" >&2; return 4; }
    docker exec "$CONTAINER" test -e /dev/ldlidar || {
        echo "Container LD19 device: FAIL" >&2; return 4;
    }
    docker exec "$CONTAINER" test -f "$HEF_PATH" -a -f "$ONNX_PATH" -a -f "$JSON_PATH" || {
        echo "Hailo model set: FAIL" >&2; return 4;
    }
    echo "device/model preflight: PASS"
}

start_runtime() {
    local endpoint="${VLA_QWEN_ENDPOINT:-}"
    local state
    if [[ -z "$endpoint" ]]; then
        echo "VLA_QWEN_ENDPOINT가 필요함" >&2
        return 2
    fi
    start_preflight
    if [[ "$DRY_RUN" != "1" ]]; then
        state="$(runtime_state)"
        echo "runtime_state: $state"
        if [[ "$state" == "COMPLETE" ]]; then
            echo "동일 boot의 정상 production runtime을 재사용함"
            return 0
        fi
        if [[ "$state" == "PARTIAL_OR_DUPLICATE" ]]; then
            echo "부분·중복 production runtime을 1회 clean stop 후 다시 시작함"
            stop_runtime
            [[ -z "$(production_processes)" ]] || {
                echo "production runtime stop 미완료" >&2
                return 5
            }
        fi
    fi

    launch_component camera "ros2 launch peripherals depth_camera.launch.py"
    run sleep "$CAMERA_WAIT_SEC"
    launch_component base "ros2 launch uncc_example uncc_frontier.launch.py start_frontier:=false start_mission:=false start_vision:=false"
    launch_component preprocess "ros2 run image_pipeline preprocess_node --ros-args -r __node:=rgb_preprocess_node -p input_topic:=/ascamera/camera_publisher/rgb0/image -p camera_info_topic:=/ascamera/camera_publisher/rgb0/camera_info -p output_topic:=/image_enhanced -p output_camera_info_topic:=/image_enhanced/camera_info -p mode:=passthrough"
    launch_component yolo "ros2 launch image_pipeline yolo.launch.py model_path:=$HEF_PATH postprocess_path:=$ONNX_PATH backend:=hailo layout:=end2end class_names:='[fire,person]'"
    launch_component detection3d "ros2 launch image_pipeline detection_3d.launch.py"
    launch_component vla "ros2 launch fire_vla_bringup topic_bridge_vla.launch.py start_perception_bridge:=true llm_backend:=remote_qwen remote_qwen_endpoint:=$endpoint remote_qwen_timeout_sec:=10.0"
    launch_component navigation "ros2 launch uncc_example vla_navigation_bridge.launch.py"
    launch_component suppression "ros2 launch uncc_example fire_extinguisher.launch.py"
    echo "production runtime 시작 요청 완료. 로그: $LOG_DIR/e2e_*.log"
}

status_runtime() {
    local endpoint="${VLA_QWEN_ENDPOINT:-}"
    local observer output
    read -r -d '' observer <<'PY' || true
import json
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from nav2_msgs.action import NavigateToPose
from interfaces.action import SuppressFire
from tf2_ros import Buffer, TransformListener

class Status(Node):
    def __init__(self):
        super().__init__("vla_hardware_e2e_status")
        self.seen = {name: False for name in (
            "RGB", "DEPTH", "CAMERA_INFO", "IMAGE_ENHANCED", "YOLO_RESULT", "DETECTION3D"
        )}
        topics = (
            ("RGB", Image, "/ascamera/camera_publisher/rgb0/image"),
            ("DEPTH", Image, "/ascamera/camera_publisher/depth0/image_raw"),
            ("CAMERA_INFO", CameraInfo, "/ascamera/camera_publisher/rgb0/camera_info"),
            ("IMAGE_ENHANCED", Image, "/image_enhanced"),
            ("YOLO_RESULT", Detection2DArray, "/yolo_result"),
            ("DETECTION3D", String, "/fire/detections/status"),
        )
        for name, msg_type, topic in topics:
            self.create_subscription(
                msg_type, topic, lambda _msg, key=name: self.seen.__setitem__(key, True),
                qos_profile_sensor_data,
            )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.suppression = ActionClient(self, SuppressFire, "/suppress_fire")
        self.create_timer(15.0, self.finish)

    def finish(self):
        for name, value in self.seen.items():
            print(f"{name}: {'PASS' if value else 'FAIL'}")
        tf_ok = self.tf_buffer.can_transform(
            "map", "base_footprint", Time(), timeout=Duration(seconds=0.2)
        )
        print(f"MAP_TO_BASE_FOOTPRINT: {'PASS' if tf_ok else 'FAIL'}")
        print(f"NAV2_ACTION_SERVER: {'PASS' if self.nav.server_is_ready() else 'FAIL'}")
        print(
            f"SUPPRESSION_ACTION_SERVER: "
            f"{'PASS' if self.suppression.server_is_ready() else 'FAIL'}"
        )
        rclpy.shutdown()

rclpy.init()
node = Status()
try:
    rclpy.spin(node)
except Exception:
    pass
PY

    if [[ "$DRY_RUN" == "1" ]]; then
        run docker exec -u root -w / "$CONTAINER" bash -lc "$ENVIRONMENT
python3 -c '<integrated rclpy status observer: ${STATUS_WAIT_SEC}s>'"
        run curl --max-time 3 "${endpoint%/infer}/health"
        return 0
    fi

    if ! output="$(docker exec -e STATUS_OBSERVER="$observer" -u root -w / "$CONTAINER" bash -lc "$ENVIRONMENT
timeout $((STATUS_WAIT_SEC + 3))s python3 -c \"\$STATUS_OBSERVER\"" 2>/dev/null)"; then
        echo "integrated_status: UNKNOWN (조회 실패)"
    else
        printf '%s\n' "$output"
    fi

    if [[ -z "$endpoint" ]]; then
        echo "QWEN_HEALTH: UNKNOWN (VLA_QWEN_ENDPOINT 미설정)"
    elif curl -fsS --max-time 3 "${endpoint%/infer}/health" >/dev/null 2>&1; then
        echo "QWEN_HEALTH: PASS"
    else
        echo "QWEN_HEALTH: UNKNOWN (HTTP 조회 실패)"
    fi
}

mission_once() {
    local text="${VLA_MISSION_TEXT:-화재를 찾아 진압해줘}"
    local mission_id="mission_fire_$(date -u +%Y%m%dT%H%M%S)_$$"
    local payload
    payload="$(printf '{\"mission_id\":\"%s\",\"text\":\"%s\"}' "$mission_id" "$text")"
    if [[ "$DRY_RUN" == "1" ]]; then
        run docker exec -e "VLA_MISSION_PAYLOAD=$payload" -u root -w / "$CONTAINER" bash -lc "$ENVIRONMENT
ros2 topic pub --once -w 1 /vla/mission std_msgs/msg/String \"{data: '\$VLA_MISSION_PAYLOAD'}\""
        echo "mission_id: $mission_id (dry-run)"
        return 0
    fi
    exec 9>"$LOCK_FILE"
    if ! flock -n 9; then
        echo "다른 mission 명령이 실행 중임. 중복 발행하지 않음" >&2
        return 3
    fi
    docker exec -e "VLA_MISSION_PAYLOAD=$payload" -u root -w / "$CONTAINER" bash -lc "$ENVIRONMENT
ros2 topic pub --once -w 1 /vla/mission std_msgs/msg/String \"{data: '\$VLA_MISSION_PAYLOAD'}\""
    echo "mission_id: $mission_id (1회 발행)"
}

stop_runtime() {
    local cancel_request="{goal_info: {goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, stamp: {sec: 0, nanosec: 0}}}"
    if [[ "$DRY_RUN" == "1" ]]; then
        run docker exec -u root -w / "$CONTAINER" bash -lc "$ENVIRONMENT
timeout 3 ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal '$cancel_request' || true
timeout 3 ros2 service call /suppress_fire/_action/cancel_goal action_msgs/srv/CancelGoal '$cancel_request' || true"
        run docker exec -u root -w / "$CONTAINER" bash -lc "$ENVIRONMENT
ros2 topic pub --once /ros_robot_controller/set_motor ros_robot_controller_msgs/msg/MotorsState \"{data: [{id: 1, rps: 0.0}, {id: 2, rps: 0.0}, {id: 3, rps: 0.0}, {id: 4, rps: 0.0}]}\""
        run docker exec "$CONTAINER" bash -lc "pkill -TERM -f '$PRODUCTION_PATTERN'"
        return 0
    fi
    container_shell "timeout 3 ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal '$cancel_request' >/dev/null 2>&1 || true
timeout 3 ros2 service call /suppress_fire/_action/cancel_goal action_msgs/srv/CancelGoal '$cancel_request' >/dev/null 2>&1 || true"
    sleep 1
    container_shell "ros2 topic pub --once /ros_robot_controller/set_motor ros_robot_controller_msgs/msg/MotorsState \"{data: [{id: 1, rps: 0.0}, {id: 2, rps: 0.0}, {id: 3, rps: 0.0}, {id: 4, rps: 0.0}]}\""
    echo "Robot stop: active goal cancel + explicit four-motor zero 전송"
    docker exec "$CONTAINER" bash -lc "pkill -TERM -f 'fire_extinguisher.launch.py|fire_suppression_node' || true"
    sleep 2
    echo "Pump OFF: suppression SIGTERM cleanup 적용"
    docker exec "$CONTAINER" bash -lc "pkill -TERM -f '$PRODUCTION_PATTERN' || true"
    echo "production runtime 종료 요청 완료"
}

if [[ $# -ne 1 ]]; then
    usage
    exit 2
fi

case "$1" in
    start) start_runtime ;;
    status) status_runtime ;;
    mission) mission_once ;;
    stop) stop_runtime ;;
    *) usage; exit 2 ;;
esac
