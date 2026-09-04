#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${VLA_CONTAINER:-IntelPi}"
WORKSPACE="/ros2_ws/phoenix_vla"
LOG_ROOT="${VLA_E2E_LOG_ROOT:-/tmp/phoenix_vla_e2e}"
CURRENT_RUN_FILE="$LOG_ROOT/current_run"
RUN_LOG_DIR=""
DRY_RUN="${VLA_E2E_DRY_RUN:-0}"
CAMERA_WAIT_SEC=8
STATUS_WAIT_SEC=15
NAVIGATION_STANDOFF_M="${VLA_NAVIGATION_STANDOFF_M:-0.15}"
SPRAY_RANGE_M="${VLA_SPRAY_RANGE_M:-0.30}"
LOCK_FILE="/tmp/vla_hardware_e2e.lock"
HEF_PATH="/ros2_ws/phoenix_vla/Hailo/models/baseline_yolo26_neural_norm.hef"
ONNX_PATH="/ros2_ws/phoenix_vla/Hailo/models/best_sim_postprocess.onnx"
JSON_PATH="/ros2_ws/phoenix_vla/Hailo/models/config_onnx_best_sim.json"

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

PRODUCTION_PATTERN='depth_camera.launch.py|uncc_frontier.launch.py|preprocess_node|yolo.launch.py|yolo_node|detection_3d.launch.py|detection_3d_node|ui_stream_node|topic_bridge_vla.launch.py|firefighter_ui|vla_navigation_bridge.launch.py|fire_extinguisher.launch.py|fire_suppression_node|vla_spray_bridge'
EXPECTED_LAUNCH_PATTERNS=(
    'depth_camera.launch.py'
    'uncc_frontier.launch.py'
    'ros2 run image_pipeline preprocess_node'
    'yolo.launch.py'
    'detection_3d.launch.py'
    'ros2 run image_pipeline ui_stream_node'
    'topic_bridge_vla.launch.py'
    'ros2 run fire_vla_core firefighter_ui'
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
export ROS_LOG_DIR='$RUN_LOG_DIR/ros'
mkdir -p \"\$ROS_LOG_DIR\"
nohup setsid $command >'$RUN_LOG_DIR/e2e_${name}.log' 2>&1 </dev/null &
pid=\$!
echo \$pid >'$RUN_LOG_DIR/${name}.pid'
echo \$pid >'$RUN_LOG_DIR/${name}.pgid'
awk '{print \$22}' /proc/\$pid/stat >'$RUN_LOG_DIR/${name}.starttime'
sha256sum /proc/\$pid/cmdline | awk '{print \$1}' >'$RUN_LOG_DIR/${name}.command_sha256'"
    run docker exec -d -u root -w / "$CONTAINER" bash -lc "$full"
    echo "$name: 시작 요청 (root, cwd=/)"
}

prepare_run_log_directory() {
    local boot_id timestamp
    boot_id="$(cat /proc/sys/kernel/random/boot_id 2>/dev/null || echo unknown_boot)"
    timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
    RUN_LOG_DIR="$LOG_ROOT/${boot_id}_${timestamp}_$$"
    run docker exec -u root -w / "$CONTAINER" bash -lc \
        "mkdir -p '$RUN_LOG_DIR' '$LOG_ROOT'; printf '%s\n' '$RUN_LOG_DIR' >'$CURRENT_RUN_FILE'"
}

load_current_run_directory() {
    if [[ "$DRY_RUN" == "1" ]]; then
        run docker exec "$CONTAINER" cat "$CURRENT_RUN_FILE"
        return 0
    fi
    RUN_LOG_DIR="$(docker exec "$CONTAINER" cat "$CURRENT_RUN_FILE" 2>/dev/null || true)"
    if [[ -n "$RUN_LOG_DIR" ]]; then
        echo "current_run: $RUN_LOG_DIR"
    fi
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

    prepare_run_log_directory
    launch_component camera "ros2 launch peripherals depth_camera.launch.py"
    run sleep "$CAMERA_WAIT_SEC"
    launch_component base "ros2 launch uncc_example uncc_frontier.launch.py start_frontier:=false start_mission:=false start_vision:=false"
    launch_component preprocess "ros2 run image_pipeline preprocess_node --ros-args -r __node:=rgb_preprocess_node -p input_topic:=/ascamera/camera_publisher/rgb0/image -p camera_info_topic:=/ascamera/camera_publisher/rgb0/camera_info -p output_topic:=/image_enhanced -p output_camera_info_topic:=/image_enhanced/camera_info -p mode:=passthrough"
    launch_component yolo "ros2 launch image_pipeline yolo.launch.py model_path:=$HEF_PATH postprocess_path:=$ONNX_PATH backend:=hailo layout:=end2end class_names:='[fire,person]'"
    launch_component detection3d "ros2 launch image_pipeline detection_3d.launch.py"
    launch_component ui_stream "ros2 run image_pipeline ui_stream_node --ros-args -p class_names:='[fire,person]'"
    launch_component vla "ros2 launch fire_vla_bringup topic_bridge_vla.launch.py start_perception_bridge:=true llm_backend:=remote_qwen remote_qwen_endpoint:=$endpoint remote_qwen_timeout_sec:=10.0 navigation_standoff_m:=$NAVIGATION_STANDOFF_M spray_range_m:=$SPRAY_RANGE_M"
    launch_component ui "ros2 run fire_vla_core firefighter_ui"
    launch_component navigation "ros2 launch uncc_example vla_navigation_bridge.launch.py"
    launch_component suppression "ros2 launch uncc_example fire_extinguisher.launch.py"
    echo "VLA_PARAMETERS: navigation_standoff_m=$NAVIGATION_STANDOFF_M spray_range_m=$SPRAY_RANGE_M"
    echo "production runtime 시작 요청 완료. 로그: $RUN_LOG_DIR/e2e_*.log"
}

status_runtime() {
    local endpoint="${VLA_QWEN_ENDPOINT:-}"
    local observer output
    load_current_run_directory
    read -r -d '' observer <<'PY' || true
import json
from datetime import datetime, timezone
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from lifecycle_msgs.srv import GetState
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
        self.bt_state = "UNKNOWN"
        self.vla_parameters = {
            "navigation_standoff_m": "UNKNOWN",
            "spray_range_m": "UNKNOWN",
        }
        self.vla_parameter_client = AsyncParameterClient(self, "/vla_orchestrator")
        if self.vla_parameter_client.wait_for_service(timeout_sec=0.2):
            future = self.vla_parameter_client.get_parameters(
                tuple(self.vla_parameters)
            )
            future.add_done_callback(self.capture_vla_parameters)
        self.bt_state_client = self.create_client(GetState, "/bt_navigator/get_state")
        if self.bt_state_client.wait_for_service(timeout_sec=0.2):
            future = self.bt_state_client.call_async(GetState.Request())
            future.add_done_callback(self.capture_bt_state)
        self.create_timer(15.0, self.finish)

    def capture_bt_state(self, future):
        try:
            self.bt_state = future.result().current_state.label
        except Exception:
            self.bt_state = "UNKNOWN"

    def capture_vla_parameters(self, future):
        try:
            for name, value in zip(self.vla_parameters, future.result().values):
                self.vla_parameters[name] = value.double_value
        except Exception:
            pass

    def finish(self):
        for name, value in self.seen.items():
            print(f"{name}: {'PASS' if value else 'FAIL'}")
        tf_ok = self.tf_buffer.can_transform(
            "map", "base_footprint", Time(), timeout=Duration(seconds=0.2)
        )
        print(f"MAP_TO_BASE_FOOTPRINT: {'PASS' if tf_ok else 'FAIL'}")
        nav_server_count = len(self.get_publishers_info_by_topic(
            "/navigate_to_pose/_action/status"
        ))
        nav_ready = nav_server_count == 1 and self.bt_state.lower() == "active"
        print(f"NAV2_ACTION_SERVER: {'PASS' if nav_ready else 'FAIL'}")
        print(
            "NAV2_READINESS: "
            f"timestamp={datetime.now(timezone.utc).isoformat()} "
            f"server_count={nav_server_count} "
            f"bt_navigator_state={self.bt_state} "
            f"result={'PASS' if nav_ready else 'FAIL'}"
        )
        print(
            f"SUPPRESSION_ACTION_SERVER: "
            f"{'PASS' if self.suppression.server_is_ready() else 'FAIL'}"
        )
        print(
            "VLA_PARAMETERS: "
            f"navigation_standoff_m={self.vla_parameters['navigation_standoff_m']} "
            f"spray_range_m={self.vla_parameters['spray_range_m']}"
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
        printf '%s\n' "$output" | grep '^NAV2_READINESS:' | \
            docker exec -i -u root "$CONTAINER" \
            sh -c "cat >'$RUN_LOG_DIR/nav2_readiness.log'"
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
    load_current_run_directory
    if [[ "$DRY_RUN" == "1" ]]; then
        run docker exec -u root -w / "$CONTAINER" bash -lc "$ENVIRONMENT
timeout -k 1s 3s ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal '$cancel_request' || true
timeout -k 1s 3s ros2 service call /suppress_fire/_action/cancel_goal action_msgs/srv/CancelGoal '$cancel_request' || true"
        run docker exec -u root -w / "$CONTAINER" bash -lc "$ENVIRONMENT
timeout -k 1s 3s ros2 topic pub --once -w 1 /ros_robot_controller/set_motor ros_robot_controller_msgs/msg/MotorsState \"{data: [{id: 1, rps: 0.0}, {id: 2, rps: 0.0}, {id: 3, rps: 0.0}, {id: 4, rps: 0.0}]}\" || true"
        return 0
    fi
    exec 9>"$LOCK_FILE"
    flock -w 2 9 || true
    curl -fsS --max-time 2 -X POST -H 'Content-Type: application/json' \
        -d '{"mode":"NONE"}' http://127.0.0.1:8080/api/control-mode \
        >/dev/null 2>&1 || true
    container_shell "timeout -k 1s 3s ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal '$cancel_request' >/dev/null 2>&1 || true
timeout -k 1s 3s ros2 service call /suppress_fire/_action/cancel_goal action_msgs/srv/CancelGoal '$cancel_request' >/dev/null 2>&1 || true"
    sleep 1
    container_shell "timeout -k 1s 3s ros2 topic pub --once -w 1 /ros_robot_controller/set_motor ros_robot_controller_msgs/msg/MotorsState \"{data: [{id: 1, rps: 0.0}, {id: 2, rps: 0.0}, {id: 3, rps: 0.0}, {id: 4, rps: 0.0}]}\" >/dev/null 2>&1 || true"
    echo "Robot stop: active goal cancel + explicit four-motor zero 전송"
    if [[ -n "$RUN_LOG_DIR" ]]; then
        docker exec -e RUN_LOG_DIR="$RUN_LOG_DIR" "$CONTAINER" bash -lc '
: >"$RUN_LOG_DIR/owned_processes.snapshot"
for file in "$RUN_LOG_DIR"/*.pgid; do
    [[ -f "$file" ]] || continue
    pgid="$(cat "$file")"
    [[ "$pgid" =~ ^[0-9]+$ && "$pgid" -gt 1 ]] || continue
    group_owned=0
    while read -r pid member_pgid stat; do
        [[ -n "$pid" && "$stat" != Z* ]] || continue
        starttime="$(awk '\''{print $22}'\'' "/proc/$pid/stat" 2>/dev/null || true)"
        command_sha256="$(sha256sum "/proc/$pid/cmdline" 2>/dev/null | awk '\''{print $1}'\'')"
        [[ -n "$starttime" && -n "$command_sha256" ]] || continue
        printf "%s %s %s %s\n" "$pid" "$member_pgid" "$starttime" "$command_sha256" \
            >>"$RUN_LOG_DIR/owned_processes.snapshot"
        group_owned=1
    done < <(ps -eo pid=,pgid=,stat= | awk -v wanted="$pgid" '\''$2 == wanted'\'')
    [[ "$group_owned" -eq 1 ]] || continue
    kill -INT -- "-$pgid" 2>/dev/null || true
done'
        sleep 5
        docker exec -e RUN_LOG_DIR="$RUN_LOG_DIR" "$CONTAINER" bash -lc '
cut -d" " -f2 "$RUN_LOG_DIR/owned_processes.snapshot" | sort -un | while read -r pgid; do
    [[ "$pgid" =~ ^[0-9]+$ && "$pgid" -gt 1 ]] || continue
    matching=0
    while read -r pid saved_pgid saved_starttime saved_hash; do
        [[ "$saved_pgid" == "$pgid" && -r "/proc/$pid/stat" ]] || continue
        stat="$(ps -o stat= -p "$pid" 2>/dev/null || true)"
        starttime="$(awk '\''{print $22}'\'' "/proc/$pid/stat" 2>/dev/null || true)"
        command_sha256="$(sha256sum "/proc/$pid/cmdline" 2>/dev/null | awk '\''{print $1}'\'')"
        [[ "$stat" != Z* && "$starttime" == "$saved_starttime" && "$command_sha256" == "$saved_hash" ]] \
            && matching=1
    done <"$RUN_LOG_DIR/owned_processes.snapshot"
    [[ "$matching" -eq 1 ]] || continue
    kill -TERM -- "-$pgid" 2>/dev/null || true
done'
        sleep 2
        docker exec -e RUN_LOG_DIR="$RUN_LOG_DIR" "$CONTAINER" bash -lc '
rm -f "$RUN_LOG_DIR/final_kill.pids"
while read -r pid pgid saved_starttime saved_hash; do
    [[ -r "/proc/$pid/stat" ]] || continue
    stat="$(ps -o stat= -p "$pid" 2>/dev/null || true)"
    current_pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d " ")"
    starttime="$(awk '\''{print $22}'\'' "/proc/$pid/stat" 2>/dev/null || true)"
    command_sha256="$(sha256sum "/proc/$pid/cmdline" 2>/dev/null | awk '\''{print $1}'\'')"
    [[ "$stat" != Z* && "$current_pgid" == "$pgid" \
        && "$starttime" == "$saved_starttime" && "$command_sha256" == "$saved_hash" ]] || continue
    printf "%s\n" "$pid" >>"$RUN_LOG_DIR/final_kill.pids"
    kill -KILL "$pid" 2>/dev/null || true
done <"$RUN_LOG_DIR/owned_processes.snapshot"'
        sleep 1
        docker exec -e RUN_LOG_DIR="$RUN_LOG_DIR" "$CONTAINER" bash -lc '
remaining=0
while read -r pid pgid saved_starttime saved_hash; do
    [[ -r "/proc/$pid/stat" ]] || continue
    stat="$(ps -o stat= -p "$pid" 2>/dev/null || true)"
    starttime="$(awk '\''{print $22}'\'' "/proc/$pid/stat" 2>/dev/null || true)"
    command_sha256="$(sha256sum "/proc/$pid/cmdline" 2>/dev/null | awk '\''{print $1}'\'')"
    [[ "$stat" == Z* || "$starttime" != "$saved_starttime" || "$command_sha256" != "$saved_hash" ]] \
        || remaining=1
done <"$RUN_LOG_DIR/owned_processes.snapshot"
for file in "$RUN_LOG_DIR"/*.pgid; do
    [[ -f "$file" ]] || continue
    pgid="$(cat "$file")"
    [[ "$pgid" =~ ^[0-9]+$ && "$pgid" -gt 1 ]] || continue
    members="$(ps -eo pgid=,stat= | awk -v wanted="$pgid" '\''$1 == wanted && $2 !~ /^Z/'\'')"
    [[ -z "$members" ]] || remaining=1
done
exit "$remaining"' || {
            echo "wrapper 소유 production process group 종료 미완료" >&2
            return 6
        }
    fi
    echo "Pump OFF: suppression process group 종료 적용"
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
