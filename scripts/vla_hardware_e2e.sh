#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${VLA_CONTAINER:-IntelPi}"
WORKSPACE="/ros2_ws/phoenix_vla"
LOG_DIR="${VLA_E2E_LOG_DIR:-/tmp}"
DRY_RUN="${VLA_E2E_DRY_RUN:-0}"
CAMERA_WAIT_SEC=8
LOCK_FILE="/tmp/vla_hardware_e2e.lock"

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

PRODUCTION_PATTERN='depth_camera.launch.py|uncc_frontier.launch.py|preprocess_node|yolo.launch.py|detection_3d.launch.py|topic_bridge_vla.launch.py|vla_navigation_bridge.launch.py|fire_extinguisher.launch.py'

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
    run docker exec "$CONTAINER" bash -lc "$ENVIRONMENT
$command"
}

production_processes() {
    docker exec "$CONTAINER" bash -lc \
        "ps -eo args= | grep -E '$PRODUCTION_PATTERN' | grep -v -E 'grep -E|vla_hardware_e2e.sh' || true"
}

launch_component() {
    local name="$1"
    local user="$2"
    local command="$3"
    local full="$ENVIRONMENT
nohup setsid $command >'$LOG_DIR/e2e_${name}.log' 2>&1 </dev/null &"
    run docker exec -d -u "$user" -w / "$CONTAINER" bash -lc "$full"
    echo "$name: 시작 요청"
}

start_runtime() {
    local endpoint="${VLA_QWEN_ENDPOINT:-}"
    if [[ -z "$endpoint" ]]; then
        echo "VLA_QWEN_ENDPOINT가 필요함" >&2
        return 2
    fi
    if [[ "$DRY_RUN" != "1" ]] && [[ -n "$(production_processes)" ]]; then
        echo "production runtime이 이미 실행 중임. 재사용하고 status로 확인"
        return 0
    fi

    launch_component camera ubuntu "ros2 launch peripherals depth_camera.launch.py"
    run sleep "$CAMERA_WAIT_SEC"
    launch_component base ubuntu "ros2 launch uncc_example uncc_frontier.launch.py start_frontier:=false start_mission:=false start_vision:=false"
    launch_component preprocess ubuntu "ros2 run image_pipeline preprocess_node --ros-args -r __node:=rgb_preprocess_node -p input_topic:=/ascamera/camera_publisher/rgb0/image -p camera_info_topic:=/ascamera/camera_publisher/rgb0/camera_info -p output_topic:=/image_enhanced -p output_camera_info_topic:=/image_enhanced/camera_info -p mode:=passthrough"
    launch_component yolo ubuntu "ros2 launch image_pipeline yolo.launch.py model_path:=$WORKSPACE/Hailo/models/baseline_yolo26_neural_norm.hef postprocess_path:=$WORKSPACE/Hailo/models/best_sim_postprocess.onnx backend:=hailo layout:=end2end class_names:='[fire,person]'"
    launch_component detection3d ubuntu "ros2 launch image_pipeline detection_3d.launch.py"
    launch_component vla ubuntu "ros2 launch fire_vla_bringup topic_bridge_vla.launch.py start_perception_bridge:=true llm_backend:=remote_qwen remote_qwen_endpoint:=$endpoint remote_qwen_timeout_sec:=10.0"
    launch_component navigation ubuntu "ros2 launch uncc_example vla_navigation_bridge.launch.py"
    launch_component suppression root "ros2 launch uncc_example fire_extinguisher.launch.py"
    echo "production runtime 시작 요청 완료. 로그: $LOG_DIR/e2e_*.log"
}

status_runtime() {
    if [[ "$DRY_RUN" == "1" ]]; then
        run docker exec "$CONTAINER" bash -lc "ps -eo args= | grep -E '$PRODUCTION_PATTERN'"
        run docker exec "$CONTAINER" bash -lc "$ENVIRONMENT
ros2 action info /navigate_to_pose; ros2 action info /suppress_fire"
        return 0
    fi
    local processes
    processes="$(production_processes)"
    if [[ -z "$processes" ]]; then
        echo "production_processes: 0"
    else
        echo "$processes" | sed 's/^/RUNNING: /'
    fi
    container_shell "timeout 3 ros2 action info /navigate_to_pose 2>/dev/null | head -3 || echo 'Nav2: UNKNOWN'
timeout 3 ros2 action info /suppress_fire 2>/dev/null | head -3 || echo 'Suppression: UNKNOWN'"
}

mission_once() {
    local text="${VLA_MISSION_TEXT:-화재를 찾아 진압해줘}"
    local mission_id="mission_fire_$(date -u +%Y%m%dT%H%M%S)_$$"
    local payload
    payload="$(printf '{\"mission_id\":\"%s\",\"text\":\"%s\"}' "$mission_id" "$text")"
    if [[ "$DRY_RUN" == "1" ]]; then
        run docker exec -e "VLA_MISSION_PAYLOAD=$payload" "$CONTAINER" bash -lc "$ENVIRONMENT
ros2 topic pub --once -w 1 /vla/mission std_msgs/msg/String \"{data: '\$VLA_MISSION_PAYLOAD'}\""
        echo "mission_id: $mission_id (dry-run)"
        return 0
    fi
    exec 9>"$LOCK_FILE"
    if ! flock -n 9; then
        echo "다른 mission 명령이 실행 중임. 중복 발행하지 않음" >&2
        return 3
    fi
    docker exec -e "VLA_MISSION_PAYLOAD=$payload" "$CONTAINER" bash -lc "$ENVIRONMENT
ros2 topic pub --once -w 1 /vla/mission std_msgs/msg/String \"{data: '\$VLA_MISSION_PAYLOAD'}\""
    echo "mission_id: $mission_id (1회 발행)"
}

stop_runtime() {
    local cancel_request="{goal_info: {goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, stamp: {sec: 0, nanosec: 0}}}"
    if [[ "$DRY_RUN" == "1" ]]; then
        run docker exec "$CONTAINER" bash -lc "$ENVIRONMENT
timeout 3 ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal '$cancel_request' || true
timeout 3 ros2 service call /suppress_fire/_action/cancel_goal action_msgs/srv/CancelGoal '$cancel_request' || true"
        run docker exec "$CONTAINER" bash -lc "$ENVIRONMENT
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
