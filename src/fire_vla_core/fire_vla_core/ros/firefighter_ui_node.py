from __future__ import annotations

import copy
import json
import math
import threading
import time
import uuid
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib.resources import files
from typing import Any, Callable
from urllib.parse import parse_qs, urlparse

try:
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from rclpy.time import Time
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import Bool, String
    from tf2_ros import Buffer, TransformException, TransformListener
except ImportError:
    rclpy = None
    Node = object
    String = None
    Bool = None
    CompressedImage = None
    OccupancyGrid = None
    Time = None
    Buffer = None
    TransformListener = None
    TransformException = Exception
    HistoryPolicy = None
    qos_profile_sensor_data = None

from fire_vla_core.ros.occupancy_png import (
    downsample_step, grid_metadata, render_occupancy_png, yaw_from_quaternion,
)


_MAX_REQUEST_BYTES = 64 * 1024
_ALLOWED_HOSTS = {"127.0.0.1", "localhost"}
_VLA_MODE = "VLA"
_RULE_BASED_MODE = "RULE_BASED"
_NONE_MODE = "NONE"
_ALLOWED_MODES = {_VLA_MODE, _RULE_BASED_MODE}
_ALLOWED_CONTROL_MODES = {*_ALLOWED_MODES, _NONE_MODE}
_RULE_BASED_COMMANDS = {"START", "STOP"}
_STREAM_BOUNDARY = "phoenixframe"
_STREAM_WAIT_SEC = 1.0
_STREAM_IDLE_LIMIT = 15


def normalize_mode(value: str | None) -> str:
    if value is not None and not isinstance(value, str):
        raise ValueError("mode는 문자열이어야 합니다.")
    mode = (value or _VLA_MODE).strip().upper()
    if mode not in _ALLOWED_MODES:
        raise ValueError("mode는 VLA 또는 RULE_BASED여야 합니다.")
    return mode


def normalize_control_mode(value: str | None) -> str:
    if not isinstance(value, str):
        raise ValueError("mode는 문자열이어야 합니다.")
    mode = value.strip().upper()
    if mode not in _ALLOWED_CONTROL_MODES:
        raise ValueError("mode는 NONE, VLA 또는 RULE_BASED여야 합니다.")
    return mode


class StatusStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._statuses: dict[str, dict[str, Any]] = {
            _VLA_MODE: {
                "timestamp": None,
                "world_model": {},
                "decision": None,
                "validation": None,
                "submission": None,
                "blocked_reason": "VLA status를 기다리는 중입니다.",
            },
            _RULE_BASED_MODE: {
                "schema_version": 1,
                "mode": _RULE_BASED_MODE,
                "timestamp": None,
                "blocked_reason": "Rule-based status를 기다리는 중입니다.",
            },
        }

    def update(self, status: dict[str, Any], mode: str = _VLA_MODE) -> None:
        mode = normalize_mode(mode)
        with self._lock:
            self._statuses[mode] = copy.deepcopy(status)

    def get(self, mode: str = _VLA_MODE) -> dict[str, Any]:
        mode = normalize_mode(mode)
        with self._lock:
            return copy.deepcopy(self._statuses[mode])


class ControlModeOwner:
    """Server-owned command mode; action lifecycle remains in existing runtimes."""

    def __init__(self, publish: Callable[[str], None]) -> None:
        self._lock = threading.Lock()
        self._mode = _NONE_MODE
        self._active = False
        self._publish = publish

    def get(self) -> dict[str, Any]:
        with self._lock:
            return {"active_control_mode": self._mode, "active_action": self._active}

    def select(self, mode: str) -> dict[str, Any]:
        mode = normalize_control_mode(mode)
        with self._lock:
            if self._active and mode != self._mode:
                raise ValueError("MODE_CHANGE_BLOCKED_ACTIVE_ACTION")
            self._mode = mode
            self._publish(mode)
            return {"active_control_mode": self._mode, "active_action": self._active}

    def begin(self, mode: str, *, stop: bool = False) -> None:
        mode = normalize_mode(mode)
        with self._lock:
            if mode != self._mode:
                raise ValueError("CONTROL_MODE_MISMATCH")
            self._active = not stop

    def observe_terminal(self, mode: str, status: dict[str, Any]) -> None:
        with self._lock:
            if mode != self._mode:
                return
            if mode == _VLA_MODE:
                world = status.get("world_model") or {}
                mission = world.get("mission") or {}
                terminal = mission.get("status") in {
                    "COMPLETED", "FAILED", "ABORTED", "CANCELED"
                }
                if terminal and world.get("current_action") is None:
                    self._active = False
            else:
                mission = status.get("mission") or {}
                if mission.get("last_command", {}).get("command") == "STOP":
                    self._active = False
                elif mission.get("state") in {
                    "COMPLETED", "FAILED", "ABORTED", "CANCELED", "IDLE"
                }:
                    self._active = False


def validate_server_config(host: str, port: int) -> tuple[str, int]:
    normalized_host = host.strip().lower()
    if normalized_host not in _ALLOWED_HOSTS:
        raise ValueError("ui_host는 localhost 또는 127.0.0.1이어야 합니다.")
    if not 0 <= port <= 65535:
        raise ValueError("ui_port는 0 이상 65535 이하여야 합니다.")
    return normalized_host, port


def create_mission_payload(text: str) -> dict[str, str]:
    if not isinstance(text, str):
        raise ValueError("Mission text는 문자열이어야 합니다.")
    normalized = text.strip()
    if not normalized:
        raise ValueError("Mission text는 비어 있을 수 없습니다.")
    if len(normalized) > 2000:
        raise ValueError("Mission text는 2000자 이하여야 합니다.")
    return {
        "mission_id": f"mission_ui_{uuid.uuid4().hex[:12]}",
        "text": normalized,
    }


def normalize_rule_based_command(text: str) -> str:
    if not isinstance(text, str):
        raise ValueError("Rule-based command는 문자열이어야 합니다.")
    command = text.strip().upper()
    if command not in _RULE_BASED_COMMANDS:
        raise ValueError("Rule-based mode는 START 또는 STOP만 지원합니다.")
    return command


class FrameStore:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._frame: bytes | None = None
        self._sequence = 0

    def update(self, jpeg: bytes) -> None:
        with self._condition:
            self._frame = bytes(jpeg)
            self._sequence += 1
            self._condition.notify_all()

    def wait_for(self, last_sequence: int, timeout: float):
        with self._condition:
            if self._sequence != last_sequence:
                return self._frame, self._sequence
            self._condition.wait(timeout)
            if self._sequence == last_sequence:
                return None, last_sequence
            return self._frame, self._sequence


class OverlayStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._overlay: dict[str, Any] | None = None

    def update(self, overlay: dict[str, Any]) -> None:
        with self._lock:
            self._overlay = copy.deepcopy(overlay)

    def get(self) -> dict[str, Any]:
        with self._lock:
            if self._overlay is None:
                return {"available": False, "boxes": []}
            return {"available": True, **copy.deepcopy(self._overlay)}

    def clear(self) -> None:
        with self._lock:
            self._overlay = None


class MapStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._png: bytes | None = None
        self._metadata: dict[str, Any] | None = None
        self._robot: dict[str, float] | None = None
        self._version = 0

    def update_map(self, png: bytes, metadata: dict[str, Any]) -> int:
        with self._lock:
            self._png = bytes(png)
            self._metadata = copy.deepcopy(metadata)
            self._version += 1
            return self._version

    def update_robot(self, pose: dict[str, float] | None) -> None:
        with self._lock:
            self._robot = None if pose is None else dict(pose)

    def png(self) -> tuple[bytes | None, int]:
        with self._lock:
            return self._png, self._version

    def get(self) -> dict[str, Any]:
        with self._lock:
            robot = None if self._robot is None else dict(self._robot)
            if self._metadata is None:
                return {"available": False, "version": 0, "robot": robot}
            return {
                "available": True,
                "version": self._version,
                "robot": robot,
                **copy.deepcopy(self._metadata),
            }


class FirefighterHTTPServer:
    def __init__(
        self,
        host: str,
        port: int,
        status_store: StatusStore,
        submit_mission: Callable[[str], dict[str, str]],
        *,
        control_owner: ControlModeOwner | None = None,
        index_html: bytes | None = None,
        frame_store: FrameStore | None = None,
        overlay_store: OverlayStore | None = None,
        map_store: MapStore | None = None,
        set_vision_enabled: Callable[[bool], dict[str, Any]] | None = None,
    ) -> None:
        host, port = validate_server_config(host, port)
        self._status_store = status_store
        self._submit_mission = submit_mission
        self._control_owner = control_owner or ControlModeOwner(lambda _mode: None)
        self._frames = frame_store or FrameStore()
        self._overlays = overlay_store or OverlayStore()
        self._maps = map_store or MapStore()
        self._set_vision_enabled = set_vision_enabled
        self._index_html = index_html or (
            files("fire_vla_core.web").joinpath("index.html").read_bytes()
        )
        self._recorded_video = files("fire_vla_core.web").joinpath(
            "fire_person_detection_result.mp4"
        ).read_bytes()
        handler = self._handler_type()
        self._server = ThreadingHTTPServer((host, port), handler)
        self._thread: threading.Thread | None = None

    @property
    def address(self) -> tuple[str, int]:
        host, port = self._server.server_address[:2]
        return str(host), int(port)

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            name="firefighter-ui-http",
            daemon=True,
        )
        self._thread.start()

    def close(self) -> None:
        if self._thread is not None:
            self._server.shutdown()
            self._thread.join(timeout=2.0)
            self._thread = None
        self._server.server_close()

    def _handler_type(self):
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                parsed = urlparse(self.path)
                if parsed.path == "/":
                    self._send_bytes(
                        HTTPStatus.OK,
                        owner._index_html,
                        "text/html; charset=utf-8",
                    )
                    return
                if parsed.path == "/media/fire_person_detection_result.mp4":
                    self._send_bytes(
                        HTTPStatus.OK, owner._recorded_video, "video/mp4"
                    )
                    return
                if parsed.path == "/api/status":
                    try:
                        query = parse_qs(parsed.query)
                        mode = normalize_mode(
                            (query.get("mode") or [_VLA_MODE])[0]
                        )
                    except ValueError as exc:
                        self._send_json(
                            HTTPStatus.BAD_REQUEST, {"error": str(exc)}
                        )
                        return
                    self._send_json(
                        HTTPStatus.OK, owner._status_store.get(mode)
                    )
                    return
                if parsed.path == "/api/control-mode":
                    self._send_json(HTTPStatus.OK, owner._control_owner.get())
                    return
                if parsed.path == "/api/vision/stream":
                    self._stream_frames()
                    return
                if parsed.path == "/api/vision/detections":
                    self._send_json(HTTPStatus.OK, owner._overlays.get())
                    return
                if parsed.path == "/api/map":
                    self._send_json(HTTPStatus.OK, owner._maps.get())
                    return
                if parsed.path == "/api/map.png":
                    self._send_map_png()
                    return
                self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

            def do_POST(self) -> None:
                if self.path == "/api/vision/enabled":
                    self._handle_vision_enabled()
                    return
                if self.path == "/api/control-mode":
                    try:
                        length = int(self.headers.get("Content-Length", "0"))
                        if length <= 0 or length > _MAX_REQUEST_BYTES:
                            raise ValueError("invalid content length")
                        data = json.loads(self.rfile.read(length).decode("utf-8"))
                        if not isinstance(data, dict):
                            raise ValueError("JSON object가 필요합니다.")
                        selected = owner._control_owner.select(data.get("mode"))
                    except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError) as exc:
                        self._send_json(HTTPStatus.CONFLICT, {"error": str(exc)})
                        return
                    self._send_json(HTTPStatus.OK, selected)
                    return
                if self.path != "/api/mission":
                    self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    if length <= 0 or length > _MAX_REQUEST_BYTES:
                        raise ValueError("invalid content length")
                    data = json.loads(self.rfile.read(length).decode("utf-8"))
                    if not isinstance(data, dict):
                        raise ValueError("JSON object가 필요합니다.")
                    text = data.get("text", "")
                    if not isinstance(text, str):
                        raise ValueError("Mission text는 문자열이어야 합니다.")
                    mode = normalize_mode(data.get("mode"))
                    if mode == _RULE_BASED_MODE:
                        text = normalize_rule_based_command(text)
                    owner._control_owner.begin(
                        mode, stop=(mode == _RULE_BASED_MODE and text == "STOP")
                    )
                    mission = (
                        owner._submit_mission(text)
                        if mode == _VLA_MODE
                        else owner._submit_mission(text, mode)
                    )
                except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError) as exc:
                    self._send_json(
                        HTTPStatus.BAD_REQUEST,
                        {"error": str(exc) or "invalid request"},
                    )
                    return
                self._send_json(HTTPStatus.ACCEPTED, mission)

            def _handle_vision_enabled(self) -> None:
                if owner._set_vision_enabled is None:
                    self._send_json(
                        HTTPStatus.SERVICE_UNAVAILABLE,
                        {"error": "vision toggle is unavailable"},
                    )
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    if length <= 0 or length > _MAX_REQUEST_BYTES:
                        raise ValueError("invalid content length")
                    data = json.loads(self.rfile.read(length).decode("utf-8"))
                    enabled = data.get("enabled") if isinstance(data, dict) else None
                    if not isinstance(enabled, bool):
                        raise ValueError("enabled must be boolean")
                except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError) as exc:
                    self._send_json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
                    return
                if not enabled:
                    owner._overlays.clear()
                self._send_json(
                    HTTPStatus.ACCEPTED, owner._set_vision_enabled(enabled)
                )

            def log_message(self, format: str, *args) -> None:
                return

            def _send_json(self, status: HTTPStatus, payload: dict) -> None:
                self._send_bytes(
                    status,
                    json.dumps(payload, ensure_ascii=False).encode("utf-8"),
                    "application/json; charset=utf-8",
                )

            def _send_bytes(
                self, status: HTTPStatus, body: bytes, content_type: str
            ) -> None:
                self.send_response(status.value)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def _stream_frames(self) -> None:
                self.send_response(HTTPStatus.OK.value)
                self.send_header(
                    "Content-Type",
                    f"multipart/x-mixed-replace; boundary={_STREAM_BOUNDARY}",
                )
                self.send_header("Cache-Control", "no-store")
                self.send_header("Connection", "close")
                self.end_headers()
                sequence, idle = 0, 0
                try:
                    while idle < _STREAM_IDLE_LIMIT:
                        frame, sequence = owner._frames.wait_for(
                            sequence, _STREAM_WAIT_SEC
                        )
                        if frame is None:
                            idle += 1
                            continue
                        idle = 0
                        self.wfile.write(
                            f"--{_STREAM_BOUNDARY}\r\n"
                            f"Content-Type: image/jpeg\r\n"
                            f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii")
                        )
                        self.wfile.write(frame + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def _send_map_png(self) -> None:
                png, version = owner._maps.png()
                if png is None:
                    self._send_json(
                        HTTPStatus.SERVICE_UNAVAILABLE,
                        {"error": "map is unavailable"},
                    )
                    return
                self.send_response(HTTPStatus.OK.value)
                self.send_header("Content-Type", "image/png")
                self.send_header("Content-Length", str(len(png)))
                self.send_header("Cache-Control", "no-cache")
                self.send_header("ETag", f'"{version}"')
                self.end_headers()
                self.wfile.write(png)

        return Handler


class FirefighterUINode(Node):
    def __init__(self) -> None:
        super().__init__("firefighter_ui")
        self.declare_parameter("ui_host", "127.0.0.1")
        self.declare_parameter("ui_port", 8080)
        self.declare_parameter("status_topic", "/vla/status")
        self.declare_parameter("mission_topic", "/vla/mission")
        self.declare_parameter(
            "rule_based_status_topic", "/rule_based/status"
        )
        self.declare_parameter(
            "rule_based_mission_topic", "/rule_based/mission"
        )
        self.declare_parameter("vision_topic", "/ui/camera/compressed")
        self.declare_parameter("vision_overlay_topic", "/ui/camera/overlay")
        self.declare_parameter("vision_enabled_topic", "/ui/camera/enabled")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("map_render_period_sec", 1.0)
        self.declare_parameter("map_max_pixels", 250_000)
        self.declare_parameter("robot_pose_period_sec", 0.2)

        self._store = StatusStore()
        self._frames = FrameStore()
        self._overlays = OverlayStore()
        self._maps = MapStore()
        self._last_map_render = 0.0
        self._map_render_period = float(
            self.get_parameter("map_render_period_sec").value
        )
        self._map_max_pixels = int(self.get_parameter("map_max_pixels").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._mission_pub = self.create_publisher(
            String, str(self.get_parameter("mission_topic").value), 10
        )
        self._rule_based_mission_pub = self.create_publisher(
            String,
            str(self.get_parameter("rule_based_mission_topic").value),
            10,
        )
        control_qos = QoSProfile(depth=1)
        control_qos.reliability = ReliabilityPolicy.RELIABLE
        control_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._control_mode_pub = self.create_publisher(
            String, "/vla/control_mode", control_qos
        )
        self._control_owner = ControlModeOwner(self._publish_control_mode)
        self._publish_control_mode(_NONE_MODE)
        self.create_subscription(
            String,
            str(self.get_parameter("status_topic").value),
            self._status_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("rule_based_status_topic").value),
            self._rule_based_status_callback,
            10,
        )
        self.create_subscription(
            CompressedImage,
            str(self.get_parameter("vision_topic").value),
            self._frame_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("vision_overlay_topic").value),
            self._overlay_callback,
            qos_profile_sensor_data,
        )
        self._vision_enabled_pub = self.create_publisher(
            Bool, str(self.get_parameter("vision_enabled_topic").value), 10
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._map_callback,
            QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_timer(
            max(0.05, float(self.get_parameter("robot_pose_period_sec").value)),
            self._poll_robot_pose,
        )
        self._http = FirefighterHTTPServer(
            str(self.get_parameter("ui_host").value),
            int(self.get_parameter("ui_port").value),
            self._store,
            self._publish_mission,
            control_owner=self._control_owner,
            frame_store=self._frames,
            overlay_store=self._overlays,
            map_store=self._maps,
            set_vision_enabled=self._set_vision_enabled,
        )
        self._http.start()
        host, port = self._http.address
        self.get_logger().info(f"Firefighter UI started: http://{host}:{port}")

    def _status_callback(self, msg: String) -> None:
        self._update_status(msg, _VLA_MODE)

    def _rule_based_status_callback(self, msg: String) -> None:
        self._update_status(msg, _RULE_BASED_MODE)

    def _frame_callback(self, msg: CompressedImage) -> None:
        if msg.data:
            self._frames.update(bytes(msg.data))

    def _overlay_callback(self, msg: String) -> None:
        try:
            overlay = json.loads(msg.data)
            if not isinstance(overlay, dict) or not isinstance(overlay.get("boxes"), list):
                raise ValueError("overlay must contain boxes")
            self._overlays.update(overlay)
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"vision overlay parsing failed: {exc}")

    def _map_callback(self, msg: OccupancyGrid) -> None:
        now = time.monotonic()
        if now - self._last_map_render < self._map_render_period:
            return
        self._last_map_render = now
        width, height = int(msg.info.width), int(msg.info.height)
        step = downsample_step(width, height, self._map_max_pixels)
        metadata = grid_metadata(msg)
        metadata["render_step"] = step
        metadata["png_width"] = math.ceil(width / step)
        metadata["png_height"] = math.ceil(height / step)
        self._maps.update_map(
            render_occupancy_png(msg.data, width, height, step), metadata
        )

    def _poll_robot_pose(self) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, Time()
            )
        except TransformException:
            self._maps.update_robot(None)
            return
        translation = transform.transform.translation
        self._maps.update_robot({
            "x": float(translation.x),
            "y": float(translation.y),
            "yaw": yaw_from_quaternion(transform.transform.rotation),
        })

    def _update_status(self, msg: String, mode: str) -> None:
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError("status payload는 JSON object여야 합니다.")
            self._store.update(data, mode)
            self._control_owner.observe_terminal(mode, data)
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                f"{mode} status parsing failed: {exc}"
            )

    def _publish_mission(
        self, text: str, mode: str = _VLA_MODE
    ) -> dict[str, str]:
        mode = normalize_mode(mode)
        payload = create_mission_payload(text)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        publisher = (
            self._mission_pub
            if mode == _VLA_MODE
            else self._rule_based_mission_pub
        )
        publisher.publish(msg)
        return payload if mode == _VLA_MODE else {**payload, "mode": mode}

    def _publish_control_mode(self, mode: str) -> None:
        msg = String()
        msg.data = mode
        self._control_mode_pub.publish(msg)

    def _set_vision_enabled(self, enabled: bool) -> dict[str, Any]:
        msg = Bool()
        msg.data = enabled
        self._vision_enabled_pub.publish(msg)
        return {"enabled": enabled}

    def destroy_node(self):
        self._http.close()
        return super().destroy_node()


def main(args=None) -> None:
    if rclpy is None:
        raise RuntimeError("ROS2 환경에서 실행해야 합니다.")
    rclpy.init(args=args)
    node = FirefighterUINode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
