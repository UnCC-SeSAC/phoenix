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
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from rclpy.time import Time
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import Bool, String
    from tf2_ros import Buffer, TransformException, TransformListener
except ImportError:
    # ROS 없이도 import되어야 pytest가 돕니다. 이름을 전부 채워 둡니다.
    rclpy = None
    Node = object
    String = None
    Bool = None
    OccupancyGrid = None
    CompressedImage = None
    Time = None
    Buffer = None
    TransformListener = None
    TransformException = Exception
    qos_profile_sensor_data = None
    QoSProfile = None
    HistoryPolicy = None
    ReliabilityPolicy = None
    DurabilityPolicy = None

from fire_vla_core.ros.occupancy_png import (
    downsample_step,
    grid_metadata,
    render_occupancy_png,
    yaw_from_quaternion,
)


_MAX_REQUEST_BYTES = 64 * 1024
_ALLOWED_HOSTS = {"127.0.0.1", "localhost"}
_VLA_MODE = "VLA"
_RULE_BASED_MODE = "RULE_BASED"
_ALLOWED_MODES = {_VLA_MODE, _RULE_BASED_MODE}
_RULE_BASED_COMMANDS = {"START", "STOP"}

_STREAM_BOUNDARY = "phoenixframe"
_STREAM_WAIT_SEC = 1.0
_STREAM_IDLE_LIMIT = 15      # 프레임 없이 15초 -> 스트림 종료


def normalize_mode(value: str | None) -> str:
    if value is not None and not isinstance(value, str):
        raise ValueError("mode는 문자열이어야 합니다.")
    mode = (value or _VLA_MODE).strip().upper()
    if mode not in _ALLOWED_MODES:
        raise ValueError("mode는 VLA 또는 RULE_BASED여야 합니다.")
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


def validate_server_config(host: str, port: int, *, allow_remote: bool = False):
    normalized_host = host.strip().lower()
    if not normalized_host:
        raise ValueError("ui_host는 비어 있을 수 없습니다.")
    if not allow_remote and normalized_host not in _ALLOWED_HOSTS:
        raise ValueError(
            "ui_host는 localhost 또는 127.0.0.1이어야 합니다. "
            "LAN 관제 PC에서 열려면 ui_allow_remote를 true로 설정하세요."
        )
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
    """최신 JPEG 프레임 하나. MJPEG 스레드가 여기서 다음 프레임을 기다립니다.

    큐를 두지 않는 이유: 관제 화면에 필요한 건 **지금**입니다. 밀린 프레임을
    쌓아 보내면 지연만 누적되고, 화면은 과거를 보여주면서 최신인 척합니다.
    """

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._frame: bytes | None = None
        self._seq = 0

    def update(self, jpeg: bytes) -> None:
        with self._condition:
            self._frame = bytes(jpeg)
            self._seq += 1
            self._condition.notify_all()

    def latest(self) -> tuple[bytes | None, int]:
        with self._condition:
            return self._frame, self._seq

    def wait_for(self, last_seq: int, timeout: float):
        """`last_seq` 이후의 프레임. 타임아웃이면 `(None, last_seq)`.

        ★ `timeout` 없이 `wait()`하면 카메라가 멈춘 순간 MJPEG 스레드가
          영원히 잠듭니다. 클라이언트가 끊긴 것도 감지하지 못해 스레드가
          그대로 새고, Pi에서는 몇 번만 반복돼도 치명적입니다.
        """
        with self._condition:
            if self._seq != last_seq:
                return self._frame, self._seq
            self._condition.wait(timeout)
            if self._seq == last_seq:
                return None, last_seq
            return self._frame, self._seq


class OverlayStore:
    """최신 YOLO 오버레이. 박스는 0..1 정규화 좌표라 해상도와 무관합니다."""

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
    """SLAM 지도 PNG + world<->pixel 메타 + 로봇 pose.

    `version`은 1부터 올라갑니다 — 0은 "지도 없음"을 뜻하므로 ETag가
    실제 지도와 절대 겹치지 않습니다.
    """

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
                # ★ 지도가 없어도 pose는 내보냅니다. 목업/VLA 모드에서 SLAM이
                #   없어도 로봇 마커는 떠야 합니다.
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
        index_html: bytes | None = None,
        frame_store: FrameStore | None = None,
        overlay_store: OverlayStore | None = None,
        map_store: MapStore | None = None,
        allow_remote: bool = False,
        max_stream_clients: int = 4,
        set_vision_enabled: Callable[[bool], dict[str, Any]] | None = None,
        default_mode: str = _VLA_MODE,
    ) -> None:
        host, port = validate_server_config(host, port, allow_remote=allow_remote)
        self._status_store = status_store
        self._submit_mission = submit_mission
        self._set_vision_enabled = set_vision_enabled
        # 비어 있는 스토어를 기본값으로 둡니다 — 엔드포인트는 항상 존재하고
        # "아직 데이터 없음"을 응답합니다. ui_vision_enabled=false일 때도
        # 프론트엔드가 404가 아니라 available:false를 받습니다.
        self._frames = frame_store or FrameStore()
        self._overlays = overlay_store or OverlayStore()
        self._maps = map_store or MapStore()
        self._max_stream_clients = int(max_stream_clients)
        self._stream_clients = 0
        self._stream_lock = threading.Lock()
        raw_index_html = index_html or (
            files("fire_vla_core.web").joinpath("index.html").read_bytes()
        )
        # ui_default_mode: hw_test(Rule-based 전용)처럼 vla_orchestrator가
        # 아예 안 뜨는 배포에서는 접속하자마자 VLA 화면부터 뜨면 "상태를
        # 기다리는 중"만 보입니다 — 서빙 시점에 index.html의 자리표시자를
        # 실제 기본 모드로 치환해 둡니다.
        self._index_html = raw_index_html.replace(
            b"__DEFAULT_MODE__", normalize_mode(default_mode).encode("ascii")
        )
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

    def _acquire_stream_slot(self) -> bool:
        with self._stream_lock:
            if self._stream_clients >= self._max_stream_clients:
                return False
            self._stream_clients += 1
            return True

    def _release_stream_slot(self) -> None:
        with self._stream_lock:
            self._stream_clients = max(0, self._stream_clients - 1)

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
                if parsed.path == "/api/vision/stream":
                    self._stream_frames()
                    return
                if parsed.path == "/api/vision/frame.jpg":
                    self._send_latest_frame()
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
                    self._handle_vision_enabled_post()
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

            def _handle_vision_enabled_post(self) -> None:
                if owner._set_vision_enabled is None:
                    self._send_json(
                        HTTPStatus.SERVICE_UNAVAILABLE,
                        {"error": "vision toggle을 지원하지 않는 설정입니다."},
                    )
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    if length <= 0 or length > _MAX_REQUEST_BYTES:
                        raise ValueError("invalid content length")
                    data = json.loads(self.rfile.read(length).decode("utf-8"))
                    if not isinstance(data, dict):
                        raise ValueError("JSON object가 필요합니다.")
                    enabled = data.get("enabled")
                    if not isinstance(enabled, bool):
                        raise ValueError("enabled는 boolean이어야 합니다.")
                except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError) as exc:
                    self._send_json(
                        HTTPStatus.BAD_REQUEST,
                        {"error": str(exc) or "invalid request"},
                    )
                    return
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

            def _stream_frames(self) -> None:
                """MJPEG. 클라이언트 하나당 스레드 하나를 붙잡습니다.

                ★ 상한이 없으면 브라우저 탭 몇 개로 Pi의 스레드가 고갈됩니다.
                ★ 프레임이 안 오면 idle 한도에서 끊습니다 — 안 그러면 카메라가
                  죽은 뒤 연결이 끊긴 클라이언트를 영원히 붙잡고 샙니다.
                  (쓰기가 없으면 연결 해제를 감지할 방법이 없습니다.)
                """
                if not owner._acquire_stream_slot():
                    self._send_json(
                        HTTPStatus.SERVICE_UNAVAILABLE,
                        {"error": "stream client 상한에 도달했습니다."},
                    )
                    return
                try:
                    self.send_response(HTTPStatus.OK.value)
                    self.send_header(
                        "Content-Type",
                        f"multipart/x-mixed-replace; boundary={_STREAM_BOUNDARY}",
                    )
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Connection", "close")
                    self.end_headers()

                    last_seq, idle = 0, 0
                    while True:
                        frame, last_seq = owner._frames.wait_for(
                            last_seq, _STREAM_WAIT_SEC
                        )
                        if frame is None:
                            idle += 1
                            if idle >= _STREAM_IDLE_LIMIT:
                                break
                            continue
                        idle = 0
                        self.wfile.write(
                            f"--{_STREAM_BOUNDARY}\r\n"
                            f"Content-Type: image/jpeg\r\n"
                            f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii")
                        )
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass          # 탭을 닫은 것뿐입니다 — 로그를 채우지 않습니다
                finally:
                    owner._release_stream_slot()

            def _send_latest_frame(self) -> None:
                frame, _ = owner._frames.latest()
                if frame is None:
                    self._send_json(
                        HTTPStatus.SERVICE_UNAVAILABLE,
                        {"error": "camera frame이 아직 없습니다."},
                    )
                    return
                self._send_bytes(HTTPStatus.OK, frame, "image/jpeg")

            def _send_map_png(self) -> None:
                png, version = owner._maps.png()
                if png is None:
                    self._send_json(
                        HTTPStatus.SERVICE_UNAVAILABLE,
                        {"error": "map이 아직 없습니다."},
                    )
                    return
                etag = f'"{version}"'
                if self.headers.get("If-None-Match") == etag:
                    # 지도는 수 MB까지 갑니다. 안 바뀌었으면 다시 보내지 않습니다.
                    self.send_response(HTTPStatus.NOT_MODIFIED.value)
                    self.send_header("ETag", etag)
                    self.end_headers()
                    return
                self._send_bytes(
                    HTTPStatus.OK, png, "image/png",
                    cache_control="no-cache", etag=etag,
                )

            def _send_bytes(
                self, status: HTTPStatus, body: bytes, content_type: str,
                *, cache_control: str = "no-store", etag: str | None = None,
            ) -> None:
                self.send_response(status.value)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", cache_control)
                if etag is not None:
                    self.send_header("ETag", etag)
                self.end_headers()
                self.wfile.write(body)

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
        self.declare_parameter("ui_default_mode", _VLA_MODE)
        self.declare_parameter("ui_allow_remote", False)
        self.declare_parameter("max_stream_clients", 4)
        self.declare_parameter("ui_vision_enabled", True)
        self.declare_parameter("ui_map_enabled", True)
        self.declare_parameter("vision_topic", "/ui/camera/compressed")
        self.declare_parameter("vision_overlay_topic", "/ui/camera/overlay")
        self.declare_parameter("vision_enabled_topic", "/ui/camera/enabled")
        self.declare_parameter("map_topic", "/map")
        # ★ src/slam/config/slam.yaml과 같은 프레임명이어야 합니다.
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("map_render_period_sec", 1.0)
        self.declare_parameter("map_max_pixels", 250_000)
        self.declare_parameter("robot_pose_period_sec", 0.2)

        self._store = StatusStore()
        self._frames = FrameStore()
        self._overlays = OverlayStore()
        self._maps = MapStore()
        self._tf_buffer = None
        self._tf_listener = None
        self._map_render_period = float(
            self.get_parameter("map_render_period_sec").value
        )
        self._map_max_pixels = int(self.get_parameter("map_max_pixels").value)
        self._last_map_render = 0.0
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

        self._vision_enabled_pub = None
        if bool(self.get_parameter("ui_vision_enabled").value):
            # ★ 정수 10을 쓰면 RELIABLE이 되어 센서 구독이 조용히 실패합니다.
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
            # ui_stream_node 가 이걸 구독해서 카메라 인코딩 구독을 통째로
            # destroy_subscription 한다 — CPU를 실제로 아낀다.
            self._vision_enabled_pub = self.create_publisher(
                Bool,
                str(self.get_parameter("vision_enabled_topic").value),
                10,
            )

        if bool(self.get_parameter("ui_map_enabled").value):
            # ★ slam_toolbox는 /map을 latched로 냅니다. TRANSIENT_LOCAL로 맞추지
            #   않으면 구독 직후 화면이 비고, 다음 지도 갱신까지 기다려야 합니다.
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

        allow_remote = bool(self.get_parameter("ui_allow_remote").value)
        self._http = FirefighterHTTPServer(
            str(self.get_parameter("ui_host").value),
            int(self.get_parameter("ui_port").value),
            self._store,
            self._publish_mission,
            frame_store=self._frames,
            overlay_store=self._overlays,
            map_store=self._maps,
            allow_remote=allow_remote,
            max_stream_clients=int(
                self.get_parameter("max_stream_clients").value
            ),
            set_vision_enabled=(
                self._set_vision_enabled
                if self._vision_enabled_pub is not None
                else None
            ),
            default_mode=str(self.get_parameter("ui_default_mode").value),
        )
        self._http.start()
        host, port = self._http.address
        self.get_logger().info(f"Firefighter UI started: http://{host}:{port}")
        if allow_remote:
            self.get_logger().warn(
                "ui_allow_remote=true — UI가 LAN에 노출됩니다. "
                "Mission/START/STOP 제어 경계도 함께 열립니다."
            )

    def _status_callback(self, msg: String) -> None:
        self._update_status(msg, _VLA_MODE)

    def _rule_based_status_callback(self, msg: String) -> None:
        self._update_status(msg, _RULE_BASED_MODE)

    def _update_status(self, msg: String, mode: str) -> None:
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError("status payload는 JSON object여야 합니다.")
            self._store.update(data, mode)
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                f"{mode} status parsing failed: {exc}"
            )

    def _frame_callback(self, msg) -> None:
        self._frames.update(bytes(msg.data))

    def _overlay_callback(self, msg) -> None:
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError("overlay payload는 JSON object여야 합니다.")
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"overlay parsing failed: {exc}")
            return
        self._overlays.update(data)

    def _map_callback(self, msg) -> None:
        # 매 메시지마다 PNG를 굽는 건 낭비입니다. slam_toolbox는 0.2초마다
        # 내는데, 관제 화면이 1초 간격으로 갱신돼도 아무도 눈치채지 못합니다.
        now = time.monotonic()
        if now - self._last_map_render < self._map_render_period:
            return
        self._last_map_render = now

        width, height = int(msg.info.width), int(msg.info.height)
        step = downsample_step(width, height, self._map_max_pixels)
        png = render_occupancy_png(msg.data, width, height, step)
        metadata = grid_metadata(msg)
        # ★ PNG는 step배 축소됐습니다. 프론트엔드가 world->pixel 변환에
        #   resolution 대신 resolution*step을 써야 마커가 제자리에 앉습니다.
        #   step=1인 작은 지도에서는 안 드러나고 큰 지도에서만 어긋납니다.
        metadata["render_step"] = step
        metadata["png_width"] = math.ceil(width / step)
        metadata["png_height"] = math.ceil(height / step)
        self._maps.update_map(png, metadata)

    def _poll_robot_pose(self) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, Time()
            )
        except TransformException:
            # ★ 마지막 pose를 붙잡으면 TF가 끊긴 뒤에도 죽은 로봇이 그 자리에
            #   살아 있는 것처럼 보입니다. 모르면 모른다고 해야 합니다.
            self._maps.update_robot(None)
            return
        translation = transform.transform.translation
        self._maps.update_robot({
            "x": float(translation.x),
            "y": float(translation.y),
            "yaw": yaw_from_quaternion(transform.transform.rotation),
        })

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

    def _set_vision_enabled(self, enabled: bool) -> dict[str, Any]:
        msg = Bool()
        msg.data = enabled
        self._vision_enabled_pub.publish(msg)
        if not enabled:
            # ui_stream_node 가 구독을 끊으면 더 이상 오버레이가 안 오므로,
            # 여기서 즉시 지워서 브라우저가 오래된 박스를 계속 보여주지
            # 않게 한다.
            self._overlays.clear()
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