from __future__ import annotations

import copy
import json
import threading
import uuid
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib.resources import files
from typing import Any, Callable
from urllib.parse import parse_qs, urlparse

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:
    rclpy = None
    Node = object
    String = None


_MAX_REQUEST_BYTES = 64 * 1024
_ALLOWED_HOSTS = {"127.0.0.1", "localhost"}
_VLA_MODE = "VLA"
_RULE_BASED_MODE = "RULE_BASED"
_ALLOWED_MODES = {_VLA_MODE, _RULE_BASED_MODE}
_RULE_BASED_COMMANDS = {"START", "STOP"}


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


class FirefighterHTTPServer:
    def __init__(
        self,
        host: str,
        port: int,
        status_store: StatusStore,
        submit_mission: Callable[[str], dict[str, str]],
        *,
        index_html: bytes | None = None,
    ) -> None:
        host, port = validate_server_config(host, port)
        self._status_store = status_store
        self._submit_mission = submit_mission
        self._index_html = index_html or (
            files("fire_vla_core.web").joinpath("index.html").read_bytes()
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
                self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

            def do_POST(self) -> None:
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

        self._store = StatusStore()
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
        self._http = FirefighterHTTPServer(
            str(self.get_parameter("ui_host").value),
            int(self.get_parameter("ui_port").value),
            self._store,
            self._publish_mission,
        )
        self._http.start()
        host, port = self._http.address
        self.get_logger().info(f"Firefighter UI started: http://{host}:{port}")

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
