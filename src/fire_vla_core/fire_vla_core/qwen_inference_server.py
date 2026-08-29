from __future__ import annotations

import argparse
import json
import logging
import threading
import time
from itertools import count
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable

from .llm import (
    ALLOWED_ACTIONS,
    LLMError,
    MockVLABrain,
    TransformersQwenAdapter,
)
from .ports import LLMPort


MAX_REQUEST_BYTES = 1_000_000
LOGGER = logging.getLogger(__name__)


def infer(backend: LLMPort, payload: dict[str, Any]) -> dict[str, Any]:
    if set(payload) != {"mission", "world_model", "allowed_actions"}:
        raise ValueError(
            "mission, world_model, allowed_actions 세 필드가 필요합니다."
        )
    mission = payload["mission"]
    world_model = payload["world_model"]
    allowed_actions = payload["allowed_actions"]
    if not isinstance(mission, str) or not mission.strip():
        raise ValueError("mission은 비어 있지 않은 문자열이어야 합니다.")
    if not isinstance(world_model, dict):
        raise ValueError("world_model은 JSON 객체여야 합니다.")
    if allowed_actions != ALLOWED_ACTIONS:
        raise ValueError("allowed_actions가 server contract와 일치하지 않습니다.")
    decision = backend.decide(mission.strip(), world_model)
    return {
        "mission_scope": (
            decision.mission_scope.value if decision.mission_scope else None
        ),
        "action": decision.action.value,
        "target": decision.target,
        "reason": decision.reason,
    }


def create_handler(backend: LLMPort) -> type[BaseHTTPRequestHandler]:
    inference_lock = threading.Lock()
    request_ids = count(1)

    class InferenceHandler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            if self.path != "/health":
                self._send(HTTPStatus.NOT_FOUND, {"error": "not found"})
                return
            self._send(HTTPStatus.OK, {"status": "ok"})

        def do_POST(self) -> None:
            if self.path != "/infer":
                self._send(HTTPStatus.NOT_FOUND, {"error": "not found"})
                return
            request_id = next(request_ids)
            received_at = time.monotonic()
            self._log_event("request_received", request_id, received_at)
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0 or length > MAX_REQUEST_BYTES:
                    raise ValueError("유효하지 않은 request 크기입니다.")
                payload = json.loads(self.rfile.read(length).decode("utf-8"))
                if not isinstance(payload, dict):
                    raise ValueError("request body는 JSON 객체여야 합니다.")
                if not inference_lock.acquire(blocking=False):
                    self._log_event("request_busy", request_id, received_at)
                    self._send(
                        HTTPStatus.TOO_MANY_REQUESTS,
                        {"error": "Qwen inference가 이미 실행 중입니다."},
                    )
                    return
                try:
                    self._log_event("inference_start", request_id, received_at)
                    result = infer(backend, payload)
                    self._log_event("inference_end", request_id, received_at)
                finally:
                    inference_lock.release()
            except (json.JSONDecodeError, UnicodeDecodeError, ValueError) as exc:
                self._send(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
                return
            except LLMError as exc:
                self._send(
                    HTTPStatus.SERVICE_UNAVAILABLE,
                    {"error": str(exc)},
                )
                return
            self._send(HTTPStatus.OK, result)
            self._log_event("response_sent", request_id, received_at)

        def log_message(self, format: str, *args: Any) -> None:
            return

        def _send(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                LOGGER.info("qwen_http event=response_disconnected")

        @staticmethod
        def _log_event(event: str, request_id: int, started_at: float) -> None:
            LOGGER.info(
                "qwen_http event=%s request_id=%d elapsed_sec=%.6f",
                event,
                request_id,
                time.monotonic() - started_at,
            )

    return InferenceHandler


def build_backend(args: argparse.Namespace) -> LLMPort:
    if args.backend == "mock":
        return MockVLABrain()
    return TransformersQwenAdapter(
        model_id=args.model_id,
        device=args.device,
        max_new_tokens=args.max_new_tokens,
    )


def create_server(
    host: str,
    port: int,
    backend: LLMPort,
    server_class: Callable[..., ThreadingHTTPServer] = ThreadingHTTPServer,
) -> ThreadingHTTPServer:
    return server_class((host, port), create_handler(backend))


def main(argv: list[str] | None = None) -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    parser = argparse.ArgumentParser(
        description="PC-only Qwen inference HTTP server"
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8088)
    parser.add_argument(
        "--backend",
        choices=("mock", "transformers"),
        default="transformers",
    )
    parser.add_argument(
        "--model-id", default="Qwen/Qwen3-1.7B"
    )
    parser.add_argument("--device", default="xpu:0")
    parser.add_argument("--max-new-tokens", type=int, default=64)
    args = parser.parse_args(argv)
    server = create_server(args.host, args.port, build_backend(args))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
