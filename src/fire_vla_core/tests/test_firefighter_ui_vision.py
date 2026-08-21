import threading

from fire_vla_core.ros.firefighter_ui_node import (
    FrameStore,
    MapStore,
    OverlayStore,
)


class TestFrameStore:
    def test_starts_empty(self):
        assert FrameStore().latest() == (None, 0)

    def test_update_bumps_sequence(self):
        store = FrameStore()
        store.update(b"a")
        store.update(b"b")
        assert store.latest() == (b"b", 2)

    def test_wait_returns_immediately_when_newer_frame_exists(self):
        store = FrameStore()
        store.update(b"a")
        assert store.wait_for(0, timeout=1.0) == (b"a", 1)

    def test_wait_times_out_instead_of_sleeping_forever(self):
        """★ 카메라가 멈추면 스레드가 영원히 잠들고 연결 해제도 못 봅니다."""
        store = FrameStore()
        store.update(b"a")
        assert store.wait_for(1, timeout=0.05) == (None, 1)

    def test_wait_wakes_on_a_late_frame(self):
        store = FrameStore()
        timer = threading.Timer(0.02, lambda: store.update(b"b"))
        timer.start()
        try:
            assert store.wait_for(0, timeout=2.0) == (b"b", 1)
        finally:
            timer.cancel()


class TestOverlayStore:
    def test_reports_unavailable_before_any_overlay(self):
        assert OverlayStore().get() == {"available": False, "boxes": []}

    def test_marks_available_and_keeps_payload(self):
        store = OverlayStore()
        store.update({"seq": 3, "width": 640, "height": 360, "boxes": [{"cx": 0.5}]})
        result = store.get()
        assert result["available"] is True
        assert result["seq"] == 3 and result["boxes"][0]["cx"] == 0.5

    def test_returns_a_copy_not_shared_mutable_state(self):
        store = OverlayStore()
        source = {"boxes": [{"cx": 0.5}]}
        store.update(source)
        source["boxes"].clear()
        store.get()["boxes"].clear()
        assert store.get()["boxes"][0]["cx"] == 0.5


class TestMapStore:
    def test_reports_unavailable_with_version_zero(self):
        result = MapStore().get()
        assert result["available"] is False
        assert result["version"] == 0 and result["robot"] is None

    def test_first_map_is_version_one(self):
        """0은 '지도 없음'이므로 ETag가 실제 지도와 겹치지 않습니다."""
        store = MapStore()
        assert store.update_map(b"png", {"width": 4}) == 1
        assert store.png() == (b"png", 1)

    def test_each_map_bumps_the_version(self):
        store = MapStore()
        store.update_map(b"a", {"width": 4})
        assert store.update_map(b"b", {"width": 4}) == 2
        assert store.get()["version"] == 2

    def test_robot_pose_is_served_without_a_map(self):
        """SLAM이 없어도 로봇 마커는 떠야 합니다 (목업/VLA 모드)."""
        store = MapStore()
        store.update_robot({"x": 1.0, "y": -2.0, "yaw": 0.5})
        result = store.get()
        assert result["available"] is False
        assert result["robot"] == {"x": 1.0, "y": -2.0, "yaw": 0.5}

    def test_metadata_is_merged_into_the_snapshot(self):
        store = MapStore()
        store.update_map(b"png", {"width": 4, "height": 3, "resolution": 0.05})
        result = store.get()
        assert result["available"] is True
        assert (result["width"], result["height"]) == (4, 3)

    def test_robot_pose_cleared_when_tf_is_lost(self):
        store = MapStore()
        store.update_robot({"x": 1.0, "y": 0.0, "yaw": 0.0})
        store.update_robot(None)
        assert store.get()["robot"] is None


import pytest, json
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from fire_vla_core.ros.firefighter_ui_node import (
    FirefighterHTTPServer,
    StatusStore,
    validate_server_config,
)


def _submit(text, mode="VLA"):
    return {"mission_id": "mission_test", "text": text}


def _get(server, path, headers=None):
    host, port = server.address
    request = Request(f"http://{host}:{port}{path}", headers=headers or {})
    try:
        with urlopen(request, timeout=3) as response:
            return response.status, dict(response.headers), response.read()
    except HTTPError as error:
        return error.code, dict(error.headers), error.read()


@pytest.fixture
def vision_server():
    frames, overlays, maps = FrameStore(), OverlayStore(), MapStore()
    server = FirefighterHTTPServer(
        "127.0.0.1", 0, StatusStore(), _submit,
        frame_store=frames, overlay_store=overlays, map_store=maps,
    )
    server.start()
    yield server, frames, overlays, maps
    server.close()


class TestRemoteBinding:
    def test_loopback_is_still_enforced_by_default(self):
        with pytest.raises(ValueError, match="ui_host"):
            validate_server_config("0.0.0.0", 8080)

    def test_remote_bind_requires_explicit_opt_in(self):
        assert validate_server_config("0.0.0.0", 8080, allow_remote=True) == (
            "0.0.0.0", 8080
        )

    def test_empty_host_is_rejected_even_with_opt_in(self):
        """빈 문자열은 전체 인터페이스 바인딩이 됩니다 — 사고로 열리면 안 됩니다."""
        with pytest.raises(ValueError, match="ui_host"):
            validate_server_config("   ", 8080, allow_remote=True)


class TestVisionEndpoints:
    def test_frame_is_unavailable_before_the_camera_publishes(self, vision_server):
        server, _, _, _ = vision_server
        status, _, _ = _get(server, "/api/vision/frame.jpg")
        assert status == 503

    def test_latest_frame_is_served_as_jpeg(self, vision_server):
        server, frames, _, _ = vision_server
        frames.update(b"\xff\xd8fake\xff\xd9")
        status, headers, body = _get(server, "/api/vision/frame.jpg")
        assert status == 200
        assert headers["Content-Type"] == "image/jpeg"
        assert body == b"\xff\xd8fake\xff\xd9"

    def test_detections_report_unavailable_without_crashing(self, vision_server):
        server, _, _, _ = vision_server
        status, _, body = _get(server, "/api/vision/detections")
        assert status == 200
        assert json.loads(body) == {"available": False, "boxes": []}

    def test_detections_expose_normalized_boxes(self, vision_server):
        server, _, overlays, _ = vision_server
        overlays.update({
            "seq": 7, "width": 640, "height": 360,
            "boxes": [{"class_name": "fire", "confidence": 0.9,
                       "cx": 0.5, "cy": 0.5, "w": 0.1, "h": 0.1}],
        })
        _, _, body = _get(server, "/api/vision/detections")
        payload = json.loads(body)
        assert payload["available"] is True and payload["seq"] == 7
        assert payload["boxes"][0]["class_name"] == "fire"

    def test_stream_sends_multipart_jpeg_parts(self, vision_server):
        server, frames, _, _ = vision_server
        frames.update(b"\xff\xd8fake\xff\xd9")
        host, port = server.address
        with urlopen(f"http://{host}:{port}/api/vision/stream", timeout=3) as response:
            content_type = response.headers["Content-Type"]
            assert "multipart/x-mixed-replace" in content_type
            assert "boundary=phoenixframe" in content_type
            # ★ 전체를 읽으면 스트림이 끝나지 않습니다 — 첫 파트 머리만 읽습니다.
            head = response.read(len(b"--phoenixframe\r\nContent-Type: image/jpeg\r\n"))
        assert head.startswith(b"--phoenixframe")

    def test_stream_is_refused_past_the_client_limit(self):
        """상한이 없으면 탭 몇 개로 Pi의 스레드가 고갈됩니다."""
        server = FirefighterHTTPServer(
            "127.0.0.1", 0, StatusStore(), _submit,
            frame_store=FrameStore(), max_stream_clients=0,
        )
        server.start()
        try:
            status, _, _ = _get(server, "/api/vision/stream")
            assert status == 503
        finally:
            server.close()


class TestMapEndpoints:
    def test_map_reports_unavailable_but_still_answers(self, vision_server):
        server, _, _, _ = vision_server
        status, _, body = _get(server, "/api/map")
        assert status == 200
        assert json.loads(body)["available"] is False

    def test_map_png_is_unavailable_before_slam_publishes(self, vision_server):
        server, _, _, _ = vision_server
        status, _, _ = _get(server, "/api/map.png")
        assert status == 503

    def test_map_png_is_served_with_an_etag(self, vision_server):
        server, _, _, maps = vision_server
        maps.update_map(b"\x89PNG-fake", {"width": 4, "height": 3})
        status, headers, body = _get(server, "/api/map.png")
        assert status == 200
        assert headers["Content-Type"] == "image/png"
        assert headers["ETag"] == '"1"'
        assert body == b"\x89PNG-fake"

    def test_unchanged_map_returns_304(self, vision_server):
        """지도는 수 MB까지 갑니다 — 안 바뀌면 다시 보내지 않습니다."""
        server, _, _, maps = vision_server
        maps.update_map(b"\x89PNG-fake", {"width": 4})
        status, _, _ = _get(server, "/api/map.png", {"If-None-Match": '"1"'})
        assert status == 304

    def test_new_map_invalidates_the_cached_version(self, vision_server):
        server, _, _, maps = vision_server
        maps.update_map(b"first", {"width": 4})
        maps.update_map(b"second", {"width": 4})
        status, headers, body = _get(server, "/api/map.png", {"If-None-Match": '"1"'})
        assert status == 200 and headers["ETag"] == '"2"' and body == b"second"

    def test_map_json_carries_robot_pose(self, vision_server):
        server, _, _, maps = vision_server
        maps.update_robot({"x": 1.0, "y": -2.0, "yaw": 0.5})
        _, _, body = _get(server, "/api/map")
        assert json.loads(body)["robot"]["x"] == 1.0