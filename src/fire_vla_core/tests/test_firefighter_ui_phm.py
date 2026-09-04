"""`/api/phm` — 로봇의 phm_monitor 가 낸 건전성 상태를 UI 가 실어 나르는 경로.

UI 는 PHM 페이로드를 **해석하지 않습니다.** `/vla/status` 와 똑같이 JSON 을 받아
그대로 넘깁니다. 그래서 검출 임계가 바뀌어도(이 프로젝트는 네 번 바꿨습니다)
UI 코드는 안 바뀝니다. 여기 테스트가 지키는 것은 그 '안 해석함' 과,
**끊긴 값을 정상으로 보여주지 않는 것** 두 가지입니다.
"""
import json
import time
from urllib.request import Request, urlopen

import pytest

from fire_vla_core.ros.firefighter_ui_node import (
    FirefighterHTTPServer,
    PhmStore,
    StatusStore,
    create_mission_payload,
)


def request(server, path):
    # tests/ 에 __init__.py 가 없어 test_firefighter_ui 에서 가져올 수 없습니다.
    host, port = server.address
    with urlopen(Request(f"http://{host}:{port}{path}"), timeout=2) as response:
        return response.status, response.headers["Content-Type"], response.read()


def sample_payload(health="OK", alarms=None):
    """phm_monitor 가 실제로 내는 모양 (phm_monitor_node.tick)."""
    return {
        "schema_version": 1,
        "mode": "PHM",
        "timestamp": "2026-09-04T12:00:00",
        "health": health,
        "alarms": alarms or [],
        "not_detected": ["SLIP"],
        "axes": {
            "yaw": {"axis": "yaw", "residual": 0.08, "threshold": 0.35,
                    "ratio": 0.0, "alarm": False, "events": 0,
                    "evaluated": 476, "fresh": True, "age_sec": 0.02,
                    "unit": "rad/s", "label": "요레이트", "meas": "자이로"},
            "fwd": {"axis": "fwd", "residual": 0.02, "threshold": 0.15,
                    "ratio": 0.0, "alarm": False, "events": 0,
                    "evaluated": 462, "fresh": True, "age_sec": 0.1,
                    "unit": "m/s", "label": "전진속도", "meas": "rf2o"},
        },
        "cmd_source": "/controller/cmd_vel",
        "battery_mv": 7826,
        "host": {"cpu_used_pct": 14.0},
        "blocked_reason": None,
        "rules": {"yaw": {"thr": 0.35}, "fwd": {"thr": 0.15}},
    }


@pytest.fixture
def phm_server():
    phm = PhmStore()
    server = FirefighterHTTPServer(
        "127.0.0.1", 0, StatusStore(),
        lambda text, mode="VLA": create_mission_payload(text),
        phm_store=phm, phm_stale_sec=1.0,
    )
    server.start()
    yield server, phm
    server.close()


def get_phm(server):
    status, content_type, body = request(server, "/api/phm")
    assert status == 200
    assert "application/json" in content_type
    return json.loads(body)


def test_endpoint_exists_before_any_status_arrives(phm_server):
    """phm_monitor 가 안 떠 있어도 404 가 아니라 available:false 여야 합니다."""
    server, _ = phm_server
    data = get_phm(server)
    assert data["available"] is False
    assert data["health"] == "UNKNOWN"
    assert data["alarms"] == []
    assert "phm_monitor" in data["blocked_reason"]


def test_payload_is_passed_through_untouched(phm_server):
    """UI 는 내용을 해석하지 않습니다 — 넣은 그대로 나와야 합니다."""
    server, phm = phm_server
    payload = sample_payload()
    phm.update(payload)
    data = get_phm(server)
    for key, value in payload.items():
        assert data[key] == value, f"{key} 가 변형됐습니다"


def test_alarm_payload_survives_round_trip(phm_server):
    server, phm = phm_server
    alarms = [{"name": "LIFT_SUSPECTED", "axis": "fwd",
               "residual": 0.187, "threshold": 0.15}]
    phm.update(sample_payload(health="ALARM", alarms=alarms))
    data = get_phm(server)
    assert data["health"] == "ALARM"
    assert data["alarms"] == alarms
    # 이 검출기가 못 잡는 것을 화면이 알아야 'ALL CLEAR' 라고 안 씁니다.
    assert data["not_detected"] == ["SLIP"]


def test_stale_status_is_not_reported_as_ok(phm_server):
    """phm_monitor 가 죽으면 마지막 OK 가 그대로 남습니다.

    그걸 OK 로 보여주면 **고장이 없는 것처럼** 보입니다. 값은 그대로 넘기되
    health 는 UNKNOWN 으로 덮어야 합니다.
    """
    server, phm = phm_server            # phm_stale_sec=1.0
    phm.update(sample_payload(health="OK"))
    assert get_phm(server)["health"] == "OK"

    time.sleep(1.1)
    data = get_phm(server)
    assert data["stale"] is True
    assert data["health"] == "UNKNOWN"
    assert "갱신되지 않았습니다" in data["blocked_reason"]
    # 값 자체는 살아 있어야 화면이 '마지막으로 본 값' 을 표시할 수 있습니다.
    assert data["axes"]["yaw"]["residual"] == 0.08


def test_store_returns_copy_not_shared_mutable_state(phm_server):
    server, phm = phm_server
    payload = sample_payload()
    phm.update(payload)
    payload["axes"]["yaw"]["residual"] = 99.0     # 넣은 뒤 밖에서 건드림
    assert get_phm(server)["axes"]["yaw"]["residual"] == 0.08


def test_age_is_reported(phm_server):
    server, phm = phm_server
    phm.update(sample_payload())
    data = get_phm(server)
    assert data["available"] is True
    assert data["stale"] is False
    assert 0.0 <= data["age_sec"] < 1.0


# ---------------------------------------------------------------------------
# 화면 쪽 — index.html 의 PHM 패널
# ---------------------------------------------------------------------------
def test_index_html_serves_phm_panel(phm_server):
    """패널 자체가 서빙되는지. 요소 id 가 바뀌면 JS 가 조용히 아무것도 안 합니다."""
    server, _ = phm_server
    status, content_type, body = request(server, "/")
    assert status == 200
    assert "text/html" in content_type
    page = body.decode("utf-8")
    assert "Robot Health (PHM)" in page
    for element_id in ("phmHealth", "phmAge", "phmAxes", "phmBlocked",
                       "phmLimit", "phmBattery", "phmCpu", "phmTemp", "phmCmd"):
        assert f'id="{element_id}"' in page, f"{element_id} 가 없습니다"
    # 모드 전환과 무관한 상시 패널이어야 합니다 — /api/status 가 아니라 /api/phm.
    assert "fetch('/api/phm'" in page
    assert "setInterval(refreshPhm,1000)" in page


def test_phm_render_rules_hold(tmp_path):
    """index.html 의 렌더링을 실제로 실행해 세 가지 규칙을 확인합니다.

    조건부 렌더링이라 마크업 검사만으로는 부족합니다. node 가 없으면 건너뜁니다.
    """
    import shutil
    import subprocess
    from pathlib import Path

    node = shutil.which("node")
    if node is None:
        pytest.skip("node 가 없어 건너뜁니다")
    here = Path(__file__).resolve().parent
    harness = here / "js" / "phm_render_test.mjs"
    index = here.parent / "fire_vla_core" / "web" / "index.html"
    result = subprocess.run([node, str(harness), str(index)],
                            capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stdout + result.stderr
