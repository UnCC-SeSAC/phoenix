#!/usr/bin/env python3
"""
메인에 보낼 JSON 페이로드 테스트 — rclpy 없이 돕니다.

여기서 잡으려는 사고 4가지. 전부 "돌아가는데 틀린" 쪽입니다:

  - 거리 불명이 0.0으로 나감        -> 메인이 로봇 발밑을 역투영
  - NaN이 JSON에 그대로 들어감       -> 파이썬끼리는 왕복되고 메인에서 깨짐
  - numpy 스칼라를 직렬화            -> 콜백에서 TypeError, 검출이 통째로 사라짐
  - 폴백 거리가 "ok"로 나감          -> +0.6m 틀린 값을 정확한 값처럼 신뢰
"""

from __future__ import annotations

import json
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from image_pipeline.depth import DistanceSample  # noqa: E402
from image_pipeline.detection_json import (  # noqa: E402
    ON_TARGET_STATUSES,
    STATE_NO_INPUT,
    STATE_OK,
    STATE_STALLED,
    STATE_WAITING_INFO,
    STATUS_BELOW,
    STATUS_BOTTOM,
    STATUS_OK,
    STATUS_RING,
    STATUS_UNKNOWN,
    SURROGATE_STATUSES,
    build_heartbeat,
    is_surrogate,
    build_payload,
    depth_status,
    heartbeat_state,
    detection_entry,
    stamp_age_sec,
    stamp_fields,
    to_json,
)


def sample(distance, region="center", reason="ok"):
    return DistanceSample(distance, 100, 100, 1.0, 0.05, region, reason)


class TestDepthStatus:
    """거리의 출처를 상태로 옮기는 규칙."""

    def test_center_ok_is_ok(self):
        assert depth_status(sample(2.0)) == STATUS_OK

    @pytest.mark.parametrize("region,expected", [
        ("bottom", STATUS_BOTTOM),
        ("below", STATUS_BELOW),
        ("ring", STATUS_RING),
    ])
    def test_fallback_regions_are_not_ok(self, region, expected):
        """★ 폴백이 `ok`로 나가면 안 됩니다.

        `ring`은 대상이 아니라 주변을 재고도 유효 100%·사유 `ok`를 냅니다
        (HANDOVER 8, 편향 +0.6m). 사유가 아니라 region으로 판단해야 하는 이유.
        """
        assert depth_status(sample(3.8, region=region)) == expected
        assert depth_status(sample(3.8, region=region)) != STATUS_OK

    @pytest.mark.parametrize("reason", [
        "no_valid_pixels", "low_valid_ratio", "box_outside_image", "too_spread",
    ])
    def test_failed_reasons_are_unknown(self, reason):
        assert depth_status(sample(None, reason=reason)) == STATUS_UNKNOWN

    def test_nan_distance_is_unknown(self):
        assert depth_status(sample(float("nan"))) == STATUS_UNKNOWN

    def test_unknown_region_raises(self):
        """새 폴백을 추가하고 계약을 안 고치면 여기서 걸립니다."""
        with pytest.raises(ValueError):
            depth_status(sample(2.0, region="diagonal"))


class TestSurrogate:
    """★ "대상을 쟀는가"를 문자열 접두사로 판단하면 안 됩니다.

    `fallback_bottom`은 이름과 달리 박스 **안**이라 대상 자체를 읽습니다
    (합성 장면 실측 오차 0.000m). 2026-08-24부터 노드 기본 region이 `bottom`이라,
    접두사로 거르면 `publish_fallback=false`에서 **모든 거리가 지워집니다.**
    """

    def test_주변을_잰_것만_대리값이다(self):
        assert is_surrogate(STATUS_BELOW) is True
        assert is_surrogate(STATUS_RING) is True

    def test_대상을_잰_것은_대리값이_아니다(self):
        assert is_surrogate(STATUS_OK) is False
        assert is_surrogate(STATUS_BOTTOM) is False

    def test_이름_접두사와_일치하지_않는다(self):
        """이 어긋남이 버그의 원천이라 테스트로 못 박습니다."""
        assert STATUS_BOTTOM.startswith("fallback")
        assert is_surrogate(STATUS_BOTTOM) is False

    def test_불명은_대리값이_아니다(self):
        """거리가 아예 없는 것과 주변을 잰 것은 다른 상태입니다."""
        assert is_surrogate(STATUS_UNKNOWN) is False

    def test_두_분류가_모든_상태를_덮는다(self):
        """상태를 새로 추가하고 분류를 빠뜨리면 여기서 걸립니다."""
        assert set(ON_TARGET_STATUSES) | set(SURROGATE_STATUSES) == {
            STATUS_OK, STATUS_BOTTOM, STATUS_BELOW, STATUS_RING}
        assert not set(ON_TARGET_STATUSES) & set(SURROGATE_STATUSES)


class TestUnknownDistance:
    """★ 계약의 핵심 — 거리 불명은 0이 아니라 null."""

    def test_unknown_depth_is_null_not_zero(self):
        entry = detection_entry("fire", 0.9, 320, 240,
                                sample(None, reason="no_valid_pixels"))
        assert entry["depth"] is None
        assert entry["depth"] != 0.0
        assert entry["depth_status"] == STATUS_UNKNOWN

    def test_zero_meters_never_appears_for_unknown(self):
        """0.0m는 카메라 스펙(0.2~4m) 밖이고, 메인이 역투영하면 발밑입니다."""
        text = to_json(build_payload(1, 0, (640, 480), [
            detection_entry("fire", 0.9, 320, 240, sample(None,
                                                          reason="low_valid_ratio")),
        ]))
        assert json.loads(text)["detections"][0]["depth"] is None

    def test_status_unknown_drops_a_stale_distance(self):
        """상태가 불명이면 값이 들어 있어도 버립니다."""
        entry = detection_entry("fire", 0.9, 10, 10, depth=2.0,
                                status=STATUS_UNKNOWN)
        assert entry["depth"] is None

    def test_missing_distance_forces_unknown_status(self):
        """거꾸로, 값이 없는데 ok로 나가는 것도 막습니다."""
        entry = detection_entry("fire", 0.9, 10, 10, depth=None, status=STATUS_OK)
        assert entry["depth_status"] == STATUS_UNKNOWN


class TestJsonValidity:
    """직렬화가 조용히 깨지는 두 경우."""

    def test_nan_is_never_serialized(self):
        """★ `NaN`은 JSON이 아닙니다 (RFC 8259).

        `json.dumps`의 기본값은 `NaN`을 그대로 씁니다. 파이썬끼리는 왕복되니
        로컬에서는 안 걸리고, 메인이 엄격한 파서를 쓰면 프레임이 버려집니다.
        """
        text = to_json(build_payload(1, 0, (640, 480), [
            detection_entry("fire", 0.9, 320, 240, sample(float("nan"))),
        ]))
        assert "NaN" not in text
        assert "Infinity" not in text
        json.loads(text)  # 표준 파서로 왕복

    def test_raw_nan_in_payload_raises_instead_of_shipping(self):
        """모듈을 우회해 NaN을 넣어도 발행되지 않고 터집니다."""
        with pytest.raises(ValueError):
            to_json({"stamp_sec": 1, "detections": [{"depth": float("nan")}]})

    def test_numpy_scalars_are_serializable(self):
        """★ `depth.py`가 내는 값은 전부 numpy입니다.

        그대로 넣으면 `json.dumps`가 TypeError를 냅니다 — 콜백 안에서 나므로
        에러 로그 없이 검출이 통째로 사라지는 형태로 보입니다.
        """
        entry = detection_entry("fire", np.float32(0.87),
                                np.int64(320), np.float64(240.4),
                                sample(np.float32(2.0)))
        text = to_json(build_payload(np.int64(1), np.int64(0), (640, 480), [entry]))
        back = json.loads(text)["detections"][0]
        assert isinstance(back["depth"], float)
        assert back["x"] == 320 and back["y"] == 240

    def test_korean_class_name_survives(self):
        text = to_json(build_payload(1, 0, (640, 480),
                                     [detection_entry("불씨", 0.9, 1, 1,
                                                      sample(2.0))]))
        assert json.loads(text)["detections"][0]["class_name"] == "불씨"


class TestPayloadShape:
    """메인이 준 예시와 모양이 같은지."""

    def test_matches_contract_example(self):
        payload = build_payload(1786329608, 489463639, (640, 480), [
            detection_entry("fire", 0.87, 320, 240, sample(2.0)),
            detection_entry("person", 0.94, 200, 304, sample(2.4)),
        ])
        assert payload == {
            "stamp_sec": 1786329608,
            "stamp_nanosec": 489463639,
            "frame_size": [640, 480],
            "detections": [
                {"class_name": "fire", "score": 0.87, "x": 320, "y": 240,
                 "depth": 2.0, "depth_status": "ok"},
                {"class_name": "person", "score": 0.94, "x": 200, "y": 304,
                 "depth": 2.4, "depth_status": "ok"},
            ],
        }

    def test_key_order_matches_example(self):
        """`ros2 topic echo`로 메인 예시와 눈으로 대조할 수 있어야 합니다."""
        entry = detection_entry("fire", 0.87, 320, 240, sample(2.0))
        assert list(entry) == ["class_name", "score", "x", "y",
                               "depth", "depth_status"]

    def test_empty_detections_builds_a_valid_payload(self):
        """빈 배열도 유효한 페이로드여야 합니다.

        2026-08-11부터 노드는 검출 0개 프레임을 **안 냅니다** — 살아있다는
        말은 하트비트가 합니다. 다만 `publish_empty:=true`로 옛 동작을 켤 수
        있으므로 조립 자체는 깨지면 안 됩니다.
        """
        payload = build_payload(1, 0, (640, 480), [])
        assert payload["detections"] == []

    def test_mixed_known_and_unknown_in_one_frame(self):
        payload = build_payload(1, 0, (640, 480), [
            detection_entry("fire", 0.87, 320, 240, sample(None,
                                                           reason="no_valid_pixels")),
            detection_entry("person", 0.94, 200, 304, sample(2.4)),
        ])
        depths = [d["depth"] for d in payload["detections"]]
        assert depths == [None, 2.4]

    def test_float_residue_is_rounded_away(self):
        entry = detection_entry("fire", 0.87, 320, 240, sample(2.0 + 4e-16))
        assert entry["depth"] == 2.0
        assert "e-" not in json.dumps(entry)


class TestStamp:
    """★ 원본 이미지 stamp가 끝까지 살아야 합니다."""

    def test_stamp_is_copied_verbatim(self):
        payload = build_payload(1786329608, 489463639, (640, 480), [])
        assert payload["stamp_sec"] == 1786329608
        assert payload["stamp_nanosec"] == 489463639

    def test_stamp_fields_reads_ros_header(self):
        class Stamp:
            sec, nanosec = 1786329608, 489463639

        class Header:
            stamp = Stamp()

        assert stamp_fields(Header()) == (1786329608, 489463639)

    def test_nanosec_out_of_range_raises(self):
        """초를 float으로 받아 nanosec를 안 맞춘 실수가 여기서 걸립니다."""
        with pytest.raises(ValueError):
            build_payload(1, 1_500_000_000, (640, 480), [])

    def test_stamp_survives_json_roundtrip_without_precision_loss(self):
        """★ nanosec를 float 초로 합치면 정밀도가 날아갑니다.

        1786329608.489463639을 double에 담으면 ns 자리가 뭉개집니다. 그래서
        계약이 sec/nanosec 두 정수입니다 — 합치지 마세요.
        """
        back = json.loads(to_json(build_payload(1786329608, 489463639,
                                                (640, 480), [])))
        assert back["stamp_nanosec"] == 489463639
        assert isinstance(back["stamp_nanosec"], int)


class TestFrameSize:
    """`x`,`y`의 기준 해상도. 없으면 메인이 축소본과 원본을 구분 못 합니다."""

    def test_frame_size_is_reported(self):
        assert build_payload(1, 0, (640, 480), [])["frame_size"] == [640, 480]

    @pytest.mark.parametrize("bad", [(0, 480), (640, 0), (-640, 480)])
    def test_bad_frame_size_raises(self, bad):
        with pytest.raises(ValueError):
            build_payload(1, 0, bad, [])


class TestPixelCoords:
    def test_pixels_are_integers(self):
        entry = detection_entry("fire", 0.9, 319.6, 240.4, sample(2.0))
        assert entry["x"] == 320 and entry["y"] == 240
        assert isinstance(entry["x"], int)

    def test_non_finite_pixel_raises_instead_of_zero(self):
        """0(화면 좌상단)을 조용히 넣으면 메인이 엉뚱한 방향을 조준합니다."""
        with pytest.raises(ValueError):
            detection_entry("fire", 0.9, float("nan"), 240, sample(2.0))


class TestHeartbeatState:
    """★ 하트비트가 거짓말하지 않는지 — 이 노드의 대표 실패는 침묵입니다."""

    def test_ok_when_recently_processed(self):
        assert heartbeat_state(0.07, camera_info_ready=True, inputs_seen=True,
                               stall_after_sec=1.0) == STATE_OK

    def test_stalled_when_inputs_arrive_but_nothing_processed(self):
        """★★ 가장 중요한 케이스.

        YOLO가 stamp를 now()로 덮으면 동기화가 영원히 안 맞습니다. 입력은
        멀쩡히 오므로 단순 펄스 하트비트는 계속 '정상'을 주장합니다.
        """
        assert heartbeat_state(None, camera_info_ready=True, inputs_seen=True,
                               stall_after_sec=1.0) == STATE_STALLED

    def test_stalled_when_last_frame_is_old(self):
        assert heartbeat_state(3.0, camera_info_ready=True, inputs_seen=True,
                               stall_after_sec=1.0) == STATE_STALLED

    def test_waiting_camera_info_beats_stalled(self):
        """시작 직후 정상적인 대기가 stalled로 보이면 매번 거짓 경보가 납니다."""
        assert heartbeat_state(None, camera_info_ready=False, inputs_seen=True,
                               stall_after_sec=1.0) == STATE_WAITING_INFO

    def test_no_input_is_distinct_from_stalled(self):
        """토픽 이름·QoS 문제와 동기화 문제는 대응이 다릅니다."""
        assert heartbeat_state(None, camera_info_ready=True, inputs_seen=False,
                               stall_after_sec=1.0) == STATE_NO_INPUT


class TestHeartbeatPayload:
    def test_carries_evidence_not_just_a_pulse(self):
        """★ 받는 쪽이 state를 안 믿어도 직접 판정할 수 있어야 합니다."""
        hb = build_heartbeat(1786329610, 0, STATE_OK,
                             last_frame=(1786329609, 500000000), age_sec=0.5)
        assert hb["last_frame_sec"] == 1786329609
        assert hb["last_frame_nanosec"] == 500000000
        assert hb["age_sec"] == 0.5

    def test_stamp_is_publish_time_not_frame_time(self):
        """★ 이벤트와 반대입니다. 둘을 섞으면 age를 못 구합니다."""
        hb = build_heartbeat(1786329610, 0, STATE_OK,
                             last_frame=(1786329609, 0), age_sec=1.0)
        assert hb["stamp_sec"] == 1786329610
        assert hb["stamp_sec"] != hb["last_frame_sec"]

    def test_no_frame_yet_is_null_not_zero(self):
        """0을 넣으면 1970년 프레임을 처리한 것처럼 보입니다."""
        hb = build_heartbeat(1786329610, 0, STATE_NO_INPUT)
        assert hb["last_frame_sec"] is None
        assert hb["age_sec"] is None

    def test_serializes_as_valid_json(self):
        text = to_json(build_heartbeat(1, 0, STATE_STALLED,
                                       counters={"published": 3, "frames": 9}))
        back = json.loads(text)
        assert back["state"] == "stalled"
        assert back["counters"]["frames"] == 9

    def test_counters_are_plain_ints(self):
        hb = build_heartbeat(1, 0, STATE_OK,
                             counters={"published": np.int64(7)})
        json.loads(to_json(hb))
        assert isinstance(hb["counters"]["published"], int)


class TestUnknownReasonCounters:
    """★ 노드가 `unknown_depth`의 **사유**를 status로 실어 보냅니다 (2026-09-05).

    `unknown_depth`는 "몇 건"만 말합니다. 대응이 갈리므로 **왜**가 필요합니다:

      no_valid_pixels   띠가 통째로 무효 -> 띠 위치(`band_offset`)를 옮겨야 함
      low_valid_ratio   픽셀은 있는데 비율 미달 -> `min_valid_ratio` 문제
      box_outside_image 띠가 화면 밖 -> 화각/카메라 각도 문제

    노드는 `reason_` 접두사로 **평평하게** 실어 보냅니다. 아래 테스트가
    그 선택의 이유(중첩 dict는 애초에 담기지 않음)를 못 박습니다.
    """

    def test_사유별_카운터가_실린다(self):
        hb = build_heartbeat(1, 0, STATE_OK,
                             counters={"unknown_depth": 150,
                                       "reason_no_valid_pixels": 142,
                                       "reason_low_valid_ratio": 8})
        back = json.loads(to_json(hb))
        assert back["counters"]["reason_no_valid_pixels"] == 142
        # 합이 unknown_depth와 맞아야 받는 쪽이 누락을 알아챌 수 있습니다
        assert (back["counters"]["reason_no_valid_pixels"]
                + back["counters"]["reason_low_valid_ratio"]
                == back["counters"]["unknown_depth"])

    def test_중첩_dict는_담기지_않는다(self):
        """★ 노드가 평평하게 펴는 이유. 중첩을 넣으면 하트비트 타이머가
        매번 터져 status가 통째로 끊깁니다 — 조용히 틀리는 게 아니라
        시끄럽게 죽지만, 죽는 위치가 진단 코드라 더 나쁩니다."""
        with pytest.raises(TypeError):
            build_heartbeat(1, 0, STATE_OK,
                            counters={"reasons": {"no_valid_pixels": 3}})

    def test_사유가_없으면_키도_없다(self):
        """정상 동작 중에는 카운터가 늘지 않아야 합니다."""
        hb = build_heartbeat(1, 0, STATE_OK, counters={"unknown_depth": 0})
        assert [k for k in hb["counters"] if k.startswith("reason_")] == []


class TestStampAge:
    def test_age_across_second_boundary(self):
        age = stamp_age_sec((11, 100000000), (10, 900000000))
        assert age == pytest.approx(0.2)

    def test_no_last_frame_is_none(self):
        assert stamp_age_sec((10, 0), None) is None

    def test_age_is_local_math_not_the_contract_stamp(self):
        """국소 계산이라 float으로 합쳐도 됩니다 — 발행 stamp와 헷갈리지 말 것."""
        assert stamp_age_sec((1786329610, 0), (1786329609, 0)) == pytest.approx(1.0)
