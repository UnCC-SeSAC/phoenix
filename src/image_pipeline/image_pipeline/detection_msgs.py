#!/usr/bin/env python3
"""
`vision_msgs` 검출 메시지 읽기/쓰기 — **ROS를 import하지 않습니다.**

전부 덕 타이핑이라 rclpy 없이 pytest로 검증됩니다. 왜 굳이 모듈로 뺐느냐면,
`vision_msgs`가 **버전마다 필드 모양이 달라서** 코드가 조용히 깨지기 때문입니다.

    vision_msgs 4.x (Humble 이후)          vision_msgs 3.x (Foxy/Galactic)
    ────────────────────────────────       ─────────────────────────────────
    bbox.center.position.x / .y            bbox.center.x / .y
    bbox.center.theta                      bbox.center.theta
    result.hypothesis.class_id (str)       result.id (int64)
    result.hypothesis.score                result.score

로봇(팀원5 Hailo 배포)과 개발 PC의 배포판이 다를 수 있고, 필드가 없으면
`AttributeError`로 콜백이 죽는 게 아니라 **예외가 잡혀서 검출이 통째로 사라지는**
형태로 나타나기 쉽습니다. 그래서 여기서 한 번에 흡수하고 테스트로 잠급니다.

박스 표현은 `depth.py`와 같은 `(x1, y1, x2, y2)` 픽셀 튜플입니다.
"""

from __future__ import annotations

from image_pipeline.depth import box_from_center


def bbox_center(bbox) -> tuple[float, float]:
    """`vision_msgs/BoundingBox2D`의 중심 (u, v). 두 버전 모두 지원."""
    center = bbox.center
    pos = getattr(center, "position", None)
    if pos is not None:                     # vision_msgs 4.x
        return (float(pos.x), float(pos.y))
    return (float(center.x), float(center.y))  # vision_msgs 3.x


def set_bbox_center(bbox, u: float, v: float) -> None:
    """`bbox.center`를 버전에 맞게 채웁니다."""
    center = bbox.center
    pos = getattr(center, "position", None)
    if pos is not None:
        pos.x, pos.y = float(u), float(v)
    else:
        center.x, center.y = float(u), float(v)


def box_from_bbox(bbox) -> tuple[float, float, float, float]:
    """`BoundingBox2D` -> `(x1, y1, x2, y2)`.

    ★ `size_x`/`size_y`는 **크기**지 우하단 좌표가 아닙니다. 그대로 x2로 쓰면
    박스가 화면 밖으로 나가고, 그러면 뎁스 샘플이 배경만 읽습니다.
    """
    u, v = bbox_center(bbox)
    return box_from_center(u, v, float(bbox.size_x), float(bbox.size_y))


def hypothesis(result) -> tuple[str, float]:
    """`ObjectHypothesisWithPose` -> `(class_id, score)`. 두 버전 모두 지원.

    3.x의 `id`는 int64라 문자열로 바꿔 돌려줍니다 — 상위 코드가 한 가지
    타입만 다루게 하려는 것입니다.
    """
    hyp = getattr(result, "hypothesis", None)
    if hyp is not None:                     # vision_msgs 4.x
        return (str(hyp.class_id), float(hyp.score))
    return (str(result.id), float(result.score))  # vision_msgs 3.x


def set_hypothesis(result, class_id: str, score: float) -> None:
    """`ObjectHypothesisWithPose`를 버전에 맞게 채웁니다."""
    hyp = getattr(result, "hypothesis", None)
    if hyp is not None:
        hyp.class_id, hyp.score = str(class_id), float(score)
    else:
        try:
            result.id = int(class_id)
        except (TypeError, ValueError):
            # 3.x의 id는 int64라 "fire" 같은 문자열을 못 담습니다.
            # 클래스 이름↔번호 매핑은 팀원5와 맞춰야 하는 값입니다 (지시서 5-3).
            raise ValueError(
                f"vision_msgs 3.x의 id는 정수입니다 — class_id={class_id!r}를 "
                "번호로 매핑하세요 (YOLO 클래스 순서를 팀원5와 확인)"
            )
        result.score = float(score)


def best_score(detection) -> float:
    """검출의 최고 신뢰도. `results`가 비어 있으면 0.0."""
    scores = [hypothesis(r)[1] for r in getattr(detection, "results", [])]
    return max(scores) if scores else 0.0


def best_class(detection) -> str:
    """최고 신뢰도 가설의 class_id. 없으면 빈 문자열."""
    best, label = None, ""
    for r in getattr(detection, "results", []):
        cid, score = hypothesis(r)
        if best is None or score > best:
            best, label = score, cid
    return label
