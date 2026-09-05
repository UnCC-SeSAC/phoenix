from __future__ import annotations

from image_pipeline.detection_msgs import box_from_bbox, hypothesis

def _clamp01(value: float) -> float:
    return min(1.0, max(0.0, float(value)))


def _class_name(class_id, class_names) -> str:
    """숫자 class_id를 이름으로. 매핑이 없거나 범위를 벗어나면 원본 그대로."""
    label = str(class_id)
    if not class_names:
        return label
    try:
        index = int(label)
    except (TypeError, ValueError):
        return label                      # 이미 이름인 4.x 경로
    if 0 <= index < len(class_names):
        return str(class_names[index])
    return label                          # ★ 지어내지 않습니다


def stream_size(src_w, src_h, max_width):
    src_w, src_h = int(src_w), int(src_h)
    if src_w <= 0 or src_h <= 0:
        return (0, 0)
    max_width = int(max_width)
    if max_width <= 0 or src_w <= max_width:
        return (src_w, src_h)
    return (max_width, max(1, round(src_h * max_width / src_w)))


def normalize_boxes(detections, src_w, src_h, class_names=None):
    src_w, src_h = float(src_w), float(src_h)
    if src_w <= 0.0 or src_h <= 0.0:
        return []

    boxes = []
    for det in detections or ():
        class_id, confidence = ("", 0.0)
        results = getattr(det, "results", None)
        if results:
            class_id, confidence = hypothesis(results[0])

        x1, y1, x2, y2 = box_from_bbox(det.bbox)
        x1, x2 = _clamp01(x1 / src_w), _clamp01(x2 / src_w)
        y1, y2 = _clamp01(y1 / src_h), _clamp01(y2 / src_h)
        w, h = x2 - x1, y2 - y1
        if w <= 0.0 or h <= 0.0:
            continue                      # 프레임 밖으로 완전히 나간 박스

        boxes.append({
            "class_name": _class_name(class_id, class_names),
            "confidence": round(float(confidence), 4),
            "cx": x1 + w / 2.0,
            "cy": y1 + h / 2.0,
            "w": w,
            "h": h,
        })
    return boxes


def throttle(last_emit_t, now, fps):
    if fps <= 0:
        return True
    if last_emit_t is None:
        return True
    return (float(now) - float(last_emit_t)) >= 1.0 / float(fps)