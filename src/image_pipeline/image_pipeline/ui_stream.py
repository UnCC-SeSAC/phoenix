from __future__ import annotations

from image_pipeline.detection_msgs import box_from_bbox, hypothesis


def _clamp01(value: float) -> float:
    return min(1.0, max(0.0, float(value)))


def _class_name(class_id, class_names) -> str:
    label = str(class_id)
    if not class_names:
        return label
    try:
        index = int(label)
    except (TypeError, ValueError):
        return label
    return str(class_names[index]) if 0 <= index < len(class_names) else label


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
    for detection in detections or ():
        class_id, confidence = ("", 0.0)
        results = getattr(detection, "results", None)
        if results:
            class_id, confidence = hypothesis(results[0])
        x1, y1, x2, y2 = box_from_bbox(detection.bbox)
        x1, x2 = _clamp01(x1 / src_w), _clamp01(x2 / src_w)
        y1, y2 = _clamp01(y1 / src_h), _clamp01(y2 / src_h)
        width, height = x2 - x1, y2 - y1
        if width <= 0.0 or height <= 0.0:
            continue
        boxes.append({
            "class_name": _class_name(class_id, class_names),
            "confidence": round(float(confidence), 4),
            "cx": x1 + width / 2.0,
            "cy": y1 + height / 2.0,
            "w": width,
            "h": height,
        })
    return boxes


def throttle(last_emit_t, now, fps):
    if fps <= 0 or last_emit_t is None:
        return True
    return (float(now) - float(last_emit_t)) >= 1.0 / float(fps)
