#!/usr/bin/env python3
"""
태스크② 더미 장면 눈으로 확인하기 — **ROS 없이** 돕니다.

`fake_detection_node`가 발행하는 것과 **같은 장면**(`depth.dummy_scene`)을 만들어
전체 사슬을 돌리고, 결과를 그림 한 장으로 저장합니다.

    컬러 박스 ──▶ [뎁스 좌표계로 투영] ──▶ [대표 거리] ──▶ [역투영] ──▶ base_link

그림에 세 가지를 겹쳐 그립니다. **빨강과 초록이 어긋나는 것**이 이 카메라의
핵심 함정입니다 (컬러 16:9 / 뎁스 4:3이라 화각이 다름).

    초록  project_box()  — K를 거친 올바른 위치
    빨강  scale_box()    — 해상도 배율. 화면 중앙만 맞고 위아래로 갈수록 어긋남
    노랑  실제로 거리를 뽑은 픽셀 (박스 중앙 central 비율)

사용:
  python3 tools/preview_depth_scene.py
  python3 tools/preview_depth_scene.py --box-y 60 -o out/     # 위쪽일수록 크게 어긋남
  python3 tools/preview_depth_scene.py --flame-hole           # 화염 위 뎁스 무효 재현
  python3 tools/preview_depth_scene.py --distance 2.0         # 카메라에 유리한 거리
  python3 tools/preview_depth_scene.py --floor 0.35 --flame-hole   # 5-1 폴백 비교 ★

`--floor H`(카메라의 바닥 위 높이)를 주면 배경이 평면 벽이 아니라 **바닥 + 벽**이
되고, 불은 바닥에 놓입니다. 그래야 5-1의 폴백 후보가 서로 갈립니다 — 평면 벽만
있는 장면에서는 셋 다 같은 벽을 재서 비교가 안 됩니다.
"""

from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from image_pipeline.depth import (  # noqa: E402
    DEFAULT_Z_MAX,
    DEFAULT_Z_MIN,
    backproject,
    box_center,
    clip_box,
    dummy_scene,
    optical_to_base_link_matrix,
    project_box,
    sample_distance_detail,
    scale_box,
    to_base_link,
    to_meters,
)

GREEN, RED, YELLOW, WHITE = (0, 220, 0), (0, 0, 255), (0, 220, 255), (255, 255, 255)


def colorize(depth_m: np.ndarray):
    """뎁스를 보기용 컬러맵으로. 무효(NaN) 픽셀은 **검정**으로 남깁니다.

    범위는 스펙(0.2~4m)이 아니라 **실제 담긴 값**에 맞춥니다. 3.2m와 3.8m처럼
    가까운 두 면이 같은 색으로 뭉개지면 그림이 아무것도 안 알려줍니다.
    """
    valid = np.isfinite(depth_m)
    if not valid.any():
        return np.zeros((*depth_m.shape, 3), dtype=np.uint8), (0.0, 0.0)
    lo, hi = float(np.min(depth_m[valid])), float(np.max(depth_m[valid]))
    pad = max((hi - lo) * 0.25, 0.05)
    lo, hi = lo - pad, hi + pad

    norm = np.zeros(depth_m.shape, dtype=np.uint8)
    norm[valid] = np.clip((depth_m[valid] - lo) / (hi - lo) * 255.0, 0, 255).astype(np.uint8)
    out = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
    out[~valid] = (0, 0, 0)          # 구멍은 까맣게 — 눈에 띄어야 합니다
    return out, (lo, hi)


def draw_box(img, box, color, label=None, thickness=2):
    clipped = clip_box(box, img.shape[1], img.shape[0])
    if clipped is None:
        return
    x1, y1, x2, y2 = clipped
    cv2.rectangle(img, (x1, y1), (x2 - 1, y2 - 1), color, thickness)
    if label:
        y = y1 - 6 if y1 > 16 else y2 + 16
        cv2.putText(img, label, (x1, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)


def panel(img, title, height):
    """패널 위에 제목줄을 붙이고 높이를 맞춥니다."""
    bar = np.zeros((26, img.shape[1], 3), dtype=np.uint8)
    cv2.putText(bar, title, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 1, cv2.LINE_AA)
    out = np.vstack([bar, img])
    if out.shape[0] < height:
        pad = np.zeros((height - out.shape[0], out.shape[1], 3), dtype=np.uint8)
        out = np.vstack([out, pad])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--out", default="local_check", help="출력 폴더")
    ap.add_argument("--distance", type=float, default=3.2, help="목표 거리 m (시나리오 3.2)")
    ap.add_argument("--background", type=float, default=3.8, help="배경 거리 m (스펙 상한 4m 안쪽)")
    ap.add_argument("--box-x", type=float, default=-1.0, help="박스 중심 x (컬러 px, 음수=중앙)")
    ap.add_argument("--box-y", type=float, default=-1.0, help="박스 중심 y (컬러 px, 음수=중앙)")
    ap.add_argument("--box-size", type=float, default=80.0, help="박스 한 변 (컬러 px)")
    ap.add_argument("--flame-hole", action="store_true", help="박스 영역 뎁스를 0으로 (지시서 5-1)")
    ap.add_argument("--noise", type=float, default=0.0, help="뎁스 노이즈 σ (m)")
    ap.add_argument("--region", default="center",
                    choices=("center", "bottom", "below", "ring"))
    ap.add_argument("--method", default="median",
                    choices=("median", "min", "p25", "p75", "max"))
    ap.add_argument("--floor", type=float, default=None, metavar="H",
                    help="카메라의 바닥 위 높이 m. 주면 바닥+벽 장면 (5-1 비교용)")
    ap.add_argument("--band", type=float, default=0.15,
                    help="below 띠 두께 (박스 높이 대비 비율)")
    ap.add_argument("--central", type=float, default=0.5, help="박스 중앙 사용 비율")
    ap.add_argument("--camera-offset", type=float, nargs=3, default=[0.1, 0.0, 0.35],
                    metavar=("X", "Y", "Z"), help="base_link 기준 카메라 위치 (더미용)")
    args = ap.parse_args()

    cw, ch = 640, 360
    sc = dummy_scene(
        color_size=(cw, ch), depth_size=(640, 480),
        box_size=(args.box_size, args.box_size),
        box_center_xy=(args.box_x if args.box_x >= 0 else cw / 2.0,
                       args.box_y if args.box_y >= 0 else ch / 2.0),
        distance_m=args.distance, background_m=args.background,
        flame_hole=args.flame_hole, noise_m=args.noise,
        floor_height_m=args.floor, box_on_floor=args.floor is not None,
    )

    depth_raw = sc.depth_image(seed=0)
    depth_m = to_meters(depth_raw)

    box_right = project_box(sc.box_color, sc.k_color, sc.k_depth)
    sx, sy = sc.depth_size[0] / cw, sc.depth_size[1] / ch
    box_wrong = scale_box(sc.box_color, sx, sy)

    opts = dict(region=args.region, central=args.central,
                method=args.method, band_ratio=args.band)
    res_right = sample_distance_detail(depth_raw, box_right, **opts)
    res_wrong = sample_distance_detail(depth_raw, box_wrong, **opts)

    # ---------------------------------------------------------------- 그림
    color_view = np.full((ch, cw, 3), 40, dtype=np.uint8)
    draw_box(color_view, sc.box_color, GREEN, "YOLO box (color frame)")
    cv2.drawMarker(color_view, tuple(int(v) for v in box_center(sc.box_color)),
                   GREEN, cv2.MARKER_CROSS, 14, 1)

    depth_view, (vlo, vhi) = colorize(depth_m)
    draw_box(depth_view, box_wrong, RED, "scale_box (WRONG)")
    draw_box(depth_view, box_right, GREEN, "project_box (right)")
    # 실제로 거리를 뽑은 픽셀 (노랑)
    if args.region == "below":
        x1, y1, x2, y2 = box_right
        band = max(2.0, (y2 - y1) * args.band)
        cx = (x1 + x2) / 2.0
        hw = (x2 - x1) * args.central / 2.0
        draw_box(depth_view, (cx - hw, y2, cx + hw, y2 + band), YELLOW, "below band", 1)
    elif args.region == "center":
        c = box_center(box_right)
        w = (box_right[2] - box_right[0]) * args.central / 2
        h = (box_right[3] - box_right[1]) * args.central / 2
        draw_box(depth_view, (c[0] - w, c[1] - h, c[0] + w, c[1] + h), YELLOW, None, 1)

    height = max(color_view.shape[0], depth_view.shape[0]) + 26
    canvas = np.hstack([panel(color_view, f"COLOR {cw}x{ch} (16:9)", height),
                        panel(depth_view,
                              f"DEPTH {sc.depth_size[0]}x{sc.depth_size[1]} (4:3)   "
                              f"colormap {vlo:.2f}~{vhi:.2f}m  (black = no data)", height)])

    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, "depth_scene.png")
    cv2.imwrite(path, canvas)

    # ---------------------------------------------------------------- 숫자
    u, v = box_center(sc.box_color)
    print(f"\n장면: 컬러 {cw}x{ch}(16:9) / 뎁스 {sc.depth_size[0]}x{sc.depth_size[1]}(4:3)"
          f" | 유효범위 {DEFAULT_Z_MIN}~{DEFAULT_Z_MAX}m")
    print(f"박스(컬러) 중심 ({u:.0f}, {v:.0f})  목표 {args.distance}m  배경 {args.background}m"
          + ("  [화염: 뎁스 무효]" if args.flame_hole else ""))
    print(f"\n  {'':22} {'중심(뎁스)':>16} {'거리':>9} {'유효':>7} {'폭(IQR)':>9}  사유")
    for name, box, res in (("project_box (정답)", box_right, res_right),
                           ("scale_box   (함정)", box_wrong, res_wrong)):
        c = box_center(box)
        d = f"{res.distance:.3f}m" if res.distance is not None else "  없음"
        print(f"  {name:22} ({c[0]:6.1f},{c[1]:6.1f}) {d:>9} {res.valid_ratio:6.0%} "
              f"{res.spread:8.3f}m  {res.reason}")

    err_px = abs(box_center(box_right)[1] - box_center(box_wrong)[1])
    print(f"\n  두 방식의 세로 차이: {err_px:.1f} px"
          f"  ->  {args.distance}m에서 {err_px * args.distance / sc.k_depth[4]:.3f} m")

    # ★ 2026-08-11: 우리는 base_link 좌표를 **발행하지 않습니다** — 메인이 계산합니다
    #   (HANDOVER 7-3). 여기 값은 "메인이 우리 JSON으로 계산해야 할 값"이고,
    #   메인 결과가 틀렸을 때 우리 거리가 문제인지 메인 역투영이 문제인지 가릅니다.
    print("\n  메인이 계산해야 할 좌표 (base_link, m) — 우리 발행분 아님:")
    truth = sc.expected_base_link(args.camera_offset)
    unknown = '거리 불명 -> depth: null, depth_status: "unknown"'
    print(f"    정답            : {unknown if truth is None else tuple(round(x, 3) for x in truth)}")
    for name, res in (("project_box 결과", res_right), ("scale_box   결과", res_wrong)):
        if res.distance is None:
            print(f"    {name}: {unknown}")
            continue
        base = to_base_link(backproject(u, v, res.distance, sc.k_color),
                            optical_to_base_link_matrix(args.camera_offset))
        if truth is None:
            # 대상의 뎁스가 비었는데 값이 나왔다 = 폴백이 **주변**을 잰 것입니다.
            # 주변은 대개 대상보다 뒤라 체계적으로 뒤로 편향됩니다. "성공"이 아닙니다.
            mark = f"  <- {args.region} 폴백이 잰 주변 거리. 대상 거리가 아닙니다"
        else:
            err = max(abs(a - b) for a, b in zip(base, truth))
            mark = "  OK" if err < 0.01 else f"  <- 오차 {err:.3f} m"
        print(f"    {name}: {tuple(round(x, 3) for x in base)}{mark}")

    # ------------------------------------------------- 5-1 폴백 비교 (바닥 있을 때만)
    if args.floor is not None:
        print("\n  [5-1] 화염 위 뎁스가 비었을 때의 폴백 후보"
              f"  (정답 = 대상 거리 {args.distance}m)")
        print(f"    {'region':8}{'method':8}{'거리':>9}{'유효':>7}{'편향':>10}  사유")
        for reg, meth in (("center", "median"), ("bottom", "median"),
                          ("below", "median"), ("below", "p75"), ("below", "max"),
                          ("ring", "median")):
            r = sample_distance_detail(depth_raw, box_right, region=reg, method=meth,
                                       central=args.central, band_ratio=args.band)
            if r.distance is None:
                print(f"    {reg:8}{meth:8}{'없음':>9}{r.valid_ratio:6.0%}"
                      f"{'—':>10}  {r.reason}")
            else:
                bias = r.distance - args.distance
                tag = "가깝게" if bias < -0.02 else ("멀게" if bias > 0.02 else "")
                print(f"    {reg:8}{meth:8}{r.distance:8.3f}m{r.valid_ratio:6.0%}"
                      f"{bias:+9.3f}m  {r.reason} {tag}")
        print("    * 'ok' 는 '맞다'가 아니라 '픽셀은 충분했다'는 뜻입니다.")
        print("      below 는 바닥이 아래로 갈수록 가까워져 **가깝게**, "
              "ring 은 주변이 대상 뒤라 **멀게** 잡습니다.")

    print(f"\n  그림: {path}\n")


if __name__ == "__main__":
    main()
