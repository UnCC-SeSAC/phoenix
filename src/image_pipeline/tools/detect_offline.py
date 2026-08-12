#!/usr/bin/env python3
"""
YOLO 오프라인 검출 — **ROS 없이** 모델을 사진에 돌려보는 도구.

모델을 받은 날 **가장 먼저** 이걸 돌리세요. 노드에서 처음 확인하면
"검출이 안 나온다"의 원인이 모델인지·전처리인지·레이아웃 파싱인지·토픽
배선인지 구분이 안 됩니다.

    # 1) 출력 텐서 모양부터 눈으로 확인 (레이아웃 합의용)
    python3 tools/detect_offline.py --model models/fire_yolo26s.onnx --describe

    # 2) 사진에 돌려서 박스 그리기
    python3 tools/detect_offline.py --model models/fire_yolo26s.onnx \\
        smoke01.jpg --names fire,person -o out/

    # 3) 태스크①을 거친 영상으로 (실제 노드와 같은 입력)
    python3 tools/detect_offline.py --model ... smoke01.jpg --preprocess full

    # 4) 속도만 (RPi5에서 imgsz 정할 때)
    python3 tools/detect_offline.py --model ... smoke01.jpg --bench 30
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from image_pipeline.intrinsics import fit_size  # noqa: E402
from image_pipeline.yolo import describe_outputs, make_detector  # noqa: E402

COLORS = [(0, 200, 255), (0, 255, 120), (255, 120, 0), (200, 0, 255)]


def draw(img, dets):
    out = img.copy()
    for d in dets:
        x1, y1, x2, y2 = (int(round(v)) for v in d.box)
        color = COLORS[d.class_id % len(COLORS)]
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        label = f"{d.class_name} {d.score:.2f}"
        cv2.putText(out, label, (x1, max(12, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        # 중심점 — 태스크②가 뎁스를 샘플링하는 자리입니다.
        u, v = d.center()
        cv2.drawMarker(out, (int(round(u)), int(round(v))), color,
                       cv2.MARKER_CROSS, 12, 2)
    return out


def main():
    ap = argparse.ArgumentParser(description="YOLO 오프라인 검출")
    ap.add_argument("images", nargs="*", help="입력 이미지")
    ap.add_argument("--model", default="", help=".onnx | .pt | .hef")
    ap.add_argument("--stub", action="store_true",
                    help="가중치 없이 배선만 확인. 모델 입력 정중앙에 박스 하나를 "
                         "놓으므로 결과가 **원본 이미지 정중앙**이어야 정상")
    ap.add_argument("--names", default="",
                    help="쉼표 구분. ★ 학습 때 순서 그대로 (예: fire,person)")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--iou", type=float, default=0.45)
    ap.add_argument("--layout", default="auto", choices=("auto", "v8", "end2end"))
    ap.add_argument("--threads", type=int, default=0)
    ap.add_argument("-o", "--out", default="", help="박스 그린 이미지 저장 폴더")
    ap.add_argument("--describe", action="store_true",
                    help="추론 없이 출력 텐서 shape만 보고 종료")
    ap.add_argument("--bench", type=int, default=0, help="반복 횟수(속도 측정)")
    ap.add_argument("--preprocess", default="",
                    help="태스크① 적용: full|dehaze|clahe|gamma (기본: 원본 그대로)")
    ap.add_argument("--process-width", type=int, default=640,
                    help="--preprocess 사용 시 축소 폭 (노드의 process_width)")
    args = ap.parse_args()

    if not args.model and not args.stub:
        ap.error("--model 을 주거나, 가중치가 없으면 --stub 으로 배선만 확인하세요")

    if args.describe:
        print(describe_outputs(args.model, args.imgsz))
        return 0

    if not args.images:
        ap.error("이미지를 하나 이상 주거나 --describe 를 쓰세요")

    names = [n.strip() for n in args.names.split(",") if n.strip()]
    if not names:
        print("※ --names 가 없습니다. class_name 이 번호로 나옵니다 "
              "(노드에서도 같습니다).")

    det = make_detector(args.model, backend="stub" if args.stub else "auto",
                        imgsz=args.imgsz, conf=args.conf,
                        iou=args.iou, class_names=names, layout=args.layout,
                        threads=args.threads)
    if args.stub:
        print("=== 스텁 모드 — 가중치를 읽지 않습니다 ===")
        print("모델 입력 정중앙에 박스 하나를 놓습니다. 레터박스를 제대로 되돌렸다면")
        print("결과 center가 **원본 이미지의 정중앙**이어야 합니다.")
        print("되돌리기를 빠뜨리면 640x480에서 y가 240이 아니라 320으로 나옵니다.\n")
    else:
        print(f"모델 {args.model} | imgsz={args.imgsz} "
              f"| 레이아웃={det.detected_layout}")

    pipe = None
    if args.preprocess:
        from image_pipeline.pipeline import Pipeline
        pipe = Pipeline(mode=args.preprocess)

    if args.out:
        os.makedirs(args.out, exist_ok=True)

    for path in args.images:
        img = cv2.imread(path)
        if img is None:
            print(f"  [건너뜀] 읽기 실패: {path}")
            continue

        if pipe is not None:
            # 노드와 같은 순서: 축소 -> 전처리. 순서를 바꾸면 속도가 달라집니다.
            h, w = img.shape[:2]
            nw, nh, _, _ = fit_size(w, h, args.process_width)
            if (nw, nh) != (w, h):
                img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA)
            img = pipe.process(img)

        dets = det.detect(img)
        t = det.timings
        print(f"\n{os.path.basename(path)}  {img.shape[1]}x{img.shape[0]} "
              f"-> 검출 {len(dets)}개 | 전처리 {t['pre']:.1f}ms "
              f"추론 {t['infer']:.1f}ms 후처리 {t['post']:.1f}ms")
        for d in dets:
            u, v = d.center()
            mark = ""
            if args.stub:
                want = (img.shape[1] / 2.0, img.shape[0] / 2.0)
                ok = abs(u - want[0]) < 1.0 and abs(v - want[1]) < 1.0
                mark = ("   <- 정중앙 일치 (정상)" if ok else
                        f"   <- ★ 정중앙({want[0]:.0f}, {want[1]:.0f})과 다릅니다")
            print(f"    {d.class_name:<10s} {d.score:.3f}  "
                  f"box=({d.box[0]:.0f},{d.box[1]:.0f})-({d.box[2]:.0f},{d.box[3]:.0f})  "
                  f"center=({u:.0f}, {v:.0f}){mark or '  <- 태스크②가 뎁스를 재는 자리'}")
        if not dets:
            print("    (없음) conf를 낮춰보세요. 그래도 없으면 --describe 로 "
                  "출력 레이아웃부터 확인하세요")

        if args.bench:
            times = []
            for _ in range(args.bench):
                t0 = time.perf_counter()
                det.detect(img)
                times.append((time.perf_counter() - t0) * 1000.0)
            arr = np.array(times)
            budget = 1000.0 / 15.0     # 드라이버 실측 15fps
            mark = "" if arr.mean() <= budget else f"   <- 15fps 미달 ({budget:.0f}ms)"
            print(f"    [bench] {args.bench}회 평균 {arr.mean():.1f}ms "
                  f"(p95 {np.percentile(arr, 95):.1f}ms){mark}")

        if args.out:
            dst = os.path.join(args.out, os.path.basename(path))
            cv2.imwrite(dst, draw(img, dets))
            print(f"    저장: {dst}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
