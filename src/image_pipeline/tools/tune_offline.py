#!/usr/bin/env python3
"""
로봇 없이 파라미터를 잡기 위한 오프라인 튜닝 도구.

ROS 없이 OpenCV/NumPy만 있으면 돌아갑니다. 로봇 수령 전에
인센스 연기 사진 몇 장으로 clipLimit / omega / t0 감을 잡아두세요.

사용 예)
  # 4조건 비교 이미지 저장
  python3 tune_offline.py smoke1.jpg smoke2.jpg -o out/

  # clipLimit 스윕
  python3 tune_offline.py smoke1.jpg --sweep clahe_clip 1.0 2.0 3.0 4.0 -o out/

  # 처리 시간 벤치마크 (해상도별) — RPi5에서 돌리면 그대로 발표 근거
  python3 tune_offline.py smoke1.jpg --bench

  # 투과율 맵 저장 (발표 그림용)
  python3 tune_offline.py smoke1.jpg --dump-transmission -o out/
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from image_pipeline.dehaze import ClaheEnhancer, DarkChannelDehazer  # noqa: E402
from image_pipeline.pipeline import Pipeline  # noqa: E402


def build(args):
    clahe = ClaheEnhancer(args.clahe_clip, (args.clahe_tile, args.clahe_tile))
    dehazer = DarkChannelDehazer(
        omega=args.omega,
        t0=args.t0,
        patch=args.patch,
        scale=args.scale,
        use_guided=not args.no_guided,
        guided_radius=args.guided_radius,
        a_max=args.a_max,
        sky_ratio=args.sky_ratio,
    )
    return clahe, dehazer


def conditions(img, clahe, dehazer, gamma=1.0):
    """(라벨, 결과, 소요ms) 4조건.

    노드와 **같은 Pipeline**을 씁니다. 여기서 좋게 나온 파라미터가
    로봇에서 다르게 동작하면 안 되므로 코드 경로를 일치시켰습니다.
    """
    # 워밍업 — 첫 호출은 OpenCV 초기화와 버퍼 할당이 섞여 수십 ms 뻥튀기됩니다.
    # 반드시 실제 해상도로 돌려야 의미가 있습니다.
    for _ in range(2):
        clahe.process(img)
        dehazer.process(img)

    out = [("original", img.copy(), 0.0)]
    for mode, label_ in (("clahe", "clahe"),
                         ("dehaze", "dehaze"),
                         ("full", "dehaze+clahe")):
        pipe = Pipeline(mode=mode, gamma=gamma, clahe=clahe, dehazer=dehazer)
        res = pipe.process(img)
        out.append((label_, res, pipe.timings["total"]))
    return out


def label(img, text):
    out = img.copy()
    cv2.rectangle(out, (0, 0), (out.shape[1], 28), (0, 0, 0), -1)
    cv2.putText(out, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
                cv2.LINE_AA)
    return out


def grid(items, cols=2):
    rows = []
    for i in range(0, len(items), cols):
        chunk = [label(im, f"{name}  {ms:.1f}ms" if ms else name)
                 for name, im, ms in items[i:i + cols]]
        while len(chunk) < cols:
            chunk.append(np.zeros_like(chunk[0]))
        rows.append(np.hstack(chunk))
    return np.vstack(rows)


def metrics(img):
    """정량 지표 (발표 표에 넣기 좋은 값들).

    - contrast : 그레이스케일 표준편차 (전역 대비)
    - entropy  : 히스토그램 엔트로피 (정보량. 뿌연 영상일수록 낮음)
    - sharpness: 라플라시안 분산 (경계 선명도)
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    hist = cv2.calcHist([gray], [0], None, [256], [0, 256]).ravel()
    p = hist / max(hist.sum(), 1)
    p = p[p > 0]
    return {
        "contrast": float(gray.std()),
        "entropy": float(-(p * np.log2(p)).sum()),
        "sharpness": float(cv2.Laplacian(gray, cv2.CV_64F).var()),
    }


def cmd_compare(paths, args):
    clahe, dehazer = build(args)
    os.makedirs(args.out, exist_ok=True)

    for path in paths:
        img = cv2.imread(path)
        if img is None:
            print(f"[skip] 읽기 실패: {path}")
            continue
        if args.width and img.shape[1] > args.width:
            h = int(img.shape[0] * args.width / img.shape[1])
            img = cv2.resize(img, (args.width, h), interpolation=cv2.INTER_AREA)

        items = conditions(img, clahe, dehazer, args.gamma)

        base = os.path.splitext(os.path.basename(path))[0]
        cv2.imwrite(os.path.join(args.out, f"{base}_compare.png"), grid(items))

        print(f"\n== {base} ({img.shape[1]}x{img.shape[0]}) ==")
        print(f"{'조건':<14}{'ms':>8}{'contrast':>11}{'entropy':>10}{'sharpness':>12}")
        for name, im, ms in items:
            m = metrics(im)
            print(f"{name:<14}{ms:>8.1f}{m['contrast']:>11.2f}"
                  f"{m['entropy']:>10.3f}{m['sharpness']:>12.1f}")

        if args.dump_transmission:
            dehazer.process(img)
            t = dehazer.last_transmission
            tv = cv2.applyColorMap((t * 255).astype(np.uint8), cv2.COLORMAP_INFERNO)
            cv2.imwrite(os.path.join(args.out, f"{base}_transmission.png"), tv)
            a = dehazer.last_a.reshape(3)
            print(f"대기광 A(BGR) = {a.round(3)}   "
                  f"(1.0에 가까우면 화염/포화 픽셀에 끌렸을 가능성 -> a_max 낮추기)")

    print(f"\n결과 이미지: {args.out}/")


def cmd_sweep(paths, args):
    key, values = args.sweep[0], [float(v) for v in args.sweep[1:]]
    os.makedirs(args.out, exist_ok=True)

    for path in paths:
        img = cv2.imread(path)
        if img is None:
            continue
        if args.width and img.shape[1] > args.width:
            h = int(img.shape[0] * args.width / img.shape[1])
            img = cv2.resize(img, (args.width, h), interpolation=cv2.INTER_AREA)

        items = [("original", img, 0.0)]
        for v in values:
            clahe, dehazer = build(args)
            if key == "clahe_clip":
                clahe.update(v, clahe.tile_grid)
            elif key == "omega":
                dehazer.omega = v
            elif key == "t0":
                dehazer.t0 = v
            elif key == "scale":
                dehazer.scale = v
            else:
                sys.exit(f"지원하지 않는 스윕 키: {key} (clahe_clip|omega|t0|scale)")
            pipe = Pipeline(mode="full", gamma=args.gamma,
                            clahe=clahe, dehazer=dehazer)
            res = pipe.process(img)
            items.append((f"{key}={v:g}", res, pipe.timings["total"]))

        base = os.path.splitext(os.path.basename(path))[0]
        cv2.imwrite(os.path.join(args.out, f"{base}_sweep_{key}.png"),
                    grid(items, cols=3))
        print(f"저장: {args.out}/{base}_sweep_{key}.png")


def cmd_bench(paths, args):
    """해상도별 처리 시간. RPi5에서 돌린 값이 곧 process_width 결정 근거."""
    budget_ms = 1000.0 / args.target_fps
    img = cv2.imread(paths[0])
    if img is None:
        sys.exit("이미지 읽기 실패")

    clahe, dehazer = build(args)
    print(f"{'해상도':<14}{'CLAHE':>10}{'디헤이즈':>12}{'합계':>10}{'최대fps':>10}")
    for w in (320, 424, 640, 848, 1280, 1920):
        h = int(img.shape[0] * w / img.shape[1])
        im = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)

        for _ in range(2):  # 워밍업
            clahe.process(im); dehazer.process(im)

        n = args.bench_iters
        t = time.perf_counter()
        for _ in range(n):
            clahe.process(im)
        tc = (time.perf_counter() - t) * 1000 / n

        t = time.perf_counter()
        for _ in range(n):
            dehazer.process(im)
        td = (time.perf_counter() - t) * 1000 / n

        total = tc + td
        mark = "" if total <= budget_ms else f"   <- {args.target_fps:g}fps 미달"
        print(f"{w}x{h:<8}{tc:>10.2f}{td:>12.2f}{total:>10.2f}"
              f"{1000 / total:>10.1f}{mark}")
    print(f"\n※ {args.target_fps:g}fps = 프레임당 {budget_ms:.1f}ms 예산. "
          "이 표가 process_width 선택의 근거입니다.")
    print("  ★ 이 PC 기준입니다. 실제 판정은 RPi5에서 다시 재세요 (HANDOVER 8 P2).")


def main():
    ap = argparse.ArgumentParser(description="태스크① 오프라인 튜닝/벤치마크")
    ap.add_argument("images", nargs="+", help="입력 이미지 경로들")
    ap.add_argument("-o", "--out", default="out", help="출력 폴더")
    ap.add_argument("--width", type=int, default=640, help="처리 폭 (0=원본)")
    ap.add_argument("--gamma", type=float, default=1.0)

    ap.add_argument("--clahe-clip", type=float, default=2.0)
    ap.add_argument("--clahe-tile", type=int, default=8)

    ap.add_argument("--omega", type=float, default=0.95)
    ap.add_argument("--t0", type=float, default=0.1)
    ap.add_argument("--patch", type=int, default=15)
    ap.add_argument("--scale", type=float, default=0.25)
    ap.add_argument("--guided-radius", type=int, default=8)
    ap.add_argument("--no-guided", action="store_true")
    ap.add_argument("--a-max", type=float, default=0.92)
    ap.add_argument("--sky-ratio", type=float, default=1.0)

    ap.add_argument("--sweep", nargs="+", metavar=("KEY", "VAL"),
                    help="파라미터 스윕: clahe_clip|omega|t0|scale 값들")
    ap.add_argument("--bench", action="store_true", help="해상도별 처리시간 측정")
    ap.add_argument("--bench-iters", type=int, default=10)
    # ★ 드라이버 실측값 15fps (ascamera launch). 인수인계 문서의 20fps는
    #   틀렸습니다 — 근거는 HANDOVER 5-2d. 하드코딩하지 않고 인자로 둡니다.
    ap.add_argument("--target-fps", type=float, default=15.0,
                    help="프레임 예산의 기준 fps (기본 15 = 드라이버 실측)")
    ap.add_argument("--dump-transmission", action="store_true")

    args = ap.parse_args()

    if args.bench:
        cmd_bench(args.images, args)
    elif args.sweep:
        cmd_sweep(args.images, args)
    else:
        cmd_compare(args.images, args)


if __name__ == "__main__":
    main()
