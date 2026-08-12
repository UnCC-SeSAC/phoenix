#!/usr/bin/env python3
"""
파라미터 결정 도구 — "무엇을 좋게 만들 건가"를 정하고 그것에 맞춰 찾습니다.

세 가지 모드:

  --suggest   이미지에서 파라미터를 자동 산출 (autotune.py 규칙 적용)
  --grid      그리드 서치. 목적함수를 지정해 최적값을 찾음
  --calibrate 정답 이미지가 있을 때, 자동 산출 규칙이 최적값을 얼마나 맞추는지 검증

★ 목적함수가 답을 결정합니다.
  같은 이미지라도 무엇을 최대화하느냐에 따라 최적 파라미터가 달라집니다.
  실측 예: PSNR을 목적으로 두면 clipLimit 최적값이 **항상 1.0(CLAHE 최소)** 로
  나옵니다. CLAHE는 원본과의 픽셀 차이를 키우는 연산이라 PSNR이 무조건 싫어하기
  때문입니다. 하지만 CLAHE의 목적은 원본 복원이 아니라 국소 대비 확보입니다.
  → **PSNR은 clipLimit을 판정할 자격이 없습니다.**

  우리의 진짜 목적함수는 YOLO 검출 성능입니다. 라벨이 준비되면
  `--objective external --scores scores.csv` 로 갈아끼우세요.

사용:
  python3 tools/find_params.py --suggest smoke*.jpg
  python3 tools/find_params.py --grid hazy.png --reference clear.png
  python3 tools/find_params.py --grid smoke.jpg --objective contrast
  python3 tools/find_params.py --calibrate            # 합성 데이터로 규칙 검증
"""

from __future__ import annotations

import argparse
import csv
import itertools
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from image_pipeline.autotune import (  # noqa: E402
    estimate_brightness,
    estimate_haze_index,
    estimate_noise_sigma,
    relative_haze,
    suggest_params,
)
from image_pipeline.dehaze import ClaheEnhancer, DarkChannelDehazer  # noqa: E402
from image_pipeline.pipeline import Pipeline  # noqa: E402


# ---------------------------------------------------------------- 목적함수


def psnr(a: np.ndarray, b: np.ndarray) -> float:
    mse = np.mean((a.astype(np.float64) - b.astype(np.float64)) ** 2)
    return float("inf") if mse == 0 else float(10 * np.log10(255.0 ** 2 / mse))


def obj_contrast(out, _ref=None):
    return float(cv2.cvtColor(out, cv2.COLOR_BGR2GRAY).std())


def obj_entropy(out, _ref=None):
    g = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)
    h = cv2.calcHist([g], [0], None, [256], [0, 256]).ravel()
    p = h / max(h.sum(), 1)
    p = p[p > 0]
    return float(-(p * np.log2(p)).sum())


def obj_contrast_noise(out, _ref=None):
    """대비를 올리되 노이즈 증폭에 벌점.

    무기준(no-reference) 목적함수 중 그나마 균형이 잡힌 형태입니다.
    대비만 최대화하면 노이즈까지 키운 결과가 1등이 되어버립니다.
    다만 이것도 **대리 지표**이지 검출률 자체는 아닙니다.
    """
    return obj_contrast(out) - 1.5 * estimate_noise_sigma(out)


def obj_psnr(out, ref):
    if ref is None:
        raise SystemExit("psnr 목적함수는 --reference 이미지가 필요합니다")
    return psnr(out, ref)


OBJECTIVES = {
    "psnr": obj_psnr,
    "contrast": obj_contrast,
    "entropy": obj_entropy,
    "contrast_noise": obj_contrast_noise,
}


# ---------------------------------------------------------------- 모드


def run_suggest(paths, args):
    for path in paths:
        img = cv2.imread(path)
        if img is None:
            print(f"[skip] {path}")
            continue
        if args.width and img.shape[1] > args.width:
            h = int(round(img.shape[0] * args.width / img.shape[1]))
            img = cv2.resize(img, (args.width, h), interpolation=cv2.INTER_AREA)

        p = suggest_params(img, fps_budget_ms=args.fps_budget,
                           haze_baseline=args.haze_baseline)
        m = p.pop("_measured")

        print(f"\n=== {os.path.basename(path)} ===")
        print(f"  측정: 연기지표 {m['haze_index']:.3f} → 상대농도 {m['haze_relative']:.2f}"
              f" | 노이즈 σ {m['noise_sigma']:.2f} | 밝기 {m['brightness']:.1f}")
        if args.haze_baseline is None:
            print("  ※ --haze-baseline 미지정. 연기 없는 프레임으로 기준선을 잡으면"
                  " 상대농도가 정확해집니다.")
        print("  # config/preprocess.yaml 에 복붙")
        for k, v in p.items():
            print(f"    {k}: {v}")


def run_grid(paths, args):
    ref = None
    if args.reference:
        ref = cv2.imread(args.reference)
        if ref is None:
            raise SystemExit(f"참조 이미지를 못 읽었습니다: {args.reference}")

    fn = OBJECTIVES[args.objective]
    omegas = [float(v) for v in args.omega]
    t0s = [float(v) for v in args.t0]
    clips = [float(v) for v in args.clip]

    for path in paths:
        img = cv2.imread(path)
        if img is None:
            continue
        if args.width and img.shape[1] > args.width:
            h = int(round(img.shape[0] * args.width / img.shape[1]))
            img = cv2.resize(img, (args.width, h), interpolation=cv2.INTER_AREA)
        r = ref
        if r is not None and r.shape[:2] != img.shape[:2]:
            r = cv2.resize(r, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_AREA)

        rows = []
        for om, t0, cl in itertools.product(omegas, t0s, clips):
            out = Pipeline(
                mode="full",
                clahe=ClaheEnhancer(cl),
                dehazer=DarkChannelDehazer(omega=om, t0=t0),
            ).process(img)
            rows.append({
                "omega": om, "t0": t0, "clip": cl,
                "score": fn(out, r),
                "contrast": obj_contrast(out),
                "entropy": obj_entropy(out),
                "noise": estimate_noise_sigma(out),
            })

        rows.sort(key=lambda x: -x["score"])
        name = os.path.basename(path)
        print(f"\n=== {name} — 목적함수 '{args.objective}' 상위 {args.top}개 "
              f"(총 {len(rows)}조합) ===")
        print(f"{'omega':>7}{'t0':>7}{'clip':>7}{'score':>10}"
              f"{'contrast':>10}{'entropy':>9}{'noise':>8}")
        for r_ in rows[:args.top]:
            print(f"{r_['omega']:7.2f}{r_['t0']:7.2f}{r_['clip']:7.2f}"
                  f"{r_['score']:10.3f}{r_['contrast']:10.2f}"
                  f"{r_['entropy']:9.3f}{r_['noise']:8.2f}")

        auto = suggest_params(img, haze_baseline=args.haze_baseline)
        print(f"  자동 산출값: omega {auto['dehaze_omega']:.2f} "
              f"t0 {auto['dehaze_t0']:.2f} clip {auto['clahe_clip_limit']:.2f}")

        if args.csv:
            out_csv = os.path.join(args.out, f"{os.path.splitext(name)[0]}_grid.csv")
            os.makedirs(args.out, exist_ok=True)
            with open(out_csv, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)
            print(f"  전체 결과: {out_csv}")

        if args.objective == "psnr":
            print("  ※ PSNR은 clipLimit을 판정하지 못합니다(항상 최소값이 1등).")
            print("    omega·t0 보정에만 쓰고, clipLimit은 YOLO mAP로 정하세요.")


def run_calibrate(args):
    """합성 데이터로 자동 산출 규칙을 검증합니다.

    "규칙이 뽑은 값"과 "그리드 서치가 찾은 최적값"이 얼마나 가까운지 봅니다.
    이 검증이 있어야 규칙을 믿고 실물에 적용할 수 있습니다.
    """
    from make_synthetic import add_haze, make_scene

    clear, depth, _ = make_scene(640, 480, 0)
    baseline = estimate_haze_index(clear)[0]
    print(f"기준선(연기 없는 프레임) = {baseline:.3f}\n")

    omegas = [0.70, 0.75, 0.80, 0.85, 0.90, 0.95]
    t0s = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35]

    print(f"{'beta':>5}{'상대농도':>9}"
          f"{'omega*':>8}{'omega자동':>10}{'t0*':>7}{'t0자동':>8}")
    for beta in (0.3, 0.8, 1.5, 2.5, 3.5):
        hazy = add_haze(clear, depth, beta, seed=1)[0]
        best, best_p = -1e9, None
        for om, t0 in itertools.product(omegas, t0s):
            out = Pipeline(mode="full", clahe=ClaheEnhancer(2.0),
                           dehazer=DarkChannelDehazer(omega=om, t0=t0)).process(hazy)
            s = psnr(out, clear)
            if s > best:
                best, best_p = s, (om, t0)
        p = suggest_params(hazy, haze_baseline=baseline)
        rel = p["_measured"]["haze_relative"]
        print(f"{beta:5.1f}{rel:9.2f}"
              f"{best_p[0]:8.2f}{p['dehaze_omega']:10.2f}"
              f"{best_p[1]:7.2f}{p['dehaze_t0']:8.2f}")

    print("\n※ clipLimit은 이 표에 없습니다. PSNR 기준으로는 최적값이 항상 1.0이라")
    print("  판정이 불가능하기 때문입니다. YOLO 라벨이 준비되면 그때 정하세요.")


def main():
    ap = argparse.ArgumentParser(description="파라미터 자동 산출 / 그리드 서치")
    ap.add_argument("images", nargs="*", help="입력 이미지")
    ap.add_argument("--suggest", action="store_true")
    ap.add_argument("--grid", action="store_true")
    ap.add_argument("--calibrate", action="store_true")

    ap.add_argument("--reference", help="정답 이미지 (psnr 목적함수용)")
    ap.add_argument("--objective", default="contrast_noise", choices=list(OBJECTIVES))
    ap.add_argument("--haze-baseline", type=float, default=None,
                    help="연기 없는 프레임의 연기지표. --suggest로 먼저 측정")
    ap.add_argument("--fps-budget", type=float, default=None,
                    help="프레임당 허용 ms (예: 50). 주면 dehaze_scale도 산출")
    ap.add_argument("--width", type=int, default=640)

    ap.add_argument("--omega", nargs="+", default=[0.7, 0.8, 0.9, 0.95])
    ap.add_argument("--t0", nargs="+", default=[0.05, 0.1, 0.2, 0.3])
    ap.add_argument("--clip", nargs="+", default=[1.0, 2.0, 3.0, 4.0])
    ap.add_argument("--top", type=int, default=8)
    ap.add_argument("--csv", action="store_true")
    ap.add_argument("-o", "--out", default="out")

    args = ap.parse_args()

    if args.calibrate:
        run_calibrate(args)
    elif args.grid:
        if not args.images:
            raise SystemExit("--grid 는 이미지가 필요합니다")
        run_grid(args.images, args)
    else:
        if not args.images:
            raise SystemExit("이미지를 지정하거나 --calibrate 를 쓰세요")
        run_suggest(args.images, args)


if __name__ == "__main__":
    main()
