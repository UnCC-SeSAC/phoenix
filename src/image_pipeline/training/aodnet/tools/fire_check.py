#!/usr/bin/env python3
"""
화염 보존율 측정 — "디헤이즈가 불씨를 지우지 않는가".

    python tools/fire_check.py --weights runs/mine2/best.pt --data data/fire_test

왜 별도 지표가 필요한가
-----------------------
PSNR·SSIM은 **화면 전체 평균**입니다. 화염은 화면의 1% 미만이라 불씨를 통째로
지워도 PSNR이 거의 안 움직입니다. 그런데 화재 탐사 로봇에서 불씨를 지우는
전처리는 **최악**입니다 — 찾으려는 대상 자체를 없애는 것이니까요.

그래서 화염 바운딩박스 안쪽만 따로 재고, **정답 대비 비율**로 봅니다.

    보존율 = AOD 출력의 화염 밝기 / 정답의 화염 밝기

    100% 근처  정상
    < 80%      불씨가 어두워짐. 학습 데이터에 불이 있었는지 확인하세요
    > 120%     과복원. 화염 주변이 번짐

실측 배경 (2026-08-10)
    frame_cut(주행 영상)은 불이 하나도 없습니다. 그것만으로 학습한 mine2는
    화염을 한 번도 본 적이 없어 보존율 **63.4%** 가 나왔습니다.
    입력에서는 140.2로 멀쩡히 보이는 불을 86.9로 지운 것입니다.
    -> SmokeConfig.fire_prob 로 학습 프레임에 화염을 합성해 넣어야 합니다.

데이터셋은 화염 박스가 기록된 것이어야 합니다:

    python tools/make_dataset.py -o data/fire_test -n 48 --seed 4242 \
        --clear-dir ../frame_cut/data --fire-prob 0.6
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from aodnet.data import list_images                       # noqa: E402
from aodnet.infer import AODNetDehazer                    # noqa: E402


def box_region(img: np.ndarray, boxes, pad: float = 0.0):
    """정규화 박스 목록 안쪽의 그레이스케일 픽셀을 모읍니다."""
    h, w = img.shape[:2]
    gray = img.astype(np.float32).mean(axis=2)
    out = []
    for _, cx, cy, bw, bh in boxes:
        bw, bh = bw * (1 + pad), bh * (1 + pad)
        x0 = max(0, int((cx - bw / 2) * w))
        x1 = min(w, int(np.ceil((cx + bw / 2) * w)))
        y0 = max(0, int((cy - bh / 2) * h))
        y1 = min(h, int(np.ceil((cy + bh / 2) * h)))
        roi = gray[y0:y1, x0:x1]
        if roi.size:
            out.append(roi)
    return out


def summarize(regions):
    if not regions:
        return None
    return np.array([[r.mean(), r.max()] for r in regions]).mean(axis=0)


def main():
    ap = argparse.ArgumentParser(description="화염 보존율 측정")
    ap.add_argument("--weights", required=True)
    ap.add_argument("--data", required=True, help="fire_boxes가 기록된 데이터셋 root")
    ap.add_argument("--device", default="auto")
    ap.add_argument("--strength", type=float, default=1.0)
    ap.add_argument("--max-side", type=int, default=0)
    args = ap.parse_args()

    root = Path(args.data)
    meta_path = root / "meta.json"
    if not meta_path.exists():
        raise SystemExit(f"{meta_path} 가 없습니다.")
    boxes = {r["name"]: r.get("fire_boxes", [])
             for r in json.loads(meta_path.read_text())["records"]}

    files = [f for f in list_images(root / "hazy") if boxes.get(f.name)]
    if not files:
        raise SystemExit("화염 박스가 기록된 샘플이 없습니다. "
                         "make_dataset.py --fire-prob 로 다시 만드세요.")

    aod = AODNetDehazer(args.weights, args.device, strength=args.strength,
                        max_side=args.max_side)
    acc = {"입력(연기)": [], "AOD-Net": [], "정답": []}

    for f in files:
        bxs = boxes[f.name]
        hazy = cv2.imread(str(f), cv2.IMREAD_COLOR)
        clear = cv2.imread(str(root / "clear" / f.name), cv2.IMREAD_COLOR)
        if hazy is None or clear is None:
            continue
        for tag, img in (("입력(연기)", hazy), ("AOD-Net", aod.process(hazy)),
                         ("정답", clear)):
            s = summarize(box_region(img, bxs))
            if s is not None:
                acc[tag].append(s)

    print(f"모델   : {args.weights}")
    print(f"데이터 : {root}  (화염 포함 {len(files)}장)\n")
    print(f"{'':<14}{'화염 영역 평균':>14}{'최대':>10}")
    stats = {}
    for k, v in acc.items():
        a = np.array(v).mean(axis=0)
        stats[k] = a
        print(f"{k:<14}{a[0]:14.1f}{a[1]:10.1f}")

    g, m = stats["정답"], stats["AOD-Net"]
    keep_mean, keep_max = m[0] / g[0] * 100, m[1] / g[1] * 100
    print(f"\n화염 보존율 (AOD/정답) : 평균 {keep_mean:.1f}%  최대 {keep_max:.1f}%")

    if keep_mean < 80:
        print("  ❌ 불씨가 어두워집니다. 학습 프레임에 화염이 있었는지 확인하세요")
        print("     (--fire-prob 0.6 으로 재학습)")
    elif keep_mean > 120:
        print("  ⚠ 과복원 — 화염 주변이 번질 수 있습니다")
    else:
        print("  ✅ 정상 범위")


if __name__ == "__main__":
    main()
