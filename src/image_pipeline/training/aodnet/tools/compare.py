#!/usr/bin/env python3
"""
DCP(phase1) vs AOD-Net 비교 — 숫자 표 + 비교 그림.

    python tools/compare.py --weights runs/base/best.pt --data data/val
    python tools/compare.py --weights runs/base/best.pt --data data/val --no-dcp   # DCP 없이

phase1의 `image_pipeline.dehaze.DarkChannelDehazer` 를 그대로 import해
**같은 입력·같은 정답**으로 붙입니다. 두 방법을 각자의 데이터로 재면
비교가 아니라 그냥 두 개의 숫자입니다.

출력:
  - 콘솔 표: 전체 / 연기 스타일별 PSNR·SSIM·엣지·대비이득·처리시간
    (엣지·대비이득은 '정답(기준)' 행과의 거리로 읽습니다 — docs/03_metrics.md §3.7)
  - assets/compare.jpg : (연기 | DCP | AOD-Net | 정답) 4열 그리드
  - assets/compare.csv : 샘플별 원자료
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from aodnet.data import bgr_to_tensor, list_images       # noqa: E402
from aodnet.infer import AODNetDehazer                   # noqa: E402
from aodnet.metrics import contrast_gain, edge_density, psnr, ssim  # noqa: E402


def load_dcp(phase1_dir: str | None):
    """phase1 DCP 디헤이저를 가져옵니다. 없으면 None."""
    candidates = [Path(phase1_dir)] if phase1_dir else []
    candidates += [ROOT.parents[1] / "ros" / "image_pipeline", ROOT / "../phase1"]
    for c in candidates:
        if (c / "image_pipeline" / "dehaze.py").exists():
            sys.path.insert(0, str(c.resolve()))
            from image_pipeline.dehaze import DarkChannelDehazer
            # a_max/sky_ratio는 phase1 기본값 그대로. 여기서 DCP를 유리하게
            # 튜닝하면 비교가 무의미해집니다.
            return DarkChannelDehazer(), str(c.resolve())
    return None, None


def metric_row(pred_bgr, clear_bgr, hazy_bgr):
    """PSNR·SSIM은 정답 대비, 엣지·대비이득은 입력 대비로 잽니다.

    엣지 하나만 보면 과평활인지 노이즈 증폭인지 구분이 안 됩니다.
    대비이득(입력 대비 표준편차 비율)을 같이 봐야 갈립니다.
      엣지↓ + 대비이득≈1  -> 과평활 (복원이 약함)
      엣지↑ + 대비이득↑↑  -> 노이즈 증폭 (대비만 올림)
    두 값 모두 '정답(기준)' 행과 비교해서 읽으세요.
    """
    p = bgr_to_tensor(pred_bgr).unsqueeze(0)
    c = bgr_to_tensor(clear_bgr).unsqueeze(0)
    hz = bgr_to_tensor(hazy_bgr).unsqueeze(0)
    return {
        "psnr": psnr(p, c).item(),
        "ssim": ssim(p, c).item(),
        "edge": edge_density(p).item(),
        "cgain": contrast_gain(p, hz).item(),
    }


def label_bar(width: int, texts: list[str], height: int = 24) -> np.ndarray:
    bar = np.zeros((height, width, 3), np.uint8)
    n = len(texts)
    cell = width // n
    for i, t in enumerate(texts):
        cv2.putText(bar, t, (i * cell + 8, height - 7),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return bar


def main():
    ap = argparse.ArgumentParser(description="DCP vs AOD-Net 비교")
    ap.add_argument("--weights", required=True)
    ap.add_argument("--data", required=True, help="make_dataset.py 로 만든 root (hazy/ clear/)")
    ap.add_argument("--device", default="auto")
    ap.add_argument("--strength", type=float, default=1.0)
    ap.add_argument("--no-dcp", action="store_true")
    ap.add_argument("--phase1-dir", default=None)
    ap.add_argument("--grid", type=int, default=4, help="비교 그림에 넣을 샘플 수")
    ap.add_argument("--out-dir", default=str(ROOT / "assets"))
    ap.add_argument("--limit", type=int, default=0, help="0이면 전체")
    args = ap.parse_args()

    root = Path(args.data)
    hazy_files = list_images(root / "hazy")
    if not hazy_files:
        raise SystemExit(f"{root/'hazy'} 가 비었습니다.")
    if args.limit:
        hazy_files = hazy_files[:args.limit]

    styles = {}
    meta_path = root / "meta.json"
    if meta_path.exists():
        meta = json.loads(meta_path.read_text())
        styles = {r["name"]: r["style"] for r in meta["records"]}

    aod = AODNetDehazer(args.weights, args.device, strength=args.strength)
    dcp, dcp_path = (None, None) if args.no_dcp else load_dcp(args.phase1_dir)
    if dcp is not None:
        print(f"DCP     : {dcp_path}")
    else:
        print("DCP     : 사용 안 함")
    print(f"AOD-Net : {args.weights}  (device={aod.device})")
    print(f"샘플    : {len(hazy_files)}장\n")

    # ★ 실측 해상도로 워밍업. AODNetDehazer가 __init__에서 320x240으로 한 번
    #   태우지만, **다른 해상도의 첫 호출은 커널을 다시 컴파일**합니다.
    #   안 하면 첫 샘플이 130ms쯤 나와서 그 샘플이 속한 스타일의 평균 ms가
    #   통째로 부풀고, 'DCP 대비 속도' 결론까지 뒤집힙니다(0.76배 -> 0.60배).
    warm = cv2.imread(str(hazy_files[0]), cv2.IMREAD_COLOR)
    if warm is not None:
        aod.process(warm)
        if dcp is not None:
            dcp.process(warm)

    rows = []
    grid_samples = []

    for f in hazy_files:
        hazy = cv2.imread(str(f), cv2.IMREAD_COLOR)
        clear = cv2.imread(str(root / "clear" / f.name), cv2.IMREAD_COLOR)
        if hazy is None or clear is None:
            continue

        rec = {"name": f.name, "style": styles.get(f.name, "?")}

        rec.update({f"hazy_{k}": v for k, v in metric_row(hazy, clear, hazy).items()})
        # 정답 자신의 엣지 밀도. 이 값이 없으면 "엣지 0.024"가 과복원인지
        # 과평활인지 판단할 기준이 없습니다. PSNR/SSIM은 정의상 상한값입니다.
        gt = metric_row(clear, clear, hazy)
        rec["gt_edge"], rec["gt_cgain"] = gt["edge"], gt["cgain"]

        if dcp is not None:
            t0 = time.perf_counter()
            dcp_out = dcp.process(hazy)
            rec["dcp_ms"] = (time.perf_counter() - t0) * 1000.0
            rec.update({f"dcp_{k}": v for k, v in metric_row(dcp_out, clear, hazy).items()})
        else:
            dcp_out = None

        aod_out = aod.process(hazy)
        rec["aod_ms"] = aod.timings["total"]
        rec.update({f"aod_{k}": v for k, v in metric_row(aod_out, clear, hazy).items()})

        rows.append(rec)
        if len(grid_samples) < args.grid:
            grid_samples.append((hazy, dcp_out, aod_out, clear, rec))

    if not rows:
        raise SystemExit("비교할 샘플이 없습니다.")

    # ------------------------------------------------------------ 표 출력
    def agg(subset, key):
        vals = [r[key] for r in subset if key in r]
        return float(np.mean(vals)) if vals else float("nan")

    methods = ["hazy"] + (["dcp"] if dcp is not None else []) + ["aod"]
    disp = {"hazy": "입력(연기)", "dcp": "DCP", "aod": "AOD-Net"}

    def print_block(title, subset):
        print(f"── {title}  (n={len(subset)})")
        print(f"   {'방법':<12}{'PSNR(dB)':>10}{'SSIM':>9}{'엣지':>9}{'대비이득':>10}{'ms':>9}")
        for m in methods:
            ms = agg(subset, f"{m}_ms")
            ms_s = f"{ms:9.1f}" if ms == ms else " " * 9
            print(f"   {disp[m]:<12}{agg(subset, f'{m}_psnr'):10.2f}"
                  f"{agg(subset, f'{m}_ssim'):9.4f}{agg(subset, f'{m}_edge'):9.4f}"
                  f"{agg(subset, f'{m}_cgain'):10.2f}{ms_s}")
        # 엣지의 기준선. 이보다 낮으면 과평활, 크게 높으면 노이즈 증폭입니다.
        print(f"   {'정답(기준)':<12}{'—':>10}{'—':>9}{agg(subset, 'gt_edge'):9.4f}"
              f"{agg(subset, 'gt_cgain'):10.2f}")
        print()

    print_block("전체", rows)
    for st in sorted({r["style"] for r in rows}):
        if st != "?":
            print_block(f"연기 스타일: {st}", [r for r in rows if r["style"] == st])

    if dcp is not None:
        d = agg(rows, "aod_psnr") - agg(rows, "dcp_psnr")
        print(f"AOD-Net − DCP : PSNR {d:+.2f} dB, "
              f"SSIM {agg(rows,'aod_ssim')-agg(rows,'dcp_ssim'):+.4f}, "
              f"속도 {agg(rows,'dcp_ms')/max(agg(rows,'aod_ms'),1e-6):.2f}배")
        print("※ PSNR은 '합성 정답 기준' 점수입니다. 최종 채택 근거는 후단 YOLO mAP로 잡으세요.\n")

    # ------------------------------------------------------------ 그림
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    panels = []
    for hazy, dcp_out, aod_out, clear, rec in grid_samples:
        imgs = [hazy] + ([dcp_out] if dcp_out is not None else []) + [aod_out, clear]
        imgs = [cv2.resize(im, (320, 240)) for im in imgs]
        row = np.hstack(imgs)
        texts = ["입력"] + (["DCP %.1fdB" % rec.get("dcp_psnr", 0)] if dcp_out is not None else []) \
            + ["AOD %.1fdB" % rec["aod_psnr"], f"정답 [{rec['style']}]"]
        panels.append(np.vstack([row, label_bar(row.shape[1], texts)]))

    grid_path = out_dir / "compare.jpg"
    cv2.imwrite(str(grid_path), np.vstack(panels))

    csv_path = out_dir / "compare.csv"
    with csv_path.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    print(f"그림 : {grid_path}")
    print(f"원자료: {csv_path}")


if __name__ == "__main__":
    main()
