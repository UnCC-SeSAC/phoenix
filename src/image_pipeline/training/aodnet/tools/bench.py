#!/usr/bin/env python3
"""
속도 벤치마크 — ROS 노드에 얹을 수 있는지 판단하는 숫자.

    python tools/bench.py --weights runs/base/best.pt

로봇 카메라가 30 FPS면 전처리 예산은 프레임당 33ms 전부가 아니라
**그중 일부**입니다. 뒤에 YOLO가 붙으니까요. 실무 기준선을 10ms로 잡고
해상도별로 잽니다.

주의:
  - warmup 없이 재면 첫 호출의 커널 컴파일(수백 ms)이 평균을 오염시킵니다.
  - synchronize() 없이 재면 GPU 큐에 넣은 시간만 재고 "0.2ms!" 같은
    거짓 숫자가 나옵니다. AODNetDehazer.process 안에 이미 들어 있습니다.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from aodnet.device import device_name                    # noqa: E402
from aodnet.infer import AODNetDehazer                   # noqa: E402

RESOLUTIONS = [(640, 480), (848, 480), (1280, 720), (1920, 1080)]


def bench(fn, frame, runs: int, warmup: int = 5) -> tuple[float, float]:
    for _ in range(warmup):
        fn(frame)
    times = []
    for _ in range(runs):
        t0 = time.perf_counter()
        fn(frame)
        times.append((time.perf_counter() - t0) * 1000.0)
    return statistics.mean(times), statistics.median(times)


def main():
    ap = argparse.ArgumentParser(description="AOD-Net / DCP 속도 비교")
    ap.add_argument("--weights", required=True)
    ap.add_argument("--device", default="auto")
    ap.add_argument("--runs", type=int, default=30)
    ap.add_argument("--half", action="store_true")
    ap.add_argument("--max-side", type=int, default=0)
    ap.add_argument("--no-dcp", action="store_true")
    args = ap.parse_args()

    aod = AODNetDehazer(args.weights, args.device, max_side=args.max_side, half=args.half)
    print(f"AOD-Net : {aod.device} ({device_name(aod.device)})"
          f"{'  fp16' if aod.half else ''}"
          f"{f'  max_side={args.max_side}' if args.max_side else ''}")

    dcp = None
    if not args.no_dcp:
        try:
            sys.path.insert(0, str(ROOT.parents[1] / "ros" / "image_pipeline"))
            from image_pipeline.dehaze import DarkChannelDehazer
            dcp = DarkChannelDehazer()
            print("DCP     : phase1 image_pipeline (CPU, scale=0.25)")
        except ImportError:
            print("DCP     : 없음 (phase1 미발견)")

    rng = np.random.default_rng(0)
    print(f"\n{'해상도':>12}{'AOD ms':>10}{'AOD FPS':>10}"
          + (f"{'DCP ms':>10}{'DCP FPS':>10}{'배속':>8}" if dcp else ""))
    print("-" * (32 + (28 if dcp else 0)))

    for w, h in RESOLUTIONS:
        frame = rng.integers(0, 255, (h, w, 3), dtype=np.uint8)
        a_mean, _ = bench(aod.process, frame, args.runs)
        line = f"{f'{w}x{h}':>12}{a_mean:10.2f}{1000/a_mean:10.1f}"
        if dcp:
            d_mean, _ = bench(dcp.process, frame, args.runs)
            line += f"{d_mean:10.2f}{1000/d_mean:10.1f}{d_mean/a_mean:8.2f}"
        print(line)

    print("\n기준선: 30 FPS 파이프라인에서 전처리 예산 ≈ 10ms")
    print("초과하면 --max-side 640 또는 --half 를 먼저 시도하세요 (재학습 불필요).")


if __name__ == "__main__":
    main()
