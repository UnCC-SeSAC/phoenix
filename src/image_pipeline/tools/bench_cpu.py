#!/usr/bin/env python3
"""
전처리 **CPU 점유** 프로파일러 — ROS 없이 OpenCV/NumPy만으로 돕니다.

`tune_offline.py --bench` 와 무엇이 다른가:
  저쪽은 **벽시계 시간**(지연)을 재서 "몇 fps 나오나"를 봅니다.
  이쪽은 **CPU 시간**(모든 스레드의 user+sys 합)을 재서 "코어를 얼마나
  먹나"를 봅니다. 둘은 어긋납니다 — OpenCV가 스레드를 늘리면 지연은 줄지만
  총 CPU는 늘어납니다. RPi5는 4코어를 nav2/SLAM/YOLO와 나눠 쓰므로
  전처리에서 중요한 건 후자입니다.

사용 예)
  # 스레드 수별 CPU 비교 (기본)
  python3 bench_cpu.py smoke1.jpg

  # 카메라가 15fps 가 아니면 지정 — 코어 점유율 계산에 씁니다
  python3 bench_cpu.py smoke1.jpg --fps 30

  # 단계별 분해 (어디가 비싼지)
  python3 bench_cpu.py smoke1.jpg --stages

  # 실제 처리 해상도로 (노드의 process_width 와 맞출 것)
  python3 bench_cpu.py smoke1.jpg --size 640x480

읽는 법)
  "코어" 열이 1.0 이면 한 코어를 꽉 쓰는 것, 2.5 면 2.5개를 씁니다.
  "@Nfps" 열이 이 노드가 상시 점유하는 코어 비율입니다. 100%를 넘으면
  코어 하나로는 부족하다는 뜻입니다.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from image_pipeline.dehaze import ClaheEnhancer, DarkChannelDehazer  # noqa: E402
from image_pipeline.pipeline import MODES, Pipeline  # noqa: E402


def measure(fn, n: int, warm: int = 8) -> tuple[float, float]:
    """(벽시계 ms/프레임, CPU ms/프레임).

    워밍업은 반드시 실제 해상도로 돌립니다 — 첫 호출은 OpenCV 초기화와 버퍼
    할당이 섞여 수십 ms 뻥튀기됩니다.
    """
    for _ in range(warm):
        fn()
    cpu0, t0 = time.process_time(), time.perf_counter()
    for _ in range(n):
        fn()
    wall = (time.perf_counter() - t0) / n * 1000.0
    cpu = (time.process_time() - cpu0) / n * 1000.0
    return wall, cpu


# 프로그램 시작 시점의 기본 스레드 수. threads=0("건드리지 않음")을 다른 값
# **뒤에** 측정할 때 앞의 설정을 물려받으면 두 줄이 같은 값이 되어버립니다.
# setNumThreads 는 프로세스 전역이라 명시적으로 되돌려야 합니다.
DEFAULT_THREADS = cv2.getNumThreads()


def set_threads(threads: int) -> None:
    cv2.setNumThreads(threads if threads > 0 else DEFAULT_THREADS)


def build(mode: str, threads: int) -> Pipeline:
    set_threads(threads)
    return Pipeline(
        mode=mode,
        gamma=1.0,
        clahe=ClaheEnhancer(2.0, (8, 8)),
        # config/preprocess.yaml 의 실사용 값과 맞춰둡니다.
        dehazer=DarkChannelDehazer(scale=0.25, use_guided=True,
                                   guided_radius=8, a_smoothing=0.85),
    )


def header(fps: float) -> None:
    print(f"  {'':<26}{'wall(ms)':>10}{'cpu(ms)':>10}{'코어':>7}{f'@{fps:g}fps':>10}")


def line(label: str, wall: float, cpu: float, fps: float) -> None:
    print(f"  {label:<26}{wall:10.2f}{cpu:10.2f}{cpu / max(wall, 1e-9):7.2f}"
          f"{cpu * fps / 10:9.0f}%")


def run_modes(img, args) -> None:
    print(f"=== 모드 x 스레드 ({img.shape[1]}x{img.shape[0]}, {args.n}프레임) ===")
    for threads in args.threads:
        print(f"--- cv2 threads={threads if threads > 0 else '기본(코어 수)'} ---")
        header(args.fps)
        for mode in args.modes:
            pipe = build(mode, threads)
            wall, cpu = measure(lambda: pipe.process(img), args.n)
            line(mode, wall, cpu, args.fps)
        print()


def run_stages(img, args) -> None:
    """mode=full 의 단계별 분해. 어디를 고쳐야 하는지 알려줍니다."""
    h, w = img.shape[:2]
    d = DarkChannelDehazer(scale=0.25, use_guided=True, guided_radius=8, a_smoothing=0.85)
    cl = ClaheEnhancer(2.0, (8, 8))
    img_f = img.astype(np.float32) / 255.0
    t, a = d.estimate_transmission(img_f)
    np.clip(t, d.t0, 1.0, out=t)
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)

    # 복원식만 따로 재려면 t/a 를 고정해 넘겨야 합니다. dehaze.py 의 구현과
    # 같은 식이며, 여기서 재는 건 추정을 뺀 **복원 단계 단독** 비용입니다.
    a255 = tuple(float(v) for v in (a.reshape(3) * 255.0))
    a_add, a_sub = a255 + (0.0,), tuple(v - 0.5 for v in a255) + (0.0,)

    def restore_only():
        inv3 = cv2.cvtColor(np.reciprocal(t), cv2.COLOR_GRAY2BGR)
        out = cv2.subtract(img, a_add, dtype=cv2.CV_32F)
        cv2.multiply(out, inv3, dst=out)
        return cv2.add(out, a_sub, dtype=cv2.CV_8U)

    stages = [
        ("uint8->float32/255", lambda: img.astype(np.float32) / 255.0),
        ("투과율 추정(1/4 축소)", lambda: d.estimate_transmission(img_f)),
        ("복원식 단독", restore_only),
        ("디헤이즈 전체", lambda: d.process(img)),
        ("CLAHE 전체", lambda: cl.process(img)),
        ("  |- cvtColor BGR<->LAB", lambda: cv2.cvtColor(
            cv2.cvtColor(img, cv2.COLOR_BGR2LAB), cv2.COLOR_LAB2BGR)),
        ("  |- clahe.apply(L)", lambda: cl._clahe.apply(lab[:, :, 0])),
    ]
    print(f"=== 단계별 ({w}x{h}, mode=full) ===")
    for threads in args.threads:
        print(f"--- cv2 threads={threads if threads > 0 else '기본(코어 수)'} ---")
        set_threads(threads)
        header(args.fps)
        for label_, fn in stages:
            wall, cpu = measure(fn, args.n)
            line(label_, wall, cpu, args.fps)
        print()


def parse_size(text: str) -> tuple[int, int]:
    w, _, h = text.lower().partition("x")
    return int(w), int(h)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("image", help="입력 이미지 (연기 사진 권장)")
    ap.add_argument("--size", default="640x480",
                    help="처리 해상도. 노드의 process_width 와 맞출 것 (기본 640x480)")
    ap.add_argument("-n", type=int, default=40, help="측정 프레임 수 (기본 40)")
    ap.add_argument("--fps", type=float, default=15.0,
                    help="카메라 fps. 코어 점유율 계산용 (기본 15 — ascamera.launch.py)")
    ap.add_argument("--threads", type=int, nargs="+", default=[0, 2, 1],
                    help="비교할 cv2 스레드 수. 0=건드리지 않음 (기본: 0 2 1)")
    ap.add_argument("--modes", nargs="+", default=["full", "dehaze", "clahe"],
                    help=f"비교할 모드. 가능: {', '.join(MODES)}")
    ap.add_argument("--stages", action="store_true", help="단계별 분해도 함께 출력")
    args = ap.parse_args()

    for mode in args.modes:
        if mode not in MODES:
            print(f"[오류] 알 수 없는 mode: {mode} (가능: {', '.join(MODES)})",
                  file=sys.stderr)
            return 2

    src = cv2.imread(args.image)
    if src is None:
        print(f"[오류] 이미지를 읽을 수 없습니다: {args.image}", file=sys.stderr)
        return 2
    w, h = parse_size(args.size)
    img = cv2.resize(src, (w, h), interpolation=cv2.INTER_AREA)

    print(f"OpenCV {cv2.__version__} | 코어 {cv2.getNumberOfCPUs()}개 | "
          f"기본 스레드 {cv2.getNumThreads()}\n")
    run_modes(img, args)
    if args.stages:
        run_stages(img, args)

    print("* CPU 시간은 모든 스레드의 합입니다. 다른 노드와 코어를 나눠 쓰는")
    print("  로봇에서는 '코어' 열이 작은 쪽이 유리합니다 (지연에 여유가 있다면).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
