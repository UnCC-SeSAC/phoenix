#!/usr/bin/env python3
"""
image_dehaze.py — dehaze.py의 DarkChannelDehazer를 정지 이미지 한 장/여러 장에 적용하는 CLI.

기본 사용법 (이미지 한 장, 출력 경로 지정):
    python3 image_dehaze.py input.jpg -o output.jpg

출력 경로를 생략하면 입력 파일명 옆에 "_dehazed"를 붙여 저장합니다:
    python3 image_dehaze.py input.jpg
    -> input_dehazed.jpg

여러 장을 한 번에 처리하려면 --outdir 로 저장 폴더를 지정하세요:
    python3 image_dehaze.py photo1.jpg photo2.jpg photo3.jpg --outdir results/
    python3 image_dehaze.py hazy_images/*.jpg --outdir results/
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

from dehaze import ClaheEnhancer, DarkChannelDehazer, apply_gamma


def build_dehazer(args: argparse.Namespace) -> DarkChannelDehazer:
    return DarkChannelDehazer(
        omega=args.omega,
        t0=args.t0,
        patch=args.patch,
        scale=args.scale,
        use_guided=not args.no_guided,
        guided_radius=args.guided_radius,
        guided_eps=args.guided_eps,
        a_top_ratio=args.a_top_ratio,
        a_max=args.a_max,
        sky_ratio=args.sky_ratio,
        a_smoothing=0.0,  # 정지 이미지는 프레임 간 평활이 의미 없음(항상 끔)
    )


def process_one(dehazer: DarkChannelDehazer, args: argparse.Namespace, img: np.ndarray) -> np.ndarray:
    if args.lowlight:
        result = dehazer.process_lowlight(img, omega=args.lowlight_omega, t0=args.lowlight_t0)
    else:
        result = dehazer.process(img)

    clahe = None
    if args.clahe:
        clahe = ClaheEnhancer(clip_limit=args.clahe_clip, tile_grid=(args.clahe_tile, args.clahe_tile))
        result = clahe.process(result)
    if abs(args.gamma - 1.0) > 1e-3:
        result = apply_gamma(result, args.gamma)

    if args.side_by_side:
        result = np.hstack([img, result])
    return result


def default_output_path(input_path: Path) -> Path:
    return input_path.with_name(f"{input_path.stem}_dehazed{input_path.suffix or '.png'}")


def main() -> int:
    p = argparse.ArgumentParser(description="Dark Channel Prior 기반 이미지 디헤이징")
    p.add_argument("inputs", nargs="+", help="입력 이미지 경로 (여러 장 가능)")
    p.add_argument(
        "-o", "--output", default=None,
        help="출력 경로. 입력이 한 장일 때만 사용 (생략 시 '_dehazed' 붙여 자동 저장)",
    )
    p.add_argument("--outdir", default=None, help="여러 장 처리 시 저장할 폴더")

    dcp = p.add_argument_group("DCP 파라미터 (dehaze.py DarkChannelDehazer와 동일)")
    dcp.add_argument("--omega", type=float, default=0.95)
    dcp.add_argument("--t0", type=float, default=0.1)
    dcp.add_argument("--patch", type=int, default=15)
    dcp.add_argument("--scale", type=float, default=1.0, help="추정 축소 배율 (정지영상 기본 1.0=원본 해상도 그대로)")
    dcp.add_argument("--no-guided", action="store_true", help="guided filter 끄기")
    dcp.add_argument("--guided-radius", type=int, default=8)
    dcp.add_argument("--guided-eps", type=float, default=1e-3)
    dcp.add_argument("--a-top-ratio", type=float, default=0.001)
    dcp.add_argument("--a-max", type=float, default=0.92)
    dcp.add_argument("--sky-ratio", type=float, default=1.0)

    mode = p.add_argument_group("모드/후처리")
    mode.add_argument("--lowlight", action="store_true", help="저조도 보정 모드 (반전-디헤이즈-반전)")
    mode.add_argument("--lowlight-omega", type=float, default=0.8)
    mode.add_argument("--lowlight-t0", type=float, default=0.25)
    mode.add_argument("--clahe", action="store_true", help="디헤이즈 후 CLAHE 추가 적용")
    mode.add_argument("--clahe-clip", type=float, default=2.0)
    mode.add_argument("--clahe-tile", type=int, default=8)
    mode.add_argument("--gamma", type=float, default=1.0, help="디헤이즈 후 감마 보정 (기본 1.0=없음)")
    mode.add_argument("--side-by-side", action="store_true", help="원본|결과 좌우 비교 이미지 출력")

    args = p.parse_args()

    input_paths = [Path(s) for s in args.inputs]

    if args.output is not None and len(input_paths) != 1:
        print("-o/--output은 입력 이미지가 한 장일 때만 쓸 수 있습니다. 여러 장은 --outdir을 쓰세요.", file=sys.stderr)
        return 1

    if len(input_paths) > 1 and args.outdir is None:
        print("입력 이미지가 여러 장입니다. --outdir 로 저장 폴더를 지정해주세요.", file=sys.stderr)
        return 1

    outdir = Path(args.outdir) if args.outdir else None
    if outdir is not None:
        outdir.mkdir(parents=True, exist_ok=True)

    dehazer = build_dehazer(args)

    ok_count = 0
    for in_path in input_paths:
        if not in_path.exists():
            print(f"[건너뜀] 파일 없음: {in_path}", file=sys.stderr)
            continue
        img = cv2.imread(str(in_path))
        if img is None:
            print(f"[건너뜀] 이미지를 읽을 수 없음: {in_path}", file=sys.stderr)
            continue

        result = process_one(dehazer, args, img)

        if outdir is not None:
            out_path = outdir / in_path.name
        elif args.output is not None:
            out_path = Path(args.output)
        else:
            out_path = default_output_path(in_path)

        cv2.imwrite(str(out_path), result)
        print(f"{in_path} -> {out_path}")
        ok_count += 1

    print(f"완료: {ok_count}/{len(input_paths)}장 처리")
    return 0 if ok_count > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())