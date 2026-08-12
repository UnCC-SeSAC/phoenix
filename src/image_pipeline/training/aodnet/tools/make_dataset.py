#!/usr/bin/env python3
"""
고정 데이터셋 생성 — 검증/평가용 (hazy/ + clear/ + meta.json).

학습은 on-the-fly 합성으로 충분하지만, **검증셋만은 반드시 고정**해야 합니다.
매번 새로 합성하면 "PSNR이 0.3dB 올랐다"가 모델 개선인지 이번 배치가 쉬웠던
건지 구분할 수 없습니다.

    python tools/make_dataset.py -o data/val -n 200 --seed 999
    python tools/make_dataset.py -o data/val_real -n 200 --clear-dir frames/ --depth-dir depth/

스타일별로 균등하게 뽑습니다(white/gray/sooty/firelit). 그래야 나중에
"검은 연기에서만 못한다" 같은 진단이 가능합니다 — meta.json 에 스타일이
기록되므로 compare.py가 스타일별로 점수를 쪼개 보여줍니다.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from aodnet.data import list_images                      # noqa: E402
from aodnet.synth import AIRLIGHT_STYLES, SmokeConfig, make_scene, synthesize  # noqa: E402


def load_depth(depth_dir: Path | None, stem: str) -> np.ndarray | None:
    if depth_dir is None:
        return None
    for ext in (".png", ".tif", ".tiff", ".npy"):
        p = depth_dir / (stem + ext)
        if not p.exists():
            continue
        d = np.load(p) if ext == ".npy" else cv2.imread(str(p), cv2.IMREAD_UNCHANGED)
        if d is None:
            continue
        d = d.astype(np.float32)
        valid = d > 0
        if not valid.any():
            return None
        d[~valid] = d[valid].max()
        return np.clip(d / (np.percentile(d[valid], 99) + 1e-6), 0, 1)
    return None


def main():
    ap = argparse.ArgumentParser(description="고정 연기 데이터셋 생성")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("-n", type=int, default=200)
    ap.add_argument("--clear-dir", default=None)
    ap.add_argument("--depth-dir", default=None)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--seed", type=int, default=999)
    ap.add_argument("--beta-min", type=float, default=0.6)
    ap.add_argument("--beta-max", type=float, default=3.2)
    ap.add_argument("--fire-prob", type=float, default=0.0,
                    help="화염 합성 확률 (학습셋과 맞춰야 평가가 의미 있습니다)")
    ap.add_argument("--save-transmission", action="store_true",
                    help="정답 투과율 t도 저장 (분석용, 용량 증가)")
    args = ap.parse_args()

    out = Path(args.out)
    (out / "hazy").mkdir(parents=True, exist_ok=True)
    (out / "clear").mkdir(parents=True, exist_ok=True)
    if args.save_transmission:
        (out / "trans").mkdir(parents=True, exist_ok=True)

    clear_files = list_images(args.clear_dir) if args.clear_dir else []
    depth_dir = Path(args.depth_dir) if args.depth_dir else None
    rng = np.random.default_rng(args.seed)

    records = []
    for i in range(args.n):
        style = AIRLIGHT_STYLES[i % len(AIRLIGHT_STYLES)]   # 균등 순환
        cfg = SmokeConfig(airlight_style=style,
                          beta_range=(args.beta_min, args.beta_max),
                          fire_prob=args.fire_prob)

        if clear_files:
            src = clear_files[i % len(clear_files)]
            scene = cv2.imread(str(src), cv2.IMREAD_COLOR)
            if scene is None:
                continue
            depth = load_depth(depth_dir, src.stem)
            name = f"{i:05d}_{src.stem}.png"
        else:
            scene = make_scene(args.width, args.height, rng)
            depth = None
            name = f"{i:05d}.png"

        hazy, target, meta = synthesize(scene, rng, cfg, depth=depth)

        cv2.imwrite(str(out / "hazy" / name), hazy)
        cv2.imwrite(str(out / "clear" / name), target)
        if args.save_transmission:
            cv2.imwrite(str(out / "trans" / name),
                        (meta.transmission * 255).astype(np.uint8))

        records.append({"name": name, "style": meta.style, "beta": meta.beta,
                        "airlight_bgr": [float(v) for v in meta.airlight],
                        **{k: (float(v) if isinstance(v, float) else v)
                           for k, v in meta.extra.items()}})

    (out / "meta.json").write_text(
        json.dumps({"seed": args.seed, "count": len(records),
                    "clear_dir": args.clear_dir, "depth_dir": args.depth_dir,
                    "used_real_depth": depth_dir is not None,
                    "records": records}, indent=2, ensure_ascii=False))

    print(f"{len(records)}쌍 생성 -> {out}")
    print(f"  hazy/  clear/  meta.json" + ("  trans/" if args.save_transmission else ""))
    if not clear_files:
        print("  ※ 절차 생성 장면입니다. 실제 프레임이 생기면 --clear-dir 로 다시 만드세요.")


if __name__ == "__main__":
    main()
