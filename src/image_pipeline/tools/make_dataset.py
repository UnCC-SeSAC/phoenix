#!/usr/bin/env python3
"""
YOLO 학습용 조건별 데이터셋 생성기.

핵심 원칙: **원본(raw)이 마스터고, 전처리본은 언제든 다시 만드는 파생물이다.**

  raw/images  +  raw/labels   ← 사람이 만드는 유일한 자산
        │
        ├─ clahe/images   (라벨은 raw/labels 복사)
        └─ full/images    (라벨은 raw/labels 복사)

왜 이렇게 하나:
  디헤이즈·CLAHE는 **픽셀을 이동시키지 않는 화소값 변환**입니다. 바운딩박스
  좌표는 기하학적 정보라 원본과 전처리본에서 **완전히 동일**합니다.
  따라서 라벨링은 딱 한 번만 하면 되고, 전처리 파라미터를 바꿔도
  **다시 라벨링할 필요가 없습니다** — 이미지만 재생성하면 끝.

  로봇을 아직 못 받아서 파라미터가 확정이 아닌 지금, 이 구조가 아니면
  튜닝할 때마다 라벨링을 다시 하게 됩니다.

라벨링 팁:
  보기는 `full/images`(잘 보임)로 하고, 저장은 `raw/labels`에 합니다.
  연기 속 원본에서는 성냥불이 사람 눈에도 잘 안 보여 라벨 품질이 떨어집니다.
  좌표가 같으므로 어느 쪽을 보며 찍든 라벨 파일은 양쪽에 그대로 유효합니다.

사용:
  # 1) bag에서 프레임을 뽑아 raw/images 에 넣어둔 뒤
  python3 tools/make_dataset.py --raw raw_frames --out dataset

  # 2) 라벨링 후 다시 돌리면 라벨도 각 조건에 배포됩니다
  python3 tools/make_dataset.py --raw raw_frames --out dataset --split 0.2

  # 파라미터를 바꿨을 때 (라벨은 그대로 두고 이미지만 재생성)
  python3 tools/make_dataset.py --raw raw_frames --out dataset --clahe-clip 3.0
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import random
import shutil
import sys
import time

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from image_pipeline.dehaze import ClaheEnhancer, DarkChannelDehazer  # noqa: E402
from image_pipeline.pipeline import Pipeline  # noqa: E402

IMG_EXT = (".png", ".jpg", ".jpeg", ".bmp")

# 생성할 조건. raw는 복사만 하므로 파이프라인을 안 태웁니다.
CONDITIONS = ("raw", "clahe", "full")


def list_images(d: str) -> list[str]:
    if not os.path.isdir(d):
        return []
    return sorted(f for f in os.listdir(d) if f.lower().endswith(IMG_EXT))


def build_pipeline(args, mode: str) -> Pipeline:
    return Pipeline(
        mode=mode,
        gamma=args.gamma,
        clahe=ClaheEnhancer(args.clahe_clip, (args.clahe_tile, args.clahe_tile)),
        dehazer=DarkChannelDehazer(
            omega=args.omega,
            t0=args.t0,
            patch=args.patch,
            scale=args.scale,
            a_max=args.a_max,
            sky_ratio=args.sky_ratio,
            # ★ 데이터셋 생성은 프레임 순서가 뒤섞일 수 있어 시간 평활을 끕니다.
            #   (연속 bag을 순서대로 처리할 때만 --a-smoothing 으로 켜세요.)
            a_smoothing=args.a_smoothing,
        ),
    )


def params_dict(args) -> dict:
    return {
        "process_width": args.width,
        "gamma": args.gamma,
        "clahe_clip_limit": args.clahe_clip,
        "clahe_tile_grid": [args.clahe_tile, args.clahe_tile],
        "dehaze_omega": args.omega,
        "dehaze_t0": args.t0,
        "dehaze_patch": args.patch,
        "dehaze_scale": args.scale,
        "dehaze_a_max": args.a_max,
        "dehaze_sky_ratio": args.sky_ratio,
        "dehaze_a_smoothing": args.a_smoothing,
    }


def main():
    ap = argparse.ArgumentParser(description="조건별 YOLO 데이터셋 생성")
    ap.add_argument("--raw", required=True,
                    help="원본 폴더 (하위에 images/, 선택적으로 labels/)")
    ap.add_argument("--out", required=True, help="출력 데이터셋 루트")
    ap.add_argument("--width", type=int, default=640,
                    help="처리 폭. ★ 노드의 process_width와 반드시 같아야 함")
    ap.add_argument("--split", type=float, default=0.2, help="val 비율 (0이면 분할 안 함)")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--classes", default="fire,smoke",
                    help="data.yaml에 넣을 클래스 이름 (쉼표 구분)")

    ap.add_argument("--gamma", type=float, default=1.0)
    ap.add_argument("--clahe-clip", type=float, default=2.0)
    ap.add_argument("--clahe-tile", type=int, default=8)
    ap.add_argument("--omega", type=float, default=0.95)
    ap.add_argument("--t0", type=float, default=0.1)
    ap.add_argument("--patch", type=int, default=15)
    ap.add_argument("--scale", type=float, default=0.25)
    ap.add_argument("--a-max", type=float, default=0.92)
    ap.add_argument("--sky-ratio", type=float, default=1.0)
    ap.add_argument("--a-smoothing", type=float, default=0.0)
    args = ap.parse_args()

    raw_img_dir = os.path.join(args.raw, "images")
    raw_lbl_dir = os.path.join(args.raw, "labels")

    # ★ 출력의 raw/ 조건 폴더가 입력 폴더와 겹치면 마스터 자산 위에 덮어씁니다.
    #   라벨이 유일본이라 사고가 나면 복구가 안 되므로 먼저 막습니다.
    raw_abs = os.path.abspath(args.raw)
    out_raw_abs = os.path.abspath(os.path.join(args.out, "raw"))
    if raw_abs == out_raw_abs or raw_abs.startswith(out_raw_abs + os.sep):
        raise SystemExit(
            f"입력 폴더가 출력의 raw/ 조건 폴더와 겹칩니다.\n"
            f"  입력: {raw_abs}\n  출력: {out_raw_abs}\n"
            "  라벨은 유일본이라 덮어쓰면 복구가 안 됩니다. 예를 들어\n"
            "    --raw raw_frames --out dataset\n"
            "  처럼 서로 다른 경로를 쓰세요."
        )

    if not os.path.isdir(raw_img_dir):
        raise SystemExit(
            f"원본 이미지 폴더가 없습니다: {raw_img_dir}\n"
            "  rosbag에서 프레임을 뽑아 이 경로에 넣어주세요."
        )

    names = list_images(raw_img_dir)
    if not names:
        raise SystemExit(f"이미지가 없습니다: {raw_img_dir}")

    # --- 라벨 현황 ---
    labels = {}
    if os.path.isdir(raw_lbl_dir):
        for n in names:
            stem = os.path.splitext(n)[0]
            p = os.path.join(raw_lbl_dir, stem + ".txt")
            if os.path.isfile(p):
                labels[n] = p

    print(f"원본 이미지 {len(names)}장 / 라벨 {len(labels)}개")
    if not labels:
        print("  ※ 라벨이 아직 없습니다. 이미지만 생성하니, 라벨링 후 다시 실행하세요.")
        print(f"  ※ 라벨은 반드시 {raw_lbl_dir} 에 저장하세요 (여기가 유일한 마스터).")

    # --- train/val 분할 (조건마다 같은 분할을 써야 비교가 성립) ---
    rng = random.Random(args.seed)
    shuffled = names[:]
    rng.shuffle(shuffled)
    n_val = int(len(shuffled) * args.split) if args.split > 0 else 0
    val_set = set(shuffled[:n_val])
    splits = {n: ("val" if n in val_set else "train") for n in names}
    print(f"분할: train {len(names) - n_val} / val {n_val}  (seed={args.seed})")

    class_names = [c.strip() for c in args.classes.split(",") if c.strip()]

    # --- 조건별 생성 ---
    for cond in CONDITIONS:
        pipe = None if cond == "raw" else build_pipeline(args, cond)
        for sub in ("train", "val") if n_val else ("train",):
            os.makedirs(os.path.join(args.out, cond, "images", sub), exist_ok=True)
            os.makedirs(os.path.join(args.out, cond, "labels", sub), exist_ok=True)

        t0 = time.time()
        for n in names:
            img = cv2.imread(os.path.join(raw_img_dir, n))
            if img is None:
                print(f"  [skip] 읽기 실패: {n}")
                continue

            if args.width and img.shape[1] > args.width:
                h = int(round(img.shape[0] * args.width / img.shape[1]))
                img = cv2.resize(img, (args.width, h), interpolation=cv2.INTER_AREA)

            if pipe is not None:
                img = pipe.process(img)

            sub = splits[n]
            cv2.imwrite(os.path.join(args.out, cond, "images", sub, n), img)

            # ★ 라벨은 변환 없이 그대로 복사. 박스 좌표는 화소값 변환에
            #   영향받지 않고, YOLO 라벨은 정규화 좌표라 리사이즈에도 불변입니다.
            if n in labels:
                stem = os.path.splitext(n)[0]
                shutil.copyfile(
                    labels[n],
                    os.path.join(args.out, cond, "labels", sub, stem + ".txt"),
                )

        # ultralytics용 data.yaml
        with open(os.path.join(args.out, cond, "data.yaml"), "w") as f:
            root = os.path.abspath(os.path.join(args.out, cond))
            f.write(f"path: {root}\n")
            f.write("train: images/train\n")
            f.write(f"val: images/{'val' if n_val else 'train'}\n")
            f.write(f"nc: {len(class_names)}\n")
            f.write(f"names: {class_names}\n")

        print(f"  [{cond}] 완료 — {time.time() - t0:.1f}초")

    # --- 재현성 기록 ---
    src = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "..", "image_pipeline", "dehaze.py")
    with open(src, "rb") as f:
        code_hash = hashlib.sha256(f.read()).hexdigest()[:12]

    manifest = {
        "created": time.strftime("%Y-%m-%d %H:%M:%S"),
        "n_images": len(names),
        "n_labels": len(labels),
        "split": {"val_ratio": args.split, "seed": args.seed,
                  "train": len(names) - n_val, "val": n_val},
        "conditions": list(CONDITIONS),
        "classes": class_names,
        "preprocess_params": params_dict(args),
        "dehaze_py_sha256_12": code_hash,
        "note": ("추론 시 노드 파라미터가 preprocess_params와 다르면 "
                 "학습/추론 분포가 어긋납니다. config/preprocess.yaml과 대조하세요."),
    }
    with open(os.path.join(args.out, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2, ensure_ascii=False)

    print(f"\n생성 완료: {args.out}/  (raw / clahe / full, 같은 라벨·같은 분할)")
    print(f"  마스터는 {args.raw} 입니다. 라벨은 거기에만 만들고 수정하세요.")
    print(f"  {args.out}/ 아래는 전부 파생물이라 언제든 재생성 가능합니다.")
    print("  manifest.json   ← 사용한 파라미터 기록 (추론 설정과 대조용)")
    print("\n★ 학습 시 주의: 데이터로더에서 CLAHE/디헤이즈를 또 걸지 마세요 (이중 적용).")
    print(f"★ 노드의 process_width가 {args.width}가 아니면 학습/추론이 어긋납니다.")


if __name__ == "__main__":
    main()
