#!/usr/bin/env python3
"""
YOLO 조건별 비교 하네스 — 전처리 파라미터를 **mAP로** 확정하기 위한 도구.

    # 1) 조건별 데이터셋 생성 (라벨은 한 번만, 이미지만 조건별로)
    python tools/yolo_pipeline.py build --raw raw_frames --out datasets/exp1 --synth-smoke

    # 2) 조건마다 같은 설정으로 학습
    python tools/yolo_pipeline.py train --data datasets/exp1 --epochs 50

    # 3) mAP 비교표
    python tools/yolo_pipeline.py report --data datasets/exp1

라벨이 아직 없다면 배관만 먼저 검증할 수 있습니다:

    python tools/yolo_pipeline.py build --stub 200 --out datasets/stub --synth-smoke
    python tools/yolo_pipeline.py train --data datasets/stub --epochs 5

왜 이게 필요한가
----------------
phase1 CLAUDE.md: "PSNR로 clipLimit 튜닝 금지 — YOLO mAP로 정할 값입니다."
지금 미확정인 값이 셋이고 **전부 mAP로만 정할 수 있습니다.**

  - CLAHE clipLimit
  - AOD-Net 채택 여부 (DCP 대비)
  - 게이팅 임계값 (지금은 PSNR로 정한 0.20)

★ 라벨은 조건별로 다시 만들지 않습니다.
  감마·디헤이즈·CLAHE는 **픽셀을 이동시키지 않는 화소값 변환**이라 바운딩박스
  좌표가 원본과 완전히 동일합니다. 라벨링은 한 번, 이미지만 재생성합니다.
  (phase1 tools/make_dataset.py 와 같은 원칙)

★ 공정성 규칙 — 이걸 어기면 비교가 아니라 그냥 여러 개의 숫자가 됩니다.
  1. **같은 연기 실현**을 모든 조건이 공유합니다. 파일명으로 시드를 고정해
     조건마다 다른 연기가 걸리는 걸 막습니다.
  2. **같은 train/val 분할**. 파일명 해시로 나눠 조건·재실행에 무관하게 동일.
  3. **같은 학습 설정·시드·에폭**.
  4. val 이미지도 train과 **같은 조건**으로 전처리합니다. 실제 배포에서
     학습과 추론의 전처리가 같아야 하므로.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shutil
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "deploy"))

from aodnet.data import list_images                       # noqa: E402
from aodnet.synth import SmokeConfig, composite_fire, synthesize  # noqa: E402

#: 비교할 전처리 조건. raw 는 기준선(무처리)입니다.
CONDITIONS = ("raw", "clahe", "dcp", "aod", "aod_gated")

IMG_EXT = (".jpg", ".jpeg", ".png", ".bmp", ".webp")


# ---------------------------------------------------------------- 공통


def load_phase1():
    """phase1 Pipeline 을 가져옵니다."""
    p1 = ROOT.parents[1] / "ros" / "image_pipeline"
    if not (p1 / "image_pipeline" / "pipeline.py").exists():
        raise SystemExit(f"phase1을 찾지 못했습니다: {p1}")
    sys.path.insert(0, str(p1))
    from image_pipeline.pipeline import Pipeline
    return Pipeline


def split_of(name: str, val_ratio: float) -> str:
    """파일명 해시로 train/val 결정.

    난수가 아니라 해시입니다. 조건마다, 재실행마다 **같은 분할**이 나와야
    조건 간 비교가 성립합니다. 랜덤 셔플을 쓰면 조건 A의 val 이미지가
    조건 B에서는 train에 들어가는 사고가 조용히 납니다.
    """
    if val_ratio <= 0:
        return "train"
    h = int(hashlib.sha1(name.encode()).hexdigest()[:8], 16)
    return "val" if (h % 1000) / 1000.0 < val_ratio else "train"


def seed_of(name: str) -> int:
    """파일명 -> 결정론적 시드. 모든 조건이 같은 연기를 보게 합니다."""
    return int(hashlib.sha1(name.encode()).hexdigest()[:8], 16)


def find_label(raw_root: Path, stem: str) -> Path | None:
    for cand in (raw_root / "labels" / f"{stem}.txt", raw_root / f"{stem}.txt"):
        if cand.exists():
            return cand
    return None


# ---------------------------------------------------------------- stub


def make_stub_scene(w: int, h: int, rng: np.random.Generator):
    """라벨이 붙은 절차 생성 장면. **배관 검증 전용**입니다.

    ★ 이걸로 나온 mAP는 아무 의미가 없습니다. 도형 몇 개짜리 장면이라
      실제 화재 탐지 성능과 무관합니다. 목적은 오직
      "데이터셋 생성 -> 학습 -> 평가 -> 비교표"가 끝까지 도는지 확인하는 것.
      실제 라벨이 생기면 --raw 로 갈아타세요.

    반환: (BGR 이미지, [(cls, cx, cy, bw, bh) 정규화 좌표])
    """
    img = np.zeros((h, w, 3), np.uint8)
    wall = int(rng.integers(45, 105))
    img[:, :] = (wall, wall, wall)
    cv2.rectangle(img, (0, 0), (w, int(h * 0.3)), (wall - 15,) * 3, -1)
    cv2.rectangle(img, (0, int(h * 0.78)), (w, h), (wall - 6,) * 3, -1)

    for _ in range(int(rng.integers(1, 4))):        # 기둥
        x0 = int(rng.uniform(0.05, 0.85) * w)
        pw = int(rng.uniform(0.05, 0.15) * w)
        c = int(np.clip(wall + rng.integers(20, 60), 0, 255))
        cv2.rectangle(img, (x0, int(0.2 * h)), (x0 + pw, int(0.8 * h)), (c, c, c), -1)

    boxes = []
    for _ in range(int(rng.integers(1, 3))):        # 성냥불 (class 0 = fire)
        r = int(rng.uniform(0.010, 0.030) * w) + 3
        fx = int(rng.uniform(0.12, 0.88) * w)
        fy = int(rng.uniform(0.35, 0.75) * h)
        cv2.circle(img, (fx, fy), r * 2, (10, 60, 150), -1)
        cv2.circle(img, (fx, fy), r, (40, 150, 250), -1)
        cv2.circle(img, (fx, fy), max(2, r // 2), (200, 240, 255), -1)
        side = r * 4
        boxes.append((0, fx / w, fy / h, side / w, side / h))

    img = np.clip(img.astype(np.float32) + rng.normal(0, 3, img.shape), 0, 255).astype(np.uint8)
    return img, boxes


# ---------------------------------------------------------------- build


def build_pipelines(args, conditions):
    """조건 이름 -> 전처리 함수(bgr->bgr)."""
    Pipeline = load_phase1()
    procs = {}

    for cond in conditions:
        if cond == "raw":
            procs[cond] = lambda x: x
        elif cond == "clahe":
            pipe = Pipeline(mode="clahe", gamma=args.gamma)
            pipe.clahe.update(args.clahe_clip, (args.clahe_tile, args.clahe_tile))
            procs[cond] = pipe.process
        elif cond == "dcp":
            pipe = Pipeline(mode="full", gamma=args.gamma)      # 기본 dehazer = DCP
            pipe.clahe.update(args.clahe_clip, (args.clahe_tile, args.clahe_tile))
            procs[cond] = pipe.process
        elif cond in ("aod", "aod_gated"):
            if not args.weights:
                raise SystemExit(f"'{cond}' 조건에는 --weights 가 필요합니다")
            from aodnet.infer import AODNetDehazer
            aod = AODNetDehazer(args.weights, args.device, max_side=args.aod_max_side)
            pipe = Pipeline(mode="full", gamma=args.gamma, dehazer=aod)
            pipe.clahe.update(args.clahe_clip, (args.clahe_tile, args.clahe_tile))

            if cond == "aod":
                procs[cond] = pipe.process
            else:
                if args.gate_baseline is None:
                    raise SystemExit("'aod_gated' 에는 --gate-baseline 이 필요합니다 "
                                     "(tools/tune_gate.py 로 측정)")
                from aod_lite import SmokeGate
                # 정지영상이라 EMA는 끕니다(프레임 순서에 의미 없음).
                gate = SmokeGate(args.gate_baseline, args.gate_threshold, smoothing=0.0)
                clahe_only = Pipeline(mode="clahe", gamma=args.gamma)
                clahe_only.clahe.update(args.clahe_clip, (args.clahe_tile, args.clahe_tile))

                def gated(x, _p=pipe, _c=clahe_only, _g=gate):
                    # 게이트가 꺼져도 감마·CLAHE는 그대로 걸립니다.
                    # 디헤이즈만 건너뛰는 게 게이팅의 정의입니다.
                    return _p.process(x) if _g.should_process(x) else _c.process(x)

                procs[cond] = gated
        else:
            raise SystemExit(f"모르는 조건: {cond}")
    return procs


def cmd_build(args):
    conditions = [c.strip() for c in args.conditions.split(",") if c.strip()]
    for c in conditions:
        if c not in CONDITIONS:
            raise SystemExit(f"모르는 조건 {c!r} (가능: {', '.join(CONDITIONS)})")

    out = Path(args.out)
    classes = [c.strip() for c in args.classes.split(",") if c.strip()]

    # ---- 소스 수집
    if args.stub:
        sources = [(f"stub_{i:05d}", None) for i in range(args.stub)]
        raw_root = None
        print(f"소스   : 절차 생성 stub {args.stub}장  ★ 배관 검증 전용, mAP 무의미")
    else:
        raw_root = Path(args.raw)
        img_dir = raw_root / "images" if (raw_root / "images").is_dir() else raw_root
        files = [p for p in sorted(img_dir.iterdir()) if p.suffix.lower() in IMG_EXT]
        if not files:
            raise SystemExit(f"이미지가 없습니다: {img_dir}")
        sources = [(p.stem, p) for p in files]
        n_lab = sum(1 for s, _ in sources if find_label(raw_root, s))
        print(f"소스   : {raw_root}  이미지 {len(files)}장 / 라벨 {n_lab}개")
        if n_lab == 0:
            print("  ⚠ 라벨이 하나도 없습니다. 학습은 되지만 mAP가 0으로 나옵니다.")

    procs = build_pipelines(args, conditions)
    cfg = SmokeConfig(style_probs=None)

    for cond in conditions:
        for sp in ("train", "val"):
            (out / cond / "images" / sp).mkdir(parents=True, exist_ok=True)
            (out / cond / "labels" / sp).mkdir(parents=True, exist_ok=True)

    counts = {c: {"train": 0, "val": 0} for c in conditions}
    t0 = time.perf_counter()

    for idx, (stem, path) in enumerate(sources):
        rng = np.random.default_rng(seed_of(stem))          # ★ 조건 간 동일 연기

        if path is None:
            base, boxes = make_stub_scene(args.width, args.height, rng)
            label_lines = [f"{c} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}"
                           for c, cx, cy, bw, bh in boxes]
        else:
            base = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if base is None:
                continue
            lp = find_label(raw_root, stem)
            label_lines = lp.read_text().strip().splitlines() if lp else []

        # ★ 화염을 합성하면 **정확한 박스가 공짜로 나옵니다.** 라벨링 없이
        #   조건 비교를 돌릴 수 있습니다. 단, 합성 화염 != 실제 화염이므로
        #   **절대 성능은 못 믿고 조건 간 상대 비교만** 유효합니다
        #   (모든 조건이 같은 화염 렌더링을 공유하므로 비교 자체는 성립).
        fmask = None
        if args.fire_labels:
            base, fbox, fmask = composite_fire(
                base, rng, n_flames=int(rng.integers(1, 3)))
            label_lines = [f"{c} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}"
                           for c, cx, cy, bw, bh in fbox]

        if args.synth_smoke:
            # 화염을 이미 넣었으면 그 마스크를 넘겨 재검출을 막습니다.
            base, _, _ = synthesize(base, rng, cfg, fire_mask_override=fmask)

        split = split_of(stem, args.split)
        for cond in conditions:
            img = procs[cond](base)
            cv2.imwrite(str(out / cond / "images" / split / f"{stem}.jpg"), img,
                        [int(cv2.IMWRITE_JPEG_QUALITY), 95])
            (out / cond / "labels" / split / f"{stem}.txt").write_text(
                "\n".join(label_lines) + ("\n" if label_lines else ""))
            counts[cond][split] += 1

        if (idx + 1) % 50 == 0:
            print(f"  {idx+1}/{len(sources)} ...")

    # ---- data.yaml (조건별)
    for cond in conditions:
        (out / cond / "data.yaml").write_text(
            f"path: {(out / cond).resolve()}\n"
            f"train: images/train\nval: images/val\n"
            f"nc: {len(classes)}\n"
            f"names: [{', '.join(classes)}]\n")

    (out / "manifest.json").write_text(json.dumps({
        "conditions": conditions,
        "classes": classes,
        "split_ratio": args.split,
        "synth_smoke": bool(args.synth_smoke),
        "stub": bool(args.stub),
        "fire_labels": bool(args.fire_labels),
        "raw": str(args.raw) if not args.stub else None,
        "counts": counts,
        "preprocess": {"gamma": args.gamma, "clahe_clip": args.clahe_clip,
                       "clahe_tile": args.clahe_tile,
                       "aod_weights": args.weights, "aod_max_side": args.aod_max_side,
                       "gate_baseline": args.gate_baseline,
                       "gate_threshold": args.gate_threshold},
        "built_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }, indent=2, ensure_ascii=False))

    dt = time.perf_counter() - t0
    print(f"\n생성 완료 ({dt:.1f}s) -> {out}")
    for cond in conditions:
        print(f"  {cond:<10} train {counts[cond]['train']:4d} / val {counts[cond]['val']:4d}")
    print("\n다음: python tools/yolo_pipeline.py train --data", out)


# ---------------------------------------------------------------- train


def cmd_train(args):
    from ultralytics import YOLO

    root = Path(args.data)
    manifest = json.loads((root / "manifest.json").read_text())
    conditions = [c for c in manifest["conditions"]
                  if not args.only or c in args.only.split(",")]

    path = root / "results.json"
    results = json.loads(path.read_text()) if path.exists() else {}

    for cond in conditions:
        runs = []
        for rep in range(args.repeats):
            tag = f"{cond}" if args.repeats == 1 else f"{cond}_r{rep}"
            print(f"\n{'='*60}\n조건: {cond}   반복 {rep+1}/{args.repeats}\n{'='*60}")

            # ★ 매 실행마다 사전학습 가중치에서 새로 시작합니다.
            #   이어서 학습하면 앞 조건/앞 반복이 뒤로 새어 들어갑니다.
            model = YOLO(args.model)
            model.train(
                data=str(root / cond / "data.yaml"),
                epochs=args.epochs, imgsz=args.imgsz, batch=args.batch,
                device=args.device, seed=args.seed + rep, workers=args.workers,
                project=str(root / "runs"), name=tag, exist_ok=True,
                pretrained=True, verbose=False, plots=False,
                # ★ 전처리 효과를 재는 실험이므로 **색 증강을 끕니다.**
                #   hsv 증강이 CLAHE/디헤이즈의 효과를 덮어써서 조건 차이가 사라집니다.
                hsv_h=0.0, hsv_s=0.0, hsv_v=0.0,
            )
            m = model.val(data=str(root / cond / "data.yaml"), device=args.device,
                          verbose=False, plots=False)
            runs.append({"map50": float(m.box.map50), "map": float(m.box.map),
                         "precision": float(m.box.mp), "recall": float(m.box.mr),
                         "seed": args.seed + rep})
            print(f"  {tag}: mAP50 {m.box.map50:.4f}  mAP50-95 {m.box.map:.4f}")

        results[cond] = {"runs": runs}
        path.write_text(json.dumps(results, indent=2))   # 조건마다 즉시 저장
        mu = np.mean([r["map50"] for r in runs])
        sd = np.std([r["map50"] for r in runs])
        print(f"  >> {cond} mAP50 = {mu:.4f} ± {sd:.4f}  (n={len(runs)})")

    print(f"\n저장: {path}")
    cmd_report(argparse.Namespace(data=args.data))


# ---------------------------------------------------------------- report


def cmd_report(args):
    root = Path(args.data)
    path = root / "results.json"
    if not path.exists():
        raise SystemExit(f"{path} 가 없습니다. train 을 먼저 돌리세요.")
    res = json.loads(path.read_text())
    manifest = json.loads((root / "manifest.json").read_text())

    def runs_of(entry):
        """구버전(단일 실행) 결과도 읽습니다."""
        return entry["runs"] if "runs" in entry else [entry]

    order = [c for c in CONDITIONS if c in res]
    stats = {c: {k: (np.mean([r[k] for r in runs_of(res[c])]),
                     np.std([r[k] for r in runs_of(res[c])]))
                 for k in ("map50", "map", "precision", "recall")}
             for c in order}
    n_rep = max(len(runs_of(res[c])) for c in order)
    base = stats.get("raw", {}).get("map50", (None, None))[0]

    label = {"raw": "무처리", "clahe": "감마+CLAHE", "dcp": "DCP+CLAHE",
             "aod": "AOD+CLAHE", "aod_gated": "AOD(게이팅)+CLAHE"}

    print(f"\n조건당 {n_rep}회 반복 · 평균 ± 표준편차")
    print(f"{'조건':<20}{'mAP50':>18}{'mAP50-95':>18}{'정밀도':>10}{'재현율':>10}{'무처리 대비':>12}")
    print("-" * 90)
    for c in order:
        st = stats[c]
        d = f"{st['map50'][0]-base:+9.4f}" if base is not None and c != "raw" else " " * 9
        print(f"{label.get(c, c):<20}"
              f"{st['map50'][0]:11.4f} ±{st['map50'][1]:5.4f}"
              f"{st['map'][0]:11.4f} ±{st['map'][1]:5.4f}"
              f"{st['precision'][0]:10.4f}{st['recall'][0]:10.4f}{d:>12}")

    # ★ 조건 간 차이가 실행 간 편차에 묻히면 결론을 낼 수 없습니다.
    if n_rep > 1 and base is not None:
        worst_sd = max(stats[c]["map50"][1] for c in order)
        gaps = [abs(stats[c]["map50"][0] - base) for c in order if c != "raw"]
        if gaps and max(gaps) < 2 * worst_sd:
            print(f"\n⚠ 조건 간 최대 차이({max(gaps):.4f})가 실행 편차의 2배"
                  f"({2*worst_sd:.4f})에 못 미칩니다.")
            print("  반복을 늘리거나 데이터를 더 모으세요 — 지금 순위는 노이즈일 수 있습니다.")

    if manifest.get("stub"):
        print("\n★ stub 데이터셋입니다 — 이 mAP는 배관 검증용이고 의미가 없습니다.")
    if manifest.get("fire_labels"):
        print("\n★ 합성 화염 라벨입니다 — **조건 간 상대 비교만** 유효합니다.")
        print("  절대 성능은 우리 렌더러의 특징을 학습한 값이라 못 믿습니다.")
        print("  실사 라벨이 나오면 반드시 다시 돌리세요.")

    with (root / "results.csv").open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["condition", "map50_mean", "map50_std",
                    "map50_95_mean", "map50_95_std", "precision", "recall", "n"])
        for c in order:
            st = stats[c]
            w.writerow([c, st["map50"][0], st["map50"][1], st["map"][0], st["map"][1],
                        st["precision"][0], st["recall"][0], len(runs_of(res[c]))])
    print(f"\nCSV: {root/'results.csv'}")


# ---------------------------------------------------------------- CLI


def main():
    ap = argparse.ArgumentParser(description="YOLO 조건별 전처리 비교")
    sub = ap.add_subparsers(dest="cmd", required=True)

    b = sub.add_parser("build", help="조건별 데이터셋 생성")
    b.add_argument("--raw", default=None, help="images/ + labels/ 를 가진 원본 디렉터리")
    b.add_argument("--stub", type=int, default=0, help="라벨 없을 때 배관 검증용 N장 생성")
    b.add_argument("--out", required=True)
    b.add_argument("--conditions", default=",".join(CONDITIONS))
    b.add_argument("--classes", default="fire,smoke")
    b.add_argument("--split", type=float, default=0.2)
    b.add_argument("--synth-smoke", action="store_true",
                   help="원본이 깨끗할 때 연기를 합성해서 넣습니다 (조건 간 동일 연기)")
    b.add_argument("--width", type=int, default=640)
    b.add_argument("--height", type=int, default=480)
    b.add_argument("--weights", default=None, help="AOD-Net 체크포인트 (aod 조건용)")
    b.add_argument("--device", default="auto")
    b.add_argument("--aod-max-side", type=int, default=256)
    b.add_argument("--gate-baseline", type=float, default=None)
    b.add_argument("--gate-threshold", type=float, default=0.15,
                   help="배포 확정값 0.15 (tools/tune_gate.py 로 재튜닝한 값)")
    b.add_argument("--fire-labels", action="store_true",
                   help="화염을 합성해 라벨을 자동 생성 (라벨링 없이 조건 비교 가능). "
                        "실사 라벨이 나오면 반드시 다시 돌리세요")
    b.add_argument("--gamma", type=float, default=1.0)
    b.add_argument("--clahe-clip", type=float, default=2.0)
    b.add_argument("--clahe-tile", type=int, default=8)
    b.set_defaults(func=cmd_build)

    t = sub.add_parser("train", help="조건마다 YOLO 학습 + 평가")
    t.add_argument("--data", required=True)
    t.add_argument("--model", default="yolo26s.pt")
    t.add_argument("--epochs", type=int, default=50)
    t.add_argument("--imgsz", type=int, default=640)
    t.add_argument("--batch", type=int, default=16)
    t.add_argument("--device", default="xpu")
    t.add_argument("--seed", type=int, default=0)
    t.add_argument("--workers", type=int, default=4)
    t.add_argument("--repeats", type=int, default=3,
                   help="조건당 반복 횟수. phase1 HANDOVER 7-5 요구사항 — 데이터가 적어 "
                        "실행 간 편차가 크므로 평균±표준편차로 보고해야 합니다")
    t.add_argument("--only", default=None, help="특정 조건만 (쉼표 구분)")
    t.set_defaults(func=cmd_train)

    r = sub.add_parser("report", help="mAP 비교표")
    r.add_argument("--data", required=True)
    r.set_defaults(func=cmd_report)

    args = ap.parse_args()
    if args.cmd == "build" and not args.raw and not args.stub:
        raise SystemExit("--raw 또는 --stub 중 하나가 필요합니다")
    args.func(args)


if __name__ == "__main__":
    main()
