#!/usr/bin/env python3
"""
게이팅 임계값 튜닝 — "연기가 있을 때만 디헤이즈"의 기준값을 정합니다.

    python tools/tune_gate.py --weights runs/mine2/best.pt \
        --val data/mine2_val --test data/mine2_test --clean-dir ../frame_cut/data

왜 필요한가
-----------
측정해 보면 AOD-Net은 **모든 입력에서 이득이 아닙니다.** 연기가 짙을수록
크게 이득이고, 검은 연기(sooty)처럼 연기가 화면을 별로 바꾸지 않는 경우엔
오히려 손해입니다. 그래서 "항상 켜기"보다 "연기가 있을 때만 켜기"가 낫습니다.

판단 근거는 phase1의 연기 지표입니다.

    idx, _ = estimate_haze_index(frame)       # 다크채널 기반 0~1
    rel    = relative_haze(idx, baseline)     # 기준선 대비 상대 농도

★ 절대값을 쓰면 안 됩니다. 연기가 없어도 0.4~0.47이 나옵니다(콘크리트·회색 벽은
  원래 다크 채널이 높음). 그래서 **연기 없는 프레임에서 기준선을 먼저 재고**
  그 차이를 봅니다. --clean-dir 이 그 용도입니다.

★ 임계값은 반드시 **검증셋에서** 정하고 테스트셋에서 확인합니다.
  테스트셋에서 최적값을 고르면 그 숫자는 이미 테스트셋에 적합된 값이라
  현장 성능을 대변하지 못합니다.

★ 영상에 적용할 때는 EMA를 꼭 거세요.
  지표가 임계값 근처에서 흔들리면 프레임마다 처리/무처리가 뒤집혀 화면이
  깜빡입니다(실측: 60프레임에서 27회 전환 -> EMA 적용 시 2회).
  phase1 DCP의 a_smoothing과 같은 문제입니다. 아래 SmokeGate 참고.
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
sys.path.insert(0, str(ROOT / "deploy"))

from aodnet.data import bgr_to_tensor, list_images       # noqa: E402
from aodnet.infer import AODNetDehazer                   # noqa: E402
from aodnet.metrics import psnr                          # noqa: E402
from aod_lite import SmokeGate                           # noqa: E402  (정의는 한 곳에만)


def load_haze_index(phase1_dir: str | None):
    """phase1의 연기 지표 함수를 가져옵니다."""
    for c in ([Path(phase1_dir)] if phase1_dir else []) + [ROOT.parents[1] / "ros" / "image_pipeline"]:
        if (c / "image_pipeline" / "autotune.py").exists():
            sys.path.insert(0, str(c.resolve()))
            from image_pipeline.autotune import estimate_haze_index, relative_haze
            return estimate_haze_index, relative_haze
    raise SystemExit("phase1/image_pipeline/autotune.py 를 찾지 못했습니다 (--phase1-dir)")


def measure_baseline(clean_dir: str, index_fn, limit: int = 60) -> float:
    """연기 없는 프레임들의 지표 중앙값 = 기준선.

    평균이 아니라 중앙값입니다. 몇 장이 실수로 연기가 껴 있어도 흔들리지 않게.
    """
    files = list_images(clean_dir)[:limit]
    if not files:
        raise SystemExit(f"깨끗한 프레임이 없습니다: {clean_dir}")
    vals = []
    for f in files:
        img = cv2.imread(str(f), cv2.IMREAD_COLOR)
        if img is not None:
            vals.append(index_fn(img)[0])
    return float(np.median(vals))


def evaluate(root: Path, dehazer, gate: SmokeGate):
    """샘플별 (상대농도, 무처리 PSNR, AOD PSNR, 스타일) 을 모읍니다."""
    styles = {}
    meta = root / "meta.json"
    if meta.exists():
        styles = {r["name"]: r["style"] for r in json.loads(meta.read_text())["records"]}

    files = list_images(root / "hazy")
    if not files:
        raise SystemExit(f"{root/'hazy'} 가 비었습니다.")

    warm = cv2.imread(str(files[0]), cv2.IMREAD_COLOR)
    if warm is not None:                      # 해상도별 커널 컴파일을 미리 태움
        dehazer.process(warm)

    rel, off, on, st = [], [], [], []
    for f in files:
        hazy = cv2.imread(str(f), cv2.IMREAD_COLOR)
        clear = cv2.imread(str(root / "clear" / f.name), cv2.IMREAD_COLOR)
        if hazy is None or clear is None:
            continue
        c = bgr_to_tensor(clear).unsqueeze(0)
        rel.append(gate.relative(hazy))
        off.append(psnr(bgr_to_tensor(hazy).unsqueeze(0), c).item())
        on.append(psnr(bgr_to_tensor(dehazer.process(hazy)).unsqueeze(0), c).item())
        st.append(styles.get(f.name, "?"))
    return (np.array(rel), np.array(off), np.array(on), np.array(st))


def gated_mean(rel, off, on, th):
    return float(np.where(rel >= th, on, off).mean())


def main():
    ap = argparse.ArgumentParser(description="게이팅 임계값 튜닝")
    ap.add_argument("--weights", required=True)
    ap.add_argument("--val", required=True, help="임계값을 정할 검증셋 root")
    ap.add_argument("--test", default=None, help="확인용 테스트셋 root (선택)")
    ap.add_argument("--clean-dir", required=True, help="연기 없는 프레임 (기준선 측정용)")
    ap.add_argument("--phase1-dir", default=None)
    ap.add_argument("--device", default="auto")
    ap.add_argument("--baseline", type=float, default=None, help="직접 지정 시 측정 생략")
    ap.add_argument("--max-side", type=int, default=0,
                   help="★ 배포 설정과 맞추세요. K 추정 해상도가 다르면 결과도 달라져"
                        "튜닝한 임계값이 실제 배포에서 최적이 아니게 됩니다")
    ap.add_argument("--step", type=float, default=0.05)
    args = ap.parse_args()

    index_fn, _ = load_haze_index(args.phase1_dir)
    baseline = args.baseline if args.baseline is not None else \
        measure_baseline(args.clean_dir, index_fn)
    print(f"기준선(연기 없는 프레임 중앙값) = {baseline:.4f}")

    # 평가 중에는 EMA를 끕니다 — 정지영상이라 프레임 순서에 의미가 없습니다.
    gate = SmokeGate(baseline, smoothing=0.0)
    dehazer = AODNetDehazer(args.weights, args.device, max_side=args.max_side)
    print(f"모델 : {args.weights} ({dehazer.device})"
          f"{f'  K={args.max_side}' if args.max_side else '  K=원본'}\n")

    rel_v, off_v, on_v, st_v = evaluate(Path(args.val), dehazer, gate)
    print(f"── 검증셋 {Path(args.val).name} ({len(rel_v)}장) 에서 임계값 탐색")
    print(f"{'임계값':>8}{'적용 수':>9}{'평균 PSNR':>11}{'무처리 대비':>12}")
    base_off = off_v.mean()
    print(f"{'항상 끔':>8}{0:9d}{base_off:11.2f}{0.0:+12.2f}")
    print(f"{'항상 켬':>8}{len(rel_v):9d}{on_v.mean():11.2f}{on_v.mean()-base_off:+12.2f}")

    best_th, best_gain = 0.0, on_v.mean() - base_off
    for th in np.arange(args.step, 0.85 + 1e-9, args.step):
        m = gated_mean(rel_v, off_v, on_v, th)
        gain = m - base_off
        mark = ""
        if gain > best_gain:
            best_th, best_gain, mark = float(th), gain, "  <-"
        print(f"{th:8.2f}{int((rel_v >= th).sum()):9d}{m:11.2f}{gain:+12.2f}{mark}")

    print(f"\n★ 선택된 임계값 = {best_th:.2f}  (검증 이득 {best_gain:+.2f} dB)\n")

    for tag, data in (("검증", (rel_v, off_v, on_v, st_v)),
                      *((("테스트", evaluate(Path(args.test), dehazer, gate)),)
                        if args.test else ())):
        rel, off, on, st = data
        g = gated_mean(rel, off, on, best_th)
        print(f"── {tag}셋 ({len(rel)}장)")
        print(f"   {'무처리':<14}{off.mean():8.2f}")
        print(f"   {'항상 켬':<14}{on.mean():8.2f}{on.mean()-off.mean():+9.2f} dB")
        print(f"   {'게이팅':<14}{g:8.2f}{g-off.mean():+9.2f} dB")
        for s in sorted(set(st)):
            if s == "?":
                continue
            m = st == s
            used = int((rel[m] >= best_th).sum())
            print(f"     {s:<10} rel {rel[m].mean():.3f}  무처리 {off[m].mean():6.2f}  "
                  f"AOD {on[m].mean():6.2f}  ({used}/{m.sum()}장 처리)")
        print()

    print("영상에 적용할 때는 EMA를 켜세요 (깜빡임 방지):")
    print(f"    gate = SmokeGate(baseline={baseline:.4f}, "
          f"threshold={best_th:.2f}, smoothing=0.7)")
    print("    if gate.should_process(frame): frame = aod.process(frame)")


if __name__ == "__main__":
    main()
