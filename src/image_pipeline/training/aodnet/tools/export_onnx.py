#!/usr/bin/env python3
"""
배포용 ONNX 내보내기 — Raspberry Pi / Hailo 등 torch 없는 환경용.

    python tools/export_onnx.py --weights runs/mine2/best.pt -o deploy/aodnet_k.onnx

기본은 **K만 출력**하는 그래프입니다(--output k). 왜 J가 아니라 K인가:

    복원식 J = K·I - K + b 를 **원본 해상도에서** 계산해야 장면의 고주파가
    보존됩니다. K는 저주파(연기 농도 지도)라 축소본에서 뽑아 업샘플해도
    손실이 거의 없고, 오히려 고주파 잡음이 걸러져 품질이 올라갑니다.
    (실측: K를 256x192에서 뽑으면 640x480 원본 대비 PSNR +0.23dB, 비용 1/4)

    그래서 신경망은 K까지만 하고, 복원식 3줄은 numpy로 원본 해상도에서 합니다.
    deploy/aod_lite.py 가 그 구조입니다.

--output full 은 J까지 포함한 그래프입니다. 축소 추론을 안 쓰거나, 가속기에
전체 그래프를 올려야 할 때 씁니다.

    ★ Hailo 등 INT8 가속기에 올린다면 반드시 fp32와 수치를 비교하세요.
      J = K·I - K + b 의 뺄셈은 비슷한 크기의 두 텐서를 빼므로 INT8에서
      유효 비트가 급감합니다. 이 모델은 파라미터가 1,761개뿐이라 특히 민감합니다.
      "돌아간다"가 아니라 tools/compare.py 의 PSNR/SSIM/엣지/대비이득으로
      판정하세요.
"""

from __future__ import annotations

import argparse
import collections
import sys
import warnings
from pathlib import Path

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from aodnet.model import load_aodnet          # noqa: E402


class KOnly(torch.nn.Module):
    """K(x)만 내보내는 래퍼."""

    def __init__(self, model):
        super().__init__()
        self.model = model

    def forward(self, x):
        return self.model.estimate_k(x)


def parse_size(text: str) -> tuple[int, int]:
    try:
        w, h = (int(v) for v in text.lower().split("x"))
        return w, h
    except Exception:
        raise SystemExit(f"--size 는 'WxH' 형식이어야 합니다 (받은 값: {text!r})")


def main():
    ap = argparse.ArgumentParser(description="AOD-Net ONNX 내보내기")
    ap.add_argument("--weights", required=True)
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--size", default="256x192",
                    help="내보낼 때의 예시 크기 'WxH'. --dynamic 이면 실행 시 바뀔 수 있습니다")
    ap.add_argument("--output", choices=("k", "full"), default="k",
                    help="k = K맵만(권장) / full = J까지")
    ap.add_argument("--dynamic", action="store_true", default=True,
                    help="H/W를 동적 축으로 (기본 켜짐). OpenCV DNN에서 동작 확인됨")
    ap.add_argument("--static", dest="dynamic", action="store_false",
                    help="고정 크기로 내보내기 (일부 가속기 컴파일러가 요구)")
    ap.add_argument("--opset", type=int, default=13)
    args = ap.parse_args()

    warnings.filterwarnings("ignore")
    w, h = parse_size(args.size)

    ck = torch.load(args.weights, map_location="cpu")
    net = load_aodnet(ck.get("model", ck)).eval()
    graph = KOnly(net) if args.output == "k" else net

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)

    dummy = torch.randn(1, 3, h, w)
    name = "k" if args.output == "k" else "output"
    dyn = {"input": {2: "height", 3: "width"}, name: {2: "height", 3: "width"}}

    torch.onnx.export(graph, (dummy,), str(out),
                      input_names=["input"], output_names=[name],
                      opset_version=args.opset, dynamo=False,
                      dynamic_axes=dyn if args.dynamic else None)

    # ---------------------------------------------------------------- 검증
    try:
        import onnx
        model = onnx.load(str(out))
        onnx.checker.check_model(model)
        ops = collections.Counter(n.op_type for n in model.graph.node)
        op_summary = "  ".join(f"{k}x{v}" for k, v in ops.most_common())
    except ImportError:
        op_summary = "(onnx 미설치 — 연산자 확인 생략)"

    # torch 결과와 실제로 같은 값이 나오는지. 여기서 안 잡으면 Pi에 올린 뒤
    # "왜 색이 이상하지"로 발견하게 됩니다.
    parity = "확인 못 함"
    try:
        import cv2
        rng = np.random.default_rng(0)
        x = rng.random((1, 3, h, w), dtype=np.float32)
        with torch.no_grad():
            ref = graph(torch.from_numpy(x)).numpy()
        dnn = cv2.dnn.readNetFromONNX(str(out))
        dnn.setInput(x)
        got = dnn.forward()
        diff = float(np.abs(ref - got).max())
        parity = f"OpenCV DNN 최대 오차 {diff:.2e} " + ("✅" if diff < 1e-4 else "⚠ 큼")
    except Exception as e:
        parity = f"검증 실패: {str(e)[:60]}"

    print(f"저장   : {out}  ({out.stat().st_size/1024:.1f} KB)")
    print(f"출력   : {args.output}  ({'K 맵만' if args.output=='k' else 'J까지'})")
    print(f"크기   : {w}x{h}  {'(동적 H/W)' if args.dynamic else '(고정)'}")
    print(f"연산자 : {op_summary}")
    print(f"수치   : {parity}")

    if args.output == "k":
        print("\n사용법 (torch 없이):")
        print("    from aod_lite import AODNetLite")
        print(f"    aod = AODNetLite('{out.name}', k_size=({w}, {h}))")
        print("    clean = aod.process(frame)")


if __name__ == "__main__":
    main()
