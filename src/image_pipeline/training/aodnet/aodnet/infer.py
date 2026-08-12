#!/usr/bin/env python3
"""
추론 — 단일 이미지 / 디렉터리 / 영상.

    python -m aodnet.infer --weights runs/base/best.pt -i smoke.jpg -o out.jpg
    python -m aodnet.infer --weights runs/base/best.pt -i frames/ -o out/ --side-by-side
    python -m aodnet.infer --weights runs/base/best.pt -i clip.mp4 -o clip_out.mp4

핵심은 아래 `AODNetDehazer` 클래스입니다.
phase1 `image_pipeline.dehaze.DarkChannelDehazer` 와 **같은 인터페이스**
(`process(bgr_uint8) -> bgr_uint8`)로 맞춰놨습니다. 그래서 ROS 노드 코드를
한 줄도 고치지 않고 갈아끼울 수 있습니다:

    from aodnet.infer import AODNetDehazer
    pipeline = Pipeline(mode="full", dehazer=AODNetDehazer("runs/base/best.pt"))
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import torch

from aodnet.data import IMG_EXT, bgr_to_tensor, tensor_to_bgr
from aodnet.device import device_name, pick_device, synchronize
from aodnet.model import load_aodnet

VIDEO_EXT = (".mp4", ".avi", ".mov", ".mkv")


class AODNetDehazer:
    """학습된 AOD-Net으로 연기를 제거합니다.

    DCP 디헤이저와 교체 가능한 API. 차이점은 주석으로 남겨둡니다.

    strength : 복원 세기 0~1. K를 그대로 쓰지 않고
                 K' = 1 + strength·(K - 1)
               로 줄입니다. 1.0이 학습된 그대로. 실제 영상이 학습 분포보다
               연기가 옅으면 과복원(색 튐)이 나는데, 재학습 없이 이 값만
               0.7 근처로 내려서 바로 잡을 수 있습니다. 현장 튜닝용 손잡이.
    max_side : 이 픽셀 수를 넘으면 축소해서 추론하고 결과만 되키웁니다.
               AOD-Net은 완전 합성곱이라 해상도 제약이 없지만, K는 저주파
               신호라 축소본에서 뽑아도 손실이 거의 없습니다 (DCP의 scale과
               같은 논리). 0이면 원본 해상도 그대로.
    """

    def __init__(self,
                 weights: str | Path,
                 device: str = "auto",
                 strength: float = 1.0,
                 max_side: int = 0,
                 half: bool = False):
        self.device = pick_device(device)

        ck = torch.load(str(weights), map_location=self.device)
        state = ck.get("model", ck)
        # 구조(learn_b)는 체크포인트에서 판별합니다 — model.py::load_aodnet 참고.
        self.model = load_aodnet(state, self.device).eval()

        # half는 XPU/CUDA에서만. CPU fp16은 오히려 몇 배 느립니다.
        self.half = bool(half) and self.device.type in ("xpu", "cuda")
        if self.half:
            self.model.half()

        self.strength = float(strength)
        self.max_side = int(max_side)

        self.last_k: np.ndarray | None = None      # 시각화용 K 맵
        self.timings: dict[str, float] = {"infer": 0.0, "total": 0.0}

        # 첫 호출은 커널 컴파일 때문에 수백 ms 걸립니다. 미리 태워둡니다.
        # ROS 노드에서 이걸 빼면 첫 프레임에 타임아웃이 납니다.
        self._warmup()

    @torch.no_grad()
    def _warmup(self, size: tuple[int, int] = (240, 320)) -> None:
        dummy = torch.zeros(1, 3, *size, device=self.device,
                            dtype=torch.half if self.half else torch.float32)
        self.model(dummy)
        synchronize(self.device)

    def reset_state(self) -> None:
        """DCP 디헤이저와 API를 맞추기 위한 no-op.

        AOD-Net은 프레임 간 상태가 없습니다 — 이게 DCP 대비 실질적 장점입니다.
        DCP는 A를 매 프레임 장면에서 추정해서 로봇이 움직이면 밝기가 깜빡였고,
        그걸 막으려고 a_smoothing(EMA)을 넣어야 했습니다. AOD-Net의 K는
        학습된 가중치로 결정되므로 같은 입력 -> 항상 같은 출력입니다.
        """

    # ------------------------------------------------------------------

    @torch.no_grad()
    def process(self, bgr: np.ndarray) -> np.ndarray:
        """bgr uint8 -> bgr uint8. 입력 배열은 변경하지 않습니다."""
        t0 = time.perf_counter()
        h, w = bgr.shape[:2]

        # ★ 축소 추론은 **K만 축소본에서 뽑고, 복원식은 원본 해상도에서** 계산합니다.
        #
        #   K는 저주파 신호(연기 농도 지도)라 축소본에서 뽑아 업샘플해도 손실이
        #   거의 없습니다. 반면 출력 이미지를 업샘플하면 **장면의 고주파가 통째로
        #   날아갑니다** — 얇은 선, 글자, 윤곽이 뭉개집니다.
        #   phase1 DCP의 scale 옵션이 투과율 t만 업샘플하는 것과 같은 논리입니다.
        scaled = self.max_side > 0 and max(h, w) > self.max_side

        x_full = bgr_to_tensor(bgr).unsqueeze(0).to(self.device)
        if self.half:
            x_full = x_full.half()

        if scaled:
            s = self.max_side / max(h, w)
            small = cv2.resize(bgr, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
            x_small = bgr_to_tensor(small).unsqueeze(0).to(self.device)
            if self.half:
                x_small = x_small.half()
            k = self.model.estimate_k(x_small)
            k = torch.nn.functional.interpolate(k, size=(h, w), mode="bilinear",
                                                align_corners=False)
        else:
            k = self.model.estimate_k(x_full)

        if self.strength != 1.0:
            k = 1.0 + self.strength * (k - 1.0)

        out = torch.relu(k * x_full - k + self.model.b)

        synchronize(self.device)
        self.timings["infer"] = (time.perf_counter() - t0) * 1000.0

        self.last_k = k[0].float().mean(0).cpu().numpy()
        result = tensor_to_bgr(out[0].float())

        self.timings["total"] = (time.perf_counter() - t0) * 1000.0
        return result

    def k_visualization(self) -> np.ndarray | None:
        """마지막 K 맵을 컬러맵 이미지로. 밝을수록 강하게 복원한 영역."""
        if self.last_k is None:
            return None
        k = self.last_k
        norm = (k - k.min()) / (np.ptp(k) + 1e-6)   # numpy 2.x는 ndarray.ptp() 제거됨
        return cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_INFERNO)


# ---------------------------------------------------------------- CLI


def _stack(hazy, clean, side_by_side: bool):
    if not side_by_side:
        return clean
    pad = np.full((hazy.shape[0], 4, 3), 255, np.uint8)
    return np.hstack([hazy, pad, clean])


def run_image(dehazer, src: Path, dst: Path, side_by_side: bool):
    img = cv2.imread(str(src), cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError(f"읽기 실패: {src}")
    out = dehazer.process(img)
    dst.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(dst), _stack(img, out, side_by_side))
    print(f"{src.name} -> {dst}  ({dehazer.timings['total']:.1f} ms)")


def run_video(dehazer, src: Path, dst: Path, side_by_side: bool):
    cap = cv2.VideoCapture(str(src))
    if not cap.isOpened():
        raise RuntimeError(f"영상 열기 실패: {src}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    ow = w * 2 + 4 if side_by_side else w

    dst.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(dst), cv2.VideoWriter_fourcc(*"mp4v"), fps, (ow, h))

    n, total_ms = 0, 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        out = dehazer.process(frame)
        total_ms += dehazer.timings["total"]
        writer.write(_stack(frame, out, side_by_side))
        n += 1
    cap.release()
    writer.release()
    print(f"{n} 프레임 -> {dst}  (평균 {total_ms/max(n,1):.1f} ms/frame, "
          f"{1000*n/max(total_ms,1e-6):.1f} FPS)")


def main():
    ap = argparse.ArgumentParser(description="AOD-Net 추론")
    ap.add_argument("--weights", required=True)
    ap.add_argument("-i", "--input", required=True, help="이미지 / 디렉터리 / 영상")
    ap.add_argument("-o", "--output", required=True)
    ap.add_argument("--device", default="auto")
    ap.add_argument("--strength", type=float, default=1.0)
    ap.add_argument("--max-side", type=int, default=0)
    ap.add_argument("--half", action="store_true")
    ap.add_argument("--side-by-side", action="store_true", help="원본|결과 나란히 저장")
    args = ap.parse_args()

    dehazer = AODNetDehazer(args.weights, args.device, args.strength,
                            args.max_side, args.half)
    print(f"device : {dehazer.device} ({device_name(dehazer.device)})")

    src, dst = Path(args.input), Path(args.output)

    if src.is_dir():
        files = sorted(p for p in src.iterdir() if p.suffix.lower() in IMG_EXT)
        if not files:
            raise SystemExit(f"이미지가 없습니다: {src}")
        dst.mkdir(parents=True, exist_ok=True)
        for f in files:
            run_image(dehazer, f, dst / f.name, args.side_by_side)
    elif src.suffix.lower() in VIDEO_EXT:
        run_video(dehazer, src, dst, args.side_by_side)
    else:
        run_image(dehazer, src, dst, args.side_by_side)


if __name__ == "__main__":
    main()
