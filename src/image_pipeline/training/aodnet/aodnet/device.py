#!/usr/bin/env python3
"""디바이스 선택 — Intel Arc(XPU) / NVIDIA(CUDA) / CPU 자동 판별.

이 프로젝트는 Intel Arc B580에서 개발했습니다. PyTorch 2.5+ 부터 `torch.xpu`가
정식 백엔드라 CUDA 코드와 거의 1:1로 대응되지만, 아래 두 개는 다릅니다.

  - `torch.cuda.amp` 대신 `torch.amp.autocast('xpu', ...)`
  - `pin_memory=True` + `num_workers>0` 조합이 XPU에서 이득이 거의 없습니다.
    (호스트→디바이스 복사가 병목이 아니라 커널 런치가 병목)

설치:
    pip install torch torchvision --index-url https://download.pytorch.org/whl/xpu
"""

from __future__ import annotations

import torch


def pick_device(prefer: str = "auto") -> torch.device:
    """prefer: 'auto' | 'xpu' | 'cuda' | 'cpu'"""
    if prefer != "auto":
        return torch.device(prefer)

    if hasattr(torch, "xpu") and torch.xpu.is_available():
        return torch.device("xpu")
    if torch.cuda.is_available():
        return torch.device("cuda")
    return torch.device("cpu")


def device_name(device: torch.device) -> str:
    if device.type == "xpu":
        return torch.xpu.get_device_properties(device.index or 0).name
    if device.type == "cuda":
        return torch.cuda.get_device_name(device.index or 0)
    return "CPU"


def synchronize(device: torch.device) -> None:
    """벤치마크에서 필수. 안 부르면 GPU 커널이 큐에만 쌓인 채로 시간을 재게 됩니다."""
    if device.type == "xpu":
        torch.xpu.synchronize()
    elif device.type == "cuda":
        torch.cuda.synchronize()


def empty_cache(device: torch.device) -> None:
    if device.type == "xpu":
        torch.xpu.empty_cache()
    elif device.type == "cuda":
        torch.cuda.empty_cache()


if __name__ == "__main__":
    d = pick_device()
    print(f"device : {d}")
    print(f"name   : {device_name(d)}")
    print(f"torch  : {torch.__version__}")
