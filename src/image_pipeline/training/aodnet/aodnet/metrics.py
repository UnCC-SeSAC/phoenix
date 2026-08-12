#!/usr/bin/env python3
"""
평가 지표 — PSNR / SSIM.

scikit-image를 안 쓴 이유: GPU 텐서를 CPU로 내려야 해서 검증이 느려집니다.
여기 구현은 torch 텐서 위에서 그대로 돌아갑니다.

★ 이 프로젝트에서 PSNR을 믿어도 되는 범위
   phase1 CLAUDE.md 에 "PSNR로 clipLimit 튜닝 금지"라고 적혀 있습니다.
   맞는 말이고, 여기서도 절반만 뒤집힙니다.

   - 믿어도 되는 경우: **정답 J를 우리가 만든 합성 쌍**에서의 PSNR.
     복원 목표가 명확히 정의되어 있으므로 학습 진행 판단에 유효합니다.
   - 믿으면 안 되는 경우: 실제 연기 영상. 정답이 없으니 "덜 건드릴수록"
     원본과 비슷해 보이는 함정이 그대로 살아 있습니다.
   - 최종 판정은 여전히 **후단 YOLO mAP**입니다. tools/compare.py 가
     PSNR/SSIM과 함께 대비·엣지 지표를 같이 뱉는 것도 그래서입니다.
"""

from __future__ import annotations

import torch
import torch.nn.functional as F


def psnr(pred: torch.Tensor, target: torch.Tensor, max_val: float = 1.0) -> torch.Tensor:
    """배치 평균 PSNR(dB). pred/target: (N,C,H,W) 0~1."""
    pred = pred.clamp(0, max_val)
    mse = F.mse_loss(pred, target, reduction="none").flatten(1).mean(1)
    # mse=0 이면 inf가 되어 평균이 오염됩니다. 하한을 둡니다.
    mse = mse.clamp_min(1e-10)
    return (10.0 * torch.log10(max_val ** 2 / mse)).mean()


def _gaussian_window(size: int, sigma: float, device, dtype) -> torch.Tensor:
    coords = torch.arange(size, device=device, dtype=dtype) - (size - 1) / 2.0
    g = torch.exp(-(coords ** 2) / (2 * sigma ** 2))
    g = g / g.sum()
    return g.outer(g)


def ssim(pred: torch.Tensor, target: torch.Tensor,
         window_size: int = 11, sigma: float = 1.5, max_val: float = 1.0) -> torch.Tensor:
    """배치 평균 SSIM. 채널별로 계산 후 평균 (skimage의 channel_axis 방식과 동일)."""
    pred = pred.clamp(0, max_val)
    c = pred.shape[1]
    win = _gaussian_window(window_size, sigma, pred.device, pred.dtype)
    win = win.expand(c, 1, window_size, window_size).contiguous()
    pad = window_size // 2

    def blur(x):
        return F.conv2d(x, win, padding=pad, groups=c)

    mu1, mu2 = blur(pred), blur(target)
    mu1_sq, mu2_sq, mu1_mu2 = mu1 * mu1, mu2 * mu2, mu1 * mu2

    sigma1 = blur(pred * pred) - mu1_sq
    sigma2 = blur(target * target) - mu2_sq
    sigma12 = blur(pred * target) - mu1_mu2

    c1 = (0.01 * max_val) ** 2
    c2 = (0.03 * max_val) ** 2

    num = (2 * mu1_mu2 + c1) * (2 * sigma12 + c2)
    den = (mu1_sq + mu2_sq + c1) * (sigma1 + sigma2 + c2)
    return (num / den).flatten(1).mean(1).mean()


# ---------------------------------------------------------------- 무참조 지표


def contrast_gain(pred: torch.Tensor, hazy: torch.Tensor) -> torch.Tensor:
    """대비 향상률. 정답이 없는 실제 영상에서 쓰는 무참조 지표.

    1.0 = 변화 없음. 2.0 = 표준편차가 두 배. 너무 크면(>4) 과복원으로
    노이즈까지 증폭된 상태입니다.
    """
    s_pred = pred.clamp(0, 1).flatten(1).std(1)
    s_hazy = hazy.flatten(1).std(1)
    return (s_pred / s_hazy.clamp_min(1e-6)).mean()


def edge_density(x: torch.Tensor) -> torch.Tensor:
    """Sobel 엣지 강도 평균. 디헤이즈로 살아난 윤곽의 양을 봅니다.

    YOLO 성능과 상관이 높은 편이라 PSNR보다 실전 감이 좋습니다.
    (단, 노이즈도 엣지로 잡히므로 contrast_gain과 **함께** 보세요.)
    """
    gray = x.clamp(0, 1).mean(1, keepdim=True)
    kx = torch.tensor([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]],
                      dtype=x.dtype, device=x.device).view(1, 1, 3, 3)
    ky = kx.transpose(2, 3)
    gx = F.conv2d(gray, kx, padding=1)
    gy = F.conv2d(gray, ky, padding=1)
    return torch.sqrt(gx ** 2 + gy ** 2 + 1e-12).flatten(1).mean(1).mean()
