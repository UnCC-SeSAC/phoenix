#!/usr/bin/env python3
"""
Dataset — 학습 쌍 (연기 I, 정답 J) 공급.

두 가지 모드를 지원합니다.

  1) on-the-fly (기본)
     매 __getitem__ 마다 연기를 새로 합성합니다. 같은 깨끗한 프레임이라도
     매번 다른 β·A·plume이 걸리므로 데이터 증강이 공짜로 따라옵니다.
     디스크도 안 먹습니다.
     ★ 대신 **재현이 안 됩니다.** 검증셋에는 쓰면 안 됩니다.

  2) 사전 생성 (paired 디렉터리)
     tools/make_dataset.py 로 미리 만든 hazy/clear 쌍을 읽습니다.
     검증·평가는 반드시 이쪽. 매 에폭 값이 흔들리면 개선인지 노이즈인지
     구분할 수 없습니다.

색 공간: 디스크는 OpenCV BGR, 텐서는 **RGB**입니다. model.py 주석 참고.
"""

from __future__ import annotations

import os
from pathlib import Path

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset

from aodnet.synth import SmokeConfig, make_scene, synthesize

IMG_EXT = (".jpg", ".jpeg", ".png", ".bmp", ".webp")


def bgr_to_tensor(bgr: np.ndarray) -> torch.Tensor:
    """uint8 BGR (H,W,3) -> float32 RGB (3,H,W), 0~1."""
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    return torch.from_numpy(rgb.transpose(2, 0, 1)).contiguous()


def tensor_to_bgr(t: torch.Tensor) -> np.ndarray:
    """float32 RGB (3,H,W) -> uint8 BGR (H,W,3). 여기서 처음으로 0~1 클리핑."""
    arr = t.detach().float().clamp(0, 1).cpu().numpy().transpose(1, 2, 0)
    return cv2.cvtColor((arr * 255.0 + 0.5).astype(np.uint8), cv2.COLOR_RGB2BGR)


def list_images(root: str | os.PathLike) -> list[Path]:
    root = Path(root)
    return sorted(p for p in root.rglob("*") if p.suffix.lower() in IMG_EXT)


# ---------------------------------------------------------------- on-the-fly


class SyntheticSmokeDataset(Dataset):
    """깨끗한 프레임 -> 매번 새 연기를 합성해 쌍을 만듭니다.

    clear_dir : 실제 깨끗한 프레임 디렉터리. None이면 절차적 장면 생성.
    depth_dir : (선택) 파일명이 clear와 같은 depth 맵 디렉터리.
                16bit png(mm) 또는 8bit png 모두 허용. 있으면 물리적으로
                정확한 t를 만들 수 있습니다 — 있으면 반드시 쓰세요.
    patch     : 학습 패치 한 변. AOD-Net은 완전 합성곱이라 학습은 작은
                패치로, 추론은 원본 해상도로 해도 됩니다. 패치 학습이
                배치를 키울 수 있어 훨씬 빠릅니다.
    """

    def __init__(self,
                 clear_dir: str | None = None,
                 depth_dir: str | None = None,
                 length: int = 4000,
                 patch: int = 240,
                 cfg: SmokeConfig | None = None,
                 seed: int | None = None,
                 scene_size: tuple[int, int] = (640, 480)):
        self.files = list_images(clear_dir) if clear_dir else []
        if clear_dir and not self.files:
            raise FileNotFoundError(f"깨끗한 이미지가 없습니다: {clear_dir}")

        self.depth_dir = Path(depth_dir) if depth_dir else None
        self.length = int(length)
        self.patch = int(patch)
        self.cfg = cfg or SmokeConfig()
        self.scene_size = scene_size
        self.seed = seed

        # 캐시: 같은 파일을 매번 디코딩하면 GPU가 굶습니다.
        self._cache: dict[int, np.ndarray] = {}

    def __len__(self) -> int:
        return self.length

    # --------------------------------------------------------------

    def _load_depth(self, img_path: Path) -> np.ndarray | None:
        if self.depth_dir is None:
            return None
        for ext in (".png", ".tif", ".tiff", ".npy"):
            p = self.depth_dir / (img_path.stem + ext)
            if not p.exists():
                continue
            if ext == ".npy":
                d = np.load(p).astype(np.float32)
            else:
                d = cv2.imread(str(p), cv2.IMREAD_UNCHANGED)
                if d is None:
                    continue
                d = d.astype(np.float32)
            # 0은 RealSense의 '측정 실패'입니다. 그대로 쓰면 그 픽셀만
            # 연기가 0이 되어 학습 데이터에 구멍이 뚫립니다. 최댓값으로 채웁니다.
            valid = d > 0
            if not valid.any():
                return None
            d[~valid] = d[valid].max()
            d = d / (np.percentile(d[valid], 99) + 1e-6)
            return np.clip(d, 0.0, 1.0)
        return None

    def _get_clear(self, rng: np.random.Generator):
        if not self.files:
            return make_scene(self.scene_size[0], self.scene_size[1], rng), None

        idx = int(rng.integers(0, len(self.files)))
        path = self.files[idx]
        img = self._cache.get(idx)
        if img is None:
            img = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if img is None:
                raise RuntimeError(f"읽기 실패: {path}")
            if len(self._cache) < 512:
                self._cache[idx] = img
        return img, self._load_depth(path)

    def _crop(self, *arrays, rng: np.random.Generator):
        h, w = arrays[0].shape[:2]
        p = self.patch
        if p <= 0 or (h <= p and w <= p):
            return arrays
        if h < p or w < p:   # 패치보다 작은 이미지는 키워서 맞춥니다
            s = p / min(h, w)
            arrays = [cv2.resize(a, None, fx=s, fy=s, interpolation=cv2.INTER_LINEAR)
                      for a in arrays]
            h, w = arrays[0].shape[:2]
        y = int(rng.integers(0, h - p + 1))
        x = int(rng.integers(0, w - p + 1))
        return [a[y:y + p, x:x + p] for a in arrays]

    def __getitem__(self, idx: int):
        # 워커별 시드: seed가 주어지면 (seed, idx) 결정론, 아니면 매번 다름.
        if self.seed is None:
            rng = np.random.default_rng()
        else:
            rng = np.random.default_rng((self.seed, idx))

        scene, depth = self._get_clear(rng)
        # 정답은 scene이 아니라 synthesize가 돌려주는 target입니다 (synth.py 주석 참고).
        hazy, target, _ = synthesize(scene, rng, self.cfg, depth=depth)

        hazy, target = self._crop(hazy, target, rng=rng)

        if rng.random() < 0.5:                       # 좌우 반전
            hazy, target = hazy[:, ::-1], target[:, ::-1]

        return bgr_to_tensor(np.ascontiguousarray(hazy)), \
            bgr_to_tensor(np.ascontiguousarray(target))


# ---------------------------------------------------------------- 사전 생성


class PairedDataset(Dataset):
    """미리 만들어둔 hazy/clear 쌍 디렉터리. 검증·평가 전용.

    구조:
        root/hazy/000123.png
        root/clear/000123.png     <- 파일명이 같아야 짝이 맞습니다
    """

    def __init__(self, root: str | os.PathLike, patch: int = 0):
        self.root = Path(root)
        self.hazy_files = list_images(self.root / "hazy")
        if not self.hazy_files:
            raise FileNotFoundError(f"{self.root/'hazy'} 가 비었습니다. make_dataset.py 를 먼저 실행하세요.")
        self.patch = int(patch)

    def __len__(self) -> int:
        return len(self.hazy_files)

    def __getitem__(self, idx: int):
        hp = self.hazy_files[idx]
        cp = self.root / "clear" / hp.name
        hazy = cv2.imread(str(hp), cv2.IMREAD_COLOR)
        clear = cv2.imread(str(cp), cv2.IMREAD_COLOR)
        if hazy is None or clear is None:
            raise RuntimeError(f"쌍을 읽지 못했습니다: {hp.name}")

        if self.patch > 0:
            # 검증셋은 **중앙 크롭**입니다. 랜덤이면 에폭마다 점수가 흔들립니다.
            h, w = hazy.shape[:2]
            p = min(self.patch, h, w)
            y, x = (h - p) // 2, (w - p) // 2
            hazy, clear = hazy[y:y + p, x:x + p], clear[y:y + p, x:x + p]

        return bgr_to_tensor(hazy), bgr_to_tensor(clear)
