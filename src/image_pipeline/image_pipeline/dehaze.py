#!/usr/bin/env python3
"""
Dark Channel Prior (DCP) 기반 디헤이즈 — He et al., CVPR 2009.

OpenCV에 완제품이 없어 직접 구현한 부분.
ROS에 의존하지 않으므로 주피터/CLI에서 그대로 import해 튜닝할 수 있습니다.

실시간성 최적화 (발표에서의 기여 포인트):
  - 대기광/투과율 추정을 축소 해상도(기본 1/4)에서 수행 → 픽셀 수 1/16
  - guided filter도 축소 해상도에서 수행 후 투과율만 업샘플
  - 복원식만 원본 해상도에서 계산 (벡터 연산 3줄)
  - min filter는 cv2.erode 로 대체 (사각 커널 erode == min filter, C 구현)

화재 도메인 특유의 함정:
  - 표준 DCP의 대기광 A 추정은 "가장 밝은 픽셀"을 고르는데, 우리 장면에서는
    그게 **불씨/화염**일 수 있습니다. A가 과대 추정되면 화면 전체가 어두워지고
    정작 불씨 주변이 뭉개집니다.
  - 대응: 상위 후보의 **평균**을 사용 + a_max로 상한 클리핑 + (옵션) 화면 상단
    영역만 후보로 삼기(sky_ratio). 아래 estimate_atmospheric_light 참고.
"""

from __future__ import annotations

import cv2
import numpy as np


# ---------------------------------------------------------------- 기본 연산


def dark_channel(img: np.ndarray, patch: int) -> np.ndarray:
    """다크 채널: 채널 최솟값 -> patch x patch 최소 필터.

    img: float32 (H, W, 3), 0~1 범위
    """
    min_ch = np.min(img, axis=2)
    if patch <= 1:
        return min_ch
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (patch, patch))
    # erode(사각 커널) == 최소 필터. 직접 슬라이딩 윈도우 짜면 수십 배 느림.
    return cv2.erode(min_ch, kernel)


def estimate_atmospheric_light(
    img: np.ndarray,
    dark: np.ndarray,
    top_ratio: float = 0.001,
    a_max: float = 0.92,
    sky_ratio: float = 1.0,
) -> np.ndarray:
    """대기광 A 추정 (1, 1, 3).

    top_ratio : 다크 채널 상위 몇 %를 후보로 볼지 (원논문 0.1%)
    a_max     : A 상한. 화염 같은 포화 픽셀이 섞였을 때 과대추정 방지
    sky_ratio : 후보를 이미지 상단 몇 비율로 제한할지 (1.0 = 제한 없음).
                지하주차장처럼 하늘이 없으면 1.0 유지, 연기가 위에 깔리면 0.5~0.7.
    """
    h, w = dark.shape
    limit = h if sky_ratio >= 1.0 else max(1, int(h * sky_ratio))

    dark_roi = dark[:limit]
    img_roi = img[:limit]

    n = max(int(dark_roi.size * top_ratio), 1)
    flat_dark = dark_roi.ravel()
    idx = np.argpartition(flat_dark, -n)[-n:]

    candidates = img_roi.reshape(-1, 3)[idx]  # (n, 3)
    # 원논문은 후보 중 "가장 밝은 한 픽셀"을 쓰지만, 스페큘러/화염 한 점에
    # 통째로 끌려갑니다. 평균이 훨씬 안정적입니다.
    a = candidates.mean(axis=0)
    a = np.clip(a, 1e-3, a_max)
    return a.reshape(1, 1, 3).astype(np.float32)


def guided_filter(guide: np.ndarray, src: np.ndarray, radius: int, eps: float) -> np.ndarray:
    """단일 채널 guided filter (He et al.).

    cv2.ximgproc.guidedFilter 는 opencv-contrib-python 이 있어야 쓸 수 있어서
    box filter 5번으로 직접 구현했습니다. 의존성 없이 동작합니다.
    """
    guide = guide.astype(np.float32)
    src = src.astype(np.float32)
    k = (2 * radius + 1, 2 * radius + 1)

    mean_i = cv2.blur(guide, k)
    mean_p = cv2.blur(src, k)
    corr_i = cv2.blur(guide * guide, k)
    corr_ip = cv2.blur(guide * src, k)

    var_i = corr_i - mean_i * mean_i
    cov_ip = corr_ip - mean_i * mean_p

    a = cov_ip / (var_i + eps)
    b = mean_p - a * mean_i

    mean_a = cv2.blur(a, k)
    mean_b = cv2.blur(b, k)
    return mean_a * guide + mean_b


# ---------------------------------------------------------------- 메인 클래스


class DarkChannelDehazer:
    """DCP 디헤이저.

    process(bgr_uint8) -> bgr_uint8  형태로 씁니다.
    파라미터는 ROS 파라미터에서 그대로 갈아끼울 수 있게 attribute로 노출.
    """

    def __init__(
        self,
        omega: float = 0.95,
        t0: float = 0.1,
        patch: int = 15,
        scale: float = 0.25,
        use_guided: bool = True,
        guided_radius: int = 8,
        guided_eps: float = 1e-3,
        a_top_ratio: float = 0.001,
        a_max: float = 0.92,
        sky_ratio: float = 1.0,
        a_smoothing: float = 0.0,
    ):
        self.omega = omega          # 1.0으로 하면 원근감이 사라져 부자연스러움 (원논문 0.95)
        self.t0 = t0                # 투과율 하한. 낮을수록 진한 연기까지 복원하지만 노이즈 폭발
        self.patch = patch          # 원본 해상도 기준 패치 크기
        self.scale = scale          # 추정 단계 축소 배율 (★ 실시간성의 핵심)
        self.use_guided = use_guided
        self.guided_radius = guided_radius  # 축소 해상도 기준 반경
        self.guided_eps = guided_eps
        self.a_top_ratio = a_top_ratio
        self.a_max = a_max
        self.sky_ratio = sky_ratio

        # 대기광 A의 프레임 간 EMA 계수 (0 = 끔, 0.9 = 강한 평활).
        # A는 매 프레임 장면 내용에서 추정되므로, 로봇이 움직여 화면이 바뀌면
        # **같은 불씨가 프레임마다 다른 밝기로 복원**됩니다. 영상에서는 깜빡임으로
        # 보이고, YOLO 학습 데이터로 쓰면 같은 물체의 외형 분산이 커집니다.
        # 시계열(동영상·rosbag·실주행)에는 켜고, 정지영상 비교 실험에는 끕니다.
        # ★ 켜면 출력이 이전 프레임에 의존하므로 더 이상 결정론적이지 않습니다.
        self.a_smoothing = a_smoothing
        self._a_ema: np.ndarray | None = None

        self.last_transmission: np.ndarray | None = None  # 디버깅/발표 그림용
        self.last_a: np.ndarray | None = None

    def reset_state(self) -> None:
        """프레임 간 누적 상태 초기화. 새 bag/영상을 시작할 때 호출."""
        self._a_ema = None

    # ------------------------------------------------------------------

    def _patch_for_scale(self) -> int:
        """축소본에서 같은 '실제 공간 범위'를 덮도록 패치도 함께 줄입니다.

        이걸 안 줄이면 축소본에서 패치가 상대적으로 너무 커져 투과율이
        과도하게 뭉개집니다(halo).
        """
        p = int(round(self.patch * self.scale))
        p = max(3, p)
        return p if p % 2 == 1 else p + 1

    def estimate_transmission(self, img_f: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """축소본에서 A와 투과율 t를 추정해 (t_full, A) 반환. img_f: float32 0~1 원본 해상도."""
        h, w = img_f.shape[:2]

        if self.scale < 1.0:
            small = cv2.resize(img_f, None, fx=self.scale, fy=self.scale,
                               interpolation=cv2.INTER_AREA)
        else:
            small = img_f

        patch_s = self._patch_for_scale()

        dark = dark_channel(small, patch_s)
        a = estimate_atmospheric_light(small, dark, self.a_top_ratio,
                                       self.a_max, self.sky_ratio)

        # 프레임 간 평활 — t 계산에 들어가기 **전에** 적용해야
        # 투과율까지 함께 안정됩니다.
        if self.a_smoothing > 0.0:
            if self._a_ema is None:
                self._a_ema = a.copy()
            else:
                k = float(np.clip(self.a_smoothing, 0.0, 0.99))
                self._a_ema = k * self._a_ema + (1.0 - k) * a
            a = self._a_ema

        # t = 1 - omega * darkchannel(I / A)
        t_small = 1.0 - self.omega * dark_channel(small / a, patch_s)

        if self.use_guided:
            gray = cv2.cvtColor((small * 255).astype(np.uint8), cv2.COLOR_BGR2GRAY)
            gray = gray.astype(np.float32) / 255.0
            t_small = guided_filter(gray, t_small, self.guided_radius, self.guided_eps)

        if self.scale < 1.0:
            # 투과율은 저주파 신호라 업샘플해도 손실이 거의 없습니다. 이게 성립하는
            # 덕분에 축소 추정이 정당화됩니다.
            t = cv2.resize(t_small, (w, h), interpolation=cv2.INTER_LINEAR)
        else:
            t = t_small

        return t, a

    # ------------------------------------------------------------------

    def process(self, bgr: np.ndarray) -> np.ndarray:
        """bgr uint8 -> 디헤이즈된 bgr uint8."""
        img_f = bgr.astype(np.float32) / 255.0

        t, a = self.estimate_transmission(img_f)
        t = np.clip(t, self.t0, 1.0)

        self.last_transmission = t
        self.last_a = a

        # 복원식 J = (I - A) / t + A
        out = (img_f - a) / t[..., None] + a
        return np.clip(out * 255.0, 0, 255).astype(np.uint8)

    def process_lowlight(
        self,
        bgr: np.ndarray,
        omega: float = 0.8,
        t0: float = 0.25,
    ) -> np.ndarray:
        """저조도 보정 (옵션).

        "저조도 영상을 반전하면 헤이즈 영상과 통계적으로 닮는다"는 관찰
        (Dong et al., 2011)을 이용해 **같은 DCP 코드를 재사용**합니다.
            반전 -> 디헤이즈 -> 반전
        CLAHE가 국소 대비를 올리는 것과 달리 이쪽은 전역 밝기를 끌어올립니다.

        ★ 반전 영역은 파라미터가 달라야 합니다 (로컬 테스트로 발견한 함정).
          - `a_max`: 연기 영역에서는 화염에 끌리는 걸 막으려 0.92로 조였지만,
            반전 영상은 원래 전체가 밝아 A가 정말 1.0 근처입니다. 여기에
            0.92를 그대로 쓰면 t가 하한까지 눌려 결과가 **오히려 어두워집니다.**
          - `omega`/`t0`: 반전 영상은 '헤이즈'가 화면 전체에 깔린 상태라
            강도를 낮추고(0.8) 하한을 올려야(0.25) 노이즈가 안 터집니다.

        조명이 완전히 꺼진 구간에서만 켜세요. 노이즈가 같이 증폭됩니다.
        """
        sub = DarkChannelDehazer(
            omega=omega,
            t0=t0,
            patch=self.patch,
            scale=self.scale,
            use_guided=self.use_guided,
            guided_radius=self.guided_radius,
            guided_eps=self.guided_eps,
            a_top_ratio=self.a_top_ratio,
            a_max=1.0,          # ★ 반전 영역에서는 조이지 않음
            sky_ratio=1.0,      # 반전되면 위/아래 의미가 뒤집혀 제한이 무의미
        )
        out = 255 - sub.process(255 - bgr)
        self.last_transmission = sub.last_transmission
        self.last_a = sub.last_a
        return out


# ---------------------------------------------------------------- CLAHE


class ClaheEnhancer:
    """LAB의 L 채널에만 CLAHE.

    BGR 각 채널에 따로 걸면 채널별 히스토그램이 제각각 늘어나 **색이 틀어집니다**.
    LAB는 밝기(L)와 색(a,b)이 분리돼 있어 L만 건드리면 색상은 보존됩니다.
    """

    def __init__(self, clip_limit: float = 2.0, tile_grid: tuple[int, int] = (8, 8)):
        self.clip_limit = clip_limit
        self.tile_grid = tuple(tile_grid)
        self._clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=self.tile_grid)

    def update(self, clip_limit: float, tile_grid) -> None:
        self.clip_limit = clip_limit
        self.tile_grid = tuple(tile_grid)
        self._clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=self.tile_grid)

    def process(self, bgr: np.ndarray) -> np.ndarray:
        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
        lab[:, :, 0] = self._clahe.apply(lab[:, :, 0])
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)


def apply_gamma(bgr: np.ndarray, gamma: float) -> np.ndarray:
    """감마 보정: out = (in/255)^gamma * 255.

    **gamma < 1 이면 밝아지고, gamma > 1 이면 어두워집니다.**

    감마는 문헌마다 지수를 뒤집어 쓰는 경우가 있어(어떤 코드는 1/gamma를 씀)
    방향이 반대가 되기 쉽습니다. 여기서는 위 식으로 고정했고
    tests/test_dehaze.py::TestGamma 가 방향을 잠가둡니다.

    LUT 256개 조회라 해상도와 무관하게 비용이 거의 0입니다.
    """
    if abs(gamma - 1.0) < 1e-3:
        return bgr
    g = max(gamma, 1e-3)
    lut = np.clip((np.arange(256) / 255.0) ** g * 255.0, 0, 255).astype(np.uint8)
    return cv2.LUT(bgr, lut)
