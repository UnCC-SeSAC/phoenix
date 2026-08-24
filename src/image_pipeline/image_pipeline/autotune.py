#!/usr/bin/env python3
"""
파라미터 자동 추정 — "실험으로 정하는 값"을 줄이기 위한 모듈.

전처리 파라미터는 성격이 세 종류이고, **섞어서 다루면 안 됩니다.**

  (1) 제약으로 결정되는 값 — 이미지 내용과 무관. 측정해서 한 번 정하면 끝.
      scale(연산 예산), patch/tileGrid(해상도), a_smoothing(프레임률)

  (2) 이미지에서 자동 추정 가능한 값 — 매 프레임 계산 가능.
      A(대기광, 이미 자동), gamma(밝기), clipLimit(연기농도+노이즈),
      omega(연기농도), t0(노이즈)

  (3) 목적함수 없이는 못 정하는 값 — "무엇을 좋게 만들 건가"를 먼저 정해야 함.
      최종 미세조정. 우리 목적함수는 "보기 좋게"가 아니라 **YOLO 검출률**입니다.

이 모듈은 (2)를 담당합니다. (1)은 `tune_offline.py --bench`로,
(3)은 `tools/find_params.py`의 그리드 서치로 처리합니다.

★ 왜 자동 추정이 편의 기능이 아니라 정확성 문제인가:
  로봇은 한 번의 임무에서 맑은 구역 → 짙은 연기 → 다시 맑은 구역을 지납니다.
  짙은 연기에 맞춰 고정한 clipLimit은 맑은 구역에서 노이즈를 증폭시키고,
  맑은 구역에 맞춘 값은 연기 속에서 아무것도 못 살립니다.
  **고정값은 어느 한쪽에서 반드시 틀립니다.**
"""

from __future__ import annotations

import cv2
import numpy as np

from image_pipeline.dehaze import dark_channel, estimate_atmospheric_light


# ---------------------------------------------------------------- 장면 측정


def estimate_noise_sigma(bgr: np.ndarray) -> float:
    """센서 노이즈 표준편차 추정 (0~255 스케일). Immerkær(1996) 방법.

    라플라시안 유사 커널로 이미지를 걸러 남는 고주파 에너지를 노이즈로 봅니다.
    커널이 1·2차 미분에 직교하도록 설계돼 있어 **평탄면·경사면·에지에 반응하지 않고**
    노이즈만 남습니다. 그래서 별도 평탄영역 검출 없이 한 번에 계산됩니다.

    저조도에서 이 값이 커지는데, 그때 CLAHE를 세게 걸면 노이즈만 증폭됩니다.

    ★ 반환값은 **휘도(gray) 영역의 σ**이지 채널별 σ가 아닙니다.
      gray = 0.299R + 0.587G + 0.114B 이므로, 채널별로 독립인 노이즈는
      sqrt(0.299² + 0.587² + 0.114²) ≈ 0.67 배로 줄어 측정됩니다.
      (실측: 채널별 σ=15 주입 → 추정 10.6 ≈ 15 × 0.67)
      CLAHE가 LAB의 L채널에 걸리므로 **휘도 영역 σ가 오히려 맞는 기준**이지만,
      채널별 σ와 비교할 때는 이 계수를 기억해야 합니다.
    """
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).astype(np.float64)
    h, w = gray.shape
    if h < 5 or w < 5:
        return 0.0
    m = np.array([[1, -2, 1], [-2, 4, -2], [1, -2, 1]], np.float64)
    conv = cv2.filter2D(gray, -1, m, borderType=cv2.BORDER_REPLICATE)
    inner = conv[2:-2, 2:-2]          # 경계 패딩 영향 제거
    sigma = np.abs(inner).sum() * np.sqrt(np.pi / 2.0) / (6.0 * inner.size)
    return float(sigma)


def estimate_haze_index(bgr: np.ndarray, scale: float = 0.25,
                        patch: int = 15) -> tuple[float, np.ndarray]:
    """연기 농도 **지표** 추정 → (0~1 지표, 추정 대기광 A).

    원리: I = J·t + A(1-t) 에서 다크 채널은 J 성분이 거의 0이라는 게 DCP의 전제이므로
        dark(I) ≈ A·(1 - t)
    따라서 `dark(I) / A` 가 연기 농도를 따라 움직입니다.
    디헤이즈가 어차피 계산하는 다크 채널을 재사용하므로 **비용이 사실상 0**입니다.

    ★ "level"이 아니라 "index"인 이유 — 절대 농도가 아닙니다.
      실측: 연기 없는 지하주차장 장면에서도 0.47이 나옵니다. 콘크리트·회색 벽 같은
      무채색 면은 원래 다크 채널이 높기 때문이고, DCP 자체의 한계라 없앨 수 없습니다.
      즉 **오프셋이 장면마다 다릅니다.**

      대신 단조성은 확실합니다 (실측, 같은 장면):
          연기 없음 0.47 → beta 0.3: 0.57 → 0.8: 0.67 → 1.5: 0.75 → 2.5: 0.81

      그래서 절대값을 쓰지 말고 **기준선 대비 상대 변화**로 쓰세요.
      `relative_haze()`와 `AdaptiveParams`가 기준선을 자동으로 잡습니다.
    """
    img_f = bgr.astype(np.float32) / 255.0
    if scale < 1.0:
        img_f = cv2.resize(img_f, None, fx=scale, fy=scale,
                           interpolation=cv2.INTER_AREA)
    p = max(3, int(round(patch * scale)) | 1)

    dark = dark_channel(img_f, p)
    a = estimate_atmospheric_light(img_f, dark, a_max=0.98)
    idx = float(np.clip(dark.mean() / max(a.mean(), 1e-3), 0.0, 1.0))
    return idx, a.reshape(3)


def relative_haze(index: float, baseline: float | None) -> float:
    """지표를 기준선 대비 0~1 상대 농도로 변환.

    baseline은 "이 장면에서 연기가 없을 때의 지표값"입니다. 이걸 빼면
    장면 고유 오프셋이 사라집니다.

        relative = (index - baseline) / (1 - baseline)

    baseline이 없으면(None) 보수적으로 0.5를 가정한 값을 돌려줍니다.
    """
    if baseline is None:
        return float(np.clip((index - 0.45) / 0.45, 0.0, 1.0))
    b = float(np.clip(baseline, 0.0, 0.95))
    return float(np.clip((index - b) / max(1.0 - b, 1e-3), 0.0, 1.0))


def estimate_brightness(bgr: np.ndarray) -> float:
    """평균 밝기 (0~255). 저조도 판정과 감마 계산에 사용."""
    return float(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).mean())


# ---------------------------------------------------------------- 파라미터 산출


def gamma_for_target(mean_level: float, target: float = 105.0) -> float:
    """평균 밝기를 target으로 옮기는 감마.

    out = in^g (정규화 좌표) 에서 평균이 대략 m^g 로 옮겨간다고 보면
        g = log(target/255) / log(m/255)
    한 줄로 풀립니다. 실험이 필요 없는 **닫힌 해**입니다.
    """
    m = float(np.clip(mean_level, 1.0, 254.0)) / 255.0
    t = float(np.clip(target, 1.0, 254.0)) / 255.0
    g = np.log(t) / np.log(m)
    return float(np.clip(g, 0.35, 2.5))


def suggest_params(
    bgr: np.ndarray,
    fps_budget_ms: float | None = None,
    target_brightness: float = 105.0,
    haze_baseline: float | None = None,
) -> dict:
    """이미지 한 장에서 권장 파라미터를 산출합니다.

    반환 dict의 `_measured` 키에 측정값이 들어 있어, 왜 그 값이 나왔는지
    추적할 수 있습니다. 블랙박스로 두면 이상한 값이 나왔을 때 못 고칩니다.
    """
    h, w = bgr.shape[:2]

    index, a = estimate_haze_index(bgr)
    haze = relative_haze(index, haze_baseline)
    sigma = estimate_noise_sigma(bgr)
    mean = estimate_brightness(bgr)

    # --- (1) 해상도에서 결정되는 값 ---
    # patch는 "이미지 폭의 약 1/42"가 경험칙. 640px에서 15가 나옵니다.
    patch = max(3, int(round(w / 42.0)) | 1)
    # tileGrid는 타일 하나가 대략 80px가 되도록. 연기 얼룩 크기와 맞물립니다.
    tile = int(np.clip(round(w / 80.0), 4, 16))

    # --- (2) 연기 농도에서 결정되는 값 ---
    # omega: 아래 계수는 감으로 정한 게 아니라, 정답을 아는 합성 장면에서
    #        PSNR 그리드 서치로 뽑은 최적값에 맞춘 것입니다.
    #        측정된 최적: 상대농도 0.18 → 0.80 / 0.52 → 0.95 / 0.63 → 0.95
    #        연기가 옅을 때 omega를 낮춰야 하는 이유는 과보정 방지입니다.
    omega = float(np.clip(0.72 + 0.45 * haze, 0.72, 0.95))

    # --- (3) 노이즈와 연기 농도가 함께 결정하는 값 ---
    # t0: 복원식의 증폭률이 1/t 이므로 노이즈도 1/t0 배 증폭됩니다.
    #     두 요구가 반대 방향으로 당깁니다.
    #       - 연기가 짙을수록 t0를 낮춰야 깊은 곳까지 복원됨 (측정 최적: 0.30→0.05)
    #       - 노이즈가 클수록 t0를 높여야 증폭을 억제함
    #     안전한 쪽(높은 t0)을 택합니다.
    t0_by_haze = 0.40 - 0.556 * haze
    t0_by_noise = sigma / 12.0
    t0 = float(np.clip(max(t0_by_haze, t0_by_noise), 0.05, 0.35))

    # clipLimit: ★ 이 값만은 자동 산출을 신뢰하지 마세요.
    #   PSNR로 그리드 서치하면 최적값이 **항상 1.0(=CLAHE를 최소로)** 로 나옵니다.
    #   CLAHE는 원본과의 픽셀 차이를 키우는 연산이라 PSNR이 무조건 싫어하기 때문입니다.
    #   그런데 CLAHE의 목적은 원본 복원이 아니라 **국소 대비 확보(=검출률)** 이므로
    #   PSNR은 애초에 이 값을 판정할 자격이 없는 지표입니다.
    #   아래는 "노이즈를 과증폭하지 않는 상한" 정도의 안전장치일 뿐이고,
    #   진짜 값은 YOLO mAP로 정해야 합니다 (tools/find_params.py 참고).
    clip_by_haze = 1.2 + 2.6 * haze
    clip_by_noise = 12.0 / max(sigma, 0.8)
    clip = float(np.clip(min(clip_by_haze, clip_by_noise), 1.0, 5.0))

    # --- 감마: 닫힌 해 ---
    gamma = gamma_for_target(mean, target_brightness)
    # 이미 충분히 밝으면 건드리지 않습니다(연기 때문에 밝은 경우가 있음).
    if mean > target_brightness:
        gamma = 1.0

    params = {
        "dehaze_patch": patch,
        "clahe_tile_grid": [tile, tile],
        "dehaze_omega": round(omega, 3),
        "dehaze_t0": round(t0, 3),
        "clahe_clip_limit": round(clip, 2),
        "gamma": round(gamma, 3),
        "_measured": {
            "haze_index": round(index, 3),
            "haze_relative": round(haze, 3),
            "noise_sigma": round(sigma, 2),
            "brightness": round(mean, 1),
            "atmospheric_light": [round(float(v), 3) for v in a],
            "resolution": [w, h],
        },
    }

    if fps_budget_ms is not None:
        # scale은 이미지 내용이 아니라 **연산 예산**에서 나옵니다.
        # 640px에서 디헤이즈가 대략 18ms(scale=0.25) 걸린다는 실측을 기준으로,
        # 비용이 scale^2에 비례한다는 성질을 이용해 역산합니다.
        ref_ms, ref_scale = 18.0, 0.25
        px_ratio = (w * h) / (640.0 * 480.0)
        allowed = max(fps_budget_ms * 0.6, 1.0)   # 40%는 CLAHE·변환 몫으로 남김
        s = ref_scale * np.sqrt(allowed / (ref_ms * px_ratio))
        params["dehaze_scale"] = float(np.clip(round(s, 3), 0.1, 1.0))

    return params


# ---------------------------------------------------------------- 실시간 적응


class AdaptiveParams:
    """프레임마다 파라미터를 갱신하는 컨트롤러.

    임무 중 연기 농도가 변하므로 고정값은 어느 한쪽에서 반드시 틀립니다.
    다만 매 프레임 값이 튀면 영상이 깜빡이고 YOLO 학습 분포도 흔들리므로
    **EMA로 평활**합니다.

    비용: 다크 채널은 디헤이저가 어차피 계산하고, 노이즈 추정은 3x3 필터
    한 번이라 640px에서 1ms 미만입니다.

    ★ 재현성 주의: 적응을 켜면 "파라미터 값"이 아니라 **"파라미터를 정하는 규칙"**
      이 실험 조건이 됩니다. 데이터셋 manifest에는 규칙(이 클래스의 설정)을
      기록해야 하고, 조건 비교 실험 중에는 꺼두는 편이 해석이 쉽습니다.
    """

    def __init__(
        self,
        smoothing: float = 0.9,
        update_every: int = 5,
        target_brightness: float = 105.0,
        enable_gamma: bool = False,
        haze_baseline: float | None = None,
        baseline_relax: float = 1e-4,
    ):
        self.smoothing = smoothing
        self.update_every = max(1, update_every)   # 매 프레임 안 해도 충분히 따라감
        self.target_brightness = target_brightness
        self.enable_gamma = enable_gamma           # 감마는 기본 끔(이중 밝기보정 방지)

        # 기준선 자동 보정. 임무 중 관측한 지표의 **최솟값**을 "연기 없음"으로 봅니다.
        # 로봇은 보통 맑은 진입구에서 출발해 연기 구역으로 들어가므로 초반 프레임이
        # 자연스럽게 기준선이 됩니다.
        #   - 이미 연기 속에서 시작하면 기준선이 과대평가되어 연기를 과소추정합니다.
        #     그 경우 haze_baseline을 직접 넘겨 고정하세요.
        #   - baseline_relax는 기준선을 아주 천천히 위로 풀어주는 값입니다. 조명 변화로
        #     한 번 낮게 찍힌 값에 영구히 붙잡히는 것을 막습니다.
        self.haze_baseline = haze_baseline
        self.baseline_fixed = haze_baseline is not None
        self.baseline_relax = baseline_relax

        self._n = 0
        self._state: dict[str, float] | None = None
        self.last_measured: dict | None = None

    def reset(self) -> None:
        self._n = 0
        self._state = None
        if not self.baseline_fixed:
            self.haze_baseline = None

    def _update_baseline(self, index: float) -> None:
        if self.baseline_fixed:
            return
        if self.haze_baseline is None or index < self.haze_baseline:
            self.haze_baseline = index
        else:
            self.haze_baseline = min(self.haze_baseline + self.baseline_relax, 0.95)

    def update(self, bgr: np.ndarray, pipeline) -> dict | None:
        """프레임을 보고 pipeline의 파라미터를 갱신. 갱신했으면 현재 값을 반환."""
        do_update = (self._n % self.update_every) == 0
        self._n += 1
        if not do_update:
            return None

        s = suggest_params(bgr, target_brightness=self.target_brightness,
                           haze_baseline=self.haze_baseline)
        self._update_baseline(s["_measured"]["haze_index"])
        # 기준선이 갱신됐으면 상대 농도를 다시 계산 (첫 프레임에서 특히 중요)
        s = suggest_params(bgr, target_brightness=self.target_brightness,
                           haze_baseline=self.haze_baseline)
        self.last_measured = dict(s["_measured"],
                                  haze_baseline=round(self.haze_baseline or 0.0, 3))

        target = {
            "omega": s["dehaze_omega"],
            "t0": s["dehaze_t0"],
            "clip": s["clahe_clip_limit"],
            "gamma": s["gamma"] if self.enable_gamma else 1.0,
        }

        if self._state is None:
            self._state = dict(target)
        else:
            k = float(np.clip(self.smoothing, 0.0, 0.99))
            for key, v in target.items():
                self._state[key] = k * self._state[key] + (1.0 - k) * v

        pipeline.dehazer.omega = self._state["omega"]
        pipeline.dehazer.t0 = self._state["t0"]
        if self.enable_gamma:
            pipeline.gamma = self._state["gamma"]
        if abs(pipeline.clahe.clip_limit - self._state["clip"]) > 0.05:
            pipeline.clahe.update(self._state["clip"], pipeline.clahe.tile_grid)

        return dict(self._state)
