#!/usr/bin/env python3
"""
로컬 기능 테스트 — ROS 없이 알고리즘만 검증합니다.

  pip install pytest opencv-python numpy
  python3 -m pytest tests/ -v

여기서 잡으려는 것은 "돌아가는가"가 아니라 **"맞는가"** 입니다.
그래서 대부분의 테스트가 정답(ground truth)을 아는 합성 장면 위에서 돕니다.

로봇에 올리기 전 이 스위트가 전부 통과하면, 이후 실패는 알고리즘이 아니라
ROS 배선(QoS·헤더·토픽 이름) 문제로 범위가 좁혀집니다. 그게 이 파일의 목적입니다.
"""

from __future__ import annotations

import os
import sys
import time

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from image_pipeline.dehaze import (  # noqa: E402
    ClaheEnhancer,
    DarkChannelDehazer,
    apply_gamma,
    dark_channel,
    estimate_atmospheric_light,
    guided_filter,
)
from image_pipeline.autotune import (  # noqa: E402
    AdaptiveParams,
    estimate_brightness,
    estimate_haze_index,
    estimate_noise_sigma,
    gamma_for_target,
    relative_haze,
    suggest_params,
)
from image_pipeline.pipeline import (  # noqa: E402
    AOD_MODES,
    MODES,
    Pipeline,
)

# AOD-Net 없이 돌릴 수 있는 모드들 (aod* 는 별도 디헤이저가 필요)
DCP_MODES = tuple(m for m in MODES if m not in AOD_MODES)
from image_pipeline.intrinsics import (  # noqa: E402
    fit_size,
    principal_point_sanity,
    scale_k,
    scale_p,
)

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))
from make_synthetic import add_haze, add_lowlight, make_scene  # noqa: E402


# ---------------------------------------------------------------- fixtures


@pytest.fixture(scope="module")
def scene():
    """깨끗한 장면 + 깊이 + 불씨 좌표."""
    return make_scene(640, 480, seed=0)


@pytest.fixture(scope="module")
def hazy_set(scene):
    """(깨끗함, 뿌옇게, 정답 투과율, 정답 대기광)."""
    clear, depth, _ = scene
    hazy, t, a = add_haze(clear, depth, beta=1.8, seed=1)
    return clear, hazy, t, a


def psnr(a: np.ndarray, b: np.ndarray) -> float:
    mse = np.mean((a.astype(np.float64) - b.astype(np.float64)) ** 2)
    return float("inf") if mse == 0 else 10 * np.log10(255.0 ** 2 / mse)


# ---------------------------------------------------------------- 기본 연산


class TestDarkChannel:
    def test_erode_matches_bruteforce_min(self):
        """erode(사각커널)이 정말 최소 필터인지 — 이 등가성 위에 성능이 서 있습니다."""
        rng = np.random.default_rng(0)
        img = rng.random((40, 40, 3)).astype(np.float32)
        patch = 5
        got = dark_channel(img, patch)

        r = patch // 2
        min_ch = img.min(axis=2)
        padded = cv2.copyMakeBorder(min_ch, r, r, r, r, cv2.BORDER_REPLICATE)
        expect = np.empty_like(min_ch)
        for y in range(min_ch.shape[0]):
            for x in range(min_ch.shape[1]):
                expect[y, x] = padded[y:y + patch, x:x + patch].min()

        assert np.allclose(got, expect, atol=1e-6)

    def test_patch1_is_channel_min(self):
        img = np.random.default_rng(1).random((10, 10, 3)).astype(np.float32)
        assert np.allclose(dark_channel(img, 1), img.min(axis=2))

    def test_bright_uniform_image_has_high_dark_channel(self):
        """흰 벽(무채색·밝음)은 다크채널이 높음 = 헤이즈로 오인되는 원리적 한계."""
        white = np.full((32, 32, 3), 0.95, np.float32)
        assert dark_channel(white, 9).mean() > 0.9

    def test_dark_channel_never_exceeds_min_channel(self):
        img = np.random.default_rng(2).random((30, 30, 3)).astype(np.float32)
        assert (dark_channel(img, 7) <= img.min(axis=2) + 1e-6).all()


class TestAtmosphericLight:
    def test_recovers_known_value_on_uniform_haze(self):
        """전면 균일 헤이즈에서는 A가 정답과 거의 같아야 합니다."""
        a_true = np.array([0.75, 0.78, 0.80], np.float32)
        img = np.tile(a_true, (64, 64, 1)).astype(np.float32)
        dark = dark_channel(img, 9)
        a = estimate_atmospheric_light(img, dark, a_max=0.95).reshape(3)
        assert np.allclose(a, a_true, atol=0.02)

    def test_a_max_clips(self):
        img = np.full((64, 64, 3), 0.99, np.float32)
        a = estimate_atmospheric_light(img, dark_channel(img, 9), a_max=0.85)
        assert (a <= 0.85 + 1e-6).all()

    def test_not_dragged_by_a_single_bright_fire_pixel(self):
        """★ 우리 도메인의 핵심 테스트.

        원논문식 '최대 밝기 한 픽셀' 방식이면 이 불씨 하나에 A가 통째로
        끌려갑니다. 상위 후보 평균을 쓰는 현재 구현은 견뎌야 합니다.
        """
        img = np.full((128, 128, 3), 0.55, np.float32)
        img[64, 64] = [1.0, 1.0, 1.0]      # 성냥불 한 점
        a = estimate_atmospheric_light(img, dark_channel(img, 9)).reshape(3)
        assert (a < 0.62).all(), f"불씨에 대기광이 끌렸습니다: {a}"

    def test_sky_ratio_restricts_candidates(self):
        """하단의 밝은 불씨를 sky_ratio로 후보에서 제외할 수 있는지."""
        img = np.full((128, 128, 3), 0.4, np.float32)
        img[100:110, 60:70] = 0.99          # 화면 아래쪽 밝은 덩어리
        dark = dark_channel(img, 9)
        a_all = estimate_atmospheric_light(img, dark, sky_ratio=1.0).reshape(3)
        a_top = estimate_atmospheric_light(img, dark, sky_ratio=0.5).reshape(3)
        assert a_top.mean() < a_all.mean()

    def test_never_zero(self):
        """A로 나누는 연산이 있어 0이면 터집니다."""
        black = np.zeros((32, 32, 3), np.float32)
        a = estimate_atmospheric_light(black, dark_channel(black, 5))
        assert (a > 0).all()


class TestGuidedFilter:
    def test_preserves_flat_region(self):
        guide = np.full((64, 64), 0.5, np.float32)
        src = np.full((64, 64), 0.3, np.float32)
        assert np.allclose(guided_filter(guide, src, 4, 1e-3), 0.3, atol=1e-3)

    def test_smooths_noise(self):
        rng = np.random.default_rng(3)
        guide = np.zeros((64, 64), np.float32)
        src = rng.normal(0.5, 0.1, (64, 64)).astype(np.float32)
        assert guided_filter(guide, src, 4, 1e-2).std() < src.std()

    def test_follows_guide_edge(self):
        """가이드에 에지가 있으면 결과도 그 경계를 따라야 합니다(halo 억제의 근거)."""
        guide = np.zeros((64, 64), np.float32)
        guide[:, 32:] = 1.0
        src = guide.copy()
        out = guided_filter(guide, src, 4, 1e-6)
        assert out[:, :28].mean() < 0.15
        assert out[:, 36:].mean() > 0.85


# ---------------------------------------------------------------- 디헤이즈


class TestDehazer:
    def test_improves_psnr_against_groundtruth(self, hazy_set):
        """★ 가장 중요한 테스트: 정답 대비 실제로 복원되는가."""
        clear, hazy, _, _ = hazy_set
        out = DarkChannelDehazer().process(hazy)
        before, after = psnr(hazy, clear), psnr(out, clear)
        assert after > before + 3.0, f"개선 부족: {before:.2f} -> {after:.2f} dB"

    def test_transmission_correlates_with_truth(self, hazy_set):
        """추정 투과율이 정답과 같은 모양인지(절대값이 아니라 상관)."""
        _, hazy, t_true, _ = hazy_set
        d = DarkChannelDehazer()
        d.process(hazy)
        corr = np.corrcoef(d.last_transmission.ravel(), t_true.ravel())[0, 1]
        assert corr > 0.85, f"투과율 상관 {corr:.3f}"

    def test_transmission_lower_where_haze_thicker(self, hazy_set):
        """멀리(위쪽)가 가까이(아래쪽)보다 투과율이 낮아야 합니다."""
        _, hazy, _, _ = hazy_set
        d = DarkChannelDehazer()
        d.process(hazy)
        t = d.last_transmission
        assert t[:120].mean() < t[-120:].mean()

    def test_increases_contrast(self, hazy_set):
        _, hazy, _, _ = hazy_set
        out = DarkChannelDehazer().process(hazy)
        g = lambda im: cv2.cvtColor(im, cv2.COLOR_BGR2GRAY).std()  # noqa: E731
        assert g(out) > g(hazy) * 1.2

    def test_output_contract(self, hazy_set):
        """dtype·shape·범위 계약. 어긋나면 cv_bridge에서 터집니다."""
        _, hazy, _, _ = hazy_set
        out = DarkChannelDehazer().process(hazy)
        assert out.dtype == np.uint8
        assert out.shape == hazy.shape
        assert out.min() >= 0 and out.max() <= 255

    @pytest.mark.parametrize("scale", [1.0, 0.5, 0.25, 0.125])
    def test_downscale_approximation_stays_close(self, hazy_set, scale):
        """★ 실시간성 최적화의 정당성 검증.

        투과율을 축소본에서 추정하는 게 정당하려면, 원본 해상도 추정 결과와
        결과가 크게 다르지 않아야 합니다. 이 테스트가 곧 발표에서
        "정확도를 얼마나 내주고 속도를 샀는가"의 근거입니다.
        """
        _, hazy, _, _ = hazy_set
        full = DarkChannelDehazer(scale=1.0).process(hazy)
        approx = DarkChannelDehazer(scale=scale).process(hazy)
        assert psnr(approx, full) > 22.0, f"scale={scale}에서 열화가 큼"

    def test_downscale_is_actually_faster(self, hazy_set):
        _, hazy, _, _ = hazy_set
        d_full, d_quarter = DarkChannelDehazer(scale=1.0), DarkChannelDehazer(scale=0.25)
        d_full.process(hazy); d_quarter.process(hazy)          # 워밍업

        t = time.perf_counter()
        for _ in range(3):
            d_full.process(hazy)
        full_ms = (time.perf_counter() - t) / 3

        t = time.perf_counter()
        for _ in range(3):
            d_quarter.process(hazy)
        quarter_ms = (time.perf_counter() - t) / 3

        assert quarter_ms < full_ms, f"축소가 더 느림: {quarter_ms:.4f}s vs {full_ms:.4f}s"

    def test_t0_floor_prevents_blowup(self):
        """투과율 하한이 없으면 진한 연기에서 0으로 나눠 값이 폭발합니다."""
        thick = np.full((64, 64, 3), 200, np.uint8)
        out = DarkChannelDehazer(t0=0.1).process(thick)
        assert np.isfinite(out.astype(np.float64)).all()

    def test_clear_image_barely_changed(self, scene):
        """연기 없는 영상에 걸었을 때 과도하게 망가지지 않아야 합니다."""
        clear, _, _ = scene
        out = DarkChannelDehazer().process(clear)
        assert psnr(out, clear) > 12.0

    def test_lowlight_brightens(self, scene):
        clear, _, _ = scene
        dark = add_lowlight(clear, 0.22, seed=2)
        out = DarkChannelDehazer().process_lowlight(dark)
        assert out.mean() > dark.mean() * 1.3

    def test_handles_odd_and_tiny_sizes(self):
        """축소 시 패치가 0이 되거나 크기가 0이 되는 경계."""
        for shape in [(37, 61, 3), (16, 16, 3), (5, 7, 3)]:
            img = np.random.default_rng(4).integers(0, 255, shape, dtype=np.uint8)
            out = DarkChannelDehazer(scale=0.25).process(img)
            assert out.shape == shape

    def test_fire_pixel_survives(self, scene):
        """★ 불씨가 전처리에 지워지면 안 됩니다 — 미션 자체가 실패합니다."""
        clear, depth, fire = scene
        hazy, _, _ = add_haze(clear, depth, beta=1.8, seed=1)
        out = DarkChannelDehazer().process(hazy)
        x, y = fire
        patch = out[y - 5:y + 6, x - 5:x + 6]
        bg = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY).mean()
        assert cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY).mean() > bg + 20


# ------------------------------------------------------- 복원식 최적화 동등성


def restore_previous(bgr, t, a):
    """복원식의 **최적화 전 numpy 구현**. 비교 기준점입니다.

    `DarkChannelDehazer.process` 는 이 식을 cv2 산술로 바꿔 CPU를 절반 이하로
    줄였습니다(RPi5 10.7ms -> 4.9ms). 여기는 건드리지 마세요.

    ★ 두 구현은 **비트 동일하지 않습니다.** 이쪽은 0~1 영역을 경유했다가
      (÷255 -> 계산 -> ×255) 마지막에 절단하고, 신규 구현은 0~255 영역에서
      바로 계산해 반올림합니다. 그래서 수학적으로 정수인 값에서 이쪽은
      15.9999990 을 얻어 15 로 절단합니다. 차이는 항상 1 레벨 이하이고,
      float64 정답 기준으로는 **신규 쪽이 더 정확합니다**
      (평균 절대오차 0.48 vs 0.50 — TestRestoreEquivalence 참고).
    """
    img_f = bgr.astype(np.float32) / 255.0
    out = (img_f - a) / t[..., None] + a
    return np.clip(out * 255.0, 0, 255).astype(np.uint8)


def restore_exact(bgr, t, a):
    """float64 로 계산한 '정답'. 양쪽 구현의 정확도를 재는 자입니다."""
    img_f = bgr.astype(np.float64) / 255.0
    out = (img_f - a.astype(np.float64)) / t[..., None].astype(np.float64) + a.astype(np.float64)
    return np.clip(out * 255.0, 0.0, 255.0)


def _equivalence_images(scene, hazy_set):
    """★ `t == 1.0` 영역이 넓은 영상을 반드시 포함해야 합니다.

    t=1 이면 복원식이 항등이 되어 결과가 정확히 정수에 떨어지고, 바로 거기서
    절단/반올림이 갈립니다. 매끈한 합성 영상은 그 영역이 거의 없어 불일치가
    0.001% 밖에 안 나오는데, **실제 실내 프레임은 7~40%**가 t=1 입니다
    (한 채널이 0인 어두운/채도 높은 영역 -> 다크채널 0 -> t=1).
    합성 영상만으로 검증하면 이 차이를 통째로 놓칩니다 — 실제로 놓쳤습니다.
    """
    clear, _, _ = scene
    _, hazy, _, _ = hazy_set
    rng = np.random.default_rng(7)

    h, w = 240, 320
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    grad = (40 + 180 * (yy / h) + 25 * np.sin(xx / 17.0)).clip(0, 255)
    zero_ch = np.dstack([grad, grad * 0.8, grad * 0.6]).clip(0, 255).astype(np.uint8)
    zero_ch[:, : w // 2, 1:] = 0          # 왼쪽 절반의 G,R 을 0 으로 -> t=1 영역

    return {
        "합성 연기": hazy,
        "원본": clear,
        "채널0 영역(t=1 다수)": zero_ch,
        "랜덤": rng.integers(0, 256, (120, 160, 3), dtype=np.uint8),
        "포화(흰색)": np.full((64, 64, 3), 255, np.uint8),
        "암부(검정)": np.zeros((64, 64, 3), np.uint8),
    }


class TestRestoreEquivalence:
    """★ cv2 복원식이 이전 numpy 식과 **1 레벨 이내로** 같은지 잠급니다.

    비트 동일을 요구하지 않습니다(위 restore_previous 참고). 대신 세 가지를
    봅니다 — 차이가 1을 넘지 않을 것, 체계적 편향이 없을 것, 정확도가
    떨어지지 않을 것. 이게 깨지면 전처리 출력 분포가 바뀐 것이고, 필터본으로
    학습한 YOLO 가중치(`yolo26/0822_filtered_model.pt`)의 입력이 어긋납니다.
    """

    @pytest.mark.parametrize("a_smoothing", [0.0, 0.85])
    def test_never_differs_by_more_than_one_level(self, scene, hazy_set, a_smoothing):
        for name, img in _equivalence_images(scene, hazy_set).items():
            d = DarkChannelDehazer(scale=0.25, a_smoothing=a_smoothing)
            out = d.process(img)
            prev = restore_previous(img, d.last_transmission, d.last_a)

            gap = np.abs(out.astype(np.int16) - prev.astype(np.int16)).max()
            assert gap <= 1, f"{name}: 최대 차이 {gap} (반올림 범위 초과)"

    def test_no_systematic_bias(self, scene, hazy_set):
        """★ CV_8U 변환은 반올림인데 이전 구현은 절단이라, A 에서 0.5를 빼는
        보정이 빠지면 결과가 통째로 +1 밀립니다. 최대차 검사로는 못 잡습니다
        (그래도 1이니까) — 부호 있는 평균이라야 잡힙니다."""
        for name, img in _equivalence_images(scene, hazy_set).items():
            d = DarkChannelDehazer(scale=0.25)
            out = d.process(img)
            prev = restore_previous(img, d.last_transmission, d.last_a)

            bias = (out.astype(np.int16) - prev.astype(np.int16)).mean()
            assert abs(bias) < 0.1, \
                f"{name}: 체계적 편향 {bias:+.4f} — 절단/반올림 보정이 어긋났습니다"

    def test_accuracy_comparable_to_previous(self, scene, hazy_set):
        """★ float64 정답 기준으로 두 구현의 정확도가 비슷한 급이어야 합니다.

        '더 정확할 것'을 요구하지 **않습니다.** t=1.0 인 픽셀에서는 복원값이
        정확히 정수라 `round(N - 0.5)` 가 타이가 되고, OpenCV 는 짝수 반올림이라
        절반이 위로 절반이 아래로 갑니다. 그래서 영상에 따라 신규가 조금 더
        정확하기도(실제 프레임 0.480 vs 0.504) 조금 덜 정확하기도(합성 영상
        0.342 vs 0.316) 합니다. 어느 쪽이든 **0.05 레벨 미만**이면 됩니다.

        타이를 옮겨(0.5 대신 0.499 를 빼서) 정확도를 항상 개선할 수는 있지만,
        그러면 이전 출력에서 **더 멀어집니다**(불일치 4.4% -> 10.8%). 우리가
        지키려는 건 절대 정확도가 아니라 학습 때와 같은 출력 분포입니다.
        """
        for name, img in _equivalence_images(scene, hazy_set).items():
            d = DarkChannelDehazer(scale=0.25)
            out = d.process(img)
            t, a = d.last_transmission, d.last_a
            exact = restore_exact(img, t, a)

            err_new = np.abs(out.astype(np.float64) - exact).mean()
            err_prev = np.abs(restore_previous(img, t, a).astype(np.float64) - exact).mean()
            assert abs(err_new - err_prev) < 0.05, \
                f"{name}: 정확도 차이 {err_new - err_prev:+.4f} (신규 {err_new:.4f} / 이전 {err_prev:.4f})"

    def test_still_uint8_and_saturated(self, hazy_set):
        """cv2 포화 변환이 clip 을 제대로 흡수했는지 (음수 -> 0, 초과 -> 255)."""
        _, hazy, _, _ = hazy_set
        # t0 를 낮추면 1/t 가 커져 복원값이 0~255 밖으로 크게 벗어납니다.
        out = DarkChannelDehazer(scale=0.25, t0=0.02).process(hazy)
        assert out.dtype == np.uint8
        assert out.min() >= 0 and out.max() <= 255


# ---------------------------------------------------------------- CLAHE


class TestClahe:
    def test_increases_local_contrast(self, scene):
        clear, _, _ = scene
        dark = add_lowlight(clear, 0.25, seed=2)
        out = ClaheEnhancer(2.0).process(dark)
        g = lambda im: cv2.cvtColor(im, cv2.COLOR_BGR2GRAY).std()  # noqa: E731
        assert g(out) > g(dark)

    def test_preserves_hue(self, scene):
        """★ LAB의 L채널에만 거는 이유. BGR 각각에 걸면 이 테스트가 깨집니다."""
        clear, _, _ = scene
        out = ClaheEnhancer(2.0).process(clear)
        h_in = cv2.cvtColor(clear, cv2.COLOR_BGR2HSV)[:, :, 0].astype(np.float32)
        h_out = cv2.cvtColor(out, cv2.COLOR_BGR2HSV)[:, :, 0].astype(np.float32)
        mask = cv2.cvtColor(clear, cv2.COLOR_BGR2HSV)[:, :, 1] > 40   # 유채색만
        diff = np.abs(h_in[mask] - h_out[mask])
        diff = np.minimum(diff, 180 - diff)      # 색상환 순환 보정
        assert diff.mean() < 8.0, f"색상 이동 {diff.mean():.2f}도"

    def test_bgr_per_channel_would_shift_hue(self, scene):
        """대조군: 채널별 CLAHE는 실제로 색을 틀어놓는다는 것을 보임."""
        clear, _, _ = scene
        c = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        bad = cv2.merge([c.apply(ch) for ch in cv2.split(clear)])

        def hue_shift(out):
            h_in = cv2.cvtColor(clear, cv2.COLOR_BGR2HSV)[:, :, 0].astype(np.float32)
            h_out = cv2.cvtColor(out, cv2.COLOR_BGR2HSV)[:, :, 0].astype(np.float32)
            mask = cv2.cvtColor(clear, cv2.COLOR_BGR2HSV)[:, :, 1] > 40
            d = np.abs(h_in[mask] - h_out[mask])
            return np.minimum(d, 180 - d).mean()

        assert hue_shift(bad) > hue_shift(ClaheEnhancer(2.0).process(clear))

    def test_higher_cliplimit_stronger(self, scene):
        clear, _, _ = scene
        dark = add_lowlight(clear, 0.25, seed=2)
        g = lambda im: cv2.cvtColor(im, cv2.COLOR_BGR2GRAY).std()  # noqa: E731
        assert g(ClaheEnhancer(4.0).process(dark)) > g(ClaheEnhancer(1.0).process(dark))

    def test_update_takes_effect(self, scene):
        clear, _, _ = scene
        e = ClaheEnhancer(1.0)
        a = e.process(clear)
        e.update(5.0, (8, 8))
        assert not np.array_equal(a, e.process(clear))

    def test_output_contract(self, scene):
        clear, _, _ = scene
        out = ClaheEnhancer().process(clear)
        assert out.dtype == np.uint8 and out.shape == clear.shape


class TestGamma:
    def test_identity(self, scene):
        clear, _, _ = scene
        assert np.array_equal(apply_gamma(clear, 1.0), clear)

    def test_below_one_brightens(self, scene):
        clear, _, _ = scene
        assert apply_gamma(clear, 0.5).mean() > clear.mean()

    def test_above_one_darkens(self, scene):
        clear, _, _ = scene
        assert apply_gamma(clear, 2.0).mean() < clear.mean()

    def test_monotonic(self):
        ramp = np.arange(256, dtype=np.uint8).reshape(1, 256, 1).repeat(3, axis=2)
        out = apply_gamma(ramp, 0.5)[0, :, 0].astype(int)
        assert (np.diff(out) >= 0).all()


# ---------------------------------------------------------------- 파이프라인


class TestCombination:
    def test_full_pipeline_beats_each_alone(self, hazy_set):
        """전처리 조합이 개별보다 나은지 — 3조건 비교 실험의 예행연습."""
        clear, hazy, _, _ = hazy_set
        clahe, dehazer = ClaheEnhancer(2.0), DarkChannelDehazer()

        results = {
            "original": hazy,
            "clahe": clahe.process(hazy),
            "dehaze": dehazer.process(hazy),
            "full": clahe.process(dehazer.process(hazy)),
        }
        contrast = {k: cv2.cvtColor(v, cv2.COLOR_BGR2GRAY).std()
                    for k, v in results.items()}
        assert contrast["full"] > contrast["original"]
        assert contrast["dehaze"] > contrast["original"]

    def test_worst_case_dark_and_hazy(self, scene):
        """연기 + 저조도 동시 — 미션의 최악 조건에서도 죽지 않아야."""
        clear, depth, _ = scene
        hazy, _, _ = add_haze(clear, depth, beta=1.8, seed=1)
        worst = add_lowlight(hazy, 0.22, seed=3)

        out = ClaheEnhancer(2.0).process(
            DarkChannelDehazer().process(apply_gamma(worst, 0.7)))
        assert out.dtype == np.uint8
        assert cv2.cvtColor(out, cv2.COLOR_BGR2GRAY).std() > \
            cv2.cvtColor(worst, cv2.COLOR_BGR2GRAY).std()

    def test_deterministic(self, hazy_set):
        """같은 입력 -> 같은 출력. 3조건 비교가 성립하려면 필수 조건입니다."""
        _, hazy, _, _ = hazy_set
        d = DarkChannelDehazer()
        assert np.array_equal(d.process(hazy), d.process(hazy))

    def test_input_not_mutated(self, hazy_set):
        """입력 배열을 건드리면 여러 조건을 한 프레임에 돌릴 때 오염됩니다."""
        _, hazy, _, _ = hazy_set
        backup = hazy.copy()
        ClaheEnhancer().process(hazy)
        DarkChannelDehazer().process(hazy)
        apply_gamma(hazy, 0.8)
        assert np.array_equal(hazy, backup)


class TestLowlightRegression:
    """★ 로컬 테스트로 잡은 실제 버그의 회귀 방지.

    반전 영상은 원래 전체가 밝아 A가 1.0 근처인데, 연기용 a_max=0.92를
    그대로 적용하면 투과율이 하한까지 눌려 **결과가 오히려 어두워졌습니다.**
    (실측: 평균 밝기 12.6 -> 8.9) 에러는 나지 않고 그림만 이상해지는 유형.
    """

    def test_brightens_even_with_tight_a_max(self, scene):
        clear, _, _ = scene
        dark = add_lowlight(clear, 0.22, seed=2)
        # 연기용으로 바짝 조인 a_max를 줘도 저조도 경로는 영향받지 않아야 함
        out = DarkChannelDehazer(a_max=0.85).process_lowlight(dark)
        assert out.mean() > dark.mean() * 1.3, \
            f"저조도 보정이 밝게 하지 못함: {dark.mean():.1f} -> {out.mean():.1f}"

    def test_increases_contrast(self, scene):
        clear, _, _ = scene
        dark = add_lowlight(clear, 0.22, seed=2)
        out = DarkChannelDehazer().process_lowlight(dark)
        g = lambda im: cv2.cvtColor(im, cv2.COLOR_BGR2GRAY).std()  # noqa: E731
        assert g(out) > g(dark)

    def test_does_not_disturb_haze_path_params(self, scene):
        """저조도 경로가 원래 디헤이저의 설정을 바꿔놓으면 안 됩니다."""
        clear, _, _ = scene
        d = DarkChannelDehazer(a_max=0.92, omega=0.95, t0=0.1)
        d.process_lowlight(add_lowlight(clear, 0.22, seed=2))
        assert (d.a_max, d.omega, d.t0) == (0.92, 0.95, 0.1)


class TestPipeline:
    """노드가 실제로 쓰는 모드 분기 로직 — rclpy 없이 검증."""

    @pytest.fixture
    def img(self, hazy_set):
        return hazy_set[1]

    def test_passthrough_is_identity(self, img):
        """1단계 뼈대의 정의. 여기서 뭐라도 바뀌면 배선 검증이 무의미해집니다."""
        assert np.array_equal(Pipeline(mode="passthrough").process(img), img)

    def test_passthrough_ignores_gamma(self, img):
        out = Pipeline(mode="passthrough", gamma=0.5).process(img)
        assert np.array_equal(out, img)

    @pytest.mark.parametrize("mode", ["passthrough", "clahe", "dehaze", "full"])
    def test_all_modes_produce_valid_output(self, img, mode):
        out = Pipeline(mode=mode).process(img)
        assert out.dtype == np.uint8 and out.shape == img.shape

    def test_modes_differ(self, img):
        # aod* 는 별도 디헤이저가 필요하므로 TestAodMode 에서 따로 봅니다.
        outs = {m: Pipeline(mode=m).process(img) for m in DCP_MODES}
        keys = list(outs)
        for i in range(len(keys)):
            for j in range(i + 1, len(keys)):
                assert not np.array_equal(outs[keys[i]], outs[keys[j]]), \
                    f"{keys[i]}와 {keys[j]}의 결과가 동일 — 모드 분기가 안 먹음"

    def test_full_equals_manual_composition(self, img):
        """full 모드가 정말 '감마 -> 디헤이즈 -> CLAHE' 순서인지."""
        clahe, dehazer = ClaheEnhancer(), DarkChannelDehazer()
        expect = clahe.process(dehazer.process(apply_gamma(img, 0.8)))
        got = Pipeline(mode="full", gamma=0.8,
                       clahe=ClaheEnhancer(), dehazer=DarkChannelDehazer()).process(img)
        assert np.array_equal(got, expect)

    def test_order_matters(self, img):
        """디헤이즈->CLAHE 와 CLAHE->디헤이즈가 다르다는 근거.

        디헤이즈는 물리 모델의 역산이라 입력이 원본 관측값이어야 하고,
        그래서 순서가 설계 선택이라는 점을 뒷받침합니다.
        """
        clahe, dehazer = ClaheEnhancer(), DarkChannelDehazer()
        a = clahe.process(dehazer.process(img))
        b = dehazer.process(clahe.process(img))
        assert not np.array_equal(a, b)

    def test_set_mode_rejects_garbage(self):
        p = Pipeline()
        with pytest.raises(ValueError):
            p.set_mode("dehazee")
        assert p.mode == "full"      # 실패해도 기존 모드 유지

    def test_mode_switch_at_runtime(self, img):
        """ros2 param set 으로 모드를 바꾸는 시나리오."""
        p = Pipeline(mode="passthrough")
        assert np.array_equal(p.process(img), img)
        p.set_mode("full")
        assert not np.array_equal(p.process(img), img)

    def test_timings_recorded(self, img):
        p = Pipeline(mode="full")
        p.process(img)
        assert p.timings["dehaze"] > 0 and p.timings["clahe"] > 0
        assert p.timings["total"] >= p.timings["dehaze"] + p.timings["clahe"] - 1e-6

    def test_timings_zero_for_skipped_stages(self, img):
        p = Pipeline(mode="clahe")
        p.process(img)
        assert p.timings["dehaze"] == 0.0 and p.timings["clahe"] > 0

    def test_does_not_mutate_input(self, img):
        backup = img.copy()
        for m in DCP_MODES:
            Pipeline(mode=m).process(img)
        assert np.array_equal(img, backup)
        for m in AOD_MODES:
            Pipeline(mode=m, aod=_StubAod()).process(img)
        assert np.array_equal(img, backup)


class TestAtmosphericLightSmoothing:
    """프레임 간 A 평활 — 학습 데이터 품질과 직결됩니다.

    A는 매 프레임 장면 내용에서 추정되므로, 로봇이 움직여 밝은 물체가
    화각에 들락거리면 **같은 불씨가 프레임마다 다른 밝기로 복원**됩니다.
    YOLO 학습 데이터로 쓰면 같은 물체의 외형 분산이 불필요하게 커집니다.
    """

    @pytest.fixture
    def sequence(self, scene):
        """밝은 물체가 가로질러 지나가는 시퀀스 — A 추정을 흔드는 조건."""
        clear, depth, _ = scene
        frames = []
        for i in range(8):
            c = clear.copy()
            cv2.rectangle(c, (20 + i * 70, 60), (120 + i * 70, 200), (250, 252, 255), -1)
            frames.append(add_haze(c, depth, beta=1.8, seed=1)[0])
        return frames

    def _a_series(self, frames, smoothing):
        d = DarkChannelDehazer(a_smoothing=smoothing)
        d.reset_state()
        out = []
        for f in frames:
            d.process(f)
            out.append(d.last_a.reshape(3).mean())
        return np.array(out)

    def test_reduces_frame_to_frame_jumps(self, sequence):
        raw = self._a_series(sequence, 0.0)
        smooth = self._a_series(sequence, 0.8)
        jump_raw = np.abs(np.diff(raw)).max()
        jump_smooth = np.abs(np.diff(smooth)).max()
        assert jump_smooth < jump_raw * 0.5, \
            f"평활 효과 없음: {jump_raw:.5f} -> {jump_smooth:.5f}"

    def test_off_by_default(self):
        """기본값이 꺼짐이어야 정지영상 비교 실험이 결정론적으로 유지됩니다."""
        assert DarkChannelDehazer().a_smoothing == 0.0

    def test_smoothing_breaks_determinism(self, sequence):
        """켜면 출력이 이전 프레임에 의존한다는 사실을 명시적으로 못박아 둡니다.

        **같은 프레임을 두 번 넣어 확인하면 안 됩니다.** 1프레임째에 EMA가 a로
        초기화되므로 2프레임째는 k·a+(1-k)·a = a 라 출력이 같게 나오고,
        평활이 켜져 있는데도 "의존성 없음"으로 보입니다(실제로 이 테스트가
        그렇게 잘못 짜여 실패했었음). 앞 프레임이 **달라야** 차이가 드러납니다.
        """
        prev, target = sequence[0], sequence[5]

        fresh = DarkChannelDehazer(a_smoothing=0.8).process(target)

        d = DarkChannelDehazer(a_smoothing=0.8)
        d.process(prev)
        after_prev = d.process(target)

        assert not np.array_equal(fresh, after_prev)

    def test_reset_state_restores_first_frame_behavior(self, hazy_set):
        _, hazy, _, _ = hazy_set
        d = DarkChannelDehazer(a_smoothing=0.8)
        first = d.process(hazy)
        d.process(hazy)
        d.reset_state()
        assert np.array_equal(d.process(hazy), first)

    def test_converges_to_unsmoothed_on_static_scene(self, hazy_set):
        """장면이 안 변하면 EMA는 결국 원래 추정값으로 수렴해야 합니다."""
        _, hazy, _, _ = hazy_set
        plain = DarkChannelDehazer(a_smoothing=0.0)
        plain.process(hazy)

        d = DarkChannelDehazer(a_smoothing=0.8)
        for _ in range(30):
            d.process(hazy)
        assert np.allclose(d.last_a, plain.last_a, atol=1e-3)


class TestAutoTune:
    """파라미터 자동 추정 — 측정 가능한 것과 아닌 것을 구분해 검증합니다."""

    def test_noise_estimator_tracks_injected_noise(self, scene):
        """알려진 σ를 주입해 추정치가 따라오는지."""
        clear, _, _ = scene
        rng = np.random.default_rng(7)
        prev = -1.0
        for sigma in (0.0, 3.0, 8.0, 15.0):
            noisy = np.clip(clear.astype(np.float32)
                            + rng.normal(0, sigma, clear.shape), 0, 255).astype(np.uint8)
            est = estimate_noise_sigma(noisy)
            assert est > prev, f"σ={sigma}에서 단조성 깨짐"
            prev = est
    def test_noise_estimator_absolute_scale(self, scene):
        """절대값 검증 — 단, 휘도 영역 기준이라 채널 σ의 약 0.67배가 나옵니다.

        gray = 0.299R + 0.587G + 0.114B 이므로 채널별 독립 노이즈는
        sqrt(0.299²+0.587²+0.114²) ≈ 0.67 배로 줄어 측정됩니다.
        이 계수를 모르고 채널 σ와 직접 비교하면 "추정이 틀렸다"고 오해합니다.
        """
        clear, _, _ = scene
        rng = np.random.default_rng(9)
        channel_sigma = 15.0
        noisy = np.clip(clear.astype(np.float32)
                        + rng.normal(0, channel_sigma, clear.shape),
                        0, 255).astype(np.uint8)
        expected = channel_sigma * 0.67
        est = estimate_noise_sigma(noisy)
        assert abs(est - expected) < 2.0, f"기대 {expected:.1f}, 추정 {est:.1f}"

    def test_haze_index_is_monotonic(self, scene):
        """★ 절대값은 못 믿어도 단조성은 보장돼야 합니다.

        이 성질이 깨지면 기준선 보정을 해도 소용이 없습니다.
        """
        clear, depth, _ = scene
        prev = estimate_haze_index(clear)[0]
        for beta in (0.3, 0.8, 1.5, 2.5, 3.5):
            hazy = add_haze(clear, depth, beta, seed=1)[0]
            idx = estimate_haze_index(hazy)[0]
            assert idx > prev, f"beta={beta}에서 지표가 감소"
            prev = idx

    def test_haze_index_has_scene_offset(self, scene):
        """연기 없는 장면도 0이 아니라는 사실을 명시적으로 못박아 둡니다.

        이걸 0으로 착각하고 절대값을 쓰면 조용히 틀립니다.
        """
        clear, _, _ = scene
        assert estimate_haze_index(clear)[0] > 0.2

    def test_relative_haze_removes_offset(self, scene):
        clear, depth, _ = scene
        base = estimate_haze_index(clear)[0]
        assert relative_haze(base, base) == 0.0
        hazy = add_haze(clear, depth, 2.5, seed=1)[0]
        assert relative_haze(estimate_haze_index(hazy)[0], base) > 0.5

    def test_gamma_closed_form_hits_target(self):
        """감마는 실험이 아니라 닫힌 해로 구합니다 — 실제로 목표 밝기에 닿는지."""
        rng = np.random.default_rng(3)
        for mean_target in (80.0, 105.0, 140.0):
            img = rng.integers(20, 90, (200, 200, 3), dtype=np.uint8)
            g = gamma_for_target(estimate_brightness(img), mean_target)
            out_mean = estimate_brightness(apply_gamma(img, g))
            assert abs(out_mean - mean_target) < 12.0, \
                f"목표 {mean_target}, 결과 {out_mean:.1f}"

    def test_omega_increases_with_haze(self, scene):
        clear, depth, _ = scene
        base = estimate_haze_index(clear)[0]
        omegas = [suggest_params(add_haze(clear, depth, b, seed=1)[0],
                                 haze_baseline=base)["dehaze_omega"]
                  for b in (0.3, 1.5, 3.0)]
        assert omegas[0] < omegas[-1]
        assert all(0.7 <= o <= 0.95 for o in omegas)

    def test_t0_decreases_with_haze(self, scene):
        """짙은 연기일수록 깊이 복원해야 하므로 하한을 낮춥니다."""
        clear, depth, _ = scene
        base = estimate_haze_index(clear)[0]
        t0s = [suggest_params(add_haze(clear, depth, b, seed=1)[0],
                              haze_baseline=base)["dehaze_t0"]
               for b in (0.3, 1.5, 3.0)]
        assert t0s[0] > t0s[-1]

    def test_t0_raised_by_noise(self, scene):
        """노이즈가 크면 증폭 억제를 위해 t0가 올라가야 합니다."""
        clear, depth, _ = scene
        base = estimate_haze_index(clear)[0]
        hazy = add_haze(clear, depth, 1.5, seed=1)[0]
        rng = np.random.default_rng(11)
        noisy = np.clip(hazy.astype(np.float32)
                        + rng.normal(0, 12.0, hazy.shape), 0, 255).astype(np.uint8)
        assert (suggest_params(noisy, haze_baseline=base)["dehaze_t0"]
                > suggest_params(hazy, haze_baseline=base)["dehaze_t0"])

    def test_patch_and_tile_scale_with_resolution(self, scene):
        """해상도에서 결정되는 값 — 이미지 내용과 무관해야 합니다."""
        clear, _, _ = scene
        small = cv2.resize(clear, (320, 240))
        big = cv2.resize(clear, (1280, 960))
        assert (suggest_params(small)["dehaze_patch"]
                < suggest_params(big)["dehaze_patch"])
        assert all(suggest_params(x)["dehaze_patch"] % 2 == 1 for x in (small, big))

    def test_scale_from_fps_budget(self, scene):
        """scale은 이미지가 아니라 연산 예산에서 나옵니다."""
        clear, _, _ = scene
        tight = suggest_params(clear, fps_budget_ms=10.0)["dehaze_scale"]
        loose = suggest_params(clear, fps_budget_ms=100.0)["dehaze_scale"]
        assert tight < loose
        assert 0.1 <= tight <= 1.0 and 0.1 <= loose <= 1.0

    def test_suggested_params_are_usable(self, hazy_set):
        """산출된 값을 그대로 넣었을 때 파이프라인이 정상 동작하는지."""
        _, hazy, _, _ = hazy_set
        p = suggest_params(hazy)
        out = Pipeline(
            mode="full",
            gamma=p["gamma"],
            clahe=ClaheEnhancer(p["clahe_clip_limit"], tuple(p["clahe_tile_grid"])),
            dehazer=DarkChannelDehazer(omega=p["dehaze_omega"], t0=p["dehaze_t0"],
                                       patch=p["dehaze_patch"]),
        ).process(hazy)
        assert out.dtype == np.uint8 and out.shape == hazy.shape


class TestAdaptiveParams:
    """실시간 적응 — 임무 중 연기 농도가 변하는 상황."""

    @pytest.fixture
    def mission(self, scene):
        """맑음 → 연기 짙어짐 → 다시 옅어짐."""
        clear, depth, _ = scene
        betas = [0.0, 0.0, 0.0, 0.8, 2.0, 3.0, 2.0, 0.8, 0.0]
        frames = [clear if b == 0 else add_haze(clear, depth, b, seed=1)[0]
                  for b in betas]
        return betas, frames

    def test_baseline_self_calibrates(self, mission):
        """맑은 시작 프레임에서 기준선이 자동으로 잡히는지."""
        _, frames = mission
        ap = AdaptiveParams(update_every=1)
        pipe = Pipeline(mode="full")
        ap.update(frames[0], pipe)
        assert ap.haze_baseline is not None
        assert 0.2 < ap.haze_baseline < 0.7

    def test_params_track_haze(self, mission):
        """연기가 짙어지면 omega가 오르고 t0가 내려가야 합니다."""
        betas, frames = mission
        ap = AdaptiveParams(smoothing=0.0, update_every=1)   # 평활 없이 추종만 확인
        pipe = Pipeline(mode="full")
        rec = []
        for f in frames:
            rec.append(ap.update(f, pipe))
        peak = betas.index(max(betas))
        assert rec[peak]["omega"] > rec[0]["omega"]
        assert rec[peak]["t0"] < rec[0]["t0"]

    def test_smoothing_limits_jumps(self, mission):
        _, frames = mission
        def jumps(sm):
            ap = AdaptiveParams(smoothing=sm, update_every=1)
            pipe = Pipeline(mode="full")
            vals = [ap.update(f, pipe)["omega"] for f in frames]
            return np.abs(np.diff(vals)).max()
        assert jumps(0.9) < jumps(0.0)

    def test_update_every_throttles(self, mission):
        _, frames = mission
        ap = AdaptiveParams(update_every=3)
        pipe = Pipeline(mode="full")
        results = [ap.update(f, pipe) for f in frames]
        assert results[0] is not None and results[1] is None and results[2] is None
        assert results[3] is not None

    def test_fixed_baseline_not_overwritten(self, mission):
        _, frames = mission
        ap = AdaptiveParams(haze_baseline=0.40, update_every=1)
        pipe = Pipeline(mode="full")
        for f in frames:
            ap.update(f, pipe)
        assert ap.haze_baseline == 0.40

    def test_reset_clears_learned_baseline(self, mission):
        _, frames = mission
        ap = AdaptiveParams(update_every=1)
        pipe = Pipeline(mode="full")
        ap.update(frames[0], pipe)
        ap.reset()
        assert ap.haze_baseline is None

    def test_actually_modifies_pipeline(self, mission):
        """컨트롤러가 파이프라인 값을 실제로 바꾸는지 (조용히 무시되면 무의미)."""
        _, frames = mission
        ap = AdaptiveParams(smoothing=0.0, update_every=1)
        pipe = Pipeline(mode="full")
        ap.update(frames[0], pipe)
        before = (pipe.dehazer.omega, pipe.dehazer.t0)
        ap.update(frames[5], pipe)          # 가장 짙은 연기
        assert (pipe.dehazer.omega, pipe.dehazer.t0) != before


class TestPerformanceBudget:
    """20fps = 프레임당 50ms 예산.

    개발 PC 기준이라 RPi5에서는 2~4배 느려집니다. 여기서 여유가 없으면
    실기에서는 확실히 못 따라간다는 뜻이라, 조기 경보로 씁니다.
    """

    @pytest.mark.parametrize("width,budget_ms", [(320, 15.0), (640, 60.0)])
    def test_within_budget(self, hazy_set, width, budget_ms):
        _, hazy, _, _ = hazy_set
        h = int(hazy.shape[0] * width / hazy.shape[1])
        img = cv2.resize(hazy, (width, h), interpolation=cv2.INTER_AREA)

        clahe, dehazer = ClaheEnhancer(), DarkChannelDehazer()
        for _ in range(2):
            clahe.process(dehazer.process(img))

        n = 5
        t = time.perf_counter()
        for _ in range(n):
            clahe.process(dehazer.process(img))
        ms = (time.perf_counter() - t) * 1000 / n

        print(f"\n  {width}px: {ms:.2f}ms/frame ({1000 / ms:.1f}fps 이론최대)")
        assert ms < budget_ms, f"{width}px에서 {ms:.1f}ms — 예산 {budget_ms}ms 초과"


# ---------------------------------------------------------------- 내부 파라미터


class TestIntrinsics:
    def test_scale_k(self):
        k = [1000.0, 0, 960.0, 0, 1000.0, 540.0, 0, 0, 1.0]
        out = scale_k(k, 0.5, 0.5)
        assert out[0] == 500.0 and out[2] == 480.0
        assert out[4] == 500.0 and out[5] == 270.0
        assert out[8] == 1.0            # 동차좌표 항은 불변

    def test_scale_k_rejects_bad_length(self):
        with pytest.raises(ValueError):
            scale_k([1, 2, 3], 0.5, 0.5)

    def test_scale_p_includes_translation(self):
        p = [1000.0, 0, 960.0, 20.0, 0, 1000.0, 540.0, 4.0, 0, 0, 1.0, 0]
        out = scale_p(p, 0.5, 0.5)
        assert out[3] == 10.0 and out[7] == 2.0

    def test_fit_size_no_upscale(self):
        assert fit_size(640, 480, 1920) == (640, 480, 1.0, 1.0)
        assert fit_size(640, 480, 0) == (640, 480, 1.0, 1.0)

    def test_fit_size_keeps_aspect(self):
        w, h, sx, sy = fit_size(1920, 1080, 640)
        assert (w, h) == (640, 360)
        assert abs(sx - sy) < 1e-6

    def test_fit_size_scale_matches_integer_result(self):
        """★ 반올림 함정: 배율은 '요청값'이 아니라 실제 정수 크기에서 나와야 함."""
        for src_w, src_h in [(1920, 1080), (1280, 721), (640, 481), (1000, 333)]:
            w, h, sx, sy = fit_size(src_w, src_h, 640)
            assert abs(w - src_w * sx) < 1e-6
            assert abs(h - src_h * sy) < 1e-6

    def test_scaled_k_still_points_at_image_center(self):
        """★ 이 테스트가 태스크②의 '조용히 틀린 거리'를 막습니다."""
        k = [1000.0, 0, 960.0, 0, 1000.0, 540.0, 0, 0, 1.0]
        w, h, sx, sy = fit_size(1920, 1080, 640)
        assert principal_point_sanity(scale_k(k, sx, sy), w, h)

    def test_unscaled_k_fails_sanity(self):
        """대조군: K를 안 고치고 축소 이미지에 쓰면 sanity가 깨져야 함."""
        k = [1000.0, 0, 960.0, 0, 1000.0, 540.0, 0, 0, 1.0]
        assert not principal_point_sanity(k, 640, 360)

    def test_deprojection_consistent_after_scaling(self):
        """★ 최종 확인: 축소 전/후 역투영이 같은 3D 점을 주는가.

        태스크②가 실제로 하는 계산을 그대로 돌려봅니다.
        같은 물체를 가리키는 픽셀이면, 해상도가 달라도 (X,Y,Z)는 같아야 합니다.
        """
        k_full = [1000.0, 0, 960.0, 0, 1000.0, 540.0, 0, 0, 1.0]
        w, h, sx, sy = fit_size(1920, 1080, 640)
        k_small = scale_k(k_full, sx, sy)

        def deproject(k, u, v, d):
            fx, cx, fy, cy = k[0], k[2], k[4], k[5]
            return ((u - cx) * d / fx, (v - cy) * d / fy, d)

        u_full, v_full, depth = 1200.0, 700.0, 3.18
        p_full = deproject(k_full, u_full, v_full, depth)
        p_small = deproject(k_small, u_full * sx, v_full * sy, depth)

        assert np.allclose(p_full, p_small, atol=1e-6)

    def test_wrong_k_gives_wrong_distance(self):
        """반대 확인: 원본 K + 축소 픽셀 = 조용히 틀린 좌표."""
        k_full = [1000.0, 0, 960.0, 0, 1000.0, 540.0, 0, 0, 1.0]
        _, _, sx, sy = fit_size(1920, 1080, 640)

        def x_of(k, u, d):
            return (u - k[2]) * d / k[0]

        correct = x_of(k_full, 1200.0, 3.18)
        wrong = x_of(k_full, 1200.0 * sx, 3.18)     # K를 안 고친 경우
        assert abs(correct - wrong) > 0.5           # 50cm 이상 틀어짐


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))


# ------------------------------------------------- AOD-Net 모드 (HANDOVER 4-9)

class _StubAod:
    """`process(bgr)->bgr` 만 맞춘 가짜 AOD 디헤이저.

    실제 AOD-Net은 onnx 파일이 있어야 도는데, 그 파일 유무가 **모드 분기
    로직의 테스트를 막으면 안 됩니다.** 여기서 검증하는 건 신경망 품질이
    아니라 "aod 모드가 정말 다른 디헤이저를 쓰는가" 하나입니다.
    """

    def __init__(self):
        self.calls = 0

    def process(self, bgr):
        self.calls += 1
        return np.clip(bgr.astype(np.int16) + 7, 0, 255).astype(np.uint8)


class TestAodMode:
    """AOD-Net을 별도 노드가 아니라 mode로 넣은 설계의 잠금 (HANDOVER 4-9)."""

    @pytest.fixture
    def img(self, hazy_set):
        return hazy_set[1]

    @pytest.mark.parametrize("mode", list(AOD_MODES))
    def test_aod_mode_uses_the_aod_dehazer(self, img, mode):
        aod = _StubAod()
        dcp = DarkChannelDehazer()
        out = Pipeline(mode=mode, aod=aod, dehazer=dcp).process(img)
        assert aod.calls == 1, "aod 모드인데 AOD 디헤이저가 안 불렸습니다"
        assert out.dtype == np.uint8 and out.shape == img.shape

    @pytest.mark.parametrize("mode", ["dehaze", "full"])
    def test_dcp_mode_does_not_touch_aod(self, img, mode):
        aod = _StubAod()
        Pipeline(mode=mode, aod=aod).process(img)
        assert aod.calls == 0, "DCP 모드인데 AOD가 불렸습니다"

    def test_aod_full_applies_clahe_but_aod_does_not(self, img):
        aod_only = Pipeline(mode="aod", aod=_StubAod()).process(img)
        aod_full = Pipeline(mode="aod_full", aod=_StubAod()).process(img)
        assert not np.array_equal(aod_only, aod_full)

    def test_missing_aod_raises_instead_of_falling_back_to_dcp(self, img):
        """★ DCP로 조용히 대체하면 'aod 모드로 측정한 결과'가 실제로는 DCP가
        되어 A/B 실험이 통째로 무의미해집니다. 그래서 예외로 막습니다."""
        for mode in AOD_MODES:
            with pytest.raises(ValueError):
                Pipeline(mode=mode).process(img)

    def test_lowlight_with_aod_raises(self, img):
        """★ AOD-Net에는 저조도 대응 경로가 없습니다 (HANDOVER 4-9).

        조용히 건너뛰면 '저조도 보정을 켠 줄 알았는데 안 돈' 상태가 됩니다.
        """
        with pytest.raises(ValueError):
            Pipeline(mode="aod", aod=_StubAod(), lowlight=True).process(img)

    def test_lowlight_with_dcp_still_works(self, img):
        out = Pipeline(mode="dehaze", lowlight=True).process(img)
        assert out.dtype == np.uint8 and out.shape == img.shape
