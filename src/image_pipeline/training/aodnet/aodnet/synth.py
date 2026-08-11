#!/usr/bin/env python3
"""
화재 연기 합성 — 학습 쌍 (연기 낀 I, 정답 J) 을 만드는 곳.

★ 이 파일이 이 튜토리얼에서 제일 중요합니다.
   AOD-Net 구현은 40줄이면 끝나고, 성능은 거의 전부 여기서 갈립니다.

왜 공개 데이터셋(RESIDE)을 안 쓰는가
-------------------------------------
AOD-Net 논문과 대부분의 구현은 RESIDE/NYU-Depth 로 학습합니다. 그건
**야외 안개**입니다. 우리 도메인(지하주차장 화재)과 세 가지가 다릅니다.

  1) 대기광 A 가 다릅니다.
     안개: A ≈ (0.8~1.0) 흰색, 하늘빛.
     화재 연기: 회색(0.4~0.7)이거나 그을음이면 **어둡고**(0.15~0.4),
     화염 근처는 주황빛으로 물듭니다. A를 흰색으로만 학습시킨 망은
     검은 연기에서 화면을 오히려 더 어둡게 만듭니다.

  2) 농도 분포가 다릅니다.
     안개: 거의 t = exp(-β·d), 깊이에만 의존하는 매끄러운 함수.
     연기: 발화점에서 솟아오르는 **덩어리(plume)**. 깊이와 무관하게
     가까운 곳이 더 진할 수 있고, 난류 때문에 경계가 너덜너덜합니다.

  3) 장면 안에 **자기 발광체(화염)**가 있습니다.
     산란 모델 I = J·t + A(1-t) 는 장면이 수동 반사체라고 가정합니다.
     화염 픽셀은 이 가정 밖이라 그대로 두는 게 맞습니다. 여기서는 화염
     마스크를 받아 그 영역의 t를 1로 되돌립니다.

  => 그래서 도메인에 맞는 데이터를 직접 만듭니다. 정답 J를 우리가 쥐고
     있으므로 PSNR/SSIM을 실제로 잴 수 있다는 부수 효과도 있습니다.

물리 모델
---------
    I(x) = J(x)·t(x) + A·(1 - t(x)),   t(x) = exp(-β · D(x))

    D(x) = w_depth · depth(x) + w_plume · plume(x)

    depth : 카메라로부터의 거리 (있으면 실제 depth, 없으면 유사 깊이)
    plume : 연기 덩어리 농도장 (0~1)
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

# ---------------------------------------------------------------- 노이즈/장


def fractal_noise(h: int, w: int, rng: np.random.Generator,
                  octaves: int = 4, persistence: float = 0.55) -> np.ndarray:
    """0~1 범위 프랙탈(멀티옥타브) 노이즈.

    Perlin 라이브러리를 쓰지 않은 이유: 의존성을 늘리지 않기 위해서입니다.
    저해상도 가우시안 노이즈를 bicubic 업샘플해 더하면 시각적으로 충분히
    비슷한 1/f 스펙트럼이 나옵니다. 연기 난류는 원래 정확한 재현이 목표가
    아니라 "경계가 매끄럽지 않다"는 성질만 학습시키면 됩니다.
    """
    total = np.zeros((h, w), np.float32)
    amplitude = 1.0
    norm = 0.0
    for o in range(octaves):
        res = max(2, int(4 * (2 ** o)))
        small = rng.normal(0.0, 1.0, (res, res)).astype(np.float32)
        layer = cv2.resize(small, (w, h), interpolation=cv2.INTER_CUBIC)
        total += amplitude * layer
        norm += amplitude
        amplitude *= persistence

    total /= max(norm, 1e-6)
    total -= total.min()
    total /= max(total.max(), 1e-6)
    return total


def pseudo_depth(h: int, w: int, rng: np.random.Generator) -> np.ndarray:
    """실제 깊이가 없을 때 쓰는 유사 깊이맵 (0~1, 클수록 멀다).

    ★ 실제 depth 프레임이 있으면 반드시 그걸 쓰세요.
      우리 로봇은 RealSense를 달고 있으므로 정합된 depth를 저장해두면
      물리적으로 정확한 학습 쌍을 만들 수 있고, 그게 이 함수보다 훨씬 낫습니다.
      (make_dataset.py --depth-dir)

    없을 때는 "화면 위쪽/중앙이 멀다"는 실내 복도의 통계적 경향만 흉내냅니다.
    소실점 위치를 랜덤화해 망이 '위쪽 = 항상 멀다'로 외우는 걸 막습니다.
    """
    vy = rng.uniform(0.25, 0.55)   # 소실점 y (화면 비율)
    vx = rng.uniform(0.30, 0.70)   # 소실점 x

    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    yy /= h
    xx /= w

    dist = np.sqrt(((xx - vx) * 0.6) ** 2 + (yy - vy) ** 2)
    depth = 1.0 - dist / (dist.max() + 1e-6)
    depth = np.clip(depth, 0.0, 1.0) ** rng.uniform(0.8, 2.0)

    # 저주파 흔들림을 섞어 완벽한 방사형 대칭을 깹니다.
    depth = 0.85 * depth + 0.15 * fractal_noise(h, w, rng, octaves=2)
    return np.clip(depth, 0.0, 1.0).astype(np.float32)


def plume_field(h: int, w: int, rng: np.random.Generator,
                n_sources: int = 2) -> np.ndarray:
    """연기 기둥(plume) 농도장 (0~1).

    발화점에서 위로 퍼지며 올라가는 원뿔을 몇 개 겹치고, 프랙탈 노이즈로
    가장자리를 뜯습니다. 깊이와 **독립**인 성분이라는 게 핵심입니다.
    안개 합성만 학습한 망은 "화면 위쪽만 뿌옇다"를 외우는데, 실제 화재
    영상에서는 카메라 바로 앞이 제일 진할 수 있습니다.
    """
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    field = np.zeros((h, w), np.float32)

    for _ in range(max(1, n_sources)):
        sx = rng.uniform(0.1, 0.9) * w
        sy = rng.uniform(0.55, 1.05) * h       # 발화점은 화면 아래쪽
        spread = rng.uniform(0.15, 0.5) * w    # 상승하며 퍼지는 정도
        rise = rng.uniform(0.4, 1.2) * h       # 얼마나 높이 올라가는지
        drift = rng.uniform(-0.4, 0.4)         # 바람에 의한 기울기

        up = np.clip((sy - yy) / max(rise, 1e-6), 0.0, None)   # 위로 갈수록 1
        axis_x = sx + drift * (sy - yy)
        half_width = 1e-3 + spread * (0.15 + up)               # 원뿔
        lateral = np.abs(xx - axis_x) / half_width

        blob = np.exp(-(lateral ** 2)) * np.exp(-1.5 * np.clip(up - 0.15, 0, None))
        field = np.maximum(field, blob.astype(np.float32))

    # 난류: 곱으로 섞어야 농도 0인 곳은 0으로 남습니다. 더하기로 하면
    # 연기 없는 영역에도 옅은 안개가 깔려버려 '부분 연기' 학습이 안 됩니다.
    turbulence = fractal_noise(h, w, rng, octaves=4)
    field = field * (0.45 + 0.75 * turbulence)

    # 방 전체에 낮게 깔린 연기층(가끔). 실제 지하공간에서 흔한 패턴.
    if rng.random() < 0.4:
        layer = np.clip((yy / h - rng.uniform(0.3, 0.7)) * rng.uniform(1.5, 4.0), 0, 1)
        field = np.maximum(field, layer.astype(np.float32) * rng.uniform(0.3, 0.8))

    return np.clip(cv2.GaussianBlur(field, (0, 0), max(1.0, w / 160.0)), 0.0, 1.0)


# ---------------------------------------------------------------- 대기광


#: 화재 도메인의 대기광 A 종류. (BGR 순서, 0~1)
AIRLIGHT_STYLES = ("white", "gray", "sooty", "firelit")

#: 스타일별 샘플링 비율 기본값. AIRLIGHT_STYLES 순서와 짝을 맞춥니다.
#
#  ★ 이 비율이 곧 "우리 망이 어떤 연기에 강해지는가"입니다.
#    앞의 셋(white/gray/firelit)은 대기광이 장면보다 **밝아서** 연기가 화면을
#    밝힙니다 -> 복원은 어둡게, 즉 K > 1.
#    sooty는 대기광이 **어두워서** 연기가 화면을 어둡게 만듭니다
#    -> 복원은 밝게, 즉 K < 1.
#    기본값에서는 K<1 예제가 25%뿐이라 망이 "연기가 짙다 = K를 키운다"는
#    단조 관계만 배우고 sooty에서 K를 1 아래로 못 내립니다.
#    tools/compare.py 의 스타일별 표에서 sooty만 무처리보다 나쁘게 나오면
#    이 비율을 의심하세요.
DEFAULT_STYLE_PROBS = (0.20, 0.40, 0.25, 0.15)


def sample_airlight(rng: np.random.Generator,
                    style: str | None = None,
                    probs=None) -> tuple[np.ndarray, str]:
    """A 를 샘플링. (A(3,), style) 반환. **BGR 순서**입니다."""
    if style is None:
        p = np.asarray(probs if probs is not None else DEFAULT_STYLE_PROBS, dtype=float)
        style = rng.choice(AIRLIGHT_STYLES, p=p / p.sum())

    if style == "white":                       # 수증기/일반 안개
        base = rng.uniform(0.80, 0.97)
        a = np.array([base, base, base], np.float32)
    elif style == "gray":                      # 일반적인 흰-회색 연기
        base = rng.uniform(0.45, 0.78)
        a = np.array([base, base, base], np.float32)
    elif style == "sooty":                     # 그을음 섞인 검은 연기
        base = rng.uniform(0.12, 0.38)
        a = np.array([base, base, base], np.float32)
    else:                                      # firelit — 화염빛을 받은 연기
        base = rng.uniform(0.35, 0.70)
        a = np.array([base * rng.uniform(0.55, 0.75),   # B 낮게
                      base * rng.uniform(0.80, 0.95),   # G
                      base * rng.uniform(1.05, 1.30)],  # R 높게
                     np.float32)

    # 완전 무채색은 현실에 없습니다. 채널별 미세 편차를 항상 더합니다.
    a = a * (1.0 + rng.normal(0.0, 0.02, 3).astype(np.float32))
    return np.clip(a, 0.02, 1.0).astype(np.float32), str(style)


# ---------------------------------------------------------------- 합성


@dataclass
class SmokeConfig:
    """합성 파라미터. 학습 시에는 매 샘플 랜덤, 평가 시에는 고정해서 씁니다."""

    beta_range: tuple[float, float] = (0.6, 3.2)   # 연기 농도(광학 두께)
    depth_weight: tuple[float, float] = (0.2, 0.9)  # D에서 깊이 성분 비중
    plume_weight: tuple[float, float] = (0.4, 1.6)  # D에서 연기덩어리 비중
    t_floor: float = 0.02                           # 투과율 하한 (완전 불투명 방지)
    airlight_style: str | None = None               # None이면 style_probs로 랜덤
    #: 스타일 샘플링 비율. None이면 DEFAULT_STYLE_PROBS.
    #  (white, gray, sooty, firelit) 순서. 합이 1이 아니어도 자동 정규화합니다.
    style_probs: tuple[float, float, float, float] | None = None
    n_sources: tuple[int, int] = (1, 3)
    noise_sigma: float = 2.0                        # 합성 후 센서 노이즈 (0~255 기준)
    jpeg_quality: tuple[int, int] | None = (70, 95)  # 압축 아티팩트 (None이면 끔)
    lowlight: tuple[float, float] | None = (0.45, 1.0)  # 밝기 배율 (저조도 동반)
    preserve_fire: bool = True                      # 화염 픽셀의 t를 1로 되돌릴지

    #: 깨끗한 프레임에 화염을 합성해 넣을 확률. 0이면 끔.
    #
    #  ★ 학습용 프레임에 불이 없으면 preserve_fire가 **한 번도 작동하지 않습니다.**
    #    실제로 frame_cut(주행 영상, 불 없음)만으로 학습한 모델은 화염을 본 적이
    #    없어서 추론 시 불씨를 어둡게 만들었습니다. 화재 탐사 로봇에서 최악입니다.
    #    깨끗한 프레임에 불이 안 들어 있다면 이 값을 켜세요.
    fire_prob: float = 0.0
    fire_count: tuple[int, int] = (1, 2)            # 한 장에 넣을 화염 개수
    fire_scale: tuple[float, float] = (0.04, 0.12)  # 화면 높이 대비 화염 높이

    # ★ 저조도 배율을 정답 J에도 적용할지 — 이 튜토리얼에서 제일 중요한 설계 스위치.
    #
    #   True (기본): 어두워진 J를 그대로 정답으로 씁니다.
    #     망이 배우는 건 순수 디헤이즈뿐입니다. 어두운 장면·밝은 장면 양쪽에서
    #     동작하도록 하는 광도 증강 역할만 합니다.
    #     -> phase1 파이프라인(감마 -> 디헤이즈 -> CLAHE)에 그대로 꽂힙니다.
    #        밝기 복원은 원래 CLAHE 담당이고, DCP도 밝기는 안 건드립니다.
    #        DCP 자리에 드롭인하려면 **같은 계약**이어야 비교와 모드 분리가 성립합니다.
    #
    #   False: I만 어둡게 하고 J는 밝게 둡니다. 망이 디헤이즈 + 밝기 복원을
    #     함께 배웁니다. 그럴싸해 보이지만 **문제가 불량조건이 됩니다** —
    #     "어두운 사진"이 원래 어두운 장면인지 조명이 약한 건지 한 장으로는
    #     구분할 수 없어서, 망은 평균 밝기를 외우고 PSNR 상한이 2~3dB 깎입니다.
    #     게다가 뒤에 CLAHE가 또 밝히므로 이중 보정이 됩니다.
    #     단독 노드로 쓸 때만 켜세요.
    lowlight_on_target: bool = True


@dataclass
class SmokeMeta:
    """합성에 쓰인 정답 값들. 디버깅·시각화·정량 분석용."""

    transmission: np.ndarray
    airlight: np.ndarray
    beta: float
    style: str
    extra: dict = field(default_factory=dict)


#: 화염 후보 덩어리의 모양 제약. fire_mask 주석 참고.
FIRE_MAX_ASPECT = 6.0        # 종횡비 상한 (긴 띠 = 차선·걸레받이 거부)
FIRE_MAX_AREA_RATIO = 0.10   # 화면 대비 면적 상한


def composite_fire(bgr: np.ndarray,
                   rng: np.random.Generator,
                   n_flames: int = 1,
                   scale: tuple[float, float] = (0.04, 0.12)):
    """깨끗한 장면에 **화염을 합성**합니다. (이미지, 박스 목록, 정확한 마스크) 반환.

    ★ 마스크를 함께 돌려주는 이유: 우리가 그린 불이므로 **어디가 불인지 이미
      알고 있습니다.** fire_mask()로 다시 찾을 이유가 없고, 찾으면 오히려
      틀립니다 — 화염의 글로우가 주변 노란 차선을 밝혀서 그게 화염으로
      오검출됩니다(실측 5.01%). 정확한 마스크를 그대로 쓰면 이 문제가 없습니다.
      fire_mask()는 **불이 어디 있는지 모르는 실사**에서만 쓰는 폴백입니다.

    ★ 왜 필요한가 — 실제로 밟은 함정입니다.
      연기 합성은 화염 영역의 투과율을 1로 되돌려 "불씨는 연기 너머로도 보인다"를
      가르치도록 돼 있습니다(synthesize의 preserve_fire). 그런데 **학습용 깨끗한
      프레임에 불이 하나도 없으면 그 장치가 한 번도 작동하지 않습니다.**
      실제로 frame_cut(주행 영상, 불 없음)으로 학습한 모델은 화염을 본 적이 없어
      추론 시 불씨를 어둡게 만들었습니다.

    화염을 어떻게 그리는가 — fire_mask가 인정하는 구조여야 합니다.
      몸통 : 고채도 주황 (H≈16, S≈214, V≈250)
      코어 : 백열, 흰색에 가까움 (V=255, S<100)
      둘 다 있고 서로 붙어 있어야 fire_mask가 화염으로 인정합니다.

    그리고 **글로우(주변 조명)를 반드시 넣습니다.** 화염은 광원이라 주변을
    밝힙니다. 이걸 빼면 "주변은 캄캄한데 불만 떠 있는" 물리적으로 불가능한
    장면이 되고, 망이 그 부자연스러움을 단서로 삼습니다.

    바운딩박스는 (cls=0, cx, cy, w, h) 정규화 좌표입니다 — YOLO 라벨로 바로 씁니다.
    """
    img = bgr.astype(np.float32).copy()
    h, w = img.shape[:2]
    boxes = []
    fire_area = np.zeros((h, w), np.float32)     # 정확한 화염 마스크(누적)

    for _ in range(max(1, n_flames)):
        fh = rng.uniform(*scale) * h                  # 화염 높이(픽셀)
        fw = fh * rng.uniform(0.35, 0.6)              # 폭은 높이보다 좁게
        cx = rng.uniform(0.12, 0.88) * w
        base_y = rng.uniform(0.45, 0.85) * h          # 불의 밑동

        # --- 화염 마스크: 밑동에서 위로 갈수록 좁아지는 물방울 모양
        mask = np.zeros((h, w), np.float32)
        steps = 14
        for i in range(steps):
            u = i / (steps - 1)                       # 0=밑동, 1=끝
            y = base_y - u * fh
            # 위로 갈수록 좁아지고, 흔들림(플리커)을 살짝 준다
            half = fw * (1.0 - u ** 1.5) * 0.5 * rng.uniform(0.85, 1.15)
            drift = rng.normal(0.0, fw * 0.06)
            ax = max(1, int(half))
            ay = max(1, int(fh / steps * 1.4))
            cv2.ellipse(mask, (int(cx + drift * u * 3), int(y)), (ax, ay),
                        0, 0, 360, 1.0, -1)
        mask = cv2.GaussianBlur(mask, (0, 0), max(1.0, fw * 0.12))
        if mask.max() <= 1e-6:
            continue
        mask /= mask.max()

        # --- 3층 색: 바깥(어두운 주황) -> 몸통(고채도) -> 코어(백열)
        #     m 값에 따라 연속 보간합니다. 계단으로 칠하면 경계가 인공적입니다.
        outer = np.array([10, 60, 150], np.float32)     # BGR
        body = np.array([40, 150, 250], np.float32)
        core = np.array([200, 240, 255], np.float32)

        m = mask[..., None]
        t_body = np.clip((mask - 0.20) / 0.35, 0, 1)[..., None]
        t_core = np.clip((mask - 0.62) / 0.30, 0, 1)[..., None]
        color = outer + (body - outer) * t_body
        color = color + (core - color) * t_core

        alpha = np.clip(mask / 0.35, 0, 1)[..., None]   # 가장자리는 반투명
        img = img * (1.0 - alpha) + color * alpha

        # --- 글로우: 화염이 주변을 비춥니다. 더하기(가산) 합성.
        glow_sigma = fh * rng.uniform(0.9, 1.6)
        glow = cv2.GaussianBlur(mask, (0, 0), glow_sigma)
        glow = glow / max(glow.max(), 1e-6)
        warm = np.array([0.35, 0.65, 1.0], np.float32)  # BGR — 주황빛
        img += glow[..., None] * warm * rng.uniform(45, 95)

        # --- 바운딩박스: 마스크가 유효한 영역
        # 투과율을 되돌릴 영역 = 화염 본체. 글로우는 제외합니다(주변을 비추는
        # 빛이지 발광체 자체가 아니므로 연기가 정상적으로 껴야 합니다).
        fire_area = np.maximum(fire_area, np.clip(mask / 0.35, 0.0, 1.0))

        ys, xs = np.where(mask > 0.15)
        if len(ys) == 0:
            continue
        x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
        boxes.append((0,
                      float((x0 + x1) / 2 / w), float((y0 + y1) / 2 / h),
                      float((x1 - x0) / w), float((y1 - y0) / h)))

    return np.clip(img, 0, 255).astype(np.uint8), boxes, fire_area


def fire_mask(bgr: np.ndarray) -> np.ndarray:
    """화염(자기 발광) 픽셀 마스크 0~1.

    화염은 산란 모델의 가정(수동 반사체) 밖이라 투과율을 1로 되돌려야 합니다.
    문제는 **과검출**입니다. 여기서 잘못 잡으면 연기가 안 지워지는 구멍이 뚫리고,
    그 구멍을 정답으로 학습한 망은 실제 영상에서도 그 색을 안 지웁니다.

    ★ 실제로 밟은 함정: "따뜻한 색 + 밝다"만으로 잡으면 **노란 차선**이 걸립니다.
      (frame_cut 데이터 기준 H=21, S=108, V=222 — 옛 임계값 H≤25·V>200·S>60을
       그대로 통과합니다. 화면 가로로 t=1인 띠가 생깁니다.)

      진짜 화염과의 차이는 **채도**입니다.
        화염 몸통 : H≈16, S≈214, V≈250   ← 채도가 압도적으로 높음
        화염 코어 : H≈22, S≈ 55, V=255   ← 흰색에 가까움 (백열)
        노란 차선 : H=21,  S=108, V=222   ← 어중간

      임계값만 조여서는 안 됩니다. 차선의 진한 부분은 S=155까지 올라가서
      "S>150" 같은 선을 그으면 또 통과합니다. 색만으로는 분리가 안 됩니다.

    ★ 실제로 통한 판별: **백열 코어의 유무**.
      불꽃은 가장 뜨거운 안쪽이 흰색에 가깝게 타올라 채도 낮은 밝은 코어를
      갖습니다. 노란 페인트에는 그런 코어가 없습니다 — 균일한 노랑입니다.

      그래서 연결 요소 단위로 봅니다.
        1) 몸통 = 따뜻 + 고채도 + 밝음
        2) 코어 = 거의 흰색 + 매우 밝음
        3) **몸통과 코어를 둘 다 포함한 덩어리만** 화염으로 인정

      이러면 코어 없는 노란 차선(몸통만)과, 몸통 없는 과노출 흰 벽·형광등
      (코어만)이 동시에 걸러집니다.

    ★ 그런데 이것만으로도 부족했습니다.
      노란 차선이 **흰 바닥에 맞닿아** 있으면, 흰 바닥이 코어 역할을 해서
      "몸통 + 코어" 조건을 통과합니다. 색과 인접성만으로는 못 막습니다.

      그래서 모양 제약을 하나 더 겁니다. 불꽃은 **덩어리**이고, 차선+바닥은
      폭 322 × 높이 16짜리 **띠**입니다.
        - 종횡비 6:1 이하           (긴 띠 거부. 세로로 솟은 불기둥도 통과)
        - 화면의 max_area_ratio 이하 (화면 절반을 덮는 '불'은 없습니다)

      대가: 백열 코어가 안 보이는 작고 어두운 불씨, 그리고 화면을 가득 채운
      대형 화염은 놓칩니다. 그래도 이쪽이 낫습니다 — 놓치면 그 불씨에 연기가
      조금 더 낄 뿐이지만, 잘못 잡으면 화면 가로로 t=1인 구멍이 뚫리고
      **그 구멍을 정답이라고 학습**합니다. 과검출이 미검출보다 훨씬 비쌉니다.

      내 데이터에서 오검출이 나는지는 `python -m aodnet.synth -i <디렉터리>`가
      화염 검출 비율을 같이 출력해 주니 그걸로 확인하세요.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h = hsv[..., 0].astype(np.float32)
    s = hsv[..., 1].astype(np.float32)
    v = hsv[..., 2].astype(np.float32)

    warm = (h <= 20) | (h >= 170)
    body = warm & (s > 150) & (v > 200)
    core = (v >= 245) & (s < 100)

    empty = np.zeros(bgr.shape[:2], np.float32)
    if not (body.any() and core.any()):
        return empty

    # 코어와 몸통은 인접해 있지만 경계에서 한두 픽셀 끊길 수 있습니다.
    # 살짝 닫아준 뒤 하나의 덩어리로 셉니다.
    blob = cv2.morphologyEx((body | core).astype(np.uint8),
                            cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(blob, connectivity=8)

    frame_area = float(bgr.shape[0] * bgr.shape[1])
    both = np.intersect1d(np.unique(labels[body]), np.unique(labels[core]))

    keep = np.zeros(n_labels, bool)
    for lab in both:
        if lab == 0:                                    # 0은 배경
            continue
        w_box = stats[lab, cv2.CC_STAT_WIDTH]
        h_box = stats[lab, cv2.CC_STAT_HEIGHT]
        area = stats[lab, cv2.CC_STAT_AREA]
        aspect = max(w_box, h_box) / max(min(w_box, h_box), 1)
        if aspect <= FIRE_MAX_ASPECT and area <= FIRE_MAX_AREA_RATIO * frame_area:
            keep[lab] = True

    if not keep.any():
        return empty

    mask = keep[labels].astype(np.float32)
    mask = cv2.GaussianBlur(mask, (0, 0), 2.0)
    return np.clip(mask, 0.0, 1.0)


def synthesize(clear_bgr: np.ndarray,
               rng: np.random.Generator,
               cfg: SmokeConfig | None = None,
               depth: np.ndarray | None = None,
               fire_mask_override: np.ndarray | None = None
               ) -> tuple[np.ndarray, np.ndarray, SmokeMeta]:
    """깨끗한 장면 J -> 연기 낀 관측 I 를 합성합니다.

    clear_bgr : uint8 BGR (H, W, 3)
    depth     : 0~1 float32 (없으면 유사 깊이 생성). 실제 depth가 있으면
                미터 단위를 0~1로 정규화해서 넣으세요.
    반환      : (hazy uint8, target uint8, SmokeMeta)

    ★ 정답을 입력 clear_bgr 이 아니라 **반환값 target** 으로 쓰세요.
      lowlight_on_target=True 면 정답도 같은 배율로 어두워지므로, clear_bgr을
      그냥 정답으로 쓰면 밝기가 어긋난 쌍으로 학습하게 됩니다.
    """
    cfg = cfg or SmokeConfig()

    # ★ 화염은 **연기 합성 전에** 넣습니다. 정답 J에 불이 있어야
    #   preserve_fire가 "불씨는 연기 너머로도 보인다"를 가르칠 수 있습니다.
    # 호출자가 이미 화염을 합성했다면 그 마스크를 그대로 받습니다.
    # (합성 위치를 아는데 fire_mask로 다시 찾으면 글로우가 주변을 오검출합니다)
    fire_boxes: list = []
    known_fire: np.ndarray | None = fire_mask_override
    if known_fire is None and cfg.fire_prob > 0 and rng.random() < cfg.fire_prob:
        n_fire = int(rng.integers(cfg.fire_count[0], cfg.fire_count[1] + 1))
        clear_bgr, fire_boxes, known_fire = composite_fire(
            clear_bgr, rng, n_flames=n_fire, scale=cfg.fire_scale)

    h, w = clear_bgr.shape[:2]

    if depth is None:
        depth = pseudo_depth(h, w, rng)
    else:
        depth = np.clip(depth.astype(np.float32), 0.0, 1.0)
        if depth.shape[:2] != (h, w):
            depth = cv2.resize(depth, (w, h), interpolation=cv2.INTER_LINEAR)

    n_src = int(rng.integers(cfg.n_sources[0], cfg.n_sources[1] + 1))
    plume = plume_field(h, w, rng, n_sources=n_src)

    w_d = rng.uniform(*cfg.depth_weight)
    w_p = rng.uniform(*cfg.plume_weight)
    beta = rng.uniform(*cfg.beta_range)

    optical_depth = beta * (w_d * depth + w_p * plume)
    t = np.exp(-optical_depth).astype(np.float32)

    if cfg.preserve_fire:
        # 화염은 산란 모델의 가정(수동 반사체) 밖입니다. t=1로 되돌려
        # "불씨는 연기 너머로도 보인다"를 학습 데이터에 새깁니다.
        # 이걸 안 하면 망이 밝은 주황 영역을 연기로 오인해 지웁니다.
        #
        # ★ 우리가 합성한 불이면 **정확한 마스크를 그대로** 씁니다.
        #   fire_mask()로 다시 찾으면 화염의 글로우가 주변 노란 차선을 밝혀
        #   그게 화염으로 오검출됩니다(실측 5.01%). 검출은 불 위치를 모르는
        #   실사에서만 쓰는 폴백입니다.
        fm = known_fire if known_fire is not None else fire_mask(clear_bgr)
        t = t + (1.0 - t) * fm

    t = np.clip(t, cfg.t_floor, 1.0)

    a_vec, style = sample_airlight(rng, cfg.airlight_style, cfg.style_probs)
    a = a_vec.reshape(1, 1, 3)

    j = clear_bgr.astype(np.float32) / 255.0

    # ★ 저조도는 **합성 전에** J에 겁니다. 순서가 중요합니다.
    #
    #   맞는 방식 (여기):   I = (g·J)·t + A(1-t)
    #     "어두운 방"은 장면 J 자체가 어둡다는 뜻입니다. 대기광 A도 그 방의
    #     조명을 받으므로 어둡고(sample_airlight의 gray/sooty가 담당),
    #     둘이 물리적으로 일관됩니다.
    #
    #   틀린 방식:          I = g·(J·t + A(1-t))
    #     합성 결과 전체에 배율을 거는 건 렌즈에 ND 필터를 씌운 것과 같습니다.
    #     밝은 대기광 A가 g로 눌려버려 "어두운 방인데 대기광만 흰색"이라는
    #     물리적으로 없는 조합이 데이터에 섞입니다.
    gain = 1.0
    if cfg.lowlight is not None:
        gain = float(rng.uniform(*cfg.lowlight))
        j_dark = j * gain
    else:
        j_dark = j

    i = j_dark * t[..., None] + a * (1.0 - t[..., None])
    target = j_dark if cfg.lowlight_on_target else j

    i = np.clip(i * 255.0, 0, 255)

    if cfg.noise_sigma > 0:
        # 노이즈는 어둡게 만든 **뒤에** 더합니다. 실제 센서도 광량이 적을 때
        # 상대적 노이즈가 커지므로, 이래야 저조도 노이즈가 재현됩니다.
        i = i + rng.normal(0.0, cfg.noise_sigma, i.shape)

    hazy = np.clip(i, 0, 255).astype(np.uint8)

    if cfg.jpeg_quality is not None:
        q = int(rng.integers(cfg.jpeg_quality[0], cfg.jpeg_quality[1] + 1))
        ok, buf = cv2.imencode(".jpg", hazy, [int(cv2.IMWRITE_JPEG_QUALITY), q])
        if ok:
            hazy = cv2.imdecode(buf, cv2.IMREAD_COLOR)

    # 정답에는 노이즈·JPEG를 걸지 않습니다. 정답이 더러우면 망이 노이즈를
    # 복원하도록 배웁니다.
    target_u8 = np.clip(target * 255.0, 0, 255).astype(np.uint8)

    meta = SmokeMeta(transmission=t, airlight=a_vec, beta=float(beta), style=style,
                     extra={"depth_weight": float(w_d), "plume_weight": float(w_p),
                            "n_sources": n_src, "gain": gain,
                            "fire_boxes": fire_boxes})
    return hazy, target_u8, meta


# ---------------------------------------------------------------- 장면 생성기


def make_scene(w: int = 640, h: int = 480, rng: np.random.Generator | None = None):
    """깨끗한 실내 장면 J 를 절차적으로 생성 (연기 없는 정답).

    실제 로봇 프레임이 없을 때의 대체재입니다. phase1 tools/make_synthetic.py 의
    지하주차장 장면을 랜덤화·다양화한 버전.

    ★ 한계를 분명히 하고 씁니다: 이런 도형 장면만으로 학습한 망은 실제
      텍스처(콘크리트 결, 배관, 표지판)에서 덜 정확합니다. 진짜 카메라
      프레임이 몇 백 장이라도 생기면 즉시 --clear-dir 로 갈아타세요.
    """
    rng = rng or np.random.default_rng()
    img = np.zeros((h, w, 3), np.uint8)

    # ★ cv2 5.x 는 color 인자에 numpy 정수를 받지 않습니다(OverloadResolution 실패).
    #   rng.integers()는 np.int64를 주므로 반드시 파이썬 int로 감싸세요.
    def gray(v: float) -> tuple[int, int, int]:
        v = int(np.clip(v, 0, 255))
        return (v, v, v)

    wall = int(rng.integers(40, 110))
    img[:, :] = (np.clip(wall + int(rng.integers(-6, 7)), 0, 255),
                 np.clip(wall + int(rng.integers(-4, 5)), 0, 255), wall)

    cv2.rectangle(img, (0, 0), (w, int(h * rng.uniform(0.2, 0.35))), gray(wall - 18), -1)
    cv2.rectangle(img, (0, int(h * rng.uniform(0.72, 0.85))), (w, h), gray(wall - 8), -1)

    # 기둥 / 벽면 구조물
    for _ in range(int(rng.integers(1, 4))):
        x0 = int(rng.uniform(0.02, 0.85) * w)
        pw = int(rng.uniform(0.05, 0.18) * w)
        y0 = int(rng.uniform(0.1, 0.35) * h)
        y1 = int(rng.uniform(0.6, 0.95) * h)
        c = int(np.clip(wall + int(rng.integers(20, 70)), 0, 255))
        cv2.rectangle(img, (x0, y0), (x0 + pw, y1), gray(c), -1)

    # 소화기/제어반 같은 색깔 있는 저장애물 (YOLO 타깃 대용)
    for _ in range(int(rng.integers(1, 4))):
        x0 = int(rng.uniform(0.05, 0.8) * w)
        y0 = int(rng.uniform(0.5, 0.8) * h)
        bw, bh = int(rng.uniform(0.05, 0.15) * w), int(rng.uniform(0.08, 0.2) * h)
        col = tuple(int(c) for c in rng.integers(30, 220, 3))
        cv2.rectangle(img, (x0, y0), (x0 + bw, y0 + bh), col, -1)
        cv2.rectangle(img, (x0, y0), (x0 + bw, y0 + bh),
                      tuple(int(c * 0.6) for c in col), 2)

    # 고대비 얇은 선 (주차선/배관) — 선명도 손실에 가장 민감한 요소
    for _ in range(int(rng.integers(2, 6))):
        p0 = (int(rng.uniform(0, 1) * w), int(rng.uniform(0.7, 1.0) * h))
        p1 = (p0[0] + int(rng.uniform(-0.2, 0.2) * w), int(rng.uniform(0.55, 0.8) * h))
        v = int(rng.integers(150, 235))
        cv2.line(img, p0, p1, (v, v, v), int(rng.integers(1, 3)))

    # 화염 (있을 때도 없을 때도 있어야 합니다)
    if rng.random() < 0.6:
        fx, fy = int(rng.uniform(0.15, 0.85) * w), int(rng.uniform(0.4, 0.75) * h)
        r = int(rng.uniform(0.01, 0.05) * w) + 3
        cv2.circle(img, (fx, fy), r * 2, (10, 60, 150), -1)
        cv2.circle(img, (fx, fy), r, (40, 150, 250), -1)
        cv2.circle(img, (fx, fy), max(2, r // 2), (200, 240, 255), -1)

    noise = rng.normal(0, 3.0, (h, w, 3))
    return np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)


def load_depth_map(depth_dir, stem: str) -> np.ndarray | None:
    """정합된 depth 파일을 찾아 0~1로 정규화. 없으면 None.

    ★ RealSense의 depth 0은 "0미터"가 아니라 **측정 실패**입니다. 그대로 쓰면
      그 픽셀만 연기 농도가 0이 되어 학습 데이터에 구멍이 뚫립니다.
      최댓값이 아니라 99퍼센타일로 나누는 것도 이유가 있습니다 — depth 맵에는
      튀는 이상치가 몇 픽셀씩 있고, 최댓값으로 나누면 나머지가 전부 0 근처로 눌립니다.
    """
    from pathlib import Path as _Path

    if depth_dir is None:
        return None
    depth_dir = _Path(depth_dir)
    for ext in (".png", ".tif", ".tiff", ".npy"):
        p = depth_dir / (stem + ext)
        if not p.exists():
            continue
        d = np.load(p) if ext == ".npy" else cv2.imread(str(p), cv2.IMREAD_UNCHANGED)
        if d is None:
            continue
        d = d.astype(np.float32)
        valid = d > 0
        if not valid.any():
            return None
        d[~valid] = d[valid].max()
        return np.clip(d / (np.percentile(d[valid], 99) + 1e-6), 0.0, 1.0)
    return None


if __name__ == "__main__":
    import argparse
    import os
    from pathlib import Path

    ap = argparse.ArgumentParser(
        description="연기 합성 미리보기 — 내 이미지로 확인하려면 -i 를 쓰세요",
        epilog="예) python -m aodnet.synth -i ../frame_cut/data -o assets/mine.jpg")
    ap.add_argument("-i", "--clear-dir", default=None,
                    help="깨끗한(연기 없는) 이미지 파일 또는 디렉터리. "
                         "생략하면 절차적 장면을 생성합니다")
    ap.add_argument("--depth-dir", default=None,
                    help="정합된 depth 디렉터리 (파일명 stem이 같아야 함)")
    ap.add_argument("-o", "--out", default="assets/synth_preview.jpg")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--width", type=int, default=320, help="미리보기 한 칸 너비")
    ap.add_argument("--beta", type=float, default=None,
                    help="연기 농도를 고정 (생략하면 0.6~3.2 랜덤)")
    ap.add_argument("--styles", default=",".join(AIRLIGHT_STYLES),
                    help=f"쉼표 구분. 가능: {','.join(AIRLIGHT_STYLES)}")
    args = ap.parse_args()

    styles = [s.strip() for s in args.styles.split(",") if s.strip()]
    for s in styles:
        if s not in AIRLIGHT_STYLES:
            raise SystemExit(f"모르는 스타일: {s!r} (가능: {', '.join(AIRLIGHT_STYLES)})")

    # 소스 이미지 목록 (파일 하나만 줘도 됩니다)
    sources: list[Path] = []
    if args.clear_dir:
        src = Path(args.clear_dir)
        if src.is_dir():
            sources = sorted(p for p in src.rglob("*")
                             if p.suffix.lower() in (".jpg", ".jpeg", ".png", ".bmp", ".webp"))
            if not sources:
                raise SystemExit(f"이미지가 없습니다: {src}")
        elif src.exists():
            sources = [src]
        else:
            raise SystemExit(f"경로를 찾을 수 없습니다: {src}")

    rng = np.random.default_rng(args.seed)
    cell_w = args.width
    rows = []
    fire_report: list[tuple[str, float]] = []

    for k, style in enumerate(styles):
        cfg = SmokeConfig(airlight_style=style)
        if args.beta is not None:
            cfg.beta_range = (args.beta, args.beta)

        if sources:
            # 스타일마다 다른 사진을 씁니다. 같은 사진만 쓰면 "이 사진에서만
            # 잘 되는지"를 판단할 수 없습니다.
            path = sources[k % len(sources)]
            scene = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if scene is None:
                raise SystemExit(f"읽기 실패: {path}")
            depth = load_depth_map(args.depth_dir, path.stem)
            tag = path.name
        else:
            scene = make_scene(640, 480, rng)
            depth = None
            tag = "make_scene()"

        hazy, target, meta = synthesize(scene, rng, cfg, depth=depth)

        # ★ 화염 오검출 점검. 내 데이터에 노란 차선·주황 표지·경광등이 있으면
        #   여기가 몇 %씩 뜹니다. 그러면 그 영역에 t=1인 구멍이 뚫린 채로
        #   학습됩니다. 0.5%를 넘으면 눈으로 확인하세요.
        fire_pct = float((fire_mask(scene) > 0.5).mean() * 100)
        fire_report.append((tag, fire_pct))

        cell_h = int(round(cell_w * scene.shape[0] / scene.shape[1]))
        resize = lambda im: cv2.resize(im, (cell_w, cell_h))       # noqa: E731
        t_vis = cv2.applyColorMap((meta.transmission * 255).astype(np.uint8),
                                  cv2.COLORMAP_VIRIDIS)

        row = np.hstack([resize(hazy), resize(target), resize(t_vis)])
        label = np.zeros((22, row.shape[1], 3), np.uint8)
        cv2.putText(label,
                    f"{style}  beta={meta.beta:.2f}  A={np.round(meta.airlight, 2)}"
                    f"  gain={meta.extra['gain']:.2f}"
                    f"  depth={'real' if depth is not None else 'pseudo'}"
                    f"  fire={fire_pct:.2f}%  {tag}",
                    (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 255, 255), 1)
        rows.append(np.vstack([row, label]))

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    cv2.imwrite(args.out, np.vstack(rows))
    print(f"저장: {args.out}  (좌: 입력 I / 중: 정답 J / 우: 투과율 t)")

    worst = max(fire_report, key=lambda r: r[1]) if fire_report else ("", 0.0)
    if worst[1] > 0.5:
        print(f"\n⚠ 화염 마스크가 {worst[0]} 에서 {worst[1]:.2f}% 발화했습니다.")
        print("  실제로 불이 없다면 오검출입니다 — 노란 차선·주황 표지·경광등이 흔한 원인.")
        print("  그 영역은 투과율 t=1인 '구멍'이 되어 연기가 안 낀 채로 학습됩니다.")
        print("  대응: SmokeConfig(preserve_fire=False) 로 끄거나 "
              "synth.py 의 FIRE_MAX_ASPECT / FIRE_MAX_AREA_RATIO 를 조이세요.")
    elif sources:
        print(f"화염 마스크 최대 발화: {worst[1]:.2f}% ({worst[0]}) — 정상 범위")

    if sources:
        used = min(len(sources), len(styles))
        print(f"소스: {args.clear_dir}  (총 {len(sources)}장, 미리보기에 {used}장 사용)")
        if args.depth_dir is None:
            print("  ※ depth 없이 유사 깊이로 합성했습니다. 정합된 depth가 있으면 "
                  "--depth-dir 로 넘기세요 (물리적으로 정확해집니다)")
    else:
        print("소스: 절차 생성 장면. 내 이미지를 쓰려면 -i <디렉터리> 를 붙이세요")
