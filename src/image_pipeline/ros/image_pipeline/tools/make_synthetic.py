#!/usr/bin/env python3
"""
합성 테스트 장면 생성기 — **정답을 아는** 입력을 만듭니다.

인센스 사진으로는 "연기 없는 원본"이 없어서 "얼마나 잘 복원했는가"를
숫자로 못 잽니다. 여기서는 깨끗한 장면 J를 먼저 만들고 물리 모델

    I = J·t + A·(1 - t),    t = exp(-beta · d)

로 연기를 **합성**하므로 J가 정답으로 남습니다. 디헤이즈 결과를 J와
비교하면 PSNR을 잴 수 있고, 이게 pytest의 판정 기준이 됩니다.

  I : 관측된(뿌연) 영상    J : 원래 장면      A : 대기광
  t : 투과율(0~1)          d : 깊이           beta : 연기 농도

지하주차장을 흉내낸 장면입니다: 어두운 바닥/천장, 벽면 기둥,
안쪽으로 갈수록 멀어지는 깊이, 그리고 **성냥불 크기의 밝은 점**.

사용:
  python3 tools/make_synthetic.py -o testdata/
  python3 tools/make_synthetic.py -o testdata/ --beta 2.5 --dark 0.25
"""

from __future__ import annotations

import argparse
import os

import cv2
import numpy as np


def make_scene(w: int = 640, h: int = 480, seed: int = 0):
    """깨끗한 장면 J(uint8 BGR)와 깊이맵 d(0~1 float)를 만듭니다."""
    rng = np.random.default_rng(seed)

    img = np.zeros((h, w, 3), np.uint8)
    img[:, :] = (55, 58, 62)                      # 콘크리트 바닥/전반

    cv2.rectangle(img, (0, 0), (w, int(h * 0.28)), (38, 40, 44), -1)      # 천장
    cv2.rectangle(img, (0, int(h * 0.78)), (w, h), (48, 50, 54), -1)      # 바닥

    # 기둥 두 개 (원근감 있게 크기 차이)
    cv2.rectangle(img, (60, 90), (155, 400), (105, 108, 112), -1)
    cv2.rectangle(img, (470, 130), (530, 360), (92, 95, 100), -1)

    # 벽면 소화기 박스 (태스크④에서 말한 '낮은 장애물')
    cv2.rectangle(img, (250, 320), (330, 395), (40, 55, 150), -1)
    cv2.rectangle(img, (250, 320), (330, 395), (30, 40, 110), 2)

    # 주차선 (고대비 얇은 선 — 선명도 지표에 민감하게 반응)
    for x in range(180, w - 60, 120):
        cv2.line(img, (x, h - 10), (x + 40, int(h * 0.80)), (190, 195, 200), 2)

    # 비상등 (약한 광원)
    cv2.circle(img, (int(w * 0.75), 70), 14, (90, 130, 150), -1)

    # ★ 성냥불 — 작고 밝고 채도 높음. DCP 대기광 추정을 교란하는 주범
    fire = (int(w * 0.5), int(h * 0.52))
    cv2.circle(img, fire, 9, (40, 140, 250), -1)
    cv2.circle(img, fire, 4, (200, 240, 255), -1)

    # 센서 노이즈
    noise = rng.normal(0, 3.5, (h, w, 3))
    img = np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)

    # 깊이: 화면 안쪽(위)일수록 멀고, 좌우 가장자리는 가까움
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    depth = 0.25 + 0.75 * (1.0 - yy / h)
    depth *= 1.0 - 0.25 * np.abs(xx / w - 0.5) * 2.0
    depth = np.clip(depth, 0.05, 1.0)

    return img, depth.astype(np.float32), fire


def add_haze(clear, depth, beta: float = 1.8, a=(0.78, 0.80, 0.82), seed: int = 1):
    """물리 모델로 연기를 합성. (뿌연 영상, 정답 투과율 t, 정답 A) 반환."""
    rng = np.random.default_rng(seed)
    a_arr = np.array(a, np.float32).reshape(1, 1, 3)

    # 연기는 균일하지 않습니다. 저주파 난류를 곱해 얼룩을 만듭니다.
    turb = rng.normal(0, 1, (depth.shape[0] // 16 + 1, depth.shape[1] // 16 + 1))
    turb = cv2.resize(turb, (depth.shape[1], depth.shape[0]), interpolation=cv2.INTER_CUBIC)
    turb = cv2.GaussianBlur(turb, (0, 0), 12)
    turb = 1.0 + 0.35 * turb / (np.abs(turb).max() + 1e-6)

    t = np.exp(-beta * depth * turb).astype(np.float32)
    t = np.clip(t, 0.02, 1.0)

    j = clear.astype(np.float32) / 255.0
    i = j * t[..., None] + a_arr * (1.0 - t[..., None])
    hazy = np.clip(i * 255.0, 0, 255).astype(np.uint8)
    return hazy, t, a_arr.reshape(3)


def add_lowlight(img, factor: float = 0.22, noise_sigma: float = 4.0, seed: int = 2):
    """저조도: 밝기를 곱으로 낮추고 노이즈를 더합니다.

    노이즈를 어둡게 만든 **뒤에** 더하는 게 중요합니다. 실제 센서도
    광량이 적을 때 상대적 노이즈가 커지므로, 이래야 CLAHE가 노이즈까지
    같이 증폭하는 현상이 재현됩니다.
    """
    rng = np.random.default_rng(seed)
    dark = img.astype(np.float32) * factor
    dark += rng.normal(0, noise_sigma, img.shape)
    return np.clip(dark, 0, 255).astype(np.uint8)


def main():
    ap = argparse.ArgumentParser(description="합성 연기/저조도 테스트 장면 생성")
    ap.add_argument("-o", "--out", default="testdata")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--beta", type=float, default=1.8, help="연기 농도 (클수록 진함)")
    ap.add_argument("--dark", type=float, default=0.22, help="저조도 배율")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)

    clear, depth, fire = make_scene(args.width, args.height, args.seed)
    hazy, t, a = add_haze(clear, depth, args.beta, seed=args.seed + 1)
    dark = add_lowlight(clear, args.dark, seed=args.seed + 2)
    dark_hazy = add_lowlight(hazy, args.dark, seed=args.seed + 3)

    files = {
        "clear.png": clear,             # 정답 J
        "hazy.png": hazy,               # 연기
        "dark.png": dark,               # 저조도
        "dark_hazy.png": dark_hazy,     # 연기 + 저조도 (최악 조건)
        "transmission_gt.png": (t * 255).astype(np.uint8),   # 정답 투과율
    }
    for name, im in files.items():
        cv2.imwrite(os.path.join(args.out, name), im)

    np.savez(os.path.join(args.out, "groundtruth.npz"),
             transmission=t, atmospheric_light=a, depth=depth, fire_xy=np.array(fire))

    print(f"생성 위치: {args.out}/")
    for name in files:
        print(f"  {name}")
    print("  groundtruth.npz  (투과율·대기광·깊이·불씨 좌표)")
    print(f"\n정답 대기광 A(BGR) = {a.round(3)}")
    print(f"불씨 픽셀 좌표 = {fire}")
    print(f"연기 영역 평균 투과율 = {t.mean():.3f}  (낮을수록 진한 연기)")


if __name__ == "__main__":
    main()
