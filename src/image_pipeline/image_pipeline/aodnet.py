#!/usr/bin/env python3
"""
경량 추론 — **torch 없이** OpenCV DNN만으로 도는 AOD-Net 디헤이저.

Raspberry Pi 5 같은 온보드에 torch(수백 MB)를 깔지 않기 위한 파일입니다.
이 파일 하나 + .onnx 하나만 복사하면 됩니다. 의존성은 opencv-python, numpy.

    python tools/export_onnx.py --weights runs/mine2/best.pt -o deploy/aodnet_k.onnx
    # deploy/aod_lite.py 와 deploy/aodnet_k.onnx 를 로봇으로 복사

    from aod_lite import AODNetLite, SmokeGate, GatedDehazer

    aod = AODNetLite("aodnet_k.onnx", k_size=(256, 192))
    clean = aod.process(frame)

phase1 파이프라인에 그대로 꽂으려면 GatedDehazer 를 쓰세요. DarkChannelDehazer
와 같은 인터페이스(process/reset_state/timings)라 노드 코드를 안 고쳐도 됩니다.

    from image_pipeline.pipeline import Pipeline
    Pipeline(mode="full", dehazer=GatedDehazer("aodnet_k.onnx", baseline=0.3910))

설계 요점 — 왜 신경망이 K까지만 하는가
---------------------------------------
    J = K·I - K + b

K는 연기 농도 지도라 **저주파**입니다. 축소본(256x192)에서 뽑아 업샘플해도
손실이 거의 없고, 오히려 고주파 잡음이 걸러져 품질이 올라갑니다.
반면 **복원식은 원본 해상도에서** 계산해야 장면의 고주파(얇은 선·글자·윤곽)가
살아남습니다. 출력 이미지를 업샘플하면 그게 전부 뭉개집니다.

실측 (mine2, 테스트 48장, 입력 640x480):

    K 해상도    PSNR    SSIM    대비이득   CPU 4스레드(x86)
    640x480    17.22   0.8288   0.63       23.8 ms
    320x240    17.37   0.8326   0.70        6.9 ms
    256x192    17.45   0.8333   0.72        5.7 ms   <- 더 빠르고 더 좋음

phase1 DCP가 투과율 t만 축소 해상도에서 추정하는 것과 같은 논리입니다.
"""

from __future__ import annotations

import time

import cv2
import numpy as np

__all__ = ["AODNetLite", "SmokeGate", "GatedDehazer"]


class AODNetLite:
    """OpenCV DNN 기반 AOD-Net 디헤이저. torch 불필요.

    onnx_path : tools/export_onnx.py --output k 로 만든 K 전용 모델
    k_size    : K를 추정할 해상도 (W, H). 작을수록 빠르고, 256x192가 기본.
                ★ 동적 축으로 내보낸 ONNX여야 런타임에 바꿀 수 있습니다.
                  고정 크기로 내보냈다면 그 크기와 반드시 일치해야 합니다.
    b         : 복원식의 상수. 학습 때 값과 같아야 합니다(기본 1.0).
                --learn-b 로 학습했다면 그 값을 넣으세요.
    strength  : 복원 세기. K' = 1 + strength·(K - 1).
                실제 영상이 학습 분포보다 연기가 옅어 과복원(색 튐)이 날 때
                재학습 없이 0.7 근처로 내려 잡습니다. 0이면 무처리.
    threads   : OpenCV 스레드 수. Pi 5(4코어)에서 ROS 노드·Hailo 드라이버와
                코어를 나눠 쓰므로 3 정도로 묶는 게 안전합니다. 0이면 건드리지 않음.
    """

    def __init__(self,
                 onnx_path: str,
                 k_size: tuple[int, int] = (256, 192),
                 b: float = 1.0,
                 strength: float = 1.0,
                 threads: int = 0):
        self.net = cv2.dnn.readNetFromONNX(str(onnx_path))
        self.k_size = (int(k_size[0]), int(k_size[1]))
        self.b = float(b)
        self.strength = float(strength)

        if threads > 0:
            cv2.setNumThreads(int(threads))

        self.last_k: np.ndarray | None = None
        self.timings: dict[str, float] = {"infer": 0.0, "total": 0.0}

        # 첫 호출은 내부 버퍼 할당 때문에 느립니다. 미리 태워둡니다.
        # ROS 노드에서 이걸 빼면 첫 프레임에 타임아웃이 납니다.
        self._warmup()

    def _warmup(self) -> None:
        dummy = np.zeros((self.k_size[1] * 2, self.k_size[0] * 2, 3), np.uint8)
        self.process(dummy)

    def reset_state(self) -> None:
        """프레임 간 상태 없음 (DCP와 API를 맞추기 위한 no-op).

        AOD-Net은 같은 입력 -> 항상 같은 출력입니다. DCP가 대기광 A를 매 프레임
        추정해 깜빡였던 문제(phase1 a_smoothing)가 여기엔 없습니다.
        """

    # ------------------------------------------------------------------

    def estimate_k(self, bgr: np.ndarray) -> np.ndarray:
        """K 맵을 원본 해상도로 반환 (H, W, 3) float32."""
        h, w = bgr.shape[:2]
        kw, kh = self.k_size

        small = cv2.resize(bgr, (kw, kh), interpolation=cv2.INTER_AREA)
        # blobFromImage: /255 스케일 + BGR->RGB + NCHW. 학습 전처리와 동일해야 합니다.
        # swapRB를 빠뜨리면 색이 뒤집히는데, 증상이 조용해서 제일 찾기 어렵습니다.
        blob = cv2.dnn.blobFromImage(small, 1.0 / 255.0, swapRB=True)
        self.net.setInput(blob)
        k = self.net.forward()[0].transpose(1, 2, 0)      # (kh, kw, 3) RGB 순서

        if (kw, kh) != (w, h):
            k = cv2.resize(k, (w, h), interpolation=cv2.INTER_LINEAR)
        return k

    def process(self, bgr: np.ndarray) -> np.ndarray:
        """bgr uint8 -> bgr uint8. 입력 배열은 변경하지 않습니다."""
        t0 = time.perf_counter()

        k = self.estimate_k(bgr)
        self.timings["infer"] = (time.perf_counter() - t0) * 1000.0

        if self.strength != 1.0:
            k = 1.0 + self.strength * (k - 1.0)
        self.last_k = k

        # 복원식은 원본 해상도에서. K는 RGB 순서로 나왔으므로 I도 RGB로 맞춥니다.
        i_rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        j_rgb = np.clip(k * i_rgb - k + self.b, 0.0, 1.0)
        result = cv2.cvtColor((j_rgb * 255.0 + 0.5).astype(np.uint8), cv2.COLOR_RGB2BGR)

        self.timings["total"] = (time.perf_counter() - t0) * 1000.0
        return result

    def k_visualization(self) -> np.ndarray | None:
        """마지막 K 맵을 컬러맵으로. 밝을수록 강하게 복원한 영역."""
        if self.last_k is None:
            return None
        k = self.last_k.mean(axis=2)
        norm = (k - k.min()) / (np.ptp(k) + 1e-6)
        return cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_INFERNO)


# ---------------------------------------------------------------- 게이트


class SmokeGate:
    """연기가 있을 때만 True.

    AOD-Net은 **모든 입력에서 이득이 아닙니다.** 실측(테스트 48장):

        흰 연기   +6.8 dB      검은 연기(sooty)  -4.9 dB
        회색 연기 +2.0 dB      화염빛 연기       +1.8 dB

    검은 연기는 화면을 별로 바꾸지 않아 복원할 게 없는데 건드리면 손해입니다.
    연기 지표로 걸러내면 이득이 거의 두 배가 됩니다:

        항상 켬 +2.14 dB  ->  게이팅 +3.33 dB   (무처리 대비)

    baseline  : 연기 없는 프레임에서의 지표 중앙값. 장면 고유 오프셋을 뺍니다.
                ★ 절대값을 쓰면 안 됩니다 — 연기가 없어도 0.4~0.47이 나옵니다.
                  카메라나 장소가 바뀌면 다시 재세요 (measure_baseline).
    threshold : tools/tune_gate.py 로 검증셋에서 정한 값. 실측 최적 0.20.
    smoothing : 지표의 EMA 계수. ★ 0으로 두면 임계값 근처에서 프레임마다
                처리/무처리가 뒤집혀 화면이 깜빡입니다.
                실측: 60프레임 시퀀스에서 전환 27회 -> EMA 0.7 적용 시 2회.
                정지영상 평가에서만 0으로 두세요.
    """

    def __init__(self, baseline: float, threshold: float = 0.20,
                 smoothing: float = 0.7):
        # phase1 지표를 그대로 씁니다. 여기서 다시 구현하면 두 구현이 갈라집니다.
        from image_pipeline.autotune import estimate_haze_index, relative_haze
        self._index = estimate_haze_index
        self._relative = relative_haze

        self.baseline = float(baseline)
        self.threshold = float(threshold)
        self.smoothing = float(smoothing)
        self.ema: float | None = None
        self.last_relative: float = 0.0

    def reset_state(self) -> None:
        self.ema = None

    def relative(self, bgr: np.ndarray) -> float:
        return float(self._relative(self._index(bgr)[0], self.baseline))

    def should_process(self, bgr: np.ndarray) -> bool:
        rel = self.relative(bgr)
        self.last_relative = rel
        if self.smoothing <= 0:
            return rel >= self.threshold
        self.ema = rel if self.ema is None else \
            self.smoothing * self.ema + (1.0 - self.smoothing) * rel
        return self.ema >= self.threshold


def measure_baseline(frames, limit: int = 60) -> float:
    """연기 없는 프레임들로 기준선을 잽니다. frames: BGR 이미지 이터러블.

    평균이 아니라 **중앙값**입니다. 몇 장에 연기가 섞여 들어와도 안 흔들리게.
    """
    from image_pipeline.autotune import estimate_haze_index
    vals = [estimate_haze_index(f)[0] for i, f in enumerate(frames) if i < limit]
    if not vals:
        raise ValueError("프레임이 없습니다")
    return float(np.median(vals))


# ---------------------------------------------------------------- 조합


class GatedDehazer:
    """AODNetLite + SmokeGate. phase1 DarkChannelDehazer 자리에 그대로 꽂힙니다.

        Pipeline(mode="full", dehazer=GatedDehazer("aodnet_k.onnx", baseline=0.3910))

    연기가 없다고 판단하면 입력을 **그대로 반환**합니다. 그래서 평균 처리 비용이
    처리 비율만큼 줄어듭니다(실측 기준 약 절반).

    ★ DCP로 폴백하지 않는 이유: 검은 연기에서 DCP는 무처리보다 8.8 dB,
      AOD보다 4.1 dB 나쁩니다. 이 도메인에서 DCP가 이기는 조건이 없습니다.
    """

    def __init__(self, onnx_path: str, baseline: float,
                 threshold: float = 0.20, smoothing: float = 0.7,
                 k_size: tuple[int, int] = (256, 192),
                 b: float = 1.0, strength: float = 1.0, threads: int = 0):
        self.dehazer = AODNetLite(onnx_path, k_size, b, strength, threads)
        self.gate = SmokeGate(baseline, threshold, smoothing)
        self.timings: dict[str, float] = {"gate": 0.0, "infer": 0.0, "total": 0.0}
        self.processed = False          # 마지막 프레임을 실제로 처리했는지

    def reset_state(self) -> None:
        """새 bag/영상을 시작할 때 호출. 게이트의 EMA를 초기화합니다."""
        self.gate.reset_state()
        self.dehazer.reset_state()

    def process(self, bgr: np.ndarray) -> np.ndarray:
        t0 = time.perf_counter()

        run = self.gate.should_process(bgr)
        self.timings["gate"] = (time.perf_counter() - t0) * 1000.0
        self.processed = run

        if not run:
            self.timings["infer"] = 0.0
            self.timings["total"] = self.timings["gate"]
            return bgr

        out = self.dehazer.process(bgr)
        self.timings["infer"] = self.dehazer.timings["total"]
        self.timings["total"] = (time.perf_counter() - t0) * 1000.0
        return out


# ---------------------------------------------------------------- 자가 점검


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="aod_lite 자가 점검 / 벤치마크")
    ap.add_argument("onnx")
    ap.add_argument("-i", "--image", default=None, help="없으면 랜덤 이미지")
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("--k-size", default="256x192")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--threads", type=int, default=0)
    ap.add_argument("--runs", type=int, default=30)
    args = ap.parse_args()

    kw, kh = (int(v) for v in args.k_size.lower().split("x"))
    aod = AODNetLite(args.onnx, k_size=(kw, kh), threads=args.threads)

    if args.image:
        img = cv2.imread(args.image, cv2.IMREAD_COLOR)
        if img is None:
            raise SystemExit(f"읽기 실패: {args.image}")
    else:
        img = (np.random.rand(args.height, args.width, 3) * 255).astype(np.uint8)

    out = aod.process(img)
    for _ in range(args.runs):
        aod.process(img)

    ts = []
    for _ in range(args.runs):
        t0 = time.perf_counter()
        aod.process(img)
        ts.append((time.perf_counter() - t0) * 1000.0)

    h, w = img.shape[:2]
    print(f"입력      : {w}x{h}")
    print(f"K 해상도  : {kw}x{kh}")
    print(f"스레드    : {cv2.getNumThreads()}")
    print(f"처리 시간 : 평균 {np.mean(ts):.1f} ms  중앙값 {np.median(ts):.1f} ms  "
          f"({1000/np.mean(ts):.0f} FPS)")
    print(f"  이 중 신경망 {aod.timings['infer']:.1f} ms, "
          f"복원식+변환 {aod.timings['total']-aod.timings['infer']:.1f} ms")

    if args.out:
        pad = np.full((h, 4, 3), 255, np.uint8)
        cv2.imwrite(args.out, np.hstack([img, pad, out]))
        print(f"저장      : {args.out}")
