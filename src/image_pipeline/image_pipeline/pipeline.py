#!/usr/bin/env python3
"""
전처리 파이프라인 — mode에 따른 처리 순서. ROS 비의존.

노드 안에 if문으로 흩어놓으면 rclpy 없이는 "mode를 바꿨을 때 정말 다른
처리가 도는가"를 검증할 수 없습니다. 여기로 빼서 노드·오프라인 도구·테스트가
**같은 코드**를 쓰게 했습니다.

  passthrough : 무처리          (1단계 뼈대 — QoS·헤더 배선만 검증)
  clahe       : 감마 -> CLAHE   (2단계)
  dehaze      : 감마 -> DCP     (기여도 분리용)
  full        : 감마 -> DCP -> CLAHE       (3단계)
  aod         : 감마 -> AOD-Net (DCP 대안)
  aod_full    : 감마 -> AOD-Net -> CLAHE

AOD-Net을 **별도 노드가 아니라 mode로** 넣은 이유는 HANDOVER 4-9. 비교 도구·
정답 채점·불씨 생존 테스트·데이터셋 생성기·노드 배선이 전부 그대로 재사용되고,
A/B가 파라미터 한 줄이 됩니다.

순서가 '디헤이즈 -> CLAHE'인 이유: 디헤이즈는 물리 모델의 역산이라
**입력이 원본 관측값 I여야** 성립합니다. CLAHE를 먼저 걸면 히스토그램이
비선형으로 변형돼 I = J·t + A(1-t) 가정이 깨집니다.
"""

from __future__ import annotations

import time

from image_pipeline.dehaze import ClaheEnhancer, DarkChannelDehazer, apply_gamma

MODES = ("passthrough", "clahe", "dehaze", "full", "aod", "aod_full")

# 디헤이즈가 도는 모드와, 그중 AOD-Net을 쓰는 모드.
DEHAZE_MODES = ("dehaze", "full", "aod", "aod_full")
AOD_MODES = ("aod", "aod_full")
CLAHE_MODES = ("clahe", "full", "aod_full")


class Pipeline:
    def __init__(
        self,
        mode: str = "full",
        gamma: float = 1.0,
        lowlight: bool = False,
        clahe: ClaheEnhancer | None = None,
        dehazer: DarkChannelDehazer | None = None,
        aod=None,
    ):
        self.set_mode(mode)
        self.gamma = gamma
        self.lowlight = lowlight
        self.clahe = clahe or ClaheEnhancer()
        self.dehazer = dehazer or DarkChannelDehazer()
        # AOD-Net 디헤이저(`aodnet.GatedDehazer` 등). `process(bgr)->bgr` 인터페이스만
        # 맞으면 됩니다. 없으면 aod* 모드를 쓸 때 예외 — **DCP로 조용히 대체하지
        # 않습니다.** 대체하면 "aod 모드로 측정한 결과"가 실제로는 DCP 결과가 되어
        # A/B 실험이 통째로 무의미해집니다.
        self.aod = aod

        # 마지막 프레임의 단계별 소요시간(ms). 병목 추적용.
        self.timings: dict[str, float] = {"dehaze": 0.0, "clahe": 0.0, "total": 0.0}

    def set_mode(self, mode: str) -> None:
        if mode not in MODES:
            raise ValueError(f"mode는 {MODES} 중 하나여야 합니다 (받은 값: {mode!r})")
        self.mode = mode

    def active_dehazer(self):
        """현재 mode가 쓰는 디헤이저. aod* 모드인데 없으면 예외."""
        if self.mode in AOD_MODES:
            if self.aod is None:
                raise ValueError(
                    f"mode={self.mode!r} 인데 AOD-Net 디헤이저가 없습니다. "
                    "onnx 경로를 주거나 dehaze/full 모드를 쓰세요. "
                    "(DCP로 자동 대체하지 않습니다 — A/B 실험이 오염됩니다)"
                )
            return self.aod
        return self.dehazer

    def process(self, bgr):
        """bgr uint8 -> bgr uint8. 입력 배열은 변경하지 않습니다."""
        t_start = time.perf_counter()
        self.timings = {"dehaze": 0.0, "clahe": 0.0, "total": 0.0}

        if self.mode == "passthrough":
            self.timings["total"] = (time.perf_counter() - t_start) * 1000.0
            return bgr

        img = apply_gamma(bgr, self.gamma)

        if self.mode in DEHAZE_MODES:
            dehazer = self.active_dehazer()
            t = time.perf_counter()
            img = dehazer.process(img)
            if self.lowlight:
                if not hasattr(dehazer, "process_lowlight"):
                    # HANDOVER 4-9: AOD-Net에는 저조도 대응 경로가 없습니다.
                    # 조용히 건너뛰면 "저조도 보정이 켜진 줄 알았는데 안 돈"
                    # 상태가 되므로 막습니다.
                    raise ValueError(
                        f"mode={self.mode!r} 의 디헤이저에는 process_lowlight가 "
                        "없습니다. AOD-Net은 저조도 대응물이 없습니다 "
                        "(HANDOVER 4-9). lowlight를 끄거나 dehaze/full을 쓰세요."
                    )
                img = dehazer.process_lowlight(img)
            self.timings["dehaze"] = (time.perf_counter() - t) * 1000.0

        if self.mode in CLAHE_MODES:
            t = time.perf_counter()
            img = self.clahe.process(img)
            self.timings["clahe"] = (time.perf_counter() - t) * 1000.0

        self.timings["total"] = (time.perf_counter() - t_start) * 1000.0
        return img
