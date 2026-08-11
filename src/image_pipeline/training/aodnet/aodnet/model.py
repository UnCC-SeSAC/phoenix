#!/usr/bin/env python3
"""
AOD-Net (All-in-One Dehazing Network) — Li et al., ICCV 2017.

핵심 아이디어 한 줄:
    "t와 A를 따로 추정하지 말고, 하나의 K(x)로 합쳐서 CNN이 직접 추정한다."

왜 그게 중요한가 (DCP와의 근본 차이):
    대기 산란 모델        I(x) = J(x)·t(x) + A·(1 - t(x))
    역산                  J(x) = (I(x) - A)/t(x) + A

    DCP는 t와 A를 **따로** 추정합니다. 그런데 위 식에서 두 오차는 곱으로
    섞입니다. A가 조금만 틀려도 1/t 배로 증폭되고, 화재 장면처럼 화염 때문에
    A 추정이 흔들리는 곳에서는 이게 색 붕괴로 나타납니다.
    (phase1 dehaze.py 가 a_max·평균 후보 같은 방어 코드를 잔뜩 달고 있는 이유)

    AOD-Net은 그 둘을 하나로 접습니다.

        J(x) = K(x)·I(x) - K(x) + b

        K(x) = [ (1/t(x))·(I(x) - A) + (A - b) ] / (I(x) - 1)

    이제 추정 대상이 K(x) 하나뿐이라 "오차의 곱" 자체가 사라집니다.
    그리고 K는 이미지 도메인의 매끄러운 맵이라 아주 작은 CNN으로 충분합니다.
    b는 상수 바이어스로 논문 기본값 1.0.

구조 (논문 그대로, conv 5개):
    conv1: 3ch  -> 3ch, k=1
    conv2: 3ch  -> 3ch, k=3
    conv3: 6ch  -> 3ch, k=5   (concat conv1, conv2)
    conv4: 6ch  -> 3ch, k=7   (concat conv2, conv3)
    conv5: 12ch -> 3ch, k=3   (concat conv1..conv4)   -> K(x)

    커널이 1,3,5,7로 커지는 건 multi-scale 수용영역입니다. 연기 농도는
    큰 스케일 정보(전역 흐림)와 작은 스케일 정보(윤곽 보존)가 둘 다 필요합니다.
    concat은 얕은 층의 고주파 정보가 마지막까지 살아남게 하는 통로입니다.

파라미터 수 1,761개. ResNet-18의 1/6000 수준이라 CPU에서도 실시간이 나옵니다.
로봇 온보드에 얹는 게 목표라면 이 크기가 곧 경쟁력입니다.
"""

from __future__ import annotations

import torch
import torch.nn as nn


class AODNet(nn.Module):
    """AOD-Net.

    입력/출력 모두 (N, 3, H, W) float32, 0~1 범위 **RGB**.
    (OpenCV BGR을 그대로 넣으면 안 됩니다 — infer.py 가 변환을 담당합니다.)
    """

    def __init__(self, b: float = 1.0, learn_b: bool = False):
        super().__init__()

        # ★ b를 학습시킬지 — 논문 기본은 상수 1.0입니다. 왜 옵션을 뒀는지:
        #
        #   K는 픽셀마다 K = (J - b)/(I - 1) 로 결정됩니다. b=1이면 분모가
        #   (I - 1) 인데, **어두운 장면에서 I ≈ 0.1 이면 분모가 -0.9 로 거의
        #   상수**가 됩니다. 즉 K의 변화폭이 눌려서, 같은 복원량을 내려면
        #   K를 훨씬 정밀하게 맞춰야 합니다. 조건수가 나쁜 상태입니다.
        #
        #   증상: 검은 연기(sooty)·저조도 장면에서만 복원이 약하고 엣지가
        #        정답보다 낮게 나옵니다. tools/compare.py 의 스타일별 표에서
        #        바로 보입니다.
        #
        #   b를 학습 가능한 스칼라(파라미터 1개 추가)로 두면 망이 자기 데이터
        #   분포에 맞는 기준점을 잡습니다. 야외 안개(RESIDE)처럼 밝은 데이터만
        #   쓴다면 b=1.0 상수로 충분하고, 굳이 켤 이유가 없습니다.
        if learn_b:
            self.b = nn.Parameter(torch.tensor(float(b)))
        else:
            self.b = float(b)     # 그냥 상수. state_dict에 키가 생기지 않습니다.
        self.learn_b = bool(learn_b)

        # padding은 전부 'same' 크기가 되도록. 입력과 출력 해상도가 같아야
        # K(x)를 픽셀 단위로 I(x)에 곱할 수 있습니다.
        self.conv1 = nn.Conv2d(3, 3, kernel_size=1, padding=0, bias=True)
        self.conv2 = nn.Conv2d(3, 3, kernel_size=3, padding=1, bias=True)
        self.conv3 = nn.Conv2d(6, 3, kernel_size=5, padding=2, bias=True)
        self.conv4 = nn.Conv2d(6, 3, kernel_size=7, padding=3, bias=True)
        self.conv5 = nn.Conv2d(12, 3, kernel_size=3, padding=1, bias=True)

        self._init_weights()

    def _init_weights(self) -> None:
        """논문 설정: 가우시안(std=0.02) 초기화, bias 0.

        Kaiming 초기화를 쓰면 초기 K가 커져서 K·I - K + b 가 발산하기 쉽습니다.
        출력이 곱셈으로 들어가는 구조라 초기값이 작아야 안전합니다.
        """
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.normal_(m.weight, mean=0.0, std=0.02)
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0.0)

    # ------------------------------------------------------------------

    def estimate_k(self, x: torch.Tensor) -> torch.Tensor:
        """K(x) 맵만 반환. 시각화·디버깅용.

        K는 "이 픽셀을 얼마나 세게 복원할지"의 지도입니다. 연기가 진한 곳에서
        커지고 맑은 곳에서 1 근처가 됩니다. 학습이 망가졌는지 판단할 때
        결과 이미지보다 이 맵을 보는 게 훨씬 빠릅니다.
        """
        # inplace=False 로 둡니다. x1, x2 를 뒤에서 다시 concat 하므로
        # inplace ReLU 를 쓰면 autograd가 덮어쓴 텐서를 참조해 터집니다.
        relu = torch.nn.functional.relu

        x1 = relu(self.conv1(x))
        x2 = relu(self.conv2(x1))
        cat1 = torch.cat((x1, x2), dim=1)

        x3 = relu(self.conv3(cat1))
        cat2 = torch.cat((x2, x3), dim=1)

        x4 = relu(self.conv4(cat2))
        cat3 = torch.cat((x1, x2, x3, x4), dim=1)

        k = relu(self.conv5(cat3))
        return k

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        k = self.estimate_k(x)
        # J = K·I - K + b  ...  논문 식 (3)
        # 마지막 ReLU는 음수 픽셀을 잘라내는 역할. 여기서 clamp(0,1)까지 하면
        # 1 근처에서 기울기가 0이 되어 밝은 영역(화염 주변) 학습이 죽습니다.
        # 1 초과는 학습 중엔 그대로 두고 추론 시점에만 자릅니다.
        return torch.nn.functional.relu(k * x - k + self.b)


def count_parameters(model: nn.Module) -> int:
    return sum(p.numel() for p in model.parameters() if p.requires_grad)


def load_aodnet(state: dict, device=None) -> AODNet:
    """체크포인트에서 구조를 알아서 맞춰 모델을 만듭니다.

    learn_b 여부는 state_dict에 'b' 키가 있는지로 판별합니다. 이걸 안 하면
    학습 때 켜둔 옵션을 추론에서 기억해야 하고, 잊으면 조용히 다른 모델이 됩니다.
    """
    net = AODNet(learn_b="b" in state)
    net.load_state_dict(state)
    return net.to(device) if device is not None else net


if __name__ == "__main__":
    net = AODNet()
    dummy = torch.rand(1, 3, 480, 640)
    out = net(dummy)
    print(f"파라미터 수 : {count_parameters(net):,}  (논문값 1,761)")
    print(f"learn_b 시  : {count_parameters(AODNet(learn_b=True)):,}")
    print(f"입력        : {tuple(dummy.shape)}")
    print(f"출력        : {tuple(out.shape)}")
    print(f"K 맵        : {tuple(net.estimate_k(dummy).shape)}")
