# AOD-Net 으로 화재 연기 제거하기 — 딥러닝 튜토리얼

conv 5개 · 파라미터 **1,761개**짜리 신경망으로 화재 연기 낀 영상을 복원합니다.
`phase1`의 DCP 디헤이저를 **한 줄 교체**로 대체할 수 있는 형태까지 만듭니다.

```
   연기 낀 프레임 I                    복원된 프레임 J
  ┌──────────────┐                   ┌──────────────┐
  │  ▒▒▒▒▒▒▒▒▒▒  │   AOD-Net         │              │
  │  ▒▒░░░░░▒▒▒  │  ──────────▶      │   ██   ▐▌    │
  │  ▒░ ██ ░▒▒▒  │  J = K·I - K + b  │        ▐▌    │
  └──────────────┘   (1,761 params)  └──────────────┘
```

**개발/검증 환경**: Intel Arc B580 (XPU, 12GB) · PyTorch 2.13+xpu · Python 3.13
CUDA·CPU에서도 그대로 돌아갑니다.

---

## 목차

| | |
|---|---|
| [0. 5분 만에 돌려보기](#0-5분-만에-돌려보기) | 설치부터 결과 이미지까지 |
| [1. 왜 AOD-Net인가](#1-왜-aod-net인가) | DCP와 무엇이 다른가 |
| [2. 이론](#2-이론) | K 하나로 접는 아이디어 → [상세](docs/01_theory.md) |
| [3. 설치](#3-설치) | Intel Arc / NVIDIA / CPU |
| [4. 데이터](#4-데이터) | 왜 직접 합성하는가 → [상세](docs/02_domain.md) |
| [5. 학습](#5-학습) | 40 에폭 25분 |
| [6. 평가](#6-평가) | DCP와 같은 자로 재기 → [지표 설명](docs/03_metrics.md) |
| [7. 추론과 ROS 통합](#7-추론과-ros-통합) | 드롭인 교체 |
| [7.5 온보드 배포](#75-온보드-배포-raspberry-pi-5--hailo) | torch 없이 Pi 5 + Hailo |
| [8. 함정 모음](#8-함정-모음) | 실제로 밟은 것만 |
| [9. 다음 단계](#9-다음-단계) | 여기서 더 나가려면 |

---

## 0. 5분 만에 돌려보기

```bash
cd /home/rjh/sesac3/00_final_project/aod_net
PY=../.venv/bin/python

# (1) 설치 — 3장 참고
$PY -m pip install torch torchvision --index-url https://download.pytorch.org/whl/xpu

# (2) 동작 확인
$PY -m aodnet.device          # xpu / Intel(R) Arc(TM) B580 Graphics
$PY -m pytest tests/ -q       # 32 passed

# (3) 합성 데이터가 어떻게 생겼는지 눈으로
$PY -m aodnet.synth -o assets/synth_preview.jpg          # 절차 생성 장면
$PY -m aodnet.synth -i <내_이미지_디렉터리> -o assets/synth_mine.jpg   # 내 사진에 연기 합성

# (4) 고정 검증셋 만들기
$PY tools/make_dataset.py -o data/val  -n 120 --seed 999
$PY tools/make_dataset.py -o data/test -n 80  --seed 4242

# (5) 학습 (B580 기준 약 25분, 40에폭 × 37초)
$PY -m aodnet.train --epochs 40 --samples-per-epoch 4000 \
                    --workers 8 --val-dir data/val --out runs/base

# (6) DCP와 비교
$PY tools/compare.py --weights runs/base/best.pt --data data/test
$PY tools/bench.py   --weights runs/base/best.pt

# (7) 실제 이미지에 적용
$PY -m aodnet.infer --weights runs/base/best.pt \
                    -i ../phase1/smoke01.jpg -o out.jpg --side-by-side --strength 0.7
```

**한 줄 요약 결과**: 테스트셋 80장에서 DCP 대비 **PSNR +3.45 dB, SSIM +0.23**.
단, **검은 연기(sooty)에서는 무처리보다 나쁩니다** — 6장에서 왜 그런지, 어떻게
대응하는지 다룹니다.

---

## 1. 왜 AOD-Net인가

`phase1/image_pipeline/dehaze.py`에 이미 DCP 디헤이저가 있습니다. 잘 돕니다.
그런데 코드를 읽어보면 방어 코드가 여러 겹 붙어 있습니다.

```python
a = candidates.mean(axis=0)          # 화염 한 점에 끌려가지 않게 평균
a = np.clip(a, 1e-3, a_max)          # 과대추정 상한
# + sky_ratio, a_smoothing(EMA), t0 하한, guided filter ...
```

전부 **대기광 `A` 추정이 화염 때문에 틀리는 문제**를 막는 코드입니다.
DCP는 `t`와 `A`를 따로 추정하고 `J = (I-A)/t + A`로 나눗셈을 하기 때문에,
`A`의 작은 오차가 `1/t` 배로 증폭됩니다.

AOD-Net은 그 구조 자체를 바꿉니다.

|  | DCP (phase1) | AOD-Net |
|---|---|---|
| 추정 대상 | `t`와 `A` 따로 | `K` 하나 |
| 오차 전파 | `A` 오차 × `1/t` | 증폭 경로 없음 |
| 화염 처리 | 수동 방어 코드 | 학습 데이터로 |
| 프레임 간 상태 | `A` EMA 필요 (깜빡임 방지) | **없음** — 결정론적 |
| 도메인 적응 | 파라미터 수동 튜닝 | 데이터 추가 |
| 파라미터 | 0 | 1,761 |
| 속도 (640×480) | 8.1 ms (CPU) | 11.4 ms (Arc B580) |
| PSNR (테스트 80장) | 17.83 dB | **21.28 dB** |

**"프레임 간 상태가 없다"**가 로봇에서 실질적으로 큰 차이입니다.
DCP는 매 프레임 장면에서 `A`를 추정하므로 로봇이 움직이면 같은 불씨가 프레임마다
다른 밝기로 복원됩니다. 영상에서는 깜빡임이고, YOLO 학습 데이터로 쓰면 같은
물체의 외형 분산이 커집니다. phase1이 `a_smoothing`(EMA)을 넣어 막았지만,
그 대가로 출력이 이전 프레임에 의존하게 되어 **더 이상 결정론적이지 않습니다.**

AOD-Net의 `K`는 학습된 가중치로 결정됩니다. 같은 입력 → 항상 같은 출력.

---

## 2. 이론

전체 유도는 **[docs/01_theory.md](docs/01_theory.md)** 에 있습니다. 핵심만:

**대기 산란 모델**

```
I(x) = J(x)·t(x) + A·(1 - t(x))
```

**AOD-Net의 재구성** — 두 미지수를 하나로 접습니다.

```
J(x) = K(x)·I(x) - K(x) + b

          (1/t(x))·(I(x) - A) + (A - b)
K(x) =  ───────────────────────────────
                  I(x) - 1
```

얻는 것 세 가지:

1. **오차의 곱이 사라진다** — 추정 대상이 하나뿐
2. **학습 목표와 실제 목표가 일치한다** — `t`가 아니라 `J`를 직접 출력하므로
   `‖J_pred − J_gt‖`를 그대로 최소화 (end-to-end의 실질적 의미)
3. **`K`는 매끄러운 함수라 conv 5개면 충분** — ResNet-18의 1/6,000

**구조**

```
I ─┬─ conv1(1×1) ─┬─ conv2(3×3) ─┬─ conv3(5×5) ─┬─ conv4(7×7) ─┬─ conv5(3×3) ─ ReLU ─ K
   │              └───[cat]──────┘   └──[cat]───┘              │
   └──────────────────────[cat 1,2,3,4]────────────────────────┘
                                                                  │
                                              J = ReLU(K·I − K + b)
```

커널이 1→3→5→7로 커지는 건 multi-scale 수용영역입니다. 연기 농도 판단에는
넓은 문맥이, 윤곽 보존에는 좁은 문맥이 필요합니다.

`aodnet/model.py` 의 실제 코드는 40줄 남짓입니다(나머지는 주석). 한 번 읽어보세요.

---

## 3. 설치

PyTorch는 하드웨어에 맞는 인덱스에서 받아야 합니다.

```bash
# Intel Arc / Xe (XPU)  ← 이 프로젝트 환경 (Arc B580)
pip install torch torchvision --index-url https://download.pytorch.org/whl/xpu

# NVIDIA (CUDA 12.x)
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124

# CPU only
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu

pip install -r requirements.txt   # numpy, opencv-python, pytest
```

### Intel Arc를 쓸 때 알아둘 것

PyTorch 2.5+ 부터 `torch.xpu`가 정식 백엔드입니다. CUDA 코드와 거의 1:1로
대응되지만 두 군데가 다릅니다.

- `torch.cuda.amp` → `torch.amp.autocast('xpu', ...)`
- `pin_memory=True`의 이득이 거의 없습니다 (호스트→디바이스 복사가 병목이 아님)

Level-Zero 드라이버가 있어야 합니다. 확인:

```bash
ls /usr/lib/x86_64-linux-gnu/libze_intel_gpu.so.1   # 있어야 함
python -m aodnet.device
```

`aodnet/device.py`가 XPU → CUDA → CPU 순으로 자동 선택합니다.
벤치마크에서 `synchronize()`를 꼭 부르세요 — 안 부르면 GPU 큐에 넣은 시간만
재고 "0.2ms!" 같은 거짓 숫자가 나옵니다.

---

## 4. 데이터

**이 튜토리얼에서 제일 중요한 장입니다.** 상세: **[docs/02_domain.md](docs/02_domain.md)**

### 공개 데이터셋(RESIDE)을 쓰지 않는 이유

RESIDE는 **야외 안개**입니다. 화재 연기와 세 가지가 다릅니다.

**(1) 대기광 `A`가 다르다**

| | `A` | 색 |
|---|---|---|
| 야외 안개 | 0.7~1.0 | 흰색 |
| 흰 연기 | 0.45~0.78 | 회색 |
| **검은 연기(그을음)** | **0.12~0.38** | **어두움** |
| 화염빛 연기 | 0.35~0.70 | 주황 (R>G>B) |

`A`가 어두우면 연기가 화면을 **어둡게** 만듭니다. 흰 안개만 학습한 망은
"뿌옇게 밝다 = 연기"를 외워서, 검은 연기에서 아무것도 안 합니다.

**(2) 농도가 깊이만의 함수가 아니다**

안개는 `t = exp(-β·d)`. 연기는 발화점에서 솟는 **덩어리**라 카메라 바로 앞이
제일 진할 수 있습니다. 그래서 광학 두께를 두 성분으로 만듭니다.

```
D(x) = w_depth · depth(x)  +  w_plume · plume(x)
       (안개형, 깊이 의존)      (연기형, 깊이 독립)
```

**(3) 화염은 산란 모델의 가정 밖**

`I = J·t + A(1-t)`는 장면이 수동 반사체라고 가정합니다. 화염은 스스로 빛을 냅니다.
이걸 무시하면 망이 **불씨를 지웁니다.** 합성 단계에서 화염 영역의 `t`를 1로 되돌립니다.

> phase1 DCP도 같은 문제를 겪었고, 거기서는 `a_max` 클리핑 같은 **수동 방어
> 코드**로 막았습니다. AOD-Net에서는 **데이터로 가르칩니다.** 이게 학습 기반
> 접근의 실질적 이점입니다.

### 합성 결과 미리보기

```bash
$PY -m aodnet.synth -o assets/synth_preview.jpg
```

![합성 예시](assets/synth_preview.jpg)

왼쪽부터 **입력 I / 정답 J / 투과율 t**. 네 행이 white·gray·sooty·firelit 스타일입니다.
sooty 행에서 연기가 화면을 어둡게 만드는 것, white 행에서 불씨가 연기 너머로
살아남는 것을 확인하세요.

### ★ 내 이미지에 연기 합성하기

절차 생성 장면은 **데이터가 0장일 때의 대체재**입니다. 실제 프레임이 있으면
즉시 갈아타세요. 조건은 하나뿐입니다 — **연기가 없는 사진**이어야 합니다.
그게 곧 정답 `J`니까요.

**1단계. 눈으로 먼저 확인** (데이터셋을 만들기 전에)

```bash
$PY -m aodnet.synth -i ../frame_cut/data -o assets/synth_mine.jpg
```

파일 하나만 줘도 되고 디렉터리를 줘도 됩니다. 스타일 4종을 각각 다른 사진에
걸어 보여줍니다.

![내 이미지 합성](assets/synth_mine.jpg)

```
저장: assets/synth_mine.jpg  (좌: 입력 I / 중: 정답 J / 우: 투과율 t)
화염 마스크 최대 발화: 0.00% (frame_0000.jpg) — 정상 범위
소스: ../frame_cut/data  (총 603장, 미리보기에 4장 사용)
```

**★ 화염 마스크 발화율을 꼭 보세요.** 실제로 불이 없는데 0.5%를 넘으면
오검출입니다. 그 영역은 투과율 `t=1`인 **구멍**이 되고, 망은 "여기는 연기가
안 끼는 곳"이라고 학습합니다. (아래 함정 참고)

옵션:

```bash
--beta 2.5              # 연기 농도 고정 (생략하면 0.6~3.2 랜덤)
--styles sooty,firelit  # 특정 스타일만
--depth-dir <dir>       # 정합된 depth (파일명 stem이 같아야 함)
--width 480             # 미리보기 한 칸 너비
```

**2단계. 고정 검증셋 생성**

```bash
$PY tools/make_dataset.py -o data/mine_val -n 200 --seed 999 \
                          --clear-dir ../frame_cut/data
```

네 스타일을 균등 순환으로 뽑고 `meta.json`에 기록합니다(스타일별 진단용).

**3단계. 학습** — 학습용 연기는 매번 새로 합성되므로 사전 생성이 필요 없습니다.

```bash
$PY -m aodnet.train --clear-dir ../frame_cut/data \
                    --val-dir data/mine_val --epochs 40 --workers 8 --out runs/mine
```

`frame_cut/data` 603장으로 실제 확인한 결과입니다 (에폭 16.6초 — 절차 생성보다
빠릅니다. 이미지가 캐시되니까요).

### ⚠ 실제로 밟은 함정: 노란 차선이 화염으로 잡힙니다

`frame_cut` 프레임에는 노란 차선이 있는데, 초기 화염 마스크가 이걸 불로
오인했습니다. 화면 가로로 `t=1`인 띠가 생기고 **그 띠를 정답이라고 학습**합니다.

```
노란 차선  H=21, S=108, V=222   ← 옛 임계값(H≤25 · V>200 · S>60)을 통과
화염 몸통  H=16, S=214, V=250   ← 채도가 압도적으로 높음
화염 코어  H=22, S= 55, V=255   ← 백열, 흰색에 가까움
```

임계값만 조여서는 못 막습니다(차선 진한 부분은 S=155까지 올라감). 최종 판별은
**백열 코어의 유무 + 모양**입니다 — 불꽃은 덩어리, 차선은 폭 322×높이 16의 띠.

```python
# aodnet/synth.py::fire_mask
body = warm & (s > 150) & (v > 200)     # 몸통
core = (v >= 245) & (s < 100)           # 백열 코어
# 연결 요소 중 몸통 + 코어를 둘 다 가지고, 종횡비 ≤ 6, 면적 ≤ 화면 10% 인 것만
```

내 데이터에 주황 표지·경광등·노란 설비가 있으면 같은 일이 생길 수 있습니다.
1단계 미리보기의 `fire=` 수치로 확인하고, 안 잡히면
`SmokeConfig(preserve_fire=False)`로 꺼도 됩니다 (장면에 불이 없다면 꺼도 무손실).

### depth가 있으면 훨씬 좋습니다

RealSense depth를 같이 저장해두면 `t = exp(-β·d)`의 `d`가 진짜 거리가 됩니다.
파일명 stem만 맞추면 됩니다 (`frame_0001.jpg` ↔ `frame_0001.png`).

단, **depth 0은 0미터가 아니라 측정 실패**입니다. 그대로 쓰면 그 픽셀만 연기가
0이 되어 구멍이 뚫립니다. `data.py::_load_depth`가 '가장 먼 곳'으로 채우고
**99퍼센타일**로 정규화합니다(최댓값으로 나누면 이상치 몇 픽셀 때문에 나머지가
전부 0 근처로 눌립니다).

---

## 5. 학습

```bash
$PY -m aodnet.train --epochs 40 --samples-per-epoch 4000 \
                    --workers 8 --val-dir data/val --out runs/base
```

```
device : xpu (Intel(R) Arc(TM) B580 Graphics)
params : 1,761
[  1/40] loss 0.14571 | val PSNR  16.97 dB  SSIM 0.7218 | lr 9.99e-04 |  37.7s
[ 20/40] loss 0.04168 | val PSNR  20.18 dB  SSIM 0.7918 | lr 5.00e-04 |  36.7s
[ 40/40] loss 0.04087 | val PSNR  20.41 dB  SSIM 0.7951 | lr 0.00e+00 |  36.2s

완료. best val PSNR = 20.41 dB
```

### 데이터 공급이 병목입니다

파라미터가 1,761개뿐이라 **GPU가 아니라 CPU 합성이 병목**입니다.
샘플당 합성 비용이 32ms이므로 `--workers`를 코어 수에 맞춰 올리세요.

```bash
nproc                      # 10 → --workers 8 정도
```

`--workers 4` → 66 samples/s, `--workers 8` → 약 두 배.

### 학습이 터지는 3대 원인

출력이 `J = K·I − K + b`, 즉 **`K`가 곱해지는 구조**입니다. 그래서:

| 증상 | 원인 | 해결 |
|---|---|---|
| loss가 nan | lr이 너무 큼 | `--lr 1e-3` 이하 (CNN 감각으로 1e-2 주면 거의 확실히 터짐) |
| 결과가 전부 검정 | `K`가 0으로 죽어 ReLU에 막힘 | 위와 동일. `train.py`가 nan을 감지하면 중단합니다 |
| 초반부터 발산 | Kaiming 초기화로 바꿈 | 가우시안 std=0.02 (논문값). `test_backward_does_not_explode_on_init`이 잠금 |
| 손실 스파이크 후 회복 불가 | gradient clipping 없음 | `--clip 0.1` (논문값, 기본 켜짐) |

### 손실 함수

기본은 `mse+ssim` 입니다.

```python
loss = MSE(pred, target) + 0.15 * (1 - SSIM(pred, target))
```

MSE만 쓰면 평균적으로는 맞지만 **뿌옇게 수렴합니다**(회귀의 평균화).
SSIM 항이 구조를 붙잡아줘서 얇은 선 — 주차선, 배관 — 이 훨씬 살아납니다.
YOLO 후단이 붙을 거라면 이 차이가 중요합니다.

### 옵티마이저

논문은 SGD(lr 1e-4, momentum 0.9)를 씁니다. `--optimizer sgd`로 재현 가능하지만,
Adam이 5배쯤 빨리 수렴합니다. 기본은 Adam + CosineAnnealing.

### `--learn-b` — 어두운 장면이 많다면

`K`는 픽셀마다 `K = (J − b)/(I − 1)` 로 결정됩니다. `b = 1.0`(논문값)이면
**어두운 장면(`I ≈ 0.1`)에서 분모가 −0.9로 거의 상수**가 되어, 같은 복원량을
내려면 `K`를 훨씬 정밀하게 맞춰야 합니다. 조건수가 나쁜 상태입니다.

증상은 6장 표에서 `sooty`(검은 연기) 행만 이득이 작고 엣지가 정답보다 낮게
나오는 걸로 보입니다. 파라미터 하나를 더 두어 기준점을 데이터에 맞출 수 있습니다.

```bash
$PY -m aodnet.train --learn-b --out runs/learnb    # 파라미터 1,761 → 1,762
```

추론 쪽은 아무것도 안 바꿔도 됩니다 — `AODNetDehazer`가 체크포인트의
`state_dict`에 `b` 키가 있는지로 자동 판별합니다.

**★ 실제로 돌려봤고, 개선되지 않았습니다.**

| (테스트셋 80장) | PSNR | SSIM | 엣지 (정답 0.0406) |
|---|---|---|---|
| `b = 1.0` (논문, 기본값) | **21.28** | 0.8150 | **0.0300** |
| `--learn-b` → b=0.4451 | 21.21 | 0.8173 | 0.0188 |

`b`는 가설대로 1.0 → 0.445로 내려갔습니다(**진단은 맞았습니다**). 그런데
검증셋 PSNR만 0.25 dB 오르고, 테스트셋에서는 되레 내려갔으며, 엣지가
정답의 절반 이하로 떨어졌습니다 — MSE를 줄이는 방향이 **더 평평한 출력**이었던 겁니다.

> 교훈 두 개.
> 1. 진단이 맞아도 처방이 맞으란 법은 없습니다.
> 2. **엣지 기준선을 표에 안 넣었다면 "0.25 dB 개선"으로 보고했을 겁니다.**
>    지표 하나로 결정하지 마세요.

기본값은 논문 그대로 `b = 1.0`입니다. 옵션은 재현용으로 남겨뒀습니다.
자세히: [docs/01_theory.md §1.4½](docs/01_theory.md), [docs/02_domain.md §2.3½](docs/02_domain.md)

---

## 6. 평가

```bash
$PY tools/compare.py --weights runs/base/best.pt --data data/test
```

`phase1/image_pipeline/dehaze.py`를 그대로 import해서 **같은 입력·같은 정답**으로
붙입니다. 두 방법을 각자의 데이터로 재면 비교가 아니라 그냥 두 개의 숫자입니다.

### 실측 결과 (테스트셋 80장, 40 에폭 학습, Arc B580)

```
── 전체  (n=80)
   방법            PSNR(dB)     SSIM       엣지      대비이득       ms
   입력(연기)           16.88   0.7310   0.0366      1.00
   DCP                  17.83   0.5824   0.0817      1.77      6.6
   AOD-Net              21.28   0.8150   0.0300      0.51     10.4
   정답(기준)               —        —   0.0406      1.09
```

**AOD-Net − DCP : PSNR +3.45 dB, SSIM +0.233**

오른쪽 두 열이 PSNR·SSIM이 못 하는 진단을 합니다. 둘 다 **정답 행과의 거리**로
읽습니다 — 절대값이 크다고 좋은 게 아닙니다.

| | 엣지 (정답 0.0406) | 대비이득 (정답 1.09) | 진단 |
|---|---|---|---|
| DCP | 0.0817 = 정답의 **2배** | 1.77 | **노이즈 증폭** — 윤곽을 살린 게 아니라 지글거림을 키움 |
| AOD-Net | 0.0300 = 정답의 74% | 0.51 | **과평활** — 복원이 보수적, 대비를 되레 낮춤 |

AOD-Net이 PSNR·SSIM에서 크게 이기는 건 "덜 건드려서 덜 틀린" 것이기도 합니다.
밝기는 정답과 거의 맞는데(평균 56.3 vs 정답 60.0) 표준편차만 절반이라
**어두워진 게 아니라 진짜로 평평해진 것**입니다. 여기가 남은 개선 여지입니다.

![DCP vs AOD-Net](assets/compare.jpg)

### 스타일별로 쪼개면 결론이 달라집니다

| 스타일 | 입력 PSNR | DCP | AOD-Net | AOD 이득 |
|---|---|---|---|---|
| white (흰 연기) | 10.36 | 16.74 | **20.84** | **+10.48** |
| gray (회색 연기) | 14.85 | 19.11 | **21.36** | **+6.51** |
| firelit (화염빛) | 17.87 | 16.78 | **20.08** | +2.21 |
| **sooty (검은 연기)** | **24.44** | 18.69 | 22.86 | **−1.58** |

**`sooty`에서는 아무것도 안 하는 게 낫습니다.** 검은 연기 + 어두운 장면이라
연기가 화면을 별로 바꾸지 않고(입력 PSNR이 이미 24 dB), 바꿀 게 없는 입력에
디헤이즈를 걸면 손해만 봅니다. DCP는 같은 조건에서 18.69로 훨씬 더 나쁩니다.

**실무 결론 — 항상 켜지 말고, 연기가 있을 때만 켜세요.** phase1에 지표가 있습니다.

```python
from image_pipeline.autotune import estimate_haze_index, relative_haze

idx = estimate_haze_index(frame)
if relative_haze(idx, baseline) > threshold:     # 연기가 실제로 낀 프레임만
    frame = aod.process(frame)
```

(`relative_haze`로 기준선 보정을 해야 합니다. 연기가 없어도 절대값은 0.47쯤 나옵니다.)



출력물:
- 콘솔: 전체 / 연기 스타일별 PSNR·SSIM·엣지·처리시간
- `assets/compare.jpg` — (입력 | DCP | AOD-Net | 정답) 4열 그리드
- `assets/compare.csv` — 샘플별 원자료

### 표를 읽는 법

**스타일별로 쪼개서 보세요.** 전체 평균만 보면 "검은 연기에서만 못한다" 같은
진단이 평균에 묻힙니다. `make_dataset.py`가 네 스타일을 균등 순환으로 뽑고
`meta.json`에 기록하는 이유입니다.

**`정답(기준)` 행의 엣지 값이 기준선입니다.** 이게 없으면 "엣지 0.024"가
과복원인지 과평활인지 판단할 수 없습니다.

- 출력 엣지 < 정답 엣지 → **과평활**. 복원이 약합니다
- 출력 엣지 ≫ 정답 엣지 → **노이즈 증폭**. 대비만 올린 것

`sooty` 행에서 이득이 작다면 `--learn-b` 또는 앞단 감마를 검토하세요 (5장).

### PSNR·SSIM이 각각 무엇을 재는가

지표를 모르고 표를 읽으면 잘못된 모델을 고릅니다. 전체 설명은
**[docs/03_metrics.md](docs/03_metrics.md)** 에 있고, 핵심만 옮기면:

![PSNR vs SSIM](assets/metrics_demo.jpg)

**PSNR** = 픽셀 오차의 크기. `10·log₁₀(MAX²/MSE)`, 클수록 좋습니다.
**+3 dB = MSE 절반**이므로 `AOD-Net − DCP = +3.45 dB`는 오차를 절반 이하로
줄였다는 뜻입니다. 약점은 **픽셀을 독립적으로 봐서 구조를 모른다**는 것 —
전역 밝기 −8, 노이즈 σ=8, 대비 0.85배가 **전부 ≈30 dB로 똑같이** 나옵니다.

**SSIM** = 구조가 얼마나 남았는가. 11×11 가우시안 윈도우를 슬라이딩하며
휘도·대비·상관을 곱해 잽니다. 위 셋을 0.980 / 0.634 / 0.985로 정확히
갈라냅니다. 약점은 **블러에 관대**하다는 것 — 노이즈(0.634)보다 심한
블러 σ=3.0(0.796)을 더 후하게 줍니다.

| 왜곡 | PSNR | SSIM | 엣지 (정답 0.2158) |
|---|---|---|---|
| 밝기 −8 | 30.13 | 0.9798 | 0.2132 |
| 노이즈 σ=8 | 30.10 | **0.6343** | 0.2664 |
| 대비 0.85배 | 30.01 | 0.9845 | 0.1849 |
| 블러 σ=3.0 | **21.58** | 0.7964 | 0.0864 |

**둘이 순위를 뒤집습니다.** 그래서 이 프로젝트는 PSNR·SSIM·엣지를 항상 함께
봅니다 — `--learn-b`는 SSIM이 올랐는데 엣지가 반토막 났고, SSIM만 봤으면
채택했을 겁니다.

### PSNR을 어디까지 믿을 것인가

phase1 `CLAUDE.md`에 **"PSNR로 clipLimit 튜닝 금지"**라고 적혀 있습니다.
맞는 말이고, 여기서도 절반만 뒤집힙니다.

- ✅ **믿어도 됨**: 정답 `J`를 우리가 만든 **합성 쌍**에서의 PSNR.
  복원 목표가 명확히 정의되어 있으므로 학습 진행 판단에 유효합니다.
- ❌ **믿으면 안 됨**: 정답 없는 실제 연기 영상. "덜 건드릴수록 원본과
  비슷해 보이는" 함정이 그대로 살아 있습니다.
- 🎯 **최종 판정**: 후단 **YOLO mAP**. `compare.py`가 PSNR·SSIM과 함께
  엣지 밀도를 같이 뱉는 것도 그래서입니다 (엣지가 YOLO 성능과 상관이 높은 편).

### 속도

```bash
$PY tools/bench.py --weights runs/base/best.pt
```

Arc B580 (AOD-Net, XPU) vs phase1 DCP (CPU, scale=0.25) 실측:

| 해상도 | AOD-Net | DCP | 배속 |
|---|---|---|---|
| 640×480 | 11.4 ms (88 FPS) | 8.1 ms (123 FPS) | 0.71× |
| 848×480 | 13.7 ms (73 FPS) | 12.4 ms (81 FPS) | 0.90× |
| 1280×720 | 30.8 ms (33 FPS) | 30.1 ms (33 FPS) | 0.98× |
| 1920×1080 | 66.8 ms (15 FPS) | 67.5 ms (15 FPS) | 1.01× |

`--max-side 640` 을 켜면:

| 해상도 | AOD-Net (max_side=640) |
|---|---|
| 1280×720 | **9.8 ms (102 FPS)** |
| 1920×1080 | **7.3 ms (137 FPS)** |

**`--half`(fp16)는 이 모델에서 효과가 없습니다** (640×480에서 13.1 ms로 오히려 느림).
파라미터 1,761개짜리라 연산량이 아니라 **커널 런치 오버헤드가 병목**이기 때문입니다.
정밀도를 낮춰도 런치 횟수는 그대로입니다.



30 FPS 파이프라인에서 전처리 예산은 프레임당 33ms 전부가 아니라 그중 일부입니다
(뒤에 YOLO가 붙으니까요). 기준선 10ms를 넘으면 **재학습 없이** 먼저 시도할 것:

```bash
--max-side 640    # 축소본에서 K 추정 후 업샘플 (K는 저주파라 손실 거의 없음 — DCP의 scale과 같은 논리)
--half            # fp16 (XPU/CUDA만. CPU fp16은 오히려 몇 배 느림)
```

---

## 7. 추론과 ROS 통합

### 단독 실행

```bash
# 이미지
$PY -m aodnet.infer --weights runs/base/best.pt -i smoke.jpg -o out.jpg --side-by-side

# 디렉터리
$PY -m aodnet.infer --weights runs/base/best.pt -i frames/ -o out/

# 영상
$PY -m aodnet.infer --weights runs/base/best.pt -i clip.mp4 -o clip_out.mp4 --side-by-side
```

### phase1 파이프라인에 드롭인

`AODNetDehazer`는 `DarkChannelDehazer`와 **같은 인터페이스**입니다
(`process(bgr_uint8) -> bgr_uint8`, `reset_state()`, `timings`).
ROS 노드 코드를 한 줄도 고치지 않고 갈아끼울 수 있습니다.

```python
from image_pipeline.pipeline import Pipeline
from aodnet.infer import AODNetDehazer

pipeline = Pipeline(
    mode="full",                                   # 감마 → 디헤이즈 → CLAHE
    dehazer=AODNetDehazer("runs/base/best.pt",
                          device="xpu",
                          max_side=640),           # 실시간 예산 맞추기
)
```

이게 성립하는 이유는 4장의 설계 판단 때문입니다 — AOD-Net도 DCP처럼
**디헤이즈만** 하고 밝기는 CLAHE에 맡깁니다 (`lowlight_on_target=True`).
계약이 같아야 `mode=dehaze` vs `mode=full` 기여도 분리 실험이 해석 가능합니다.

### 실제 사진에서 — 도메인 갭이 이렇게 보입니다

`phase1/smoke01.jpg` (실제 화재 연기 사진)에 그대로 걸어본 결과입니다.

![원본 vs 복원](assets/real_smoke01.jpg)

`strength`를 0 → 1로 훑어보면:

![실제 사진 strength 스윕](assets/real_strength_sweep.jpg)

`strength=1.0`에서 연기는 확실히 걷힙니다 — 건물 벽, 잔디, 사람이 훨씬 또렷합니다.
그런데 **전체가 어두워지고 녹색으로 물듭니다.**

원인은 명확합니다. 학습 데이터는 **어두운 실내 + 어두운 대기광**이고, 이 사진은
**밝은 실외 + 흰 대기광**입니다. 망이 "이 정도 밝기면 연기가 이만큼 있다"를
학습 분포 기준으로 판단해 과복원합니다.

두 가지 대응이 있고, 둘 다 하세요.

1. **`strength`로 즉시 완화** (재학습 없음) — `0.7`이 이 사진에서는 자연스럽습니다
2. **실제 프레임으로 재학습** (근본 해결) — 4장 참고

### 현장 튜닝 손잡이: `strength`

```
K' = 1 + strength · (K - 1)      # strength=1.0이 학습된 그대로
```

```python
AODNetDehazer("best.pt", strength=0.7)
```

`K=1`이 항등원(`J = I`)이므로 `strength=0`이면 무처리, `1`이면 학습된 세기입니다.
학습 분포와 실제 입력이 어긋났을 때의 **1차 방어선**입니다.

### 첫 프레임 지연

`AODNetDehazer.__init__`이 워밍업을 합니다. 첫 호출은 커널 컴파일 때문에
수백 ms 걸리는데, ROS 노드에서 이걸 빼면 **첫 프레임에 타임아웃**이 납니다.

### K 맵 보기

```python
dehazer.process(frame)
cv2.imshow("K", dehazer.k_visualization())   # 밝을수록 강하게 복원한 영역
```

학습이 망가졌는지 판단할 때 결과 이미지보다 이 맵을 보는 게 훨씬 빠릅니다.
연기가 진한 곳에서 커지고 맑은 곳에서 1 근처가 되어야 정상입니다.

---

## 7.5 온보드 배포 (Raspberry Pi 5 + Hailo)

로봇에 얹을 때는 **torch를 깔지 않습니다.** ONNX + OpenCV DNN이면 됩니다.

### 파일 두 개만 복사

```bash
# 개발 머신에서
$PY tools/export_onnx.py --weights runs/mine2/best.pt -o deploy/aodnet_k.onnx

# 로봇으로
scp deploy/aod_lite.py deploy/aodnet_k.onnx  pi@robot:~/
```

**ONNX가 8.8 KB입니다.** 의존성은 `opencv-python`, `numpy` 뿐입니다.

### K만 신경망, 복원식은 원본 해상도에서

내보내는 그래프는 **K 맵까지만**입니다(`Conv×5 ReLU×5 Concat×3`).
복원식 `J = K·I − K + b` 세 줄은 numpy로 원본 해상도에서 계산합니다.

이유는 K가 **저주파**(연기 농도 지도)라서입니다. 축소본에서 뽑아 업샘플해도
손실이 거의 없고, 오히려 고주파 잡음이 걸러져 **품질이 올라갑니다.**
반면 출력 이미지를 업샘플하면 얇은 선·글자·윤곽이 통째로 뭉개집니다.
(phase1 DCP가 투과율 `t`만 축소 해상도에서 추정하는 것과 같은 논리)

실측 — mine2, 테스트 48장, 입력 640×480:

| K 해상도 | PSNR | SSIM | 대비이득 | CPU 4스레드 (x86) |
|---|---|---|---|---|
| 640×480 (원본) | 17.22 | 0.8288 | 0.63 | 23.8 ms |
| 320×240 | 17.37 | 0.8326 | 0.70 | 6.9 ms |
| **256×192** | **17.45** | **0.8333** | **0.72** | **6.1 ms** |

**더 빠르면서 더 좋습니다.** 기본값이 256×192인 이유입니다.

### 사용법

```python
from aod_lite import GatedDehazer, measure_baseline

# 1) 연기 없는 프레임으로 기준선 측정 (카메라/장소가 바뀌면 다시)
baseline = measure_baseline(clean_frames)      # 예: 0.3910

# 2) phase1 Pipeline에 그대로 드롭인
from image_pipeline.pipeline import Pipeline
pipe = Pipeline(mode="full", dehazer=GatedDehazer(
    "aodnet_k.onnx", baseline=baseline, threshold=0.20, threads=3))
```

`GatedDehazer`는 `DarkChannelDehazer`와 같은 인터페이스(`process` / `reset_state` /
`timings`)라 **ROS 노드 코드를 한 줄도 안 고쳐도** 됩니다.

동작 확인(실측, 640×480):

```
연기 프레임 24장 : 처리 22장 / 건너뜀 2장 → 파이프라인 전체 15.7 ms
깨끗한 프레임 24장: 처리  0장 / 건너뜀 24장 → 2.2 ms
```

### 자가 점검 / 벤치마크

```bash
python deploy/aod_lite.py aodnet_k.onnx --threads 4 -i frame.jpg -o out.jpg
```

```
입력      : 640x480
K 해상도  : 256x192
처리 시간 : 평균 6.1 ms  (164 FPS)
  이 중 신경망 4.2 ms, 복원식+변환 1.9 ms
```

### Raspberry Pi 5 에서

위 6.1 ms는 개발 머신(Intel Core Ultra) 4스레드 기준입니다.
Pi 5의 Cortex-A76은 NEON 128비트 · 2.4 GHz라 **3.5~5배 느릴 것으로 추정**합니다
(외삽이지 실측이 아닙니다).

| | 추정 |
|---|---|
| AOD (K 256×192) | 21 ~ 31 ms |
| CLAHE | 3 ~ 4 ms |
| **게이팅 적용 시 AOD 평균** | **10 ~ 15 ms** |

**Pi 5에서 반드시 실측하세요.** `aod_lite.py`의 `__main__`이 그 용도입니다.

주의할 점:

- **스레드를 다 주지 마세요.** 실측에서 10스레드(9.1 ms)가 4스레드(6.1 ms)보다
  **느렸습니다.** 워크로드가 작아 스레드 오버헤드가 더 큽니다.
  Pi 5(4코어)에서 ROS 노드·Hailo 드라이버와 나눠 쓰려면 `threads=3` 권장.
- Pi OS 기본 `opencv-python`은 NEON 최적화가 약할 수 있습니다. 느리면
  `onnxruntime`(ARM NEON 커널이 잘 돼 있음)과 비교해 보세요.

### AOD를 Hailo에 올리지 않는 이유

| | |
|---|---|
| **CLAHE가 NN 연산이 아님** | 히스토그램·보간이라 컴파일 불가 → 무조건 CPU. AOD까지 Hailo면 `Hailo→CPU→Hailo` 왕복 2회 |
| **INT8 양자화 위험** | `J = K·I − K + b`의 뺄셈이 비슷한 크기 두 텐서를 뺌 → 유효 비트 급감. 파라미터 1,761개라 이미 민감(`b` 하나 바꿨을 때 엣지 40% 변동) |
| **가속기 효율 나쁨** | 전 계층이 3~12채널이라 MAC 어레이 대부분이 놀고, YOLO가 쓸 자원만 잠식 |
| **CPU로 충분함** | 게이팅 포함 평균 10~15 ms |

**Hailo는 YOLO 전용으로 쓰는 게 자원 배분상 합리적입니다.**

그래도 올려야 한다면 `--output full`로 J까지 포함해 내보내고, **반드시 fp32와
수치를 비교하세요.** "돌아간다"가 아니라 `tools/compare.py`의 PSNR·SSIM·엣지·
대비이득으로 판정해야 합니다.

---

## 8. 함정 모음

이 프로젝트를 만들면서 실제로 밟은 것만 적었습니다. 전부 `tests/`에 잠겨 있습니다.

| # | 함정 | 증상 | 잠금 테스트 |
|---|---|---|---|
| 1 | **BGR/RGB 뒤집힘** | 학습 다 끝난 뒤 "색이 이상한데?" | `test_bgr_tensor_roundtrip` |
| 2 | **Kaiming 초기화** | 첫 스텝부터 발산 | `test_backward_does_not_explode_on_init` |
| 3 | **마지막에 `clamp(0,1)`** | 밝은 영역(화염 주변) 학습이 죽음 | `test_tensor_to_bgr_clips` |
| 4 | **inplace ReLU** | autograd가 덮어쓴 텐서 참조 → 에러 | (model.py 주석) |
| 5 | **`preserve_fire` 없이 학습** | 망이 불씨를 지움 | `test_fire_pixels_stay_visible` |
| 6 | **검은 연기 미포함** | 실화재에서 화면이 더 어두워짐 | `test_sooty_airlight_is_dark` |
| 7 | **밝기를 정답에만 적용** | 문제가 불량조건 → 18dB 근처에 갇힘, CLAHE와 이중 보정 | `test_lowlight_gain_applies_to_target_by_default` |
| 7b | **저조도를 합성 *후*에 적용** | 대기광까지 눌려 "어두운 방인데 A만 흰색"이 데이터에 섞임 | (synth.py 주석) |
| 7c | **`b=1.0`을 어두운 도메인에 그대로** | `sooty`에서만 이득 없음 + 과평활 | `--learn-b` / 앞단 감마 |
| 7d | **노란 차선을 화염으로 오검출** | 화면 가로로 `t=1`인 띠 → 그 띠를 정답으로 학습 | `test_fire_mask_rejects_yellow_lane_line` |
| 8 | **on-the-fly 검증셋** | 개선인지 노이즈인지 구분 불가 | (data.py `PairedDataset`) |
| 8b | **축소 추론에서 출력 이미지를 업샘플** | 얇은 선·글자가 뭉개짐. K만 업샘플해야 함 | (infer.py `process`) |
| 8c | **게이트를 단순 임계값으로만** | 경계 근처에서 60프레임 중 27회 on/off 깜빡임 | (aod_lite.py `SmokeGate` EMA) |
| 9 | **depth 0을 0미터로 해석** | 학습 데이터에 구멍 | `test_real_depth_is_used` |
| 10 | **PSNR에 inf** | 배치 평균 전체가 오염 | `test_psnr_identical_is_high_and_finite` |
| 11 | **워밍업 없이 벤치** | 첫 호출 커널 컴파일이 평균 오염 | (bench.py) |
| 12 | **`synchronize()` 없이 벤치** | "0.2ms!" 같은 거짓 숫자 | (device.py) |

추가로 이 환경 특유의 것:

- **numpy 2.x는 `ndarray.ptp()`를 제거**했습니다 → `np.ptp(arr)`
- **OpenCV 5.x는 color 인자에 numpy 정수를 안 받습니다** → `rng.integers()` 결과를
  반드시 `int()`로 감싸세요. 에러 메시지(`Can't parse 'rec'`)가 원인을 전혀 안 알려줍니다.

### 스모크 테스트를 먼저 돌리세요

```bash
$PY -m pytest tests/test_aodnet.py::TestIntegration -q
```

한 배치를 40스텝 오버피팅해서 손실이 30% 이상 줄어드는지 봅니다.
안 줄면 배선(색공간·정규화·옵티마이저)이 틀린 것이고, 전체 학습을 돌리기 전에
여기서 걸러집니다.

---

## 9. 다음 단계

이 튜토리얼은 **동작하는 최소 구성**입니다. 여기서 더 나가려면:

**1) 실제 프레임으로 재학습 (효과 가장 큼)**
절차적 도형 장면은 데이터 0장일 때의 대체재입니다. 7장의 실제 사진 결과가
보여주듯 도메인 갭이 그대로 색 왜곡으로 나타납니다. 로봇에서 연기 없는 주행
프레임을 몇 백 장만 모아도 `--clear-dir`로 바로 개선됩니다. depth까지 있으면 더 좋습니다.

**2) 연기 지표로 게이팅 (당장 할 수 있음)**
6장에서 봤듯 `sooty`에서는 무처리가 낫습니다. `relative_haze()` 임계값으로
켜고 끄면 그 손실을 그냥 없앨 수 있습니다. 코드 세 줄입니다.

**3) YOLO mAP로 최종 판정**
PSNR은 학습 진행 지표일 뿐입니다. phase1 `tools/make_dataset.py`가 이미
조건별 YOLO 학습 데이터셋을 만들 수 있으니, `passthrough / clahe / dcp+clahe /
aod+clahe` 4조건의 mAP를 재세요. 그게 발표에 쓸 숫자입니다.

**4) 시간 일관성**
AOD-Net은 프레임 독립이라 DCP보다 이미 안정적이지만, 완전하지는 않습니다.
연속 프레임 간 K의 급변을 억제하려면 추론 시 K에 EMA를 걸 수 있습니다
(DCP의 `a_smoothing`과 같은 발상이지만, K가 훨씬 안정적이라 계수를 낮게 잡아도 됩니다).

**5) 양자화 / IPEX**
1,761개짜리 모델이라 양자화 이득은 크지 않지만, `torch.compile` 또는
`intel-extension-for-pytorch`로 커널 런치 오버헤드를 줄일 수 있습니다.
이 모델은 연산량이 아니라 **런치 오버헤드가 병목**입니다.

**6) 더 큰 모델과 비교**
FFA-Net, DehazeFormer 등이 PSNR은 훨씬 높습니다. 다만 파라미터가 4~5자릿수
많아서 온보드 실시간이 안 나옵니다. "왜 1,761개짜리를 골랐는가"를 발표에서
정당화하려면 이 비교 표가 필요합니다.

---

## 파일 지도

```
aod_net/
├── README.md                    이 문서
├── docs/
│   ├── 01_theory.md             대기산란모델 → K 통합 유도
│   ├── 02_domain.md             화재 연기 ≠ 안개, 데이터 설계
│   └── 03_metrics.md            PSNR·SSIM이 재는 것과 못 재는 것
├── aodnet/
│   ├── model.py                 AOD-Net 정의 — 먼저 읽으세요
│   ├── synth.py                 ★ 연기 합성. 성능이 여기서 갈립니다
│   ├── data.py                  Dataset (on-the-fly / 고정 쌍)
│   ├── metrics.py               PSNR·SSIM·대비·엣지
│   ├── device.py                XPU/CUDA/CPU 자동 선택
│   ├── train.py                 학습 루프
│   └── infer.py                 추론 + AODNetDehazer (드롭인 교체용)
├── tools/
│   ├── make_dataset.py          고정 검증셋 생성
│   ├── compare.py               DCP vs AOD-Net
│   ├── tune_gate.py             게이팅 임계값 튜닝
│   ├── export_onnx.py           배포용 ONNX 내보내기 + 수치 검증
│   └── bench.py                 해상도별 속도
├── deploy/                      ★ 로봇에 복사할 것 (torch 불필요)
│   ├── aod_lite.py              OpenCV DNN 추론 + SmokeGate + GatedDehazer
│   └── aodnet_k.onnx            K 전용 모델 (8.8 KB)
└── tests/test_aodnet.py         32개. 전부 실제 함정의 잠금장치
```

읽는 순서 추천: `model.py` → `docs/01_theory.md` → `synth.py` → `docs/02_domain.md` → `train.py` → `docs/03_metrics.md`

---

## 참고문헌

- Li et al., **AOD-Net: All-in-One Dehazing Network**, ICCV 2017
- He et al., **Single Image Haze Removal Using Dark Channel Prior**, CVPR 2009 (phase1 DCP의 원논문)
- Li et al., **RESIDE: A Benchmark for Single Image Dehazing**, 2018 (표준 데이터셋 — 이 프로젝트는 도메인이 달라 사용하지 않음)
