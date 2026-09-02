#!/usr/bin/env python3
"""
화재 검출 유무 판정 — **ROS도 numpy도 import하지 않습니다.**

`check_fire_status` 서비스의 알맹이입니다. 노드는 구독·서비스 배선만 하고,
"꺼졌는가"의 판단은 전부 여기서 합니다 (HANDOVER 4-1). rclpy 없이 돌아야
`tests/test_fire_status.py`가 판정 경계를 잠글 수 있습니다.

    hist = DetectionHistory(window_sec=10.0, fire_class="fire")
    hist.add(t, [("fire", 0.87), ("person", 0.41)])   # 프레임 1장
    hist.add(t + 0.07, [])                            # 검출 0개 = "불 없음"
    v = hist.verdict(now, observation_sec=3.0, ratio_threshold=0.3)
    v.is_extinguished, v.confidence

--- 왜 이 모듈이 따로 있나 ---

이 판정은 **조용히 틀리는** 종류입니다. 진압 성공/실패를 가르는데 예외가
안 나므로, 틀려도 로그에 "꺼짐"이라고만 찍히고 넘어갑니다. 그래서 노드
안에 두지 않고 여기서 처리하고 테스트로 잠급니다.

특히 아래 4개는 실제로 밟았거나 밟을 뻔한 함정입니다:

  1. **"데이터 없음"을 "불 없음"으로 해석**
     파이프라인이 죽어도 검출은 0건입니다. 둘을 같게 보면 카메라가 빠져도
     "진압 성공"이 됩니다. `Verdict.reason`으로 구분하고, 데이터가 없으면
     **판정을 거부**합니다 (is_extinguished=False).

  2. **관찰 구간이 히스토리 창보다 길 때 조용한 절삭**
     3초를 요청했는데 창이 1초면 1초만 보고 답합니다. 에러도 경고도 없이
     의미만 달라집니다. `Verdict.truncated`로 노출해 노드가 경고합니다.

  3. **confidence가 판정과 반대로 움직임**
     "평균 불꽃 점수"를 confidence로 내면, 꺼졌다고 판정할 때 값이 0에
     수렴합니다. 로그에 "꺼짐 (확신도 0.00)"으로 찍혀 판정을 못 믿게 됩니다.
     여기서는 **판정 자체의 확신도**를 냅니다 (아래 `Verdict.confidence`).

  4. **클래스 이름 대소문자 불일치**
     라벨이 'Fire'인데 'fire'로 찾으면 **불이 영원히 안 보입니다** — 검출은
     계속 오는데 화재만 0건이라 "꺼짐"이 됩니다. `near_miss_labels()`가
     이 경우를 잡아내 노드가 시작 직후 경고할 수 있게 합니다.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence

# 판정 사유. 서비스 응답에는 안 실리지만(계약이 bool+float뿐) 로그로 나갑니다.
REASON_OK = "ok"                 # 표본이 충분해 정상 판정
REASON_NO_DATA = "no_data"       # 관찰 구간에 표본 0개 — 판정 불가
REASON_STALE = "stale"           # 표본은 있지만 전부 낡음 — 파이프라인 정지 의심


@dataclass(frozen=True)
class Verdict:
    """판정 결과.

    is_extinguished : 꺼졌다고 판단했는가
    confidence      : **판정 자체의 확신도** [0, 1].
                      꺼짐이면 `1 - fire_ratio`(불이 안 보인 비율),
                      안꺼짐이면 `fire_ratio`(불이 보인 비율).
                      ★ 평균 불꽃 점수가 아닙니다 — 그건 판정과 반대로
                        움직여서 로그가 자기모순처럼 보입니다 (모듈 문서 3번).
    fire_ratio      : 관찰 구간에서 화재가 보인 프레임 비율 [0, 1]
    samples         : 판정에 쓴 프레임 수
    mean_score      : 화재로 잡힌 프레임들의 평균 점수 (진단용, 0건이면 0.0)
    truncated       : 요청 구간이 히스토리 창보다 길어 잘렸는가
    reason          : REASON_* 중 하나
    """

    is_extinguished: bool
    confidence: float
    fire_ratio: float
    samples: int
    mean_score: float
    truncated: bool
    reason: str


class DetectionHistory:
    """프레임별 "화재가 보였는가"를 시간과 함께 쌓아두는 링버퍼.

    window_sec  : 이보다 오래된 기록은 버립니다. **서비스가 요청할 수 있는
                  최대 observation_seconds보다 넉넉히** 잡으세요 (함정 2번).
    fire_class  : 화재로 볼 class_id 문자열. 학습 라벨과 **정확히** 같아야 합니다
    min_score   : 이 점수 미만 검출은 없는 것으로 봅니다. 오검출 1건이
                  "안꺼짐"을 붙드는 걸 막습니다. 0.0이면 끔
    """

    def __init__(self, window_sec: float = 10.0, fire_class: str = "fire",
                 min_score: float = 0.0):
        if window_sec <= 0.0:
            raise ValueError(f"window_sec는 양수여야 합니다: {window_sec!r}")
        self.window_sec = float(window_sec)
        self.fire_class = str(fire_class)
        self.min_score = float(min_score)
        # (t, fire_seen, best_fire_score) — best_fire_score는 fire_seen일 때만 의미 있음
        self._items: deque[tuple[float, bool, float]] = deque()
        #: 지금까지 본 class_id 전부. 대소문자 함정 진단용 (함정 4번).
        self._labels_seen: set[str] = set()

    # ------------------------------------------------------------------ 입력

    def add(self, t: float, labels: Iterable[tuple[str, float]]) -> None:
        """프레임 1장을 기록합니다.

        `labels`는 그 프레임의 `(class_id, score)` 목록입니다. **빈 목록도
        반드시 넣으세요** — 그게 "불이 안 보였다"는 유일한 증거입니다.
        안 넣으면 침묵이 되고, 침묵은 "노드가 죽었다"와 구분되지 않습니다.
        """
        fire_seen = False
        best = 0.0
        for class_id, score in labels:
            self._labels_seen.add(str(class_id))
            if str(class_id) != self.fire_class:
                continue
            if float(score) < self.min_score:
                continue
            fire_seen = True
            best = max(best, float(score))

        self._items.append((float(t), fire_seen, best))
        self.trim(t)

    def trim(self, now: float) -> None:
        """`window_sec`를 넘긴 기록을 버립니다."""
        cutoff = float(now) - self.window_sec
        while self._items and self._items[0][0] < cutoff:
            self._items.popleft()

    # ------------------------------------------------------------------ 진단

    def __len__(self) -> int:
        return len(self._items)

    @property
    def last_time(self) -> Optional[float]:
        """마지막으로 기록한 시각. 없으면 None."""
        return self._items[-1][0] if self._items else None

    def near_miss_labels(self) -> list[str]:
        """`fire_class`와 대소문자만 다른 라벨들 (함정 4번).

        비어 있지 않으면 **fire_class 설정이 틀렸을 가능성이 큽니다.**
        검출은 계속 오는데 화재만 0건이면 조용히 "꺼짐"이 되므로,
        노드가 이걸 보고 경고를 띄웁니다.
        """
        target = self.fire_class.casefold()
        return sorted(lbl for lbl in self._labels_seen
                      if lbl != self.fire_class and lbl.casefold() == target)

    # ------------------------------------------------------------------ 판정

    def verdict(self, now: float, observation_sec: float,
                ratio_threshold: float = 0.3,
                stale_after_sec: float = 0.0) -> Verdict:
        """관찰 구간을 집계해 꺼졌는지 판정합니다.

        observation_sec : 되돌아볼 시간. 0 이하면 히스토리 창 전체를 씁니다
        ratio_threshold : 화재 프레임 비율이 이 값 **미만**이면 꺼짐
        stale_after_sec : 마지막 기록이 이보다 낡으면 판정 거부 (0이면 끔).
                          표본이 남아 있어도 새 프레임이 안 들어오는 상태는
                          "불이 꺼진 것"이 아니라 "파이프라인이 멈춘 것"입니다
        """
        now = float(now)
        self.trim(now)

        # ① 아무 기록도 없음 -> 판정 불가. 절대 "꺼짐"이라고 하지 않습니다.
        if not self._items:
            return Verdict(False, 0.0, 0.0, 0, 0.0, False, REASON_NO_DATA)

        # ② 기록은 있지만 전부 낡음 -> 파이프라인 정지 의심. 역시 판정 불가.
        last = self._items[-1][0]
        if stale_after_sec > 0.0 and (now - last) > stale_after_sec:
            return Verdict(False, 0.0, 0.0, 0, 0.0, False, REASON_STALE)

        # ③ 관찰 구간을 잘라냅니다. 창보다 길게 요청하면 조용히 자르지 않고
        #    truncated로 알립니다 (함정 2번).
        truncated = observation_sec > self.window_sec
        if observation_sec <= 0.0:
            recent = list(self._items)
        else:
            start = now - float(observation_sec)
            recent = [it for it in self._items if it[0] >= start]

        if not recent:
            return Verdict(False, 0.0, 0.0, 0, 0.0, truncated, REASON_NO_DATA)

        fire_frames = [it for it in recent if it[1]]
        ratio = len(fire_frames) / len(recent)
        mean_score = (sum(it[2] for it in fire_frames) / len(fire_frames)
                      if fire_frames else 0.0)

        is_ext = ratio < float(ratio_threshold)
        # 판정 방향과 같이 움직이는 확신도 (함정 3번).
        confidence = (1.0 - ratio) if is_ext else ratio

        return Verdict(is_ext, confidence, ratio, len(recent), mean_score,
                       truncated, REASON_OK)


def labels_from_detections(detections: Iterable, hypothesis_fn) -> list[tuple[str, float]]:
    """`Detection2DArray.detections` -> `[(class_id, score), ...]`.

    `hypothesis_fn`은 `detection_msgs.hypothesis`를 넘기세요 — vision_msgs
    3.x/4.x의 필드 모양 차이를 그쪽이 흡수합니다. 여기서 직접 필드를 읽으면
    이 모듈이 ROS에 묶입니다.
    """
    out: list[tuple[str, float]] = []
    for det in detections:
        for res in getattr(det, "results", ()):
            out.append(hypothesis_fn(res))
    return out


def describe(verdict: Verdict, observation_sec: float) -> str:
    """로그 한 줄. 판정 근거가 안 보이면 현장에서 원인을 못 찾습니다."""
    if verdict.reason == REASON_NO_DATA:
        return (f"판정 불가 — 최근 {observation_sec:.1f}초에 표본 0건. "
                "YOLO 파이프라인이 멈췄는지 확인하세요 (안꺼짐으로 응답)")
    if verdict.reason == REASON_STALE:
        return ("판정 불가 — 표본이 전부 낡았습니다. 파이프라인 정지 의심 "
                "(안꺼짐으로 응답)")
    tail = " ★관찰구간이 히스토리 창보다 길어 잘림" if verdict.truncated else ""
    return (f"{verdict.samples}건 관찰, 화재 비율 {verdict.fire_ratio:.2f} "
            f"(평균 점수 {verdict.mean_score:.2f}) -> "
            f"{'꺼짐' if verdict.is_extinguished else '안꺼짐'} "
            f"(확신도 {verdict.confidence:.2f}){tail}")
