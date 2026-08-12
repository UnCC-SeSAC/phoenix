#!/usr/bin/env python3
"""
인터랙티브 미리보기 — 트랙바를 돌리며 파라미터 감을 잡는 도구.

로드맵의 "주피터에서 파라미터 감을 잡기"를 대신합니다. 슬라이더를 움직이면
결과가 즉시 갱신되므로, clipLimit·omega·t0가 실제로 무엇을 바꾸는지
숫자가 아니라 눈으로 익힐 수 있습니다.

  python3 tools/live_preview.py testdata/hazy.png
  python3 tools/live_preview.py smoke.mp4
  python3 tools/live_preview.py 0                 # 웹캠

키:
  1/2/3/4  모드 전환 (passthrough / clahe / dehaze / full)
  s        현재 화면 저장
  p        현재 파라미터를 ROS YAML 형식으로 출력 (config에 그대로 복붙)
  space    동영상 일시정지
  q, ESC   종료

GUI가 없는 환경(SSH·컨테이너)이면 자동으로 --headless 모드를 안내합니다.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from image_pipeline.dehaze import ClaheEnhancer, DarkChannelDehazer  # noqa: E402
from image_pipeline.pipeline import MODES, Pipeline  # noqa: E402

WIN = "image_pipeline  |  left: original   right: processed"

# (트랙바 이름, 최대값, 기본 정수값, 정수 -> 실제값 변환)
TRACKBARS = [
    ("mode 1-4",        3,   3,   lambda v: v),
    ("clipLimit x10",   80,  20,  lambda v: max(v, 1) / 10.0),
    ("tileGrid",        16,  8,   lambda v: max(v, 2)),
    ("omega x100",      100, 95,  lambda v: v / 100.0),
    ("t0 x100",         50,  10,  lambda v: max(v, 1) / 100.0),
    ("patch",           31,  15,  lambda v: max(3, v | 1)),
    ("scale x100",      100, 25,  lambda v: max(v, 5) / 100.0),
    ("gamma x100",      300, 100, lambda v: max(v, 10) / 100.0),
    ("guided 0/1",      1,   1,   lambda v: bool(v)),
    ("lowlight 0/1",    1,   0,   lambda v: bool(v)),
]


def has_gui() -> bool:
    """GUI 사용 가능 여부.

    주의: `cv2.namedWindow`를 try/except로 감싸 확인하면 안 됩니다.
    Qt 백엔드는 디스플레이가 없을 때 예외가 아니라 **abort()** 를 호출해
    파이썬 프로세스가 통째로 죽습니다(except가 안 잡힘). 그래서 창을 열기 전에
    환경변수로 먼저 판단합니다.
    """
    if os.environ.get("QT_QPA_PLATFORM") == "offscreen":
        return False
    if sys.platform.startswith(("win", "darwin")):
        return True
    # Linux: X11 또는 Wayland 세션이 있어야 함
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        return False
    if not hasattr(cv2, "namedWindow"):
        return False   # opencv-python-headless 휠

    # DISPLAY가 설정돼 있어도 X 서버가 죽어 있으면(SSH 포워딩 끊김 등) 여전히
    # abort 합니다. 그래서 **버리는 자식 프로세스**에서 창을 열어보고,
    # 자식이 죽으면 우리는 멀쩡히 headless로 넘어갑니다.
    try:
        r = subprocess.run(
            [sys.executable, "-c",
             "import cv2; cv2.namedWindow('p'); cv2.destroyAllWindows()"],
            capture_output=True, timeout=15,
        )
        return r.returncode == 0
    except Exception:  # noqa: BLE001
        return False


class Source:
    """이미지 / 동영상 / 웹캠을 같은 인터페이스로 감쌉니다."""

    def __init__(self, path: str, width: int):
        self.width = width
        self.is_still = False
        self.cap = None
        self.still = None

        if path.isdigit():
            self.cap = cv2.VideoCapture(int(path))
        elif os.path.isfile(path) and path.lower().endswith(
                (".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff")):
            self.still = cv2.imread(path)
            if self.still is None:
                raise SystemExit(f"이미지를 읽을 수 없습니다: {path}")
            self.is_still = True
        else:
            self.cap = cv2.VideoCapture(path)

        if self.cap is not None and not self.cap.isOpened():
            raise SystemExit(f"소스를 열 수 없습니다: {path}")

    def _fit(self, img):
        if self.width and img.shape[1] > self.width:
            h = int(round(img.shape[0] * self.width / img.shape[1]))
            img = cv2.resize(img, (self.width, h), interpolation=cv2.INTER_AREA)
        return img

    def read(self):
        if self.is_still:
            return self._fit(self.still.copy())
        ok, img = self.cap.read()
        if not ok:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)   # 루프
            ok, img = self.cap.read()
            if not ok:
                return None
        return self._fit(img)


def read_params() -> dict:
    vals = {}
    for name, _, _, conv in TRACKBARS:
        vals[name] = conv(cv2.getTrackbarPos(name, WIN))
    return vals


def apply_params(pipe: Pipeline, v: dict) -> None:
    pipe.set_mode(MODES[int(v["mode 1-4"])])
    pipe.gamma = v["gamma x100"]
    pipe.lowlight = v["lowlight 0/1"]

    tile = int(v["tileGrid"])
    if (pipe.clahe.clip_limit != v["clipLimit x10"]
            or pipe.clahe.tile_grid != (tile, tile)):
        pipe.clahe.update(v["clipLimit x10"], (tile, tile))

    d = pipe.dehazer
    d.omega = v["omega x100"]
    d.t0 = v["t0 x100"]
    d.patch = int(v["patch"])
    d.scale = v["scale x100"]
    d.use_guided = v["guided 0/1"]


def overlay(img, lines):
    out = img.copy()
    pad = 6 + 18 * len(lines)
    cv2.rectangle(out, (0, 0), (out.shape[1], pad), (0, 0, 0), -1)
    for i, text in enumerate(lines):
        cv2.putText(out, text, (8, 18 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def print_yaml(v: dict) -> None:
    tile = int(v["tileGrid"])
    print("\n# --- config/preprocess.yaml 에 복붙 ---")
    print(f"    mode: {MODES[int(v['mode 1-4'])]}")
    print(f"    gamma: {v['gamma x100']:.2f}")
    print(f"    lowlight_dehaze: {str(v['lowlight 0/1']).lower()}")
    print(f"    clahe_clip_limit: {v['clipLimit x10']:.1f}")
    print(f"    clahe_tile_grid: [{tile}, {tile}]")
    print(f"    dehaze_omega: {v['omega x100']:.2f}")
    print(f"    dehaze_t0: {v['t0 x100']:.2f}")
    print(f"    dehaze_patch: {int(v['patch'])}")
    print(f"    dehaze_scale: {v['scale x100']:.2f}")
    print(f"    dehaze_use_guided: {str(v['guided 0/1']).lower()}")
    print("# -------------------------------------\n")


def run_gui(args):
    src = Source(args.source, args.width)
    pipe = Pipeline(clahe=ClaheEnhancer(), dehazer=DarkChannelDehazer())

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    for name, maxv, default, _ in TRACKBARS:
        cv2.createTrackbar(name, WIN, default, maxv, lambda _v: None)

    paused = False
    frame = None
    ema_ms = None
    os.makedirs(args.out, exist_ok=True)
    print(__doc__.split("키:")[1].strip() if "키:" in __doc__ else "")

    while True:
        if not paused or frame is None:
            frame = src.read()
            if frame is None:
                break

        v = read_params()
        apply_params(pipe, v)

        result = pipe.process(frame)
        ms = pipe.timings["total"]
        ema_ms = ms if ema_ms is None else 0.9 * ema_ms + 0.1 * ms

        def stat(im):
            return cv2.cvtColor(im, cv2.COLOR_BGR2GRAY).std()

        left = overlay(frame, [f"original   contrast {stat(frame):5.1f}"])
        right = overlay(result, [
            f"{pipe.mode}   contrast {stat(result):5.1f}",
            f"{ema_ms:5.1f} ms  ({1000 / max(ema_ms, 1e-6):5.1f} fps)"
            + ("   << 20fps 미달" if ema_ms > 50 else ""),
        ])
        cv2.imshow(WIN, np.hstack([left, right]))

        key = cv2.waitKey(1 if not src.is_still else 30) & 0xFF
        if key in (ord("q"), 27):
            break
        elif key == ord(" "):
            paused = not paused
        elif key in (ord("1"), ord("2"), ord("3"), ord("4")):
            cv2.setTrackbarPos("mode 1-4", WIN, key - ord("1"))
        elif key == ord("p"):
            print_yaml(v)
        elif key == ord("s"):
            path = os.path.join(args.out, f"preview_{int(time.time())}.png")
            cv2.imwrite(path, np.hstack([left, right]))
            print(f"저장: {path}")

    cv2.destroyAllWindows()


def run_headless(args):
    """GUI가 없을 때: 파라미터 조합을 격자 이미지로 뽑아 파일로 확인."""
    src = Source(args.source, args.width)
    frame = src.read()
    if frame is None:
        raise SystemExit("프레임을 읽지 못했습니다")

    os.makedirs(args.out, exist_ok=True)
    clahe, dehazer = ClaheEnhancer(), DarkChannelDehazer()

    tiles = []
    for mode in MODES:
        pipe = Pipeline(mode=mode, gamma=args.gamma, clahe=clahe, dehazer=dehazer)
        res = pipe.process(frame)
        contrast = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY).std()
        tiles.append(overlay(
            res, [f"{mode}  {pipe.timings['total']:.1f}ms  contrast {contrast:.1f}"]))

    rows = [np.hstack(tiles[0:2]), np.hstack(tiles[2:4])]
    path = os.path.join(args.out, "headless_modes.png")
    cv2.imwrite(path, np.vstack(rows))
    print(f"GUI가 없어 격자 이미지로 저장했습니다: {path}")
    print("세밀한 튜닝은 tools/tune_offline.py --sweep 을 쓰세요.")


def main():
    ap = argparse.ArgumentParser(description="인터랙티브 파라미터 미리보기")
    ap.add_argument("source", help="이미지/동영상 경로 또는 웹캠 인덱스(0)")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--gamma", type=float, default=1.0, help="headless 모드에서만 사용")
    ap.add_argument("-o", "--out", default="out")
    ap.add_argument("--headless", action="store_true", help="GUI 없이 격자 이미지 저장")
    args = ap.parse_args()

    if args.headless or not has_gui():
        if not args.headless:
            print("GUI를 열 수 없는 환경입니다. headless 모드로 전환합니다.")
        run_headless(args)
    else:
        run_gui(args)


if __name__ == "__main__":
    main()
