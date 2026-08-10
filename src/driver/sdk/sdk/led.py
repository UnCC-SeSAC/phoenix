#!/usr/bin/python3
# coding=utf8
# 빵판(breadboard) 다색 LED 드라이버. 온보드 RGB LED와 '동일하게' 상태를 표시하기 위해 사용.
#   - GPIO 4핀(active-high): GREEN=24 / RED=25 / YELLOW_LEFT=23 / YELLOW_RIGHT=18
#   - 점멸은 데몬 스레드에서 돌아 논블로킹(메인 루프/YOLO를 막지 않음).
#   - set_mode에 '변화 가드'가 있어 매 프레임 mode_*()를 불러도 이미 같은 모드면 즉시 리턴(GPIO 안 때림).
import RPi.GPIO as GPIO
import threading

PIN_GREEN = 24
PIN_RED = 25
PIN_YELLOW_LEFT = 23
PIN_YELLOW_RIGHT = 18
ALL_PINS = [PIN_GREEN, PIN_RED, PIN_YELLOW_LEFT, PIN_YELLOW_RIGHT]

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
for pin in ALL_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 0)

_blink_stop_event = None
_blink_thread = None
current_mode = None
mode_lock = threading.Lock()


def stop_blink():
    global _blink_stop_event, _blink_thread
    if _blink_stop_event:
        _blink_stop_event.set()   # 스레드에게 종료 신호만 보냄
    # join 없음 — daemon 스레드가 알아서 종료됨
    _blink_stop_event = None
    _blink_thread = None


def start_blink(pins, interval=0.3):
    global _blink_stop_event, _blink_thread
    stop_blink()
    ev = threading.Event()
    _blink_stop_event = ev

    def worker():
        state = 1
        while not ev.is_set():
            for p in pins:
                GPIO.output(p, state)
            state = 1 - state
            ev.wait(interval)   # set되면 즉시 깨어남

    _blink_thread = threading.Thread(target=worker, daemon=True)
    _blink_thread.start()


def set_mode(name, fn):
    global current_mode
    with mode_lock:
        if current_mode == name:
            return
        current_mode = name
        fn()


def mode_straight():
    # 주행 → 초록 상시 점등 (온보드: 녹색)
    def _do():
        stop_blink()
        for p in ALL_PINS:
            GPIO.output(p, 0)
        GPIO.output(PIN_GREEN, 1)
    set_mode("straight", _do)


def mode_turn_right():
    # 우회전 표지판 인식/회전 → 우측 노랑 점멸 (온보드: 우측 노란 점멸)
    def _do():
        stop_blink()
        for p in ALL_PINS:
            GPIO.output(p, 0)
        start_blink([PIN_YELLOW_RIGHT])
    set_mode("turn_right", _do)


def mode_go():
    # 직진 화살표 표지판 인식 → 양쪽 노랑 점멸 (온보드: 양쪽 노란 점멸)
    def _do():
        stop_blink()
        for p in ALL_PINS:
            GPIO.output(p, 0)
        start_blink([PIN_YELLOW_LEFT, PIN_YELLOW_RIGHT])
    set_mode("go", _do)


def mode_drive_right():
    # 움직이며 우회전 중 → 초록 상시 ON + 우측 노랑만 점멸.
    #   (초록은 blink 목록에 없어 worker가 안 건드림 → 계속 켜져있고, 우측 노랑만 깜빡인다.)
    def _do():
        stop_blink()
        for p in ALL_PINS:
            GPIO.output(p, 0)
        GPIO.output(PIN_GREEN, 1)           # 초록 상시 ON(움직임 표시)
        start_blink([PIN_YELLOW_RIGHT])     # 우측 노랑만 점멸(우회전 표시)
    set_mode("drive_right", _do)


def mode_stop():
    # 정지/대기 → 빨강 상시 점등 (온보드: 빨강)
    def _do():
        stop_blink()
        for p in ALL_PINS:
            GPIO.output(p, 0)
        GPIO.output(PIN_RED, 1)
    set_mode("stop", _do)


def mode_park_done():
    # 주차 완료 → 전체 점멸 (온보드: 양쪽 노란 점멸)
    def _do():
        stop_blink()
        for p in ALL_PINS:
            GPIO.output(p, 0)
        start_blink(ALL_PINS, interval=0.3)
    set_mode("park_done", _do)


def all_off():
    global current_mode
    with mode_lock:
        current_mode = None
    stop_blink()
    for p in ALL_PINS:
        GPIO.output(p, 0)


def cleanup():
    all_off()
    GPIO.cleanup()


# ===== 구버전 단일핀 API 호환(혹시 다른 코드가 on/off/set을 부를 때 대비) =====
def on():
    mode_straight()


def off():
    all_off()


def set(new_state):
    if new_state:
        all_off()
    else:
        mode_straight()
