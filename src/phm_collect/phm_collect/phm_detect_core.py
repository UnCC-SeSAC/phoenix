#!/usr/bin/env python3
"""잔차 검출의 공통 코어. 오프라인 스윕(`08`)과 로봇 실시간 노드가 함께 씁니다.

왜 이 파일이 따로 있나
----------------------
`phm_wire.py` 와 같은 이유입니다. 검출 수식을 두 군데에 손으로 적어두면 반드시
어긋납니다. 특히 이 프로젝트는 임계를 실측으로 몇 번이나 바꿔 왔습니다
(16.2 -> 16.10 -> 16.11 -> 18.3). 오프라인에서 정한 임계가 로봇 위에서 다른 뜻이
되는 순간, 화면의 경보는 아무 근거가 없는 값이 됩니다.

그래서 **경보 판정 상태기계를 한 벌만 둡니다.** `08` 의 스윕도 로봇 노드도 같은
`TimeWindowAlarm` 을 돌립니다. 스윕 결과표의 숫자가 곧 로봇 위에서 나올 숫자입니다.

오프라인과 온라인의 유일한 차이: 시간 방향
------------------------------------------
오프라인은 지령 시각 t 를 기준으로 **미래**의 실측을 봅니다.

    e(t) = | meas(t + lag) - gain * cmd(t) |

로봇 위에서는 미래를 못 봅니다. 그래서 실측이 들어온 시각을 기준으로 **과거**의
지령을 봅니다. 같은 쌍을 다른 쪽에서 부르는 것뿐이라 값은 같습니다.

    e(t) = | meas(t) - gain * cmd(t - lag) |

바뀌는 것은 잔차가 붙는 시각뿐입니다(오프라인 t, 온라인 t+lag). 경보 시점이
lag 만큼 늦게 찍히므로, 실시간 검출 지연은 스윕이 보여주는 '첫 검출' 에
**lag 을 더한 값**입니다 (yaw 0.10초, fwd 0.35초).
"""
from __future__ import annotations

import bisect
import collections

# ---------------------------------------------------------------------------
# 축 정의
# ---------------------------------------------------------------------------
CMD_TOPICS = ("/cmd_vel", "/controller/cmd_vel")

# /cmd_vel 은 odom_publisher_node.py:181-192 에서 아래 값으로 잘린 뒤 모터로 갑니다.
# 토픽에 기록된 값이 아니라 '잘린 값' 이 실제 지령입니다.
# /controller/cmd_vel 은 cmd_vel_callback 직결이라 클램프가 없습니다.
CMD_CLAMP = {"/cmd_vel": (0.2, 0.5)}   # 토픽 -> (|vx| 한계 m/s, |wz| 한계 rad/s)

AXES = {
    "yaw": {
        "cmd_index": 2,                                  # (t, vx, wz) 의 wz
        "topic": "/ros_robot_controller/imu_raw",
        "field": ("angular_velocity", "z"),
        "sign": 1.0,
        "unit": "rad/s",
        "label": "요레이트",
        "meas": "자이로",
        "thr": [0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50],
    },
    "fwd": {
        "cmd_index": 1,                                  # (t, vx, wz) 의 vx
        "topic": "/odom_rf2o",
        "field": ("twist", "twist", "linear", "x"),
        # ★ 부호 반전: 라이다가 180도 돌아 장착돼 있는데(lidar.urdf.xacro:8 이
        # rpy="0 0 pi") rf2o 가 자기 출력을 레이저 프레임 그대로 발행합니다.
        # 그래서 전진할 때 vx 가 음수로 나옵니다. yaw 는 정상이라 x,y 만 뒤집습니다.
        "sign": -1.0,
        "unit": "m/s",
        "label": "전진속도",
        "meas": "rf2o",
        "thr": [0.02, 0.03, 0.04, 0.05, 0.06, 0.08, 0.10],
    },
}

# ---------------------------------------------------------------------------
# 확정 규칙 (18.3) — 정상 런 3개(live24/live12/live15) 기준, t_robot 정렬
# ---------------------------------------------------------------------------
# 여기 숫자를 바꾸면 `08` 의 스윕과 로봇의 경보가 **동시에** 바뀝니다. 그게 목적입니다.
#
# ★ 이 규칙은 '들림'만 잡습니다. 슬립은 0% 입니다 (18.4).
#   슬립의 잔차 중앙값이 정상 런의 p95 보다 작아서 절대 잔차로는 원리적으로 못 가릅니다.
#   화면에 'TRACTION LOSS' 같은 포괄적인 이름을 쓰면 거짓말이 됩니다.
RULES = {
    #        지연(ms) 이득    임계   창(초)  비율
    "yaw": dict(lag_ms=100, gain=0.886, thr=0.35, win_sec=2.0, frac=22/24),
    "fwd": dict(lag_ms=350, gain=0.914, thr=0.15, win_sec=2.0, frac=20/24),
}

MIN_CMD = 0.05     # 이 값 미만의 |지령| 은 평가하지 않습니다(정지 중 잡음 배제)

# 실시간 평가 격자. `08` 의 오프라인 스윕이 지령을 20Hz ZOH 격자로 뽑아 평가하므로
# 로봇도 같은 밀도로 평가해야 같은 규칙이 같은 뜻이 됩니다. AxisMonitor 주석 참고.
EVAL_HZ = 20.0

# 경보를 내기 전에 창이 이만큼은 표본으로 차 있어야 합니다(창 길이 대비).
# TimeWindowAlarm.__init__ 의 주석에 왜 필요한지 적어 뒀습니다.
MIN_FILL = 0.20


def dig(m, path):
    for k in path:
        m = m[k]
    return m


def meas_of(msg, axis):
    """메시지 dict -> 그 축의 실측 스칼라 (부호 보정 포함)."""
    ax = AXES[axis]
    return ax.get("sign", 1.0) * dig(msg, ax["field"])


# ---------------------------------------------------------------------------
# 경보 상태기계 — 오프라인/온라인 공용
# ---------------------------------------------------------------------------
class TimeWindowAlarm:
    """최근 win_sec 초 안의 표본 중 frac 비율 이상이 임계를 넘으면 경보.

    왜 '연속 N' 이 아니라 '창 안의 비율' 인가
        실측 고장은 3~8 샘플짜리 버스트가 끊겼다 이어졌다 합니다(live17 타임라인).
        연속을 요구하면 버스트를 놓치고, 연속 조건을 풀면 정상 주행의 순간
        스파이크에 걸립니다. 창 안의 빈도로 보면 둘이 갈립니다.

    왜 창이 '초' 인가
        판정 스트림 주기가 런마다 다릅니다(실측 7.8~16.3Hz). 창을 샘플 수로 고정하면
        같은 K=24 가 어떤 런에선 1.47초, 어떤 런에선 3.07초가 되어 빠른 런만
        불리해집니다. 비교가 성립하지 않습니다.
    """

    def __init__(self, thr, win_sec, frac, nominal_dt=None, min_fill=MIN_FILL):
        self.thr = thr
        self.win_sec = win_sec
        self.frac = frac
        # ★ 창이 '시간' 으로 찼는지가 아니라 '표본' 으로 찼는지를 봐야 합니다.
        #
        # 지령이 min_cmd 아래면 평가에서 빠집니다. 그래서 오래 서 있다가 다시 움직이면
        # 창 안에 표본이 두세 개뿐인 채로 비율이 1.0 이 됩니다 — 그런데 그 두세 개는
        # 하필 **가속 구간**이라 잔차가 큽니다. 정지 후 재출발이 곧 경보가 됩니다.
        # 실측(live24_normal3 23.8초): 22초간 정지 -> 지령 +0.5 투입 -> 자이로가 아직
        # -0.13 -> 창 표본 3개 전부 임계 초과 -> 비율 1.000 -> 가짜 경보.
        # 임계를 0.45 까지 올려도 그대로 남습니다. 임계가 아니라 창이 문제였습니다.
        if nominal_dt:
            self.min_samples = max(3, int(min_fill * win_sec / nominal_dt))
        else:
            self.min_samples = 3
        self._win = collections.deque()      # (t, hit)
        self._cnt = 0
        self._t0 = None
        self.active = False
        self.events = 0
        self.alarmed = 0
        self.first = None
        self.ratio = 0.0

    def feed(self, t, e):
        """잔차 한 건을 넣고 현재 경보 상태를 돌려줍니다."""
        if self._t0 is None:
            self._t0 = t
        hit = 1 if e >= self.thr else 0
        self._win.append((t, hit))
        self._cnt += hit
        while self._win and t - self._win[0][0] > self.win_sec:
            self._cnt -= self._win.popleft()[1]
        n = len(self._win)
        self.ratio = self._cnt / n if n else 0.0
        # 창이 표본으로 안 찼으면 판정하지 않습니다(min_samples 주석 참고).
        if n < self.min_samples:
            self.active = False
            return False
        if self._cnt >= self.frac * n:
            self.alarmed += 1
            if not self.active:
                self.events += 1
                self.active = True
                if self.first is None:
                    self.first = t - self._t0
            return True
        self.active = False
        return False

    def reset(self):
        self._win.clear()
        self._cnt = 0
        self._t0 = None
        self.active = False


def run_alarm(res, thr, win_sec, frac):
    """(t, |e|) 스트림 전체를 흘려서 (events, alarmed, first) 를 돌려줍니다.

    `08` 의 스윕이 부르는 진입점입니다. 로봇 노드는 `TimeWindowAlarm.feed()` 를
    직접 부르지만 **상태기계는 같은 것**입니다.

    표본 주기는 스트림에서 직접 재서 넘깁니다 — `08` 의 판정 스트림 주기가 런마다
    7.8~16.3Hz 로 다르기 때문에 고정값을 쓰면 min_samples 의 뜻이 런마다 달라집니다.
    """
    dt = None
    if len(res) > 8:
        gaps = sorted(res[i + 1][0] - res[i][0] for i in range(len(res) - 1))
        dt = gaps[len(gaps) // 2]
    a = TimeWindowAlarm(thr, win_sec, frac, nominal_dt=dt)
    for t, e in res:
        a.feed(t, e)
    return a.events, a.alarmed, a.first


# ---------------------------------------------------------------------------
# 지령 유지 재샘플링
# ---------------------------------------------------------------------------
def zero_order_hold(cmd, hz):
    """지령을 고정 주기로 다시 뽑습니다. 마지막 값을 다음 지령까지 유지합니다.

    joystick_control.py:150-164 는 축 값이 '바뀔 때만' 발행합니다. 스틱을 일정하게
    유지하면 cmd_vel 이 아예 안 나가고 joy_node 의 autorepeat_rate 도 거기서
    걸러집니다. 그대로 두면 판정 스트림이 3Hz 로 떨어지고, 더 나쁘게는 잔차를
    '지령이 변하는 순간' 에서만 평가하게 됩니다 — 로봇이 아직 못 따라가는 게 정상인
    가속 구간만 골라 보는 셈이라 기준선이 부풀어 오릅니다.

    모터는 다음 지령이 올 때까지 마지막 값을 계속 실행하므로, 유지 재샘플링이
    편법이 아니라 실제 동작에 맞는 모델입니다.

    실측(live11): 원본 3.7Hz 상관 0.764 -> ZOH 20Hz 상관 0.910.
    """
    if not cmd or hz <= 0:
        return cmd
    step = 1.0 / hz
    out = []
    t, end, i = cmd[0][0], cmd[-1][0], 0
    while t <= end:
        while i + 1 < len(cmd) and cmd[i + 1][0] <= t:
            i += 1
        out.append((t, cmd[i][1], cmd[i][2]))
        t += step
    return out


class CmdHold:
    """온라인 ZOH. 지령을 받아두고 임의 과거 시각의 유지값을 돌려줍니다.

    오프라인 `zero_order_hold` 의 스트리밍 판입니다. 로봇 노드는 실측이 올 때마다
    `at(t - lag)` 로 그 시점에 모터가 실행 중이던 지령을 꺼냅니다.
    """

    def __init__(self, keep_sec=10.0):
        self.keep_sec = keep_sec
        self._t = []
        self._v = []          # (vx, wz)

    def push(self, t, vx, wz, topic=None):
        lim = CMD_CLAMP.get(topic) if topic else None
        if lim:
            vlim, wlim = lim
            vx = max(-vlim, min(vlim, vx))
            wz = max(-wlim, min(wlim, wz))
        self._t.append(t)
        self._v.append((vx, wz))
        cut = t - self.keep_sec
        while len(self._t) > 2 and self._t[0] < cut:
            self._t.pop(0)
            self._v.pop(0)

    def at(self, t):
        """t 시점에 유지되고 있던 지령. 없으면 None."""
        if not self._t or t < self._t[0]:
            return None
        i = bisect.bisect_right(self._t, t) - 1
        return self._v[max(0, i)]


# ---------------------------------------------------------------------------
# 축 하나를 실시간으로 감시
# ---------------------------------------------------------------------------
class AxisMonitor:
    """실측이 올 때마다 잔차를 만들고 경보 상태를 갱신합니다.

    로봇 노드가 축마다 하나씩 들고 있습니다. `08` 의 스윕과 같은 `TimeWindowAlarm`
    을 쓰므로 스윕 결과표의 숫자가 그대로 로봇 위 숫자입니다.
    """

    def __init__(self, axis, rule=None, min_cmd=MIN_CMD, hold=None,
                 eval_hz=EVAL_HZ):
        self.axis = axis
        r = dict(RULES[axis])
        if rule:
            r.update(rule)
        self.lag = r["lag_ms"] / 1000.0
        self.gain = r["gain"]
        self.thr = r["thr"]
        self.min_cmd = min_cmd
        self.ci = AXES[axis]["cmd_index"] - 1          # (vx, wz) 안의 위치
        self.hold = hold if hold is not None else CmdHold()
        self.alarm = TimeWindowAlarm(r["thr"], r["win_sec"], r["frac"],
                                     nominal_dt=(1.0 / eval_hz) if eval_hz else None)
        self.eval_step = (1.0 / eval_hz) if eval_hz else 0.0
        self._next_eval = None
        self.last_e = None
        self.evaluated = 0
        self.skipped_idle = 0
        self.skipped_rate = 0

    def push_cmd(self, t, vx, wz, topic=None):
        self.hold.push(t, vx, wz, topic)

    def push_meas(self, t, value):
        """실측 한 건. (잔차, 경보) 또는 평가 대상이 아니면 (None, 현상태)."""
        # ★ 평가는 고정 격자에서만 합니다.
        #
        # 경보는 '창 안의 비율' 이라 **표본 밀도가 판정을 바꿉니다.** 오프라인 스윕은
        # 지령 ZOH 격자(20Hz)에서 평가하는데, 로봇 위에서 실측이 오는 대로 평가하면
        # 자이로가 43Hz 라 표본이 2.1배 촘촘해집니다. 그러면 같은 규칙인데도 짧은
        # 이탈이 창을 채워 버립니다 — 실측(live24_normal3): 60ms 짜리 가짜 경보 1건.
        # 격자를 맞추면 스윕에서 오경보 0 인 규칙이 로봇 위에서도 0 입니다.
        if self.eval_step:
            if self._next_eval is None:
                self._next_eval = t
            elif t < self._next_eval:
                self.skipped_rate += 1
                return None, self.alarm.active
            # 한참 쉬었다 들어와도 격자가 어긋나지 않게 t 기준으로 다시 잡습니다.
            self._next_eval = max(t, self._next_eval) + self.eval_step
        pair = self.hold.at(t - self.lag)
        if pair is None:
            return None, self.alarm.active
        c = pair[self.ci]
        if abs(c) < self.min_cmd:
            # 정지 중에는 판정하지 않습니다. 지령이 0 이면 잔차가 곧 센서 잡음이라
            # 임계를 아무리 올려도 의미가 없습니다.
            self.skipped_idle += 1
            return None, self.alarm.active
        e = abs(value - self.gain * c)
        self.last_e = e
        self.evaluated += 1
        return e, self.alarm.feed(t, e)

    def state(self):
        return {
            "axis": self.axis,
            "residual": self.last_e,
            "threshold": self.thr,
            "ratio": round(self.alarm.ratio, 3),
            "alarm": self.alarm.active,
            "events": self.alarm.events,
            "evaluated": self.evaluated,
        }
