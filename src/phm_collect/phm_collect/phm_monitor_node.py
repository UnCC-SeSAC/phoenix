#!/usr/bin/env python3
"""로봇 위에서 도는 PHM 노드. 잔차 경보를 `/phm/status` 로 발행합니다.

왜 로봇 위인가
--------------
`firefighter_ui` 는 로봇 안에서 ROS 토픽을 읽어 그대로 웹으로 서빙합니다. 호스트도
DB도 거치지 않습니다. PHM 값을 같은 화면에 올리려면 같은 자리에서 나와야 합니다.
호스트의 ClickHouse 를 조회하는 방식은 로봇 -> 호스트 역방향 의존을 만들고, DB 왕복
지연이 라이브 경로에 들어갑니다.

`firefighter_ui` 에 어떻게 붙나
-------------------------------
`firefighter_ui_node.py` 는 `/vla/status` 와 `/rule_based/status` 를 **std_msgs/String
안의 JSON** 으로 받아 내용을 해석하지 않고 그대로 `/api/status` 에 실어 줍니다
(`_update_status` -> `StatusStore.update`). 그래서 PHM 은 **같은 모양의 생산자를
하나 더 붙이는 일**입니다. UI 노드를 뜯어고칠 필요가 없습니다.

검출 규칙은 여기 없습니다
-------------------------
`phm_detect_core` 에 있습니다. 오프라인 스윕(`08`)이 쓰는 것과 **같은 상태기계**라,
스윕 결과표의 숫자가 그대로 이 노드의 숫자입니다. 임계를 여기 다시 적으면 어긋납니다.

검증
----
로봇 없이 `11_replay_detect.py` 로 검증했습니다(19.4). 정상 6런 오경보 0,
들림 76~86%, **슬립 0%**.

★ 슬립은 못 잡습니다
--------------------
잔차 중앙값이 정상 런의 p95 보다 작아서 절대 잔차로는 원리적으로 못 가릅니다(18.4).
그래서 이 노드는 경보 이름을 **`LIFT_SUSPECTED`** 로만 냅니다. 화면에
'TRACTION LOSS' 같은 포괄적인 이름을 쓰면 거짓말이 됩니다.
"""
from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, UInt16
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# ROS 패키지 안에서는 패키지 상대, data_analysis 폴더에서는 평면 import 입니다.
# 같은 파일이 두 곳에 있습니다 — data_analysis/sync_phm_core.sh 가 해시로 확인합니다.
try:
    from phm_collect import phm_detect_core as core
except ImportError:
    import phm_detect_core as core

try:
    try:
        from phm_collect import phm_host_metrics as hostm
    except ImportError:
        import phm_host_metrics as hostm
except ImportError:                      # 파이가 아닌 곳에서 돌릴 때
    hostm = None

SCHEMA_VERSION = 1
ALARM_NAME = "LIFT_SUSPECTED"            # 18.4 — 슬립은 이 검출기로 안 잡힙니다


def _sensor_qos(depth=50):
    """센서 토픽은 BEST_EFFORT 입니다. RELIABLE 로 잡으면 아예 안 붙습니다."""
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=depth)


class PhmMonitor(Node):
    def __init__(self):
        super().__init__("phm_monitor")
        self.declare_parameter("status_topic", "/phm/status")
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("host_period_sec", 5.0)
        self.declare_parameter("stale_sec", 3.0)
        self.declare_parameter("imu_topic", core.AXES["yaw"]["topic"])
        self.declare_parameter("rf2o_topic", core.AXES["fwd"]["topic"])
        self.declare_parameter("battery_topic", "/ros_robot_controller/battery")
        # 지령 토픽은 둘 다 구독하고 **실제로 오는 쪽**을 씁니다. 주행 방식에 따라
        # 살아 있는 토픽이 다릅니다 (nav2 는 /cmd_vel, 조이스틱은 /controller/cmd_vel).
        self.declare_parameter("cmd_topics", list(core.CMD_TOPICS))

        gp = lambda n: self.get_parameter(n).value

        # 지령 보관은 축끼리 **공유**합니다. 축마다 따로 두면 같은 지령을 두 벌
        # 저장하게 되고, 클램프가 한쪽에만 걸리는 사고가 납니다.
        self.hold = core.CmdHold()
        self.mons = {a: core.AxisMonitor(a, hold=self.hold) for a in ("yaw", "fwd")}

        self.cmd_counts: dict[str, int] = {}
        self.last_seen: dict[str, float] = {}
        self.battery_mv = None
        self._host = {}
        self._jiffies = None

        for t in gp("cmd_topics"):
            self.create_subscription(
                Twist, t, lambda m, tp=t: self.on_cmd(m, tp), _sensor_qos(20))
        self.create_subscription(Imu, gp("imu_topic"), self.on_imu, _sensor_qos())
        self.create_subscription(Odometry, gp("rf2o_topic"), self.on_rf2o, _sensor_qos())
        self.create_subscription(UInt16, gp("battery_topic"), self.on_battery,
                                 _sensor_qos(5))

        self.pub = self.create_publisher(String, gp("status_topic"), 10)
        self.create_timer(float(gp("publish_period_sec")), self.tick)
        if hostm is not None:
            self.create_timer(float(gp("host_period_sec")), self.tick_host)
            self.tick_host()

        self.get_logger().info(
            f"phm_monitor 시작 — 규칙 {core.RULES}  경보명 {ALARM_NAME} "
            f"(슬립은 검출되지 않습니다)")

    # ---- 구독 ----
    def on_cmd(self, msg: Twist, topic: str):
        self.cmd_counts[topic] = self.cmd_counts.get(topic, 0) + 1
        self.last_seen[topic] = time.time()
        # 클램프는 토픽마다 다릅니다(/cmd_vel 만 펌웨어에서 잘립니다). CmdHold 가
        # 토픽을 보고 알아서 자릅니다 — 여기서 자르면 안 됩니다.
        self.hold.push(self._stamp(msg), msg.linear.x, msg.angular.z, topic)

    def on_imu(self, msg: Imu):
        self.last_seen["imu"] = time.time()
        self.mons["yaw"].push_meas(self._stamp(msg, msg.header),
                                   msg.angular_velocity.z)

    def on_rf2o(self, msg: Odometry):
        self.last_seen["rf2o"] = time.time()
        # 부호 반전은 코어가 압니다 — 라이다가 180도 돌아 장착돼 있어서
        # rf2o 가 전진할 때 vx 를 음수로 냅니다.
        v = core.AXES["fwd"].get("sign", 1.0) * msg.twist.twist.linear.x
        self.mons["fwd"].push_meas(self._stamp(msg, msg.header), v)

    def on_battery(self, msg: UInt16):
        self.last_seen["battery"] = time.time()
        self.battery_mv = int(msg.data)

    def _stamp(self, msg, header=None):
        """ROS 헤더 시각이 있으면 그것, 없으면 지금.

        헤더가 있는 쪽을 쓰는 이유는 **측정이 일어난 시각**이 필요해서입니다.
        콜백이 불린 시각을 쓰면 실행기가 밀릴 때 두 스트림이 어긋납니다 —
        호스트 도착 시각으로 정렬하다 임계가 무너졌던 것과 같은 실수입니다(18.1).
        """
        if header is not None:
            s = header.stamp
            if s.sec or s.nanosec:
                return s.sec + s.nanosec * 1e-9
        return time.time()

    # ---- 호스트 상태 ----
    def tick_host(self):
        if hostm is None:
            return
        out = {"thermal_c": hostm.read_thermal()}
        out.update(hostm.read_freq_cap())
        out.update(hostm.read_meminfo())
        thr = hostm.read_throttled(None) or hostm.read_vcio_throttled()
        if "throttled_raw" not in thr:
            sysfs = hostm.read_sysfs_throttled()
            if sysfs:
                sysfs.update(hostm.decode_throttle_bits(int(sysfs["throttled_raw"], 16)))
                thr = sysfs
        out.update(thr)
        # CPU 사용률은 두 샘플의 jiffies 차이로만 구할 수 있습니다.
        j = hostm.read_cpu_jiffies()
        if j and self._jiffies:
            dt = j["total"] - self._jiffies["total"]
            di = j["idle"] - self._jiffies["idle"]
            if dt > 0:
                out["cpu_used_pct"] = round(100.0 * (dt - di) / dt, 1)
        if j:
            self._jiffies = j
        self._host = out

    # ---- 발행 ----
    def _cmd_src(self):
        return max(self.cmd_counts, key=self.cmd_counts.get) if self.cmd_counts else None

    def tick(self):
        now = time.time()
        stale = float(self.get_parameter("stale_sec").value)

        axes = {}
        alarms = []
        for name, mon in self.mons.items():
            st = mon.state()
            ax = core.AXES[name]
            st.update(unit=ax["unit"], label=ax["label"], meas=ax["meas"])
            # 실측 토픽이 끊기면 잔차가 '마지막 값' 으로 굳습니다. 조용히 굳은 값을
            # 정상으로 보여주면 안 되므로 신선도를 같이 냅니다.
            key = "imu" if name == "yaw" else "rf2o"
            age = now - self.last_seen[key] if key in self.last_seen else None
            st["fresh"] = age is not None and age <= stale
            st["age_sec"] = round(age, 2) if age is not None else None
            axes[name] = st
            if st["alarm"] and st["fresh"]:
                alarms.append({"name": ALARM_NAME, "axis": name,
                               "residual": st["residual"], "threshold": st["threshold"]})

        blocked = None
        if not self.cmd_counts:
            blocked = "지령 토픽이 아직 없습니다 — 주행을 시작하면 판정이 시작됩니다."
        elif not any(a["fresh"] for a in axes.values()):
            blocked = "실측 토픽이 끊겼습니다 (imu / rf2o 확인)."
        elif not any(a["evaluated"] for a in axes.values()):
            blocked = "정지 상태입니다 — 지령이 있어야 판정합니다."

        payload = {
            "schema_version": SCHEMA_VERSION,
            "mode": "PHM",
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(now)),
            "health": "ALARM" if alarms else ("UNKNOWN" if blocked else "OK"),
            "alarms": alarms,
            # ★ 이 검출기가 못 잡는 것을 화면이 알 수 있게 같이 실어 보냅니다.
            #    받는 쪽이 'ALL CLEAR' 라고 쓰지 않게 하려는 것입니다.
            "not_detected": ["SLIP"],
            "axes": axes,
            "cmd_source": self._cmd_src(),
            "battery_mv": self.battery_mv,
            "host": self._host,
            "blocked_reason": blocked,
            "rules": {k: dict(v) for k, v in core.RULES.items()},
        }
        m = String()
        m.data = json.dumps(payload, ensure_ascii=False, allow_nan=False)
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PhmMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
