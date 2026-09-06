#!/usr/bin/env python3
"""수집한 JSONL 을 **진짜 ROS 토픽**으로 다시 틀어 주는 목업 발행기.

왜 필요한가
-----------
센서가 하나도 없는 파이(192.168.1.174 의 IntelPi 컨테이너)에서 PHM 전체 경로를
확인하기 위한 것입니다. `11_replay_detect.py` / `12_test_phm_monitor.py` 는 콜백을
직접 부르므로 **ROS 를 안 지납니다.** 그래서 이런 것들을 못 봅니다:

    colcon 빌드 (aarch64 / Humble / Python 3.10)
    QoS 협상 (BEST_EFFORT 센서 토픽)
    DDS 왕복과 실행기 스케줄링
    firefighter_ui 가 LAN 으로 서빙되는지

이 노드는 그 사이를 메웁니다. 실기 검증(4단계)을 대신하지는 못합니다 —
실제 센서 주기·CPU 부하·진짜 주행은 여전히 로봇이 필요합니다.

★ 타임스탬프를 '지금' 으로 다시 씁니다
--------------------------------------
`phm_monitor` 는 두 시계를 씁니다.

    정렬(잔차)   메시지 헤더 스탬프    <- 측정이 일어난 시각이어야 합니다
    신선도(fresh) 벽시계 time.time()   <- 토픽이 끊겼는지 보는 용도

녹화 파일의 스탬프를 그대로 내보내면 헤더는 몇 달 전이고 벽시계는 지금이라
**모든 축이 stale** 로 잡힙니다. 그래서 원본의 **간격은 보존하고 기준점만 지금으로**
옮깁니다. 잔차 계산은 간격에만 의존하므로 결과가 안 바뀝니다.

    ros2 run phm_collect phm_mock_source --ros-args -p jsonl:=/shared/live25_lift.jsonl
    ros2 launch phm_collect phm_monitor_mock.launch.py jsonl:=/shared/live25_lift.jsonl
"""
from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt16

try:
    from phm_collect import phm_detect_core as core
except ImportError:
    import phm_detect_core as core

BATTERY_TOPIC = "/ros_robot_controller/battery"


def _sensor_qos(depth=50):
    # 실제 센서와 같게 BEST_EFFORT 로 냅니다. phm_monitor 도 BEST_EFFORT 로 받으므로
    # 여기서 RELIABLE 로 내면 '되긴 하지만' 실기와 다른 조건에서 시험하게 됩니다.
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=depth)


class PhmMockSource(Node):
    def __init__(self):
        super().__init__("phm_mock_source")
        self.declare_parameter("jsonl", "")
        self.declare_parameter("speed", 1.0)
        self.declare_parameter("loop", False)
        self.declare_parameter("cmd_topic", "")      # 비우면 파일에서 고릅니다
        self.declare_parameter("tick_hz", 200.0)

        path = str(self.get_parameter("jsonl").value)
        if not path:
            raise SystemExit("jsonl 파라미터가 필요합니다 (-p jsonl:=/경로/run.jsonl)")
        self.speed = max(0.01, float(self.get_parameter("speed").value))
        self.loop = bool(self.get_parameter("loop").value)

        self.rows, self.src = self._load(path)
        if not self.rows:
            raise SystemExit(f"{path}: 재생할 메시지가 없습니다")

        self.pub_cmd = self.create_publisher(Twist, self.src, _sensor_qos(20))
        self.pub_imu = self.create_publisher(Imu, core.AXES["yaw"]["topic"], _sensor_qos())
        self.pub_odom = self.create_publisher(Odometry, core.AXES["fwd"]["topic"], _sensor_qos())
        self.pub_bat = self.create_publisher(UInt16, BATTERY_TOPIC, _sensor_qos(5))

        self.i = 0
        self.t_file0 = self.rows[0][0]
        self.t_wall0 = time.time()
        self.sent = 0
        self.laps = 0
        self.create_timer(1.0 / float(self.get_parameter("tick_hz").value), self.tick)
        self.create_timer(10.0, self.report)

        dur = self.rows[-1][0] - self.t_file0
        self.get_logger().info(
            f"목업 재생 시작 — {path}  {len(self.rows)}건 / {dur:.1f}초  "
            f"x{self.speed}  loop={self.loop}")
        self.get_logger().info(
            f"  지령 {self.src} · 자이로 {core.AXES['yaw']['topic']} · "
            f"rf2o {core.AXES['fwd']['topic']} · 배터리 {BATTERY_TOPIC}")

    def _load(self, path):
        want_cmd = str(self.get_parameter("cmd_topic").value)
        keep = {core.AXES["yaw"]["topic"], core.AXES["fwd"]["topic"], BATTERY_TOPIC}
        rows, counts = [], {}
        for line in open(path):
            try:
                r = json.loads(line)
            except ValueError:
                continue
            # 정렬은 t_robot 입니다. t_recv 로 하면 안 됩니다 — 도착 지터가 최대
            # 14.7초라 두 스트림이 어긋납니다(18.1).
            t = r.get("t_robot") or r.get("t_recv")
            tp = r.get("topic")
            if t is None or tp is None:
                continue
            if tp in core.CMD_TOPICS:
                counts[tp] = counts.get(tp, 0) + 1
                rows.append((t, tp, r.get("msg") or {}))
            elif tp in keep:
                rows.append((t, tp, r.get("msg") or {}))
        rows.sort(key=lambda x: x[0])
        src = want_cmd or (max(counts, key=counts.get) if counts else core.CMD_TOPICS[1])
        # 고른 지령 토픽만 남깁니다. 둘이 섞여 나가면 클램프된 스트림과 안 된
        # 스트림이 교차해 잔차가 망가집니다(08 의 load 가 경고하는 그 상황).
        rows = [r for r in rows if r[1] not in core.CMD_TOPICS or r[1] == src]
        return rows, src

    def _stamp(self, header, wall):
        header.stamp.sec = int(wall)
        header.stamp.nanosec = int((wall - int(wall)) * 1e9)

    def tick(self):
        now = time.time()
        elapsed = (now - self.t_wall0) * self.speed
        while self.i < len(self.rows) and (self.rows[self.i][0] - self.t_file0) <= elapsed:
            t, tp, m = self.rows[self.i]
            self.i += 1
            # 원본 간격은 보존하고 기준점만 지금으로 옮깁니다(모듈 설명 참고).
            wall = self.t_wall0 + (t - self.t_file0) / self.speed
            try:
                if tp == self.src:
                    msg = Twist()
                    msg.linear.x = float(m["linear"]["x"])
                    msg.angular.z = float(m["angular"]["z"])
                    self.pub_cmd.publish(msg)
                elif tp == core.AXES["yaw"]["topic"]:
                    msg = Imu()
                    self._stamp(msg.header, wall)
                    msg.angular_velocity.z = float(m["angular_velocity"]["z"])
                    self.pub_imu.publish(msg)
                elif tp == core.AXES["fwd"]["topic"]:
                    msg = Odometry()
                    self._stamp(msg.header, wall)
                    # 원본(부호 안 뒤집힌) 값을 그대로 냅니다 — phm_monitor 가
                    # AXES['fwd']['sign'] 으로 자기가 뒤집습니다.
                    msg.twist.twist.linear.x = float(m["twist"]["twist"]["linear"]["x"])
                    self.pub_odom.publish(msg)
                elif tp == BATTERY_TOPIC:
                    msg = UInt16()
                    msg.data = int(m["data"])
                    self.pub_bat.publish(msg)
                else:
                    continue
                self.sent += 1
            except (KeyError, TypeError, ValueError):
                continue

        if self.i >= len(self.rows):
            if not self.loop:
                if self.laps == 0:
                    self.laps = 1
                    self.get_logger().info(
                        f"재생 완료 — {self.sent}건 발행. 노드는 계속 떠 있습니다"
                        f"(토픽이 끊기므로 phm_monitor 가 stale 로 바뀌는 것도 확인하세요).")
                return
            self.laps += 1
            self.i = 0
            self.t_wall0 = time.time()
            self.get_logger().info(f"반복 {self.laps}회차")

    def report(self):
        if self.i < len(self.rows):
            pct = 100.0 * self.i / len(self.rows)
            self.get_logger().info(f"재생 {pct:.0f}%  발행 {self.sent}건")


def main(args=None):
    rclpy.init(args=args)
    node = PhmMockSource()
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
