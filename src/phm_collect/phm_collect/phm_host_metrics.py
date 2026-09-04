#!/usr/bin/env python3
"""파이 호스트 상태 읽기. `04` 브리지와 로봇의 PHM 노드가 함께 씁니다.

왜 이 파일이 따로 있나
----------------------
원래 `04_robot_zmq_bridge.py` 안에 있었습니다. 그런데 로봇에서 도는 PHM 노드도
같은 값(온도·스로틀·CPU·메모리)을 화면에 올려야 해서, 그대로 두면 200줄이 두 벌이
됩니다. `phm_wire.py`(전선 형식)·`phm_detect_core.py`(검출 규칙) 와 같은 이유로
한 곳에만 둡니다.

이 값들은 **파이 안에서만** 읽힙니다. 원격 호스트에서는 어떤 방법으로도 가져올 수
없습니다. psutil 을 쓰지 않고 /proc, /sys 를 직접 읽습니다 — 컨테이너에 없을 수
있어서입니다.

배포 주의: 로봇에 04 를 올릴 때 **이 파일도 같이** 올라가야 합니다
(`ensure_bridge.sh` 의 scp 목록에 들어 있습니다).
"""
from __future__ import annotations

import glob
import os
import subprocess

def read_thermal() -> dict:
    """/sys/class/thermal/thermal_zone*/temp -> {zone이름: 섭씨}"""
    import glob, os
    out = {}
    for zone in sorted(glob.glob("/sys/class/thermal/thermal_zone*")):
        try:
            with open(os.path.join(zone, "temp")) as f:
                milli = int(f.read().strip())
            name = os.path.basename(zone)
            try:
                with open(os.path.join(zone, "type")) as f:
                    name = f"{name}:{f.read().strip()}"
            except OSError:
                pass
            out[name] = milli / 1000.0
        except (OSError, ValueError):
            continue
    return out


def read_cpu_jiffies() -> dict | None:
    """/proc/stat 의 aggregate cpu 행. 사용률은 두 샘플의 차이로만 구합니다."""
    try:
        with open("/proc/stat") as f:
            for line in f:
                if line.startswith("cpu "):
                    v = [int(x) for x in line.split()[1:]]
                    idle = v[3] + (v[4] if len(v) > 4 else 0)  # idle + iowait
                    return {"total": sum(v), "idle": idle}
    except (OSError, ValueError):
        pass
    return None


def read_meminfo() -> dict:
    try:
        info = {}
        with open("/proc/meminfo") as f:
            for line in f:
                k, _, rest = line.partition(":")
                info[k] = int(rest.split()[0])  # kB
        total, avail = info.get("MemTotal", 0), info.get("MemAvailable", 0)
        return {
            "mem_total_mb": round(total / 1024, 1),
            "mem_avail_mb": round(avail / 1024, 1),
            "mem_used_pct": round((1 - avail / total) * 100, 1) if total else None,
        }
    except (OSError, ValueError, KeyError):
        return {}


def read_cpu_freq() -> float | None:
    """현재 CPU 클럭 (MHz). 스로틀이 걸리면 여기가 먼저 떨어집니다."""
    try:
        with open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq") as f:
            return int(f.read().strip()) / 1000.0
    except (OSError, ValueError):
        return None



def read_freq_cap() -> dict:
    """클럭 상한 대비 현재 클럭. vcgencmd 없이 쓰는 스로틀 대체 지표입니다.

    스로틀이 걸리면 scaling_cur_freq 가 cpuinfo_max_freq 아래로 눌립니다.
    /sys 만 읽으므로 컨테이너 안에서도 파이 값이 그대로 나옵니다.
    """
    base = "/sys/devices/system/cpu/cpu0/cpufreq/"
    out = {}
    for key, fname in (("cpu_mhz", "scaling_cur_freq"),
                       ("cpu_mhz_max", "cpuinfo_max_freq"),
                       ("cpu_mhz_min", "cpuinfo_min_freq")):
        try:
            with open(base + fname) as f:
                out[key] = int(f.read().strip()) / 1000.0
        except (OSError, ValueError):
            pass
    try:
        with open(base + "scaling_governor") as f:
            out["cpu_governor"] = f.read().strip()
    except OSError:
        pass

    cur, mx, mn = out.get("cpu_mhz"), out.get("cpu_mhz_max"), out.get("cpu_mhz_min")
    if cur and mx:
        out["freq_ratio"] = round(cur / mx, 3)
    # freq_ratio 만 보면 오독합니다. 유휴 시 거버너가 최소 클럭으로 내려두면
    # ratio 가 낮게 나오는데 이건 스로틀이 아닙니다 (라즈베리파이5 는 min=1500,
    # max=2400 이라 유휴 시 항상 0.625 입니다).
    # 최소 클럭에 붙어 있는지를 따로 표시해서 둘을 구분합니다.
    if cur and mn:
        out["at_min_freq"] = bool(abs(cur - mn) < 1.0)
    return out


def read_hwmon() -> dict:
    """hwmon 의 알람 비트와 전압. 저전압 감지의 vcgencmd 없는 경로입니다.

    라즈베리파이는 rpi_volt 드라이버가 in0_lcrit_alarm=1 로 저전압을 알립니다.
    보드마다 이름이 달라서 특정하지 않고 전부 훑어 1 인 알람만 담습니다.
    """
    import glob, os
    out = {}
    for hw in sorted(glob.glob("/sys/class/hwmon/hwmon*")):
        try:
            with open(os.path.join(hw, "name")) as f:
                name = f.read().strip()
        except OSError:
            continue
        for alarm in sorted(glob.glob(os.path.join(hw, "*_alarm"))):
            try:
                with open(alarm) as f:
                    if f.read().strip() == "1":
                        out[f"{name}:{os.path.basename(alarm)}"] = True
            except (OSError, ValueError):
                continue
        for vin in sorted(glob.glob(os.path.join(hw, "in*_input"))):
            try:
                with open(vin) as f:
                    out[f"{name}:{os.path.basename(vin)}_V"] = int(f.read().strip()) / 1000.0
            except (OSError, ValueError):
                continue
    return out


def read_sysfs_throttled() -> dict:
    """일부 파이 커널이 노출하는 get_throttled sysfs. 있으면 vcgencmd 와 같은 값입니다."""
    for path in ("/sys/devices/platform/soc/soc:firmware/get_throttled",
                 "/sys/devices/platform/soc/soc:firmware/get_throttled/get_throttled"):
        try:
            with open(path) as f:
                return {"throttled_raw": hex(int(f.read().strip(), 0))}
        except (OSError, ValueError):
            continue
    return {}


def decode_throttle_bits(raw: int) -> dict:
    """get_throttled 비트 해석 (vcgencmd / sysfs 공용).

    하위 비트 = 지금 발생 중, 상위(16~19) = 부팅 후 한 번이라도 발생.
    PHM 에서는 상위 래치 비트가 특히 중요합니다 — 순간 스로틀은 샘플링에서
    놓쳐도 이력은 남기 때문입니다.
    """
    bits = {
        0: "under_voltage_now", 1: "arm_freq_capped_now",
        2: "throttled_now", 3: "soft_temp_limit_now",
        16: "under_voltage_occurred", 17: "arm_freq_capped_occurred",
        18: "throttled_occurred", 19: "soft_temp_limit_occurred",
    }
    return {name: bool(raw & (1 << b)) for b, name in bits.items()}


# 라즈베리파이 메일박스 property 인터페이스 태그
RPI_TAG_GET_THROTTLED = 0x00030046


def _vcio_ioctl_op() -> int:
    """_IOWR(100, 0, char*) 를 계산합니다.

    포인터 크기가 인코딩에 들어가서 32/64비트가 다릅니다.
    64비트: 0xC0086400, 32비트: 0xC0046400
    """
    import ctypes
    size = ctypes.sizeof(ctypes.c_void_p)
    return (3 << 30) | (size << 16) | (100 << 8) | 0


def read_vcio_throttled() -> dict:
    """/dev/vcio 에 직접 ioctl 해서 get_throttled 를 읽습니다.

    vcgencmd 바이너리 없이도 동작합니다 — vcgencmd 자체가 이 ioctl 의
    얇은 래퍼일 뿐이라, 컨테이너에 /dev/vcio 만 넘어와 있으면 충분합니다.

    메일박스 메시지 구조 (u32 워드 7개):
        [0] 전체 크기(바이트)   [1] 요청코드 0
        [2] 태그 ID             [3] 값 버퍼 크기  [4] 요청/응답 표시
        [5] 값 (응답에 결과)    [6] 끝 태그 0

    권한 주의: /dev/vcio 는 root:video 소유입니다. 컨테이너에서 -u ubuntu 로
    도는데 ubuntu 가 video 그룹이 아니면 PermissionError 가 납니다.
    """
    import fcntl
    import struct

    try:
        buf = bytearray(struct.pack("7I", 28, 0, RPI_TAG_GET_THROTTLED, 4, 0, 0, 0))
        with open("/dev/vcio", "rb") as f:
            fcntl.ioctl(f.fileno(), _vcio_ioctl_op(), buf, True)
        raw = struct.unpack("7I", bytes(buf))[5]
    except PermissionError:
        return {"throttled_err": "vcio 권한 없음 (video 그룹 필요)"}
    except (OSError, ValueError, struct.error):
        return {}

    res = {"throttled_raw": hex(raw), "throttled_src": "vcio"}
    res.update(decode_throttle_bits(raw))
    return res


def read_throttled(vcgencmd: str | None) -> dict:
    """vcgencmd get_throttled. 컨테이너에는 보통 없어서 실패해도 정상입니다."""
    if not vcgencmd:
        return {}
    import subprocess
    try:
        out = subprocess.run([vcgencmd, "get_throttled"], capture_output=True,
                             text=True, timeout=2).stdout.strip()
        raw = int(out.split("=")[1], 16)
    except (OSError, ValueError, IndexError, subprocess.SubprocessError):
        return {}
    res = {"throttled_raw": hex(raw), "throttled_src": "vcgencmd"}
    res.update(decode_throttle_bits(raw))
    return res
