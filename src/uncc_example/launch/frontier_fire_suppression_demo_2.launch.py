"""시나리오 2: 확인된 fire 처리 → 필요시 base 재탐색 → 무화재 확인 후 종료."""

# 기존 H/W 체인에 객체 접근 설정과 시나리오 상태 관리자를 적용한다.
import importlib.util
import os

from ament_index_python.packages import get_package_share_directory


def _load_hardware_test_launch():
    launch_path = os.path.join(
        get_package_share_directory("uncc_example"),
        "launch",
        "frontier_fire_suppression_hw_test.launch.py",
    )
    spec = importlib.util.spec_from_file_location(
        "uncc_example_frontier_fire_suppression_hw_test",
        launch_path,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"기반 launch를 불러올 수 없습니다: {launch_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def generate_launch_description():
    base_launch = _load_hardware_test_launch()
    original_node = base_launch.Node

    def scenario_2_node(*args, **kwargs):
        if kwargs.get("executable") == "mission_executor":
            kwargs["parameters"] = list(kwargs.get("parameters", [])) + [
                {
                    "object_approach_enabled": True,
                    "front_wheel_offset_m": 0.12,
                    "object_clearance_m": 0.15,
                    "person_clearance_m": 0.25,
                    "person_goal_tolerance_m": 0.10,
                }
            ]
        if kwargs.get("executable") == "state_manager":
            kwargs["executable"] = "demo_state_manager_2"
            kwargs["parameters"] = [
                {
                    "sweep_angle_deg": 15.0,
                    "sweep_dwell_sec": 1.0,
                    "initial_scan_max_rounds": 2,
                    "cluster_detection_timeout_sec": 2.0,
                    "single_fire_timeout_sec": 8.0,
                    "heading_tolerance_deg": 2.0,
                }
            ]
        return original_node(*args, **kwargs)

    base_launch.Node = scenario_2_node
    try:
        return base_launch.generate_launch_description()
    finally:
        base_launch.Node = original_node
