"""시나리오 3: 단일 객체도 우선 처리하고 base 재탐색 후 최종 판정."""

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
    """Build the hardware demo launch with demo3 mission policy."""
    base_launch = _load_hardware_test_launch()
    original_node = base_launch.Node

    def scenario_3_node(*args, **kwargs):
        if kwargs.get("executable") == "mission_executor":
            kwargs["parameters"] = list(kwargs.get("parameters", [])) + [
                {
                    # mission_executor converts the object coordinate into a
                    # costmap-validated stand-off pose before calling Nav2.
                    "object_approach_enabled": True,
                    "approach_allow_unknown": True,
                    "front_wheel_offset_m": 0.12,
                    "object_clearance_m": 0.20,
                    "person_clearance_m": 0.20,
                    "person_goal_tolerance_m": 0.10,
                }
            ]
        if kwargs.get("executable") == "state_manager":
            kwargs["executable"] = "demo_state_manager_3"
            kwargs["parameters"] = list(kwargs.get("parameters", [])) + [
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

    base_launch.Node = scenario_3_node
    try:
        return base_launch.generate_launch_description()
    finally:
        base_launch.Node = original_node
