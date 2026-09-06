"""기존 H/W 체인을 사용하되 state_manager만 데모 전용 노드로 교체한다.

기반 launch를 그대로 재사용하므로 카메라, YOLO, SLAM, Nav2, GPIO와
fire suppression 설정은 기존 H/W 테스트와 항상 동일하게 유지된다.
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory


def _load_hardware_test_launch():
    launch_path = os.path.join(
        get_package_share_directory('uncc_example'),
        'launch',
        'frontier_fire_suppression_hw_test.launch.py',
    )
    spec = importlib.util.spec_from_file_location(
        'uncc_example_frontier_fire_suppression_hw_test',
        launch_path,
    )

    if spec is None or spec.loader is None:
        raise RuntimeError(f'기반 launch를 불러올 수 없습니다: {launch_path}')

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def generate_launch_description():
    base_launch = _load_hardware_test_launch()
    original_node = base_launch.Node

    def demo_node(*args, **kwargs):
        if kwargs.get('executable') == 'state_manager':
            kwargs['executable'] = 'demo_state_manager'
            kwargs['parameters'] = [{
                'sweep_angle_deg': 15.0,
                'sweep_dwell_sec': 1.0,
                'initial_scan_max_rounds': 2,
                'cluster_detection_timeout_sec': 2.0,
                'base_detection_dwell_sec': 2.0,
                'single_fire_timeout_sec': 8.0,
                'heading_tolerance_deg': 2.0,
            }]

        return original_node(*args, **kwargs)

    # 기반 파일 자체는 수정하지 않고 Node 생성 시점에 state_manager 하나만
    # 교체한다. 생성 후에는 원래 심볼을 복원해 다른 launch에 영향이 없다.
    base_launch.Node = demo_node
    try:
        return base_launch.generate_launch_description()
    finally:
        base_launch.Node = original_node
