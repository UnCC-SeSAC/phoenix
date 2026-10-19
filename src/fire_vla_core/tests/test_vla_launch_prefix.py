import importlib.util
import shlex
import sys
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.utilities import evaluate_parameters


BRINGUP_LAUNCH = (
    Path(__file__).parents[2] / "fire_vla_bringup" / "launch"
)


def load_launch_module(filename):
    path = BRINGUP_LAUNCH / filename
    spec = importlib.util.spec_from_file_location(
        filename.replace(".", "_"),
        path,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    "filename",
    ["local_mock_vla.launch.py", "topic_bridge_vla.launch.py"],
)
def test_default_vla_python_keeps_empty_prefix(filename):
    module = load_launch_module(filename)
    context = LaunchContext()
    context.launch_configurations["vla_python_executable"] = ""

    node = module._create_vla_node(context)[0]

    assert perform_substitutions(
        context,
        node.process_description.prefix,
    ) == ""


@pytest.mark.parametrize(
    "filename",
    ["local_mock_vla.launch.py", "topic_bridge_vla.launch.py"],
)
def test_custom_vla_python_is_the_only_prefix(filename):
    module = load_launch_module(filename)
    context = LaunchContext()
    context.launch_configurations["vla_python_executable"] = sys.executable

    node = module._create_vla_node(context)[0]

    assert perform_substitutions(
        context,
        node.process_description.prefix,
    ) == shlex.quote(sys.executable)


@pytest.mark.parametrize(
    "filename",
    ["local_mock_vla.launch.py", "topic_bridge_vla.launch.py"],
)
def test_missing_vla_python_fails_explicitly(filename):
    module = load_launch_module(filename)
    context = LaunchContext()
    context.launch_configurations[
        "vla_python_executable"
    ] = "/definitely/missing/python"

    with pytest.raises(RuntimeError, match="파일이 없습니다"):
        module._create_vla_node(context)


@pytest.mark.parametrize("ab_demo", ["false", "true"])
def test_topic_bridge_distance_defaults_and_overrides_reach_orchestrator(ab_demo):
    module = load_launch_module("topic_bridge_vla.launch.py")
    description = module.generate_launch_description()
    defaults = {
        action.name: perform_substitutions(LaunchContext(), action.default_value)
        for action in description.entities
        if hasattr(action, "name") and hasattr(action, "default_value")
    }
    assert defaults["navigation_standoff_m"] == "0.15"
    assert defaults["spray_range_m"] == "0.30"
    assert defaults["entity_merge_distance_m"] == "0.15"
    assert defaults["ab_demo_mode"] == "false"

    context = LaunchContext()
    context.launch_configurations.update({
        "vla_python_executable": "",
        "llm_backend": "mock",
        "transformers_model_id": "test-model",
        "transformers_device": "cpu",
        "transformers_max_new_tokens": "96",
        "remote_qwen_endpoint": "http://127.0.0.1:8088/infer",
        "remote_qwen_timeout_sec": "3.0",
        "person_confidence_threshold": "0.50",
        "fire_confidence_threshold": "0.25",
        "person_fire_risk_distance_m": "0.30",
        "entity_merge_distance_m": "0.15",
        "ab_demo_mode": ab_demo,
        "navigation_standoff_m": "0.35",
        "spray_range_m": "0.40",
    })
    node = module._create_vla_node(context)[0]
    parameters = evaluate_parameters(context, node._Node__parameters)[0]

    assert parameters["navigation_standoff_m"] == pytest.approx(0.35)
    assert parameters["spray_range_m"] == pytest.approx(0.40)
    assert parameters["entity_merge_distance_m"] == pytest.approx(0.15)
    assert parameters["ab_demo_mode"] is (ab_demo == "true")


def test_hardware_wrapper_exposes_only_distance_overrides():
    wrapper = (
        Path(__file__).parents[3] / "scripts" / "vla_hardware_e2e.sh"
    ).read_text(encoding="utf-8")

    assert '${VLA_NAVIGATION_STANDOFF_M:-0.25}' in wrapper
    assert '${VLA_SPRAY_RANGE_M:-0.45}' in wrapper
    assert "navigation_standoff_m:=$NAVIGATION_STANDOFF_M" in wrapper
    assert "spray_range_m:=$SPRAY_RANGE_M" in wrapper
    assert 'VLA_PARAMETERS: navigation_standoff_m=' in wrapper
    assert '${VLA_AB_DEMO_MODE:-true}' in wrapper
    assert 'ab_demo_mode:=$AB_DEMO_MODE' in wrapper
    assert 'from rcl_interfaces.srv import GetParameters' in wrapper
    assert 'rclpy.parameter_client' not in wrapper
    assert '"/vla_orchestrator/get_parameters"' in wrapper


def test_wrapper_presentation_defaults_need_no_exports():
    import os
    import subprocess
    wrapper = Path(__file__).parents[3] / "scripts" / "vla_hardware_e2e.sh"
    env = {k: v for k, v in os.environ.items() if not k.startswith("VLA_")}
    env["VLA_E2E_DRY_RUN"] = "1"
    for command in ("start", "status", "stop"):
        result = subprocess.run(["bash", str(wrapper), command], env=env,
                                capture_output=True, text=True, timeout=5, check=True)
        assert "/ros2_ws/phoenix_vla/log/hardware_e2e" in result.stdout
        if command == "start":
            assert "AB_DEMO_MODE: true" in result.stdout
            assert "navigation_standoff_m=0.25 spray_range_m=0.45" in result.stdout
            assert "192.168.1.198:8088/infer" in result.stdout
        if command == "status":
            assert "192.168.1.198:8088/health" in result.stdout
    env.update(VLA_NAVIGATION_STANDOFF_M="0.35", VLA_SPRAY_RANGE_M="0.40",
               VLA_QWEN_ENDPOINT="http://override.invalid:8088/infer")
    result = subprocess.run(["bash", str(wrapper), "start"], env=env,
                            capture_output=True, text=True, timeout=5, check=True)
    assert "navigation_standoff_m=0.35 spray_range_m=0.40" in result.stdout
    assert "override.invalid:8088/infer" in result.stdout


def test_wrapper_status_exposes_observer_error_without_starting_runtime(tmp_path):
    import os
    import subprocess
    docker = tmp_path / "docker"
    docker.write_text("""#!/bin/bash
case "$*" in
  *STATUS_OBSERVER=*) echo 'DIAGNOSTIC_IMPORT_ERROR' >&2; exit 1 ;;
  *' cat '*) echo '/ros2_ws/phoenix_vla/log/hardware_e2e/test_run' ;;
  *'tail -n 8'*) echo 'map target frame does not exist' ;;
  *'exec -i'*) cat >/dev/null ;;
  *) echo 'unexpected docker action' >&2; exit 99 ;;
esac
""")
    curl = tmp_path / "curl"
    curl.write_text('#!/bin/bash\necho "health ok"\n')
    docker.chmod(0o755)
    curl.chmod(0o755)
    env = {k: v for k, v in os.environ.items() if not k.startswith("VLA_")}
    env["PATH"] = f"{tmp_path}:{env['PATH']}"
    wrapper = Path(__file__).parents[3] / "scripts" / "vla_hardware_e2e.sh"
    result = subprocess.run(["bash", str(wrapper), "status"], env=env,
                            capture_output=True, text=True, timeout=5, check=True)
    assert "integrated_status: UNKNOWN" in result.stdout
    assert "DIAGNOSTIC_IMPORT_ERROR" in result.stdout
    assert "status_observer.log (container 내부)" in result.stdout
    assert "map target frame does not exist" in result.stdout
    assert "QWEN_HEALTH: PASS" in result.stdout
