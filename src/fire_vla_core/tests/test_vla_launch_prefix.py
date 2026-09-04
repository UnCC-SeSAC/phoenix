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


def test_topic_bridge_distance_defaults_and_overrides_reach_orchestrator():
    module = load_launch_module("topic_bridge_vla.launch.py")
    description = module.generate_launch_description()
    defaults = {
        action.name: perform_substitutions(LaunchContext(), action.default_value)
        for action in description.entities
        if hasattr(action, "name") and hasattr(action, "default_value")
    }
    assert defaults["navigation_standoff_m"] == "0.15"
    assert defaults["spray_range_m"] == "0.30"

    context = LaunchContext()
    context.launch_configurations.update({
        "vla_python_executable": "",
        "llm_backend": "mock",
        "transformers_model_id": "test-model",
        "transformers_device": "cpu",
        "transformers_max_new_tokens": "64",
        "remote_qwen_endpoint": "http://127.0.0.1:8088/infer",
        "remote_qwen_timeout_sec": "3.0",
        "person_fire_risk_distance_m": "0.10",
        "navigation_standoff_m": "0.35",
        "spray_range_m": "0.40",
    })
    node = module._create_vla_node(context)[0]
    parameters = evaluate_parameters(context, node._Node__parameters)[0]

    assert parameters["navigation_standoff_m"] == pytest.approx(0.35)
    assert parameters["spray_range_m"] == pytest.approx(0.40)


def test_hardware_wrapper_exposes_only_distance_overrides():
    wrapper = (
        Path(__file__).parents[3] / "scripts" / "vla_hardware_e2e.sh"
    ).read_text(encoding="utf-8")

    assert '${VLA_NAVIGATION_STANDOFF_M:-0.15}' in wrapper
    assert '${VLA_SPRAY_RANGE_M:-0.30}' in wrapper
    assert "navigation_standoff_m:=$NAVIGATION_STANDOFF_M" in wrapper
    assert "spray_range_m:=$SPRAY_RANGE_M" in wrapper
    assert 'VLA_PARAMETERS: navigation_standoff_m=' in wrapper
