import importlib.util
import shlex
import sys
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.utilities import perform_substitutions


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
