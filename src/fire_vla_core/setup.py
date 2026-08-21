from setuptools import find_packages, setup

package_name = "fire_vla_core"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["tests"]),
    package_data={"fire_vla_core.web": ["index.html"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    python_requires=">=3.10",
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Fire VLA Team",
    maintainer_email="maintainer@example.com",
    description="VLA Brain subsystem core package",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "vla_orchestrator = fire_vla_core.ros.orchestrator_node:main",
            "mock_demo = fire_vla_core.mock_demo:main",
            "vla_demo_input = fire_vla_core.ros.demo_input_node:main",
            "vla_mock_slam = fire_vla_core.ros.mock_slam_node:main",
            "firefighter_ui = fire_vla_core.ros.firefighter_ui_node:main",
            "vla_perception_bridge = fire_vla_core.ros.perception_bridge_node:main",
            "vla_short_nav_preflight = fire_vla_core.short_nav_preflight:main",
            "qwen_inference_server = fire_vla_core.qwen_inference_server:main",
        ],
    },
)
