import os
from glob import glob
from setuptools import find_packages, setup

package_name = "uncc_example"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "docs"),
            glob("docs/*"),
        ),
        (
            os.path.join("share", package_name, "scripts"),
            glob("scripts/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UNCC project",
    maintainer_email="user@example.com",
    description=(
        "Frontier exploration using SLAM, Nav2 global/local costmaps, "
        "and a semantic hazard map for Hiwonder on ROS 2 Humble."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "avoidance_manager = uncc_example.avoidance_manager:main",
            "state_manager = uncc_example.state_manager:main",
            "demo_state_manager = uncc_example.demo_state_manager:main",
            "demo_state_manager_2 = uncc_example.demo_state_manager_2:main",
            "mission_executor = uncc_example.mission_executor:main",
            "vision_detector = uncc_example.vision_detector:main",
            "fire_keepout_node = uncc_example.fire_keepout_node:main",
            "fire_suppression_node = uncc_example.fire_suppression_node:main",
            "mission_manager = uncc_example.mission_manager:main",
            "fire_suppression_node_dummy_stub = uncc_example.fire_suppression_node_dummy_stub:main",
            "fire_status_service_node = uncc_example.fire_status_service_node:main",
            "navigate_to_pose_dummy_stub = uncc_example.navigate_to_pose_dummy_stub:main",
            (
                "frontier_state_controller = "
                "uncc_example.frontier_state_controller:main"
            ),
        ],
    },
)
