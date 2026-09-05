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
    install_requires=["setuptools"],
    python_requires=">=3.10",
    zip_safe=True,
    maintainer="Fire VLA Team",
    maintainer_email="maintainer@example.com",
    description="Firefighter UI web server (map/camera/mission dashboard).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "firefighter_ui = fire_vla_core.ros.firefighter_ui_node:main",
        ],
    },
)
