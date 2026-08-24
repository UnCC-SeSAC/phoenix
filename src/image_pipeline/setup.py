from glob import glob
import os

from setuptools import setup

package_name = "image_pipeline"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "models"), glob("models/*.onnx")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="team4",
    maintainer_email="team4@example.com",
    description="태스크① RGB 전처리 + 태스크② 검출 3D 좌표 (base_link)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "preprocess_node = image_pipeline.preprocess_node:main",
            "fake_camera_node = image_pipeline.fake_camera_node:main",
            "fake_detection_node = image_pipeline.fake_detection_node:main",
            "yolo_node = image_pipeline.yolo_node:main",
            "detection_3d_node = image_pipeline.detection_3d_node:main",
            "grpc_image_server = image_pipeline.grpc_image_server:main",  # gRPC
            "grpc_bbox_viewer = image_pipeline.grpc_bbox_viewer:main",
        ],
    },
)
