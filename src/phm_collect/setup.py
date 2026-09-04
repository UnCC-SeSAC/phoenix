import os
from glob import glob
from setuptools import setup

package_name = 'phm_collect'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rjh',
    maintainer_email='rjh2280@gmail.com',
    description='PHM 잔차 수집 + 실시간 건전성 감시',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        # 로봇 위에서 잔차 경보를 계산해 /phm/status (std_msgs/String 안의 JSON) 로
        # 냅니다. firefighter_ui 가 /vla/status 를 다루는 것과 같은 모양이라,
        # UI 노드는 이 값을 해석하지 않고 그대로 웹으로 넘길 수 있습니다.
        'phm_monitor = phm_collect.phm_monitor_node:main',
    ]},
)
