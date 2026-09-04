import os
from glob import glob
from setuptools import setup

package_name = 'controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='1270161395@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_publisher = controller.odom_publisher_node:main',
            'init_pose = controller.init_pose:main',
            # rf2o 는 covariance 를 전부 0 으로 내고, 이 로봇은 라이다가 180도 돌아
            # 장착돼 있어 x,y 부호가 반대로 나옵니다. 릴레이가 둘 다 고쳐서
            # odom_rf2o_fixed 로 재발행합니다 — EKF 는 이쪽을 써야 합니다.
            'rf2o_covariance_relay = controller.rf2o_covariance_relay:main'
        ],
    },
)
