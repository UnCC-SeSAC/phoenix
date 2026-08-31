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
    description='PHM 잔차 수집 전용 최소 기동 패키지',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
