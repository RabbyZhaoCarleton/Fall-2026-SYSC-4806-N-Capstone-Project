import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lidar_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'rviz'), glob('config/rviz/*.rviz')),
        (os.path.join('share', package_name, 'udev'), glob('udev/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'scan_monitor = lidar_bringup.scan_monitor_node:main',
            'load_generator = lidar_bringup.load_generator:main',
            'mock_hardware_demo = lidar_bringup.mock_hardware:main',
            'validate_pinout = lidar_bringup.pinout:main',
        ],
    },
)
