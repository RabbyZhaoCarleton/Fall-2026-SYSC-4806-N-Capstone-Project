from glob import glob

from setuptools import find_packages, setup

package_name = 'maze_planning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/scenarios', glob('config/scenarios/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/layouts', glob('layouts/*.txt')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rabbynun',
    maintainer_email='rabbynun@todo.todo',
    description='Global path planning: the most time-efficient order to deliver the objects to their exits.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_planner = maze_planning.ros.global_planner_node:main',
            'mock_task_manager = maze_planning.ros.mock_task_manager_node:main',
            'fake_world = maze_planning.ros.fake_world_node:main',
            'plan_offline = maze_planning.tools.plan_offline:main',
        ],
    },
)
