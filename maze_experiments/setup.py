from glob import glob

from setuptools import find_packages, setup

package_name = 'maze_experiments'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config/robots', glob('config/robots/*.yaml')),
        ('share/' + package_name + '/config/recording', glob('config/recording/*.yaml')),
        ('share/' + package_name + '/config/examples', glob('config/examples/*.yaml')),
        ('share/' + package_name + '/docs', glob('docs/*.md')),
        # "maze_experiments" on the PATH (colcon adds <prefix>/bin), in addition to
        # "ros2 run maze_experiments maze_experiments".
        ('bin', ['scripts/maze_experiments']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rabbynun',
    maintainer_email='rabbynun@todo.todo',
    description='Experiment suite for the maze robot.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'maze_experiments = maze_experiments.main:main',
        ],
    },
)
