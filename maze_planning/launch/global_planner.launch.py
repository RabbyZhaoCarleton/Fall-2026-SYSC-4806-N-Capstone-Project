"""Start the global planner on its own (the real robot, or with any other simulation).

    ros2 launch maze_planning global_planner.launch.py
    ros2 launch maze_planning global_planner.launch.py capacity:=2 use_sim_time:=true

It needs Nav2's global costmap, TF (map -> base_link), and a task manager that
publishes /delivery/objects and /delivery/exits (see docs/global-planner-guide.md).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('maze_planning')
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=os.path.join(share, 'config',
                                                                   'global_planner.yaml'),
                              description='global_planner parameters file'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use the simulation clock'),
        DeclareLaunchArgument('capacity', default_value='1',
                              description='Objects the robot carries at once'),
        Node(
            package='maze_planning',
            executable='global_planner',
            name='global_planner',
            output='screen',
            parameters=[LaunchConfiguration('params'), {
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'capacity': ParameterValue(LaunchConfiguration('capacity'), value_type=int),
            }],
        ),
    ])
