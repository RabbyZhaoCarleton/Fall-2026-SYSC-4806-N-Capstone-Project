"""Lightweight delivery simulation: no Gazebo, no SLAM, no Nav2. Starts in seconds.

    ros2 launch maze_planning sim_lightweight.launch.py
    ros2 launch maze_planning sim_lightweight.launch.py maze:=default_maze capacity:=2 rviz:=false

Nodes:
    fake_world          publishes the explored maze (/map, /global_costmap/costmap), the
                        robot's TF, explore_lite's "done" and exits, and serves Nav2's
                        navigate_to_pose action with a simple kinematic robot
    global_planner      the real global planner, unchanged
    mock_task_manager   scans the objects, labels the exits, runs the plan trip by trip
    rviz2               (rviz:=true) shows the map, costmap, plan and robot

The maze comes from layouts/<maze>.txt and the objects from config/scenarios/<maze>.yaml;
both can be overridden with layout:= and scenario:=.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    share = get_package_share_directory('maze_planning')
    maze = LaunchConfiguration('maze').perform(context)
    layout = LaunchConfiguration('layout').perform(context) or \
        os.path.join(share, 'layouts', maze + '.txt')
    scenario = LaunchConfiguration('scenario').perform(context) or \
        os.path.join(share, 'config', 'scenarios', maze + '.yaml')
    for f in (layout, scenario):
        if not os.path.isfile(f):
            raise RuntimeError(f'File not found: {f}')
    with open(scenario) as f:
        start = yaml.safe_load(f).get('start', {'x': 0.25, 'y': -0.25, 'yaw': 0.0})
    capacity = int(LaunchConfiguration('capacity').perform(context))
    speed_scale = float(LaunchConfiguration('speed_scale').perform(context))

    return [
        Node(package='maze_planning', executable='fake_world', name='fake_world',
             output='screen',
             parameters=[{'layout_file': layout, 'start_x': float(start['x']),
                          'start_y': float(start['y']), 'start_yaw': float(start.get('yaw', 0.0)),
                          'speed_scale': speed_scale}]),
        Node(package='maze_planning', executable='global_planner', name='global_planner',
             output='screen',
             parameters=[os.path.join(share, 'config', 'global_planner.yaml'),
                         {'capacity': capacity}]),
        Node(package='maze_planning', executable='mock_task_manager', name='mock_task_manager',
             output='screen',
             parameters=[{'scenario_file': scenario, 'exits_source': 'explore',
                          'start_on': 'explore_done'}]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(share, 'rviz', 'delivery.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('maze', default_value='three_exits_3m',
                              description='Name of the layout / scenario pair (three_exits_3m, default_maze)'),
        DeclareLaunchArgument('layout', default_value='',
                              description='Maze drawing file (overrides maze)'),
        DeclareLaunchArgument('scenario', default_value='',
                              description='Objects / classes / exits file (overrides maze)'),
        DeclareLaunchArgument('capacity', default_value='1',
                              description='Objects the robot carries at once'),
        DeclareLaunchArgument('speed_scale', default_value='1.0',
                              description='Fake robot speed relative to the planner\'s time model'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        OpaqueFunction(function=nodes),
    ])
