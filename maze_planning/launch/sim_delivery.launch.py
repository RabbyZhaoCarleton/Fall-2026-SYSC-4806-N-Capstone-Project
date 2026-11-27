"""Full delivery simulation in Gazebo: explore the maze, then deliver the objects.

    ros2 launch maze_planning sim_delivery.launch.py
    ros2 launch maze_planning sim_delivery.launch.py maze:=default_maze gui:=true

This starts maze_sim's simulation (Gazebo, the TurtleBot3, slam_toolbox, Nav2 and
explore_lite, exactly as `ros2 launch maze_sim maze.launch.py explore:=true`), plus:

    global_planner      the global planner (sim time)
    mock_task_manager   waits for explore_lite to return to the start, scans the
                        objects (from the scenario file), names the exits explore_lite
                        found, then runs the plan with Nav2, trip by trip

Mazes: maze:=three_exits_3m (worlds/three_exits_3m.sdf, three exits; default) or
maze:=default_maze (maze_sim's maze.sdf, two exits). world:= and scenario:= override.
The objects are not in the Gazebo world: they exist only in the scenario file and
show in RViz as yellow cubes.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    share = get_package_share_directory('maze_planning')
    maze_sim = get_package_share_directory('maze_sim')
    maze = LaunchConfiguration('maze').perform(context)
    world = LaunchConfiguration('world').perform(context)
    if not world:
        world = (os.path.join(maze_sim, 'worlds', 'maze.sdf') if maze == 'default_maze'
                 else os.path.join(share, 'worlds', maze + '.sdf'))
    scenario = LaunchConfiguration('scenario').perform(context) or \
        os.path.join(share, 'config', 'scenarios', maze + '.yaml')
    for f in (world, scenario):
        if not os.path.isfile(f):
            raise RuntimeError(f'File not found: {f}')
    with open(scenario) as f:
        start = yaml.safe_load(f).get('start', {'x': 0.25, 'y': -0.25})
    capacity = int(LaunchConfiguration('capacity').perform(context))

    passthrough = {name: LaunchConfiguration(name).perform(context)
                   for name in ('gui', 'rviz', 'monitor')}
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(maze_sim, 'launch', 'maze.launch.py')),
            launch_arguments={
                'world': world,
                'x_pose': str(start['x']),
                'y_pose': str(start['y']),
                'explore': 'true',
                'rviz_config': os.path.join(share, 'rviz', 'delivery.rviz'),
                **passthrough,
            }.items(),
        ),
        Node(package='maze_planning', executable='global_planner', name='global_planner',
             output='screen',
             parameters=[os.path.join(share, 'config', 'global_planner.yaml'),
                         {'use_sim_time': True, 'capacity': capacity}]),
        Node(package='maze_planning', executable='mock_task_manager', name='mock_task_manager',
             output='screen',
             parameters=[{'use_sim_time': True, 'scenario_file': scenario,
                          'exits_source': 'explore', 'start_on': 'explore_done'}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('maze', default_value='three_exits_3m',
                              description='three_exits_3m or default_maze'),
        DeclareLaunchArgument('world', default_value='', description='World file (overrides maze)'),
        DeclareLaunchArgument('scenario', default_value='',
                              description='Objects / classes / exits file (overrides maze)'),
        DeclareLaunchArgument('capacity', default_value='1',
                              description='Objects the robot carries at once'),
        DeclareLaunchArgument('gui', default_value='false', description='Open the Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('monitor', default_value='false',
                              description='Open the maze monitor (maze_sim)'),
        OpaqueFunction(function=nodes),
    ])
