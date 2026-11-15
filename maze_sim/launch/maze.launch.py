import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start_gazebo_server(context):
    """Resolve the world path (expands ~ and relative paths) and start Gazebo."""
    # Refuse to start if an old Gazebo server is still running. A leftover server
    # publishes its own /clock, /odom and /scan, and the new robot may even be
    # spawned into it, which makes the robot stutter or ignore commands.
    leftover = subprocess.run(['pgrep', '-af', 'gz sim .*-s'],
                              capture_output=True, text=True).stdout.strip()
    if leftover:
        raise RuntimeError(
            'A Gazebo server is already running:\n' + leftover +
            '\nStop it first with:  pkill -f "gz sim"')

    world_arg = LaunchConfiguration('world').perform(context)
    world_path = os.path.abspath(os.path.expanduser(world_arg))

    if not os.path.isfile(world_path):
        raise RuntimeError(f'World file not found: {world_path}')

    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    return [
        LogInfo(msg=f'Loading world: {world_path}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': f'-r -s -v2 {world_path}',
                'on_exit_shutdown': 'true',
            }.items(),
        ),
    ]


def is_true(value):
    return value.strip().lower() in ('true', '1', 'yes')


def start_nav_and_explore(context):
    """Start Nav2 if nav or explore is true; start explore_lite if explore is true."""
    nav = is_true(LaunchConfiguration('nav').perform(context))
    explore = is_true(LaunchConfiguration('explore').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    nav_params = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('nav_params').perform(context)))
    explore_params = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('explore_params').perform(context)))

    actions = []
    if nav or explore:
        if not os.path.isfile(nav_params):
            raise RuntimeError(f'Nav2 params file not found: {nav_params}')
        nav2_bringup = get_package_share_directory('nav2_bringup')
        actions.append(LogInfo(msg=f'Starting Nav2 with: {nav_params}'))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav_params,
                'autostart': 'true',
            }.items(),
        ))

    if explore:
        if not os.path.isfile(explore_params):
            raise RuntimeError(f'Explore params file not found: {explore_params}')
        actions.append(LogInfo(msg=f'Starting explore_lite with: {explore_params}'))
        # Give Gazebo, SLAM and Nav2 time to come up before exploring.
        actions.append(TimerAction(period=15.0, actions=[Node(
            package='explore_lite',
            executable='explore',
            name='explore_node',
            output='screen',
            parameters=[explore_params, {'use_sim_time': is_true(use_sim_time)}],
        )]))
    return actions


def generate_launch_description():
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    launch_file_dir = os.path.join(tb3_gazebo, 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    slam_toolbox = get_package_share_directory('slam_toolbox')
    maze_sim = get_package_share_directory('maze_sim')
    maze_navigation = get_package_share_directory('maze_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    slam = LaunchConfiguration('slam')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    declare_args = [
        DeclareLaunchArgument(
            'world', default_value=os.path.join(maze_sim, 'worlds', 'maze.sdf'),
            description='Path to the SDF world file to load'),
        DeclareLaunchArgument(
            'x_pose', default_value='0.25',
            description='Robot spawn x position (m)'),
        DeclareLaunchArgument(
            'y_pose', default_value='-0.25',
            description='Robot spawn y position (m)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo clock'),
        DeclareLaunchArgument(
            'slam', default_value='true',
            description='Start slam_toolbox'),
        DeclareLaunchArgument(
            'slam_params',
            default_value=os.path.join(maze_navigation, 'config', 'slam_maze.yaml'),
            description='slam_toolbox parameters file'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz 2'),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(maze_sim, 'config', 'slam.rviz'),
            description='RViz config file (empty for the RViz default layout)'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Open the Gazebo window (the simulation runs either way)'),
        DeclareLaunchArgument(
            'nav', default_value='false',
            description='Start Nav2 (needed to send goals in RViz)'),
        DeclareLaunchArgument(
            'explore', default_value='false',
            description='Start Nav2 and explore_lite for autonomous mapping'),
        DeclareLaunchArgument(
            'nav_params',
            default_value=os.path.join(maze_navigation, 'config', 'nav2_maze.yaml'),
            description='Nav2 parameters file'),
        DeclareLaunchArgument(
            'explore_params',
            default_value=os.path.join(maze_navigation, 'config', 'explore_maze.yaml'),
            description='explore_lite parameters file'),
    ]

    # The Gazebo window is optional. If it crashes or is closed, the simulation,
    # SLAM and Nav2 keep running (on_exit_shutdown is false), and RViz shows everything.
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(tb3_gazebo, 'models'))

    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': LaunchConfiguration('slam_params'),
        }.items(),
        condition=IfCondition(slam),
    )

    def start_rviz(context):
        cfg = rviz_config.perform(context)
        args = ['-d', os.path.abspath(os.path.expanduser(cfg))] if cfg else []
        return [Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=args,
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz),
        )]

    rviz_cmd = OpaqueFunction(function=start_rviz)

    ld = LaunchDescription()
    for arg in declare_args:
        ld.add_action(arg)
    ld.add_action(set_env_vars_resources)
    ld.add_action(OpaqueFunction(function=start_gazebo_server))
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(slam_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(OpaqueFunction(function=start_nav_and_explore))
    return ld
