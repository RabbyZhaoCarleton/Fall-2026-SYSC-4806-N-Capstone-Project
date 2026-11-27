import json
import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            Shutdown, TimerAction)
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
        ground_truth_bridge(world_path),
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


def ground_truth_bridge(world_path):
    """Bridge the simulator's true poses to /ground_truth/poses (geometry_msgs/PoseArray).

    Gazebo publishes the pose of every moving entity on
    /world/<world>/dynamic_pose/info, ordered by entity: the robot model first
    (its base_footprint in world coordinates), then its links relative to the
    model. PoseArray keeps the sim-time stamp; the entity names are lost in any
    ROS message type, so readers take poses[0] as the robot (one robot only).
    Odometry, the EKF and SLAM never see this topic. It is recorded so drift can
    be measured against the truth (tools/drift_report.py).
    """
    with open(world_path) as f:
        match = re.search(r'<world\s+name\s*=\s*"([^"]+)"', f.read())
    if not match:
        raise RuntimeError(f'No <world name="..."> in {world_path}')
    gz_topic = f'/world/{match.group(1)}/dynamic_pose/info'
    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ground_truth_bridge',
        arguments=[f'{gz_topic}@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V'],
        remappings=[(gz_topic, '/ground_truth/poses')],
        output='screen',
    )


def is_true(value):
    return value.strip().lower() in ('true', '1', 'yes')


# Seconds after launch. Nav2 takes about 7 s to activate, so explore_lite starts
# well after it is ready.
NAV2_START_DELAY = 5.0
EXPLORE_START_DELAY = 20.0


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
        actions.append(LogInfo(
            msg=f'Starting Nav2 in {NAV2_START_DELAY:g} s with: {nav_params}'))
        # Start Nav2 after Gazebo and SLAM instead of all at once. When ~20 processes
        # start together, a lifecycle reply from a Nav2 server can be lost while the
        # nodes are still discovering each other; the lifecycle manager then waits
        # forever and Nav2 never activates ("Waiting to connect to move_base nav2 server").
        actions.append(TimerAction(period=NAV2_START_DELAY, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav_params,
                'autostart': 'true',
            }.items(),
        )]))

    if explore:
        # explore_lite exits once the robot is back at its start (or could not get
        # there). With shutdown_when_done, everything else stops then too, which
        # also closes the run's recording.
        on_exit = None
        if is_true(LaunchConfiguration('shutdown_when_done').perform(context)):
            on_exit = [Shutdown(reason='explore_lite has ended navigation')]
        if not os.path.isfile(explore_params):
            raise RuntimeError(f'Explore params file not found: {explore_params}')
        actions.append(LogInfo(
            msg=f'Starting explore_lite in {EXPLORE_START_DELAY:g} s with: {explore_params}'))
        # Give Gazebo, SLAM and Nav2 time to come up before exploring.
        actions.append(TimerAction(period=EXPLORE_START_DELAY, actions=[Node(
            package='explore_lite',
            executable='explore',
            name='explore_node',
            output='screen',
            parameters=[explore_params, {'use_sim_time': is_true(use_sim_time)}],
            on_exit=on_exit,
        )]))
    return actions


def spawn_robot(context):
    """Spawn the TurtleBot3 and bridge its topics; with ekf:=true, also start the EKF.

    Without the EKF this is turtlebot3_gazebo's own spawn launch, where Gazebo's
    wheel odometry publishes odom -> base_footprint. With the EKF, that transform
    is left out of the bridge and robot_localization publishes it instead.
    """
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    x_pose = LaunchConfiguration('x_pose').perform(context)
    y_pose = LaunchConfiguration('y_pose').perform(context)
    if not is_true(LaunchConfiguration('ekf').perform(context)):
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')),
            launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
        )]

    model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    if model != 'burger':
        raise RuntimeError(
            f'ekf:=true has a bridge config for the burger only (TURTLEBOT3_MODEL={model}). '
            'Use ekf:=false or add a bridge config for this model.')
    ekf_params = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('ekf_params').perform(context)))
    if not os.path.isfile(ekf_params):
        raise RuntimeError(f'EKF params file not found: {ekf_params}')
    use_sim_time = is_true(LaunchConfiguration('use_sim_time').perform(context))
    maze_sim = get_package_share_directory('maze_sim')

    return [
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', model,
                       '-file', os.path.join(tb3_gazebo, 'models', 'turtlebot3_' + model,
                                             'model.sdf'),
                       '-x', x_pose, '-y', y_pose, '-z', '0.01'],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['--ros-args', '-p', 'config_file:=' + os.path.join(
                maze_sim, 'config', 'turtlebot3_burger_bridge.yaml')],
            output='screen',
        ),
        # Gazebo stamps IMU messages with its own frame name (model/link/sensor),
        # which is not in the robot's TF tree. Tie it to imu_link so the EKF can
        # use the IMU.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--frame-id', 'imu_link',
                       '--child-frame-id', f'{model}/imu_link/tb3_imu'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_params, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ]


MONITORED_ARGS = ('world', 'x_pose', 'y_pose', 'use_sim_time', 'slam', 'rviz', 'gui', 'nav',
                  'explore', 'shutdown_when_done', 'ekf', 'slam_params', 'nav_params',
                  'explore_params', 'ekf_params')


def start_monitor(context):
    """Start the maze_monitor GUI, labelled with the launch mode, if monitor is true.

    --attach makes the monitor record this one run. To start, stop and restart
    runs from the monitor instead, run it on its own (ros2 run maze_monitor
    maze_monitor); it then starts this launch file with monitor:=false.
    """
    if not is_true(LaunchConfiguration('monitor').perform(context)):
        return []
    values = {name: LaunchConfiguration(name).perform(context) for name in MONITORED_ARGS}
    if is_true(values['explore']):
        label = 'explore'
    elif is_true(values['nav']):
        label = 'nav'
    else:
        label = 'manual'
    arguments = ['--attach', '--label', label, '--launch-args', json.dumps(values)]
    if not is_true(LaunchConfiguration('monitor_record').perform(context)):
        arguments.append('--no-record')
    return [Node(
        package='maze_monitor',
        executable='maze_monitor',
        name='maze_monitor',
        output='screen',
        arguments=arguments,
    )]


def generate_launch_description():
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    launch_file_dir = os.path.join(tb3_gazebo, 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    slam_toolbox = get_package_share_directory('slam_toolbox')
    maze_sim = get_package_share_directory('maze_sim')
    maze_navigation = get_package_share_directory('maze_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
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
            'ekf', default_value='true',
            description='Get odom -> base_footprint from an EKF that combines wheel '
                        'odometry with the IMU gyro, instead of wheel odometry alone'),
        DeclareLaunchArgument(
            'ekf_params',
            default_value=os.path.join(maze_navigation, 'config', 'ekf_maze.yaml'),
            description='robot_localization EKF parameters file'),
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
            'shutdown_when_done', default_value='false',
            description='Stop everything (Gazebo, SLAM, Nav2, the recording) once '
                        'explore_lite ends navigation, or if it exits for any '
                        'other reason. Off: the rest keeps running to look at the map'),
        DeclareLaunchArgument(
            'nav_params',
            default_value=os.path.join(maze_navigation, 'config', 'nav2_maze.yaml'),
            description='Nav2 parameters file'),
        DeclareLaunchArgument(
            'explore_params',
            default_value=os.path.join(maze_navigation, 'config', 'explore_maze.yaml'),
            description='explore_lite parameters file'),
        DeclareLaunchArgument(
            'monitor', default_value='true',
            description='Open the maze monitor (live view, run recording, and replay)'),
        DeclareLaunchArgument(
            'monitor_record', default_value='true',
            description='Save the run to ~/maze_runs (only with monitor:=true)'),
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
    ld.add_action(OpaqueFunction(function=spawn_robot))
    ld.add_action(slam_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(OpaqueFunction(function=start_nav_and_explore))
    ld.add_action(OpaqueFunction(function=start_monitor))
    return ld
