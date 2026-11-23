"""Launch the RPLIDAR C1 (sllidar_ros2) on ROS 2 Jazzy.

Starts:
  1. sllidar_node            -> publishes /scan (sensor_msgs/LaserScan, frame laser_frame)
  2. static_transform_publisher base_link -> laser_frame (edit mounting offsets via args)
  3. (optional) scan_monitor -> rate / latency / jitter reporting
  4. (optional) rviz2        -> preconfigured for /scan (Best Effort QoS)

Examples:
  ros2 launch lidar_bringup rplidar_c1.launch.py
  ros2 launch lidar_bringup rplidar_c1.launch.py use_rviz:=true
  ros2 launch lidar_bringup rplidar_c1.launch.py serial_port:=/dev/ttyUSB0 lidar_x:=0.10 lidar_z:=0.15
  ros2 launch lidar_bringup rplidar_c1.launch.py auto_attach:=false     # skip the usbipd step
  ros2 launch lidar_bringup rplidar_c1.launch.py lidar_busid:=1-1       # force a BUSID (WSL)

On WSL the LiDAR's USB device is found and attached automatically (see usb_attach.py), so the
manual 'usbipd list' / 'usbipd attach --wsl --busid X' steps are no longer needed.
"""
import glob
import glob
import os

import launch.logging
import launch.logging
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('lidar_bringup')
    default_params = os.path.join(pkg_share, 'config', 'rplidar_c1.yaml')
    default_rviz = os.path.join(pkg_share, 'config', 'rviz', 'lidar.rviz')

    # ---- Launch arguments (all overridable from the CLI) -------------------
    args = [
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='YAML with sllidar_node parameters'),
        DeclareLaunchArgument('serial_port', default_value='/dev/rplidar',
                              description='Persistent udev symlink (see udev/99-rplidar.rules)'),
        # --- Automatic USB detection / attach (WSL; harmless no-op wait on the Pi) -----
        DeclareLaunchArgument('auto_attach', default_value='true',
                              description='Find the LiDAR by USB ID and attach it to WSL via usbipd'),
        DeclareLaunchArgument('lidar_vid_pid', default_value=DEFAULT_VID_PID,
                              description='USB VID:PID of the LiDAR adapter (CP210x)'),
        DeclareLaunchArgument('lidar_busid', default_value='',
                              description='Force a usbipd BUSID (e.g. 1-1) if several devices match'),
        DeclareLaunchArgument('lidar_description', default_value='',
                              description='Only match devices whose description contains this text'),
        DeclareLaunchArgument('auto_bind', default_value='true',
                              description='Allow a one-time UAC prompt to run "usbipd bind"'),
        DeclareLaunchArgument('attach_timeout', default_value='15',
                              description='Seconds to wait for the serial port after attaching'),
        DeclareLaunchArgument('frame_id', default_value='laser_frame'),
        DeclareLaunchArgument('parent_frame', default_value='base_link'),
        # LiDAR mounting pose relative to base_link (metres / radians). MEASURE THESE
        # on the real chassis - SLAM accuracy depends on it.
        DeclareLaunchArgument('lidar_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.10'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_monitor', default_value='true',
                              description='Run the scan performance monitor'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Open RViz2 (do NOT enable on the Pi)'),
    ]

    # ---- Static TF: base_link -> laser_frame -------------------------------
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            '--x', LaunchConfiguration('lidar_x'),
            '--y', LaunchConfiguration('lidar_y'),
            '--z', LaunchConfiguration('lidar_z'),
            '--yaw', LaunchConfiguration('lidar_yaw'),
            '--pitch', '0', '--roll', '0',
            '--frame-id', LaunchConfiguration('parent_frame'),
            '--child-frame-id', LaunchConfiguration('frame_id'),
        ],
    )

    monitor = Node(
        package='lidar_bringup',
        executable='scan_monitor',
        name='scan_monitor',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_monitor')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription(args + [lidar_node, static_tf, monitor, rviz])
