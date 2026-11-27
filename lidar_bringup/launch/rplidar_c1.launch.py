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
import os

import launch.logging
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from lidar_bringup.usb_attach import DEFAULT_VID_PID, LidarPortError, ensure_lidar_port


def _is_wsl() -> bool:
    try:
        with open('/proc/sys/kernel/osrelease') as f:
            return 'microsoft' in f.read().lower()
    except OSError:
        return False


def serial_port_problem(port: str):
    """Return a human-readable explanation if `port` cannot be used, else None.

    sllidar_node does not report this itself: when it cannot open the port it fails later
    with the misleading "Error, unexpected error, code: 80008004" and the motor never starts.
    """
    found = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
    found_txt = ', '.join(found) if found else 'none'

    if not os.path.exists(port):
        if os.path.islink(port):
            what = f"'{port}' points to '{os.readlink(port)}', which no longer exists"
        else:
            what = f"'{port}' does not exist"
        lines = [f'Serial port problem: {what}. USB serial devices present: {found_txt}.']
        if not found and _is_wsl():
            lines.append(
                'This is WSL: launch with auto_attach:=true (default) or attach it from Windows PowerShell with '
                "'usbipd list' then 'usbipd attach --wsl --busid <BUSID>' (again after every "
                'replug or WSL restart). See lidar_bringup/README.md section 1b.')
        elif not found:
            lines.append('Check the USB cable, then dmesg | tail for a cp210x message.')
        elif port == '/dev/rplidar':
            lines.append(
                'The udev rule that creates /dev/rplidar is missing or has not been applied: run '
                'lidar_bringup/scripts/install_udev.sh, then replug the LiDAR (WSL: usbipd '
                f'detach + attach). To test right away: serial_port:={found[0]}')
        return '\n'.join(lines)

    if not os.access(port, os.R_OK | os.W_OK):
        real = os.path.realpath(port)
        return (f"Serial port problem: no read/write permission on '{real}'. Run "
                'lidar_bringup/scripts/install_udev.sh (sets mode 0666) and replug the LiDAR, '
                "or add yourself to 'dialout' (sudo usermod -aG dialout $USER) and log in again.")
    return None


def _truthy(value: str) -> bool:
    """Launch arguments arrive as strings; accept the usual spellings of 'true'."""
    return value.strip().lower() in ('true', '1', 'yes', 'on')


def _lidar_driver(context):
    """Get the LiDAR's USB port ready, check it, then build the sllidar_node action.

    Runs at launch time (OpaqueFunction) because the serial port is only known after the USB
    device has been found/attached. Steps:
      1. auto_attach: find the LiDAR by USB ID and, on WSL, hand it to Ubuntu via usbipd
         (no-op on the Raspberry Pi). Returns the port path to use.
      2. serial_port_problem(): last sanity check (exists, readable) with a clear message.
      3. Start sllidar_node on the resolved port.
    On any failure we log what to fix and shut the launch down instead of letting the driver
    loop on its misleading error 80008004.
    """
    logger = launch.logging.get_logger('lidar_bringup')
    cfg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    port = cfg('serial_port')

    if _truthy(cfg('auto_attach')):
        try:
            port = ensure_lidar_port(
                vid_pid=cfg('lidar_vid_pid'),
                busid=cfg('lidar_busid') or None,
                description_contains=cfg('lidar_description') or None,
                auto_bind=_truthy(cfg('auto_bind')),
                timeout=float(cfg('attach_timeout')),
                port_link=port,          # normally /dev/rplidar; falls back to the lone ttyUSB
                log=logger.info)
        except LidarPortError as exc:
            logger.error(str(exc))
            return [Shutdown(reason='RPLIDAR USB attach failed')]

    problem = serial_port_problem(port)
    if problem is not None:
        logger.error(problem)
        return [Shutdown(reason='RPLIDAR serial port is not usable')]

    # Later entries in `parameters` override earlier ones, so the resolved port and CLI args
    # beat the YAML. The port is a plain string here because it was resolved above.
    return [Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'serial_port': port,
             'frame_id': LaunchConfiguration('frame_id')},
        ],
        # Restart if the USB link glitches (brown-out, cable wiggle).
        respawn=True,
        respawn_delay=2.0,
    )]


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
        # The C1's published 0 deg points OPPOSITE the arrow on its housing (measured
        # 2026-11-26 with scripts/find_front.py: flat board at the arrow read -178.7 deg).
        # yaw = pi makes base_link forward (+x) = the arrow side. Set 0.0 if the LiDAR
        # is mounted with the arrow pointing backwards.
        DeclareLaunchArgument('lidar_yaw', default_value='3.14159',
                              description='LiDAR yaw on the robot (rad); pi = arrow forward'),
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

    # Attach + check the serial port, then start the driver (see _lidar_driver).
    driver = OpaqueFunction(function=_lidar_driver)

    return LaunchDescription(args + [driver, static_tf, monitor, rviz])
