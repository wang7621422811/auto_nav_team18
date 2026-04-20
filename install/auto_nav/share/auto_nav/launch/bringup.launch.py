"""
bringup.launch.py — Step 0: bring all hardware alive.

Launches (in order of dependency):
  1. ros2aria         — Pioneer 3-AT chassis driver  → /odom /cmd_vel
  2. sick_scan_xd OR lakibeam_ros2 — LiDAR driver    → /scan
  3. depthai_ros_driver             — OAK-D V2 camera → /camera/*
  4. joy_node                       — Bluetooth gamepad → /joy
  5. static_transform_publisher ×2  — odom→base_link already from aria;
                                       base_link→laser, base_link→camera_link
  6. gamepad_watchdog               — /joy_connected /deadman_ok
  7. home_pose_recorder             — /mission/home_pose, saves YAML

CLI arguments (all have defaults; override on command line):
  lidar_type:=sick | lakibeam
  serial_port:=/dev/ttyUSB0
  sick_ip:=192.168.0.1
  lakibeam_ip:=192.168.1.200
  camera_mx_id:=          (empty = auto)
  use_sim_time:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _pkg(name: str) -> str:
    return get_package_share_directory(name)


def _cfg(filename: str) -> str:
    return os.path.join(_pkg('auto_nav'), 'config', filename)


# ---------------------------------------------------------------------------
def generate_launch_description() -> LaunchDescription:

    # ---- Declare all overridable arguments --------------------------------
    args = [
        DeclareLaunchArgument(
            'lidar_type', default_value='sick',
            description='LiDAR driver to use: "sick" or "lakibeam"',
        ),
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyUSB0',
            description='Serial port for Pioneer chassis (e.g. /dev/ttyUSB0)',
        ),
        DeclareLaunchArgument(
            'sick_ip', default_value='192.168.0.1',
            description='IP address of SICK LiDAR',
        ),
        DeclareLaunchArgument(
            'lakibeam_ip', default_value='192.168.1.200',
            description='IP address of Lakibeam LiDAR',
        ),
        DeclareLaunchArgument(
            'camera_mx_id', default_value='',
            description='OAK-D MX device ID (empty = auto-detect)',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'joy_dev', default_value='/dev/input/js0',
            description='Joystick device path',
        ),
        DeclareLaunchArgument(
            'gamepad', default_value='ps4',
            description='Gamepad profile: ps4 | switch_pro',
        ),
    ]

    # Convenience references
    lidar_type   = LaunchConfiguration('lidar_type')
    serial_port  = LaunchConfiguration('serial_port')
    sick_ip      = LaunchConfiguration('sick_ip')
    lakibeam_ip  = LaunchConfiguration('lakibeam_ip')
    camera_mx_id = LaunchConfiguration('camera_mx_id')
    use_sim_time = LaunchConfiguration('use_sim_time')
    joy_dev      = LaunchConfiguration('joy_dev')

    # ---- 1. Pioneer chassis (ros2aria) ------------------------------------
    aria_node = Node(
        package='ros2aria',
        executable='ros2aria',
        name='aria_node',
        output='screen',
        parameters=[
            _cfg('robot.yaml'),
            {
                'port': serial_port,
                'use_sim_time': use_sim_time,
            },
        ],
        remappings=[
            # ros2aria publishes to /RosAria/pose — remap to standard /odom
            ('/RosAria/pose', '/odom'),
            ('/RosAria/cmd_vel', '/cmd_vel_safe'),
        ],
    )

    # ---- 2a. SICK LiDAR (sick_scan_xd) ------------------------------------
    # No condition here — selection is handled inside OpaqueFunction _lidar_nodes().
    sick_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_scan',
        output='screen',
        parameters=[
            _cfg('lidar.yaml'),
            {
                'hostname': sick_ip,
                'frame_id': 'laser',
                'use_sim_time': use_sim_time,
            },
        ],
        remappings=[('/cloud', '/scan_cloud')],
    )

    # ---- 2b. Lakibeam LiDAR -----------------------------------------------
    lakibeam_node = Node(
        package='lakibeam_ros2',
        executable='lakibeam_ros2_node',
        name='lakibeam_scan',
        output='screen',
        parameters=[
            _cfg('lidar.yaml'),
            {
                'hostname': lakibeam_ip,
                'frame_id': 'laser',
                'use_sim_time': use_sim_time,
            },
        ],
    )

    # ---- 3. OAK-D V2 camera (depthai_ros_driver) --------------------------
    camera_node = Node(
        package='depthai_ros_driver',
        executable='camera_node',
        name='camera',
        output='screen',
        parameters=[
            _cfg('camera.yaml'),
            {
                'i_mx_id': camera_mx_id,
                'use_sim_time': use_sim_time,
            },
        ],
        remappings=[
            ('~/color/image', '/camera/color/image_raw'),
            ('~/depth/image', '/camera/depth/image_raw'),
        ],
    )

    # ---- 4. Joy + Gamepad watchdog (profile resolved via OpaqueFunction) ------
    # Actual node creation happens in _gamepad_nodes() below.

    # ---- 5. Static TF publishers -------------------------------------------
    # base_link → laser
    tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_laser',
        arguments=[
            '0.20', '0.00', '0.18',   # x y z (metres) — matches robot.yaml
            '0',    '0',    '0',       # roll pitch yaw (radians)
            'base_link', 'laser',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # base_link → camera_link
    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_camera',
        arguments=[
            '0.25', '0.00', '0.22',
            '0',    '0',    '0',
            'base_link', 'camera_link',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ---- 7. Home pose recorder ---------------------------------------------
    home_pose_node = Node(
        package='auto_nav',
        executable='home_pose_recorder',
        name='home_pose_recorder',
        output='screen',
        parameters=[
            _cfg('mission.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    # ---- Assemble with lidar + gamepad resolved via OpaqueFunction ---------
    def _lidar_nodes(context, *args, **kwargs):
        """Select LiDAR driver at launch time — avoids IfCondition string hack."""
        lt = context.launch_configurations.get('lidar_type', 'sick').strip().lower()
        if lt == 'lakibeam':
            LogInfo(msg='[bringup] Starting Lakibeam LiDAR driver').execute(context)
            return [lakibeam_node]
        else:
            LogInfo(msg='[bringup] Starting SICK LiDAR driver (sick_scan_xd)').execute(context)
            return [sick_node]

    def _gamepad_nodes(context, *args, **kwargs):
        """Resolve gamepad profile and create joy + watchdog nodes."""
        gamepad      = context.launch_configurations.get('gamepad', 'ps4')
        joy_dev      = context.launch_configurations.get('joy_dev', '/dev/input/js0')
        use_sim_time = context.launch_configurations.get('use_sim_time', 'false')
        sim_params   = {'use_sim_time': use_sim_time == 'true'}

        cfg = os.path.join(
            get_package_share_directory('auto_nav'),
            'config', 'gamepad', f'{gamepad}.yaml',
        )
        if not os.path.isfile(cfg):
            raise FileNotFoundError(
                f"[bringup] Unknown gamepad profile '{gamepad}'. "
                f"Expected: {cfg}. Available: ps4, switch_pro"
            )

        LogInfo(msg=f'[bringup] gamepad profile: {gamepad}').execute(context)

        joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy',
            output='screen',
            parameters=[cfg, {'dev': joy_dev, **sim_params}],
        )
        watchdog_node = Node(
            package='auto_nav',
            executable='gamepad_watchdog',
            name='gamepad_watchdog',
            output='screen',
            parameters=[cfg, sim_params],
        )
        return [joy_node, watchdog_node]

    ld = LaunchDescription(
        args + [
            LogInfo(msg='=== auto_nav bringup: Step 0 ==='),
            aria_node,
            OpaqueFunction(function=_lidar_nodes),
            camera_node,
            OpaqueFunction(function=_gamepad_nodes),
            tf_base_to_laser,
            tf_base_to_camera,
            home_pose_node,
        ]
    )

    return ld
