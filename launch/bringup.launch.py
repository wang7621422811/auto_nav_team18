"""
bringup.launch.py — Step 0: bring all hardware alive.

Launches (in order of dependency):
  1. ariaNode         — Pioneer 3-AT chassis driver  → /odom /cmd_vel
  2. odom_tf_broadcaster OR outdoor_pose_fuser
                       — Step-0 TF from /odom, or GPS-aligned /nav/odom + TF
  3. nmea_navsat_driver             — GPS serial driver → /fix          (optional)
  4. sick_scan_xd OR lakibeam_ros2 — LiDAR driver      → /scan
  5. oakd_camera                    — OAK-D V2 camera   → /camera/*   (optional)
  6. joy_node                       — Bluetooth gamepad → /joy
  7. static_transform_publisher ×2  — base_link→laser, base_link→camera_link
     (translation xyz read from config/robot.yaml at launch)
  8. gamepad_watchdog               — /joy_connected /deadman_ok

NOT here (belongs in navigation.launch.py):
  home_pose_recorder  — moved to avoid duplicate when navigation.launch.py runs

CLI arguments (all have defaults; override on command line):
  lidar_type:=sick | lakibeam
  serial_port:=/dev/ttyUSB0
  sick_ip:=192.168.0.1
  lakibeam_ip:=192.168.1.200
  use_camera:=true | false  (false skips oakd_camera; safe when camera not available)
  camera_mx_id:=          (empty = auto)
  use_sim_time:=false
  use_gps:=false
  use_nmea_gps:=false
  gps_port:=/dev/ttyACM0
  gps_baud:=9600
  gps_frame_id:=gps
  aria_pkg:=ariaNode
  aria_exec:=ariaNode     (confirmed via `ros2 pkg executables ariaNode` on pioneer1)
"""

import os

from auto_nav.config_params import load_ros_param_from_files
from auto_nav.robot_extrinsics import (
    load_sensor_xyz_from_files,
    static_transform_arguments,
)
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
            'use_gps', default_value='false',
            description='GPS outdoor mode: launch outdoor_pose_fuser instead of odom_tf_broadcaster',
        ),
        DeclareLaunchArgument(
            'use_nmea_gps', default_value='false',
            description='Launch nmea_navsat_driver for serial GPS fixes on /fix',
        ),
        DeclareLaunchArgument(
            'gps_port', default_value='/dev/ttyACM0',
            description='Serial device for the NMEA GPS receiver',
        ),
        DeclareLaunchArgument(
            'gps_baud', default_value='9600',
            description='Baud rate for the NMEA GPS receiver',
        ),
        DeclareLaunchArgument(
            'gps_frame_id', default_value='gps',
            description='frame_id published by nmea_navsat_driver',
        ),
        DeclareLaunchArgument(
            'joy_dev', default_value='/dev/input/js0',
            description='Joystick device path',
        ),
        DeclareLaunchArgument(
            'gamepad', default_value='ps4',
            description='Gamepad profile: ps4 | switch_pro',
        ),
        DeclareLaunchArgument(
            'aria_pkg',
            default_value='ariaNode',
            description='ROS 2 package name for Pioneer chassis (course stack)',
        ),
        DeclareLaunchArgument(
            'aria_exec',
            default_value='ariaNode',
            description='Executable inside aria_pkg; override if different on this machine',
        ),
        DeclareLaunchArgument(
            'use_camera', default_value='true',
            description='Launch OAK-D camera node (set false if camera hardware is not available)',
        ),
    ]

    # Convenience references
    lidar_type   = LaunchConfiguration('lidar_type')
    serial_port  = LaunchConfiguration('serial_port')
    sick_ip      = LaunchConfiguration('sick_ip')
    lakibeam_ip  = LaunchConfiguration('lakibeam_ip')
    camera_mx_id = LaunchConfiguration('camera_mx_id')
    use_sim_time = LaunchConfiguration('use_sim_time')
    aria_pkg     = LaunchConfiguration('aria_pkg')
    aria_exec    = LaunchConfiguration('aria_exec')

    # ---- 1. Pioneer chassis (ariaNode / configurable) --------------------
    # Doc CLI: ros2 run ariaNode ariaNode --rp /dev/ttyUSB0
    # "--rp" is a direct executable arg for the serial port.
    # ROS2 params (serial_port / port) are also passed via robot.yaml in case
    # the node reads them; actual param names confirmed via `ros2 param list`.
    aria_node = Node(
        package=aria_pkg,
        executable=aria_exec,
        name='aria_node',
        output='screen',
        arguments=['--rp', serial_port],
        parameters=[
            _cfg('robot.yaml'),
            _cfg('real.yaml'),
            {
                'serial_port': serial_port,
                'port': serial_port,          # alias — whichever name ariaNode uses
                'use_sim_time': use_sim_time,
            },
        ],
        remappings=[
            # ariaNode uses standard names: subscribes /cmd_vel, publishes /odom.
            # Remap its /cmd_vel input to /cmd_vel_safe so only the safety gate
            # (cmd_gate_node) can drive the chassis.
            ('/cmd_vel', '/cmd_vel_safe'),
        ],
    )

    # ---- 1b. Convert /odom pose to the standard odom -> base_link TF -----
    def _tf_nodes(context, *args, **kwargs):
        """Select the TF source: raw odom in bench mode, GPS fuser outdoors."""
        use_gps = context.launch_configurations.get('use_gps', 'false').lower() == 'true'
        sim_params = {
            'use_sim_time': context.launch_configurations.get('use_sim_time', 'false') == 'true'
        }
        if use_gps:
            LogInfo(
                msg='[bringup] GPS mode enabled: outdoor_pose_fuser publishes /nav/odom and TF'
            ).execute(context)
            return [
                Node(
                    package='auto_nav',
                    executable='outdoor_pose_fuser',
                    name='outdoor_pose_fuser',
                    output='screen',
                    parameters=[
                        _cfg('gps.yaml'),
                        sim_params,
                    ],
                )
            ]

        return [
            Node(
                package='auto_nav',
                executable='odom_tf_broadcaster',
                name='odom_tf_broadcaster',
                output='screen',
                parameters=[sim_params],
            )
        ]

    # ---- 2b. Lakibeam LiDAR -----------------------------------------------
    lakibeam_node = Node(
        package='lakibeam_ros2',
        executable='lakibeam_ros2_node',
        name='lakibeam_scan',
        output='screen',
        parameters=[
            _cfg('lidar.yaml'),
            _cfg('real.yaml'),
            {
                'hostname': lakibeam_ip,
                'frame_id': 'laser',
                'use_sim_time': use_sim_time,
            },
        ],
    )

    # ---- 3. OAK-D V2 camera (local DepthAI node) — optional -----------------

    # ---- 4. Joy + Gamepad watchdog (profile resolved via OpaqueFunction) ------
    # Actual node creation happens in _gamepad_nodes() below.

    # ---- 5. Static TF publishers (xyz from config/robot.yaml) ------------
    def _sensor_static_tf(context, *args, **kwargs):
        """Publish base_link→laser and base_link→camera_link from layered config."""
        cfg_paths = [_cfg('robot.yaml'), _cfg('real.yaml')]
        try:
            ex = load_sensor_xyz_from_files(cfg_paths)
        except (OSError, ValueError) as e:
            raise RuntimeError(
                f'[bringup] Failed to read laser/camera XYZ from {cfg_paths}: {e}'
            ) from e

        use_sim = context.launch_configurations.get('use_sim_time', 'false')
        sim_params = {'use_sim_time': use_sim == 'true'}

        laser_args = static_transform_arguments(ex['laser'], child='laser')
        cam_args = static_transform_arguments(ex['camera'], child='camera_link')

        return [
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='tf_base_to_laser',
                arguments=laser_args,
                parameters=[sim_params],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='tf_base_to_camera',
                arguments=cam_args,
                parameters=[sim_params],
            ),
        ]

    # ---- Assemble with lidar + gamepad resolved via OpaqueFunction ---------
    # home_pose_recorder is intentionally NOT here — it lives in navigation.launch.py
    # to avoid duplicate nodes when both launch files run together.
    def _sick_scan_topic() -> str:
        """Resolve the driver-native SICK LaserScan topic from layered config."""
        cfg_paths = [_cfg('lidar.yaml'), _cfg('real.yaml')]
        scanner_type = load_ros_param_from_files(
            cfg_paths,
            node_name='sick_scan',
            key='scanner_type',
        ).strip()
        if not scanner_type:
            raise ValueError('scanner_type for sick_scan cannot be empty')
        return f'/{scanner_type}/scan'

    def _lidar_nodes(context, *args, **kwargs):
        """Select LiDAR driver at launch time — avoids IfCondition string hack."""
        lt = context.launch_configurations.get('lidar_type', 'sick').strip().lower()
        if lt == 'lakibeam':
            LogInfo(msg='[bringup] Starting Lakibeam LiDAR driver').execute(context)
            return [lakibeam_node]
        else:
            sick_scan_topic = _sick_scan_topic()
            LogInfo(msg='[bringup] Starting SICK LiDAR driver (sick_scan_xd)').execute(context)
            return [
                Node(
                    package='sick_scan_xd',
                    executable='sick_generic_caller',
                    name='sick_scan',
                    output='screen',
                    parameters=[
                        _cfg('lidar.yaml'),
                        _cfg('real.yaml'),
                        {
                            'hostname': sick_ip,
                            'frame_id': 'laser',
                            'use_sim_time': use_sim_time,
                        },
                    ],
                    remappings=[
                        # Normalise the driver-native topic to the project-wide /scan API.
                        (sick_scan_topic, '/scan'),
                        ('scan', '/scan'),
                        # Keep vendor TF out of the project-wide TF tree. The course
                        # stack already defines odom -> base_link -> laser explicitly.
                        ('/tf', '/sick_scan/tf'),
                        ('/tf_static', '/sick_scan/tf_static'),
                        ('/cloud', '/scan_cloud'),
                    ],
                )
            ]

    def _camera_nodes(context, *args, **kwargs):
        """Only launch camera driver when use_camera:=true."""
        if context.launch_configurations.get('use_camera', 'true').lower() != 'true':
            LogInfo(msg='[bringup] Camera disabled (use_camera:=false)').execute(context)
            return []
        LogInfo(msg='[bringup] Starting OAK-D camera node').execute(context)
        cam_mx_id    = context.launch_configurations.get('camera_mx_id', '')
        use_sim_time = context.launch_configurations.get('use_sim_time', 'false')
        return [
            Node(
                package='auto_nav',
                executable='oakd_camera',
                name='camera',
                output='screen',
                parameters=[
                    _cfg('camera.yaml'),
                    _cfg('real.yaml'),
                    {
                        'mx_id': cam_mx_id,
                        'use_sim_time': use_sim_time == 'true',
                    },
                ],
            )
        ]

    def _gps_nodes(context, *args, **kwargs):
        """Optionally launch a serial NMEA GPS driver that publishes /fix."""
        if context.launch_configurations.get('use_nmea_gps', 'false').lower() != 'true':
            return []

        gps_port = context.launch_configurations.get('gps_port', '/dev/ttyACM0').strip()
        gps_baud_raw = context.launch_configurations.get('gps_baud', '9600').strip()
        gps_frame_id = context.launch_configurations.get('gps_frame_id', 'gps').strip()
        use_sim_time = context.launch_configurations.get('use_sim_time', 'false') == 'true'

        try:
            gps_baud = int(gps_baud_raw)
        except ValueError as exc:
            raise ValueError(f'[bringup] gps_baud must be an integer, got {gps_baud_raw!r}') from exc

        LogInfo(
            msg=f'[bringup] Starting NMEA GPS driver on {gps_port} @ {gps_baud} baud'
        ).execute(context)
        return [
            Node(
                package='nmea_navsat_driver',
                executable='nmea_serial_driver',
                name='nmea_navsat_driver',
                output='screen',
                parameters=[
                    {
                        'port': gps_port,
                        'baud': gps_baud,
                        'frame_id': gps_frame_id,
                        'use_sim_time': use_sim_time,
                    }
                ],
            )
        ]

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
            OpaqueFunction(function=_tf_nodes),
            OpaqueFunction(function=_gps_nodes),
            OpaqueFunction(function=_lidar_nodes),
            OpaqueFunction(function=_camera_nodes),
            OpaqueFunction(function=_gamepad_nodes),
            OpaqueFunction(function=_sensor_static_tf),
        ]
    )

    return ld
