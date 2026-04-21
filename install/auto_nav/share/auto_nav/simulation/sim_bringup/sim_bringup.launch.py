"""
sim_bringup.launch.py  —  Pioneer 3-AT simulation  (Step G2)

Starts:
  1. gz sim      Gazebo with GUI (renderer selectable; default Ogre for Parallels)
  2. robot_state_publisher  — static TF from URDF
  3. ros_gz_bridge  — /cmd_vel_safe /odom /tf /joint_states /clock /scan
  4. ros_gz_sim create  — spawns the robot at the bottom-centre of the map
  5. teleop.launch.py  — gamepad safety chain (/joy → /cmd_vel_safe)
  6. rviz2  — LaserScan, RobotModel, Odometry, TF (delayed 6 s)

Verify:
  ros2 topic echo /scan --once
  ros2 topic echo /cmd_vel_safe --once
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_RVIZ_CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rviz_config.rviz')

# ── Paths ──────────────────────────────────────────────────────────────────────
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_SIM_DIR = os.path.dirname(_THIS_DIR)

_WORLD_PATH = os.path.join(_SIM_DIR, 'sim_worlds', 'new_world.sdf')
_URDF_PATH = os.path.join(_SIM_DIR, 'sim_robot_description', 'pioneer.urdf')
_BRIDGE_YAML = os.path.join(_THIS_DIR, 'bridge.yaml')
_TELEOP_LAUNCH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(_THIS_DIR))),
    'launch',
    'teleop.launch.py',
)


def _load_urdf() -> str:
    with open(_URDF_PATH, 'r', encoding='utf-8') as fh:
        content = fh.read()
    # Replace package:// with file:// so Gazebo finds meshes without colcon install
    return content.replace(
        'package://auto_nav/simulation/',
        f'file://{_SIM_DIR}/',
    )


def generate_launch_description() -> LaunchDescription:

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    declare_x = DeclareLaunchArgument('x', default_value='0.0')
    declare_y = DeclareLaunchArgument('y', default_value='-18.0')
    declare_z = DeclareLaunchArgument('z', default_value='0.25')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='1.5707963')
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=_WORLD_PATH,
        description='Absolute path to the Gazebo world/SDF file',
    )
    declare_render_engine = DeclareLaunchArgument(
        'render_engine',
        default_value='ogre',
        description='Gazebo render engine (ogre works well on Parallels)',
    )
    declare_gamepad = DeclareLaunchArgument(
        'gamepad',
        default_value='ps4',
        description='Gamepad profile forwarded to teleop.launch.py',
    )
    declare_joy_dev = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device path forwarded to teleop.launch.py',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    render_engine = LaunchConfiguration('render_engine')

    # 1. Gazebo with GUI (-r auto-runs; no -s so the GUI window opens)
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '--render-engine', render_engine, world],
        output='screen',
        additional_env={'GZ_SIM_RESOURCE_PATH': _SIM_DIR},
    )

    # 2. Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': _load_urdf(),
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # 3. ROS-GZ bridge  (/cmd_vel_safe /odom /tf /joint_states /clock)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[{
            'config_file': _BRIDGE_YAML,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # 4. Spawn robot (delayed 4 s)
    spawn_robot = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_pioneer3at',
                arguments=[
                    '-name', 'pioneer3at',
                    '-topic', 'robot_description',
                    '-x', LaunchConfiguration('x'),
                    '-y', LaunchConfiguration('y'),
                    '-z', LaunchConfiguration('z'),
                    '-Y', LaunchConfiguration('yaw'),
                ],
                output='screen',
            )
        ],
    )

    # 5. Teleop safety chain — manual drive uses the same /cmd_vel_safe API as AUTO
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_TELEOP_LAUNCH),
        launch_arguments={
            'gamepad': LaunchConfiguration('gamepad'),
            'joy_dev': LaunchConfiguration('joy_dev'),
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # 6. RViz2 (delayed 6 s — robot must be spawned and /scan bridged first)
    rviz = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', _RVIZ_CFG],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        declare_sim_time,
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        declare_world,
        declare_render_engine,
        declare_gamepad,
        declare_joy_dev,
        gazebo,
        robot_state_pub,
        bridge,
        spawn_robot,
        teleop,
        rviz,
    ])
