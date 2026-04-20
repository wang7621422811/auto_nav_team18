"""
sim_bringup.launch.py  —  Pioneer 3-AT simulation  (Step G2)

Starts:
  1. gz sim      Gazebo with GUI (Ogre2 renderer)
  2. robot_state_publisher  — static TF from URDF
  3. ros_gz_bridge  — /cmd_vel /odom /tf /joint_states /clock /scan
  4. ros_gz_sim create  — spawns the robot (delayed 4 s)
  5. rviz2  — LaserScan, RobotModel, Odometry, TF (delayed 6 s)

Teleop (separate terminal):
  source install/setup.bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

Verify:
  ros2 topic echo /scan --once
  ros2 topic hz /scan
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_RVIZ_CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rviz_config.rviz')

# ── Paths ──────────────────────────────────────────────────────────────────────
_THIS_DIR  = os.path.dirname(os.path.abspath(__file__))
_SIM_DIR   = os.path.dirname(_THIS_DIR)

_WORLD_PATH  = os.path.join(_SIM_DIR, 'sim_worlds',            'pioneer3at_slalom.world')
_URDF_PATH   = os.path.join(_SIM_DIR, 'sim_robot_description', 'pioneer.urdf')
_BRIDGE_YAML = os.path.join(_THIS_DIR, 'bridge.yaml')


def _load_urdf() -> str:
    with open(_URDF_PATH, 'r') as fh:
        content = fh.read()
    # Replace package:// with file:// so Gazebo finds meshes without colcon install
    return content.replace(
        'package://auto_nav/simulation/',
        f'file://{_SIM_DIR}/',
    )


def generate_launch_description() -> LaunchDescription:

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    declare_x   = DeclareLaunchArgument('x',   default_value='0.0')
    declare_y   = DeclareLaunchArgument('y',   default_value='-8.0')
    declare_z   = DeclareLaunchArgument('z',   default_value='0.25')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='1.5707963')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # 1. Gazebo with GUI (-r auto-runs; no -s so the GUI window opens)
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', _WORLD_PATH],
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

    # 3. ROS-GZ bridge  (/cmd_vel /odom /tf /joint_states /clock)
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
                    '-name',  'pioneer3at',
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

    # 5. RViz2 (delayed 6 s — robot must be spawned and /scan bridged first)
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
        declare_sim_time, declare_x, declare_y, declare_z, declare_yaw,
        gazebo,
        robot_state_pub,
        bridge,
        spawn_robot,
        rviz,
    ])
