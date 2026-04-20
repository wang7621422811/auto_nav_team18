"""
teleop.launch.py — Step 1: full gamepad safety control layer.

Assumes bringup.launch.py is already running (/joy live, chassis live).

Nodes started here:
  joy_node          — gamepad → /joy  (skipped if already running from bringup)
  teleop_twist_joy  — /joy → /cmd_vel_manual  (axis mapping from joystick.yaml)
  joy_mapper        — /joy → /joy/btn_* semantic topics
  gamepad_watchdog  — /deadman_ok  /joy_connected
  mode_manager      — /control_mode  /emergency_stop
  cmd_gate          — arbitrates all sources → /cmd_vel_safe (chassis input)

CLI arguments:
  joy_dev:=/dev/input/js0
  use_sim_time:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _cfg(filename: str) -> str:
    return os.path.join(
        get_package_share_directory('auto_nav'), 'config', filename
    )


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument(
            'joy_dev', default_value='/dev/input/js0',
            description='Joystick device path',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock',
        ),
    ]

    joy_dev      = LaunchConfiguration('joy_dev')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ---- joy_node: gamepad driver → /joy ----------------------------------
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        output='screen',
        parameters=[
            _cfg('joystick.yaml'),
            {'dev': joy_dev, 'use_sim_time': use_sim_time},
        ],
    )

    # ---- teleop_twist_joy: /joy axes → /cmd_vel_manual --------------------
    # Reuses the upstream package; axis/scale params come from joystick.yaml.
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[
            _cfg('joystick.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel_manual'),
        ],
    )

    # ---- joy_mapper: /joy buttons → semantic Bool topics ------------------
    joy_mapper_node = Node(
        package='auto_nav',
        executable='joy_mapper',
        name='joy_mapper',
        output='screen',
        parameters=[
            _cfg('joystick.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    # ---- gamepad_watchdog: monitors /joy timestamps -----------------------
    watchdog_node = Node(
        package='auto_nav',
        executable='gamepad_watchdog',
        name='gamepad_watchdog',
        output='screen',
        parameters=[
            _cfg('joystick.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    # ---- mode_manager: X/O state machine → /control_mode -----------------
    mode_manager_node = Node(
        package='auto_nav',
        executable='mode_manager',
        name='mode_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ---- cmd_gate: safety arbiter → /cmd_vel_safe -------------------------
    cmd_gate_node = Node(
        package='auto_nav',
        executable='cmd_gate',
        name='cmd_gate',
        output='screen',
        parameters=[
            _cfg('joystick.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription(
        args + [
            LogInfo(msg='=== auto_nav Step 1: gamepad safety control ==='),
            joy_node,
            teleop_node,
            joy_mapper_node,
            watchdog_node,
            mode_manager_node,
            cmd_gate_node,
        ]
    )
