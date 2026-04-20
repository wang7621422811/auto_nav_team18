"""
teleop.launch.py — manual-drive layer (Step 0).

Assumes bringup.launch.py is already running (/joy live, chassis live).

Starts:
  joy_node          — if not already running from bringup (idempotent name guard)
  teleop_twist_joy  — converts /joy → /cmd_vel_manual  (joy → twist)
  gamepad_watchdog  — /deadman_ok / /joy_connected

Note: /cmd_vel_manual is NOT sent to the chassis yet.
      In Step 1, cmd_gate_node will route manual/auto through safety gate.

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

    # teleop_twist_joy: reads /joy, publishes to /cmd_vel_manual
    # Config keys follow the teleop_twist_joy parameter schema.
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[
            _cfg('joystick.yaml'),
            {
                # Map our param names to teleop_twist_joy conventions
                'axis_linear.x':        LaunchConfiguration('joy_dev'),   # re-used arg
                'use_sim_time':         use_sim_time,
                # Actual axes/buttons come from joystick.yaml loaded above
            },
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel_manual'),  # safety gate reads this in Step 1
        ],
    )

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

    return LaunchDescription(
        args + [
            LogInfo(msg='=== auto_nav teleop layer ==='),
            joy_node,
            teleop_node,
            watchdog_node,
        ]
    )
