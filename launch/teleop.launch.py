"""
teleop.launch.py — Step 1: full gamepad safety control layer.

This launch file is self-contained for manual-drive validation: start it and
the full joystick-to-safety chain comes up without needing bringup.launch.py.
That matches the Step-1 test goal of validating forward drive, turning, and
dead-man stop from a single entrypoint.

Nodes started here:
  joy_node          — gamepad → /joy
  teleop_twist_joy  — /joy axes → /cmd_vel_manual
  joy_mapper        — /joy buttons → /joy/btn_* semantic topics
  gamepad_watchdog  — /deadman_ok  /joy_connected
  mode_manager      — /control_mode  /emergency_stop
  cmd_gate          — safety arbiter → /cmd_vel_safe (chassis input)

CLI arguments:
  gamepad:=ps4          (default) or switch_pro  ← pick your controller
  joy_dev:=/dev/input/js0
  use_sim_time:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context, *args, **kwargs):
    """Resolve gamepad profile at launch time and create all nodes."""
    gamepad      = context.launch_configurations.get('gamepad', 'ps4')
    joy_dev      = context.launch_configurations.get('joy_dev', '/dev/input/js0')
    use_sim_time = context.launch_configurations.get('use_sim_time', 'false')

    cfg = os.path.join(
        get_package_share_directory('auto_nav'),
        'config', 'gamepad', f'{gamepad}.yaml',
    )

    if not os.path.isfile(cfg):
        raise FileNotFoundError(
            f"[teleop] Unknown gamepad profile '{gamepad}'. "
            f"Expected file: {cfg}. "
            "Available: ps4, switch_pro"
        )

    sim_params = {'use_sim_time': use_sim_time == 'true'}

    # ---- joy_node: gamepad driver → /joy ---------------------------------
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        output='screen',
        parameters=[cfg, {'dev': joy_dev, **sim_params}],
    )

    # ---- teleop_twist_joy: /joy axes → /cmd_vel_manual --------------------
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[cfg, sim_params],
        remappings=[('/cmd_vel', '/cmd_vel_manual')],
    )

    # ---- joy_mapper: /joy buttons → semantic Bool topics ------------------
    joy_mapper_node = Node(
        package='auto_nav',
        executable='joy_mapper',
        name='joy_mapper',
        output='screen',
        parameters=[cfg, sim_params],
    )

    # ---- gamepad_watchdog: monitors /joy timestamps ----------------------
    watchdog_node = Node(
        package='auto_nav',
        executable='gamepad_watchdog',
        name='gamepad_watchdog',
        output='screen',
        parameters=[cfg, sim_params],
    )

    # ---- mode_manager: state machine → /control_mode ----------------------
    mode_manager_node = Node(
        package='auto_nav',
        executable='mode_manager',
        name='mode_manager',
        output='screen',
        parameters=[sim_params],
    )

    # ---- cmd_gate: safety arbiter → /cmd_vel_safe -------------------------
    cmd_gate_node = Node(
        package='auto_nav',
        executable='cmd_gate',
        name='cmd_gate',
        output='screen',
        parameters=[cfg, sim_params],
    )

    return [
        LogInfo(msg=f'[teleop] gamepad profile: {gamepad}  ({cfg})'),
        joy_node,
        teleop_node,
        joy_mapper_node,
        watchdog_node,
        mode_manager_node,
        cmd_gate_node,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'gamepad', default_value='ps4',
            description='Gamepad profile: ps4 | switch_pro',
        ),
        DeclareLaunchArgument(
            'joy_dev', default_value='/dev/input/js0',
            description='Joystick device path',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock',
        ),
        LogInfo(msg='=== auto_nav Step 1: gamepad safety control ==='),
        OpaqueFunction(function=_nodes),
    ])
