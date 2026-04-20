"""
teleop.launch.py — Step 1: full gamepad safety control layer.

Assumes bringup.launch.py is already running (joy + watchdog + chassis live).
bringup already starts joy_node and gamepad_watchdog; this file adds the
software control stack on top.

Nodes started here:
  teleop_twist_joy  — /joy axes → /cmd_vel_manual
  joy_mapper        — /joy buttons → /joy/btn_* semantic topics
  mode_manager      — /control_mode  /emergency_stop
  cmd_gate          — safety arbiter → /cmd_vel_safe (chassis input)

NOT started here (already in bringup.launch.py):
  joy_node, gamepad_watchdog

CLI arguments:
  gamepad:=ps4          (default) or switch_pro  ← pick your controller
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

    # ---- teleop_twist_joy: /joy axes → /cmd_vel_manual --------------------
    # joy_node is started by bringup.launch.py; no need to start it again here.
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

    # ---- mode_manager: state machine → /control_mode ----------------------
    # gamepad_watchdog is started by bringup.launch.py; no need to start it again here.
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
        teleop_node,
        joy_mapper_node,
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
            'use_sim_time', default_value='false',
            description='Use simulation clock',
        ),
        LogInfo(msg='=== auto_nav Step 1: gamepad safety control ==='),
        OpaqueFunction(function=_nodes),
    ])
