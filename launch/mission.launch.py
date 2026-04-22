"""
mission.launch.py — Step 5: complete mission state machine.

Brings up all nodes required for a full autonomous mission:

  ┌── Teleop safety layer (assumed already running via teleop.launch.py) ──┐
  │  gamepad_watchdog · joy_mapper · mode_manager · cmd_gate               │
  └──────────────────────────────────────────────────────────────────────── ┘
  ┌── Navigation layer (assumed already running via navigation.launch.py) ─┐
  │  home_pose_recorder · path_follower · obstacle_guard                   │
  │  gap_planner · weave_planner · local_planner                           │
  └──────────────────────────────────────────────────────────────────────── ┘
  ┌── Perception layer (assumed already running via perception.launch.py) ─┐
  │  cone_detector · object_detector · distance_estimator · photo_capture  │
  └──────────────────────────────────────────────────────────────────────── ┘

  ─ Nodes started by THIS file ─────────────────────────────────────────────
  mission_controller   — top-level FSM (/mission/state, /journey/event,
                         /mission/hold, /mission/search_active)
  journey_logger       — persists /journey/event to JSONL file
  summary_generator    — writes YAML summary on COMPLETE / ABORTED

CLI arguments
-------------
  use_sim_time:=false
  use_gps:=false

Typical usage (all layers together)
------------------------------------
  ros2 launch auto_nav bringup.launch.py
  ros2 launch auto_nav teleop.launch.py
  ros2 launch auto_nav navigation.launch.py
  ros2 launch auto_nav perception.launch.py
  ros2 launch auto_nav mission.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch_ros.actions import Node


def _cfg(filename: str) -> str:
    return os.path.join(
        get_package_share_directory('auto_nav'), 'config', filename
    )


def generate_launch_description() -> LaunchDescription:

    args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'use_gps', default_value='false',
            description='GPS outdoor mode: remap mission odom consumers to /nav/odom',
        ),
    ]

    def _mission_nodes(context, *args, **kwargs):
        use_gps = context.launch_configurations.get('use_gps', 'false').lower() == 'true'
        use_sim_time = context.launch_configurations.get('use_sim_time', 'false') == 'true'
        odom_remaps = [('/odom', '/nav/odom')] if use_gps else []

        if use_gps:
            LogInfo(msg='[mission] GPS mode: remapping mission odometry to /nav/odom').execute(context)

        # ── mission_controller ─────────────────────────────────────────────
        mission_controller_node = Node(
            package='auto_nav',
            executable='mission_controller',
            name='mission_controller',
            output='screen',
            parameters=[
                _cfg('mission.yaml'),
                {'use_sim_time': use_sim_time},
            ],
            remappings=odom_remaps,
        )

        # ── journey_logger ─────────────────────────────────────────────────
        journey_logger_node = Node(
            package='auto_nav',
            executable='journey_logger',
            name='journey_logger',
            output='screen',
            parameters=[
                _cfg('mission.yaml'),
                {'use_sim_time': use_sim_time},
            ],
        )

        # ── summary_generator ──────────────────────────────────────────────
        summary_generator_node = Node(
            package='auto_nav',
            executable='summary_generator',
            name='summary_generator',
            output='screen',
            parameters=[
                _cfg('mission.yaml'),
                {'use_sim_time': use_sim_time},
            ],
        )

        return [
            mission_controller_node,
            journey_logger_node,
            summary_generator_node,
        ]

    return LaunchDescription(
        args + [
            LogInfo(msg='=== auto_nav Step 5: Mission Controller state machine ==='),
            OpaqueFunction(function=_mission_nodes),
        ]
    )
