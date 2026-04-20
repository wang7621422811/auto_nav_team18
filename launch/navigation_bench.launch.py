"""
navigation_bench.launch.py — indoor wheel-spin / dry-run waypoint validation.

Use this when the robot is lifted off the ground and you want to validate only
the waypoint sequence:
  HOME -> WP1 -> WP2 -> WP3 -> WP4 -> HOME

Compared with navigation.launch.py this bench entrypoint:
  - does not require LiDAR planners
  - does not require marker detection
  - records key progress events into JourneyLogger

Assumes teleop.launch.py is already running so /control_mode and /cmd_vel_safe
are available and AUTO still goes through the existing safety gate.
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
            'use_sim_time', default_value='false',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=_cfg('waypoints_bench_4.yaml'),
            description='Absolute path to the bench waypoint YAML file',
        ),
        DeclareLaunchArgument(
            'log_filename',
            default_value='journey_bench.jsonl',
            description='JSONL filename created under artifacts/logs',
        ),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    waypoints_file = LaunchConfiguration('waypoints_file')
    log_filename = LaunchConfiguration('log_filename')

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

    path_follower_node = Node(
        package='auto_nav',
        executable='path_follower',
        name='path_follower',
        output='screen',
        parameters=[
            _cfg('waypoints.yaml'),
            _cfg('robot.yaml'),
            {
                'waypoints_file': waypoints_file,
                'use_sim_time': use_sim_time,
                'require_marker': False,
                'emit_journey_events': True,
            },
        ],
    )

    journey_logger_node = Node(
        package='auto_nav',
        executable='journey_logger',
        name='journey_logger',
        output='screen',
        parameters=[
            _cfg('mission.yaml'),
            {
                'use_sim_time': use_sim_time,
                'log_filename': log_filename,
            },
        ],
    )

    return LaunchDescription(
        args + [
            LogInfo(msg='=== auto_nav bench navigation: 4 waypoints + return-home logging ==='),
            home_pose_node,
            path_follower_node,
            journey_logger_node,
        ]
    )
