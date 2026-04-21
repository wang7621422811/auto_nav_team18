"""
navigation.launch.py — Step 3: waypoint navigation + LiDAR avoidance + weaving.

Assumes teleop.launch.py (or bringup.launch.py + teleop.launch.py) is already
running so that /odom, /cmd_vel_safe, /control_mode, and /deadman_ok exist.
A LiDAR driver must also be running and publishing /scan.

Nodes started here:
  home_pose_recorder  — /mission/home_pose  (latched, once on first odom)
  path_follower       — /cmd_vel_auto  /waypoint/current  /waypoint/status
                        /navigation/segment
  obstacle_guard      — /emergency_stop  (safety stop on front-sector obstacles;
                        whitelist target cone in FINAL_APPROACH)
  gap_planner         — /gap/local_target  (gap-follower for normal segments)
  weave_planner       — /weave/local_target  (corridor-aware, active on segment "1")
  local_planner       — /local_target  (routes gap vs weave to path_follower)

CLI arguments:
  use_sim_time:=false
  waypoints_file:=<absolute path>   (default: auto-detected from package share)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
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
            default_value=_cfg('waypoints_data.yaml'),
            description='Absolute path to the waypoints data YAML file (not the ROS params file)',
        ),
        DeclareLaunchArgument(
            'use_gps',
            default_value='false',
            description='Remap navigation odom inputs from /odom to /nav/odom',
        ),
    ]

    use_sim_time  = LaunchConfiguration('use_sim_time')
    waypoints_file = LaunchConfiguration('waypoints_file')
    use_gps = LaunchConfiguration('use_gps')

    def _nav_nodes(context, *args, **kwargs):
        use_nav_odom = context.launch_configurations.get('use_gps', 'false').lower() == 'true'
        odom_remap = [('/odom', '/nav/odom')] if use_nav_odom else []

        home_pose_node = Node(
            package='auto_nav',
            executable='home_pose_recorder',
            name='home_pose_recorder',
            output='screen',
            parameters=[
                _cfg('mission.yaml'),
                {'use_sim_time': use_sim_time},
            ],
            remappings=odom_remap,
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
                },
            ],
            remappings=odom_remap,
        )

        obstacle_guard_node = Node(
            package='auto_nav',
            executable='obstacle_guard',
            name='obstacle_guard',
            output='screen',
            parameters=[
                _cfg('lidar.yaml'),
                {'use_sim_time': use_sim_time},
            ],
            remappings=odom_remap,
        )

        gap_planner_node = Node(
            package='auto_nav',
            executable='gap_planner',
            name='gap_planner',
            output='screen',
            parameters=[
                _cfg('lidar.yaml'),
                {'use_sim_time': use_sim_time},
            ],
            remappings=odom_remap,
        )

        weave_planner_node = Node(
            package='auto_nav',
            executable='weave_planner',
            name='weave_planner',
            output='screen',
            parameters=[
                _cfg('lidar.yaml'),
                {
                    'waypoints_file': waypoints_file,
                    'use_sim_time': use_sim_time,
                },
            ],
            remappings=odom_remap,
        )

        local_planner_node = Node(
            package='auto_nav',
            executable='local_planner',
            name='local_planner',
            output='screen',
            parameters=[
                _cfg('lidar.yaml'),
                {'use_sim_time': use_sim_time},
            ],
        )

        return [
            home_pose_node,
            path_follower_node,
            obstacle_guard_node,
            gap_planner_node,
            weave_planner_node,
            local_planner_node,
        ]

    return LaunchDescription(
        args + [
            LogInfo(msg='=== auto_nav Step 3: navigation + LiDAR avoidance + weaving ==='),
            OpaqueFunction(function=_nav_nodes),
        ]
    )
