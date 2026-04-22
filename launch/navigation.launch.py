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
  use_gps:=false
  waypoints_file:=<absolute path>   (empty = auto-select local or GPS file)
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


def _normalize_waypoint_path(path: str) -> str:
    return os.path.basename(path.strip())


def generate_launch_description() -> LaunchDescription:

    args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'use_gps', default_value='false',
            description='GPS outdoor mode: remap odom subscribers to /nav/odom',
        ),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value='',
            description='Absolute path to the waypoints data YAML file (empty = auto-select)',
        ),
    ]

    def _navigation_nodes(context, *args, **kwargs):
        use_gps = context.launch_configurations.get('use_gps', 'false').lower() == 'true'
        use_sim_time = context.launch_configurations.get('use_sim_time', 'false') == 'true'
        waypoints_file = context.launch_configurations.get('waypoints_file', '').strip()
        if not waypoints_file:
            waypoints_file = _cfg('waypoints_real_gps.yaml' if use_gps else 'waypoints_data.yaml')
        waypoint_basename = _normalize_waypoint_path(waypoints_file)

        if use_gps and waypoint_basename == 'waypoints_data.yaml':
            raise ValueError(
                '[navigation] GPS mode requires a real outdoor waypoint file; '
                f'refusing local file {waypoints_file!r}.'
            )

        odom_remaps = [('/odom', '/nav/odom')] if use_gps else []
        if use_gps:
            LogInfo(
                msg=f'[navigation] GPS mode: remapping /odom -> /nav/odom using {waypoints_file}'
            ).execute(context)
        else:
            LogInfo(msg=f'[navigation] Local odom mode: using {waypoints_file}').execute(context)

        # ---- home_pose_recorder -------------------------------------------
        # Records the robot pose the first time odometry is received.
        home_pose_node = Node(
            package='auto_nav',
            executable='home_pose_recorder',
            name='home_pose_recorder',
            output='screen',
            parameters=[
                _cfg('mission.yaml'),
                {'use_sim_time': use_sim_time},
            ],
            remappings=odom_remaps,
        )

        # ---- path_follower ------------------------------------------------
        path_follower_node = Node(
            package='auto_nav',
            executable='path_follower',
            name='path_follower',
            output='screen',
            parameters=[
                _cfg('robot.yaml'),
                _cfg('waypoints.yaml'),
                {
                    'waypoints_file': waypoints_file,
                    'use_sim_time': use_sim_time,
                },
            ],
            remappings=odom_remaps,
        )

        # ---- obstacle_guard -----------------------------------------------
        obstacle_guard_node = Node(
            package='auto_nav',
            executable='obstacle_guard',
            name='obstacle_guard',
            output='screen',
            parameters=[
                _cfg('lidar.yaml'),
                {'use_sim_time': use_sim_time},
            ],
            remappings=odom_remaps,
        )

        # ---- gap_planner --------------------------------------------------
        gap_planner_node = Node(
            package='auto_nav',
            executable='gap_planner',
            name='gap_planner',
            output='screen',
            parameters=[
                _cfg('lidar.yaml'),
                {'use_sim_time': use_sim_time},
            ],
            remappings=odom_remaps,
        )

        # ---- weave_planner ------------------------------------------------
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
            remappings=odom_remaps,
        )

        # ---- local_planner ------------------------------------------------
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
            OpaqueFunction(function=_navigation_nodes),
        ]
    )
