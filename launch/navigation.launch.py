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
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
            default_value=_cfg('waypoints.yaml'),
            description='Absolute path to the waypoints YAML data file',
        ),
    ]

    use_sim_time  = LaunchConfiguration('use_sim_time')
    waypoints_file = LaunchConfiguration('waypoints_file')

    # ---- home_pose_recorder -----------------------------------------------
    # Records the robot pose the first time /odom is received.
    # path_follower also independently records home at mission start (first AUTO),
    # but this node provides a latched /mission/home_pose for other consumers.
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

    # ---- path_follower ----------------------------------------------------
    path_follower_node = Node(
        package='auto_nav',
        executable='path_follower',
        name='path_follower',
        output='screen',
        parameters=[
            _cfg('waypoints.yaml'),       # ROS2 params (path_follower section)
            _cfg('robot.yaml'),           # max_linear_vel / max_angular_vel
            {
                # Pass the resolved absolute path so WaypointProvider can open the file
                'waypoints_file': waypoints_file,
                'use_sim_time':   use_sim_time,
            },
        ],
    )

    # ---- obstacle_guard ---------------------------------------------------
    # Hard safety stop: monitors front sector, publishes /emergency_stop.
    # In FINAL_APPROACH mode it whitelists the target cone so the robot can
    # approach within 1–2 m without triggering a false estop.
    obstacle_guard_node = Node(
        package='auto_nav',
        executable='obstacle_guard',
        name='obstacle_guard',
        output='screen',
        parameters=[
            _cfg('lidar.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    # ---- gap_planner ------------------------------------------------------
    # Selects the best passable gap from /scan and publishes a local target
    # point for ordinary (non-weave) navigation segments.
    gap_planner_node = Node(
        package='auto_nav',
        executable='gap_planner',
        name='gap_planner',
        output='screen',
        parameters=[
            _cfg('lidar.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    # ---- weave_planner ----------------------------------------------------
    # Corridor-aware gap planner — active only on segment "1" (WP[0]→WP[1]).
    weave_planner_node = Node(
        package='auto_nav',
        executable='weave_planner',
        name='weave_planner',
        output='screen',
        parameters=[
            _cfg('lidar.yaml'),
            {
                'waypoints_file': waypoints_file,
                'use_sim_time':   use_sim_time,
            },
        ],
    )

    # ---- local_planner ----------------------------------------------------
    # Orchestrates gap vs weave output and forwards it as /local_target
    # which path_follower uses during NAVIGATING state.
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

    return LaunchDescription(
        args + [
            LogInfo(msg='=== auto_nav Step 3: navigation + LiDAR avoidance + weaving ==='),
            home_pose_node,
            path_follower_node,
            obstacle_guard_node,
            gap_planner_node,
            weave_planner_node,
            local_planner_node,
        ]
    )
