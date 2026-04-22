"""
perception.launch.py — Step 4: camera-based cone detection, object detection,
                        distance estimation, and photo capture.

Prerequisites:
  - An OAK-D camera driver must be publishing:
      /camera/color/image_raw   (sensor_msgs/Image)
      /camera/depth/image_raw   (sensor_msgs/Image)
  - path_follower must be running and publishing /waypoint/status so that
    photo_capture knows when to trigger.

Nodes started here:
  cone_detector      — /marker/detection  /marker/bbox
  object_detector    — /object/detection
  distance_estimator — /perception/distance
  photo_capture      — /photo/saved

CLI arguments:
  use_sim_time:=false
  photo_dir:=artifacts/photos    (override output directory)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _cfg(filename: str) -> str:
    return os.path.join(
        get_package_share_directory("auto_nav"), "config", filename
    )


def generate_launch_description() -> LaunchDescription:

    sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulated clock"
    )
    photo_dir_arg = DeclareLaunchArgument(
        "photo_dir", default_value="artifacts/photos",
        description="Directory for captured waypoint photos"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    photo_dir    = LaunchConfiguration("photo_dir")

    camera_yaml = _cfg("camera.yaml")

    cone_detector_node = Node(
        package="auto_nav",
        executable="cone_detector",
        name="cone_detector",
        output="screen",
        parameters=[
            camera_yaml,
            {"use_sim_time": use_sim_time},
        ],
    )

    object_detector_node = Node(
        package="auto_nav",
        executable="object_detector",
        name="object_detector",
        output="screen",
        parameters=[
            camera_yaml,
            {"use_sim_time": use_sim_time},
        ],
    )

    distance_estimator_node = Node(
        package="auto_nav",
        executable="distance_estimator",
        name="distance_estimator",
        output="screen",
        parameters=[
            camera_yaml,
            {"use_sim_time": use_sim_time},
        ],
    )

    photo_capture_node = Node(
        package="auto_nav",
        executable="photo_capture",
        name="photo_capture",
        output="screen",
        parameters=[
            camera_yaml,
            {
                "use_sim_time": use_sim_time,
                "photo_dir":    photo_dir,
            },
        ],
    )

    return LaunchDescription([
        sim_time_arg,
        photo_dir_arg,
        LogInfo(msg="[perception.launch] Starting Step-4 perception nodes…"),
        LogInfo(msg="[perception.launch] Camera driver is NOT started here; "
                    "requires /camera/color/image_raw and /camera/depth/image_raw from bringup.launch.py or another publisher."),
        cone_detector_node,
        object_detector_node,
        distance_estimator_node,
        photo_capture_node,
    ])
