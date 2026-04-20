"""
sim_perception.launch.py  —  Perception stack for simulation  (Step G3)

Starts:
  1. cone_detector   — orange cone detection on /camera/color/image_raw
                       publishes /marker/detection, /marker/bbox
  2. photo_capture   — saves annotated photos on /waypoint/status trigger
  3. sim_photo_trigger — fires FINAL_APPROACH:0 on first cone detection
                         (replaces mission_controller trigger for sim testing)

All nodes use sim_camera.yaml for sim-tuned HSV / image-size params.

Launch after sim_bringup.launch.py is up:
  ros2 launch auto_nav/simulation/sim_bringup/sim_perception.launch.py

Verify:
  ros2 topic echo /marker/bbox
  ros2 topic echo /photo/saved
  ls artifacts/photos/
"""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

_THIS_DIR   = os.path.dirname(os.path.abspath(__file__))
_CAM_YAML   = os.path.join(_THIS_DIR, 'sim_camera.yaml')
_TRIGGER_PY = os.path.join(_THIS_DIR, 'sim_photo_trigger.py')


def generate_launch_description() -> LaunchDescription:

    cone_detector = Node(
        package='auto_nav',
        executable='cone_detector',
        name='cone_detector',
        parameters=[_CAM_YAML, {'use_sim_time': True}],
        output='screen',
        remappings=[],
    )

    photo_capture = Node(
        package='auto_nav',
        executable='photo_capture',
        name='photo_capture',
        parameters=[_CAM_YAML, {'use_sim_time': True}],
        output='screen',
    )

    # Standalone script — no console_scripts entry needed.
    sim_photo_trigger = ExecuteProcess(
        cmd=['python3', _TRIGGER_PY],
        output='screen',
    )

    return LaunchDescription([
        cone_detector,
        photo_capture,
        sim_photo_trigger,
    ])
