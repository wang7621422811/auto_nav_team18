"""
rviz.launch.py  —  RViz visualisation for Pioneer 3-AT simulation.

Shows:  grid, robot model (URDF), laser scan (red points), odometry trail, TF.
Run in a second terminal after sim_bringup.launch.py is up.

  source install/setup.bash
  ros2 launch auto_nav/simulation/sim_bringup/rviz.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_RVIZ_CFG = os.path.join(_THIS_DIR, 'rviz_config.rviz')


def generate_launch_description() -> LaunchDescription:
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', _RVIZ_CFG],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    return LaunchDescription([rviz])
