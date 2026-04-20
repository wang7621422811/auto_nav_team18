from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'auto_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        # Install all config files
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        # Install per-gamepad profiles (config/gamepad/<profile>.yaml)
        (os.path.join('share', package_name, 'config', 'gamepad'),
         glob('config/gamepad/*.yaml')),
        # ── Simulation assets ────────────────────────────────────────────────
        # launch + bridge config
        (os.path.join('share', package_name, 'simulation', 'sim_bringup'),
         glob('auto_nav/simulation/sim_bringup/*.py')
         + glob('auto_nav/simulation/sim_bringup/*.yaml')),
        # URDF
        (os.path.join('share', package_name, 'simulation', 'sim_robot_description'),
         glob('auto_nav/simulation/sim_robot_description/*.urdf')),
        # World files
        (os.path.join('share', package_name, 'simulation', 'sim_worlds'),
         glob('auto_nav/simulation/sim_worlds/*.world')),
        # Meshes (.dae and .stl) — required for package:// URI resolution
        (os.path.join('share', package_name, 'simulation', 'meshes'),
         glob('auto_nav/simulation/meshes/*.dae')
         + glob('auto_nav/simulation/meshes/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team18',
    maintainer_email='team18@example.com',
    description='Autonomous navigation for Pioneer 3-AT — Step 0 bringup',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gamepad_watchdog   = auto_nav.teleop.gamepad_watchdog_node:main',
            'joy_mapper         = auto_nav.teleop.joy_mapper_node:main',
            'mode_manager       = auto_nav.teleop.mode_manager_node:main',
            'cmd_gate           = auto_nav.teleop.cmd_gate_node:main',
            'home_pose_recorder = auto_nav.mission.home_pose_recorder:main',
            'path_follower      = auto_nav.navigation.path_follower:main',
            # Step 3: LiDAR avoidance + weaving
            'obstacle_guard     = auto_nav.navigation.obstacle_guard:main',
            'gap_planner        = auto_nav.navigation.gap_planner:main',
            'weave_planner      = auto_nav.navigation.weave_planner:main',
            'local_planner      = auto_nav.navigation.local_planner:main',
            # Step 4: perception — cone, object, distance, photo
            'cone_detector      = auto_nav.perception.cone_detector:main',
            'object_detector    = auto_nav.perception.object_detector:main',
            'distance_estimator = auto_nav.perception.distance_estimator:main',
            'photo_capture      = auto_nav.perception.photo_capture:main',
            # Step 5: mission state machine
            'mission_controller = auto_nav.mission.mission_controller:main',
            'journey_logger     = auto_nav.mission.journey_logger:main',
            'summary_generator  = auto_nav.mission.summary_generator:main',
            # Simulation helpers
            'sim_init           = auto_nav.simulation.sim_bringup.sim_init_node:main',
        ],
    },
)
