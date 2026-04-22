# Copyright 2026 team18
"""Contract tests for the minimal TF tree required by Step 0."""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path

from auto_nav.config_params import load_ros_param_from_files
from auto_nav.robot_extrinsics import load_sensor_xyz_from_files


_ROOT = Path(__file__).resolve().parents[1]


class _FakeLaunchDescription:
    """Small stand-in so launch files can be imported in unit tests."""

    def __init__(self, entities):
        self.entities = list(entities)


class _FakeDeclareLaunchArgument:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs


class _FakeLogInfo:
    def __init__(self, *, msg: str):
        self.msg = msg

    def execute(self, context) -> None:
        return None


class _FakeOpaqueFunction:
    def __init__(self, *, function):
        self.function = function


class _FakeLaunchConfiguration:
    def __init__(self, name: str):
        self.name = name

    def __str__(self) -> str:
        return f'<LaunchConfiguration {self.name}>'


class _FakeNode:
    def __init__(self, **kwargs):
        self.package = kwargs.get('package')
        self.executable = kwargs.get('executable')
        self.name = kwargs.get('name')
        self.arguments = kwargs.get('arguments', [])
        self.parameters = kwargs.get('parameters', [])
        self.output = kwargs.get('output')
        self.remappings = kwargs.get('remappings', [])


def _install_launch_stubs() -> None:
    """Stub ROS-only modules so the launch file can be imported under pytest."""
    launch_mod = types.ModuleType('launch')
    launch_mod.LaunchDescription = _FakeLaunchDescription

    actions_mod = types.ModuleType('launch.actions')
    actions_mod.DeclareLaunchArgument = _FakeDeclareLaunchArgument
    actions_mod.LogInfo = _FakeLogInfo
    actions_mod.OpaqueFunction = _FakeOpaqueFunction

    substitutions_mod = types.ModuleType('launch.substitutions')
    substitutions_mod.LaunchConfiguration = _FakeLaunchConfiguration

    launch_ros_mod = types.ModuleType('launch_ros')
    launch_ros_actions_mod = types.ModuleType('launch_ros.actions')
    launch_ros_actions_mod.Node = _FakeNode
    launch_ros_mod.actions = launch_ros_actions_mod

    ament_mod = types.ModuleType('ament_index_python')
    ament_packages_mod = types.ModuleType('ament_index_python.packages')

    def _fake_get_package_share_directory(name: str) -> str:
        if name != 'auto_nav':
            raise KeyError(name)
        return str(_ROOT)

    ament_packages_mod.get_package_share_directory = _fake_get_package_share_directory
    ament_mod.packages = ament_packages_mod

    sys.modules['launch'] = launch_mod
    sys.modules['launch.actions'] = actions_mod
    sys.modules['launch.substitutions'] = substitutions_mod
    sys.modules['launch_ros'] = launch_ros_mod
    sys.modules['launch_ros.actions'] = launch_ros_actions_mod
    sys.modules['ament_index_python'] = ament_mod
    sys.modules['ament_index_python.packages'] = ament_packages_mod


def _load_launch_module(filename: str):
    _install_launch_stubs()
    spec = importlib.util.spec_from_file_location(
        filename.replace('.', '_'),
        _ROOT / 'launch' / filename,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_minimal_tf_frame_contract_matches_docs() -> None:
    robot_cfg = _ROOT / 'config' / 'robot.yaml'
    lidar_cfgs = [_ROOT / 'config' / 'lidar.yaml', _ROOT / 'config' / 'real.yaml']
    camera_cfg = _ROOT / 'config' / 'camera.yaml'

    assert load_ros_param_from_files([robot_cfg], node_name='aria_node', key='odom_frame') == 'odom'
    assert load_ros_param_from_files([robot_cfg], node_name='aria_node', key='base_frame') == 'base_link'
    assert load_ros_param_from_files(lidar_cfgs, node_name='sick_scan', key='frame_id') == 'laser'
    assert load_ros_param_from_files([camera_cfg], node_name='camera', key='base_frame') == 'base_link'
    assert load_ros_param_from_files([camera_cfg], node_name='camera', key='camera_frame') == 'camera_link'


def test_bringup_launch_publishes_static_sensor_tf_chain() -> None:
    module = _load_launch_module('bringup.launch.py')
    launch_description = module.generate_launch_description()

    static_tf_op = next(
        entity
        for entity in launch_description.entities
        if isinstance(entity, _FakeOpaqueFunction) and entity.function.__name__ == '_sensor_static_tf'
    )

    context = types.SimpleNamespace(launch_configurations={'use_sim_time': 'false'})
    nodes = static_tf_op.function(context)
    by_name = {node.name: node for node in nodes}

    assert set(by_name) == {'tf_base_to_laser', 'tf_base_to_camera'}

    expected_xyz = load_sensor_xyz_from_files(
        [_ROOT / 'config' / 'robot.yaml', _ROOT / 'config' / 'real.yaml']
    )

    laser_node = by_name['tf_base_to_laser']
    assert laser_node.package == 'tf2_ros'
    assert laser_node.executable == 'static_transform_publisher'
    assert laser_node.arguments == [
        str(expected_xyz['laser'][0]),
        str(expected_xyz['laser'][1]),
        str(expected_xyz['laser'][2]),
        '0.0',
        '0.0',
        '0.0',
        'base_link',
        'laser',
    ]

    camera_node = by_name['tf_base_to_camera']
    assert camera_node.package == 'tf2_ros'
    assert camera_node.executable == 'static_transform_publisher'
    assert camera_node.arguments == [
        str(expected_xyz['camera'][0]),
        str(expected_xyz['camera'][1]),
        str(expected_xyz['camera'][2]),
        '0.0',
        '0.0',
        '0.0',
        'base_link',
        'camera_link',
    ]


def test_bringup_launch_includes_odom_tf_broadcaster_without_gps() -> None:
    module = _load_launch_module('bringup.launch.py')
    launch_description = module.generate_launch_description()

    tf_op = next(
        entity
        for entity in launch_description.entities
        if isinstance(entity, _FakeOpaqueFunction) and entity.function.__name__ == '_tf_nodes'
    )

    context = types.SimpleNamespace(
        launch_configurations={'use_gps': 'false', 'use_sim_time': 'false'}
    )
    nodes = tf_op.function(context)
    assert len(nodes) == 1
    odom_tf_node = nodes[0]

    assert odom_tf_node.package == 'auto_nav'
    assert odom_tf_node.executable == 'odom_tf_broadcaster'


def test_bringup_launch_uses_outdoor_pose_fuser_with_gps() -> None:
    module = _load_launch_module('bringup.launch.py')
    launch_description = module.generate_launch_description()

    tf_op = next(
        entity
        for entity in launch_description.entities
        if isinstance(entity, _FakeOpaqueFunction) and entity.function.__name__ == '_tf_nodes'
    )

    context = types.SimpleNamespace(
        launch_configurations={'use_gps': 'true', 'use_sim_time': 'false'}
    )
    nodes = tf_op.function(context)
    assert len(nodes) == 1

    gps_node = nodes[0]
    assert gps_node.package == 'auto_nav'
    assert gps_node.executable == 'outdoor_pose_fuser'
    assert gps_node.name == 'outdoor_pose_fuser'


def test_bringup_launch_isolates_vendor_sick_tf_from_main_tree() -> None:
    module = _load_launch_module('bringup.launch.py')
    launch_description = module.generate_launch_description()

    lidar_op = next(
        entity
        for entity in launch_description.entities
        if isinstance(entity, _FakeOpaqueFunction) and entity.function.__name__ == '_lidar_nodes'
    )

    context = types.SimpleNamespace(launch_configurations={'lidar_type': 'sick'})
    nodes = lidar_op.function(context)
    assert len(nodes) == 1
    sick_node = nodes[0]

    assert sick_node.name == 'sick_scan'
    assert ('/tf', '/sick_scan/tf') in sick_node.remappings
    assert ('/tf_static', '/sick_scan/tf_static') in sick_node.remappings


def test_navigation_launch_remaps_odom_in_gps_mode() -> None:
    module = _load_launch_module('navigation.launch.py')
    launch_description = module.generate_launch_description()

    nav_op = next(
        entity
        for entity in launch_description.entities
        if isinstance(entity, _FakeOpaqueFunction) and entity.function.__name__ == '_navigation_nodes'
    )

    context = types.SimpleNamespace(
        launch_configurations={
            'use_gps': 'true',
            'use_sim_time': 'false',
            'waypoints_file': '',
        }
    )
    nodes = nav_op.function(context)
    by_name = {node.name: node for node in nodes}

    for name in [
        'home_pose_recorder',
        'path_follower',
        'obstacle_guard',
        'gap_planner',
        'weave_planner',
    ]:
        assert ('/odom', '/nav/odom') in by_name[name].remappings

    path_param_overrides = [
        params
        for params in by_name['path_follower'].parameters
        if isinstance(params, dict)
    ]
    assert {
        'waypoints_file': str(_ROOT / 'config' / 'waypoints_real_gps.yaml'),
        'use_sim_time': False,
    } in path_param_overrides


def test_navigation_launch_defaults_to_local_waypoints_without_gps() -> None:
    module = _load_launch_module('navigation.launch.py')
    launch_description = module.generate_launch_description()

    nav_op = next(
        entity
        for entity in launch_description.entities
        if isinstance(entity, _FakeOpaqueFunction) and entity.function.__name__ == '_navigation_nodes'
    )

    context = types.SimpleNamespace(
        launch_configurations={
            'use_gps': 'false',
            'use_sim_time': 'false',
            'waypoints_file': '',
        }
    )
    nodes = nav_op.function(context)
    by_name = {node.name: node for node in nodes}

    assert ('/odom', '/nav/odom') not in by_name['path_follower'].remappings
    path_param_overrides = [
        params
        for params in by_name['path_follower'].parameters
        if isinstance(params, dict)
    ]
    assert {
        'waypoints_file': str(_ROOT / 'config' / 'waypoints_data.yaml'),
        'use_sim_time': False,
    } in path_param_overrides


def test_mission_launch_remaps_controller_odom_in_gps_mode() -> None:
    module = _load_launch_module('mission.launch.py')
    launch_description = module.generate_launch_description()

    mission_op = next(
        entity
        for entity in launch_description.entities
        if isinstance(entity, _FakeOpaqueFunction) and entity.function.__name__ == '_mission_nodes'
    )

    context = types.SimpleNamespace(
        launch_configurations={'use_gps': 'true', 'use_sim_time': 'false'}
    )
    nodes = mission_op.function(context)
    by_name = {node.name: node for node in nodes}

    assert ('/odom', '/nav/odom') in by_name['mission_controller'].remappings
    assert by_name['journey_logger'].remappings == []
    assert by_name['summary_generator'].remappings == []
