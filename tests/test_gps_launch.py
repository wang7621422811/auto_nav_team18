# Copyright 2026 team18
"""Launch contract tests for GPS pose-fusion mode."""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path


_ROOT = Path(__file__).resolve().parents[1]


class _FakeLaunchDescription:
    def __init__(self, entities):
        self.entities = list(entities)


class _FakeDeclareLaunchArgument:
    def __init__(self, name, **kwargs):
        self.name = name
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


class _FakeNode:
    def __init__(self, **kwargs):
        self.package = kwargs.get('package')
        self.executable = kwargs.get('executable')
        self.name = kwargs.get('name')
        self.parameters = kwargs.get('parameters', [])
        self.remappings = kwargs.get('remappings', [])


def _install_launch_stubs() -> None:
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


def _load_launch(name: str):
    _install_launch_stubs()
    spec = importlib.util.spec_from_file_location(
        f'{name}_under_test',
        _ROOT / 'launch' / f'{name}.launch.py',
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_bringup_pose_source_switches_with_use_gps() -> None:
    module = _load_launch('bringup')
    ld = module.generate_launch_description()
    pose_op = next(
        entity
        for entity in ld.entities
        if isinstance(entity, _FakeOpaqueFunction) and entity.function.__name__ == '_pose_source_nodes'
    )

    gps_nodes = pose_op.function(types.SimpleNamespace(launch_configurations={'use_gps': 'true'}))
    odom_nodes = pose_op.function(types.SimpleNamespace(launch_configurations={'use_gps': 'false'}))

    assert gps_nodes[0].name == 'outdoor_pose_fuser'
    assert gps_nodes[0].executable == 'outdoor_pose_fuser'
    assert odom_nodes[0].name == 'odom_tf_broadcaster'
    assert odom_nodes[0].executable == 'odom_tf_broadcaster'


def test_navigation_launch_remaps_odom_when_gps_enabled() -> None:
    module = _load_launch('navigation')
    ld = module.generate_launch_description()
    nav_op = next(
        entity
        for entity in ld.entities
        if isinstance(entity, _FakeOpaqueFunction) and entity.function.__name__ == '_nav_nodes'
    )

    gps_nodes = nav_op.function(types.SimpleNamespace(launch_configurations={'use_gps': 'true'}))
    by_name = {node.name: node for node in gps_nodes}

    assert ('/odom', '/nav/odom') in by_name['home_pose_recorder'].remappings
    assert ('/odom', '/nav/odom') in by_name['path_follower'].remappings
    assert ('/odom', '/nav/odom') in by_name['obstacle_guard'].remappings
    assert ('/odom', '/nav/odom') in by_name['gap_planner'].remappings
    assert ('/odom', '/nav/odom') in by_name['weave_planner'].remappings


def test_mission_launch_remaps_controller_odom_when_gps_enabled() -> None:
    module = _load_launch('mission')
    ld = module.generate_launch_description()
    mission_op = next(
        entity
        for entity in ld.entities
        if isinstance(entity, _FakeOpaqueFunction) and entity.function.__name__ == '_mission_nodes'
    )

    gps_nodes = mission_op.function(types.SimpleNamespace(launch_configurations={'use_gps': 'true'}))
    by_name = {node.name: node for node in gps_nodes}

    assert ('/odom', '/nav/odom') in by_name['mission_controller'].remappings
    assert by_name['journey_logger'].remappings == []
    assert by_name['summary_generator'].remappings == []
