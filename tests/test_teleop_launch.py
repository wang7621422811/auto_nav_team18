# Copyright 2026 team18
"""Launch-level regression tests for the Step-1 teleop entrypoint."""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
TELEOP_LAUNCH = REPO_ROOT / 'launch' / 'teleop.launch.py'


class _FakeNode:
    """Lightweight stand-in for launch_ros.actions.Node used in tests."""

    def __init__(self, **kwargs):
        self.package = kwargs['package']
        self.executable = kwargs['executable']
        self.name = kwargs.get('name')
        self.output = kwargs.get('output')
        self.parameters = kwargs.get('parameters', [])
        self.remappings = kwargs.get('remappings', [])


class _FakeDeclareLaunchArgument:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs


class _FakeLogInfo:
    def __init__(self, **kwargs):
        self.msg = kwargs.get('msg')


class _FakeOpaqueFunction:
    def __init__(self, **kwargs):
        self.function = kwargs.get('function')


class _FakeLaunchDescription:
    def __init__(self, entities):
        self.entities = entities


def _load_teleop_launch_module():
    """Import launch/teleop.launch.py with minimal launch stubs installed."""
    ament_index = types.ModuleType('ament_index_python')
    ament_packages = types.ModuleType('ament_index_python.packages')
    ament_packages.get_package_share_directory = lambda _pkg: str(REPO_ROOT)

    launch = types.ModuleType('launch')
    launch.LaunchDescription = _FakeLaunchDescription
    launch.actions = types.ModuleType('launch.actions')
    launch.actions.DeclareLaunchArgument = _FakeDeclareLaunchArgument
    launch.actions.LogInfo = _FakeLogInfo
    launch.actions.OpaqueFunction = _FakeOpaqueFunction
    launch.substitutions = types.ModuleType('launch.substitutions')
    launch.substitutions.LaunchConfiguration = lambda name: name

    launch_ros = types.ModuleType('launch_ros')
    launch_ros.actions = types.ModuleType('launch_ros.actions')
    launch_ros.actions.Node = _FakeNode

    old_modules = {}
    stub_modules = {
        'ament_index_python': ament_index,
        'ament_index_python.packages': ament_packages,
        'launch': launch,
        'launch.actions': launch.actions,
        'launch.substitutions': launch.substitutions,
        'launch_ros': launch_ros,
        'launch_ros.actions': launch_ros.actions,
    }

    for name, module in stub_modules.items():
        old_modules[name] = sys.modules.get(name)
        sys.modules[name] = module

    spec = importlib.util.spec_from_file_location('test_teleop_launch_module', TELEOP_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    try:
        spec.loader.exec_module(module)
    finally:
        for name, previous in old_modules.items():
            if previous is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = previous

    return module


def test_teleop_launch_nodes_cover_manual_drive_and_deadman():
    module = _load_teleop_launch_module()
    context = types.SimpleNamespace(
        launch_configurations={
            'gamepad': 'ps4',
            'joy_dev': '/dev/input/test-js0',
            'use_sim_time': 'true',
        }
    )

    nodes = [entity for entity in module._nodes(context) if isinstance(entity, _FakeNode)]
    by_name = {node.name: node for node in nodes}

    assert set(by_name) == {
        'joy',
        'teleop_twist_joy',
        'joy_mapper',
        'gamepad_watchdog',
        'mode_manager',
        'cmd_gate',
    }
    assert by_name['joy'].parameters[1]['dev'] == '/dev/input/test-js0'
    assert by_name['joy'].parameters[1]['use_sim_time'] is True
    assert by_name['teleop_twist_joy'].remappings == [('/cmd_vel', '/cmd_vel_manual')]
    assert by_name['gamepad_watchdog'].parameters[1]['use_sim_time'] is True


def test_teleop_launch_rejects_unknown_gamepad_profile():
    module = _load_teleop_launch_module()
    context = types.SimpleNamespace(
        launch_configurations={
            'gamepad': 'does_not_exist',
            'joy_dev': '/dev/input/js0',
            'use_sim_time': 'false',
        }
    )

    with pytest.raises(FileNotFoundError, match="Unknown gamepad profile"):
        module._nodes(context)
