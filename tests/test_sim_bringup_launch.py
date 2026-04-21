# Copyright 2026 team18
"""Regression tests for the Gazebo simulation bringup entrypoint."""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
SIM_LAUNCH = REPO_ROOT / 'auto_nav' / 'simulation' / 'sim_bringup' / 'sim_bringup.launch.py'
BRIDGE_YAML = REPO_ROOT / 'auto_nav' / 'simulation' / 'sim_bringup' / 'bridge.yaml'


class _FakeExecuteProcess:
    def __init__(self, **kwargs):
        self.cmd = kwargs.get('cmd', [])
        self.output = kwargs.get('output')
        self.additional_env = kwargs.get('additional_env', {})


class _FakeDeclareLaunchArgument:
    def __init__(self, name, **kwargs):
        self.name = name
        self.kwargs = kwargs


class _FakeTimerAction:
    def __init__(self, **kwargs):
        self.period = kwargs.get('period')
        self.actions = kwargs.get('actions', [])


class _FakeIncludeLaunchDescription:
    def __init__(self, launch_description_source, launch_arguments=None):
        self.launch_description_source = launch_description_source
        self.launch_arguments = dict(launch_arguments or [])


class _FakePythonLaunchDescriptionSource:
    def __init__(self, location):
        self.location = location


class _FakeLaunchDescription:
    def __init__(self, entities):
        self.entities = entities


class _FakeNode:
    def __init__(self, **kwargs):
        self.package = kwargs['package']
        self.executable = kwargs['executable']
        self.name = kwargs.get('name')
        self.arguments = kwargs.get('arguments', [])
        self.parameters = kwargs.get('parameters', [])


def _load_module():
    launch = types.ModuleType('launch')
    launch.LaunchDescription = _FakeLaunchDescription
    launch.actions = types.ModuleType('launch.actions')
    launch.actions.DeclareLaunchArgument = _FakeDeclareLaunchArgument
    launch.actions.ExecuteProcess = _FakeExecuteProcess
    launch.actions.IncludeLaunchDescription = _FakeIncludeLaunchDescription
    launch.actions.TimerAction = _FakeTimerAction
    launch.substitutions = types.ModuleType('launch.substitutions')
    launch.substitutions.LaunchConfiguration = lambda name: f'LC<{name}>'
    launch.launch_description_sources = types.ModuleType('launch.launch_description_sources')
    launch.launch_description_sources.PythonLaunchDescriptionSource = _FakePythonLaunchDescriptionSource

    launch_ros = types.ModuleType('launch_ros')
    launch_ros.actions = types.ModuleType('launch_ros.actions')
    launch_ros.actions.Node = _FakeNode

    old_modules = {}
    stub_modules = {
        'launch': launch,
        'launch.actions': launch.actions,
        'launch.substitutions': launch.substitutions,
        'launch.launch_description_sources': launch.launch_description_sources,
        'launch_ros': launch_ros,
        'launch_ros.actions': launch_ros.actions,
    }

    for name, module in stub_modules.items():
        old_modules[name] = sys.modules.get(name)
        sys.modules[name] = module

    spec = importlib.util.spec_from_file_location('test_sim_bringup_launch_module', SIM_LAUNCH)
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


def test_sim_bringup_launch_uses_ogre_and_includes_teleop():
    module = _load_module()
    ld = module.generate_launch_description()

    args = {entity.name for entity in ld.entities if isinstance(entity, _FakeDeclareLaunchArgument)}
    assert {'world', 'render_engine', 'gamepad', 'joy_dev', 'x', 'y', 'z', 'yaw', 'use_sim_time'} <= args

    gazebo = next(entity for entity in ld.entities if isinstance(entity, _FakeExecuteProcess))
    assert gazebo.cmd[:4] == ['gz', 'sim', '-r', '--render-engine']
    assert gazebo.cmd[4] == 'LC<render_engine>'
    assert gazebo.cmd[5] == 'LC<world>'

    teleop = next(entity for entity in ld.entities if isinstance(entity, _FakeIncludeLaunchDescription))
    assert teleop.launch_description_source.location.endswith('launch/teleop.launch.py')
    assert teleop.launch_arguments['gamepad'] == 'LC<gamepad>'
    assert teleop.launch_arguments['joy_dev'] == 'LC<joy_dev>'
    assert teleop.launch_arguments['use_sim_time'] == 'LC<use_sim_time>'


def test_bridge_consumes_cmd_vel_safe():
    with BRIDGE_YAML.open('r', encoding='utf-8') as fh:
        config = yaml.safe_load(fh)

    cmd_bridge = next(entry for entry in config if entry['gz_topic_name'] == '/cmd_vel')
    assert cmd_bridge['ros_topic_name'] == '/cmd_vel_safe'
