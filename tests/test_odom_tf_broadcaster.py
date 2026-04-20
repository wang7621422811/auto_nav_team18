# Copyright 2026 team18
"""Tests for forwarding /odom pose messages onto /tf."""

from __future__ import annotations

import sys
import types
import unittest
from unittest.mock import MagicMock


def _install_stubs():
    rclpy = types.ModuleType('rclpy')
    rclpy.node = types.ModuleType('rclpy.node')

    class _Node:
        def __init__(self, name):  # pragma: no cover - constructor bypassed in tests
            self._name = name

        def create_subscription(self, *args, **kwargs):  # pragma: no cover
            return MagicMock()

        def get_logger(self):
            return MagicMock()

    rclpy.node.Node = _Node
    rclpy.init = MagicMock()
    rclpy.spin = MagicMock()
    rclpy.try_shutdown = MagicMock()

    geometry_msgs = types.ModuleType('geometry_msgs')
    geometry_msgs.msg = types.ModuleType('geometry_msgs.msg')

    class _Header:
        def __init__(self):
            self.stamp = MagicMock()
            self.frame_id = ''

    class _Translation:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class _Rotation:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.w = 1.0

    class _Transform:
        def __init__(self):
            self.translation = _Translation()
            self.rotation = _Rotation()

    class TransformStamped:
        def __init__(self):
            self.header = _Header()
            self.child_frame_id = ''
            self.transform = _Transform()

    geometry_msgs.msg.TransformStamped = TransformStamped

    nav_msgs = types.ModuleType('nav_msgs')
    nav_msgs.msg = types.ModuleType('nav_msgs.msg')

    class Odometry:
        def __init__(self):
            self.header = _Header()
            self.child_frame_id = ''
            pose = types.SimpleNamespace(
                position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
            self.pose = types.SimpleNamespace(pose=pose)

    nav_msgs.msg.Odometry = Odometry

    tf2_ros = types.ModuleType('tf2_ros')
    tf2_ros.TransformBroadcaster = MagicMock()

    for mod, name in [
        (rclpy, 'rclpy'),
        (rclpy.node, 'rclpy.node'),
        (geometry_msgs, 'geometry_msgs'),
        (geometry_msgs.msg, 'geometry_msgs.msg'),
        (nav_msgs, 'nav_msgs'),
        (nav_msgs.msg, 'nav_msgs.msg'),
        (tf2_ros, 'tf2_ros'),
    ]:
        sys.modules.setdefault(name, mod)

    return Odometry


_Odometry = _install_stubs()

from auto_nav.odom_tf_broadcaster import OdomTfBroadcasterNode  # noqa: E402


def _make_node() -> OdomTfBroadcasterNode:
    node = OdomTfBroadcasterNode.__new__(OdomTfBroadcasterNode)
    node._tf_pub = MagicMock()
    logger = MagicMock()
    node.get_logger = lambda: logger
    return node


def _make_odom(parent: str = 'odom', child: str = 'base_link') -> _Odometry:
    msg = _Odometry()
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.header.stamp = types.SimpleNamespace(sec=1, nanosec=2)
    msg.pose.pose.position.x = 1.25
    msg.pose.pose.position.y = -0.5
    msg.pose.pose.position.z = 0.1
    msg.pose.pose.orientation.z = 0.2
    msg.pose.pose.orientation.w = 0.98
    return msg


class TestOdomTfBroadcaster(unittest.TestCase):

    def test_odom_message_is_forwarded_to_tf(self):
        node = _make_node()
        msg = _make_odom()

        node._odom_cb(msg)

        node._tf_pub.sendTransform.assert_called_once()
        tf_msg = node._tf_pub.sendTransform.call_args.args[0]
        self.assertEqual(tf_msg.header.frame_id, 'odom')
        self.assertEqual(tf_msg.child_frame_id, 'base_link')
        self.assertAlmostEqual(tf_msg.transform.translation.x, 1.25)
        self.assertAlmostEqual(tf_msg.transform.translation.y, -0.5)
        self.assertAlmostEqual(tf_msg.transform.translation.z, 0.1)
        self.assertAlmostEqual(tf_msg.transform.rotation.z, 0.2)
        self.assertAlmostEqual(tf_msg.transform.rotation.w, 0.98)

    def test_missing_frame_names_are_ignored(self):
        node = _make_node()
        msg = _make_odom(parent='', child='base_link')

        node._odom_cb(msg)

        node._tf_pub.sendTransform.assert_not_called()
        node.get_logger().warn.assert_called_once()
