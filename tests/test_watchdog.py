"""
Unit tests for GamepadWatchdogNode.

We test the business logic without a live ROS2 runtime by mocking rclpy
and the node infrastructure. This keeps tests fast and dependency-free.
"""

import sys
import time
import types
import unittest
from unittest.mock import MagicMock, patch, call


# ---------------------------------------------------------------------------
# Minimal rclpy / ROS2 stubs so we can import the node without a live daemon
# ---------------------------------------------------------------------------

def _make_rclpy_stub():
    rclpy = types.ModuleType('rclpy')
    rclpy.node = types.ModuleType('rclpy.node')
    rclpy.qos  = types.ModuleType('rclpy.qos')

    # QoS stubs
    class _QoSProfile:
        def __init__(self, **kw): pass

    class _Reliability:
        RELIABLE = 'RELIABLE'

    class _Durability:
        TRANSIENT_LOCAL = 'TRANSIENT_LOCAL'

    rclpy.qos.QoSProfile        = _QoSProfile
    rclpy.qos.ReliabilityPolicy = _Reliability
    rclpy.qos.DurabilityPolicy  = _Durability

    # Node stub
    class _Node:
        def __init__(self, name):
            self._name = name
            self._logger = MagicMock()
            self._clock  = MagicMock()
            self._params  = {}

        def get_logger(self): return self._logger

        def get_clock(self):
            mock_clock = MagicMock()
            mock_clock.now.return_value = _Time()
            return mock_clock

        def declare_parameter(self, name, default):
            self._params[name] = default

        def get_parameter(self, name):
            val = self._params.get(name, None)
            p = MagicMock()
            if isinstance(val, int):
                p.get_parameter_value.return_value.integer_value = val
            elif isinstance(val, float):
                p.get_parameter_value.return_value.double_value = val
            else:
                p.get_parameter_value.return_value.string_value = val or ''
            return p

        def create_publisher(self, *a, **kw): return MagicMock()
        def create_subscription(self, *a, **kw): return MagicMock()
        def create_timer(self, *a, **kw): return MagicMock()

    class _Time:
        def __init__(self, ns=0):
            self.nanoseconds = ns

        def __sub__(self, other):
            t = _Time()
            t.nanoseconds = self.nanoseconds - other.nanoseconds
            return t

    rclpy.node.Node = _Node
    rclpy.init          = MagicMock()
    rclpy.spin          = MagicMock()
    rclpy.try_shutdown  = MagicMock()
    return rclpy, _Time


_rclpy_stub, _Time = _make_rclpy_stub()

# Stub sensor_msgs and std_msgs
_sensor = types.ModuleType('sensor_msgs')
_sensor.msg = types.ModuleType('sensor_msgs.msg')
_sensor.msg.Joy = MagicMock  # treated as a type

_std = types.ModuleType('std_msgs')
_std.msg = types.ModuleType('std_msgs.msg')


class _Bool:
    def __init__(self, data=False): self.data = data


_std.msg.Bool = _Bool

sys.modules.setdefault('rclpy',              _rclpy_stub)
sys.modules.setdefault('rclpy.node',         _rclpy_stub.node)
sys.modules.setdefault('rclpy.qos',          _rclpy_stub.qos)
sys.modules.setdefault('sensor_msgs',        _sensor)
sys.modules.setdefault('sensor_msgs.msg',    _sensor.msg)
sys.modules.setdefault('std_msgs',           _std)
sys.modules.setdefault('std_msgs.msg',       _std.msg)

# Now import the node under test
from auto_nav.teleop.gamepad_watchdog_node import GamepadWatchdogNode  # noqa: E402


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_node(axis_deadman=5, deadman_threshold=0.5, joy_timeout_sec=1.0):
    """Build a GamepadWatchdogNode with given params (no live ROS2 needed)."""
    node = GamepadWatchdogNode.__new__(GamepadWatchdogNode)
    # Inject params directly
    node._params = {
        'axis_deadman':      axis_deadman,
        'deadman_threshold': deadman_threshold,
        'joy_timeout_sec':   joy_timeout_sec,
    }
    node._logger = MagicMock()

    # Build a clock that we can control
    node._clock_ns = 0

    def _now():
        t = _Time(node._clock_ns)
        return t

    mock_clock = MagicMock()
    mock_clock.now.side_effect = _now

    # Manually call __init__ logic without super().__init__
    node._axis_deadman       = axis_deadman
    node._deadman_threshold  = deadman_threshold
    node._timeout            = joy_timeout_sec
    node._pub_deadman        = MagicMock()
    node._pub_connected      = MagicMock()
    node._connected          = False
    node._deadman_ok         = False
    node._last_joy_time      = _Time(0)

    # Attach controllable clock
    node.get_clock = lambda: mock_clock

    return node


def _make_joy(axes=None, buttons=None):
    joy = MagicMock()
    joy.axes    = axes    if axes    is not None else [0.0] * 8
    joy.buttons = buttons if buttons is not None else [0]   * 12
    return joy


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestWatchdogDeadman(unittest.TestCase):

    def test_deadman_false_when_trigger_below_threshold(self):
        node = _make_node(axis_deadman=5, deadman_threshold=0.5)
        joy  = _make_joy(axes=[0.0] * 8)
        joy.axes[5] = 0.3  # below threshold
        node._joy_cb(joy)
        self.assertFalse(node._deadman_ok)

    def test_deadman_true_when_trigger_above_threshold(self):
        node = _make_node(axis_deadman=5, deadman_threshold=0.5)
        joy  = _make_joy(axes=[0.0] * 8)
        joy.axes[5] = 0.9  # above threshold
        node._joy_cb(joy)
        self.assertTrue(node._deadman_ok)

    def test_deadman_exactly_at_threshold_is_false(self):
        """Edge: threshold comparison is strict >."""
        node = _make_node(axis_deadman=5, deadman_threshold=0.5)
        joy  = _make_joy(axes=[0.0] * 8)
        joy.axes[5] = 0.5
        node._joy_cb(joy)
        self.assertFalse(node._deadman_ok)

    def test_axis_out_of_range_does_not_crash(self):
        """If Joy message has fewer axes than configured index, deadman stays False."""
        node = _make_node(axis_deadman=10)
        joy  = _make_joy(axes=[0.0] * 3)  # only 3 axes
        node._joy_cb(joy)
        self.assertFalse(node._deadman_ok)


class TestWatchdogConnection(unittest.TestCase):

    def test_connected_after_first_joy_message(self):
        node = _make_node()
        self.assertFalse(node._connected)
        node._joy_cb(_make_joy())
        self.assertTrue(node._connected)

    def test_disconnected_after_timeout(self):
        node = _make_node(joy_timeout_sec=1.0)
        node._joy_cb(_make_joy())           # connect
        self.assertTrue(node._connected)

        # Advance clock beyond timeout
        node._clock_ns = int(2.0 * 1e9)    # 2 seconds
        node._watchdog_tick()
        self.assertFalse(node._connected)

    def test_deadman_cleared_on_disconnect(self):
        node = _make_node(joy_timeout_sec=1.0, deadman_threshold=0.5)
        joy  = _make_joy(axes=[0.0] * 8)
        joy.axes[5] = 0.9
        node._joy_cb(joy)
        self.assertTrue(node._deadman_ok)

        node._clock_ns = int(2.0 * 1e9)
        node._watchdog_tick()
        self.assertFalse(node._deadman_ok)

    def test_no_disconnect_before_timeout(self):
        node = _make_node(joy_timeout_sec=1.0)
        node._joy_cb(_make_joy())

        # Only 0.5 s elapsed — still connected
        node._clock_ns = int(0.5 * 1e9)
        node._watchdog_tick()
        self.assertTrue(node._connected)


class TestWatchdogPublishes(unittest.TestCase):

    def test_publish_called_on_joy_message(self):
        node = _make_node()
        node._joy_cb(_make_joy())
        node._pub_connected.publish.assert_called()
        node._pub_deadman.publish.assert_called()

    def test_publish_called_on_watchdog_tick(self):
        node = _make_node()
        node._watchdog_tick()
        node._pub_connected.publish.assert_called()
        node._pub_deadman.publish.assert_called()


if __name__ == '__main__':
    unittest.main()
