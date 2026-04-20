"""
Unit tests for Step-1 teleop nodes: JoyMapperNode, ModeManagerNode, CmdGateNode.

Tests run without a live ROS2 daemon by stubbing rclpy and message types.
"""

import sys
import types
import unittest
from unittest.mock import MagicMock


# ---------------------------------------------------------------------------
# Minimal ROS2 / message stubs
# ---------------------------------------------------------------------------

def _build_stubs():
    rclpy = types.ModuleType('rclpy')
    rclpy.node = types.ModuleType('rclpy.node')
    rclpy.qos  = types.ModuleType('rclpy.qos')

    class _QoS:
        def __init__(self, **kw): pass

    class _Rel:
        RELIABLE = 'RELIABLE'

    class _Dur:
        TRANSIENT_LOCAL = 'TRANSIENT_LOCAL'

    rclpy.qos.QoSProfile        = _QoS
    rclpy.qos.ReliabilityPolicy = _Rel
    rclpy.qos.DurabilityPolicy  = _Dur
    rclpy.init         = MagicMock()
    rclpy.spin         = MagicMock()
    rclpy.try_shutdown = MagicMock()

    class _Node:
        def __init__(self, name):
            self._name   = name
            self._logger = MagicMock()
            self._params = {}

        def get_logger(self):             return self._logger
        def create_publisher(self, *a, **kw):    return MagicMock()
        def create_subscription(self, *a, **kw): return MagicMock()
        def create_timer(self, *a, **kw):        return MagicMock()

        def declare_parameter(self, name, default):
            self._params[name] = default

        def get_parameter(self, name):
            val = self._params.get(name)
            p = MagicMock()
            if isinstance(val, int):
                p.get_parameter_value.return_value.integer_value = val
            elif isinstance(val, float):
                p.get_parameter_value.return_value.double_value = val
            else:
                p.get_parameter_value.return_value.string_value = val or ''
            return p

    rclpy.node.Node = _Node

    # --- geometry_msgs ---
    geo = types.ModuleType('geometry_msgs')
    geo.msg = types.ModuleType('geometry_msgs.msg')

    class _Twist:
        def __init__(self):
            self.linear  = MagicMock(x=0.0, y=0.0, z=0.0)
            self.angular = MagicMock(x=0.0, y=0.0, z=0.0)

    geo.msg.Twist = _Twist

    # --- sensor_msgs ---
    sensor = types.ModuleType('sensor_msgs')
    sensor.msg = types.ModuleType('sensor_msgs.msg')
    sensor.msg.Joy = MagicMock

    # --- std_msgs ---
    std = types.ModuleType('std_msgs')
    std.msg = types.ModuleType('std_msgs.msg')

    class _Bool:
        def __init__(self, data=False): self.data = data

    class _String:
        def __init__(self, data=''): self.data = data

    std.msg.Bool   = _Bool
    std.msg.String = _String

    for name, mod in [
        ('rclpy',               rclpy),
        ('rclpy.node',          rclpy.node),
        ('rclpy.qos',           rclpy.qos),
        ('geometry_msgs',       geo),
        ('geometry_msgs.msg',   geo.msg),
        ('sensor_msgs',         sensor),
        ('sensor_msgs.msg',     sensor.msg),
        ('std_msgs',            std),
        ('std_msgs.msg',        std.msg),
    ]:
        sys.modules.setdefault(name, mod)

    # If another test registered geometry_msgs.msg first (without Twist), patch it in.
    if not hasattr(sys.modules.get('geometry_msgs.msg'), 'Twist'):
        sys.modules['geometry_msgs.msg'].Twist = _Twist

    return _Bool, _String, _Twist


_Bool, _String, _Twist = _build_stubs()

# Import nodes under test
from auto_nav.teleop.joy_mapper_node   import JoyMapperNode    # noqa: E402
from auto_nav.teleop.mode_manager_node import ModeManagerNode  # noqa: E402
from auto_nav.teleop.cmd_gate_node     import CmdGateNode      # noqa: E402


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_joy(buttons=None, axes=None):
    joy = MagicMock()
    joy.buttons = buttons if buttons is not None else [0] * 12
    joy.axes    = axes    if axes    is not None else [0.0] * 8
    return joy


def _make_joy_mapper(btn_auto=0, btn_manual=1, btn_emergency=3):
    node = JoyMapperNode.__new__(JoyMapperNode)
    node._params = {
        'btn_auto_mode':      btn_auto,
        'btn_manual_mode':    btn_manual,
        'btn_emergency_stop': btn_emergency,
    }
    node._logger       = MagicMock()
    node._btn_auto     = btn_auto
    node._btn_manual   = btn_manual
    node._btn_emergency = btn_emergency
    node._pub_auto      = MagicMock()
    node._pub_manual    = MagicMock()
    node._pub_emergency = MagicMock()
    return node


def _make_mode_manager():
    node = ModeManagerNode.__new__(ModeManagerNode)
    node._logger              = MagicMock()
    node._pub_mode            = MagicMock()
    node._pub_estop           = MagicMock()
    node._mode                = ModeManagerNode.MANUAL
    node._prev_btn_auto       = False
    node._prev_btn_manual     = False
    node._prev_btn_emergency  = False
    node._connected           = True
    node._deadman_ok          = False
    node._estop_latched       = False
    return node


def _make_cmd_gate():
    node = CmdGateNode.__new__(CmdGateNode)
    node._logger      = MagicMock()
    node._pub_safe    = MagicMock()
    node._cmd_manual  = _Twist()
    node._cmd_auto    = _Twist()
    node._mode        = CmdGateNode.MANUAL
    node._deadman_ok  = False
    node._connected   = True
    node._estop       = False
    return node


# ---------------------------------------------------------------------------
# JoyMapperNode tests
# ---------------------------------------------------------------------------

class TestJoyMapper(unittest.TestCase):

    def test_auto_button_published_when_pressed(self):
        node = _make_joy_mapper(btn_auto=0)
        buttons = [0] * 12
        buttons[0] = 1
        node._joy_cb(_make_joy(buttons=buttons))
        last_call = node._pub_auto.publish.call_args[0][0]
        self.assertTrue(last_call.data)

    def test_auto_button_false_when_not_pressed(self):
        node = _make_joy_mapper(btn_auto=0)
        node._joy_cb(_make_joy(buttons=[0] * 12))
        last_call = node._pub_auto.publish.call_args[0][0]
        self.assertFalse(last_call.data)

    def test_manual_button_published_when_pressed(self):
        node = _make_joy_mapper(btn_manual=1)
        buttons = [0] * 12
        buttons[1] = 1
        node._joy_cb(_make_joy(buttons=buttons))
        last_call = node._pub_manual.publish.call_args[0][0]
        self.assertTrue(last_call.data)

    def test_emergency_button_published_when_pressed(self):
        node = _make_joy_mapper(btn_emergency=3)
        buttons = [0] * 12
        buttons[3] = 1
        node._joy_cb(_make_joy(buttons=buttons))
        last_call = node._pub_emergency.publish.call_args[0][0]
        self.assertTrue(last_call.data)

    def test_out_of_range_button_index_is_false(self):
        """Controller with fewer buttons than configured index → no crash, False."""
        node = _make_joy_mapper(btn_auto=15)
        node._joy_cb(_make_joy(buttons=[1] * 4))
        last_call = node._pub_auto.publish.call_args[0][0]
        self.assertFalse(last_call.data)

    def test_custom_button_mapping(self):
        """Switch Pro maps X to button 2 — remapping must work."""
        node = _make_joy_mapper(btn_auto=2)
        buttons = [0] * 12
        buttons[2] = 1
        node._joy_cb(_make_joy(buttons=buttons))
        last_call = node._pub_auto.publish.call_args[0][0]
        self.assertTrue(last_call.data)


# ---------------------------------------------------------------------------
# ModeManagerNode tests
# ---------------------------------------------------------------------------

class TestModeManager(unittest.TestCase):

    # ---- Mode transitions -------------------------------------------------

    def test_initial_mode_is_manual(self):
        node = _make_mode_manager()
        self.assertEqual(node._mode, ModeManagerNode.MANUAL)

    def test_btn_auto_rising_edge_sets_auto(self):
        node = _make_mode_manager()
        node._btn_auto_cb(_Bool(data=True))
        self.assertEqual(node._mode, ModeManagerNode.AUTO)

    def test_btn_auto_hold_does_not_re_trigger(self):
        """Holding the button should not flap modes."""
        node = _make_mode_manager()
        node._btn_auto_cb(_Bool(data=True))   # press
        node._mode = ModeManagerNode.MANUAL    # force back
        node._btn_auto_cb(_Bool(data=True))   # still held — no rising edge
        self.assertEqual(node._mode, ModeManagerNode.MANUAL)

    def test_btn_manual_returns_to_manual(self):
        node = _make_mode_manager()
        node._btn_auto_cb(_Bool(data=True))    # go AUTO
        node._prev_btn_auto = False
        node._btn_manual_cb(_Bool(data=True))  # press O
        self.assertEqual(node._mode, ModeManagerNode.MANUAL)

    def test_btn_manual_clears_estop(self):
        node = _make_mode_manager()
        node._estop_latched = True
        node._btn_manual_cb(_Bool(data=True))
        self.assertFalse(node._estop_latched)
        node._pub_estop.publish.assert_called()

    def test_btn_emergency_sets_aborted(self):
        node = _make_mode_manager()
        node._btn_emergency_cb(_Bool(data=True))
        self.assertEqual(node._mode, ModeManagerNode.ABORTED)
        self.assertTrue(node._estop_latched)

    # ---- Joystick disconnect ---------------------------------------------

    def test_disconnect_forces_manual(self):
        node = _make_mode_manager()
        node._btn_auto_cb(_Bool(data=True))    # go AUTO
        node._prev_btn_auto = False
        node._joy_connected_cb(_Bool(data=False))
        self.assertEqual(node._mode, ModeManagerNode.MANUAL)

    def test_reconnect_does_not_auto_restore_auto(self):
        """Reconnecting should not silently re-enter AUTO."""
        node = _make_mode_manager()
        node._joy_connected_cb(_Bool(data=False))
        node._joy_connected_cb(_Bool(data=True))
        self.assertEqual(node._mode, ModeManagerNode.MANUAL)

    # ---- Deadman ---------------------------------------------------------

    def test_deadman_drop_in_auto_pauses(self):
        node = _make_mode_manager()
        node._mode = ModeManagerNode.AUTO
        node._deadman_cb(_Bool(data=False))
        self.assertEqual(node._mode, ModeManagerNode.PAUSED)

    def test_deadman_restore_in_paused_resumes_auto(self):
        node = _make_mode_manager()
        node._mode = ModeManagerNode.PAUSED
        node._deadman_cb(_Bool(data=True))
        self.assertEqual(node._mode, ModeManagerNode.AUTO)

    def test_deadman_drop_in_manual_no_change(self):
        node = _make_mode_manager()
        node._mode = ModeManagerNode.MANUAL
        node._deadman_cb(_Bool(data=False))
        self.assertEqual(node._mode, ModeManagerNode.MANUAL)

    # ---- No mode change while disconnected --------------------------------

    def test_auto_button_ignored_when_disconnected(self):
        node = _make_mode_manager()
        node._connected = False
        node._btn_auto_cb(_Bool(data=True))
        self.assertEqual(node._mode, ModeManagerNode.MANUAL)


# ---------------------------------------------------------------------------
# CmdGateNode tests
# ---------------------------------------------------------------------------

class TestCmdGate(unittest.TestCase):

    def _last_published(self, node):
        return node._pub_safe.publish.call_args[0][0]

    # ---- Joystick lost ---------------------------------------------------

    def test_joystick_lost_publishes_zero(self):
        node = _make_cmd_gate()
        node._connected = False
        node._gate()
        out = self._last_published(node)
        self.assertEqual(out.linear.x, 0.0)

    # ---- Emergency stop --------------------------------------------------

    def test_estop_publishes_zero_even_in_manual(self):
        node = _make_cmd_gate()
        node._mode  = CmdGateNode.MANUAL
        node._estop = True
        manual_cmd  = _Twist()
        manual_cmd.linear.x = 1.0
        node._cmd_manual = manual_cmd
        node._gate()
        out = self._last_published(node)
        self.assertEqual(out.linear.x, 0.0)

    # ---- MANUAL mode -----------------------------------------------------

    def test_manual_mode_forwards_manual_cmd(self):
        node = _make_cmd_gate()
        node._mode = CmdGateNode.MANUAL
        manual_cmd = _Twist()
        manual_cmd.linear.x = 0.5
        node._cmd_manual = manual_cmd
        node._gate()
        out = self._last_published(node)
        self.assertIs(out, manual_cmd)

    def test_manual_mode_ignores_auto_cmd(self):
        node = _make_cmd_gate()
        node._mode = CmdGateNode.MANUAL
        auto_cmd = _Twist()
        auto_cmd.linear.x = 1.0
        node._cmd_auto = auto_cmd
        node._gate()
        out = self._last_published(node)
        self.assertIsNot(out, auto_cmd)

    # ---- AUTO mode with deadman ------------------------------------------

    def test_auto_mode_with_deadman_forwards_auto_cmd(self):
        node = _make_cmd_gate()
        node._mode       = CmdGateNode.AUTO
        node._deadman_ok = True
        auto_cmd = _Twist()
        auto_cmd.linear.x = 0.8
        node._cmd_auto = auto_cmd
        node._gate()
        out = self._last_published(node)
        self.assertIs(out, auto_cmd)

    def test_auto_mode_without_deadman_publishes_zero(self):
        node = _make_cmd_gate()
        node._mode       = CmdGateNode.AUTO
        node._deadman_ok = False
        auto_cmd = _Twist()
        auto_cmd.linear.x = 0.8
        node._cmd_auto = auto_cmd
        node._gate()
        out = self._last_published(node)
        # zero Twist is a different object
        self.assertIsNot(out, auto_cmd)

    # ---- PAUSED mode (no motion) ----------------------------------------

    def test_paused_mode_publishes_zero(self):
        node = _make_cmd_gate()
        node._mode       = 'PAUSED'
        node._deadman_ok = True
        auto_cmd = _Twist()
        auto_cmd.linear.x = 1.0
        node._cmd_auto = auto_cmd
        node._gate()
        out = self._last_published(node)
        self.assertIsNot(out, auto_cmd)

    # ---- State update callbacks -----------------------------------------

    def test_mode_cb_triggers_gate(self):
        node = _make_cmd_gate()
        node._mode_cb(_String(data='AUTO'))
        self.assertEqual(node._mode, 'AUTO')
        node._pub_safe.publish.assert_called()

    def test_deadman_cb_triggers_gate(self):
        node = _make_cmd_gate()
        node._deadman_cb(_Bool(data=True))
        self.assertTrue(node._deadman_ok)
        node._pub_safe.publish.assert_called()

    def test_connected_cb_false_triggers_zero(self):
        node = _make_cmd_gate()
        node._connected_cb(_Bool(data=False))
        out = self._last_published(node)
        # When disconnected, zero is published
        self.assertFalse(node._connected)

    def test_estop_cb_triggers_gate(self):
        node = _make_cmd_gate()
        node._estop_cb(_Bool(data=True))
        self.assertTrue(node._estop)
        node._pub_safe.publish.assert_called()


if __name__ == '__main__':
    unittest.main()
