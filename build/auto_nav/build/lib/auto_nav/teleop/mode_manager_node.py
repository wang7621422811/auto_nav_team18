"""
ModeManagerNode — tracks driving mode and publishes /control_mode.

Transitions:
  MANUAL  ─► AUTO    : btn_auto pressed (rising edge) while joystick connected
  AUTO    ─► PAUSED  : deadman_ok drops to False while in AUTO
  PAUSED  ─► AUTO    : deadman_ok rises to True while in PAUSED
  any     ─► MANUAL  : btn_manual pressed (rising edge)
  any     ─► MANUAL  : joystick disconnects (/joy_connected → False)
  any     ─► ABORTED : btn_emergency pressed (rising edge); also latches /emergency_stop

Subscribes:
  /joy/btn_auto      (std_msgs/Bool)
  /joy/btn_manual    (std_msgs/Bool)
  /joy/btn_emergency (std_msgs/Bool)
  /joy_connected     (std_msgs/Bool)
  /deadman_ok        (std_msgs/Bool)

Publishes:
  /control_mode   (std_msgs/String)  — MANUAL | AUTO | PAUSED | ABORTED
  /emergency_stop (std_msgs/Bool)    — latched True on E-stop; cleared on MANUAL
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class ModeManagerNode(Node):

    MANUAL  = 'MANUAL'
    AUTO    = 'AUTO'
    PAUSED  = 'PAUSED'
    ABORTED = 'ABORTED'

    def __init__(self):
        super().__init__('mode_manager')

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub_mode = self.create_publisher(String, '/control_mode', latched_qos)
        self._pub_estop = self.create_publisher(Bool, '/emergency_stop', latched_qos)

        self.create_subscription(Bool, '/joy/btn_auto',      self._btn_auto_cb,      latched_qos)
        self.create_subscription(Bool, '/joy/btn_manual',    self._btn_manual_cb,    latched_qos)
        self.create_subscription(Bool, '/joy/btn_emergency', self._btn_emergency_cb, latched_qos)
        self.create_subscription(Bool, '/joy_connected',     self._joy_connected_cb, latched_qos)
        self.create_subscription(Bool, '/deadman_ok',        self._deadman_cb,       latched_qos)

        # Internal state
        self._mode = self.MANUAL
        self._prev_btn_auto      = False
        self._prev_btn_manual    = False
        self._prev_btn_emergency = False
        self._connected          = True   # optimistic until first message
        self._deadman_ok         = False
        self._estop_latched      = False

        # Publish initial state
        self._publish_mode()
        self._pub_estop.publish(Bool(data=False))

        self.get_logger().info('ModeManager ready — initial mode: MANUAL')

    # ------------------------------------------------------------------
    # Button callbacks — react on rising edges only to avoid mode flapping
    # ------------------------------------------------------------------

    def _btn_auto_cb(self, msg: Bool) -> None:
        pressed = msg.data
        rising  = pressed and not self._prev_btn_auto
        self._prev_btn_auto = pressed

        if rising and self._connected and self._mode not in (self.ABORTED,):
            self._set_mode(self.AUTO)

    def _btn_manual_cb(self, msg: Bool) -> None:
        pressed = msg.data
        rising  = pressed and not self._prev_btn_manual
        self._prev_btn_manual = pressed

        if rising:
            self._estop_latched = False
            self._pub_estop.publish(Bool(data=False))
            self._set_mode(self.MANUAL)

    def _btn_emergency_cb(self, msg: Bool) -> None:
        pressed = msg.data
        rising  = pressed and not self._prev_btn_emergency
        self._prev_btn_emergency = pressed

        if rising:
            self._estop_latched = True
            self._pub_estop.publish(Bool(data=True))
            self._set_mode(self.ABORTED)
            self.get_logger().error('EMERGENCY STOP triggered!')

    # ------------------------------------------------------------------
    # Connectivity / dead-man callbacks
    # ------------------------------------------------------------------

    def _joy_connected_cb(self, msg: Bool) -> None:
        was_connected   = self._connected
        self._connected = msg.data

        if was_connected and not self._connected:
            self.get_logger().warn('Joystick disconnected — forcing MANUAL')
            self._set_mode(self.MANUAL)

    def _deadman_cb(self, msg: Bool) -> None:
        self._deadman_ok = msg.data

        if self._mode == self.AUTO and not self._deadman_ok:
            self._set_mode(self.PAUSED)
        elif self._mode == self.PAUSED and self._deadman_ok:
            self._set_mode(self.AUTO)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _set_mode(self, new_mode: str) -> None:
        if new_mode != self._mode:
            self.get_logger().info(f'Mode: {self._mode} → {new_mode}')
        self._mode = new_mode
        self._publish_mode()

    def _publish_mode(self) -> None:
        self._pub_mode.publish(String(data=self._mode))


def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
