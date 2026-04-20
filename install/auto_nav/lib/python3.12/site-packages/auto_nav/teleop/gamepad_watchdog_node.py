"""
GamepadWatchdogNode — monitors /joy and declares joystick connected/disconnected.

Publishes:
  /deadman_ok  (std_msgs/Bool)  True while joystick is alive and dead-man pressed.
  /joy_connected (std_msgs/Bool) True while /joy messages arrive within timeout.

All timing and button indices come from joystick.yaml — nothing hardcoded.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class GamepadWatchdogNode(Node):
    def __init__(self):
        super().__init__('gamepad_watchdog')

        # Parameters from joystick.yaml (all have safe defaults)
        self.declare_parameter('axis_deadman', 5)
        self.declare_parameter('deadman_threshold', 0.5)
        self.declare_parameter('joy_timeout_sec', 1.0)

        self._axis_deadman: int = (
            self.get_parameter('axis_deadman').get_parameter_value().integer_value
        )
        self._deadman_threshold: float = (
            self.get_parameter('deadman_threshold').get_parameter_value().double_value
        )
        self._timeout: float = (
            self.get_parameter('joy_timeout_sec').get_parameter_value().double_value
        )

        # Latched QoS so new subscribers immediately get current state
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub_deadman = self.create_publisher(Bool, '/deadman_ok', latched_qos)
        self._pub_connected = self.create_publisher(Bool, '/joy_connected', latched_qos)

        self._sub_joy = self.create_subscription(Joy, '/joy', self._joy_cb, 10)

        self._last_joy_time = self.get_clock().now()
        self._connected = False
        self._deadman_ok = False

        # Watchdog timer fires at 2× the timeout rate for responsiveness
        timer_period = max(0.1, self._timeout / 2.0)
        self._timer = self.create_timer(timer_period, self._watchdog_tick)

        self.get_logger().info(
            f'GamepadWatchdog ready — timeout={self._timeout}s, '
            f'deadman axis={self._axis_deadman}, '
            f'threshold={self._deadman_threshold}'
        )

    # ------------------------------------------------------------------
    def _joy_cb(self, msg: Joy) -> None:
        self._last_joy_time = self.get_clock().now()

        if not self._connected:
            self._connected = True
            self.get_logger().info('Joystick CONNECTED')

        # Evaluate dead-man: trigger axis must exceed threshold
        deadman_active = False
        if self._axis_deadman < len(msg.axes):
            deadman_active = msg.axes[self._axis_deadman] > self._deadman_threshold

        if deadman_active != self._deadman_ok:
            self._deadman_ok = deadman_active
            self.get_logger().debug(f'Dead-man state changed → {self._deadman_ok}')

        self._publish_state()

    # ------------------------------------------------------------------
    def _watchdog_tick(self) -> None:
        elapsed = (
            self.get_clock().now() - self._last_joy_time
        ).nanoseconds * 1e-9

        if elapsed > self._timeout:
            if self._connected or self._deadman_ok:
                self.get_logger().warn(
                    f'Joystick DISCONNECTED — no /joy for {elapsed:.2f}s'
                )
            self._connected = False
            self._deadman_ok = False

        self._publish_state()

    # ------------------------------------------------------------------
    def _publish_state(self) -> None:
        self._pub_connected.publish(Bool(data=self._connected))
        self._pub_deadman.publish(Bool(data=self._deadman_ok))


def main(args=None):
    rclpy.init(args=args)
    node = GamepadWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
