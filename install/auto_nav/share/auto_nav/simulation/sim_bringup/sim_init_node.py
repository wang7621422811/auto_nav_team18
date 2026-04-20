"""
sim_init_node — simulation-only state initialiser.

In real operation, a joystick driver publishes /joy_connected and
gamepad_watchdog publishes /deadman_ok.  In simulation those drivers
don't exist, so this node holds the latched state that cmd_gate expects:

  /joy_connected  Bool   True    — "joystick present" (sim has no HW req.)
  /deadman_ok     Bool   True    — deadman always held in sim
  /control_mode   String MANUAL  — start in keyboard-teleop mode

All three topics use TRANSIENT_LOCAL / RELIABLE QoS so any node that
starts later will still receive the latest value.

This node re-publishes every second to survive late-starting subscribers.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class SimInitNode(Node):

    def __init__(self) -> None:
        super().__init__('sim_init')

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub_connected = self.create_publisher(Bool,   '/joy_connected', latched)
        self._pub_deadman   = self.create_publisher(Bool,   '/deadman_ok',    latched)
        self._pub_mode      = self.create_publisher(String, '/control_mode',  latched)

        # Publish immediately, then refresh every second
        self._publish()
        self.create_timer(1.0, self._publish)

        self.get_logger().info(
            'SimInit: joy_connected=True  deadman_ok=True  control_mode=MANUAL'
        )

    def _publish(self) -> None:
        self._pub_connected.publish(Bool(data=True))
        self._pub_deadman.publish(Bool(data=True))
        mode = String()
        mode.data = 'MANUAL'
        self._pub_mode.publish(mode)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimInitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
