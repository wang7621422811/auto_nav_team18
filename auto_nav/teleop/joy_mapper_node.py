"""
JoyMapperNode — translates raw /joy into hardware-agnostic semantic button topics.

Different gamepad clones (DualShock, Switch Pro, generic BT) use different button
indices.  All indices live in joystick.yaml — nothing is hardcoded here.

Subscribes:
  /joy  (sensor_msgs/Joy)

Publishes:
  /joy/btn_auto      (std_msgs/Bool)  True while AUTO-mode button is held
  /joy/btn_manual    (std_msgs/Bool)  True while MANUAL-mode button is held
  /joy/btn_emergency (std_msgs/Bool)  True while emergency-stop button is held
                                      (-1 button index disables the mapping)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class JoyMapperNode(Node):
    def __init__(self):
        super().__init__('joy_mapper')

        self.declare_parameter('btn_auto_mode', 0)
        self.declare_parameter('btn_manual_mode', 1)
        self.declare_parameter('btn_emergency_stop', 3)

        self._btn_auto: int = (
            self.get_parameter('btn_auto_mode').get_parameter_value().integer_value
        )
        self._btn_manual: int = (
            self.get_parameter('btn_manual_mode').get_parameter_value().integer_value
        )
        self._btn_emergency: int = (
            self.get_parameter('btn_emergency_stop').get_parameter_value().integer_value
        )

        # Latched so late-joining subscribers get the latest state immediately
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub_auto = self.create_publisher(Bool, '/joy/btn_auto', latched_qos)
        self._pub_manual = self.create_publisher(Bool, '/joy/btn_manual', latched_qos)
        self._pub_emergency = self.create_publisher(Bool, '/joy/btn_emergency', latched_qos)

        self._sub_joy = self.create_subscription(Joy, '/joy', self._joy_cb, 10)

        self.get_logger().info(
            f'JoyMapper ready — btn_auto={self._btn_auto}, '
            f'btn_manual={self._btn_manual}, '
            f'btn_emergency={self._btn_emergency}'
        )

    def _joy_cb(self, msg: Joy) -> None:
        def _pressed(idx: int) -> bool:
            """Return True if button index is valid, enabled, and pressed."""
            return bool(msg.buttons[idx]) if 0 <= idx < len(msg.buttons) else False

        self._pub_auto.publish(Bool(data=_pressed(self._btn_auto)))
        self._pub_manual.publish(Bool(data=_pressed(self._btn_manual)))
        self._pub_emergency.publish(Bool(data=_pressed(self._btn_emergency)))


def main(args=None):
    rclpy.init(args=args)
    node = JoyMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
