# Copyright 2026 team18
"""Broadcast ``odom -> base_link`` TF from the chassis Odometry topic.

The course-provided ``aria_node`` publishes ``/odom`` but does not guarantee a
matching TF transform. Step 0 requires the minimal TF chain:

  odom -> base_link -> laser
  odom -> base_link -> camera_link

This node closes that gap by forwarding the pose embedded in ``/odom`` onto
``/tf`` using the frame names already carried by the Odometry message.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBroadcasterNode(Node):
    """Forward ``Odometry`` pose messages as TF transforms."""

    def __init__(self) -> None:
        super().__init__('odom_tf_broadcaster')
        self._tf_pub = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 20)

    def _odom_cb(self, msg: Odometry) -> None:
        """Broadcast odom pose as ``header.frame_id -> child_frame_id`` TF."""
        parent = (msg.header.frame_id or '').strip()
        child = (msg.child_frame_id or '').strip()
        if not parent or not child:
            self.get_logger().warn(
                'Skipping /odom message with empty frame_id or child_frame_id'
            )
            return

        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = parent
        tf_msg.child_frame_id = child
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self._tf_pub.sendTransform(tf_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OdomTfBroadcasterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
