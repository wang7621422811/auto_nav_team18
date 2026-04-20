#!/usr/bin/env python3
"""
sim_photo_trigger.py  —  Simulation-only photo-capture trigger (Step G3).

In the real robot, /waypoint/status is published by mission_controller.
In simulation the full mission stack isn't running, so this node provides
the trigger: when a cone detection arrives on /marker/bbox it publishes
"FINAL_APPROACH:0" to /waypoint/status once, causing PhotoCaptureNode
to save the annotated image to artifacts/photos/.

Run via sim_perception.launch.py — not intended for real-robot use.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimPhotoTrigger(Node):
    def __init__(self) -> None:
        super().__init__("sim_photo_trigger")
        self._triggered = False

        self._pub = self.create_publisher(String, "/waypoint/status", 10)
        self.create_subscription(String, "/marker/bbox", self._on_bbox, 10)
        self.get_logger().info(
            "SimPhotoTrigger ready — will fire FINAL_APPROACH:0 on first cone detection."
        )

    def _on_bbox(self, _msg: String) -> None:
        if self._triggered:
            return
        self._triggered = True
        self._pub.publish(String(data="FINAL_APPROACH:0"))
        self.get_logger().info(
            "Cone detected — published FINAL_APPROACH:0 to /waypoint/status. "
            "Photo should be saved to artifacts/photos/wp_00_annotated.jpg"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimPhotoTrigger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
