"""
HomePoseRecorderNode — subscribes to /odom and persists the first received pose
as the mission home pose.

Publishes:
  /mission/home_pose  (geometry_msgs/PoseStamped)  latched, once on first odom.

Saves:
  <home_pose_file>  (YAML)  for offline use / replanning.

Parameters (from mission.yaml):
  home_pose_file  — path to output YAML file
"""

import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class HomePoseRecorderNode(Node):
    def __init__(self):
        super().__init__('home_pose_recorder')

        self.declare_parameter('home_pose_file', 'artifacts/logs/home_pose.yaml')
        self._home_pose_file: str = (
            self.get_parameter('home_pose_file').get_parameter_value().string_value
        )

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(
            PoseStamped, '/mission/home_pose', latched_qos
        )

        self._sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10
        )

        self._recorded = False
        self.get_logger().info(
            f'HomePoseRecorder ready — will save to {self._home_pose_file!r}'
        )

    # ------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry) -> None:
        if self._recorded:
            return  # only capture once per session

        self._recorded = True
        pose = msg.pose.pose

        stamped = PoseStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = msg.header.frame_id or 'odom'
        stamped.pose = pose

        self._pub.publish(stamped)

        self._save_yaml(stamped)

        self.get_logger().info(
            f'Home pose recorded: '
            f'x={pose.position.x:.3f}  '
            f'y={pose.position.y:.3f}  '
            f'z={pose.position.z:.3f}'
        )

    # ------------------------------------------------------------------
    def _save_yaml(self, stamped: PoseStamped) -> None:
        p = stamped.pose.position
        q = stamped.pose.orientation
        data = {
            'frame_id': stamped.header.frame_id,
            'stamp': {
                'sec': stamped.header.stamp.sec,
                'nanosec': stamped.header.stamp.nanosec,
            },
            'position': {'x': float(p.x), 'y': float(p.y), 'z': float(p.z)},
            'orientation': {
                'x': float(q.x),
                'y': float(q.y),
                'z': float(q.z),
                'w': float(q.w),
            },
        }

        try:
            os.makedirs(os.path.dirname(os.path.abspath(self._home_pose_file)), exist_ok=True)
            with open(self._home_pose_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
            self.get_logger().info(f'Home pose saved → {self._home_pose_file}')
        except OSError as e:
            self.get_logger().error(f'Failed to save home pose: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = HomePoseRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
