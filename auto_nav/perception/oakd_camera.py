#!/usr/bin/env python3
"""ROS 2 OAK-D camera node implemented directly with DepthAI.

This node replaces the external ``depthai_ros_driver`` dependency for the
project's minimal perception contract. It publishes:

  - ``/camera/color/image_raw`` as ``bgr8``
  - ``/camera/depth/image_raw`` as ``mono16``

All camera configuration is loaded from ``config/camera.yaml`` under the
``camera`` namespace so the existing config layering keeps working.
"""

from __future__ import annotations

import threading
import time

import depthai as dai
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class OakDCamera(Node):
    """Publish OAK-D colour and depth images using the DepthAI Python API."""

    def __init__(self) -> None:
        super().__init__("oakd_camera")

        self.declare_parameter("mx_id", "")
        self.declare_parameter("rgb_width", 1280)
        self.declare_parameter("rgb_height", 720)
        self.declare_parameter("rgb_fps", 15)
        self.declare_parameter("depth_fps", 15)
        self.declare_parameter("queue_size", 4)
        self.declare_parameter("retry_delay_s", 3.0)
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("color_frame", "camera_color_optical_frame")
        self.declare_parameter("depth_frame", "camera_depth_optical_frame")

        gp = self.get_parameter
        self._mx_id = gp("mx_id").get_parameter_value().string_value.strip()
        self._rgb_width = gp("rgb_width").get_parameter_value().integer_value
        self._rgb_height = gp("rgb_height").get_parameter_value().integer_value
        self._rgb_fps = gp("rgb_fps").get_parameter_value().integer_value
        self._depth_fps = gp("depth_fps").get_parameter_value().integer_value
        self._queue_size = gp("queue_size").get_parameter_value().integer_value
        self._retry_delay_s = gp("retry_delay_s").get_parameter_value().double_value
        self._camera_frame = gp("camera_frame").get_parameter_value().string_value
        self._color_frame = gp("color_frame").get_parameter_value().string_value
        self._depth_frame = gp("depth_frame").get_parameter_value().string_value

        self.rgb_pub = self.create_publisher(Image, "/camera/color/image_raw", 10)
        self.depth_pub = self.create_publisher(Image, "/camera/depth/image_raw", 10)

        self._stop_event = threading.Event()
        self._device = None
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

        self.get_logger().info(
            "OakDCamera ready "
            f"(rgb={self._rgb_width}x{self._rgb_height}@{self._rgb_fps}Hz, "
            f"depth={self._depth_fps}Hz, mx_id={'auto' if not self._mx_id else self._mx_id})"
        )

    def _build_pipeline(self) -> dai.Pipeline:
        """Construct the OAK-D RGB + depth pipeline."""
        pipeline = dai.Pipeline()

        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setInterleaved(False)
        cam_rgb.setVideoSize(self._rgb_width, self._rgb_height)
        cam_rgb.setFps(float(self._rgb_fps))

        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setFps(float(self._depth_fps))
        mono_right.setFps(float(self._depth_fps))

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.video.link(xout_rgb.input)

        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        return pipeline

    def _open_device(self, pipeline: dai.Pipeline) -> dai.Device:
        """Open a specific OAK-D if MX ID is set, otherwise use the first one."""
        if self._mx_id:
            return dai.Device(pipeline, dai.DeviceInfo(self._mx_id))
        return dai.Device(pipeline)

    def _capture_loop(self) -> None:
        """Reconnect automatically when the camera is unplugged or busy."""
        while not self._stop_event.is_set() and rclpy.ok():
            try:
                pipeline = self._build_pipeline()
                self.get_logger().info("Connecting to OAK-D...")
                with self._open_device(pipeline) as device:
                    self._device = device
                    self.get_logger().info(f"OAK-D connected: {device.getMxId()}")

                    q_rgb = device.getOutputQueue(
                        "rgb",
                        maxSize=self._queue_size,
                        blocking=False,
                    )
                    q_depth = device.getOutputQueue(
                        "depth",
                        maxSize=self._queue_size,
                        blocking=False,
                    )

                    while not self._stop_event.is_set() and rclpy.ok():
                        rgb_packet = q_rgb.tryGet()
                        if rgb_packet is not None:
                            self.rgb_pub.publish(
                                self._to_image_msg(
                                    frame=rgb_packet.getCvFrame(),
                                    encoding="bgr8",
                                    frame_id=self._color_frame,
                                )
                            )

                        depth_packet = q_depth.tryGet()
                        if depth_packet is not None:
                            self.depth_pub.publish(
                                self._to_image_msg(
                                    frame=depth_packet.getCvFrame(),
                                    encoding="mono16",
                                    frame_id=self._depth_frame,
                                )
                            )

                        time.sleep(0.01)

            except Exception as exc:  # noqa: BLE001
                self._device = None
                if self._stop_event.is_set() or not rclpy.ok():
                    break
                self.get_logger().warn(
                    f"OAK-D error: {exc}. Retrying in {self._retry_delay_s:.1f}s..."
                )
                time.sleep(self._retry_delay_s)

    def _to_image_msg(self, frame, encoding: str, frame_id: str) -> Image:
        """Convert a numpy frame from DepthAI into a ROS Image message."""
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id or self._camera_frame
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = 0
        if len(frame.shape) == 2:
            bytes_per_pixel = frame.dtype.itemsize
        else:
            bytes_per_pixel = frame.dtype.itemsize * frame.shape[2]
        msg.step = msg.width * bytes_per_pixel
        msg.data = frame.tobytes()
        return msg

    def destroy_node(self) -> bool:
        """Stop the capture thread before the ROS node is destroyed."""
        self._stop_event.set()
        if self._capture_thread.is_alive():
            self._capture_thread.join(timeout=2.0)
        self._device = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OakDCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
