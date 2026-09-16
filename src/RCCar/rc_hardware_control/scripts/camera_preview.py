#!/usr/bin/env python3
"""Camera preview: a small, slow JPEG view of the camera for Foxglove over wifi (issue #16).

The full-rate images are far more than the car's wifi link carries (about
20 Mbit/s for the infra stream against ~7 Mbit/s, 2026-09-16), and the view
fell seconds behind. This keeps about rate_hz frames a second, scales them to
max_width and publishes each as a JPEG CompressedImage with the frame's header.
Frames that are not due are dropped before they are scaled or encoded; they are
still delivered to this process, so the camera's publisher serialises every one.

The default input is visual SLAM's emitter-off infra1 stream, which has no IR
dot pattern (issue #15). Supported encodings: mono8, rgb8, bgr8. A frame it
cannot show is skipped; each distinct reason is logged once.

Parameters: input_topic, output_topic, rate_hz, max_width, jpeg_quality.
"""
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

from rc_hardware_control.camera_preview import FrameGate, encode_preview


class CameraPreview(Node):
    def __init__(self):
        super().__init__('camera_preview')
        input_topic = self._parameter('input_topic', '/emitter_off/infra1/image_rect_raw')
        output_topic = self._parameter('output_topic', '/camera_preview/image/compressed')
        rate_hz = self._parameter('rate_hz', 5.0)
        self._max_width = self._parameter('max_width', 424)
        self._quality = self._parameter('jpeg_quality', 60)

        self._gate = FrameGate(rate_hz)
        self._warned = set()
        # Best effort both ways: a late preview frame is worth nothing.
        self._publisher = self.create_publisher(CompressedImage, output_topic, qos_profile_sensor_data)
        self.create_subscription(Image, input_topic, self._on_image, qos_profile_sensor_data)
        self.get_logger().info(
            f'{input_topic} -> {output_topic}: {rate_hz:g} Hz, at most {self._max_width} px wide, '
            f'JPEG quality {self._quality}')

    def _parameter(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _on_image(self, msg: Image):
        if not self._gate.admit(time.monotonic()):
            return
        try:
            jpeg = encode_preview(msg.encoding, msg.width, msg.height, msg.step, msg.data,
                                  self._max_width, self._quality)
        except ValueError as error:
            reason = str(error)
            if reason not in self._warned:
                self._warned.add(reason)
                self.get_logger().warning(f'skipping frames: {reason}')
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = jpeg
        self._publisher.publish(out)


def main():
    rclpy.init()
    node = CameraPreview()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
