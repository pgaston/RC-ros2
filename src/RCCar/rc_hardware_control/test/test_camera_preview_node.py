"""Camera preview through its ROS interface: a 15 Hz infra stream comes out as
a 5 Hz, half-width JPEG with the frame's header; a frame it cannot show is
skipped without stopping the stream. Needs rclpy (the Isaac image); about 10 s.
"""
import array
import io
import os
import pathlib
import signal
import subprocess
import sys
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from PIL import Image as PilImage  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import CompressedImage, Image  # noqa: E402

PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
PREVIEW = PACKAGE_ROOT / 'scripts' / 'camera_preview.py'
INPUT = '/emitter_off/infra1/image_rect_raw'
OUTPUT = '/camera_preview/image/compressed'


def wait_until(predicate, timeout_s, what):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError(f'timed out waiting for {what}')


def frame(index, encoding='mono8'):
    msg = Image()
    msg.header.frame_id = 'camera_infra1_optical_frame'
    msg.header.stamp.sec = 1000 + index
    msg.encoding = encoding
    if encoding == 'mono8':
        msg.width, msg.height, msg.step = 848, 480, 848
    else:   # one the preview cannot show; kept small
        msg.width, msg.height, msg.step = 16, 16, 32
    # An array: assigning bytes converts byte by byte, about 75 ms a frame.
    msg.data = array.array('B', bytes([index % 256]) * (msg.step * msg.height))
    return msg


def test_a_15_hz_stream_comes_out_at_5_hz_and_half_width():
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    env = {**os.environ, 'PYTHONPATH': f"{PACKAGE_ROOT}{os.pathsep}{os.environ.get('PYTHONPATH', '')}"}
    preview = subprocess.Popen([sys.executable, str(PREVIEW)], env=env)
    rclpy.init()
    node = rclpy.create_node('camera_preview_test')
    try:
        images = node.create_publisher(Image, INPUT, 10)
        received = []
        node.create_subscription(CompressedImage, OUTPUT, received.append, qos_profile_sensor_data)

        wait_until(lambda: images.get_subscription_count() and node.count_publishers(OUTPUT), 20.0,
                   'the preview to subscribe and advertise')

        def spin_until(end):
            while (left := end - time.monotonic()) > 0:
                rclpy.spin_once(node, timeout_sec=left)

        def spin_for(seconds):
            spin_until(time.monotonic() + seconds)

        def stream(indices, encoding):
            # 15 Hz on an absolute schedule, so publishing time does not slow it.
            start = time.monotonic()
            for n, i in enumerate(indices):
                images.publish(frame(i, encoding))
                spin_until(start + (n + 1) / 15.0)

        # Best effort drops what is sent before the two ends have matched, so
        # warm up until a preview arrives.
        give_up = time.monotonic() + 20.0
        while not received:
            assert time.monotonic() < give_up, 'no preview during warm-up'
            stream(range(200, 215), 'mono8')
        spin_for(0.3)
        received.clear()

        # Frames it cannot show for half a second, then 2.5 s of good ones.
        stream(range(0, 8), '16UC1')
        stream(range(8, 45), 'mono8')
        spin_for(1.0)

        stamps = [m.header.stamp.sec - 1000 for m in received]
        assert all(8 <= s < 45 for s in stamps)
        assert stamps == sorted(stamps)
        # 37 frames at 15 Hz is 2.5 s; at 5 Hz about 12, allowing for scheduling.
        assert 10 <= len(stamps) <= 14
        for msg in received:
            assert msg.format == 'jpeg'
            assert msg.header.frame_id == 'camera_infra1_optical_frame'
            image = PilImage.open(io.BytesIO(bytes(msg.data)))
            assert image.size == (424, 240)
            # The pixels are the frame the header names.
            assert abs(image.getpixel((212, 120)) - (msg.header.stamp.sec - 1000)) <= 2
        assert preview.poll() is None
    finally:
        preview.send_signal(signal.SIGINT)
        try:
            preview.wait(timeout=5)
        except subprocess.TimeoutExpired:
            preview.kill()
        node.destroy_node()
        rclpy.try_shutdown()
