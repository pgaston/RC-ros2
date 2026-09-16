"""A small, slow camera view for a thin link (issue #16).

Pure Python and Pillow, no ROS: the node asks the gate whether a frame arriving
now is due, and turns the due ones into a downscaled JPEG. Frames that are not
due are dropped before they are scaled or encoded.
"""
import io

from PIL import Image

# sensor_msgs/Image encoding: (Pillow mode, Pillow raw mode, bytes per pixel)
_ENCODINGS = {
    'mono8': ('L', 'L', 1),
    'rgb8': ('RGB', 'RGB', 3),
    'bgr8': ('RGB', 'BGR', 3),
}


class FrameGate:
    """Passes at most about rate_hz frames per second, dropping the rest.

    A frame passes once 80% of the period has gone by since the last one that
    passed, so arrival jitter never skips the frame that is due: a 15 Hz stream
    at 5 Hz keeps exactly every third frame. Time running backwards passes the
    frame and starts over.
    """

    SLACK = 0.8

    def __init__(self, rate_hz: float):
        if rate_hz <= 0.0:
            raise ValueError('the preview rate must be positive')
        self._min_gap_s = self.SLACK / rate_hz
        self._last_s = None

    def admit(self, now_s: float) -> bool:
        if self._last_s is not None and 0.0 <= now_s - self._last_s < self._min_gap_s:
            return False
        self._last_s = now_s
        return True


def encode_preview(encoding: str, width: int, height: int, step: int, data,
                   max_width: int, quality: int) -> bytes:
    """A JPEG of the frame, scaled down to max_width if wider, aspect kept.

    encoding, width, height, step and data are the sensor_msgs/Image fields;
    data is any buffer. Raises ValueError for an encoding it cannot show or a
    buffer too short for the frame.
    """
    if encoding not in _ENCODINGS:
        raise ValueError(f'cannot preview {encoding!r} images; supported: {", ".join(_ENCODINGS)}')
    mode, raw_mode, pixel_bytes = _ENCODINGS[encoding]
    if step < width * pixel_bytes:
        raise ValueError(f'{width}x{height} {encoding} needs {width * pixel_bytes} bytes a row; '
                         f'step is {step}')
    if len(data) < step * height:
        raise ValueError(f'{width}x{height} {encoding} needs {step * height} bytes; got {len(data)}')
    image = Image.frombuffer(mode, (width, height), memoryview(data), 'raw', raw_mode, step, 1)
    if width > max_width:
        image = image.resize((max_width, max(1, round(height * max_width / width))), Image.BILINEAR)
    out = io.BytesIO()
    image.save(out, format='JPEG', quality=quality)
    return out.getvalue()
