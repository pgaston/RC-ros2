"""Camera preview: which frames pass, and the JPEG made from each (issue #16).
Pure Python and Pillow; no ROS.
"""
import io

import pytest
from PIL import Image

from rc_hardware_control.camera_preview import FrameGate, encode_preview


def admitted(gate, times):
    return [t for t in times if gate.admit(t)]


def test_every_third_frame_of_a_15_hz_stream_passes_at_5_hz():
    frames = [i / 15.0 for i in range(15)]
    assert admitted(FrameGate(rate_hz=5.0), frames) == pytest.approx([0.0, 0.2, 0.4, 0.6, 0.8])


def test_arrival_jitter_does_not_skip_a_due_frame():
    # The third frame lands a few ms early; it is still the one due.
    frames = [0.0, 0.070, 0.195, 0.270, 0.330, 0.405]
    assert admitted(FrameGate(rate_hz=5.0), frames) == [0.0, 0.195, 0.405]


def test_time_running_backwards_passes_the_frame_and_starts_over():
    gate = FrameGate(rate_hz=5.0)
    assert admitted(gate, [10.0, 10.1, 3.0, 3.1, 3.2]) == [10.0, 3.0, 3.2]


def test_a_rate_that_is_not_positive_is_refused():
    with pytest.raises(ValueError):
        FrameGate(rate_hz=0.0)


def decoded(jpeg):
    image = Image.open(io.BytesIO(jpeg))
    assert image.format == 'JPEG'
    return image


def test_a_wide_mono_frame_is_halved_to_the_preview_width():
    data = bytes(range(256)) * (848 * 480 // 256) + bytes(848 * 480 % 256)
    image = decoded(encode_preview('mono8', 848, 480, 848, data, max_width=424, quality=60))
    assert image.size == (424, 240)
    assert image.mode == 'L'


def test_a_frame_no_wider_than_the_preview_is_not_enlarged():
    image = decoded(encode_preview('mono8', 320, 240, 320, bytes(320 * 240), max_width=424, quality=60))
    assert image.size == (320, 240)


def test_row_padding_beyond_the_pixels_is_ignored():
    # Four pixels per row, two padding bytes that would show as white stripes.
    row = bytes([0, 0, 0, 0, 255, 255])
    image = decoded(encode_preview('mono8', 4, 2, 6, row * 2, max_width=424, quality=95))
    assert image.size == (4, 2)
    assert max(image.getdata()) < 16


@pytest.mark.parametrize('encoding, pixel', [('rgb8', (255, 0, 0)), ('bgr8', (0, 0, 255))])
def test_colour_frames_keep_their_colours(encoding, pixel):
    image = decoded(encode_preview(encoding, 8, 8, 24, bytes(pixel) * 64, max_width=424, quality=95))
    red, green, blue = image.convert('RGB').getpixel((4, 4))
    assert red > 200 and green < 60 and blue < 60


def test_an_encoding_it_cannot_show_is_named():
    with pytest.raises(ValueError, match='16UC1'):
        encode_preview('16UC1', 4, 4, 8, bytes(32), max_width=424, quality=60)


@pytest.mark.parametrize('step, data_bytes, message', [
    (2, 8, 'needs 3 bytes a row'),          # rows shorter than the pixels
    (3, 8, 'needs 12 bytes; got 8'),        # buffer shorter than the frame
])
def test_a_frame_whose_buffer_does_not_fit_says_what_it_needed(step, data_bytes, message):
    with pytest.raises(ValueError, match=message):
        encode_preview('rgb8', 1, 4, step, bytes(data_bytes), max_width=424, quality=60)
