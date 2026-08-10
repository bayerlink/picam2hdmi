"""The camera source's pure logic, off-target: no Pi, no camera needed."""
from __future__ import annotations

import numpy as np
import pytest

from bayerlink import decode_frame, encode_packed, pack_samples
from picam2hdmi.capture import (check_rate, crop_packed, order_from_format,
                                validate_crop)


def test_order_from_format_reads_libcamera_names():
    assert order_from_format("SBGGR10_CSI2P") == ("BGGR", 10)
    assert order_from_format("SRGGB12_CSI2P") == ("RGGB", 12)
    assert order_from_format("R8") == (None, 8)
    with pytest.raises(ValueError, match="not a packed"):
        order_from_format("SBGGR16")            # unpacked: refused


def test_crop_rules_are_the_packings():
    assert validate_crop((4, 2, 36, 240), (1296, 972), 10) == (4, 2, 36, 240)
    with pytest.raises(ValueError, match="multiples of 4"):
        validate_crop((2, 0, 36, 240), (1296, 972), 10)
    with pytest.raises(ValueError, match="even"):
        validate_crop((4, 1, 36, 240), (1296, 972), 10)
    with pytest.raises(ValueError, match="does not fit"):
        validate_crop((1280, 0, 36, 240), (1296, 972), 10)


def test_packed_crop_equals_sample_crop():
    """The byte slice must equal cropping samples then packing -- the
    whole point of group-aligned windows."""
    rng = np.random.default_rng(11)
    raw = rng.integers(0, 1024, (6, 64)).astype(np.uint16)
    packed = pack_samples(raw, 10)
    window = crop_packed(packed, 8, 24, 10)
    assert np.array_equal(window, pack_samples(raw[:, 8:32], 10))


def test_rate_rule_names_the_budget():
    check_rate(1296, (1920, 1080))               # binned mode fits
    with pytest.raises(ValueError, match="2200 line slots"):
        check_rate(2592, (1920, 1080))           # full-res does not


def test_sensor_bytes_ride_verbatim_to_a_decodable_container():
    """The synthetic end-to-end: stride padding, crop, encode_packed,
    decode_frame -- everything but the silicon."""
    rng = np.random.default_rng(12)
    sensor_w, sensor_h, bits = 64, 16, 10
    raw = rng.integers(0, 1024, (sensor_h, sensor_w)).astype(np.uint16)
    line_bytes = sensor_w * 5 // 4
    stride = line_bytes + 32                     # DMA alignment padding
    buffer = np.zeros((sensor_h, stride), np.uint8)
    buffer[:, :line_bytes] = pack_samples(raw, bits)

    x, y, w, h = 8, 2, 32, 12
    rows = buffer[y:y + h, :line_bytes]
    window = crop_packed(rows, x, w, bits)
    container = encode_packed(np.ascontiguousarray(window), "BGGR",
                              frame_seq=7, bits=bits, display=(64, 16))
    header, decoded = decode_frame(container)
    assert header.bits == 10 and header.bayer_order == "BGGR"
    assert np.array_equal(decoded, raw[y:y + h, x:x + w])
