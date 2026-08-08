"""rawlink v1 against its own spec: round trips, refusals, and the rate rule."""
import struct
import zlib

import numpy as np
import pytest

from picam2hdmi import pattern, protocol
from picam2hdmi.protocol import Header


# --------------------------------------------------------------------------- #
# 12P packing
# --------------------------------------------------------------------------- #

def test_pack12p_matches_the_spec_byte_for_byte():
    """The layout in PROTOCOL.md, checked against hand-computed bytes."""
    samples = np.array([0xABC, 0x123], dtype=np.uint16)
    packed = protocol.pack12p(samples)
    #   byte0 = P0[11:4] = 0xAB;  byte1 = P1[11:4] = 0x12
    #   byte2 = P1[3:0]<<4 | P0[3:0] = 0x3C
    assert packed.tolist() == [0xAB, 0x12, 0x3C]


@pytest.mark.parametrize("width,height", [(2, 1), (2028, 4), (16, 16)])
def test_pack12p_round_trips(width, height, rng=np.random.default_rng(20260808)):
    frame = rng.integers(0, 0x1000, (height, width)).astype(np.uint16)
    assert np.array_equal(protocol.unpack12p(protocol.pack12p(frame)), frame)


def test_pack12p_refuses_odd_width_and_wide_samples():
    with pytest.raises(ValueError, match="even"):
        protocol.pack12p(np.zeros((4, 3), np.uint16))
    with pytest.raises(ValueError, match="4095"):
        protocol.pack12p(np.array([0x1000, 0], np.uint16))


# --------------------------------------------------------------------------- #
# Header
# --------------------------------------------------------------------------- #

def test_header_round_trips_with_crc():
    header = Header(fourcc="pRCC", width=2028, height=1080,
                    frame_seq=42, flags=protocol.FLAG_TEST_PATTERN)
    again = Header.unpack(header.pack())
    assert again == header
    assert again.bayer_order == "RGGB"
    assert again.bayer_phase == 0b00
    assert again.is_test_pattern


def test_a_corrupted_header_is_refused_not_guessed():
    good = bytearray(Header(fourcc="pBCC", width=4, height=2,
                            frame_seq=0).pack())
    flipped = bytearray(good)
    flipped[12] ^= 0x01                          # width, inside the CRC
    with pytest.raises(ValueError, match="CRC"):
        Header.unpack(bytes(flipped))
    wrong_magic = bytearray(good)
    wrong_magic[0] ^= 0xFF
    with pytest.raises(ValueError, match="magic"):
        Header.unpack(bytes(wrong_magic))


def test_an_unknown_version_or_fourcc_is_refused():
    body = struct.pack("<IHHIIIII", protocol.MAGIC, 2, 32,
                       protocol.fourcc_code("pRCC"), 4, 2, 0, 0)
    with pytest.raises(ValueError, match="version 2"):
        Header.unpack(body + struct.pack("<I", zlib.crc32(body)))
    body = struct.pack("<IHHIIIII", protocol.MAGIC, 1, 32,
                       protocol.fourcc_code("BA10"), 4, 2, 0, 0)
    with pytest.raises(ValueError, match="not implemented"):
        Header.unpack(body + struct.pack("<I", zlib.crc32(body)))


def test_every_bayer_order_maps_to_its_phase_bits():
    for order, phase in {"RGGB": 0b00, "GRBG": 0b01,
                         "GBRG": 0b10, "BGGR": 0b11}.items():
        header = Header(fourcc=protocol.fourcc_for(order), width=4, height=2,
                        frame_seq=0)
        assert header.bayer_phase == phase, order


# --------------------------------------------------------------------------- #
# The frame container
# --------------------------------------------------------------------------- #

def test_encode_decode_round_trips_at_reference_geometry():
    """The reference setup: HQ camera at 2028x1078 inside 1080p.

    1078, not 1080: the header line costs one display line, so an N-line
    display carries at most N-1 camera lines — the first bug this suite
    caught, pinned by the refusal test below.
    """
    raw = pattern.generate("corners", 2028, 1078)
    frame = protocol.encode_frame(raw, "RGGB", frame_seq=7)
    assert frame.shape == (1080, 1920, 3) and frame.dtype == np.uint8

    header, decoded = protocol.decode_frame(frame)
    assert np.array_equal(decoded, raw)
    assert (header.width, header.height, header.frame_seq) == (2028, 1078, 7)

    # Padding really is padding: beyond the payload bytes, zeros only.
    lines = frame.reshape(1080, 1920 * 3)
    assert not lines[1:, header.line_bytes:].any()
    assert not lines[0, protocol.HEADER_BYTES:].any()


def test_a_full_height_frame_is_refused_the_header_line_is_not_free():
    with pytest.raises(ValueError, match="header line"):
        protocol.encode_frame(np.zeros((1080, 2028), np.uint16), "RGGB",
                              frame_seq=0)


def test_geometry_that_does_not_fit_is_refused_before_encoding():
    with pytest.raises(ValueError, match="display line carries"):
        protocol.check_geometry(3842, 100, (1920, 1080))
    with pytest.raises(ValueError, match="header line"):
        protocol.check_geometry(2028, 1080, (1920, 1080 - 1))


def test_the_rate_rule_matches_the_worked_examples():
    """PROTOCOL.md's examples, held as assertions: 1080p fits, 720p does not."""
    assert protocol.fits_line_rate(2028, total_line_slots=2200)      # 1080p
    assert not protocol.fits_line_rate(2028, total_line_slots=1650)  # 720p


def test_every_pattern_survives_the_full_container_round_trip():
    for name in pattern.PATTERNS:
        raw = pattern.generate(name, 64, 32)
        header, decoded = protocol.decode_frame(
            protocol.encode_frame(raw, "BGGR", frame_seq=1,
                                  display=(64, 40),
                                  flags=protocol.FLAG_TEST_PATTERN))
        assert np.array_equal(decoded, raw), name
        assert header.is_test_pattern


def test_patterns_hit_the_values_a_bad_link_clamps():
    """checker and corners must contain 0 and 4095 -- the limited-range
    casualties -- or the phase-0 loopback cannot catch a clamped link."""
    for name in ("checker", "corners"):
        raw = pattern.generate(name, 64, 32)
        assert raw.min() == 0 and raw.max() == 0xFFF, name
