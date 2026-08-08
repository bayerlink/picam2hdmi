"""picam2hdmi -- a Raspberry Pi camera as a raw-Bayer HDMI source.

Raw sensor frames travel the HDMI link as bytes, self-described by one
header line: the rawlink protocol, specified in PROTOCOL.md and implemented
in .protocol -- which runs on BOTH ends of the link, encoding on the Pi and
decoding whatever the receiver captured.
"""
from .protocol import (
    BAYER_PHASE,
    FLAG_TEST_PATTERN,
    Header,
    check_geometry,
    decode_frame,
    encode_frame,
    fits_line_rate,
    fourcc_for,
    pack12p,
    unpack12p,
)
from . import pattern

__version__ = "0.1.0"

__all__ = [
    "Header", "encode_frame", "decode_frame", "pack12p", "unpack12p",
    "check_geometry", "fits_line_rate", "fourcc_for",
    "BAYER_PHASE", "FLAG_TEST_PATTERN", "pattern",
]
