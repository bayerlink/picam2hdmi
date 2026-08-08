"""picam2hdmi -- a Raspberry Pi camera as a bayerlink HDMI source.

The wire format is the bayerlink protocol (github.com/bayerlink/bayerlink);
this tool is its reference ENCODER for the Pi. The protocol package runs on
both ends of the link, so the container-building lives there and what lives
here is only what is the Pi's: capture (Picamera2) and scanout (KMS).
"""
from bayerlink import (  # noqa: F401  (re-exported for convenience)
    Header,
    decode_frame,
    encode_frame,
    pattern,
)

__version__ = "0.1.0"

__all__ = ["Header", "encode_frame", "decode_frame", "pattern"]
