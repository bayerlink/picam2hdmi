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

# The one version statement is pyproject.toml; this reads it.
from importlib.metadata import version as _version
try:
    __version__ = _version("picam2hdmi")
except Exception:
    __version__ = "unknown"

__all__ = ["Header", "encode_frame", "decode_frame", "pattern"]
