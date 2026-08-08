"""Scanout: putting encoded frames on the HDMI connector.

NOT IMPLEMENTED YET -- this module states the intended design so the work is
specified before it is written. The protocol and patterns are complete and
tested; this is the next milestone, and it only runs on a Pi.

Intended design
---------------

DRM/KMS directly (no X, no Wayland, no desktop):

- open the DRM device, pick the connector's mode (1080p30 for the reference
  setup -- see PROTOCOL.md's rate rule for why not 720p);
- create TWO dumb buffers in the container format and page-flip between
  them on vsync: the classic double buffer. A flip replaces the WHOLE frame
  atomically, so a receiver never sees half an update -- the same reasoning
  as shadow registers, applied to pixels;
- force FULL-RANGE RGB on the connector ("Broadcast RGB" property where the
  driver exposes it, plus the firmware setting), because a limited-range
  link clamps exactly the values raw data lives on;
- with a camera attached, import the capture buffer as a dmabuf and flip it
  directly where stride and format allow -- zero copies from sensor DMA to
  scanout. Where strides disagree, fall back to one memcpy per line.

The container format is chosen so the MEMORY byte order matches PROTOCOL.md;
whether that is DRM_FORMAT_RGB888 or BGR888 on a given platform is resolved
once, in the phase-0 pattern loopback, and recorded in the board notes.
"""
from __future__ import annotations


def stream(frames, mode=(1920, 1080, 30)):  # pragma: no cover - hardware only
    """Scan an iterable of encoded frames out over HDMI. Pi-only; see above."""
    raise NotImplementedError(
        "KMS scanout is the next milestone; the module docstring is its "
        "design. Until then, `picam2hdmi pattern --out frame.npy` produces "
        "the exact bytes this will display.")
