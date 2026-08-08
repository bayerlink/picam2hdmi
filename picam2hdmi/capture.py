"""Capture: raw Bayer frames from the camera, via the Linux camera stack.

NOT IMPLEMENTED YET -- this module states the intended design so the work is
specified before it is written. It only runs on a Pi with a camera attached.

Intended design
---------------

Picamera2 (libcamera), raw stream only:

- configure the RAW stream at the chosen sensor mode (for the reference
  setup: the HQ camera's 2028x1080 12-bit mode -- 2028 obeys the 1080p rate
  rule in PROTOCOL.md);
- the ISP is NOT in the loop: the raw stream is the sensor's own packed
  output, DMA-written by the CSI receiver. The Bayer order and bit depth
  come from libcamera's reported format and go straight into the rawlink
  fourcc -- this tool never holds a sensor register table, which is the
  point: the kernel drivers that know those tables run in THEIR project,
  under their licence, and every camera libcamera supports works here
  without this tool learning anything about it;
- hand each completed buffer to output.stream(), preferably as a dmabuf so
  the scanout flips the capture buffer itself (zero copies), otherwise via
  one packed-line copy into the container;
- stamp frame_seq once per delivered camera frame, so receivers can
  deduplicate scanout repeats.

Auto-exposure and auto-white-balance are DISABLED by default: a raw source
should be deterministic unless the operator asks otherwise. Fixed exposure
and gain are CLI arguments.
"""
from __future__ import annotations


def frames(mode=None, exposure_us=None, analogue_gain=None):  # pragma: no cover
    """Yield (raw_frame, bayer_order, frame_seq) from the camera. Pi-only."""
    raise NotImplementedError(
        "Picamera2 capture is the milestone after scanout; the module "
        "docstring is its design. Until then, pattern.generate() supplies "
        "frames with the same shape and dtype.")
