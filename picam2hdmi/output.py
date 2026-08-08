"""Scanout: bayerlink containers onto the HDMI connector, double-buffered.

The kms module owns the DRM mechanics; this one owns the bayerlink shape of
the job: write a (height, width, 3) container into whichever pixel format
the driver granted, patch the header's frame counter per frame, and flip on
vsync. Draw-then-flip-then-wait, always into the buffer the display just
left -- a receiver never sees a torn container, for the same reason the
register file on the far end commits on frame boundaries.

Implemented and testable off-target down to the ioctl layer; the first
on-Pi run is part of the phase-0 bring-up session, and until then this
module's claim is "written and reviewed", not "proven". The pure parts
(container packing for both formats, header patching, mode choice) ARE
proven, by the test suite, off-target.
"""
from __future__ import annotations

import dataclasses

import numpy as np

from bayerlink.protocol import HEADER_BYTES, Header

from . import kms


def write_container(framebuffer: "kms.Framebuffer", container: np.ndarray,
                    pixel_format: int) -> None:
    """One container into one dumb buffer, honouring pitch and format.

    RG24 is the container memcpy'd row by row (pitch may pad each line).
    XR24 spreads the three container bytes across four-byte pixels; the
    channel each byte lands on is the same in both formats, so the
    receiver's lane_map never depends on which format the driver granted.
    """
    height, width, _ = container.shape
    rows = np.frombuffer(framebuffer.view, dtype=np.uint8)
    rows = rows[:height * framebuffer.pitch].reshape(height, framebuffer.pitch)
    if pixel_format == kms.FORMAT_RG24:
        rows[:, :width * 3] = container.reshape(height, width * 3)
    elif pixel_format == kms.FORMAT_XR24:
        pixels = rows[:, :width * 4].reshape(height, width, 4)
        pixels[:, :, :3] = container
        pixels[:, :, 3] = 0
    else:
        raise ValueError(f"unhandled pixel format {pixel_format:#x}")


def patch_frame_seq(container: np.ndarray, frame_seq: int) -> None:
    """Re-stamp the header's frame counter (and CRC) in place.

    A static payload -- a test pattern, or a repeated camera frame -- still
    advances ``frame_seq`` per NEW frame, because receivers deduplicate by
    the counter and never by timing. Only the 32 header bytes change, so a
    pattern source re-encodes nothing per frame.
    """
    line0 = container.reshape(container.shape[0], -1)[0]
    header = Header.unpack(line0[:HEADER_BYTES].tobytes())
    stamped = dataclasses.replace(header, frame_seq=frame_seq)
    line0[:HEADER_BYTES] = np.frombuffer(stamped.pack(), np.uint8)


def stream(frames, mode: tuple[int, int, int] | None = None,
           connector: int | None = None, card_path: str | None = None,
           on_frame=None) -> None:
    """Scan an iterable of encoded containers out over HDMI. Runs on the Pi.

    Args:
        frames: yields (display_height, display_width, 3) uint8 containers,
            already encoded (the geometry must match the chosen mode).
        mode: (width, height, hz) to REQUIRE, or None for the display's
            preferred mode. Requiring is the right default posture for a
            bring-up: every budget on the receiving side is per-mode.
        on_frame: optional callback(frame_index) after each flip completes;
            the daemon uses it for liveness logging.
    """
    card = kms.Card.open(card_path)
    connector_id, crtc_id, modes = card.connector(connector)
    chosen = kms.pick_mode(modes, mode)
    forced = card.force_full_range(connector_id)
    pixel_format, buffers = card.framebuffers(chosen, count=2)

    name = chosen.name.decode(errors="replace")
    print(f"{card.path}: connector {connector_id}, {name} "
          f"@{chosen.vrefresh}Hz, format "
          f"{'RG24 (memcpy)' if pixel_format == kms.FORMAT_RG24 else 'XR24'}, "
          f"Broadcast RGB {'forced Full' if forced else 'not present'}")

    iterator = iter(frames)
    first = next(iterator)
    write_container(buffers[0], first, pixel_format)
    card.set_crtc(crtc_id, connector_id, chosen, buffers[0])

    back = 1
    for index, container in enumerate(iterator, start=1):
        write_container(buffers[back], container, pixel_format)
        card.flip(crtc_id, buffers[back])
        card.wait_flip()
        back ^= 1
        if on_frame is not None:
            on_frame(index)


def pattern_frames(mode_name: str, cam_width: int, cam_height: int,
                   bayer: str, display: tuple[int, int]):
    """An endless container source from one test pattern.

    The payload is encoded ONCE; each frame only re-stamps the header's
    frame counter. This is the phase-0 source: deterministic, regenerable on
    the receiving side by the same published package, and cheap enough that
    the loop is all flips.
    """
    from bayerlink import encode_frame, pattern
    from bayerlink.protocol import FLAG_TEST_PATTERN

    raw = pattern.generate(mode_name, cam_width, cam_height)
    container = encode_frame(raw, bayer, frame_seq=0, display=display,
                             flags=FLAG_TEST_PATTERN)
    sequence = 0
    while True:
        patch_frame_seq(container, sequence)
        yield container
        sequence += 1
