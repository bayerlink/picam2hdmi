# Copyright 2026 Serge Rabyking
# SPDX-License-Identifier: MIT
"""The Pi as a thin bridge: sensor lines land on the wire in LINES.

The serve path is a frame-store bridge: libcamera completes a request
only when the sensor's last row has landed, Python packs the frame,
KMS re-serialises it over a whole scanout -- 1.5 to 2.5 frames of
latency, structural, not tunable. This path removes the structure:

    sensor --CSI-2--> Unicam DMA, stride = DISPLAY PITCH,
                      straight into the scanout payload buffer
    CPU    --------> ONE 48-byte header per frame, on its own plane
    HVS    --------> scans the same buffer out over HDMI, racing the
                      DMA write beam by a phase margin of LINES

Three facts make it work, each measured on this Pi before a line of
this file existed (2026-08-18):
  - Unicam accepts a DMABUF import (REQBUFS capabilities 0x15), so the
    KMS dumb buffer IS the capture buffer -- nothing per-pixel ever
    crosses the CPU;
  - Unicam honours an oversized bytesperline, so a 2400-byte packed
    sensor line lands at the head of each 5760-byte display row, and
    bayerlink's own definition -- "CSI-2 packed bytes, zero-padded to
    display width" -- is satisfied by DMA stride alone;
  - ov5647 exposes vertical_blanking with STEP=1, the one-line trim a
    software genlock needs.

The header rides its OWN one-row plane at the top of the mode, because
a V4L2 DMABUF import always starts at byte zero of the buffer: there is
no way to ask the DMA to leave row 0 alone, so row 0 is simply not the
DMA's buffer. The HVS composites the two planes for free.

No libcamera anywhere: the kernel's own sensor driver carries the init,
media-ctl plumbs the graph, and the FPGA on the far end is the ISP --
which is the whole premise of the system this feeds.

Rates: the sensor free-runs near the mode's rate and the display near
30Hz, on unrelated crystals. Unsteered, the write beam laps the read
beam slowly and a tear seam drifts through the picture -- visible,
honest, and the measurement the genlock loop will close on. This first
version STATES the drift rather than hiding it; `--vblank` is the
manual trim, and the closed loop is the next commit's job.

    python3 -m picam2hdmi.bridge --seconds 30
"""
from __future__ import annotations

import argparse
import ctypes
import fcntl
import os
import select
import struct
import subprocess
import sys
import time

from . import kms

u8, u32, u64 = ctypes.c_uint8, ctypes.c_uint32, ctypes.c_uint64

WIDTH, HEIGHT = 1920, 1080          # the ov5647's native 1080p mode
CONTAINER_HZ = 30
BITS = 10
# The header's order is OUR claim, and for ov5647 1080p the driver's
# GBRG is CORRECT -- confirmed 2026-08-19 by walking all four orders
# against REAL LIGHT (BGGR mixed, GRBG swapped R/B, GBRG true). The
# sensor's colour-bar pattern briefly "proved" otherwise and that was
# the lesson: the test pattern carries the GENERATOR's parity, one
# column off the array's crop parity -- bars validate transport and
# demosaic, never the mosaic phase of real light.
BAYER_ORDER = "GBRG"                # pGAA: 10-bit GBGB/RGRG packed
LINE_BYTES = WIDTH * BITS // 8      # 2400 packed bytes per sensor line


# --------------------------------------------------------------------------- #
# V4L2, the eight ioctls this file actually uses (the kms.py principle)
# --------------------------------------------------------------------------- #

class _Timeval(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_usec", ctypes.c_long)]


class _Timecode(ctypes.Structure):
    _fields_ = [("type", u32), ("flags", u32), ("frames", u8),
                ("seconds", u8), ("minutes", u8), ("hours", u8),
                ("userbits", u8 * 4)]


class _BufferM(ctypes.Union):
    _fields_ = [("offset", u32), ("userptr", ctypes.c_ulong),
                ("planes", u64), ("fd", ctypes.c_int32)]


class V4l2Buffer(ctypes.Structure):
    _fields_ = [("index", u32), ("type", u32), ("bytesused", u32),
                ("flags", u32), ("field", u32),
                ("timestamp", _Timeval), ("timecode", _Timecode),
                ("sequence", u32), ("memory", u32),
                ("m", _BufferM), ("length", u32), ("reserved2", u32),
                ("request_fd", u32)]


class V4l2RequestBuffers(ctypes.Structure):
    _fields_ = [("count", u32), ("type", u32), ("memory", u32),
                ("capabilities", u32), ("flags", u8),
                ("reserved", u8 * 3)]


# The kernel dispatches on the exact encoded size, so the struct sizes
# are asserted rather than hoped: a mislaid pad byte here is ENOTTY.
assert ctypes.sizeof(V4l2Buffer) == 88, ctypes.sizeof(V4l2Buffer)
assert ctypes.sizeof(V4l2RequestBuffers) == 20


def _v4l2_iowr(nr: int, structure) -> int:
    return (3 << 30) | (ctypes.sizeof(structure) << 16) | (0x56 << 8) | nr


VIDIOC_REQBUFS = _v4l2_iowr(8, V4l2RequestBuffers)
VIDIOC_QBUF = _v4l2_iowr(15, V4l2Buffer)
VIDIOC_DQBUF = _v4l2_iowr(17, V4l2Buffer)
VIDIOC_STREAMON = (1 << 30) | (4 << 16) | (0x56 << 8) | 18
VIDIOC_STREAMOFF = (1 << 30) | (4 << 16) | (0x56 << 8) | 19

BUF_TYPE_CAPTURE = 1
MEMORY_DMABUF = 4


def _sh(*argv: str) -> str:
    """Run one plumbing command; its stderr is the diagnosis on failure."""
    done = subprocess.run(argv, capture_output=True, text=True)
    if done.returncode:
        raise RuntimeError(f"{' '.join(argv)}: {done.stderr.strip()}")
    return done.stdout


# --------------------------------------------------------------------------- #
# Camera plumbing: the kernel driver owns the init, media-ctl the graph
# --------------------------------------------------------------------------- #

def find_sensor(media: str) -> str:
    """The sensor entity's name, discovered rather than assumed."""
    for line in _sh("media-ctl", "-p", "-d", media).splitlines():
        if "entity" in line and "ov5647" in line:
            return line.split(":", 1)[1].split("(")[0].strip()
    raise RuntimeError(f"no ov5647 entity on {media} -- is the overlay loaded?")


def setup_camera(media: str, video: str, subdev: str, pitch: int,
                 exposure: int | None, gain: int | None,
                 vblank: int | None) -> None:
    entity = find_sensor(media)
    _sh("media-ctl", "-d", media, "-V",
        f'"{entity}":0 [fmt:SGBRG10_1X10/{WIDTH}x{HEIGHT}]')
    _sh("v4l2-ctl", "-d", video, "--set-fmt-video",
        f"width={WIDTH},height={HEIGHT},pixelformat=pGAA,"
        f"bytesperline={pitch}")
    got = _sh("v4l2-ctl", "-d", video, "--get-fmt-video")
    if f"Bytes per Line    : {pitch}" not in got:
        raise RuntimeError(
            f"unicam did not keep bytesperline={pitch}; the stride trick "
            f"is the whole bridge, so this refuses rather than tearing:\n{got}")
    # AE and AWB stay off for the same reason libcamera stays out: the
    # FPGA is the ISP. What is set here is what capture.py sets.
    ctrls = ["auto_exposure=1", "gain_automatic=0",
             "white_balance_automatic=0"]
    if exposure is not None:
        ctrls.append(f"exposure={exposure}")
    if gain is not None:
        ctrls.append(f"analogue_gain={gain}")
    if vblank is not None:
        ctrls.append(f"vertical_blanking={vblank}")
    _sh("v4l2-ctl", "-d", subdev, "--set-ctrl", ",".join(ctrls))


# --------------------------------------------------------------------------- #
# The bridge
# --------------------------------------------------------------------------- #

def overlay_plane_for(card: kms.Card, crtc_id: int) -> int:
    """A free plane that can drive this CRTC, for the payload."""
    res = kms.CardRes()
    card._ioctl(kms.IOCTL_GETRESOURCES, res)
    crtc_ids = (u32 * max(1, res.count_crtcs))()
    res.crtc_id_ptr = ctypes.addressof(crtc_ids)
    res.count_connectors = res.count_encoders = res.count_fbs = 0
    card._ioctl(kms.IOCTL_GETRESOURCES, res)
    crtc_index = list(crtc_ids[:res.count_crtcs]).index(crtc_id)
    for plane_id, possible, fb_id in card.planes():
        if possible & (1 << crtc_index) and fb_id == 0:
            return plane_id
    raise RuntimeError("no free plane for the payload; the HVS should "
                       "have several -- is something else compositing?")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--media", default="/dev/media0")
    parser.add_argument("--video", default="/dev/video2")
    parser.add_argument("--subdev", default="/dev/v4l-subdev0")
    parser.add_argument("--exposure", type=int, default=None)
    parser.add_argument("--gain", type=int, default=None)
    parser.add_argument("--vblank", type=int, default=None,
                        help="vertical_blanking in sensor lines, step 1: "
                             "the genlock trim, manual for now")
    parser.add_argument("--seconds", type=float, default=0.0)
    args = parser.parse_args()

    from bayerlink.protocol import Header, fourcc_for

    # --- display: one mode, two planes ---------------------------------
    card = kms.Card.open()
    try:
        card.enable_universal_planes()
        connector_id, crtc_id, _modes = card.connector()
        mode = kms.container_mode(WIDTH, HEIGHT + 1, CONTAINER_HZ)
        card.force_full_range(connector_id)

        # The PRIMARY covers the whole mode -- DRM refuses a primary
        # smaller than the raster (vc4 says ENOSPC, which is its word
        # for "does not fit", measured the hard way). Row 0 is the
        # header's home; every other row hides behind the overlay.
        size = lambda w, h: type(mode)(hdisplay=w, vdisplay=h)  # noqa: E731
        base_fb = card.framebuffer(mode, kms.FORMAT_RG24)
        pay_fb = card.framebuffer(size(WIDTH, HEIGHT), kms.FORMAT_RG24)
        # Zero once: the pad bytes past each 2400-byte line are the
        # protocol's zero padding, and the DMA never touches them.
        base_fb.view[:] = b"\x00" * len(base_fb.view)
        pay_fb.view[:] = b"\x00" * len(pay_fb.view)
        head_row = base_fb.view

        # The payload rides an overlay at y=1: a DMABUF import starts
        # at byte zero, so row zero of the mode is simply not the DMA's
        # buffer -- it is the primary's, where the CPU writes 48 bytes.
        card.set_crtc(crtc_id, connector_id, mode, base_fb)
        plane_id = overlay_plane_for(card, crtc_id)
        card.set_plane(plane_id, crtc_id, pay_fb.fb_id,
                       0, 1, WIDTH, HEIGHT)

        # --- camera, writing into the payload plane's own bytes --------
        setup_camera(args.media, args.video, args.subdev, pay_fb.pitch,
                     args.exposure, args.gain, args.vblank)
        dmabuf = card.prime_fd(pay_fb.handle)
        vfd = os.open(args.video, os.O_RDWR | os.O_NONBLOCK)
        try:
            req = V4l2RequestBuffers(count=1, type=BUF_TYPE_CAPTURE,
                                     memory=MEMORY_DMABUF)
            fcntl.ioctl(vfd, VIDIOC_REQBUFS, req)

            def queue():
                buf = V4l2Buffer(index=0, type=BUF_TYPE_CAPTURE,
                                 memory=MEMORY_DMABUF,
                                 length=len(pay_fb.view))
                buf.m.fd = dmabuf
                fcntl.ioctl(vfd, VIDIOC_QBUF, buf)

            queue()
            fcntl.ioctl(vfd, VIDIOC_STREAMON,
                        struct.pack("i", BUF_TYPE_CAPTURE))

            sid = 0          # one sensor on this device
            fourcc = fourcc_for(BAYER_ORDER, BITS)
            sequence = 0
            t0 = time.monotonic()
            last_ts, last_report = None, t0
            print(f"bridge: {WIDTH}x{HEIGHT} pGAA -> "
                  f"{WIDTH}x{HEIGHT + 1}@{CONTAINER_HZ} container, "
                  f"payload pitch {pay_fb.pitch}, source_id {sid}")
            while args.seconds <= 0 or time.monotonic() - t0 < args.seconds:
                ready, _w, _x = select.select([vfd], [], [], 2.0)
                if not ready:
                    print("bridge: no frame for 2s -- sensor stalled?")
                    continue
                buf = V4l2Buffer(index=0, type=BUF_TYPE_CAPTURE,
                                 memory=MEMORY_DMABUF)
                buf.m.fd = dmabuf
                fcntl.ioctl(vfd, VIDIOC_DQBUF, buf)
                # The frame that just COMPLETED wore this header while
                # it was being scanned; stamp the next one and requeue.
                # A reader that catches the 48 bytes mid-rewrite gets a
                # CRC miss and refuses ONE frame -- the receiver's
                # refused counter is the honest tally of that race
                # until the genlock loop pins the phase.
                sequence += 1
                head_row[:48] = Header(
                    fourcc=fourcc, width=WIDTH, height=HEIGHT,
                    frame_seq=sequence, source_id=sid).pack()
                queue()
                ts = buf.timestamp.tv_sec + buf.timestamp.tv_usec / 1e6
                if last_ts is not None and time.monotonic() - last_report > 5:
                    last_report = time.monotonic()
                    period = (ts - last_ts)
                    print(f"bridge: seq {sequence}, sensor frame period "
                          f"{period * 1000:.3f} ms (display wants "
                          f"{1000 / CONTAINER_HZ:.3f})")
                last_ts = ts
            fcntl.ioctl(vfd, VIDIOC_STREAMOFF,
                        struct.pack("i", BUF_TYPE_CAPTURE))
        finally:
            os.close(vfd)
            os.close(dmabuf)
    finally:
        card.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
