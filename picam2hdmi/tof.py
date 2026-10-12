# Copyright 2026 Serge Rabyking
# SPDX-License-Identifier: MIT
"""Arducam ToF depth camera, raw phase frames only. Runs on the Pi.

The ISP is NOT in the loop, for the same reason it is not in `capture`:
what belongs on the wire is what the sensor measured, and everything that
interprets it belongs downstream where it can be changed without a
recapture.

A ToF sensor does not measure distance. It measures the correlation
between the returning light and four reference phases, and distance is an
ARCTANGENT of two differences of those four. Arducam's SDK will do that
conversion and hand over millimetres; this module deliberately asks for
``FrameType.RAW`` instead, so the four phase planes travel intact and the
conversion happens where the rest of the pipeline lives.

The SDK is still what opens the camera, because it also owns the sensor's
configuration -- modulation frequency, exposure, work mode. Reading the
V4L2 node directly would mean restating that register set here, which is
the second copy of a fact this project exists to avoid.

**Not a libcamera camera.** `rpicam-hello` reports "No cameras available"
with this sensor attached and the driver correctly bound; Picamera2 cannot
open it either. That is the design, not a fault: the kernel driver
(`arducam-pivariety`, in-tree) delivers frames, and the SDK owns them.
Which is why this is a separate source and not a mode of `capture`.

One plane per frame, because that is what the sensor does
---------------------------------------------------------

The SDK hands over one array of ``PHASES * height`` lines with the phases
INTERLEAVED BY ROW: line ``r`` is phase ``r % 4`` of sensor line ``r // 4``.
That is a convenience of the SDK's buffer, not of the link. Each phase is
its own sensor readout -- its own exposure, its own frame -- and a CSI-2
receiver wired to this sensor directly would see four ordinary frames go
past, not one tall one.

So this module DE-INTERLEAVES and emits one bayerlink frame per phase
plane, at the sensor's real geometry. The wire then carries what a MIPI
receiver would deliver, which is the whole point: a pipeline developed
against this source takes the sensor directly, later, unchanged.

Which phase is which travels in ``frame_seq``, which the protocol already
defines as incrementing once per new camera frame -- and each phase
readout is one. ``frame_seq % PHASES`` is the phase index and
``frame_seq // PHASES`` is the measurement the four belong to. Nothing is
added to the header for this. In particular:

- NOT ``source_id``: bayerlink defines it as which CAMERA, and a receiver
  pairs stripes by ``(source_id, frame_seq)``. One sensor announcing four
  identities would break that, and would give one module four entries in
  any table keyed by source.
- NOT a spare ``flags`` bit: v2 requires a receiver to refuse unknown flag
  bits, so it would be a protocol version bump for a convenience.

A receiver that joins mid-stream needs no negotiation: ``frame_seq`` is
absolute and this module anchors it, so ``% PHASES`` is the true index.
Reading the sensor over CSI-2 instead, the fabric counts frame starts and
its alignment is arbitrary -- but a rotation of the phase labelling turns
the quadrature vector by exactly a quarter turn, which is a CONSTANT
distance, and a constant distance is already a calibrated register
downstream. Either way the arithmetic is the same; only the offset differs.

The cost is that a measurement whose four frames are not consecutive in
``frame_seq`` must be discarded, exactly as the protocol already requires
of an incomplete stripe set, and for the same reason.

Requires, on the Pi::

    dtoverlay=arducam-pivariety      # in /boot/firmware/config.txt
    camera_auto_detect=0             # the two conflict
    sudo pip3 install --break-system-packages --no-deps ArducamDepthCamera

``--no-deps`` is not optional. The SDK pins numpy 1.26, which installed
into the system Python would shadow the apt numpy that picamera2 is built
against -- and it runs against that numpy perfectly well.
"""

import numpy as np

# Raw phase samples are SIGNED: a correlation against a reference can fall
# either side of zero, and the sensor reports it that way. Every mono
# fourcc bayerlink assigns is unsigned, so the samples ride as unsigned
# with this offset removed at the far end. It is lossless -- int16 plus
# 2**15 is exactly the uint16 range -- and it is a fact the consumer needs,
# so it is named here rather than left as a convention.
PHASE_OFFSET = 1 << 15

# Reference phases per measurement. The SDK's raw buffer interleaves them
# by ROW -- line r is phase r % PHASES of sensor line r // PHASES -- and
# this module sends them as PHASES separate frames.
PHASES = 4


def frames(display: tuple[int, int], flags: int = 0, peek=None,
           source_id: int | None = None, timeout_ms: int = 3000):
    """An endless container source from the ToF sensor's raw phases.

    Yields scanout-shaped frames exactly like the pattern, camera and file
    sources, so scanout takes them without knowing what they carry.

    One frame per PHASE PLANE, at the sensor's own geometry, 16-bit mono,
    so the header says ``Y16``. Four consecutive ``frame_seq`` values are
    one measurement; see the module docstring for why the phase index
    rides there and not in a header field of its own.

    ``peek``, if given, receives every emitted container BEFORE the
    content, exactly as in `capture.frames`. It never touches the wire.
    """
    try:
        import ArducamDepthCamera as ac
    except ImportError as error:
        raise ValueError(
            "ToF capture needs the Arducam SDK (sudo pip3 install "
            "--break-system-packages --no-deps ArducamDepthCamera), which "
            f"is Pi-only: {error}") from None

    from bayerlink import encode_packed

    cam = ac.ArducamCamera()
    status = cam.open(ac.Connection.CSI, 0)
    if status != ac.TofErrorCode.ArducamSuccess:
        raise RuntimeError(
            f"cannot open the ToF camera: {status}. The camera is exclusive "
            "hardware -- if another process holds it, this fails here. Check "
            "also that dtoverlay=arducam-pivariety is set and "
            "camera_auto_detect=0, because the two conflict and auto-detect "
            "wins.")
    status = cam.start(ac.FrameType.RAW)
    if status != ac.TofErrorCode.ArducamSuccess:
        cam.close()
        raise RuntimeError(f"cannot start the ToF camera in RAW mode: {status}")

    info = cam.getCameraInfo()
    sid = 0 if source_id is None else int(source_id)
    print(f"tof: {info.device_type} (source_id {sid}"
          + ("" if sid else ", UNKNOWN to bayerlink's table") + "), "
          f"{info.width}x{info.height}, {PHASES} phase frames per "
          f"measurement, 16-bit mono; phase = frame_seq % {PHASES}")

    sequence = 0
    try:
        while True:
            frame = cam.requestFrame(timeout_ms)
            if frame is None:
                # A dropped measurement is not an error: the sensor
                # free-runs and a timeout means this interval produced
                # nothing. Skipping the WHOLE measurement is what keeps
                # `frame_seq % PHASES` meaning the phase index -- the
                # sequence only ever advances four at a time.
                continue
            raw = np.asarray(frame.raw_data)
            cam.releaseFrame(frame)

            if raw.dtype != np.int16:
                raise ValueError(
                    f"expected signed 16-bit raw phases, got {raw.dtype}; "
                    "the SDK's raw layout has changed and PHASE_OFFSET no "
                    "longer describes it")
            if raw.shape[0] % PHASES:
                raise ValueError(
                    f"{raw.shape[0]} raw lines is not {PHASES} whole phase "
                    "planes; the SDK's raw layout has changed and the "
                    "row interleave no longer describes it")
            unsigned = (raw.astype(np.int32) + PHASE_OFFSET).astype(np.uint16)

            for phase in range(PHASES):
                # Stride-PHASES slice IS one phase plane: the sensor
                # interleaves them by row, so this undoes the readout
                # order rather than cutting the frame up.
                plane = np.ascontiguousarray(unsigned[phase::PHASES])
                lines, width = plane.shape
                payload = plane.view(np.uint8).reshape(lines, width * 2)

                container = encode_packed(payload, None, frame_seq=sequence,
                                          bits=16, display=display,
                                          flags=flags, source_id=sid)
                if peek is not None:
                    peek(container)
                yield container
                sequence += 1
    finally:
        # Exclusive hardware: release it when the generator closes, not
        # when a collector gets to it.
        cam.stop()
        cam.close()
