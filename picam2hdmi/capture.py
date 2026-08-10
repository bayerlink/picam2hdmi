"""Capture: raw Bayer frames from the camera, via the Linux camera stack.

Picamera2 (libcamera), raw stream only. The ISP is NOT in the loop: the
raw stream is the sensor's own packed output, DMA-written by the CSI
receiver -- and CSI-2 packing IS bayerlink's payload layout, so each
captured line goes onto the wire verbatim through ``encode_packed``,
with no unpack/repack anywhere. The Bayer order and bit depth come from
libcamera's reported format and go straight into the fourcc: this tool
never holds a sensor register table, which is the point -- every camera
libcamera supports works here without this tool learning anything about
it.

Auto-exposure and auto-white-balance are DISABLED by default: a raw
source should be deterministic unless the operator asks otherwise.

Cropping is done in the PACKED domain -- a horizontal crop is a byte
slice when it lands on packing-group boundaries, so the constraint is
honest: x and width must be multiples of the group (4 samples at 10
bits), y and height even so the CFA phase survives. The crop is how a
big sensor meets a link budget: a 2592-wide mode does not obey 1080p's
2200-slot rate rule, a cropped window does -- and a *small* crop fits
the luma tunnel, which is how real photons get byte-verified through a
$10 capture stick.
"""
from __future__ import annotations

import numpy as np

# Total line slots (active + blanking) of the display modes the bench
# uses; the receiver rate rule is samples <= slots (PROTOCOL.md).
LINE_SLOTS = {(1920, 1080): 2200, (1280, 720): 1650, (640, 480): 800}


def order_from_format(fmt: str) -> tuple[str | None, int]:
    """libcamera raw-format name -> (bayer order or None, bits).

    'SBGGR10_CSI2P' -> ('BGGR', 10); 'R8' style monochrome maps to
    (None, bits). Refuses unpacked formats: the whole design rides on
    the DMA'd bytes already being the wire payload.
    """
    name = fmt.upper()
    if name.startswith("S") and "_CSI2P" in name:
        order = name[1:5]
        digits = "".join(ch for ch in name[5:name.index("_")] if ch.isdigit())
        if order in ("RGGB", "GRBG", "GBRG", "BGGR") and digits:
            return order, int(digits)
    if name.startswith("R") and name[1:].isdigit():
        return None, int(name[1:])
    raise ValueError(
        f"raw format {fmt!r} is not a packed CSI-2 Bayer or mono format; "
        "this source carries the sensor's packed bytes verbatim and does "
        "not repack unpacked streams")


def validate_crop(crop, sensor: tuple[int, int],
                  bits: int) -> tuple[int, int, int, int]:
    """(x, y, w, h) checked against the sensor window and the packing.

    Group-aligned x/w make the crop a byte slice; even y/h keep the CFA
    phase; and a bad crop is refused, not clamped -- a silently moved
    window is a calibration surprise three measurements later.
    """
    from bayerlink.protocol import _GROUP

    group = _GROUP[bits][0]
    x, y, w, h = (int(v) for v in crop)
    sensor_w, sensor_h = sensor
    if x % group or w % group:
        raise ValueError(
            f"crop x={x}, w={w} must be multiples of {group} samples: at "
            f"{bits} bits a group is one indivisible byte cluster")
    if y % 2 or h % 2:
        raise ValueError(
            f"crop y={y}, h={h} must be even, or the window's CFA phase "
            "flips relative to its header")
    if w <= 0 or h <= 0 or x < 0 or y < 0 or x + w > sensor_w \
            or y + h > sensor_h:
        raise ValueError(
            f"crop {x},{y},{w}x{h} does not fit the {sensor_w}x{sensor_h} "
            "sensor window")
    return x, y, w, h


def crop_packed(rows: np.ndarray, x: int, w: int, bits: int) -> np.ndarray:
    """Horizontal crop in the packed domain: a pure byte slice."""
    from bayerlink.protocol import _GROUP

    group_samples, group_bytes = _GROUP[bits]
    start = x // group_samples * group_bytes
    stop = (x + w) // group_samples * group_bytes
    return rows[:, start:stop]


def check_rate(width: int, display: tuple[int, int]) -> None:
    """The receiver rate rule, refused at the source where it is fixable."""
    slots = LINE_SLOTS.get(tuple(display))
    if slots is not None and width > slots:
        raise ValueError(
            f"{width} samples/line exceeds the {slots} line slots of "
            f"{display[0]}x{display[1]} (the one-sample-per-clock receiver "
            "budget); crop the sensor window, e.g. --crop with width "
            f"<= {slots}")


def frames(display: tuple[int, int], mode: tuple[int, int] | None = None,
           crop: tuple[int, int, int, int] | None = None,
           exposure_us: int | None = None,
           analogue_gain: float | None = None,
           luma_tunnel: bool = False, flags: int = 0):
    """An endless container source from the camera. Runs on the Pi.

    Yields scanout-shaped frames exactly like the pattern and file
    sources. ``mode`` picks the sensor's raw mode (default 1296x972, the
    OV5647's 2x2-binned mode and the largest of its modes that meets
    1080p's rate rule); ``crop`` selects a window of it.
    """
    try:
        from picamera2 import Picamera2
    except ImportError as error:
        raise ValueError(
            "camera capture needs Picamera2 (sudo apt install "
            f"python3-picamera2), which is Pi-only: {error}") from None

    from bayerlink import encode_packed
    from bayerlink import tunnel as _tunnel
    from bayerlink.protocol import _GROUP

    picam2 = Picamera2()
    size = tuple(mode) if mode else (1296, 972)
    controls = {"AeEnable": False, "AwbEnable": False}
    if exposure_us is not None:
        controls["ExposureTime"] = int(exposure_us)
    if analogue_gain is not None:
        controls["AnalogueGain"] = float(analogue_gain)
    # Controls ride in the CONFIGURATION, not set after start: the very
    # first frame obeys them, which is what deterministic means.
    config = picam2.create_video_configuration(raw={"size": size},
                                               buffer_count=4,
                                               controls=controls)
    picam2.configure(config)
    raw_config = picam2.camera_configuration()["raw"]
    fmt = raw_config["format"]
    order, bits = order_from_format(fmt)
    sensor_w, sensor_h = raw_config["size"]

    if crop is not None:
        x, y, w, h = validate_crop(crop, (sensor_w, sensor_h), bits)
    else:
        x, y, w, h = 0, 0, sensor_w, sensor_h

    target = _tunnel.inner_display(*display) if luma_tunnel else display
    check_rate(w, display)

    picam2.start()
    print(f"camera: {fmt} {sensor_w}x{sensor_h}, window {w}x{h}+{x}+{y}, "
          f"AE/AWB off"
          + (f", exposure {exposure_us}us" if exposure_us else "")
          + (f", gain {analogue_gain}" if analogue_gain else ""))

    group_samples, group_bytes = _GROUP[bits]
    line_bytes = sensor_w // group_samples * group_bytes

    sequence = 0
    try:
        while True:
            buffer = picam2.capture_array("raw")
            # The buffer is (rows, stride) uint8; the stride carries
            # padding beyond the packed line; vertical crop is row slicing.
            rows = buffer[y:y + h, :line_bytes]
            window = crop_packed(rows, x, w, bits) if (x or w != sensor_w) \
                else rows
            container = encode_packed(np.ascontiguousarray(window), order,
                                      frame_seq=sequence, bits=bits,
                                      display=target, flags=flags)
            yield _tunnel.encode(container, display) if luma_tunnel \
                else container
            sequence += 1
    finally:
        # The camera is exclusive hardware; release it the moment this
        # generator is closed, not whenever a collector gets around to it.
        picam2.stop()
        picam2.close()
