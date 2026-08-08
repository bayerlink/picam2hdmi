"""Test patterns: the frames used to prove a link before any camera exists.

Phase 0 of any receiver bring-up is a pattern loopback -- encode a known
frame, scan it out, capture it on the far side, and compare every sample.
That test finds range clamps, YCbCr conversion, byte-lane permutation and
scaling BEFORE a camera adds its own variables, which is why these are in the
package rather than in somebody's notebook.

Every pattern is a pure function of its arguments, so both ends regenerate it
independently and compare -- nothing is transmitted out of band.
"""
from __future__ import annotations

import numpy as np

FULL_SCALE = 0xFFF                     # 12-bit


def counting(width: int, height: int) -> np.ndarray:
    """Every sample = its raster index mod 4096.

    The strictest ordering check: a single swapped, dropped or duplicated
    byte anywhere desynchronises the remainder of the frame, so the failure
    is unmissable and its position says where the link broke.
    """
    index = np.arange(width * height, dtype=np.uint32)
    return (index % (FULL_SCALE + 1)).astype(np.uint16).reshape(height, width)


def gradient(width: int, height: int) -> np.ndarray:
    """A horizontal ramp, offset per row.

    Survives partial capture and is readable by eye in a plot; catches
    truncated lines and vertical shifts that counting() reports less
    legibly.
    """
    ramp = np.linspace(0, FULL_SCALE, width, dtype=np.uint16)
    rows = (np.arange(height, dtype=np.uint16)[:, None] * 7) & FULL_SCALE
    return ((ramp[None, :] + rows) & FULL_SCALE).astype(np.uint16)


def checker(width: int, height: int) -> np.ndarray:
    """CFA-position checker: 0 or full scale by (row+col) parity.

    Exercises both extremes of the range on every line -- the values a
    limited-range link clamps first -- and makes a Bayer-phase confusion in
    the receiver visible as an inverted pattern.
    """
    row = np.arange(height)[:, None]
    col = np.arange(width)[None, :]
    return np.where((row + col) & 1, FULL_SCALE, 0).astype(np.uint16)


def corners(width: int, height: int) -> np.ndarray:
    """gradient() plus the awkward values pinned at known positions.

    Zero, full scale, and the values either side of half scale -- where a
    sign confusion or an off-by-one in a receiver's unpacker shows first.
    """
    frame = gradient(width, height)
    pinned = [0, FULL_SCALE, (FULL_SCALE + 1) // 2 - 1, (FULL_SCALE + 1) // 2,
              1, FULL_SCALE - 1]
    for i, value in enumerate(pinned):
        frame[(i * 2) % height, (i * 3) % width] = value
    return frame


PATTERNS = {fn.__name__: fn for fn in (counting, gradient, checker, corners)}


def generate(name: str, width: int, height: int) -> np.ndarray:
    try:
        return PATTERNS[name](width, height)
    except KeyError:
        raise ValueError(
            f"no pattern {name!r}; available: {sorted(PATTERNS)}") from None
