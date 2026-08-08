"""rawlink v1: the reference implementation of PROTOCOL.md.

This module IS the executable form of the specification. It has no camera and
no display in it -- encoding a frame produces a byte container any scanout can
carry, and decoding one accepts a byte container any capture produced. The
same module therefore runs on BOTH ends of the link: the Pi encodes with it,
and a receiver's host software decodes captured frames with it, so there is
exactly one implementation to disagree with the spec, which is the smallest
number attainable while having one at all.
"""
from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass

import numpy as np

MAGIC = 0x4B4C5752            # the bytes "RWLK", little-endian
VERSION = 1
HEADER_BYTES = 32
_HEADER_FMT = "<IHHIIIII"     # magic, version, header_bytes, fourcc,
                              # width, height, frame_seq, flags

FLAG_TEST_PATTERN = 1 << 0

# V4L2 raw fourccs carried by version 1: the 12-bit packed Bayer family.
# The fourcc encodes format, bit depth AND Bayer order in one field that
# already has an external authority defining it.
_FOURCC_12P = {
    "pRCC": "RGGB",
    "pgCC": "GRBG",
    "pGCC": "GBRG",
    "pBCC": "BGGR",
}
_ORDER_TO_FOURCC = {order: code for code, order in _FOURCC_12P.items()}

# Bayer order -> the two phase bits revela-style pipelines consume:
# bit 1 = row parity of R, bit 0 = column parity of R.
BAYER_PHASE = {"RGGB": 0b00, "GRBG": 0b01, "GBRG": 0b10, "BGGR": 0b11}


def fourcc_code(text: str) -> int:
    """The u32 a four-character code occupies, little-endian."""
    if len(text) != 4:
        raise ValueError(f"a fourcc is four characters, got {text!r}")
    return int.from_bytes(text.encode("ascii"), "little")


def fourcc_text(code: int) -> str:
    return int(code).to_bytes(4, "little").decode("ascii", errors="replace")


def fourcc_for(order: str, bits: int = 12) -> str:
    """The fourcc for a Bayer order at a bit depth version 1 carries."""
    if bits != 12:
        raise ValueError(
            f"rawlink v1 carries the 12-bit packed family; {bits}-bit payloads "
            "are a future fourcc, not a variation of this one")
    try:
        return _ORDER_TO_FOURCC[order.upper()]
    except KeyError:
        raise ValueError(
            f"unknown Bayer order {order!r}; expected one of "
            f"{sorted(_ORDER_TO_FOURCC)}") from None


# --------------------------------------------------------------------------- #
# Header
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Header:
    """The 32 bytes at the start of line 0, parsed."""

    fourcc: str
    width: int
    height: int
    frame_seq: int
    flags: int = 0
    version: int = VERSION

    @property
    def bayer_order(self) -> str:
        return _FOURCC_12P[self.fourcc]

    @property
    def bayer_phase(self) -> int:
        return BAYER_PHASE[self.bayer_order]

    @property
    def bits(self) -> int:
        return 12

    @property
    def line_bytes(self) -> int:
        return self.width * 3 // 2

    @property
    def is_test_pattern(self) -> bool:
        return bool(self.flags & FLAG_TEST_PATTERN)

    def pack(self) -> bytes:
        body = struct.pack(
            _HEADER_FMT, MAGIC, self.version, HEADER_BYTES,
            fourcc_code(self.fourcc), self.width, self.height,
            self.frame_seq, self.flags)
        return body + struct.pack("<I", zlib.crc32(body))

    @classmethod
    def unpack(cls, raw: bytes) -> "Header":
        """Parse and VERIFY a header. Refusal is the API: no guessed decodes."""
        if len(raw) < HEADER_BYTES:
            raise ValueError(f"header needs {HEADER_BYTES} bytes, got {len(raw)}")
        body, crc = raw[:28], struct.unpack("<I", raw[28:32])[0]
        (magic, version, header_bytes, code,
         width, height, frame_seq, flags) = struct.unpack(_HEADER_FMT, body)
        if magic != MAGIC:
            raise ValueError(
                f"not a rawlink stream: magic {magic:#010x}, expected {MAGIC:#010x}. "
                "The usual causes are a limited-range or YCbCr link, or byte-lane "
                "permutation -- see PROTOCOL.md, 'Requirements on the link'.")
        if crc != zlib.crc32(body):
            raise ValueError(
                "header CRC mismatch: the magic survived but the header did not. "
                "Suspect a link that modifies pixel values (range clamp, dithering).")
        if version != VERSION:
            raise ValueError(
                f"rawlink version {version} is not the {VERSION} this "
                "implementation speaks; refusing rather than guessing")
        if header_bytes < HEADER_BYTES:
            raise ValueError(f"header_bytes {header_bytes} is shorter than v1's minimum")
        text = fourcc_text(code)
        if text not in _FOURCC_12P:
            raise ValueError(
                f"payload fourcc {text!r} is not implemented here; v1 requires "
                f"{sorted(_FOURCC_12P)}. Refusing a format beats decoding it wrongly.")
        if width <= 0 or width % 2:
            raise ValueError(f"width {width} must be positive and even for 12P")
        if height <= 0:
            raise ValueError(f"height {height} must be positive")
        return cls(fourcc=text, width=width, height=height,
                   frame_seq=frame_seq, flags=flags, version=version)


# --------------------------------------------------------------------------- #
# 12-bit packed payload (V4L2 *12P)
# --------------------------------------------------------------------------- #

def pack12p(samples: np.ndarray) -> np.ndarray:
    """Samples (…, N even) uint16 -> packed bytes (…, N*3/2) uint8.

    Layout per pair P0, P1:  [ P0[11:4] ][ P1[11:4] ][ P1[3:0]<<4 | P0[3:0] ]
    """
    samples = np.asarray(samples)
    if samples.shape[-1] % 2:
        raise ValueError("12P packs sample PAIRS; the last axis must be even")
    if samples.dtype != np.uint16:
        raise TypeError(f"samples must be uint16, got {samples.dtype}")
    if int(samples.max(initial=0)) > 0xFFF:
        raise ValueError("a 12-bit sample exceeds 4095; refusing to truncate")
    p0 = samples[..., 0::2]
    p1 = samples[..., 1::2]
    out = np.empty(samples.shape[:-1] + (samples.shape[-1] * 3 // 2,), np.uint8)
    out[..., 0::3] = (p0 >> 4).astype(np.uint8)
    out[..., 1::3] = (p1 >> 4).astype(np.uint8)
    out[..., 2::3] = (((p1 & 0xF) << 4) | (p0 & 0xF)).astype(np.uint8)
    return out


def unpack12p(packed: np.ndarray) -> np.ndarray:
    """Inverse of :func:`pack12p`: bytes (…, M multiple of 3) -> uint16 samples."""
    packed = np.asarray(packed, dtype=np.uint8)
    if packed.shape[-1] % 3:
        raise ValueError("12P bytes come in triples; the last axis must divide by 3")
    b0 = packed[..., 0::3].astype(np.uint16)
    b1 = packed[..., 1::3].astype(np.uint16)
    b2 = packed[..., 2::3].astype(np.uint16)
    out = np.empty(packed.shape[:-1] + (packed.shape[-1] * 2 // 3,), np.uint16)
    out[..., 0::2] = (b0 << 4) | (b2 & 0xF)
    out[..., 1::2] = (b1 << 4) | (b2 >> 4)
    return out


# --------------------------------------------------------------------------- #
# Frame container
# --------------------------------------------------------------------------- #

def check_geometry(width: int, height: int, display: tuple[int, int]) -> None:
    """Refuse a camera mode the container cannot carry.

    ``display`` is (active_width, active_height) of the display mode. The line
    budget is bytes; the height budget loses one line to the header.
    """
    display_width, display_height = display
    line_bytes = width * 3 // 2
    if line_bytes > display_width * 3:
        raise ValueError(
            f"{width} samples/line needs {line_bytes} bytes, but a "
            f"{display_width}-pixel display line carries {display_width * 3}")
    if height + 1 > display_height:
        raise ValueError(
            f"{height} camera lines + 1 header line exceed the "
            f"{display_height}-line display frame")


def fits_line_rate(width: int, total_line_slots: int) -> bool:
    """The one-sample-per-clock receiver budget (see PROTOCOL.md).

    True iff a receiver clocked at the display pixel clock keeps up with a
    small line FIFO: the camera line must fit in the WHOLE line time,
    blanking included. 2028 fits 1080p (2200 slots); it does not fit 720p
    (1650), and no FIFO depth fixes a violated average.
    """
    return width <= total_line_slots


def encode_frame(raw: np.ndarray, bayer_order: str, frame_seq: int,
                 display: tuple[int, int] = (1920, 1080),
                 flags: int = 0) -> np.ndarray:
    """One camera frame -> the (height, width, 3) uint8 container to scan out.

    ``raw`` is (lines, samples) uint16, one camera line per row. The result is
    the display frame's memory image: header in line 0, one packed camera line
    per display line, zeros elsewhere.
    """
    raw = np.asarray(raw)
    if raw.ndim != 2:
        raise ValueError(f"raw frame must be (lines, samples), got {raw.shape}")
    height, width = raw.shape
    check_geometry(width, height, display)
    display_width, display_height = display

    header = Header(fourcc=fourcc_for(bayer_order), width=width, height=height,
                    frame_seq=frame_seq, flags=flags)
    frame = np.zeros((display_height, display_width * 3), np.uint8)
    frame[0, :HEADER_BYTES] = np.frombuffer(header.pack(), np.uint8)
    frame[1:1 + height, :header.line_bytes] = pack12p(raw.astype(np.uint16))
    return frame.reshape(display_height, display_width, 3)


def decode_frame(frame: np.ndarray) -> tuple[Header, np.ndarray]:
    """The inverse: a captured (height, width, 3) container -> (Header, raw).

    Runs on the RECEIVER's host, over whatever its capture path stored -- the
    same module that encoded the frame, so the two ends cannot drift apart.
    Raises rather than guessing on anything malformed; the error messages name
    the usual link-integrity suspects.
    """
    frame = np.asarray(frame, dtype=np.uint8)
    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError(f"expected (height, width, 3) bytes, got {frame.shape}")
    lines = frame.reshape(frame.shape[0], frame.shape[1] * 3)
    header = Header.unpack(lines[0, :HEADER_BYTES].tobytes())
    if header.height + 1 > lines.shape[0]:
        raise ValueError(
            f"header claims {header.height} payload lines but the container "
            f"has {lines.shape[0] - 1}")
    payload = lines[1:1 + header.height, :header.line_bytes]
    return header, unpack12p(payload)
