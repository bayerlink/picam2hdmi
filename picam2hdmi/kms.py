"""DRM/KMS via raw ioctls: mode setting, dumb buffers, vsync page flips.

ctypes against /dev/dri directly, no bindings package: an appliance should
not carry a dependency for the eight ioctls it actually uses, and the DRM
uapi these structs mirror is a stable kernel ABI. Every struct here is
checked against its known uapi size by the test suite, because a mislaid
field in a ctypes definition fails as corruption at runtime and as a
one-line assert at test time.

The scanout model is the classic double buffer: two dumb buffers, draw into
the back one, PAGE_FLIP with an event, wait for the event, swap. A flip
replaces the whole frame atomically at vblank -- the receiver never sees a
half-written container, for the same reason shadow registers exist on the
other end of this link.

The one policy this layer enforces rather than exposes: **full-range RGB**.
If the connector has a "Broadcast RGB" property, it is set to "Full" during
setup, because a limited-range link clamps to 16..235 and silently destroys
exactly the sample values raw Bayer lives on. That failure looks like a
mysteriously broken image three components later; forcing the property here
turns it into a non-event.
"""
from __future__ import annotations

import ctypes
import fcntl
import mmap as _mmap
import os
from dataclasses import dataclass

# --------------------------------------------------------------------------- #
# uapi structs (drm.h / drm_mode.h), sizes pinned by tests
# --------------------------------------------------------------------------- #

u16, u32, u64 = ctypes.c_uint16, ctypes.c_uint32, ctypes.c_uint64


class ModeInfo(ctypes.Structure):
    _fields_ = [("clock", u32),
                ("hdisplay", u16), ("hsync_start", u16), ("hsync_end", u16),
                ("htotal", u16), ("hskew", u16),
                ("vdisplay", u16), ("vsync_start", u16), ("vsync_end", u16),
                ("vtotal", u16), ("vscan", u16),
                ("vrefresh", u32), ("flags", u32), ("type", u32),
                ("name", ctypes.c_char * 32)]          # 68 bytes



class DisplayNotReady(RuntimeError):
    """The bench has no lit display RIGHT NOW -- a state, not a
    mistake. Callers that hold an intent may keep it and try again;
    everything else treats it as the RuntimeError it also is."""

class CardRes(ctypes.Structure):
    _fields_ = [("fb_id_ptr", u64), ("crtc_id_ptr", u64),
                ("connector_id_ptr", u64), ("encoder_id_ptr", u64),
                ("count_fbs", u32), ("count_crtcs", u32),
                ("count_connectors", u32), ("count_encoders", u32),
                ("min_width", u32), ("max_width", u32),
                ("min_height", u32), ("max_height", u32)]   # 64 bytes


class GetConnector(ctypes.Structure):
    _fields_ = [("encoders_ptr", u64), ("modes_ptr", u64),
                ("props_ptr", u64), ("prop_values_ptr", u64),
                ("count_modes", u32), ("count_props", u32),
                ("count_encoders", u32), ("encoder_id", u32),
                ("connector_id", u32), ("connector_type", u32),
                ("connector_type_id", u32), ("connection", u32),
                ("mm_width", u32), ("mm_height", u32),
                ("subpixel", u32), ("pad", u32)]            # 80 bytes


class GetEncoder(ctypes.Structure):
    _fields_ = [("encoder_id", u32), ("encoder_type", u32),
                ("crtc_id", u32), ("possible_crtcs", u32),
                ("possible_clones", u32)]                   # 20 bytes


class CreateDumb(ctypes.Structure):
    _fields_ = [("height", u32), ("width", u32), ("bpp", u32), ("flags", u32),
                ("handle", u32), ("pitch", u32), ("size", u64)]  # 32 bytes


class MapDumb(ctypes.Structure):
    _fields_ = [("handle", u32), ("pad", u32), ("offset", u64)]  # 16 bytes


class FbCmd2(ctypes.Structure):
    _fields_ = [("fb_id", u32), ("width", u32), ("height", u32),
                ("pixel_format", u32), ("flags", u32),
                ("handles", u32 * 4), ("pitches", u32 * 4),
                ("offsets", u32 * 4), ("modifier", u64 * 4)]  # 104 bytes


class Crtc(ctypes.Structure):
    _fields_ = [("set_connectors_ptr", u64), ("count_connectors", u32),
                ("crtc_id", u32), ("fb_id", u32), ("x", u32), ("y", u32),
                ("gamma_size", u32), ("mode_valid", u32),
                ("mode", ModeInfo)]                         # 104 bytes


class PageFlip(ctypes.Structure):
    _fields_ = [("crtc_id", u32), ("fb_id", u32), ("flags", u32),
                ("reserved", u32), ("user_data", u64)]      # 24 bytes


class GetProperty(ctypes.Structure):
    _fields_ = [("values_ptr", u64), ("enum_blob_ptr", u64),
                ("prop_id", u32), ("flags", u32),
                ("name", ctypes.c_char * 32),
                ("count_values", u32), ("count_enum_blobs", u32)]  # 64 bytes


class PropertyEnum(ctypes.Structure):
    _fields_ = [("value", u64), ("name", ctypes.c_char * 32)]  # 40 bytes


class SetConnectorProperty(ctypes.Structure):
    _fields_ = [("value", u64), ("prop_id", u32),
                ("connector_id", u32)]                      # 16 bytes


def _iowr(nr: int, struct) -> int:
    return (3 << 30) | (ctypes.sizeof(struct) << 16) | (ord("d") << 8) | nr


IOCTL_GETRESOURCES = _iowr(0xA0, CardRes)
IOCTL_SETCRTC = _iowr(0xA2, Crtc)
IOCTL_GETENCODER = _iowr(0xA6, GetEncoder)
IOCTL_GETCONNECTOR = _iowr(0xA7, GetConnector)
IOCTL_GETPROPERTY = _iowr(0xAA, GetProperty)
IOCTL_SETPROPERTY = _iowr(0xAB, SetConnectorProperty)
IOCTL_PAGE_FLIP = _iowr(0xB0, PageFlip)
IOCTL_CREATE_DUMB = _iowr(0xB2, CreateDumb)
IOCTL_MAP_DUMB = _iowr(0xB3, MapDumb)
IOCTL_ADDFB2 = _iowr(0xB8, FbCmd2)

CONNECTOR_HDMIA = 11
CONNECTED = 1
MODE_TYPE_PREFERRED = 1 << 3
PAGE_FLIP_EVENT = 0x01
EVENT_FLIP_COMPLETE = 0x02


def fourcc(text: str) -> int:
    return int.from_bytes(text.encode("ascii"), "little")


# RG24: 3 bytes per pixel, memory B,G,R -- the container memcpy'd, byte for
# byte. XR24: 4 bytes per pixel, memory B,G,R,X -- universally supported
# fallback. Container byte k lands on the SAME colour channel either way
# (0->B, 1->G, 2->R), so the receiver's lane_map does not depend on which
# format won; only the in-memory stride does.
FORMAT_RG24 = fourcc("RG24")
FORMAT_XR24 = fourcc("XR24")


def pick_mode(modes, want=None):
    """Choose the display mode: exact request, else the connector's preferred.

    ``modes`` is a sequence of ModeInfo (or anything with hdisplay/vdisplay/
    vrefresh/type). ``want`` is (width, height, hz) or None. Pure function,
    tested off-target, because mode selection is where a bring-up quietly
    lands on 1024x768 and every budget in PROTOCOL.md is silently wrong.
    """
    modes = list(modes)
    if not modes:
        raise RuntimeError("connector reports no modes; is a cable attached?")
    if want is not None:
        width, height, hz = want
        for mode in modes:
            if (mode.hdisplay, mode.vdisplay, mode.vrefresh) == (width, height, hz):
                return mode
        available = sorted({(m.hdisplay, m.vdisplay, m.vrefresh) for m in modes})
        raise RuntimeError(
            f"no {width}x{height}@{hz} on this connector; it offers {available}. "
            "Refusing a near miss: the receiver's budgets are per-mode.")
    for mode in modes:
        if mode.type & MODE_TYPE_PREFERRED:
            return mode
    return modes[0]


@dataclass
class Framebuffer:
    """One dumb buffer: the kernel object ids and the mapped bytes."""

    fb_id: int
    handle: int
    pitch: int
    view: memoryview


class Card:
    """One DRM device, opened for mode setting.

    The lifecycle a caller sees: ``open()``, ``connector()``, ``mode()``,
    ``force_full_range()``, ``framebuffers()``, ``set_crtc()``, then
    ``flip()`` / ``wait_flip()`` per frame. Everything else is plumbing.
    """

    def __init__(self, fd: int, path: str):
        self.fd = fd
        self.path = path

    @classmethod
    def open(cls, path: str | None = None) -> "Card":
        candidates = [path] if path else sorted(
            f"/dev/dri/{name}" for name in os.listdir("/dev/dri")
            if name.startswith("card"))
        last = None
        for candidate in candidates:
            try:
                return cls(os.open(candidate, os.O_RDWR), candidate)
            except OSError as error:
                last = error
        raise RuntimeError(f"no usable DRM device among {candidates}: {last}")

    def _ioctl(self, request: int, arg) -> None:
        fcntl.ioctl(self.fd, request, arg)

    def close(self) -> None:
        """Release the device -- and with it, DRM master.

        A daemon that restarts its stream must close the old card before
        opening the next one, or the second modeset loses the master fight
        against its own predecessor."""
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1

    # -- discovery ------------------------------------------------------------ #

    def connector(self, connector_id: int | None = None):
        """The connected connector (HDMI preferred), its modes, and a CRTC.

        Returns ``(connector_id, crtc_id, [ModeInfo, ...])``.
        """
        res = CardRes()
        self._ioctl(IOCTL_GETRESOURCES, res)
        connector_ids = (u32 * max(1, res.count_connectors))()
        crtc_ids = (u32 * max(1, res.count_crtcs))()
        res.connector_id_ptr = ctypes.addressof(connector_ids)
        res.crtc_id_ptr = ctypes.addressof(crtc_ids)
        # The kernel writes through EVERY pointer whose count is nonzero --
        # fbcon guarantees at least one framebuffer on a real device, so the
        # categories this caller does not want must say count 0, exactly as
        # _get_connector already does for encoders. A NULL pointer with a
        # nonzero count is EFAULT on hardware and silence in a fake.
        res.count_fbs = 0
        res.count_encoders = 0
        self._ioctl(IOCTL_GETRESOURCES, res)

        chosen = None
        for cid in connector_ids:
            if connector_id is not None and cid != connector_id:
                continue
            conn, modes = self._get_connector(cid)
            if conn.connection != CONNECTED or not modes:
                continue
            hdmi = conn.connector_type == CONNECTOR_HDMIA
            if chosen is None or (hdmi and not chosen[3]):
                chosen = (cid, conn, modes, hdmi)
        if chosen is None:
            raise DisplayNotReady(
                "no connected connector with modes"
                + (f" (id {connector_id})" if connector_id else "")
                + "; is the cable in and the receiver awake?")
        cid, conn, modes, _ = chosen

        crtc_id = None
        if conn.encoder_id:
            encoder = GetEncoder(encoder_id=conn.encoder_id)
            self._ioctl(IOCTL_GETENCODER, encoder)
            crtc_id = encoder.crtc_id or None
        if crtc_id is None:
            if not res.count_crtcs:
                raise RuntimeError("device has no CRTCs")
            crtc_id = crtc_ids[0]
        return cid, crtc_id, modes

    def _get_connector(self, connector_id: int):
        conn = GetConnector(connector_id=connector_id)
        self._ioctl(IOCTL_GETCONNECTOR, conn)
        modes = (ModeInfo * max(1, conn.count_modes))()
        props = (u32 * max(1, conn.count_props))()
        values = (u64 * max(1, conn.count_props))()
        conn.modes_ptr = ctypes.addressof(modes)
        conn.props_ptr = ctypes.addressof(props)
        conn.prop_values_ptr = ctypes.addressof(values)
        conn.count_encoders = 0
        self._ioctl(IOCTL_GETCONNECTOR, conn)
        self._props = list(props[:conn.count_props])
        return conn, list(modes[:conn.count_modes])

    # -- the one policy --------------------------------------------------------#

    def force_full_range(self, connector_id: int) -> bool:
        """Set "Broadcast RGB" to "Full" if the connector has it.

        Returns whether the property existed. A limited-range link clamps to
        16..235 -- precisely the values raw data lives on -- and presents as
        an image that is wrong in a way nothing names. Forced here, once,
        instead of debugged downstream, repeatedly.
        """
        self._get_connector(connector_id)
        for prop_id in self._props:
            prop = GetProperty(prop_id=prop_id)
            self._ioctl(IOCTL_GETPROPERTY, prop)
            if prop.name != b"Broadcast RGB":
                continue
            enums = (PropertyEnum * max(1, prop.count_enum_blobs))()
            prop.enum_blob_ptr = ctypes.addressof(enums)
            values = (u64 * max(1, prop.count_values))()
            prop.values_ptr = ctypes.addressof(values)
            self._ioctl(IOCTL_GETPROPERTY, prop)
            for entry in enums[:prop.count_enum_blobs]:
                if entry.name == b"Full":
                    self._ioctl(IOCTL_SETPROPERTY, SetConnectorProperty(
                        value=entry.value, prop_id=prop_id,
                        connector_id=connector_id))
                    return True
            raise RuntimeError(
                "connector has Broadcast RGB but no 'Full' choice; refusing "
                "to stream data through a range conversion")
        return False

    # -- buffers ---------------------------------------------------------------#

    def framebuffer(self, mode: ModeInfo, pixel_format: int) -> Framebuffer:
        bpp = 24 if pixel_format == FORMAT_RG24 else 32
        dumb = CreateDumb(height=mode.vdisplay, width=mode.hdisplay, bpp=bpp)
        self._ioctl(IOCTL_CREATE_DUMB, dumb)

        cmd = FbCmd2(width=mode.hdisplay, height=mode.vdisplay,
                     pixel_format=pixel_format)
        cmd.handles[0] = dumb.handle
        cmd.pitches[0] = dumb.pitch
        self._ioctl(IOCTL_ADDFB2, cmd)

        where = MapDumb(handle=dumb.handle)
        self._ioctl(IOCTL_MAP_DUMB, where)
        mapping = _mmap.mmap(self.fd, dumb.size, offset=where.offset)
        return Framebuffer(fb_id=cmd.fb_id, handle=dumb.handle,
                           pitch=dumb.pitch, view=memoryview(mapping))

    def framebuffers(self, mode: ModeInfo, count: int = 2):
        """``(pixel_format, [Framebuffer, ...])``: RG24 if the driver takes it,
        XR24 otherwise. The container's channel placement is identical either
        way; only the stride differs, and the writer handles that."""
        for pixel_format in (FORMAT_RG24, FORMAT_XR24):
            try:
                buffers = [self.framebuffer(mode, pixel_format)
                           for _ in range(count)]
                return pixel_format, buffers
            except OSError:
                continue
        raise RuntimeError("driver refused both RG24 and XR24 dumb buffers")

    # -- scanout ---------------------------------------------------------------#

    def set_crtc(self, crtc_id: int, connector_id: int, mode: ModeInfo,
                 framebuffer: Framebuffer) -> None:
        connectors = (u32 * 1)(connector_id)
        crtc = Crtc(set_connectors_ptr=ctypes.addressof(connectors),
                    count_connectors=1, crtc_id=crtc_id,
                    fb_id=framebuffer.fb_id, mode_valid=1, mode=mode)
        self._ioctl(IOCTL_SETCRTC, crtc)

    def flip(self, crtc_id: int, framebuffer: Framebuffer) -> None:
        self._ioctl(IOCTL_PAGE_FLIP, PageFlip(
            crtc_id=crtc_id, fb_id=framebuffer.fb_id, flags=PAGE_FLIP_EVENT))

    def wait_flip(self) -> None:
        """Block until the queued flip completes (the vblank event arrives).

        Draw into a buffer only after the flip AWAY from it has completed;
        that discipline, not the double buffer itself, is what makes tearing
        impossible.
        """
        while True:
            data = os.read(self.fd, 1024)
            offset = 0
            while offset + 8 <= len(data):
                kind, length = (int.from_bytes(data[offset:offset + 4], "little"),
                                int.from_bytes(data[offset + 4:offset + 8], "little"))
                if kind == EVENT_FLIP_COMPLETE:
                    return
                offset += max(length, 8)
