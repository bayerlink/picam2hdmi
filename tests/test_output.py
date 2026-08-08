"""The off-target half of scanout: everything that does not need a Pi.

The ioctl path is proven on hardware in phase 0; these tests pin what CAN
be pinned here -- the ctypes struct sizes against the kernel uapi (a mislaid
field fails as corruption at runtime and as one line here), the container
packing for both pixel formats, the header re-stamping, and mode choice.
"""
import ctypes

import numpy as np
import pytest
from bayerlink import decode_frame, encode_frame, pattern

from picam2hdmi import kms
from picam2hdmi.output import patch_frame_seq, write_container


def test_struct_sizes_match_the_kernel_uapi():
    expected = {
        kms.ModeInfo: 68, kms.CardRes: 64, kms.GetConnector: 80,
        kms.GetEncoder: 20, kms.CreateDumb: 32, kms.MapDumb: 16,
        kms.FbCmd2: 104, kms.Crtc: 104, kms.PageFlip: 24,
        kms.GetProperty: 64, kms.PropertyEnum: 40,
        kms.SetConnectorProperty: 16,
    }
    for struct, size in expected.items():
        assert ctypes.sizeof(struct) == size, struct.__name__


class FakeBuffer:
    def __init__(self, height, pitch):
        self.pitch = pitch
        self.view = memoryview(bytearray(height * pitch))


def _container():
    raw = pattern.generate("corners", 16, 4)
    return raw, encode_frame(raw, "RGGB", frame_seq=5, display=(16, 8))


def test_rg24_is_the_container_memcpyd_with_pitch_padding():
    raw, container = _container()
    fb = FakeBuffer(height=8, pitch=16 * 3 + 8)          # padded pitch
    write_container(fb, container, kms.FORMAT_RG24)
    rows = np.frombuffer(fb.view, np.uint8).reshape(8, 16 * 3 + 8)
    assert np.array_equal(rows[:, :48].reshape(8, 16, 3), container)
    assert not rows[:, 48:].any()

    header, decoded = decode_frame(rows[:, :48].reshape(8, 16, 3))
    assert np.array_equal(decoded, raw) and header.frame_seq == 5


def test_xr24_places_the_same_bytes_on_the_same_channels():
    _, container = _container()
    fb = FakeBuffer(height=8, pitch=16 * 4)
    write_container(fb, container, kms.FORMAT_XR24)
    pixels = np.frombuffer(fb.view, np.uint8).reshape(8, 16, 4)
    assert np.array_equal(pixels[:, :, :3], container)   # channel k identical
    assert not pixels[:, :, 3].any()                     # X byte zero


def test_patch_frame_seq_restamps_header_and_crc_only():
    raw, container = _container()
    before = container.copy()
    patch_frame_seq(container, 77)
    header, decoded = decode_frame(container)            # CRC must still pass
    assert header.frame_seq == 77
    assert np.array_equal(decoded, raw)
    # Nothing outside the 32 header bytes moved.
    flat_a = before.reshape(8, -1)
    flat_b = container.reshape(8, -1)
    assert np.array_equal(flat_a[1:], flat_b[1:])
    assert np.array_equal(flat_a[0, 32:], flat_b[0, 32:])


def _mode(w, h, hz, preferred=False):
    mode = kms.ModeInfo()
    mode.hdisplay, mode.vdisplay, mode.vrefresh = w, h, hz
    mode.type = kms.MODE_TYPE_PREFERRED if preferred else 0
    return mode


def test_pick_mode_takes_the_exact_request_or_refuses():
    modes = [_mode(1920, 1080, 60, preferred=True), _mode(1920, 1080, 30),
             _mode(1280, 720, 60)]
    assert kms.pick_mode(modes, (1920, 1080, 30)).vrefresh == 30
    with pytest.raises(RuntimeError, match="Refusing a near miss"):
        kms.pick_mode(modes, (1920, 1080, 25))
    assert kms.pick_mode(modes, None).vrefresh == 60     # preferred flag wins


def test_pattern_frames_through_the_luma_tunnel_round_trip():
    """Encoder-side proof: what stream() would scan out, decoded end to end."""
    from bayerlink import tunnel
    from picam2hdmi.output import pattern_frames

    display = (192, 40)
    frames = pattern_frames("corners", 32, 8, "RGGB", display,
                            luma_tunnel=True)
    grey0 = next(frames)
    grey1 = next(frames)
    assert grey0.shape == (40, 192, 3)
    assert (grey0[:, :, 0] == grey0[:, :, 1]).all()

    inner_w, inner_h = tunnel.inner_display(*display)
    for index, grey in ((0, grey0), (1, grey1)):
        container = tunnel.decode(grey[:, :, 0], inner_height=inner_h)
        header, raw = decode_frame(container)
        assert header.frame_seq == index
        assert np.array_equal(raw, pattern.generate("corners", 32, 8))
