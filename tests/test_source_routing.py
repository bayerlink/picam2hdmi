# Copyright 2026 Serge Rabyking
# SPDX-License-Identifier: MIT
"""One decision about which engine draws a source.

The panel used to make it twice: the HTTP route chose the bridge, and
the keeper -- realising the same stored intent unattended -- called this
daemon's own engine. A recording selected by a person went to the
bridge; the identical recording revived after a restart was drawn here,
which took the display and left the bridge unable to modeset.
"""
from pathlib import Path

import pytest

from picam2hdmi import serve


def test_a_pattern_is_not_the_bridges_to_draw(monkeypatch):
    monkeypatch.setattr(serve, "_bridge_up", lambda *a, **k: True)
    assert serve.route_to_bridge({"source": "pattern"}, Path("/tmp")) is None


def test_without_a_bridge_nothing_routes(monkeypatch):
    monkeypatch.setattr(serve, "_bridge_up", lambda *a, **k: False)
    assert serve.route_to_bridge({"source": "camera"}, Path("/tmp")) is None


def test_a_camera_goes_to_the_bridge(monkeypatch):
    """Streaming the sensor already, the camera asks nothing of it."""
    sent = []
    monkeypatch.setattr(serve, "_bridge_up", lambda *a, **k: True)
    monkeypatch.setattr(serve, "_bridge_plays_clip", lambda: False)
    monkeypatch.setattr(serve, "_bridge_command",
                        lambda line, **k: sent.append(line) or "ok")
    assert serve.route_to_bridge({"source": "camera"}, Path("/tmp")) == "sensor"
    assert sent == []


def test_a_camera_after_a_clip_is_a_reconfiguration(monkeypatch, tmp_path):
    """A clip run has no sensor: the camera comes back by reconfiguring the
    bridge to the options it streamed with."""
    sent = []
    env = tmp_path / "camera.env"
    env.write_text("BRIDGE_ARGS=--mode 1920x1080 --line-ns 29623\n")
    monkeypatch.setattr(serve, "CAMERA_ENV", env)
    monkeypatch.setattr(serve, "_bridge_up", lambda *a, **k: True)
    monkeypatch.setattr(serve, "_bridge_plays_clip", lambda: True)
    monkeypatch.setattr(serve, "_bridge_reconfigure",
                        lambda args: sent.append(args) or "ok sensor")
    assert serve.route_to_bridge({"source": "camera"}, Path("/tmp")) == "sensor"
    assert sent == ["--mode 1920x1080 --line-ns 29623"]


def test_a_recording_goes_to_the_bridge_as_a_clip(monkeypatch, tmp_path):
    """A recording plays as a clip of its own containers -- their width,
    and the raster streaming now, which holds them."""
    (tmp_path / "clip.npy").write_bytes(b"x")
    sent = []
    monkeypatch.setattr(serve, "_bridge_up", lambda *a, **k: True)
    monkeypatch.setattr(serve, "_remember_camera", lambda: None)
    monkeypatch.setattr(serve, "_prepare_clip",
                        lambda p: {"frames": 3, "width": 800, "height": 1080})
    monkeypatch.setattr(serve, "_bridge_status",
                        lambda: {"h_total": "4400", "v_total": "1125"})
    monkeypatch.setattr(serve, "_bridge_reconfigure",
                        lambda args: sent.append(args) or "ok clip")
    drawn = serve.route_to_bridge({"source": "file", "file": "clip.npy"},
                                  tmp_path)
    assert "clip.npy" in drawn
    assert sent == [f"--clip {serve.CLIP_PATH} --containers 800 "
                    "--line-clocks 4400 --frame-lines 1125"]


def test_a_recording_wider_than_the_line_gets_a_line_that_holds_it():
    """An old recording, 1920 link pixels wide, on 1080p60's 2200-clock
    line still fits; on a line too short it gets the least that holds it,
    and a frame too short for it the least that does."""
    info = {"frames": 1, "width": 1920, "height": 1080}
    assert serve.recording_args(info, {"h_total": 2200, "v_total": 1125}) \
        .endswith("--line-clocks 2200 --frame-lines 1125")
    assert serve.recording_args(info, {"h_total": 1000, "v_total": 600}) \
        .endswith("--line-clocks 1968 --frame-lines 1091")


def test_a_refusal_is_raised_not_swallowed(monkeypatch, tmp_path):
    (tmp_path / "clip.npy").write_bytes(b"x")
    monkeypatch.setattr(serve, "_bridge_up", lambda *a, **k: True)
    monkeypatch.setattr(serve, "_remember_camera", lambda: None)
    monkeypatch.setattr(serve, "_prepare_clip",
                        lambda p: {"frames": 1, "width": 800, "height": 1080})
    monkeypatch.setattr(serve, "_bridge_status",
                        lambda: {"h_total": "4400", "v_total": "1125"})
    monkeypatch.setattr(serve, "_bridge_reconfigure", lambda args: "err busy")
    with pytest.raises(ValueError, match="err busy"):
        serve.route_to_bridge({"source": "file", "file": "clip.npy"}, tmp_path)


def test_a_missing_recording_is_named(monkeypatch, tmp_path):
    monkeypatch.setattr(serve, "_bridge_up", lambda *a, **k: True)
    with pytest.raises(ValueError, match="no recording"):
        serve.route_to_bridge({"source": "file", "file": "gone.npy"}, tmp_path)
