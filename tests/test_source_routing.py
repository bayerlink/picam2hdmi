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
    sent = []
    monkeypatch.setattr(serve, "_bridge_up", lambda *a, **k: True)
    monkeypatch.setattr(serve, "_bridge_command",
                        lambda line: sent.append(line) or "ok")
    assert serve.route_to_bridge({"source": "camera"}, Path("/tmp")) == "sensor"
    assert sent == ["bridge source sensor"]


def test_a_recording_goes_to_the_bridge_as_a_clip(monkeypatch, tmp_path):
    (tmp_path / "clip.npy").write_bytes(b"x")
    sent = []
    monkeypatch.setattr(serve, "_bridge_up", lambda *a, **k: True)
    monkeypatch.setattr(serve, "_prepare_clip", lambda p: {"frames": 3})
    monkeypatch.setattr(serve, "_bridge_command",
                        lambda line: sent.append(line) or "ok")
    drawn = serve.route_to_bridge({"source": "file", "file": "clip.npy"},
                                  tmp_path)
    assert "clip.npy" in drawn
    assert sent == [f"bridge source clip {serve.CLIP_PATH}"]


def test_a_refusal_is_raised_not_swallowed(monkeypatch, tmp_path):
    (tmp_path / "clip.npy").write_bytes(b"x")
    monkeypatch.setattr(serve, "_bridge_up", lambda *a, **k: True)
    monkeypatch.setattr(serve, "_prepare_clip", lambda p: {"frames": 1})
    monkeypatch.setattr(serve, "_bridge_command", lambda line: "err busy")
    with pytest.raises(ValueError, match="err busy"):
        serve.route_to_bridge({"source": "file", "file": "clip.npy"}, tmp_path)


def test_a_missing_recording_is_named(monkeypatch, tmp_path):
    monkeypatch.setattr(serve, "_bridge_up", lambda *a, **k: True)
    with pytest.raises(ValueError, match="no recording"):
        serve.route_to_bridge({"source": "file", "file": "gone.npy"}, tmp_path)
