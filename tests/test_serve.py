"""The instrument's contract, off-target: a fake runner, a real server."""
from __future__ import annotations

import http.client
import json
import threading

import numpy as np
import pytest

from bayerlink import encode_frame, pattern
from picam2hdmi.serve import Supervisor, _handler
from http.server import ThreadingHTTPServer


def _fake_runner(frames, mode=None, connector=None, card_path=None,
                 on_frame=None):
    """Consume frames like the real scanout, minus the silicon."""
    for index, _ in enumerate(frames):
        if on_frame is not None:
            on_frame(index)                    # raises _Stop on shutdown


@pytest.fixture()
def instrument(tmp_path):
    supervisor = Supervisor(tmp_path / "spool", "64x16@30",
                            runner=_fake_runner)
    server = ThreadingHTTPServer(("127.0.0.1", 0), _handler(supervisor))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield supervisor, server.server_address[1]
    server.shutdown()
    supervisor.stop()


def _request(port, method, path, body=None):
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    conn.request(method, path,
                 body=json.dumps(body).encode() if isinstance(body, dict)
                 else body)
    response = conn.getresponse()
    return response.status, json.loads(response.read())


def test_status_starts_idle(instrument):
    _, port = instrument
    code, status = _request(port, "GET", "/status")
    assert code == 200
    assert status["running"] is False and status["spool"] == []


def test_source_switches_and_counts_frames(instrument):
    supervisor, port = instrument
    code, status = _request(port, "PUT", "/source", {
        "source": "pattern", "pattern": "counting",
        "width": 16, "height": 4, "bayer": "RGGB"})
    assert code == 200 and status["running"] is True
    for _ in range(200):
        if supervisor.status()["frames"] > 10:
            break
    code, status = _request(port, "PUT", "/source", {"source": "off"})
    assert code == 200 and status["running"] is False


def test_a_bad_spec_is_a_400_with_the_reason(instrument):
    _, port = instrument
    code, body = _request(port, "PUT", "/source", {
        "source": "pattern", "width": 4096, "height": 4})
    assert code == 400
    assert "display line carries" in body["error"]
    code, body = _request(port, "PUT", "/source", {"source": "camera"})
    assert code == 400 and "Picamera2" in body["error"]
    code, body = _request(port, "PUT", "/source", {"source": "webcam"})
    assert code == 400 and "not one of" in body["error"]


def test_recordings_upload_list_and_replay(instrument, tmp_path):
    supervisor, port = instrument
    frame = encode_frame(pattern.generate("counting", 16, 4), "RGGB",
                         frame_seq=0, display=(64, 16))
    payload = tmp_path / "up.npy"
    np.save(payload, np.stack([frame, frame]))

    code, body = _request(port, "PUT", "/recordings/session.npy",
                          payload.read_bytes())
    assert code == 200 and body["saved"] == "session.npy"
    code, body = _request(port, "GET", "/recordings")
    assert body["recordings"] == ["session.npy"]

    code, status = _request(port, "PUT", "/source",
                            {"source": "file", "file": "session.npy"})
    assert code == 200 and status["running"] is True


def test_traversal_names_are_refused(instrument):
    _, port = instrument
    code, body = _request(port, "PUT", "/recordings/..%2fetc.npy", b"x")
    assert code == 400
    code, body = _request(port, "PUT", "/source",
                          {"source": "file", "file": "../../etc/passwd"})
    assert code == 400


def test_panel_preview_download_delete(instrument, tmp_path):
    supervisor, port = instrument
    # the panel
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    conn.request("GET", "/")
    response = conn.getresponse()
    assert response.status == 200
    assert b"picam2hdmi" in response.read()

    # stream a pattern -> preview is a PNG of the decoded raw
    code, _ = _request(port, "PUT", "/source", {
        "source": "pattern", "pattern": "gradient",
        "width": 16, "height": 4, "bayer": "RGGB"})
    assert code == 200
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    conn.request("GET", "/preview.png")
    response = conn.getresponse()
    body = response.read()
    assert response.status == 200 and body[:8] == b"\x89PNG\r\n\x1a\n"

    # upload -> download round-trips byte-identical; preview reads inside
    frame = encode_frame(pattern.generate("counting", 16, 4), "RGGB",
                         frame_seq=5, display=(64, 16))
    blob = tmp_path / "b.npy"
    np.save(blob, frame)
    payload = blob.read_bytes()
    code, _ = _request(port, "PUT", "/recordings/keep.npy", payload)
    assert code == 200
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    conn.request("GET", "/recordings/keep.npy")
    response = conn.getresponse()
    assert response.status == 200 and response.read() == payload
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    conn.request("GET", "/preview.png?recording=keep.npy")
    response = conn.getresponse()
    assert response.status == 200 and response.read()[:4] == b"\x89PNG"

    # delete, with traversal refused
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    conn.request("DELETE", "/recordings/..%2fkeep.npy")
    assert conn.getresponse().status in (400, 404)
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    conn.request("DELETE", "/recordings/keep.npy")
    assert conn.getresponse().status == 200
    assert supervisor.status()["spool"] == []


def test_crop_only_change_retargets_the_running_camera(instrument):
    import time

    supervisor, port = instrument
    # A fake camera mid-stream: a live thread, plus the facts
    # capture.frames fills into the crop ref on startup.
    thread = threading.Thread(target=time.sleep, args=(30,), daemon=True)
    thread.start()
    supervisor._thread = thread
    supervisor._camera_ref = {"sensor": (64, 48), "bits": 10,
                              "order": "GBRG", "display": (64, 16),
                              "target": (64, 16), "crop": (0, 0, 64, 12)}
    spec = {"source": "camera", "cam_mode": [64, 48], "exposure_us": 1000,
            "gain": 2.0, "luma_tunnel": False, "mode": "64x16@30"}
    supervisor.spec = dict(spec)

    # Only the window differs: retargeted live, same thread, no restart.
    code, status = _request(port, "PUT", "/source",
                            dict(spec, crop=[8, 2, 16, 8]))
    assert code == 200 and status["running"] is True
    assert supervisor._camera_ref["crop"] == (8, 2, 16, 8)
    assert supervisor._thread is thread

    # A refused window is still a named 400, and the stream keeps its
    # old window -- validation happens before anything is written.
    code, body = _request(port, "PUT", "/source",
                          dict(spec, crop=[1, 0, 16, 8]))
    assert code == 400 and "multiples" in body["error"]
    assert supervisor._camera_ref["crop"] == (8, 2, 16, 8)

    # Too tall for the display: the library's own refusal comes back.
    code, body = _request(port, "PUT", "/source",
                          dict(spec, crop=[0, 0, 16, 48]))
    assert code == 400 and "display" in body["error"]
    assert supervisor._camera_ref["crop"] == (8, 2, 16, 8)

    # Anything beyond geometry (gain here) is a real restart path.
    assert not supervisor._crop_only_change(dict(spec, gain=4.0))
    supervisor._thread = None             # hand the fake back untouched


def test_state_retention_and_restore(instrument):
    supervisor, port = instrument
    spec = {"source": "pattern", "pattern": "gradient",
            "width": 16, "height": 4, "bayer": "GBRG"}
    code, _ = _request(port, "PUT", "/source", spec)
    assert code == 200

    # The accepted spec survives in the spool, invisible to /recordings.
    stored = supervisor.stored_spec()
    assert stored["pattern"] == "gradient" and "mode" in stored
    code, body = _request(port, "GET", "/recordings")
    assert body["recordings"] == []

    # A fresh supervisor over the same spool restores it -- the boot path.
    twin = Supervisor(supervisor.spool, "64x16@30", runner=_fake_runner)
    assert twin.stored_spec()["pattern"] == "gradient"

    # 'off' is a retained state too: an instrument switched off stays off.
    code, _ = _request(port, "PUT", "/source", {"source": "off"})
    assert code == 200
    assert supervisor.stored_spec() is None or \
        supervisor.stored_spec()["source"] == "off"


def test_power_button(instrument, tmp_path, monkeypatch):
    import time

    from picam2hdmi import serve as serve_module

    supervisor, port = instrument
    halted = tmp_path / "halted"
    monkeypatch.setattr(serve_module, "POWEROFF", ("touch", str(halted)))

    # Anything but {"off": true} is refused -- power is not a default.
    code, body = _request(port, "PUT", "/power", {})
    assert code == 400 and "off" in body["error"]
    assert not halted.exists()

    code, body = _request(port, "PUT", "/power", {"off": True})
    assert code == 200 and body["powering_off"] is True
    deadline = time.monotonic() + 5
    while not halted.exists() and time.monotonic() < deadline:
        time.sleep(0.05)
    assert halted.exists()


def test_the_keeper_revives_retained_intent(instrument):
    supervisor, port = instrument
    # Streaming, then the world takes the stream away (thread dies).
    code, _ = _request(port, "PUT", "/source", {
        "source": "pattern", "pattern": "counting", "width": 16, "height": 4})
    assert code == 200
    supervisor._stop.set()
    supervisor._thread.join(timeout=5)
    assert not supervisor._thread.is_alive()

    # One keeper beat brings it back from the persisted intent.
    supervisor._stop.clear()
    assert supervisor.revive_once() is True
    assert supervisor.status()["running"] is True

    # An explicit off is intent too: the keeper leaves it alone.
    code, _ = _request(port, "PUT", "/source", {"source": "off"})
    assert code == 200
    assert supervisor.revive_once() is False
    assert supervisor.status()["running"] is False


def test_cameras_endpoint_lists_or_is_empty(instrument):
    _, port = instrument
    code, body = _request(port, "GET", "/cameras")
    assert code == 200
    assert isinstance(body["cameras"], list)   # no camera stack here: []


def test_a_choice_made_in_the_dark_is_kept_as_intent(instrument, monkeypatch):
    """A dark display refuses the START, not the CHOICE.

    Choosing a source while the cable is out must still become the
    retained intent -- the keeper realises it the moment a display
    appears -- while a genuinely bad request is refused AND forgotten.
    """
    from picam2hdmi import output
    from picam2hdmi.kms import DisplayNotReady

    supervisor, port = instrument
    real = output.pattern_frames

    def dark(*args, **kwargs):
        raise DisplayNotReady("no connected connector with modes")
    monkeypatch.setattr(output, "pattern_frames", dark)

    wanted = {"source": "pattern", "pattern": "counting",
              "width": 16, "height": 4, "bayer": "RGGB"}
    code, body = _request(port, "PUT", "/source", wanted)
    assert code == 400 and "connector" in body["error"]
    assert supervisor.status()["running"] is False
    stored = supervisor.stored_spec()
    assert stored["source"] == "pattern" and stored["pattern"] == "counting"

    # A bad REQUEST is a different animal: refused and not retained.
    code, body = _request(port, "PUT", "/source", {"source": "file",
                                                   "file": "nope.txt"})
    assert code == 400
    assert supervisor.stored_spec()["source"] == "pattern"

    # The display comes back; one keeper beat realises the intent.
    monkeypatch.setattr(output, "pattern_frames", real)
    assert supervisor.revive_once() is True
    assert supervisor.status()["running"] is True


def test_capture_freezes_the_emitted_frame_into_the_spool(instrument):
    supervisor, port = instrument
    code, _ = _request(port, "POST", "/capture")
    assert code == 400                       # nothing streaming yet

    code, _ = _request(port, "PUT", "/source", {
        "source": "pattern", "pattern": "counting",
        "width": 16, "height": 4, "bayer": "RGGB"})
    assert code == 200
    code, body = _request(port, "POST", "/capture")
    assert code == 200 and body["saved"] == "capture-001.npy"
    code, body = _request(port, "POST", "/capture")
    assert body["saved"] == "capture-002.npy"    # subsequent, not clobbered
    assert "capture-001.npy" in supervisor.status()["spool"]

    # And a capture IS a recording: it streams like one.
    code, _ = _request(port, "PUT", "/source",
                       {"source": "file", "file": "capture-001.npy"})
    assert code == 200
    assert supervisor.status()["running"] is True


def test_the_panel_is_stamped_and_status_names_the_stamp(instrument):
    _, port = instrument
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    conn.request("GET", "/")
    page = conn.getresponse().read().decode()
    assert "@PANEL@" not in page             # stamped, not the placeholder
    code, status = _request(port, "GET", "/status")
    assert code == 200 and status["panel"] in page


def test_the_source_runs_as_a_viewfinder_when_the_display_is_dark(
        instrument, monkeypatch):
    """No display does not mean no source: the stream falls back to a
    monitor loop -- frames flow, the preview feeds, status says the
    wire is NOT driven -- and the loop ends by itself the moment a
    display appears, handing the keeper a dead thread and a live
    intent, which is exactly what it revives."""
    import time as _time

    from picam2hdmi import kms, serve

    supervisor, port = instrument
    dark = {"value": True}
    monkeypatch.setattr(kms, "display_present",
                        lambda: not dark["value"])

    def blind_runner(frames, mode=None, connector=None, card_path=None,
                     on_frame=None):
        raise kms.DisplayNotReady("no connected connector with modes")
    supervisor._runner = blind_runner

    code, status = _request(port, "PUT", "/source", {
        "source": "pattern", "pattern": "counting",
        "width": 16, "height": 4, "bayer": "RGGB"})
    assert code == 200
    for _ in range(100):
        if supervisor.status()["frames"] > 3:
            break
        _time.sleep(0.05)
    status = supervisor.status()
    assert status["running"] is True and status["display"] is False
    assert status["frames"] > 3                  # the viewfinder is alive

    # A display appears: the monitor loop ends within its poll second...
    supervisor._runner = _fake_runner
    dark["value"] = False
    supervisor._thread.join(timeout=5)
    assert not supervisor._thread.is_alive()
    # ...and one keeper beat brings the same intent up on the real path.
    assert supervisor.revive_once() is True
    status = supervisor.status()
    assert status["running"] is True and status["display"] is True
