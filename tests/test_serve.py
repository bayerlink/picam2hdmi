# Copyright 2026 Serge Rabyking
# SPDX-License-Identifier: MIT
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
                 container=False,
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
            "gain": 2.0, "mode": "64x16@30"}
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

    # A gain in a spec reaches the SENSOR through nothing here. Exposure
    # and gain live in the sensor package, with the AE loop; this daemon
    # writing them would be a second owner of values that loop is
    # already driving. The live lane exists for the WINDOW.
    class _Recorder:
        def __init__(self):
            self.controls = []

        def set_controls(self, c):
            self.controls.append(c)

    recorder = _Recorder()
    supervisor._camera_ref["picam2"] = recorder
    supervisor._live_controls(dict(spec, gain=4.0))
    assert not any("AnalogueGain" in c or "ExposureTime" in c
                   for c in recorder.controls), (
        "the daemon must not write the sensor's exposure or gain: they "
        "belong to the sensor package, where the AE loop lives")

    # The world-touching keys still restart: a different sensor mode
    # is a different stream.
    assert not supervisor._crop_only_change(dict(spec, cam_mode=[32, 24]))

    # The sensor's OWN mode list reaches the panel through /cameras.
    # A panel that cannot see which modes exist keeps a list of its
    # own, and a stale list is how one click re-applies a mode nobody
    # asked for -- which is exactly how a 1920x1080 stream once fell
    # back to 1296x972 mid-bench.
    assert supervisor.sensor_modes() == []
    supervisor._camera_ref["modes"] = [(16, 8), (32, 24)]
    assert supervisor.sensor_modes() == [(16, 8), (32, 24)]
    code, body = _request(port, "GET", "/cameras")
    assert code == 200
    reported = [c for c in body["cameras"] if c.get("modes")]
    assert reported == [] or reported[0]["modes"] == [[16, 8], [32, 24]]
    supervisor._thread = None             # hand the fake back untouched


def test_state_retention_and_restore(instrument):
    supervisor, port = instrument
    spec = {"source": "pattern", "pattern": "gradient",
            "width": 16, "height": 4, "bayer": "GBRG"}
    code, _ = _request(port, "PUT", "/source", spec)
    assert code == 200

    # The accepted spec survives in the spool, invisible to /recordings.
    stored = supervisor.stored_spec()
    assert stored["pattern"] == "gradient"
    # what was STATED is stored; what was DERIVED is not -- a derived
    # value written back would become an explicit ask next round trip
    assert "mode" not in stored
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
                     container=False,
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


def test_the_container_is_derived_content_plus_one(instrument):
    """State geometry once; the +1 and the raster are nobody's business."""
    from picam2hdmi.serve import _Stop
    import time
    supervisor, port = instrument
    calls = []

    def spy(frames, mode=None, connector=None, card_path=None,
            container=False, on_frame=None):
        calls.append((mode, container))
        next(iter(frames))
        raise _Stop()

    supervisor._runner = spy
    _request(port, "PUT", "/source",
             {"source": "pattern", "pattern": "counting",
              "width": 512, "height": 240, "bayer": "RGGB"})
    deadline = time.monotonic() + 2
    while not calls and time.monotonic() < deadline:
        time.sleep(0.01)
    assert calls and calls[0] == ((512, 241, 60), True)

    # an explicit mode still wins, and is not a container
    calls.clear()
    _request(port, "PUT", "/source",
             {"source": "pattern", "pattern": "counting", "width": 512,
              "height": 240, "bayer": "RGGB", "mode": "1280x720@60"})
    deadline = time.monotonic() + 2
    while not calls and time.monotonic() < deadline:
        time.sleep(0.01)
    assert calls and calls[0] == ((1280, 720, 60), False)


def test_a_query_string_does_not_hide_a_route(instrument):
    """Routing is on the PATH, not the path plus its query.

    The panel appends a cache-buster to the preview image on purpose --
    `/bridge/preview.png?t=...` -- and every exact-match route compared
    the whole of self.path, so that request 404'd every single time and
    the panel showed a broken-image icon where the sensor preview should
    be. It had never worked. Only /preview.png survived, by the accident
    of needing startswith for its ?recording= form.
    """
    _, port = instrument
    for path in ("/status", "/recordings", "/modules"):
        bare, _ = _request(port, "GET", path)
        with_query, _ = _request(port, "GET", path + "?t=123456")
        assert bare == with_query, (
            f"{path} answered {bare} bare but {with_query} with a query "
            "string; a cache-buster must not hide a route")


def test_the_firmware_is_brought_up_at_what_the_bridge_streams():
    """The init line carries the sensor's frame, a TV raster on the link's
    timing, the picture centred, the 3A settings -- and no rate."""
    from picam2hdmi.serve import firmware_init
    line = firmware_init({"width": "640", "height": "480", "bits": "10",
                          "order": "GBRG", "link_khz": "74250",
                          "h_total": "2358", "v_total": "506",
                          "exposure_max": "496"}, {"af": 1})
    got = dict(kv.split("=") for kv in line[len("init "):].split())
    assert got["frame.width"] == "640" and got["frame.height"] == "480"
    assert got["frame.bayer_phase"] == "2"
    assert not any(k.startswith("frame.") and k not in (
        "frame.width", "frame.height", "frame.bits", "frame.bayer_phase")
        for k in got), "the frame carries no timing"
    assert (got["display.h_total"], got["display.v_total"]) == ("2358", "506")
    assert got["window.center"] == "1"
    assert (got["aaa.ae"], got["aaa.awb"], got["aaa.af"]) == ("1", "1", "1")
    assert got["aaa.exposure_max"] == "496"


MODES = ("ok cropped_1080p=1920x1080x10,2,87500000,2416,1104 "
         "binned_640x480=640x480x10,2,58333000,1852,504")


def test_the_firmwares_modes_read_as_the_sensor_module_states_them():
    from picam2hdmi.serve import parse_modes
    modes = parse_modes(MODES)
    assert [m["name"] for m in modes] == ["cropped_1080p", "binned_640x480"]
    assert modes[1] == {"name": "binned_640x480", "width": 640,
                        "height": 480, "bits": 10, "bayer_phase": 2,
                        "pixel_rate_hz": 58333000, "line_length_pck": 1852,
                        "frame_length_lines": 504}
    assert parse_modes("ok") == [] and parse_modes("err no module") == []


def test_the_firmware_is_told_the_streamed_mode_by_name():
    """Where the sensor module has a mode of what the bridge streams, the
    init names it and the frame comes with it; where it has none, the
    frame is stated itself."""
    from picam2hdmi.serve import firmware_init, parse_modes
    status = {"width": "640", "height": "480", "bits": "10",
              "order": "GBRG", "link_khz": "74250", "h_total": "2358",
              "v_total": "506"}
    named = dict(kv.split("=") for kv in firmware_init(
        status, modes=parse_modes(MODES))[len("init "):].split())
    assert named["mode"] == "binned_640x480"
    assert not any(k.startswith("frame.") for k in named)
    plain = dict(kv.split("=") for kv in firmware_init(
        {**status, "bits": "12"},
        modes=parse_modes(MODES))[len("init "):].split())
    assert "mode" not in plain and plain["frame.bits"] == "12"


def test_the_tv_raster_runs_on_the_links_line_and_frame_times():
    """A TV line lasts as long as a link line and a TV frame as long as a
    link frame, whatever the TV's clock; its active area is the largest
    familiar one that fits, independent of the input's size."""
    from picam2hdmi.serve import display_raster
    # the 640x480 mode at its own 62 fps: a 506-line frame holds 480 lines
    vga = display_raster(640, 480, 74250, 2358, 506)
    assert (vga["h_total"], vga["v_total"]) == (2358, 506)
    assert (vga["h_active"], vga["v_active"]) == (720, 480)
    # a crop smaller than the frame changes the picture, not the raster
    assert display_raster(320, 240, 74250, 2358, 506)["h_active"] == 720
    # a TV clock twice the link's: the same times, twice the pixels in a
    # line -- the area is still bounded by the frame's 506 lines
    fast = display_raster(640, 480, 74250, 2358, 506, tv_khz=148500)
    assert fast["h_total"] == 4716 and fast["v_total"] == 506
    assert fast["v_active"] <= 506 - 6
    for r in (vga, fast):
        assert r["h_active"] < r["hs_beg"] < r["hs_end"] < r["h_total"]
        assert r["v_active"] < r["vs_beg"] < r["vs_end"] < r["v_total"]
    # the 640x480 mode on 480p's frame at the 480p clock: within a hair of
    # 480p itself, the TV gets 480p whole -- its totals, syncs, polarities
    p = display_raster(640, 480, 74250, 2364, 524, tv_khz=27000)
    assert (p["h_total"], p["v_total"], p["h_active"], p["hsync_positive"]) \
        == (858, 525, 720, 0)
    # 1080p at the sensor's 32.7 fps: the frame holds the whole area
    hd = display_raster(1920, 1080, 74250, 2052, 1106)
    assert (hd["h_active"], hd["v_active"], hd["h_total"], hd["v_total"]) \
        == (1920, 1080, 2052, 1106)

def test_the_keeper_brings_the_firmware_up_once_per_bridge_run(
        tmp_path, monkeypatch):
    """A bridge that restarted -- a module installed, a reboot -- leaves
    the firmware waiting: the keeper brings it up, once for that run, and
    leaves a firmware that has proved its build alone."""
    from picam2hdmi import serve
    sup = Supervisor(tmp_path, default_mode="1920x1080@30")
    sup._persist_intent({"source": "camera", "aaa": {"af": 1}})
    state = {"uptime": 3, "fw": "ok build=unproven frame=0x0x0"}
    calls = []
    monkeypatch.setattr(serve, "_bridge_owns_bench", lambda: True)
    monkeypatch.setattr(serve, "_bridge_status",
                        lambda: {"uptime": str(state["uptime"])})
    monkeypatch.setattr(serve, "_bridge_command", lambda line, **k: state["fw"])
    monkeypatch.setattr(serve, "_firmware_up",
                        lambda aaa=None: calls.append(aaa) or "ok")
    clock = {"now": 1_000.0}
    monkeypatch.setattr(serve.time, "time", lambda: clock["now"])
    assert sup.firmware_once()
    clock["now"], state["uptime"] = 1_005.0, 8   # the same run, a beat on
    assert not sup.firmware_once()
    assert calls == [{"af": 1}]
    clock["now"], state["uptime"] = 1_010.0, 1   # the bridge restarted
    assert sup.firmware_once()
    state["fw"] = "ok build=proven frame=1920x1080x12"
    clock["now"], state["uptime"] = 2_000.0, 1   # and again, but it is up
    assert not sup.firmware_once()
    assert len(calls) == 2


def test_the_panel_and_the_keeper_do_not_both_bring_the_firmware_up(
        tmp_path, monkeypatch):
    """The panel's camera start brings the firmware up; a keeper beat that
    then still reads it unproven -- the init was in flight -- leaves the
    run alone."""
    from picam2hdmi import serve
    sup = Supervisor(tmp_path, default_mode="1920x1080@30")
    sup._persist_intent({"source": "camera"})
    calls = []
    monkeypatch.setattr(serve, "_bridge_owns_bench", lambda: True)
    monkeypatch.setattr(serve, "_bridge_status", lambda: {"uptime": "4"})
    monkeypatch.setattr(serve, "_bridge_command",
                        lambda line, **k: "ok build=unproven")
    monkeypatch.setattr(serve, "_firmware_up",
                        lambda aaa=None: calls.append(aaa) or "ok")
    monkeypatch.setattr(serve.time, "time", lambda: 500.0)
    assert sup.firmware_now() == "ok"
    assert not sup.firmware_once()
    assert len(calls) == 1


def test_a_480_line_picture_goes_out_at_480ps_clock():
    """480 lines go out at 27 MHz where the board makes it -- the only form
    a TV takes them in -- anything taller at the slowest clock fast enough
    for its area at the link's line time."""
    from picam2hdmi.serve import display_clock
    rows = ["27000", "74250", "148500"]
    assert display_clock(rows, 640, 480) == 27000
    # 1080p at the sensor's 32.7 fps: a 2052-pixel link line holds 1920
    # active and blanking at 74.25 MHz
    assert display_clock(rows, 1920, 1080, 74250, 2052) == 74250
    # a 1080p sensor at 60 fps: a 1100-pixel link line at 74.25 MHz holds
    # 1920 only at twice the clock
    assert display_clock(rows, 1920, 1080, 74250, 1100) == 148500
    assert display_clock(["74250"], 640, 480) == 74250


CLOCKS = [27000, 74250, 148500]


def _link(mode, timing, link_khz=74250):
    """What the bridge makes of the options: the sensor line it reads back
    from --line-ns, the link line rounded up to an even count, and
    --frame-lines as the link frame."""
    import math
    line = int(timing["line_ns"] * 1e-9 * mode["pixel_rate_hz"])
    ht = math.ceil(line / mode["pixel_rate_hz"] * link_khz * 1000.0)
    return line, ht + (ht & 1), timing["frame_lines"]


def test_each_mode_is_timed_onto_a_raster_the_tv_knows():
    """640x480 lands on 480p60 -- the one 480-line raster the TV takes --
    and the 1080 crop on 1080p30, each by sensor line and frame lengths
    the init then turns into exactly that raster."""
    from bayerlink.rasters import RASTERS
    from picam2hdmi.serve import (display_clock, display_raster,
                                  parse_modes, sensor_timing)
    for mode, want, line, frame in (
            ("binned_640x480", "720x480p60", 1856, 524),
            ("cropped_1080p", "1920x1080p30", 2591, 1125)):
        m = next(x for x in parse_modes(MODES) if x["name"] == mode)
        timing = sensor_timing(m, CLOCKS, 74250, 2)
        assert timing["raster"] == want
        got_line, ht, v = _link(m, timing)
        assert (got_line, v) == (line, frame)
        clock = display_clock(CLOCKS, m["width"], m["height"], 74250, ht)
        tv = display_raster(m["width"], m["height"], 74250, ht, v, clock)
        r = RASTERS[want]
        assert (clock, tv["h_total"], tv["v_total"]) == (
            r.pixel_khz, r.h_total, r.v_total)
        # and the link frame lasts the raster's to within what the genlock
        # follows: 2.1e-4 slipped until it jammed, and the TV blinked
        from picam2hdmi.serve import GENLOCK_EASE
        link_frame = ht * v / 74250e3
        tv_frame = r.h_total * r.v_total / (r.pixel_khz * 1e3)
        assert abs(link_frame - tv_frame) / tv_frame <= GENLOCK_EASE


def test_a_mode_no_raster_can_take_runs_at_its_own_timing():
    """A mode whose shortest frame is longer than every raster that holds
    it gets no timing: nothing is forced."""
    from picam2hdmi.serve import sensor_timing
    slow = {"name": "x", "width": 640, "height": 480, "bits": 10,
            "bayer_phase": 2, "pixel_rate_hz": 58333000,
            "line_length_pck": 1852, "frame_length_lines": 4000}
    assert sensor_timing(slow, CLOCKS, 74250, 2) is None
