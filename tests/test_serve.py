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


def test_sensor_png_is_the_samples_top_bits():
    # The full-view preview never unpacks: at CSI-2 packing the first
    # group_samples bytes of a group ARE the top 8 bits. Pin that against
    # the real packer, through a real PNG decode.
    import zlib

    from bayerlink.protocol import pack_samples
    from picam2hdmi.serve import _sensor_png

    rng = np.random.default_rng(7)
    samples = rng.integers(0, 1024, size=(6, 16), dtype=np.uint16)
    rows = np.stack([np.frombuffer(pack_samples(line, 10), np.uint8)
                     for line in samples])
    png = _sensor_png(rows, bits=10, step=1)
    assert png[:8] == b"\x89PNG\r\n\x1a\n"

    idat = png.index(b"IDAT")
    length = int.from_bytes(png[idat - 4:idat], "big")
    raster = zlib.decompress(png[idat + 4:idat + 4 + length])
    shown = np.frombuffer(raster, np.uint8).reshape(6, 17)[:, 1:]  # drop filters
    assert np.array_equal(shown, (samples >> 2).astype(np.uint8))


def test_sensor_preview_route_and_lifecycle(instrument):
    supervisor, port = instrument

    # No camera has streamed: the full view honestly does not exist.
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    conn.request("GET", "/preview.png?sensor=1")
    assert conn.getresponse().status == 404

    supervisor.sensor_view = (np.zeros((4, 20), np.uint8), 10)
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    conn.request("GET", "/preview.png?sensor=1")
    response = conn.getresponse()
    assert response.status == 200 and response.read()[:4] == b"\x89PNG"

    # Any non-camera source clears it -- a stale full view aims nothing.
    supervisor.start({"source": "pattern", "width": 16, "height": 4})
    assert supervisor.sensor_view is None
    supervisor.stop()
