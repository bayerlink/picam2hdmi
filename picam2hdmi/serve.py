"""The source as a bench instrument: a control daemon over HTTP.

Lab instruments have always had control ports; HTTP + JSON is today's
SCPI. This daemon owns the scanout and listens for three things:

    GET  /                    the panel: a phone-friendly control page
    GET  /status              what is streaming, and how it is going
    PUT  /source              switch source: pattern, a recording, or off
    GET  /preview.png         what is being EMITTED, as an image -- add
                              ?recording=<name> to look inside the spool
    GET  /recordings          list the spool
    PUT  /recordings/<name>   upload a recording (.npy of containers)
    GET  /recordings/<name>   download it back, byte-identical
    DELETE /recordings/<name> remove it

With that, a test rig ORCHESTRATES the physical bench: set a pattern,
capture on the receiving side, judge with bayertap, switch format,
repeat -- hardware in the loop, unattended.

This is a LAN bench instrument, not an internet service. It runs with
the privileges DRM master needs, binds where you tell it, and writes
only into its spool directory; put it behind a firewall, not behind a
login page it does not have. Refusals travel as HTTP 400 with the same
messages the CLI prints -- a bad geometry is named, not guessed at.
"""
from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


class _Stop(Exception):
    """Raised inside the scanout loop when the supervisor wants it back."""


def _png_gray(img) -> bytes:
    """A grayscale-8 PNG from a 2-D uint8 array, stdlib only.

    A PNG is four chunks and a zlib stream; an instrument page needs
    nothing more, and pulling in an imaging library for previews would
    end the numpy-plus-nothing install story."""
    import struct
    import zlib

    height, width = img.shape
    def chunk(tag, data):
        body = tag + data
        return (struct.pack(">I", len(data)) + body
                + struct.pack(">I", zlib.crc32(body)))
    header = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    rows = b"".join(b"\x00" + bytes(img[r]) for r in range(height))
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header)
            + chunk(b"IDAT", zlib.compress(rows, 6)) + chunk(b"IEND", b""))


def _container_preview(container) -> bytes:
    """Decode a container and render its raw samples as a grayscale PNG."""
    import numpy as np
    from bayerlink import decode_frame

    header, raw = decode_frame(np.asarray(container))
    if header.bits > 8:
        shown = (raw >> (header.bits - 8)).astype(np.uint8)
    else:
        shown = raw.astype(np.uint8)
    return _png_gray(shown)


def _parse_mode(text: str) -> tuple[int, int, int]:
    size, _, hz = text.partition("@")
    width, _, height = size.partition("x")
    return int(width), int(height), int(hz or 60)


class Supervisor:
    """Owns the one stream thread and the facts about it."""

    def __init__(self, spool: Path, default_mode: str,
                 connector=None, card_path=None, runner=None):
        self.spool = Path(spool)
        self.spool.mkdir(parents=True, exist_ok=True)
        self.default_mode = default_mode
        self.connector = connector
        self.card_path = card_path
        if runner is None:
            from . import output
            runner = output.stream
        self._runner = runner
        self._lock = threading.Lock()
        self._thread = None
        self._stop = threading.Event()
        self._frames = 0
        self._started = None
        self.spec = {"source": "off"}
        self.last_error = None
        self.preview_container = None

    # -- facts ---------------------------------------------------------------

    def status(self) -> dict:
        alive = self._thread is not None and self._thread.is_alive()
        return {
            "source": self.spec if alive else {"source": "off"},
            "running": alive,
            "frames": self._frames,
            "uptime_s": round(time.monotonic() - self._started, 1)
            if alive and self._started else 0,
            "last_error": self.last_error,
            "spool": sorted(p.name for p in self.spool.glob("*.npy")),
        }

    # -- control ---------------------------------------------------------------

    def start(self, spec: dict) -> None:
        """Validate the spec, stop the old stream, start the new one.

        Validation happens HERE, in the caller's thread, so a bad request
        is a 400 with the reason -- never a daemon that died in the dark.
        """
        from . import output

        source = spec.get("source")
        if source == "off":
            self.stop()
            self.spec = {"source": "off"}
            self.preview_container = None
            return
        mode_text = spec.get("mode", self.default_mode)
        mode = _parse_mode(mode_text)
        display = (mode[0], mode[1])
        tunnel = bool(spec.get("luma_tunnel", False))
        if source == "pattern":
            frames = output.pattern_frames(
                spec.get("pattern", "counting"),
                int(spec.get("width", 512)), int(spec.get("height", 240)),
                spec.get("bayer", "RGGB"), display, luma_tunnel=tunnel)
            # Prove the spec by building its first frame NOW; the
            # generator defers everything, and a refusal belongs in the
            # HTTP response, not in the thread's obituary.
            first = next(frames)
            frames = _chain(first, frames)
            preview = first
        elif source == "file":
            name = str(spec.get("file", ""))
            path = self.spool / name
            if path.name != name or not name.endswith(".npy"):
                raise ValueError(f"recording name {name!r} is not a plain "
                                 "<name>.npy in the spool")
            if not path.exists():
                raise ValueError(f"no recording {name!r}; the spool has "
                                 f"{self.status()['spool']}")
            frames = output.file_frames(str(path), display,
                                        luma_tunnel=tunnel,
                                        restamp=bool(spec.get("restamp",
                                                              True)))
            first = next(frames)
            frames = _chain(first, frames)
            import numpy as np
            stack = np.load(path, mmap_mode="r")
            preview = np.array(stack[0] if stack.ndim == 4 else stack)
        elif source == "camera":
            from . import capture

            cam_mode = spec.get("cam_mode")
            crop = spec.get("crop")
            frames = capture.frames(
                display,
                mode=tuple(int(v) for v in cam_mode) if cam_mode else None,
                crop=tuple(int(v) for v in crop) if crop else None,
                exposure_us=spec.get("exposure_us"),
                analogue_gain=spec.get("gain"),
                luma_tunnel=tunnel)
            first = next(frames)
            preview = first
            # A camera moves; its preview should too. Stash every 30th
            # container so the panel's poll shows a moving image.
            supervisor = self

            def _peeking(source_frames):
                for index, frame in enumerate(source_frames):
                    if index % 30 == 0 and not tunnel:
                        supervisor.preview_container = frame
                    yield frame
            frames = _peeking(_chain(first, frames))
        else:
            raise ValueError(f"source {source!r} is not one of "
                             "'pattern', 'file', 'camera', 'off'")

        # For a tunnel source the scanout frame is the grey wrap; the
        # preview shows the CONTENT -- the inner container -- because an
        # instrument answers "what am I emitting", not "what shade of grey".
        if tunnel and source == "pattern":
            from bayerlink import tunnel as _tunnel
            inner = _tunnel.inner_display(*display)
            preview_frames = output.pattern_frames(
                spec.get("pattern", "counting"),
                int(spec.get("width", 512)), int(spec.get("height", 240)),
                spec.get("bayer", "RGGB"), inner)
            preview = next(preview_frames)

        self.stop()
        self.preview_container = preview
        self.spec = dict(spec, mode=mode_text)
        self.last_error = None
        self._stop.clear()
        self._frames = 0
        self._started = time.monotonic()
        self._thread = threading.Thread(
            target=self._run, args=(frames, mode), daemon=True)
        self._thread.start()

    def stop(self) -> None:
        with self._lock:
            thread, self._thread = self._thread, None
        if thread is not None and thread.is_alive():
            self._stop.set()
            thread.join(timeout=5)

    def _run(self, frames, mode) -> None:
        def on_frame(index):
            self._frames = index
            if self._stop.is_set():
                raise _Stop
        try:
            self._runner(frames, mode=mode, connector=self.connector,
                         card_path=self.card_path, on_frame=on_frame)
        except _Stop:
            pass
        except Exception as error:            # noqa: BLE001 -- reported, not eaten
            self.last_error = f"{type(error).__name__}: {error}"


def _chain(first, rest):
    yield first
    yield from rest


# ----------------------------------------------------------------------------- #
# HTTP surface
# ----------------------------------------------------------------------------- #

def _handler(supervisor: Supervisor):
    class Handler(BaseHTTPRequestHandler):
        server_version = "picam2hdmi"

        def _send(self, code: int, payload: dict) -> None:
            body = json.dumps(payload, indent=2).encode() + b"\n"
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, fmt, *args):    # quiet by default; status IS the log
            pass

        def _send_bytes(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _spool_name(self, name: str):
            if (Path(name).name != name or name.startswith(".")
                    or not name.endswith(".npy")):
                return None
            return supervisor.spool / name

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                page = (Path(__file__).parent / "ui.html").read_bytes()
                return self._send_bytes(200, page, "text/html; charset=utf-8")
            if self.path == "/status":
                return self._send(200, supervisor.status())
            if self.path == "/recordings":
                return self._send(200, {"recordings":
                                        supervisor.status()["spool"]})
            if self.path.startswith("/preview.png"):
                _, _, query = self.path.partition("?")
                container = supervisor.preview_container
                if query.startswith("recording="):
                    import numpy as np
                    path = self._spool_name(query[len("recording="):])
                    if path is None or not path.exists():
                        return self._send(404, {"error": "no such recording"})
                    stack = np.load(path, mmap_mode="r")
                    container = np.array(stack[0] if stack.ndim == 4
                                         else stack)
                if container is None:
                    return self._send(404, {"error": "nothing streaming"})
                try:
                    return self._send_bytes(200,
                                            _container_preview(container),
                                            "image/png")
                except ValueError as error:
                    return self._send(400, {"error": str(error)})
            if self.path.startswith("/recordings/"):
                path = self._spool_name(self.path[len("/recordings/"):])
                if path is None:
                    return self._send(400, {"error": "recording name must "
                                            "be a plain <name>.npy"})
                if not path.exists():
                    return self._send(404, {"error": "no such recording"})
                return self._send_bytes(200, path.read_bytes(),
                                        "application/octet-stream")
            return self._send(404, {"error": f"no route {self.path!r}"})

        def do_DELETE(self):
            if self.path.startswith("/recordings/"):
                path = self._spool_name(self.path[len("/recordings/"):])
                if path is None:
                    return self._send(400, {"error": "recording name must "
                                            "be a plain <name>.npy"})
                if not path.exists():
                    return self._send(404, {"error": "no such recording"})
                path.unlink()
                return self._send(200, {"deleted": path.name})
            return self._send(404, {"error": f"no route {self.path!r}"})

        def do_PUT(self):
            length = int(self.headers.get("Content-Length", 0))
            if self.path == "/source":
                try:
                    spec = json.loads(self.rfile.read(length) or b"{}")
                    supervisor.start(spec)
                except Exception as error:   # noqa: BLE001 -- named, not dropped
                    # Whatever went wrong belongs in the response: an
                    # instrument that answers with a closed connection
                    # turns a missing dependency into a mystery.
                    return self._send(400, {
                        "error": f"{type(error).__name__}: {error}"})
                return self._send(200, supervisor.status())
            if self.path.startswith("/recordings/"):
                name = self.path[len("/recordings/"):]
                path = self._spool_name(name)
                if path is None:
                    return self._send(400, {
                        "error": f"recording name {name!r} must be a plain "
                                 "<name>.npy"})
                data = self.rfile.read(length)
                path.write_bytes(data)
                return self._send(200, {"saved": name, "bytes": len(data)})
            return self._send(404, {"error": f"no route {self.path!r}"})

    return Handler


def serve(bind: str, port: int, supervisor: Supervisor,
          autostart: dict | None = None) -> None:
    if autostart:
        try:
            supervisor.start(autostart)
        except Exception as error:            # noqa: BLE001
            supervisor.last_error = f"autostart: {error}"
    server = ThreadingHTTPServer((bind, port), _handler(supervisor))
    print(f"picam2hdmi instrument on http://{bind}:{port} "
          f"(spool: {supervisor.spool})")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        supervisor.stop()
