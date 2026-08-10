"""The source as a bench instrument: a control daemon over HTTP.

Lab instruments have always had control ports; HTTP + JSON is today's
SCPI. This daemon owns the scanout and listens for three things:

    GET  /status              what is streaming, and how it is going
    PUT  /source              switch source: pattern, a recording, or off
    GET  /recordings          list the spool
    PUT  /recordings/<name>   upload a recording (.npy of containers)

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
        else:
            raise ValueError(f"source {source!r} is not one of "
                             "'pattern', 'file', 'off'")

        self.stop()
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

        def do_GET(self):
            if self.path == "/status":
                return self._send(200, supervisor.status())
            if self.path == "/recordings":
                return self._send(200, {"recordings":
                                        supervisor.status()["spool"]})
            return self._send(404, {"error": f"no route {self.path!r}"})

        def do_PUT(self):
            length = int(self.headers.get("Content-Length", 0))
            if self.path == "/source":
                try:
                    spec = json.loads(self.rfile.read(length) or b"{}")
                    supervisor.start(spec)
                except (ValueError, KeyError) as error:
                    return self._send(400, {"error": str(error)})
                return self._send(200, supervisor.status())
            if self.path.startswith("/recordings/"):
                name = self.path[len("/recordings/"):]
                if (Path(name).name != name or name.startswith(".")
                        or not name.endswith(".npy")):
                    return self._send(400, {
                        "error": f"recording name {name!r} must be a plain "
                                 "<name>.npy"})
                data = self.rfile.read(length)
                (supervisor.spool / name).write_bytes(data)
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
