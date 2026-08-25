# Copyright 2026 Serge Rabyking
# SPDX-License-Identifier: MIT
"""The source as a bench instrument: a control daemon over HTTP.

Lab instruments have always had control ports; HTTP + JSON is today's
SCPI. This daemon owns the scanout and listens for three things:

    GET  /                    the panel: a phone-friendly control page
    GET  /status              what is streaming, and how it is going
    PUT  /source              switch source: pattern, a recording, or off
    GET  /preview.png         what is being EMITTED, as an image;
                              ?recording=<name> to look inside the spool
    GET  /recordings          list the spool
    PUT  /recordings/<name>   upload a recording (.npy of containers)
    GET  /recordings/<name>   download it back, byte-identical
    DELETE /recordings/<name> remove it
    GET  /modules             the two module slots' manifests
    PUT  /modules/<slot>      upload a package (tar.gz): isp or sensor;
                              validated, compiled in, bridge restarted
    DELETE /modules/<slot>    remove it and rebuild plain
    PUT  /power               {"off": true} -> clean shutdown; instruments
                              have power buttons, and SD cards prefer them

Like a bench instrument, it RETAINS STATE: every accepted source spec
is persisted in the spool, and (unless --no-autostart) the daemon boots
straight back into the last one -- configure the camera once in the
panel and the Pi is a camera with a power plug from then on.

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


# What the power button runs. A tuple so tests can point it at something
# harmless without touching the handler.
POWEROFF = ("systemctl", "poweroff")


def _png_gray(img, level: int = 6) -> bytes:
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
            + chunk(b"IDAT", zlib.compress(rows, level)) + chunk(b"IEND", b""))


def _container_preview(container, level: int = 6) -> bytes:
    """Decode a container and render its raw samples as a grayscale PNG."""
    import numpy as np
    from bayerlink import decode_frame

    header, raw = decode_frame(np.asarray(container))
    if header.bits > 8:
        shown = (raw >> (header.bits - 8)).astype(np.uint8)
    else:
        shown = raw.astype(np.uint8)
    return _png_gray(shown, level)


def _panel_hash() -> str:
    import hashlib

    return hashlib.md5(
        (Path(__file__).parent / "ui.html").read_bytes()).hexdigest()[:12]


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
        self.derived = None            # the container the rule chose, if any
        self.last_error = None
        self.preview_container = None
        self._camera_ref = None
        self._start_lock = threading.Lock()
        # True while the source runs WITHOUT a display: framing,
        # focusing, setting exposure -- everything but the wire.
        self.monitoring = False

    # -- facts ---------------------------------------------------------------

    def status(self) -> dict:
        alive = self._thread is not None and self._thread.is_alive()
        return {
            "source": self.spec if alive else {"source": "off"},
            # What the instrument INTENDS to stream -- differs from
            # `source` only while idle (a dark display, a camera still
            # enumerating). The panel mirrors this, never its own copy.
            "intent": self.stored_spec() or {"source": "off"},
            "running": alive,
            "derived_container": self.derived,
            # Whether the WIRE is being driven: False while idle, and
            # False in monitor mode -- the panel says so instead of
            # letting a live viewfinder imply a fed receiver.
            "display": alive and not self.monitoring,
            "panel": _panel_hash(),
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
        Serialised: the keeper thread and HTTP handlers may both call in.
        """
        with self._start_lock:
            return self._start(spec)

    def _start(self, spec: dict) -> None:
        from . import output

        source = spec.get("source")
        if source == "off":
            self.stop()
            self.spec = {"source": "off"}
            self.preview_container = None
            self._persist()
            return
        from .kms import DisplayNotReady

        # An explicit container overrides mode: a private link's frame,
        # not a display's. Absent both, the container is DERIVED --
        # content plus the header line, on the first proven raster that
        # holds it -- so nobody ever types the +1. `mode` remains the
        # explicit way to ask for a standard display mode.
        container_text = spec.get("container") or self._derived_container(spec)
        mode_text = container_text or spec.get("mode", self.default_mode)
        self.derived = None if spec.get("container") or spec.get("mode") \
            else mode_text
        try:
            return self._start_source(spec, mode_text,
                                      container=bool(container_text))
        except DisplayNotReady:
            # The display being dark is a STATE, not a verdict on the
            # request: on a bench, cables come and go. The choice becomes
            # the retained intent -- the keeper realises it the moment a
            # display appears -- and the refusal still goes back to the
            # caller so the panel says why nothing is on the wire yet.
            self._persist_intent(dict(spec))
            raise

    def _derived_container(self, spec: dict) -> str | None:
        """content+1 on a proven raster, when the spec states its geometry.

        A pattern states width and height; a camera's stream is its
        crop. Full-sensor cameras (no crop) keep the explicit paths:
        their geometry is only known after the sensor opens.
        """
        if spec.get("mode"):
            return None                    # an explicit ask wins
        source = spec.get("source")
        if source == "pattern":
            shape = (int(spec.get("width", 512)),
                     int(spec.get("height", 240)))
        elif source == "camera" and spec.get("crop"):
            shape = (int(spec["crop"][2]), int(spec["crop"][3]))
        elif source == "file" and spec.get("file"):
            # A recording IS a container -- header line included -- so
            # it replays in exactly its own geometry, no +1, no defaults.
            import numpy as np
            path = self.spool / str(spec["file"])
            if not path.exists():
                return None                # the file path refuses later
            shape = np.load(path, mmap_mode="r").shape
            # one container (H, W, 3) or a sequence (N, H, W, 3)
            height, width = shape[1:3] if len(shape) == 4 else shape[:2]
            from . import kms
            for hz in (60, 30):
                if kms.container_fits(width, height, hz):
                    return f"{width}x{height}@{hz}"
            return None
        else:
            return None
        from . import kms
        width, lines = shape[0], shape[1] + 1
        for hz in (60, 30):
            if kms.container_fits(width, lines, hz):
                return f"{width}x{lines}@{hz}"
        return None                        # let the explicit paths refuse

    def sensor_modes(self) -> list:
        """Sizes the STREAMING sensor declares, or an empty list.

        Only the open handle can answer this -- nobody else may open the
        camera while it streams -- so the daemon publishes what it sees
        rather than letting a panel keep its own idea of what exists.
        """
        return list((self._camera_ref or {}).get("modes") or [])

    def camera_index(self) -> int | None:
        """Which camera the modes above belong to."""
        if not self._camera_ref:
            return None
        return int((self.spec or {}).get("index", 0))

    def _live_controls(self, spec: dict) -> None:
        """Retune the running camera's exposure and gain, live.

        Mid-stream controls land a few frames later -- the interactive
        contract. The deterministic first-frame contract still exists:
        it is called restarting the source, and scripts that want it
        change a non-live key."""
        ref = self._camera_ref or {}
        picam2 = ref.get("picam2")
        # Nothing to push: exposure and gain are the sensor package's,
        # so a live retune here is the WINDOW only. A panel that could
        # write them would be a second owner of the AE loop's values.
        controls = {}
        if picam2 is not None and controls:
            picam2.set_controls(controls)
        self.spec = dict(spec)
        self._persist()

    def _start_source(self, spec: dict, mode_text: str,
                      container: bool = False) -> None:
        from . import output

        source = spec.get("source")
        mode = _parse_mode(mode_text)
        display = (mode[0], mode[1])
        crop_ref = None                   # camera only; filled on startup
        if source == "pattern":
            frames = output.pattern_frames(
                spec.get("pattern", "counting"),
                int(spec.get("width", 512)), int(spec.get("height", 240)),
                spec.get("bayer", "RGGB"), display)
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
                                        restamp=bool(spec.get("restamp",
                                                              True)))
            first = next(frames)
            frames = _chain(first, frames)
            import numpy as np
            stack = np.load(path, mmap_mode="r")
            preview = np.array(stack[0] if stack.ndim == 4 else stack)
        elif source == "camera":
            from . import capture

            # A crop is a byte slice, not a camera setting: if the SAME
            # camera is already streaming and only the window moved,
            # retarget it live -- no restart, no dropped frames.
            if self._crop_only_change(spec):
                self._live_crop(spec, mode_text)
                return self._live_controls(spec)

            # The camera is exclusive: a second open while the old stream
            # holds it cannot even initialise, so this one source stops
            # the world FIRST and validates after -- a bad camera spec
            # costs the running stream, and the 400 still names why.
            self.stop()
            cam_mode = spec.get("cam_mode")
            crop = spec.get("crop")
            supervisor = self

            # A camera moves; its preview should too. The capture path
            # peeks every 5th container, so the panel's poll shows the
            # moving window.
            def _stash(container):
                supervisor.preview_container = container

            crop_ref = {}
            frames = capture.frames(
                display,
                mode=tuple(int(v) for v in cam_mode) if cam_mode else None,
                crop=tuple(int(v) for v in crop) if crop else None,
                peek=_stash, crop_ref=crop_ref,
                source_id=spec.get("source_id"))
            first = next(frames)          # frame 0 peeked during this pull
            preview = self.preview_container
            frames = _chain(first, frames)
        else:
            raise ValueError(f"source {source!r} is not one of "
                             "'pattern', 'file', 'camera', 'off'")

        self.stop()
        self.preview_container = preview
        self._camera_ref = crop_ref
        self.spec = dict(spec)
        self.last_error = None
        self._stop.clear()
        self._frames = 0
        self._started = time.monotonic()
        self._frames_gen = frames
        self._thread = threading.Thread(
            target=self._run,
            args=(frames, mode, source != "camera", container),
            daemon=True)
        self._thread.start()
        self._persist()

    def revive_once(self) -> bool:
        """One keeper beat: if the instrument is idle but its retained
        source says it should be streaming, try to make it so.

        Cables come and go on a bench; the INTENT is persisted, so the
        daemon acts on it whenever reality catches up -- a capture stick
        plugged in minutes after boot, a display re-attached mid-day, a
        camera that finished enumerating. An explicit 'off' is intent
        too, and is left alone.
        """
        desired = self.stored_spec()
        if not desired or desired.get("source", "off") == "off":
            return False
        # While a bridge runs it owns the bench -- the sensor AND the
        # display -- and this daemon is a controller, never an engine.
        # Reviving an intent here would reach for hardware the bridge
        # holds: the camera, whose frames it would steal, or the
        # display, which answers EACCES. Both were happening on a five
        # second timer, and the second one kept re-creating the very
        # PermissionError the panel was showing, so a working bench
        # read as a broken one no matter what was clicked.
        if _bridge_owns_bench():
            return False
        if self._thread is not None and self._thread.is_alive():
            return False
        try:
            # The bridge first, exactly as the HTTP route would choose:
            # an intent realised here must reach the same engine as the
            # same intent clicked in the panel, or this daemon takes a
            # display the bridge is about to need.
            drawn = route_to_bridge(desired, self.spool)
            if drawn:
                self.stop()
                self.last_error = ""
                return True
            self.start(desired)
            return True
        except Exception as error:            # noqa: BLE001 -- kept as status
            self.last_error = f"waiting to start: {error}"
            return False

    def keep(self, interval: float = 5.0) -> None:
        """Run revive_once forever, gently, in the background."""
        def beat():
            while True:
                time.sleep(interval)
                self.revive_once()
        threading.Thread(target=beat, daemon=True).start()

    def capture(self) -> str:
        """Freeze the frame being emitted into a sequential spool file.

        What is captured is the CONTAINER -- header, window, everything
        the wire carries -- so a capture replays exactly like a
        recording, because it is one. The stream is not touched: this
        reads the peek copy the runner already keeps for previews.
        """
        import numpy as np

        container = self.preview_container
        if container is None:
            raise ValueError("nothing is streaming, so there is no frame "
                             "to capture")
        number = 1
        while (self.spool / f"capture-{number:03d}.npy").exists():
            number += 1
        path = self.spool / f"capture-{number:03d}.npy"
        np.save(path, np.asarray(container))
        return path.name

    def _persist(self) -> None:
        """State retention, best-effort: streaming is the job, not this."""
        self._persist_intent(self.spec)

    def _persist_intent(self, spec: dict) -> None:
        """Record what the instrument should be doing, which is not always
        what it IS doing: a source chosen while the display is dark is
        intent too, and the keeper realises it when the bench allows."""
        try:
            (self.spool / "last-source.json").write_text(json.dumps(spec))
        except OSError:
            pass

    def stored_spec(self) -> dict | None:
        """The spec the instrument was last set to, if one survives."""
        try:
            spec = json.loads((self.spool / "last-source.json").read_text())
        except (OSError, ValueError):
            return None
        return spec if isinstance(spec, dict) and spec.get("source") else None

    def _crop_only_change(self, spec: dict) -> bool:
        """True when a camera streams and only LIVE-TUNABLE facts differ.

        The window, the exposure and the gain are runtime facts: they
        retune the running stream and never touch the world -- a gain
        slider must have NO PATH to a display-mode negotiation."""
        alive = self._thread is not None and self._thread.is_alive()
        if not (alive and self.spec.get("source") == "camera"
                and getattr(self, "_camera_ref", None)):
            return False
        old = self.spec

        def norm(v):
            return [int(n) for n in v] if v else None
        return (spec.get("container") == old.get("container")
                and spec.get("mode") == old.get("mode")
                and norm(spec.get("cam_mode")) == norm(old.get("cam_mode"))
                and spec.get("flags") == old.get("flags"))

    def _live_crop(self, spec: dict, mode_text: str) -> None:
        """Retarget the running camera's window; refusals stay identical.

        Validation is the same code the stream runs: the geometry rules
        via validate_crop/check_rate, and the display-fit rule by
        encoding a zero-filled window -- so a window accepted HERE
        cannot kill the stream THERE, and a refused one carries the
        library's own message."""
        import numpy as np
        from bayerlink import encode_packed
        from bayerlink.protocol import _GROUP

        from . import capture

        ref = self._camera_ref
        crop = spec.get("crop")
        if crop is not None:
            window = capture.validate_crop(crop, ref["sensor"], ref["bits"])
        else:
            window = (0, 0, *ref["sensor"])
        x, y, w, h = window
        capture.check_rate(w, ref["display"])
        group_samples, group_bytes = _GROUP[ref["bits"]]
        line_bytes = w // group_samples * group_bytes
        encode_packed(np.zeros((h, line_bytes), np.uint8), ref["order"],
                      frame_seq=0, bits=ref["bits"], display=ref["target"])
        ref["crop"] = window
        self.spec = dict(spec)
        self.last_error = None
        self._persist()

    def stop(self) -> None:
        self._camera_ref = None
        with self._lock:
            thread, self._thread = self._thread, None
            generator, self._frames_gen = getattr(self, "_frames_gen",
                                                  None), None
        if thread is not None and thread.is_alive():
            self._stop.set()
            thread.join(timeout=5)
        if generator is not None:
            # Runs the generator's finally NOW -- an exclusive device
            # (the camera) must be free before the next source opens it.
            generator.close()

    def _run(self, frames, mode, pace: bool = False,
             container: bool = False) -> None:
        from .kms import DisplayNotReady

        def on_frame(index):
            self._frames = index
            if self._stop.is_set():
                raise _Stop
        try:
            self._runner(frames, mode=mode, connector=self.connector,
                         card_path=self.card_path, on_frame=on_frame,
                         container=container)
        except _Stop:
            pass
        except DisplayNotReady:
            self._monitor(frames, mode, pace, on_frame)
        except Exception as error:            # noqa: BLE001 -- reported, not eaten
            self.last_error = f"{type(error).__name__}: {error}"

    def _monitor(self, frames, mode, pace: bool, on_frame) -> None:
        """The source runs even with no display: a viewfinder needs a
        sensor, not a receiver.

        Frames are pulled and discarded -- pulling is what makes the
        camera integrate, the peek feed the live view, and the frame
        counter tell the truth -- while a cheap sysfs poll watches for
        a display. The moment one appears this loop simply ENDS: a dead
        thread plus retained intent is exactly the state the keeper
        already revives, through the real scanout this time.
        """
        from .kms import display_present

        self.monitoring = True
        self.last_error = None
        interval = 1.0 / (mode[2] or 30)
        checked = time.monotonic()
        try:
            for index, _ in enumerate(frames):
                on_frame(index)
                if pace:
                    time.sleep(interval)
                now = time.monotonic()
                if now - checked >= 1.0:
                    checked = now
                    if display_present():
                        return               # the keeper takes it from here
        except _Stop:
            pass
        except Exception as error:            # noqa: BLE001 -- reported, not eaten
            self.last_error = f"{type(error).__name__}: {error}"
        finally:
            self.monitoring = False


def _chain(first, rest):
    yield first
    yield from rest


# ----------------------------------------------------------------------------- #
# HTTP surface
# ----------------------------------------------------------------------------- #

def _bridge_command(line: str) -> str:
    """One command over the bridge's control socket, one reply.

    The bridge owns the far-end register channel and the module in it
    owns the vocabulary; this daemon is transport between the browser
    and that socket, and adds no meaning of its own.
    """
    import socket as sockets

    s = sockets.socket(sockets.AF_UNIX, sockets.SOCK_STREAM)
    s.settimeout(2.0)
    try:
        s.connect("/run/bridge.sock")
        s.sendall(line.encode() + b"\n")
        return s.recv(4096).decode(errors="replace").strip()
    finally:
        s.close()


def _sensors_from_sysfs() -> list:
    """The sensors present, read from sysfs -- no device is opened.

    Enumerating through libcamera ACQUIRES the camera, which is exactly
    what starved the bridge of frames. The kernel already names every
    v4l2 subdev ("ov5647 10-0036"), and a name is all a source list
    needs.
    """
    found = []
    for name in sorted(Path("/sys/class/video4linux").glob("v4l-subdev*/name")):
        try:
            model = name.read_text().split()[0]
        except OSError:
            continue
        if model and model not in found:
            found.append(model)
    return found


def _bridge_service(action: str) -> bool:
    """Start or stop the genlocked bridge. The panel orchestrates which
    engine owns the bench, because a person choosing a source should not
    have to know there are two daemons."""
    import subprocess

    try:
        if action == "start" or action == "restart":
            # CLEAR THE FAILURE FIRST. The bridge's restarts are bounded
            # (StartLimitBurst), so a mode it cannot carry leaves the
            # unit `failed` and systemd then refuses to start it again --
            # "Start request repeated too quickly". Without this, one bad
            # choice in the panel is a dead end: the picture is black and
            # picking a GOOD mode does not bring it back either, because
            # the start is refused before the bridge ever runs. Asking
            # for a source is asking for a fresh attempt, so the rate
            # limit from the last attempt must not veto it.
            subprocess.run(["systemctl", "reset-failed", "bridge"],
                           check=False, capture_output=True)
        return subprocess.run(["systemctl", action, "bridge"],
                              check=False).returncode == 0
    except OSError:
        # No systemd, or no bridge unit: this daemon is the only engine
        # here, which is the ordinary case on a development machine.
        return False


def _bridge_last_words() -> str:
    """The last thing the bridge said before it stopped.

    The bridge diagnoses its own refusals precisely -- "the sensor is
    sending 2592x1944, which does not fit the 1920x1081 raster this link
    advertises. Choose a smaller sensor mode." Throwing that away and
    substituting a guess ("the sensor may not offer that mode") turns a
    fixable answer into a black screen, so the panel repeats what the
    bridge actually said.
    """
    import subprocess

    try:
        out = subprocess.run(
            ["journalctl", "-u", "bridge", "-n", "40", "--no-pager",
             "-o", "cat"],
            check=False, capture_output=True, text=True).stdout
    except OSError:
        return ""
    said = [ln.strip() for ln in out.splitlines()
            if ln.startswith("bridge: ") or ln.startswith("revela: ")]
    # Its running status lines are chatter; a refusal is the last thing
    # that is not one of those.
    for line in reversed(said):
        if " seq " in line or "LOCKED" in line or "winsorized" in line:
            continue
        return line
    return ""


CLIP_PATH = "/run/bridge-clip.raw"


def route_to_bridge(spec: dict, spool: Path) -> str | None:
    """Hand a source to the bridge if the bridge is its engine.

    Returns what the bridge was asked to draw, or None if this source is
    not the bridge's to draw -- patterns, and anything at all on a
    machine that has no bridge. Raises ValueError with the reason when
    the bridge should have taken it and could not.

    ONE place decides which engine draws a source, because there were
    two: the HTTP route chose correctly, and the keeper -- realising the
    SAME stored intent, unattended -- called this daemon's own engine
    directly. So a recording a person selected was drawn by the bridge,
    and the identical recording revived hours later was drawn here,
    taking the display and leaving the bridge unable to modeset. The
    symptom was a sensor that would not stream in any mode; the cause
    was a second copy of a decision.
    """
    wanted = spec.get("source")
    if wanted not in ("camera", "file") or not _bridge_up():
        return None
    if wanted == "camera":
        reply = _bridge_command("bridge source sensor")
        if not reply.startswith("ok"):
            raise ValueError(reply)
        return "sensor"
    name = str(spec.get("file", ""))
    path = spool / name
    if path.name != name or not name.endswith(".npy"):
        raise ValueError(f"recording name {name!r} is not a plain <name>.npy")
    if not path.exists():
        raise ValueError(f"no recording {name!r}")
    info = _prepare_clip(path)
    reply = _bridge_command(f"bridge source clip {CLIP_PATH}")
    if not reply.startswith("ok"):
        raise ValueError(reply)
    return f"clip {name} ({info.get('frames', '?')} frames)"


def _prepare_clip(path: Path) -> dict:
    """A recording into the raw containers the bridge plays.

    The FORMAT is bayerlink's -- what a recorded frame is, and that the
    header row is part of it, is the codec's to define, not this
    instrument's. All that happens here is the flattening: whole
    containers, tightly packed, for a bridge that copies rows.

    Recordings predating the header row still exist on benches. They
    remain playable, but only by generating one, and a generated header
    is a guess about mosaic order -- so it is built by the codec that
    owns the format and reported as generated rather than passed off as
    recorded.
    """
    import numpy as np
    from bayerlink import recording

    try:
        stack = recording.load(path)
        recording.split(stack[0])
        header_source = "recorded"
    except ValueError:
        # No header row, or one that does not parse: fall back to the
        # legacy shape and say so.
        from bayerlink.protocol import Header, fourcc_for

        legacy = np.load(path, mmap_mode="r")
        if legacy.ndim == 3:
            legacy = legacy[None]
        if legacy.ndim != 4 or legacy.shape[1:] != (1080, 1920, 3):
            raise ValueError(
                f"{path.name}: not a bayerlink recording, and not a bare "
                f"1920x1080 payload either (shape {legacy.shape}); the "
                "bridge's raster is fixed at build time") from None
        row = np.frombuffer(
            Header(fourcc=fourcc_for("GBRG", 10), width=1920, height=1080,
                   frame_seq=0, source_id=0).pack().ljust(1920 * 3, b"\x00"),
            dtype=np.uint8).reshape(1, 1920, 3)
        stack = np.concatenate(
            [np.broadcast_to(row, (len(legacy), 1, 1920, 3)), legacy], axis=1)
        header_source = "generated"

    if stack.shape[1:] != (1081, 1920, 3):
        raise ValueError(
            f"{path.name}: {stack.shape[2]}x{stack.shape[1] - 1} is not the "
            "bridge's 1920x1080 raster, which is fixed at build time")
    with open(CLIP_PATH + ".new", "wb") as out:
        for i in range(len(stack)):
            out.write(np.ascontiguousarray(stack[i]).tobytes())
    Path(CLIP_PATH + ".new").rename(CLIP_PATH)
    return {"frames": len(stack), "header": header_source}


RAW_CAPTURE = "/run/bridge-capture.raw"


def _capture_from_bridge(spool: Path) -> str:
    """Freeze, copy the displayed container, carry on.

    The bridge is the engine, so the frame to capture is the one IT is
    showing. Taken while paused, which costs the picture nothing and
    makes the copy whole by construction: the sensor is writing the
    scratch buffer, so there is no next frame racing the read. Grabbing
    it live would tear, because the following frame begins the instant
    the last one lands.

    Saved through bayerlink's recording format, header row included --
    so a capture is a container, and replays as the bytes it was.
    """
    import time

    import numpy as np
    from bayerlink import recording

    held = _bridge_command("bridge status").find("paused=1") >= 0
    if not held:
        _bridge_command("bridge pause")
        time.sleep(0.15)          # one frame at any rate this bench runs
    try:
        reply = _bridge_command(f"bridge capture {RAW_CAPTURE}")
    finally:
        if not held:
            _bridge_command("bridge resume")
    if not reply.startswith("ok "):
        raise ValueError(reply)
    _, width, lines, _steady = reply[3:].split()
    raw = np.fromfile(RAW_CAPTURE, dtype=np.uint8)
    container = raw.reshape(int(lines), int(width), 3)

    number = 1
    while (spool / f"capture-{number:03d}.npy").exists():
        number += 1
    path = spool / f"capture-{number:03d}.npy"
    recording.save(path, container)
    return path.name


def _bridge_up(attempts: int = 40) -> bool:
    """Start the bridge if needed and wait for it to answer.

    Starting is asynchronous -- systemd returns long before the daemon
    has built, opened the sensor and reached its loop -- so a command
    issued straight after start gets ECONNREFUSED. Bounded on purpose:
    a bridge that has not answered in twenty seconds is not coming, and
    a wait without a limit outlives the bench that is waiting on it.
    """
    import time

    if _bridge_running():
        return True
    if not _bridge_service("start"):
        # No systemd, or no bridge unit: there is nothing to wait for,
        # and waiting anyway would hang every request on a machine that
        # simply does not have one.
        return False
    for _ in range(attempts):
        time.sleep(0.5)
        if _bridge_running():
            return True
    return False


def _bridge_owns_bench() -> bool:
    """Whether the bridge is the engine here -- asked of systemd, not of
    a socket.

    Ownership and reachability are different questions, and confusing
    them cost the picture: during the seconds a bridge spends building,
    opening the sensor and reaching its loop, its control socket does
    not answer yet. Anything that reads "no bridge" from that silence
    -- the keeper above all -- reaches for the camera the bridge is
    about to stream, and the bridge then starves for as long as it
    runs. The unit is active from the moment it is started, which is
    exactly when the hands must come off.
    """
    import subprocess

    # INSTALLED, not running. Whether the bridge happens to be up is a
    # state that changes by the second; whether this bench HAS a bridge
    # is a property of the machine, and it is the second question that
    # decides who may open the sensor.
    #
    # Asking the first question is what cost the bench on 2026-08-25. A
    # power cut left the bridge failing on a stale device path. It
    # flapped, and every reader of "is it active?" saw `no` in the gaps
    # -- so the panel enumerated cameras through libcamera, libcamera
    # opened the sensor, and the panel then HELD it. Repairing the
    # bridge no longer helped: the camera it needed was taken, by the
    # very daemon that is not supposed to touch it. Widening the check
    # to accept `activating` did not fix it either, because a unit that
    # has properly given up reads `failed`, and a failed bridge is
    # exactly when the sensor must stay untouched so that fixing the
    # bridge is enough.
    #
    # So: if a bridge unit is installed here, the bridge owns the
    # camera -- running, restarting, failed or stopped. On a machine
    # with no bridge unit this daemon is the engine, which is the
    # ordinary development case and the only case that changes.
    try:
        enabled = subprocess.run(
            ["systemctl", "is-enabled", "bridge"],
            check=False, capture_output=True, text=True).stdout.strip()
    except OSError:
        return False
    return enabled in ("enabled", "enabled-runtime", "static", "linked",
                       "indirect", "generated", "alias")


def _bridge_running() -> bool:
    """Whether the genlocked bridge is the one driving the bench.

    Asked by anything that would otherwise TOUCH the camera or the
    display: while the bridge runs it owns both, and merely opening the
    sensor to enumerate it is enough to starve the loop of frames --
    which is not a theory, it is what took the picture down on
    2026-08-22 every time a panel tab polled for cameras.
    """
    try:
        _bridge_command("bridge status")
        return True
    except OSError:
        return False


def _isp_regmap():
    """The register map the installed ISP module carries, or None."""
    from . import modules_mgr

    path = modules_mgr.slot_dir("isp") / "regmap.json"
    if not path.exists():
        return None
    return json.loads(path.read_text())


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
            # ROUTE ON THE PATH, not on the path plus its query. Every
            # exact-match route below would otherwise 404 the moment a
            # caller appended anything -- and the panel appends a
            # cache-buster to the preview image on purpose, so
            # /bridge/preview.png?t=... never once reached this handler
            # and the panel showed a broken-image icon for it. Only
            # /preview.png survived, by the accident of needing
            # startswith for its ?recording= form.
            #
            # self.path is left intact: the handlers that read a query
            # parse it from there.
            route = self.path.split("?", 1)[0]
            if route in ("/", "/index.html"):
                page = (Path(__file__).parent / "ui.html").read_bytes()
                # Stamp the page with its own hash; status reports the
                # current one. A tab that outlives a deploy sees the
                # mismatch and reloads itself -- a stale panel is how a
                # stale spec gets re-applied to a live stream.
                page = page.replace(b"@PANEL@", _panel_hash().encode())
                return self._send_bytes(200, page, "text/html; charset=utf-8")
            if route == "/status":
                return self._send(200, supervisor.status())
            if route == "/cameras":
                # Do not open the sensor behind the bridge's back. The
                # enumeration itself acquires the camera, and the bridge
                # is mid-stream on it.
                if _bridge_owns_bench():
                    # Named from sysfs, never acquired: the bridge is
                    # mid-stream on it and enumeration would steal its
                    # frames. It stays IN the list -- it is what is on
                    # the television -- and says who is driving it.
                    return self._send(200, {
                        "cameras": [{"index": i, "model": model,
                                     "driven_by": "bridge"}
                                    for i, model in
                                    enumerate(_sensors_from_sysfs())],
                        "owner": "bridge"})
                # Enumerated fresh on each ask: cameras appear when a ribbon
                # is reseated, and a panel poll should see that without a
                # daemon restart. On a bench with no camera stack this is
                # simply an empty list, never an error.
                try:
                    from picamera2 import Picamera2
                    cameras = [{"index": i,
                                "model": info.get("Model", f"camera {i}")}
                               for i, info in
                               enumerate(Picamera2.global_camera_info())]
                except Exception:               # noqa: BLE001 -- no camera stack
                    cameras = []
                # The streaming sensor publishes its own mode list (the
                # handle is open; nobody else may open it). A panel that
                # cannot see the real modes cannot mirror the stream,
                # and a panel that cannot mirror will overwrite.
                modes = supervisor.sensor_modes()
                if modes:
                    for camera in cameras:
                        if camera["index"] == supervisor.camera_index():
                            camera["modes"] = [list(m) for m in modes]
                return self._send(200, {"cameras": cameras})
            if route == "/modules":
                from . import modules_mgr
                return self._send(200, modules_mgr.status())
            if route == "/bridge/status":
                # The bridge is the real-time owner on this bench: it
                # holds the sensor and the display and keeps the picture
                # alive whether or not this daemon is running. The panel
                # does not stream alongside it -- it drives it.
                # A stopped bridge is a STATE, not a failure to answer:
                # with no source selected it is exactly what should be
                # true. Reporting it as an HTTP error made "nothing is
                # streaming" read as a fault, errno and all.
                try:
                    r = _bridge_command("bridge status")
                except OSError:
                    return self._send(200, {
                        "running": False,
                        # Active but silent means it is still building,
                        # opening the sensor and reaching its loop.
                        "starting": _bridge_owns_bench()})
                if not r.startswith("ok"):
                    return self._send(400, {"error": r})
                fields = {}
                for token in r[3:].split():
                    key, _, value = token.partition("=")
                    try:
                        fields[key] = float(value) if "." in value else int(value)
                    except ValueError:
                        fields[key] = value
                fields["running"] = True
                return self._send(200, fields)
            if route == "/bridge/preview.png":
                # What the SENSOR is writing, decimated by the bridge --
                # the scratch buffer while paused, the displayed one
                # otherwise. Raw Bayer sampled at constant CFA phase, so
                # it is a grey frame: the source's view, not the ISP's.
                import numpy as np
                try:
                    r = _bridge_command("bridge preview")
                except OSError as error:
                    return self._send(503, {
                        "error": f"bridge control socket: {error}"})
                if not r.startswith("ok "):
                    return self._send(400, {"error": r})
                path, width, height = r[3:].split()
                raw = Path(path).read_bytes()
                body = raw.split(b"\n", 3)[3]
                img = np.frombuffer(body[:int(width) * int(height)],
                                    dtype=np.uint8).reshape(int(height),
                                                            int(width))
                return self._send_bytes(200, _png_gray(img), "image/png")
            if route == "/isp/params":
                regmap = _isp_regmap()
                if regmap is None:
                    return self._send(404, {
                        "error": "no ISP module installed, or it predates "
                                 "the map-carrying package format -- "
                                 "reinstall the isp slot"})
                return self._send(200, regmap)
            if route == "/isp/values":
                # Read every register back off the far end, one command
                # per key. Sequential and a little slow by design: the
                # DDC wire is the truth being asked for, and the panel
                # asks once, on demand.
                regmap = _isp_regmap()
                if regmap is None:
                    return self._send(404, {"error": "no ISP module "
                                                     "installed"})
                values, unreachable = {}, []
                try:
                    for block in regmap["blocks"]:
                        for register in block["registers"]:
                            k = f"{block['path']}.{register['name']}"
                            r = _bridge_command(f"get {k}")
                            if r.startswith("ok "):
                                values[k] = int(r[3:])
                            else:
                                unreachable.append(k)
                except (OSError, ValueError) as error:
                    return self._send(503, {
                        "error": f"bridge control socket: {error}"})
                return self._send(200, {"values": values,
                                        "unreachable": unreachable})
            if route == "/recordings":
                return self._send(200, {"recordings":
                                        supervisor.status()["spool"]})
            if route.startswith("/preview.png"):
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
            if route.startswith("/recordings/"):
                path = self._spool_name(route[len("/recordings/"):])
                if path is None:
                    return self._send(400, {"error": "recording name must "
                                            "be a plain <name>.npy"})
                if not path.exists():
                    return self._send(404, {"error": "no such recording"})
                return self._send_bytes(200, path.read_bytes(),
                                        "application/octet-stream")
            return self._send(404, {"error": f"no route {self.path!r}"})

        def do_POST(self):
            route = self.path.split("?", 1)[0]
            if route == "/capture":
                if _bridge_running():
                    try:
                        name = _capture_from_bridge(supervisor.spool)
                    except (OSError, ValueError) as error:
                        return self._send(400, {
                            "error": f"{type(error).__name__}: {error}"})
                    return self._send(200, {"saved": name, "by": "bridge"})
                try:
                    name = supervisor.capture()
                except ValueError as error:
                    return self._send(400, {"error": str(error)})
                return self._send(200, {"saved": name})
            if route == "/isp/set":
                length = int(self.headers.get("Content-Length", 0))
                try:
                    body = json.loads(self.rfile.read(length) or b"{}")
                    key, value = str(body["key"]), int(body["value"])
                except (ValueError, KeyError) as error:
                    return self._send(400, {
                        "error": f'/isp/set takes {{"key": '
                                 f'"block.register", "value": raw-int}}: '
                                 f"{error}"})
                if any(c.isspace() for c in key):
                    return self._send(400, {"error": "key cannot contain "
                                                     "whitespace"})
                try:
                    r = _bridge_command(f"set {key} {value}")
                except OSError as error:
                    return self._send(503, {
                        "error": f"bridge control socket: {error}"})
                if not r.startswith("ok"):
                    return self._send(400, {"error": r})
                return self._send(200, {"set": key, "value": value})
            if route in ("/bridge/pause", "/bridge/resume"):
                verb = route.rsplit("/", 1)[1]
                try:
                    r = _bridge_command(f"bridge {verb}")
                except OSError as error:
                    return self._send(503, {
                        "error": f"bridge control socket: {error}"})
                if not r.startswith("ok"):
                    return self._send(400, {"error": r})
                return self._send(200, {verb: True, "detail": r[3:].strip()})
            if route == "/isp/commit":
                try:
                    r = _bridge_command("commit")
                except OSError as error:
                    return self._send(503, {
                        "error": f"bridge control socket: {error}"})
                if not r.startswith("ok"):
                    return self._send(400, {"error": r})
                return self._send(200, {"committed": True})
            self._send(404, {"error": "unknown path"})

        def do_DELETE(self):
            route = self.path.split("?", 1)[0]
            if route.startswith("/modules/"):
                from . import modules_mgr
                try:
                    result = modules_mgr.remove(
                        route[len("/modules/"):])
                except ValueError as error:
                    return self._send(400, {"error": str(error)})
                return self._send(200, result)
            if route.startswith("/recordings/"):
                path = self._spool_name(route[len("/recordings/"):])
                if path is None:
                    return self._send(400, {"error": "recording name must "
                                            "be a plain <name>.npy"})
                if not path.exists():
                    return self._send(404, {"error": "no such recording"})
                path.unlink()
                return self._send(200, {"deleted": path.name})
            return self._send(404, {"error": f"no route {self.path!r}"})

        def do_PUT(self):
            route = self.path.split("?", 1)[0]
            length = int(self.headers.get("Content-Length", 0))
            if route == "/power":
                try:
                    spec = json.loads(self.rfile.read(length) or b"{}")
                except ValueError as error:
                    return self._send(400, {"error": str(error)})
                if spec.get("off") is not True:
                    return self._send(400, {"error":
                                            'PUT /power takes {"off": true}; '
                                            "nothing else is a power action"})

                def _off():
                    import subprocess
                    subprocess.run(list(POWEROFF), check=False)
                # Answer first, act second: the reply must leave the wire
                # before the network interface goes down with the OS.
                threading.Timer(1.0, _off).start()
                return self._send(200, {"powering_off": True})
            if route == "/source":
                try:
                    spec = json.loads(self.rfile.read(length) or b"{}")
                    # ONE selection decides what is on the television,
                    # and the panel picks the engine for it: the
                    # genlocked bridge drives the live camera, this
                    # daemon drives files and patterns, and the two can
                    # never run at once because there is one display.
                    # Somebody choosing a source should not have to know
                    # any of that.
                    wanted = spec.get("source", "off")
                    # A new selection supersedes whatever went wrong
                    # before it. The panel shows last_error, so an error
                    # kept past its cause makes every later success look
                    # like a failure -- which is exactly how a working
                    # bench reads as broken.
                    supervisor.last_error = None
                    if wanted == "camera" and _bridge_up():
                        # A bridge exists and it owns live camera: hand
                        # the bench over. Where there is no bridge --  a
                        # development box -- this daemon stays the camera
                        # engine and nothing below changes.
                        # Anything the bridge cannot act on is REFUSED,
                        # not quietly dropped. Accepting a mode change
                        # and then not changing the mode is the worst of
                        # the three possible answers.
                        # A MODE change is a restart, by design: the
                        # whole start-up path recomputes the line lock
                        # from the sensor's pixel rate and the new
                        # geometry. The frame need only fit the raster;
                        # it sits inside it and the header carries the
                        # truth. Disturbing the picture is the agreed
                        # price of changing geometry.
                        mode = spec.get("cam_mode")
                        if mode and len(mode) == 2:
                            want = f"{int(mode[0])}x{int(mode[1])}"
                            now = ""
                            try:
                                r = _bridge_command("bridge status")
                                fields = dict(
                                    t.split("=", 1) for t in r[3:].split()
                                    if "=" in t)
                                now = f"{fields.get('width')}x{fields.get('height')}"
                            except OSError:
                                pass
                            if want != now:
                                env = Path("/run/bridge.env")
                                # Keep what worked. A mode the link
                                # cannot carry must not leave the bench
                                # unable to come back: without this the
                                # refused mode stays in the env file and
                                # every later start fails on it too, so
                                # one bad choice is a dead end.
                                before = (env.read_text()
                                          if env.exists() else None)
                                env.write_text(
                                    f"BRIDGE_ARGS=--mode {want}\n")
                                _bridge_service("restart")
                                if not _bridge_up():
                                    if before is None:
                                        env.unlink(missing_ok=True)
                                    else:
                                        env.write_text(before)
                                    _bridge_service("restart")
                                    said = _bridge_last_words()
                                    return self._send(503, {
                                        "error": said or (
                                            f"the bridge did not come back "
                                            f"after asking for {want}"),
                                        "reverted": True})
                        ignored = [k for k in ("crop",)
                                   if spec.get(k) is not None]
                        supervisor.stop()
                        try:
                            route_to_bridge({"source": "camera"},
                                            supervisor.spool)
                        except ValueError as error:
                            return self._send(400, {"error": str(error)})
                        supervisor._persist_intent({"source": "camera"})
                        status = supervisor.status()
                        if ignored:
                            status["ignored"] = {
                                "settings": ignored,
                                "why": "the bridge streams at the geometry "
                                       "it was built for and drives "
                                       "exposure and gain from its own "
                                       "loop; these were not applied"}
                        return self._send(200, status)
                    if wanted == "file" and spec.get("file") and _bridge_up():
                        # ONE engine: the bridge draws the clip into the
                        # same buffer the sensor would have written, so
                        # the raster, the header and the lock are the
                        # ones already running. This daemon never takes
                        # the display. The routing itself lives in
                        # route_to_bridge, so the keeper reaches the same
                        # engine for the same intent.
                        supervisor.stop()
                        try:
                            drawn = route_to_bridge(spec, supervisor.spool)
                        except ValueError as error:
                            return self._send(400, {"error": str(error)})
                        if not drawn:
                            return self._send(503, {
                                "error": "the bridge did not take the clip"})
                        supervisor._persist_intent(spec)
                        status = supervisor.status()
                        status["playing"] = {"file": spec["file"],
                                             "by": "bridge", "drawn": drawn}
                        return self._send(200, status)
                    # Whatever is left -- patterns, and anything at all
                    # on a machine with no bridge -- this daemon serves,
                    # so a running bridge hands the display back first.
                    if _bridge_running():
                        _bridge_service("stop")
                    if wanted == "off":
                        supervisor.stop()
                        supervisor._persist_intent({"source": "off"})
                        return self._send(200, supervisor.status())
                    supervisor.start(spec)
                except Exception as error:   # noqa: BLE001 -- named, not dropped
                    # Whatever went wrong belongs in the response: an
                    # instrument that answers with a closed connection
                    # turns a missing dependency into a mystery.
                    return self._send(400, {
                        "error": f"{type(error).__name__}: {error}"})
                return self._send(200, supervisor.status())
            if route.startswith("/modules/"):
                from . import modules_mgr
                slot = route[len("/modules/"):]
                try:
                    result = modules_mgr.install(slot,
                                                 self.rfile.read(length))
                except ValueError as error:
                    return self._send(400, {"error": str(error)})
                except Exception as error:  # noqa: BLE001 -- the compile log
                    return self._send(400, {
                        "error": f"{type(error).__name__}: {error}"})
                return self._send(200, result)
            if route.startswith("/recordings/"):
                name = route[len("/recordings/"):]
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
        # An appliance boots into its source -- and keeps heading there.
        # Early boot is exactly when the neighbours are not ready (a
        # camera still enumerating, a capture stick that wakes seconds
        # after power), and a bench UNPLUGS things: the keeper retries
        # whenever the instrument is idle but its retained intent says
        # otherwise, forever, gently. Plug the display back in and the
        # stream returns by itself; an explicit 'off' is intent too and
        # is respected.
        try:
            supervisor.start(autostart)
        except Exception as error:            # noqa: BLE001
            supervisor.last_error = f"waiting to start: {error}"
    supervisor.keep()
    server = ThreadingHTTPServer((bind, port), _handler(supervisor))
    print(f"picam2hdmi instrument on http://{bind}:{port} "
          f"(spool: {supervisor.spool})")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        supervisor.stop()
