# Copyright 2026 Serge Rabyking
# SPDX-License-Identifier: MIT
"""Build the C bridge ON DEVICE, on request.

The binary is not a fixed artifact: it is the output of `bridge.c plus
whatever modules are present` (AE and friends -- csrc/bridge.h holds
the hook contract), so it is compiled where it runs, from the source
this package carries as data. The cache key is the source itself --
change a module, and the next run rebuilds; same source, same binary,
no work. Nothing binary ever ships in the wheel.
"""
from __future__ import annotations

import hashlib
import os
import shutil
import subprocess
import sys
from pathlib import Path

CSRC = Path(__file__).parent / "csrc"
CACHE = Path(os.environ.get("XDG_CACHE_HOME",
                            Path.home() / ".cache")) / "picam2hdmi"


def _sources() -> list[Path]:
    files = (sorted(CSRC.glob("*.c")) + sorted(CSRC.glob("modules/*.c"))
             + sorted(CSRC.glob("modules/*/*.c")))
    files = sorted(CSRC.glob("*.h")) + files
    # editors and macOS both leave dot-droppings that match *.c
    return [f for f in files if not f.name.startswith(".")]


def _cc_line() -> list[str]:
    pkg = subprocess.run(["pkg-config", "--cflags", "--libs", "libdrm"],
                         capture_output=True, text=True)
    if pkg.returncode != 0:
        raise RuntimeError(
            "pkg-config cannot find libdrm -- install libdrm-dev; the "
            "bridge is built on the device it runs on, and that is the "
            "one library it needs")
    flags = pkg.stdout.split()
    srcs = [str(p) for p in _sources() if p.suffix == ".c"]
    return ["cc", "-O2", "-Wall", "-Wextra", *srcs, *flags, "-lm"]


def ensure_built(rebuild: bool = False) -> Path:
    """Compile if the source changed; return the binary path."""
    if shutil.which("cc") is None:
        raise RuntimeError("no C compiler -- install gcc; the bridge is "
                           "built on the device it runs on")
    key = hashlib.sha256()
    for src in _sources():
        key.update(src.name.encode())
        key.update(src.read_bytes())
    out = CACHE / f"bridge-{key.hexdigest()[:12]}"
    if out.exists() and not rebuild:
        return out
    CACHE.mkdir(parents=True, exist_ok=True)
    cmd = _cc_line() + ["-o", str(out)]
    run = subprocess.run(cmd, capture_output=True, text=True,
                         errors="replace")
    if run.returncode != 0:
        raise RuntimeError(f"bridge build failed:\n{run.stderr}")
    if run.stderr:
        print(run.stderr, file=sys.stderr)      # warnings are not clean
    return out


def check(binary: Path) -> bool:
    """The twin discipline: the C copy is verified against the codec that
    owns these facts, never trusted.

    Three claims, all proven here at every start-up: the header bytes
    match Python's, every row of the format table carries the fourcc
    bayerlink says it should, and the crop rule -- an odd offset flips
    that axis's Bayer parity -- agrees with bayerlink's phase bits. A
    wrong row is a header that misdescribes its payload, and that reads
    at the far end as a broken picture rather than a broken table.
    """
    from bayerlink.protocol import BAYER_PHASE, Header, fourcc_for

    out = subprocess.run([str(binary), "--selftest", "--order", "GBRG"],
                         capture_output=True, text=True).stdout.splitlines()
    headers = [l for l in out if not l.startswith(("fmt ", "crop "))]
    fc = fourcc_for("GBRG", 10)
    want = [Header(fourcc=fc, width=1920, height=1080, frame_seq=s,
                   source_id=0).pack().hex() for s in (7, 0, 123456)]
    if headers != want:
        return False

    for line in (l for l in out if l.startswith("fmt ")):
        _, order, bits, stamped = line.split()
        if stamped != fourcc_for(order, int(bits)):
            return False

    by_phase = {phase: order for order, phase in BAYER_PHASE.items()}
    for line in (l for l in out if l.startswith("crop ")):
        _, order, dx, dy, after = line.split()
        if after != by_phase[BAYER_PHASE[order] ^ (int(dy) << 1) ^ int(dx)]:
            return False
    return True
