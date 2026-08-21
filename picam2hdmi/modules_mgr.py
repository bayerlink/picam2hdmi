# Copyright 2026 Serge Rabyking
# SPDX-License-Identifier: MIT
"""Module packages: uploaded, validated, compiled in, one per slot.

The bridge's algorithm seam takes source, not binaries: a package is
a tar.gz of C files, headers and a manifest, unpacked into a SLOT
under csrc/modules/ and compiled in by the on-device rebuild.
Three slots exist because the responsibilities do: `isp` (a use-case's
policy -- owns the bridge hooks and the far end's registers),
`sensor` (a driver -- writes its own sensor's registers, carries
its facts as data), and `algo` (an algorithm -- pure computation
through the APIs the policy package lends it; autofocus and its
kin). The manifests' provides/requires contracts are checked here,
at upload, so an ABI mismatch is an HTTP 400 and never a compile
error at three in the morning.

A LAN instrument accepting compilable C is a bench posture, the same
one the service file already declares -- firewall the port if the
LAN is not yours.
"""
from __future__ import annotations

import io
import json
import shutil
import subprocess
import tarfile

from .native import CSRC, ensure_built

SLOTS = ("isp", "sensor", "algo")
MAX_PACKAGE = 4 << 20


def slot_dir(slot: str):
    return CSRC / "modules" / slot


def manifest_of(slot: str) -> dict | None:
    p = slot_dir(slot) / "manifest.json"
    if not p.exists():
        return None
    try:
        return json.loads(p.read_text())
    except ValueError:
        return {"error": "unreadable manifest"}


def status() -> dict:
    return {slot: manifest_of(slot) for slot in SLOTS}


def _as_list(v) -> list:
    return v if isinstance(v, list) else ([v] if v else [])


def _check_pair(slot: str, manifest: dict) -> str | None:
    """Every requires must be satisfiable: a contract an OCCUPIED
    other slot provides under a different version is a mismatch; an
    empty slot is fine -- install order is the operator's."""
    installed = {sl: manifest_of(sl) for sl in SLOTS if sl != slot}
    installed[slot] = manifest
    provides: dict[str, str] = {}
    for m in installed.values():
        for p in _as_list((m or {}).get("provides")):
            name, _, ver = p.partition(":")
            provides[name] = ver
    for sl, m in installed.items():
        for r in _as_list((m or {}).get("requires")):
            name, _, ver = r.partition(":")
            if name in provides and provides[name] != ver:
                return (f"ABI mismatch: {sl!r} requires {r!r} but "
                        f"{name!r} is provided at {provides[name]!r}")
    return None


def install(slot: str, blob: bytes) -> dict:
    """Unpack, validate, install, rebuild. Raises ValueError with the
    reason on anything a caller can fix."""
    if slot not in SLOTS:
        raise ValueError(f"slot must be one of {SLOTS}")
    if len(blob) > MAX_PACKAGE:
        raise ValueError("package too large")
    try:
        tar = tarfile.open(fileobj=io.BytesIO(blob), mode="r:gz")
    except tarfile.TarError as e:
        raise ValueError(f"not a tar.gz: {e}") from None
    members = tar.getmembers()
    for m in members:
        if not m.isfile() or "/" in m.name or m.name.startswith("."):
            raise ValueError(f"package must be flat files; {m.name!r} is not")
    names = {m.name for m in members}
    if "manifest.json" not in names:
        raise ValueError("no manifest.json in package")
    manifest = json.loads(tar.extractfile("manifest.json").read())
    if manifest.get("kind") != "picam2hdmi-module" or manifest.get("abi") != 1:
        raise ValueError("manifest kind/abi is not picam2hdmi-module/1")
    if manifest.get("slot") != slot:
        raise ValueError(f"package is for slot {manifest.get('slot')!r}, "
                         f"not {slot!r}")
    for src in manifest.get("sources", []):
        if src not in names:
            raise ValueError(f"manifest names {src!r} but it is not in "
                             "the package")
    clash = _check_pair(slot, manifest)
    if clash:
        raise ValueError(clash)

    d = slot_dir(slot)
    if d.exists():
        shutil.rmtree(d)
    d.mkdir(parents=True)
    for m in members:
        (d / m.name).write_bytes(tar.extractfile(m).read())
    try:
        ensure_built(rebuild=False)   # source changed => new cache key
    except Exception:
        # a package that does not compile does not stay installed
        shutil.rmtree(d, ignore_errors=True)
        raise
    restarted = _restart_bridge()
    return {"installed": manifest.get("name"), "slot": slot,
            "version": manifest.get("version"),
            "bridge_restarted": restarted}


def remove(slot: str) -> dict:
    if slot not in SLOTS:
        raise ValueError(f"slot must be one of {SLOTS}")
    d = slot_dir(slot)
    if not d.exists():
        raise ValueError(f"nothing installed in slot {slot!r}")
    shutil.rmtree(d)
    ensure_built(rebuild=False)
    restarted = _restart_bridge()
    return {"removed": slot, "bridge_restarted": restarted}


def _restart_bridge() -> bool:
    """Best-effort: if the bridge runs as the systemd fixture, bounce
    it so the new build takes over. Anything else is the operator's
    arrangement and stays untouched."""
    r = subprocess.run(["systemctl", "is-active", "--quiet", "bridge"],
                       check=False)
    if r.returncode != 0:
        return False
    subprocess.run(["systemctl", "restart", "bridge"], check=False)
    return True
