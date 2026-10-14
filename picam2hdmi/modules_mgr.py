# Copyright 2026 Serge Rabyking
# SPDX-License-Identifier: MIT
"""Module packages: uploaded, validated, compiled in, one per slot.

The bridge's algorithm seam takes source, not binaries: a package is
a tar.gz of C files, headers and a manifest, unpacked into a SLOT
under csrc/modules/ and compiled in by the on-device rebuild.
The slots are the firmware's parts: `isp` (its core -- owns the bridge
hooks and every far-end register), `sensor` (the sensor's facts and
fitted profile, as data), and `ae`, `awb`, `af` (each 3A loop its own
module: computation through what the core lends it). The manifests'
provides/requires contracts pair them. The manifests' provides/requires contracts are checked here,
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

SLOTS = ("isp", "sensor", "ae", "awb", "af")
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


def _check_set(new: dict) -> str | None:
    """Every requires must be satisfiable across the slots as they will
    be: those installed, with `new` ({slot: manifest}) in place of what
    its slots hold. A contract an OCCUPIED slot provides under a different
    version is a mismatch; an empty slot is fine -- install order is the
    operator's."""
    installed = {sl: manifest_of(sl) for sl in SLOTS}
    installed.update(new)
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


def _unpack(slot: str, blob: bytes) -> tuple[dict, dict]:
    """A package's manifest and files, validated for its slot. Raises
    ValueError with the reason on anything a caller can fix."""
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
    files = {m.name: tar.extractfile(m).read() for m in members}
    if "manifest.json" not in files:
        raise ValueError("no manifest.json in package")
    manifest = json.loads(files["manifest.json"])
    if manifest.get("kind") != "picam2hdmi-module" or manifest.get("abi") != 1:
        raise ValueError("manifest kind/abi is not picam2hdmi-module/1")
    if manifest.get("slot") != slot:
        raise ValueError(f"package is for slot {manifest.get('slot')!r}, "
                         f"not {slot!r}")
    for src in manifest.get("sources", []):
        if src not in files:
            raise ValueError(f"manifest names {src!r} but it is not in "
                             "the package")
    return manifest, files


def install(slot: str, blob: bytes) -> dict:
    """Unpack, validate, install, rebuild. Raises ValueError with the
    reason on anything a caller can fix."""
    result = install_many({slot: blob})
    return {**result["installed"][0], "bridge_restarted":
            result["bridge_restarted"]}


def install_bundle(blob: bytes) -> dict:
    """A bundle: one tar.gz of `<slot>.tar.gz` packages, installed as one
    change (install_many). What an ABI bump needs -- a provider and its
    consumers move together, and one at a time each is refused against
    the other's old version."""
    try:
        tar = tarfile.open(fileobj=io.BytesIO(blob), mode="r:gz")
    except tarfile.TarError as e:
        raise ValueError(f"not a tar.gz: {e}") from None
    packages = {}
    for m in tar.getmembers():
        if (not m.isfile() or not m.name.endswith(".tar.gz") or "/" in m.name
                or m.name.startswith(".")):
            raise ValueError(f"a bundle holds <slot>.tar.gz files; {m.name!r} "
                             "is not one")
        packages[m.name[:-len(".tar.gz")]] = tar.extractfile(m).read()
    if not packages:
        raise ValueError("an empty bundle")
    return install_many(packages)


def install_many(packages: dict) -> dict:
    """Install {slot: package} as ONE change: every package validated, the
    contracts checked across the slots as they will be, all written, one
    rebuild and one bridge restart. If the result does not compile, every
    slot goes back to what it held."""
    unpacked = {slot: _unpack(slot, blob) for slot, blob in packages.items()}
    clash = _check_set({slot: m for slot, (m, _) in unpacked.items()})
    if clash:
        raise ValueError(clash)
    saved = {}
    for slot, (_, files) in unpacked.items():
        d = slot_dir(slot)
        saved[slot] = ({p.name: p.read_bytes() for p in d.iterdir()
                        if p.is_file()} if d.exists() else None)
        if d.exists():
            shutil.rmtree(d)
        d.mkdir(parents=True)
        for name, data in files.items():
            (d / name).write_bytes(data)
    try:
        ensure_built(rebuild=False)   # source changed => new cache key
    except Exception:
        # what does not compile does not stay installed
        for slot, old in saved.items():
            d = slot_dir(slot)
            shutil.rmtree(d, ignore_errors=True)
            if old is not None:
                d.mkdir(parents=True)
                for name, data in old.items():
                    (d / name).write_bytes(data)
        raise
    restarted = _restart_bridge()
    return {"installed": [{"installed": m.get("name"), "slot": slot,
                           "version": m.get("version")}
                          for slot, (m, _) in unpacked.items()],
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
    """Best-effort: if the bridge runs as the systemd fixture -- or did,
    until a module crashed it -- bounce it so the new build takes over:
    installing modules is how a crashing one is replaced. A bridge stopped
    on purpose, or one this daemon did not start, stays as it is."""
    r = subprocess.run(["systemctl", "is-active", "--quiet", "bridge"],
                       check=False)
    failed = subprocess.run(["systemctl", "is-failed", "--quiet", "bridge"],
                            check=False)
    if r.returncode != 0 and failed.returncode != 0:
        return False
    # A restart asked for is not a crash: clear the unit's start count
    # first, or a handful of uploads in two minutes trips the limit that
    # exists to stop a crash loop, and the bridge stays down.
    subprocess.run(["systemctl", "reset-failed", "bridge"], check=False)
    subprocess.run(["systemctl", "restart", "bridge"], check=False)
    return True
