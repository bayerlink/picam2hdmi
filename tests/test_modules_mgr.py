# Copyright 2026 Serge Rabyking
# SPDX-License-Identifier: MIT
"""The module slots: a package is refused when its contracts break the
slots as they would be, and a bundle installs a provider and its consumers
as one change -- the only way an ABI bump can land at all."""
import io
import json
import tarfile

import pytest

from picam2hdmi import modules_mgr


def _pkg(slot, version, provides=(), requires=()):
    manifest = {"kind": "picam2hdmi-module", "abi": 1, "slot": slot,
                "name": f"{slot}-{version}", "version": version,
                "sources": [f"{slot}.c"], "provides": list(provides),
                "requires": list(requires)}
    return _tar({"manifest.json": json.dumps(manifest).encode(),
                 f"{slot}.c": f"/* {slot} {version} */\n".encode()})


def _tar(files):
    out = io.BytesIO()
    with tarfile.open(fileobj=out, mode="w:gz") as tar:
        for name, data in files.items():
            info = tarfile.TarInfo(name)
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))
    return out.getvalue()


@pytest.fixture
def slots(tmp_path, monkeypatch):
    monkeypatch.setattr(modules_mgr, "CSRC", tmp_path)
    built = []
    monkeypatch.setattr(modules_mgr, "ensure_built",
                        lambda rebuild=False: built.append(1))
    monkeypatch.setattr(modules_mgr, "_restart_bridge", lambda: True)
    modules_mgr.install("isp", _pkg("isp", 4, provides=["api:4"]))
    modules_mgr.install("sensor", _pkg("sensor", 4, requires=["api:4"]))
    return tmp_path


def test_each_half_of_an_abi_bump_alone_is_refused(slots):
    with pytest.raises(ValueError, match="ABI mismatch"):
        modules_mgr.install("isp", _pkg("isp", 5, provides=["api:5"]))
    with pytest.raises(ValueError, match="ABI mismatch"):
        modules_mgr.install("sensor", _pkg("sensor", 5, requires=["api:5"]))
    assert modules_mgr.manifest_of("isp")["version"] == 4


def test_a_bundle_lands_both_halves_as_one_change(slots):
    bundle = _tar({"isp.tar.gz": _pkg("isp", 5, provides=["api:5"]),
                   "sensor.tar.gz": _pkg("sensor", 5, requires=["api:5"])})
    result = modules_mgr.install_bundle(bundle)
    assert {r["slot"] for r in result["installed"]} == {"isp", "sensor"}
    assert modules_mgr.manifest_of("isp")["version"] == 5
    assert modules_mgr.manifest_of("sensor")["version"] == 5


def test_a_bundle_still_breaking_a_slot_is_refused(slots):
    modules_mgr.install("ae", _pkg("ae", 4, requires=["api:4"]))
    bundle = _tar({"isp.tar.gz": _pkg("isp", 5, provides=["api:5"]),
                   "sensor.tar.gz": _pkg("sensor", 5, requires=["api:5"])})
    with pytest.raises(ValueError, match="'ae' requires"):
        modules_mgr.install_bundle(bundle)
    assert modules_mgr.manifest_of("isp")["version"] == 4


def test_a_bundle_that_does_not_build_restores_every_slot(slots,
                                                          monkeypatch):
    def broken(rebuild=False):
        raise RuntimeError("bridge build failed")
    monkeypatch.setattr(modules_mgr, "ensure_built", broken)
    bundle = _tar({"isp.tar.gz": _pkg("isp", 5, provides=["api:5"]),
                   "sensor.tar.gz": _pkg("sensor", 5, requires=["api:5"])})
    with pytest.raises(RuntimeError):
        modules_mgr.install_bundle(bundle)
    assert modules_mgr.manifest_of("isp")["version"] == 4
    assert modules_mgr.manifest_of("sensor")["version"] == 4
