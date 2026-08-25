# Copyright 2026 Serge Rabyking
# SPDX-License-Identifier: MIT
"""Who may open the sensor, and why the answer must not flap.

On a bridge bench the C bridge is the ONLY thing that opens the camera.
This daemon asks one question to know whether it is on such a bench, and
on 2026-08-25 it asked the wrong one and cost the bench a day:

A power cut left the bridge failing on a device path that had moved.
`Restart=always` turned that into a unit that flapped between
`activating`, `active` and `auto-restart` every couple of seconds. The
check was `systemctl is-active`, so several times a minute it answered
"no bridge here" -- and on one of those beats the panel enumerated
cameras through libcamera, libcamera opened the sensor, and the panel
HELD it. From then on repairing the bridge did not help, because the
camera it needed was taken by the daemon that must never touch it.

Widening the check to accept `activating` is NOT enough, and that is the
subtle part: a unit that has properly given up reads `failed`, and a
failed bridge is precisely when the sensor must stay untouched, so that
fixing the bridge is sufficient on its own.

So the question is whether a bridge is INSTALLED, which is a property of
the machine and does not change while the bench is running.
"""
from __future__ import annotations

import subprocess

import pytest

from picam2hdmi import serve


def _systemctl(reply: str, *, raises: bool = False):
    """Stand in for `systemctl is-enabled bridge`."""
    def run(cmd, *a, **k):
        if raises:
            raise OSError("no systemd here")
        assert cmd[:2] == ["systemctl", "is-enabled"], cmd
        return subprocess.CompletedProcess(cmd, 0, stdout=reply, stderr="")
    return run


@pytest.mark.parametrize("state", ["enabled", "enabled-runtime", "static",
                                   "linked", "indirect", "generated", "alias"])
def test_an_installed_bridge_owns_the_camera(monkeypatch, state):
    """Installed in any form means the bridge owns the sensor."""
    monkeypatch.setattr(subprocess, "run", _systemctl(state))
    assert serve._bridge_owns_bench() is True


def test_a_failed_bridge_still_owns_the_camera(monkeypatch):
    """THE regression. A bridge that has given up is still the owner.

    `is-enabled` answers the same for a failed unit as for a healthy
    one, which is the whole point: the sensor stays free for the bridge
    to reclaim once someone fixes it, instead of being taken by this
    daemon the moment the bridge stops."""
    monkeypatch.setattr(subprocess, "run", _systemctl("enabled"))
    assert serve._bridge_owns_bench() is True


def test_no_bridge_unit_means_this_daemon_is_the_engine(monkeypatch):
    """A development machine has no bridge, and must still work."""
    monkeypatch.setattr(subprocess, "run", _systemctl("disabled"))
    assert serve._bridge_owns_bench() is False


def test_no_systemd_at_all_means_this_daemon_is_the_engine(monkeypatch):
    monkeypatch.setattr(subprocess, "run", _systemctl("", raises=True))
    assert serve._bridge_owns_bench() is False


def test_the_answer_does_not_depend_on_whether_the_bridge_is_running():
    """The check must not consult ActiveState at all.

    Asserted structurally rather than by behaviour: a future edit that
    reaches for `is-active` reintroduces exactly the flap that emptied
    the bench, and would otherwise pass every test above."""
    import inspect

    source = inspect.getsource(serve._bridge_owns_bench)
    body = source.split('"""')[-1]          # past the docstring
    assert "is-active" not in body, (
        "_bridge_owns_bench must not ask whether the bridge is RUNNING: "
        "a unit under Restart=always flaps, and a flapping answer lets "
        "this daemon open the sensor between restarts")
    assert "ActiveState" not in body, (
        "ActiveState flaps for the same reason; ask is-enabled instead")
