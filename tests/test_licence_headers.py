# Copyright 2026 Serge Rabyking
# SPDX-License-Identifier: MIT
"""Every source file carries this repository's licence, and the right one.

Checked mechanically because a licence header applied inconsistently is
worse than useless: a reviewer cannot tell whether a file without one was
an oversight or was deliberately contributed under different terms, and
that ambiguity is exactly what a licence audit stops on. A file carrying
the WRONG identifier is worse still -- it is a claim, not a gap.

The identifier is not written here. It is read from the packaging
metadata, which is where this project already keeps the version, so the
licence has one owner too and this test cannot drift from what is
actually published.
"""
from __future__ import annotations

import re
import sys
import tomllib
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
SOURCE_ROOTS = ("picam2hdmi", "tests", "contrib")
SUFFIXES = (".py", ".c", ".h", ".v", ".sv", ".sh", ".tcl", ".xdc",
            ".core", ".html")
# Source files with no suffix to match on.
NAMED = ("Makefile",)
# JSON carries no comment syntax, so a description in .json cannot
# hold a header and is covered by the repository's LICENSE instead.
# Named here so the gap is a decision, not an oversight.
NO_COMMENT_SYNTAX = (".json",)


def declared_licence() -> str:
    """The licence this project publishes under, from pyproject."""
    data = tomllib.loads((ROOT / "pyproject.toml").read_text())
    licence = data.get("project", {}).get("license")
    if isinstance(licence, dict):          # the older table form
        licence = licence.get("text")
    assert licence, "pyproject.toml declares no licence to check against"
    return licence


def sources() -> list[Path]:
    files: list[Path] = []
    for root in SOURCE_ROOTS:
        directory = ROOT / root
        if not directory.is_dir():
            continue
        files.extend(sorted(p for p in directory.rglob("*")
                            if (p.suffix in SUFFIXES
                                or p.name in NAMED)
                            and "__pycache__" not in p.parts))
    return files


def test_there_are_files_to_check():
    """A check that finds nothing passes forever."""
    assert len(sources()) > 3, (
        f"only {len(sources())} source files found; this check is not "
        "covering the tree it is meant to cover")


@pytest.mark.parametrize("path", sources(),
                         ids=lambda p: str(p.relative_to(ROOT)))
def test_file_carries_the_projects_licence(path: Path):
    ident = declared_licence()
    head = "".join(path.read_text(encoding="utf-8",
                                  errors="replace").splitlines(True)[:6])
    found = re.search(r"SPDX-License-Identifier:\s*(.+?)\s*"
                      r"(?:-->|\*/)?\s*$", head, re.M)
    assert found, (
        f"{path.relative_to(ROOT)} has no SPDX-License-Identifier in its "
        f"first lines; every source file states the licence it is under.\n"
        f"Expected: SPDX-License-Identifier: {ident}")
    assert found.group(1) == ident, (
        f"{path.relative_to(ROOT)} claims {found.group(1)!r} but this "
        f"project publishes as {ident!r} (pyproject.toml). A file under "
        "different terms is a decision, not a typo -- make it deliberately "
        "or fix the header.")
