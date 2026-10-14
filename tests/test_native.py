# Copyright 2026 Serge Rabyking
# SPDX-License-Identifier: MIT
"""The bridge is rebuilt whenever anything the compiler reads changes --
a module's generated header included -- and never for an editor's
leftovers."""
from picam2hdmi import native


def _tree(tmp_path, monkeypatch):
    csrc = tmp_path / "csrc"
    (csrc / "modules" / "isp").mkdir(parents=True)
    (csrc / "bridge.c").write_text("int main(void) { return 0; }\n")
    (csrc / "modules" / "isp" / "core.c").write_text("#include \"core_map.h\"\n")
    (csrc / "modules" / "isp" / "core_map.h").write_text("#define ROW 5\n")
    monkeypatch.setattr(native, "CSRC", csrc)
    return csrc


def _key(monkeypatch):
    seen = []
    monkeypatch.setattr(native.shutil, "which", lambda _: "/usr/bin/cc")
    monkeypatch.setattr(native, "CACHE", native.Path("/nonexistent"))

    def stop(out):
        seen.append(out.name)
        raise StopIteration
    monkeypatch.setattr(native.Path, "exists",
                        lambda self: stop(self) if self.name.startswith("bridge-")
                        else False)
    try:
        native.ensure_built()
    except StopIteration:
        pass
    return seen[0]


def test_a_module_header_is_part_of_the_build_key(tmp_path, monkeypatch):
    csrc = _tree(tmp_path, monkeypatch)
    before = _key(monkeypatch)
    (csrc / "modules" / "isp" / "core_map.h").write_text("#define ROW 1\n")
    assert _key(monkeypatch) != before


def test_dot_files_are_not(tmp_path, monkeypatch):
    csrc = _tree(tmp_path, monkeypatch)
    before = _key(monkeypatch)
    (csrc / "modules" / "isp" / "._core.c").write_text("junk")
    (csrc / ".cache").mkdir()
    (csrc / ".cache" / "x.h").write_text("junk")
    assert _key(monkeypatch) == before
