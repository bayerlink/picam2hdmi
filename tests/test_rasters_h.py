# Copyright 2026 Serge Rabyking
# SPDX-License-Identifier: MIT
"""The bridge's raster table is bayerlink's, generated: compiled and read
back, every row is the row bayerlink states."""
import shutil
import subprocess

import pytest
from bayerlink.rasters import RASTERS

from picam2hdmi.native import CSRC, rasters_h

pytestmark = pytest.mark.skipif(shutil.which("cc") is None,
                                reason="needs a C compiler")

PRINT = r'''
#include <stdio.h>
#include "rasters.h"
int main(void) {
    for (size_t i = 0; i < BRIDGE_N_RASTERS; i++) {
        const struct bridge_raster *r = &BRIDGE_RASTERS[i];
        printf("%s %u %u %u %u %u %u %u %u %u %u %u %u\n", r->name,
               r->pixel_khz, r->h_total, r->v_total, r->h_active,
               r->v_active, r->hs_beg, r->hs_end, r->vs_beg, r->vs_end,
               r->hsync_positive, r->vsync_positive, r->hz);
    }
    return 0;
}
'''


def test_the_bridges_rasters_are_bayerlinks(tmp_path):
    (tmp_path / "rasters.h").write_text(rasters_h())
    (tmp_path / "print.c").write_text(PRINT)
    binary = tmp_path / "print"
    subprocess.run(["cc", "-Wall", "-Wextra", "-Werror", f"-I{CSRC}",
                    f"-I{tmp_path}", str(tmp_path / "print.c"), "-o",
                    str(binary)], check=True)
    got = subprocess.run([str(binary)], capture_output=True, text=True,
                         check=True).stdout.split("\n")[:-1]
    fields = ("pixel_khz", "h_total", "v_total", "h_active", "v_active",
              "hs_beg", "hs_end", "vs_beg", "vs_end", "hsync_positive",
              "vsync_positive", "hz")
    want = [" ".join([name, *(str(int(getattr(r, f))) for f in fields)])
            for name, r in RASTERS.items()]
    assert got == want
