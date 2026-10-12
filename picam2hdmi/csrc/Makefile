# Copyright 2026 Serge Rabyking
# SPDX-License-Identifier: MIT
# The bridge is rebuilt ON DEVICE from this directory: bridge.c plus
# every module in modules/ (AE and friends -- see bridge.h for the
# hook contract). `picam2hdmi bridge` drives the same build through
# native.py; this Makefile is the bench-side equivalent, and `make
# check` holds the C and Python twins byte-identical where they must
# agree -- the bayerlink header.
CFLAGS ?= -O2 -Wall -Wextra $(shell pkg-config --cflags libdrm)
LDLIBS ?= $(shell pkg-config --libs libdrm) -lm
SRC = $(wildcard *.c) $(wildcard modules/*.c) $(wildcard modules/*/*.c)

# rasters.h is bayerlink's raster table, generated, never written here
gen/rasters.h: bridge.h
	@mkdir -p gen
	python3 -c "from picam2hdmi.native import rasters_h; \
	print(rasters_h(), end='')" > $@

bridge: $(SRC) bridge.h gen/rasters.h
	$(CC) $(CFLAGS) -I. -Igen -o $@ $(SRC) $(LDLIBS)

check: bridge
	@./bridge --selftest --order GBRG > /tmp/bridge_c_headers
	@python3 -c "\
	from bayerlink.protocol import Header, fourcc_for; \
	fc = fourcc_for('GBRG', 10); \
	print('\n'.join(Header(fourcc=fc, width=1920, height=1080, \
	frame_seq=s, source_id=1).pack().hex() for s in (7, 0, 123456)))" \
	> /tmp/bridge_py_headers
	@diff /tmp/bridge_c_headers /tmp/bridge_py_headers \
	&& echo "check: C and Python headers byte-identical"

clean:
	rm -rf bridge gen
.PHONY: check clean
