/* Copyright 2026 Serge Rabyking
 * SPDX-License-Identifier: MIT
 *
 * The module seam. The bridge is rebuilt ON DEVICE from this source
 * tree (see native.py), and every .c file in modules/ is compiled in
 * -- that is how algorithm code (AE first, others after it) rides
 * along and runs in real time on the Pi, one hook call per frame.
 *
 * A module overrides the weak bridge_frame_hook. The contract:
 *   - called once per captured frame, after the genlock has run;
 *   - sensor_fd is open on the camera subdevice: V4L2 controls
 *     (exposure, analogue gain) are the module's actuators, and
 *     writes land on the NEXT frame, the sensor's own cadence;
 *   - stay well under a millisecond -- the hook shares the frame
 *     budget with the header write and the genlock;
 *   - statistics arrive over the HDMI parameter back-channel in a
 *     later version; until then a module sees only what is here.
 */
#ifndef BRIDGE_H
#define BRIDGE_H

#include <stddef.h>
#include <stdint.h>

struct bridge_frame {
    uint32_t seq;           /* frames since STREAMON                  */
    double phase_lines;     /* filtered genlock phase, display lines  */
    int locked;             /* genlock believes the beams are held    */
    int sensor_fd;          /* the camera subdev, for V4L2 controls   */
};

void bridge_frame_hook(const struct bridge_frame *f);

/* A display raster: totals, active area, sync windows and polarities,
 * pixel clock -- one row of bayerlink.rasters, which the on-device
 * build generates into rasters.h, so C keeps no copy of its own.
 * Positions count from the first active pixel (line). */
struct bridge_raster {
    const char *name;       /* "1920x1080p30" */
    uint32_t pixel_khz;
    uint32_t h_total, v_total, h_active, v_active;
    uint32_t hs_beg, hs_end, vs_beg, vs_end;
    uint8_t hsync_positive, vsync_positive;
    uint32_t hz;            /* to the nearest whole frame */
};

/* The MODE: what this run scans out and what it carries. The raster
 * is the link's; the frame is the sensor's, sitting inside it, with
 * the header row above -- `lines` active lines on the wire. */
struct bridge_mode {
    const struct bridge_raster *raster;
    uint32_t width, height;     /* the sensor's frame */
    uint32_t lines;             /* active lines scanned: height + header */
};

/* The LINK hook: called once after start-up with the initial state,
 * and again on every HDMI hot-plug edge. This is where a module
 * reloads the far end -- the receiver forgets its registers when
 * the cable or its power goes, and the sensor's owner is the one
 * who knows what to write back. `edid` is the sink's EDID (256
 * bytes) when the link is up and readable, else NULL; a module can
 * recognise its far end there before touching a register. `mode`
 * is this run's, always set. */
struct bridge_link {
    int up;
    const uint8_t *edid;    /* 256 bytes, or NULL */
    const struct bridge_mode *mode;
};

void bridge_link_hook(const struct bridge_link *l);

/* The far-end register channel: the display cable's own I2C bus,
 * reaching the receiver's register personality (16-bit register
 * address, 32-bit big-endian value, auto-increment -- the wire
 * protocol of the receiver's ddc engine). The bridge owns bus
 * discovery; a module just reads and writes. Both return 0 on
 * success, -1 with errno on a bus that is absent or not answering
 * -- which a module must expect: cables are cables. */
int bridge_ddc_write(uint16_t reg, uint32_t value);
int bridge_ddc_read(uint16_t reg, uint32_t *value);

/* The COMMAND hook: one short text line from the local control
 * surface (the instrument's web panel, via the bridge's UNIX socket
 * at /run/bridge.sock -- one command per connection). The bridge
 * owns the transport and nothing else: only a module knows what its
 * far end's registers mean, so the vocabulary is the module's. Fill
 * `reply` (NUL-terminated, within `cap`) with "ok ..." or
 * "err <reason>"; the transport sends it verbatim. Called on the
 * frame thread between frames -- same budget rules as the frame
 * hook. */
void bridge_command_hook(const char *cmd, char *reply, size_t cap);

#endif
