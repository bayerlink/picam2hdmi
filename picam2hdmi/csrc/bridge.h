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

#include <stdint.h>

struct bridge_frame {
    uint32_t seq;           /* frames since STREAMON                  */
    double phase_lines;     /* filtered genlock phase, display lines  */
    int locked;             /* genlock believes the beams are held    */
    int sensor_fd;          /* the camera subdev, for V4L2 controls   */
};

void bridge_frame_hook(const struct bridge_frame *f);

/* The LINK hook: called once after start-up with the initial state,
 * and again on every HDMI hot-plug edge. This is where a module
 * reloads the far end -- the receiver forgets its registers when
 * the cable or its power goes, and the sensor's owner is the one
 * who knows what to write back. `edid` is the sink's EDID (256
 * bytes) when the link is up and readable, else NULL; a module can
 * recognise its far end there before touching a register. */
struct bridge_link {
    int up;
    const uint8_t *edid;    /* 256 bytes, or NULL */
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

#endif
