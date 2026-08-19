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

#endif
