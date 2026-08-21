# Genlock: slaving the HDMI output to the sensor

The bridge (`csrc/bridge.c`) locks the Pi's HDMI output timing to
the sensor's frame delivery, so that every captured frame leaves on
the cable exactly once — no drops, no doubles, ever. This document
is the why and the how.

## Why: coherent is not equal

The sensor and the display pipeline share the Pi's crystal, so
their clocks hold a fixed ratio forever — there is no drift. But
each side's *frame period* is set by integers rounded
independently: libcamera programs the sensor's VTS/HTS to
approximate the requested rate within the sensor's register
granularity, and the display driver synthesizes its pixel clock
within its divider granularity. Nobody chooses the two sets of
integers together, so a constant residual of tens of ppm remains
between the two frame periods.

A constant rate mismatch slips phase at a constant speed. Every so
many seconds, a capture completion crosses a vblank boundary, and
the framebuffer handoff silently drops or doubles a frame. For a
preview nobody notices; for a link whose receiver judges frames
bit-exactly, every drop is a defect that looks like transport loss
and isn't.

## How: two knobs, split by what each does safely

**Rate, once — hblank.** At start the bridge sets the display's
horizontal blanking so the display line time equals the sensor
line time (and the vertical total equals the sensor's VTS): the
rates then match as closely as the integers allow. This knob is
never touched live, because writing the sensor's line length
mid-readout glitches a row — the sensor has no group hold on that
register.

**Phase, per frame — vblank.** What remains after the rate match is
absorbed by trimming the display's vertical total ±1 line at frame
boundaries. A one-line trim is glitch-free by construction and
invisible to the receiver at the far end of the cable.

## The hard part is measuring, not adjusting

The phase estimate comes from frame timestamps in userspace Linux,
and that ground is hostile:

- The capture driver delivers zero timestamps despite advertising
  monotonic ones, so the bridge stamps frames itself at dequeue.
- Self-stamps ride scheduling noise with fat tails: a hard
  rejection window goes deaf exactly when it matters, so samples
  are winsorized toward dead reckoning instead — an outlier pulls
  the estimate a bounded amount, never owns it.
- A trim's effect appears one frame later. Corrections are
  slew-limited to one line per frame with a cooldown, so a trim is
  never judged by the sample still measuring the old phase.
- The safety margin (how early the frame must be ready before its
  vblank) is an adaptive ratchet: it widens instantly on a close
  call and narrows by one line per minute — so it converges to
  what this Pi, this sensor and this load actually need, rather
  than a worst case measured once. A six-hour soak settled at six
  lines of margin, about a quarter millisecond.

## What this buys the chain

With the bridge running, the cable carries the sensor's true
cadence — the receiver sees one container frame per sensor frame,
at crystal-accurate spacing. The receiving board then genlocks its
own display to that cadence at the far end (the receiver's story:
`docs/clocking.md` in bayerlink-fpga). Each side disciplines the
clock domain it owns, and the frame count stays exact from
photosite to glass.
