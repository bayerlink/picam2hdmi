# Changelog

Notable changes to picam2hdmi. This release is dominated by one theme:
**one engine drives the bench, and one owner holds each fact.** Most of
what follows is a consequence of settling that.

Pre-1.0, so the HTTP surface and the CLI can still move. Anything that
moves is named here.

## Unreleased

### Added

- **The link carries the sensor as it is.** The link raster is derived
  from the sensor's own timing in its mode -- one sensor line a link line,
  one frame a frame, at the link's clock (`--link-khz`, 74250 by default:
  what the receiver is built for) -- so no frame rate is chosen anywhere;
  the bridge's own copy of the 1080p30 modeline is gone. `--mode WxH` sets
  the sensor's size and nothing else, and with no mode the bridge takes
  the sensor's current one rather than assuming 1920x1080. Every link edge
  hands modules the mode -- raster, frame, lines -- and `bridge status`
  reports the link's clock and totals, the depth and the mosaic order.
- **Slots for each 3A loop.** `ae`, `awb` and `af` replace the single
  `algo` slot: each loop is its own module.
- **The firmware is brought up at what streams.** Starting the camera
  sends the firmware module its `init` -- the frame and its timing from
  `bridge status`, a TV raster in a familiar shape whose frame lasts as
  long as the sensor's, the 3A settings -- and the keeper does the same
  once per bridge run, so a module installed, a reboot or a restart does
  not leave it waiting. `/status` carries its answer.
- **No line budget in the panel.** The link's line is the sensor's, so the
  fixed 2200-slot budget the panel enforced is gone.

- **`stream --source tof`**: an Arducam ToF depth camera's raw phase
  frames, through Arducam's SDK (which owns the sensor's configuration).
  Its row-interleaved buffer is de-interleaved and each phase plane sent
  as its own bayerlink frame at the sensor's geometry -- what a CSI-2
  receiver would deliver -- with the phase in `frame_seq % 4`. Samples
  are signed correlations, sent as unsigned with a named offset of 2**15.

- **A 480p120 raster** (CEA-861 VIC 48, 54 MHz) for `stream`, for sensors
  that read out faster than 60 frames a second: the slowest standard raster that
  scans out that fast, a third of 720p60's data rate. Measured on a Pi 2
  at 119.9 Hz. Sync polarity now travels with each raster -- this one is
  negative on both, 720p and 1080p positive.

### Changed

- **The rasters are bayerlink's** (`bayerlink.rasters`). The proven
  container rasters and the line-slot budget read the one table both ends
  of a link share instead of keeping their own timings; the numbers are
  unchanged, and 720x480 now has its budget checked too.

### Fixed

- **A command is read to its newline, and answered in full.** The
  control socket took one `read` into 256 bytes and replied in 256; a
  firmware configuration is hundreds of bytes, and arrived cut. Both are
  2048 bytes now, the line read until its newline and the reply read to
  the close.
- **Uploading modules no longer stops the bridge.** Each install restarts
  it, and six in two minutes tripped the unit's start limit, which is
  there for crash loops; an asked-for restart clears the count first.

- **A fresh Lite install builds the bridge.** `SETUP.md` and
  `contrib/install.sh` install `python3-pip` and `libdrm-dev` by name:
  with `--no-install-recommends` nothing else brings them, Lite ships no
  pip, and without libdrm's headers the bridge's on-device build stopped
  at first stream with `pkg-config cannot find libdrm`.

- **The bridge waits for its display at power-on.** The board downstream
  boots after the Pi, and the bridge used to refuse at once: the unit
  spent its restart budget in seconds and ended `failed` with the board
  nearly ready. It now waits up to five minutes for a connected HDMI
  sink, saying so every 30 s, and then refuses as before.

## 0.4.0

### Added

- **The C bridge** — sensor lines land on the wire in *lines*, not
  frames. It writes into the very buffer HDMI is scanning out of and the
  write beam chases the read beam, so the lead between them is the whole
  of this device's contribution to latency. `hblank` makes the line times
  equal once; `vblank` trims the phase continuously. Built on the device
  it runs on, so the binary is always this package's C plus whatever
  modules are installed.

- **A module seam.** Packages upload into slots — `isp`, `sensor`,
  `algo` — and are compiled into the bridge on upload. A package that
  does not build does not stay installed.

- **The panel**, a phone-friendly control page over the same HTTP the
  CLI uses, plus the cable's own register channel and link events.

- **`Pause`** — hold the picture without stopping the camera, and
  `Capture`, which saves the frame on the display as a recording. Capture
  *is* a pause: it freezes, copies the held container, and resumes,
  because grabbing it live would tear.

### Changed — one engine, one owner

- **The bridge draws the picture, whatever it is of**, and the panel
  picks the engine rather than competing with it. One source list; the
  keeper stands down while a bridge owns the bench. Where there is no
  bridge — a development machine — this daemon stays the engine and
  nothing changes.

- **Exposure and gain leave this package entirely** — the bridge, the
  CLI (`--exposure-us`, `--gain`) and the panel's fields. **Breaking** for
  anyone passing those flags. They belong to the sensor package, where
  the AE loop lives; a second writer here wins or loses a race with that
  loop and the picture is wrong in a way that gets blamed on whatever is
  looked at next. The bridge keeps *geometry* — format, blanking, and the
  switches that turn the sensor's own AE, AGC and AWB off — and publishes
  the VTS bound rather than enforcing a value it does not own.

- **`source_id` is stamped as the index it is**, following the protocol
  library's correction.

- **The instrument stops having opinions about the file format.** A
  recording is the wire, kept, and bayerlink says what that means.

### Fixed

- **The bridge finds its sensor by name, not by number.** `/dev/videoN`
  is enumeration order, not identity: after a power cut the Pi came back
  with unicam on `video0` and a capture card on `video2`, so the bridge
  asked a UVC card for 10-bit Bayer and got `S_FMT: Invalid argument` —
  the right complaint about the wrong device. It now matches
  `unicam-image` (or `rp1-cfe-csi2_ch0` on Pi 5) and refuses rather than
  guessing.

- **The bridge owns the camera whether or not it is running.** The panel
  asked whether the bridge was *active*, which flaps under
  `Restart=always`; on one of those beats it opened the sensor itself and
  held it, after which repairing the bridge no longer helped. It now asks
  whether a bridge is *installed*, which does not flap.

- **A sensor mode the link cannot carry is no longer a dead end.** The
  panel repeats the bridge's own diagnosis instead of guessing, reverts
  to the mode that was working, and clears the failure so a later choice
  can start.

- **A still outlives the captures already in flight.** Selecting a
  recording froze the display on the last camera frame until you clicked
  twice: the queue is two deep against one buffer, so the capture still
  in flight landed on top of the still, which was drawn only once.

- **Routing ignores the query string.** Fourteen routes matched the whole
  path, so `/bridge/preview.png?t=…` — the panel's own cache-buster —
  404'd every time and the sensor preview had never once loaded.

### Removed

- **The luma tunnel**, following the protocol library.
- **The wire overlay** in the crop area, which could never have shown
  anything once the bridge became the camera engine.
- **The emitting-now view**; it had nothing left to show.

### Internal

- Every source file carries its copyright and SPDX identifier, checked in
  the suite — including `contrib/`, the bridge's `Makefile` and the
  panel's HTML, which were outside every checked root.
