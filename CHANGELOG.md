# Changelog

Notable changes to picam2hdmi. This release is dominated by one theme:
**one engine drives the bench, and one owner holds each fact.** Most of
what follows is a consequence of settling that.

Pre-1.0, so the HTTP surface and the CLI can still move. Anything that
moves is named here.

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
