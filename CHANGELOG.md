# Changelog

Notable changes to picam2hdmi. This release is dominated by one theme:
**one engine drives the bench, and one owner holds each fact.** Most of
what follows is a consequence of settling that.

Pre-1.0, so the HTTP surface and the CLI can still move. Anything that
moves is named here.

## Unreleased

### Added

- **The bridge carries a sensor whose frames come in loops** (`--loop
  N`) -- a time-of-flight module's reference phases, four to a
  measurement -- as it carries any sensor's: unicam writes each frame into
  a buffer the display scans, the samples untouched, and the CPU writes
  the header. Such a sensor takes a loop's frames a few milliseconds
  apart and then waits most of a loop, so each frame lands in a buffer of
  its own and goes on the wire at the first vertical blank after it
  arrived, in order; a blank with nothing new clears the header, so the
  far end -- a frame store -- sees no frame there. The link runs 5%
  faster than the sensor's frame rate, timed from its frames at start, so
  the frames waiting drain between bursts and none is dropped. Where in
  its loop a frame is comes from when it arrived -- a loop starts after
  the long gap -- and each loop's first frame takes the next multiple of
  N in `frame_seq`, so `frame_seq % N` is its place on the wire. The
  format is the sensor's own, found by its media-bus code.

- **A loop run's frame and its count go out together.** Each frame has a
  header row of its own, and the header's buffer and the payload's swap in
  one atomic commit, so the display takes both in the same blank. Written
  into the buffer already on the wire once the payload's overlay change
  returned, the header missed that frame's top line and went out with the
  next, so every frame reached the far end with its predecessor's count
  -- in order, none lost, every phase one place off.

- **Driver controls by name** (`--ctrl name=value`, repeatable), set
  before the sensor streams, each one the driver must have.

- **The sensor's module says how its frames come.** A new hook,
  `bridge_source_hook`, lets the module describing the sensor state its
  loop and its driver controls; the bridge asks it before each sensor
  run, and the run's own options win where given. So a bridge started
  with no options -- the service, after a power cut -- streams a
  time-of-flight module correctly with nothing restated on the Pi.

- **The display is the mode's where the board keeps the pictures.** When
  the firmware reports `keeps=1`, the panel brings it up on the sensor
  module's mode of the frame the bridge streams, which brings the display
  its pictures are shown on; a display the source names (`"display":
  "1080p60"`, bayerlink's raster table) is that run's alone, never stored.
  A board showing the link's own timing refuses a named display.

- **An image plays at the rate asked, and changes the least it can.** The
  panel lists the spool's raw images apart from its recordings, each
  played by the bridge with no sensor; an image's rate is the link's own
  -- one image frame a link frame, so the ISP sees exactly it, and the TV
  follows as it must with no frame store, with no floor applied. The
  image's line is kept and the frame made as long as the rate takes;
  where the display's counters (from the firmware's status) cannot reach
  it, the line moves, down to 480p's clock for the longest. Where the
  raster already on the wire carries the image at exactly the rate asked
  -- the camera's 1080p30 does a 1080 image at 30 fps -- the image is
  planned onto it, so the switch changes nothing on the cable or on the
  TV. A rate the link cannot carry is refused with the range it can.

- **The bridge reconfigures without restarting.** `bridge reconfigure
  <args>` takes the words BRIDGE_ARGS holds, parsed by the one parser the
  command line uses, and the panel switches every source and rate with
  it: the bridge owns the display for its whole life, so nothing else
  drives the cable between two configurations. The wire changes the
  least it can -- nothing where the mode and the buffers already hold,
  so a new image of the same timing is simply the next frame; a mode set
  once, in a vertical blank, where the timing moves; new buffers, drawn
  before they go on, where the geometry does. The run ending finishes its
  frame whole, every run counts its frames from 0, and the answer is the
  outcome: ok once the new configuration streams, or the refusal with the
  previous one streaming again. Before a mode set the header is cleared a
  vertical blank ahead of it: the display drops the old mode ~20 ms after
  the call, and the cut used to land in a picture, which the far end
  showed cut short.

- **The link carries the sensor as it is.** The link raster is derived
  from the sensor's own timing in its mode -- one sensor line a link line,
  one frame a frame, at the link's clock -- the far end's, read from the
  first detailed timing of the sink's EDID, since its receiver is built for
  one (`--link-khz` overrides; 74250 where the sink states none) -- so no
  frame rate is chosen anywhere;
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
- **A link line is as wide as the sensor's packed line**, three bytes a
  link pixel, at the first stride unicam keeps (asked with `TRY_FMT`, not
  assumed): 800 link pixels carry a 1920-sample 10-bit line, where 1920
  did. A sensor line shorter than its width in link clocks -- bayerlink's
  rate rule for a one-sample-a-clock receiver -- is stretched to it, so
  the full 2592x1944 array streams at about 15 fps on a 74.25 MHz link.
  The panel plans no timing a link line cannot carry, and sizes the
  display for what the ISP's crop keeps of a frame bigger than any TV
  area, not for the frame. A recording plays at whatever width it was
  made (below).
- **A raw image plays as a mode, with no sensor.** `PUT /source
  {"source": "image", "file": "<name>.npy", "fps": N}` takes a raw Bayer
  `.npy` from the spool -- one frame or several -- packs it by
  bayerlink's own rule, and plans its timing as the camera's is planned:
  a mode whose pixel rate is the link's clock, its line no shorter than
  its width nor than the ISP (the firmware's `isp_khz`) can drain, onto
  a raster of the TV's table. The bridge runs it as a clip run
  (`--clip`, `--size`, `--bits`, `--line-clocks`, `--frame-lines`): no
  camera opened, frames paced by the display's vblank, one image frame a
  link frame for the rate asked; modules are told there is no sensor
  (`bridge_mode.sensor`), and `bridge status` says `sensor=0`. Choosing the camera after an image restores the camera's
  options.
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

- **A recording plays as a clip of its own containers.** `--containers
  <width>` makes a clip run play whole bayerlink containers -- each
  frame's recorded header row and its lines, the format, size and mosaic
  order as the recording's first header states them -- so a replay is the
  wire as it was captured. A recording of any width plays, on the raster
  already streaming where its line and frame hold it (a recording made on
  the camera plays on the camera's), and the camera comes back by
  reconfiguring the bridge to the options it streamed with. The sensor run
  no longer replays anything: `bridge source` is gone, with the state and
  the redraw it needed in the frame loop.
- **The rasters are bayerlink's** (`bayerlink.rasters`). The proven
  container rasters and the line-slot budget read the one table both ends
  of a link share instead of keeping their own timings; the numbers are
  unchanged, and 720x480 now has its budget checked too.

### Removed

- **`picam2hdmi/bridge.py`**, the Python prototype the C bridge grew
  from. Nothing ran it -- `picam2hdmi bridge` builds and runs the C
  bridge, whose header `make check` holds to bayerlink's codec -- and it
  had none of the genlock, the clip runs or the reconfiguration.

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
