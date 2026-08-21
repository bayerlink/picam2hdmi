# picam2hdmi

**A Raspberry Pi camera as a raw-Bayer HDMI source.**

FPGA boards rarely have a camera connector, but nearly all of them have
HDMI-in. A Raspberry Pi has the opposite: a first-class camera stack — every
sensor libcamera supports, drivers, modes, controls — and an HDMI output.
picam2hdmi turns the Pi into a **sensor module with an HDMI plug**: raw
12-bit Bayer frames, straight from the sensor's CSI-2 output with the ISP
bypassed, carried over the display link as bytes and self-described by a
header line.

```
sensor ──CSI-2──▶ Pi (libcamera raw, zero-copy) ──HDMI──▶ any receiver with HDMI-in
```

No transcoding, no compression, no per-sensor code in this tool: the Bayer
order and bit depth travel in the stream itself, so a receiver built once
works with every camera the Pi supports.

## The protocol: bayerlink

The wire format is **[bayerlink](https://github.com/bayerlink/bayerlink)** —
the display's active area as a byte container, one header line making the
stream self-describing. The spec, the reference codec and the conformance
vectors live in the protocol's own repository, because this tool is one
encoder among several and should not own the contract receivers implement.

The codec runs on **both ends**: this tool encodes with it on the Pi, and a
receiver's host software decodes captured frames with the same package —

```python
import bayerlink

header, raw = bayerlink.decode_frame(captured)    # (H, W, 3) uint8 in
print(header.bayer_order, header.width, header.height, header.frame_seq)
# raw: (lines, samples) uint16, exactly what the sensor produced
```

## Status

| Piece | State |
| --- | --- |
| bayerlink v2 protocol, patterns, vectors | **done** — in [bayerlink](https://github.com/bayerlink/bayerlink) |
| CLI: pattern → container file | **done** |
| KMS scanout (double-buffered, full-range RGB forced) | **working** — pure ctypes DRM, proven on the bench |
| Picamera2 raw capture | **working** — the sensor's packed bytes ride to the wire verbatim; crop, fixed exposure/gain, every libcamera camera unseen |
| Instrument mode | **working** — `picam2hdmi serve`: HTTP control, recordings spool, a control panel with a live viewfinder crop editor and a power button; the last source is restored at boot (`contrib/picam2hdmi.service`) |
| Genlocked bridge | **working** — `picam2hdmi bridge`: the C daemon, sensor lines onto the wire in lines, output phase-locked to the sensor so latency is lines and no frame is shown twice or dropped (`contrib/bridge.service`) |
| Module slots | **working** — C packages uploaded from the panel and compiled into the bridge: `isp` (policy: the bridge hooks and the far end's registers), `sensor` (a driver), `algo` (pure computation) |

## Usage

Off-target, on any machine (numpy only):

```bash
pip install picam2hdmi
picam2hdmi pattern --mode counting --width 2028 --height 1078 --out frame.npy
```

That file is bit-for-bit what the HDMI link will carry — receivers can be
built and tested against it before any cable exists.

The same file IS a test pattern for the instrument: drop it into the
panel's recordings and it streams like any capture, at whatever geometry
you generated. Replay re-stamps the frame counter, so a single-frame
pattern file plays as a proper continuous stream:

```bash
picam2hdmi pattern --mode checker --width 512 --height 240 --out checker_512x240.npy
# then drag it into the panel at http://<pi>:8080/
```

On a Pi, **[SETUP.md](SETUP.md)** is the step-by-step from blank SD card to
streaming instrument (or `contrib/install.sh` does the install in one go).
The short of it:

```bash
sudo picam2hdmi stream --source camera --exposure-us 30000 --gain 4.0
sudo picam2hdmi serve                  # instrument: panel + HTTP on :8080
```

## Modules: someone else's algorithm, running on this Pi

The bridge is generic and stays that way — it knows frames, links and
cables, and nothing about what a picture *means*. What gives it meaning
is a **module**: a flat tar of C and a manifest, uploaded from the panel,
compiled into the bridge on the device, and rejected outright if it does
not build. Three slots, one per responsibility, so a package is only ever
replaced by another of its own kind:

| slot | what belongs there |
| --- | --- |
| `isp` | a use-case's policy: owns the bridge hooks and every write to the far end's registers |
| `sensor` | a driver: writes its own sensor's controls, carries the sensor's facts as data |
| `algo` | pure computation, reaching hardware only through what the policy package lends it |

The seam is four weak symbols in [`csrc/bridge.h`](picam2hdmi/csrc/bridge.h):
per-frame, on every HDMI hot-plug edge (with the sink's EDID, so a module
can recognise its far end before touching it), the cable's own register
channel, and one text command from the panel. Packages declare what they
provide and require; the set is checked at upload, so a mismatch is an
HTTP 400 rather than a compile error at three in the morning.

A first-class use of this is an ISP on the far end of the cable:
[revela](https://github.com/lanserge/revela) generates both packages —
policy and sensor — and its ISP package carries its register map, which
the panel turns into widgets. That path is the reference, not a
requirement: the slots take anyone's C.

## The one integration rule

The link must deliver bytes unmodified: **full-range RGB, no scaling, no
overscan, RGB 4:4:4**. The most common failure is a limited-range clamp
(16–235) quietly destroying sample values; the `checker` and `corners`
patterns exist to catch exactly that on day one. Details and the receiver
rate rule are in [PROTOCOL.md](PROTOCOL.md). The frame count is kept
exact too: the bridge genlocks the HDMI output to the sensor so every
captured frame leaves the cable exactly once — why and how in
[GENLOCK.md](GENLOCK.md).

## Why this exists

Built as the sensor front-end for FPGA image pipelines — for example, as the
input to hardware generated with [np2hw](https://github.com/lanserge/np2hw)
— but useful to anyone who wants real sensor data into a board without
MIPI hardware, deserialisers, or per-sensor bring-up.

## Funding

Developed independently; recurring support via
[github.com/sponsors/lanserge](https://github.com/sponsors/lanserge), or write
first: **s.rabykin@gmail.com**. Sponsorable capability targets carry the
`sponsorable` label on the issue tracker —
[Pi 5 camera support](https://github.com/bayerlink/picam2hdmi/issues/1), and
**adopt a sensor**: a one-time sponsorship puts a module on the bench and its
verified data pack in the library, with your name on the release
([IMX477](https://github.com/bayerlink/picam2hdmi/issues/2),
[IMX296 GS](https://github.com/bayerlink/picam2hdmi/issues/3),
[IMX708](https://github.com/bayerlink/picam2hdmi/issues/4),
[IMX219](https://github.com/bayerlink/picam2hdmi/issues/5)). Scope is agreed in writing before
work starts; sponsored work lands in the open tree immediately, MIT like
everything else — sponsorship buys ordering and named credit, not
exclusivity. The person behind it:
[serge.rabyking.com](https://serge.rabyking.com).

## Licence

MIT.
