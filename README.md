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
| Instrument mode | **working** — `picam2hdmi serve`: HTTP control, recordings spool, a control panel with a live viewfinder crop editor (`contrib/picam2hdmi.service` for power-on) |

## Usage

Off-target, on any machine (numpy only):

```bash
pip install picam2hdmi
picam2hdmi pattern --mode counting --width 2028 --height 1078 --out frame.npy
```

That file is bit-for-bit what the HDMI link will carry — receivers can be
built and tested against it before any cable exists.

On a Pi, **[SETUP.md](SETUP.md)** is the step-by-step from blank SD card to
streaming instrument (or `contrib/install.sh` does the install in one go).
The short of it:

```bash
sudo picam2hdmi stream --source camera --exposure-us 30000 --gain 4.0
sudo picam2hdmi serve                  # instrument: panel + HTTP on :8080
```

## The one integration rule

The link must deliver bytes unmodified: **full-range RGB, no scaling, no
overscan, RGB 4:4:4**. The most common failure is a limited-range clamp
(16–235) quietly destroying sample values; the `checker` and `corners`
patterns exist to catch exactly that on day one. Details and the receiver
rate rule are in [PROTOCOL.md](PROTOCOL.md).

## Why this exists

Built as the sensor front-end for FPGA image pipelines — for example, as the
input to hardware generated with [np2hw](https://github.com/lanserge/np2hw)
— but useful to anyone who wants real sensor data into a board without
MIPI hardware, deserialisers, or per-sensor bring-up.

## Funding

Developed independently; recurring support via
[github.com/sponsors/lanserge](https://github.com/sponsors/lanserge), or write
first: **s.rabykin@gmail.com**. Sponsorable capability targets carry the
`sponsorable` label on the issue tracker — currently
[Pi 5 camera support](https://github.com/bayerlink/picam2hdmi/issues/1)
(the RP1's 16-bit raw, carried verbatim). Scope is agreed in writing before
work starts; sponsored work lands in the open tree immediately, MIT like
everything else — sponsorship buys ordering and named credit, not
exclusivity.

## Licence

MIT.
