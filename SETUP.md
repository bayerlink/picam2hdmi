# Setup: from blank SD card to streaming instrument

Every step here was run as written on a Raspberry Pi 3B with an OV5647
camera module. Other Pis and cameras differ only where noted.

## What you need

- A Raspberry Pi (3B or later) and its power supply
- A camera module on the CSI ribbon (optional — patterns work without one)
- An SD card (8 GB is plenty)
- An HDMI cable to your receiver: an FPGA board's HDMI-in, or a USB
  capture stick for the no-FPGA bench (see the
  [bayerlink guide](https://github.com/bayerlink/bayerlink/blob/main/GUIDE.md))

## 1. Flash the card

Use Raspberry Pi Imager with **Raspberry Pi OS Lite** (64-bit,
Bookworm or later). Lite matters: the streamer must own the display,
so there must be no desktop session.

In the Imager's settings (the gear icon), before writing:

- set a hostname (this guide uses `picam`),
- enable SSH with your public key,
- configure your Wi-Fi (or plan to use Ethernet).

Boot the Pi and confirm you can reach it:

```sh
ssh pi@picam.local
```

## 2. Install

On the Pi:

```sh
sudo apt update
sudo apt install -y --no-install-recommends \
    python3-picamera2 python3-pip libdrm-dev
sudo pip3 install --break-system-packages picam2hdmi
```

`python3-picamera2` comes from apt, not pip — it drags in the matching
libcamera for your OS release. The `--break-system-packages` flag is
Bookworm's required acknowledgement for installing into the system
Python, which is where a root-run service needs it.

The other two are named because `--no-install-recommends` means nothing
else will bring them. Lite ships no pip at all. And the bridge is compiled
on the device it runs on, against libdrm — without the headers the build
stops with `pkg-config cannot find libdrm`, and it stops at first stream
rather than here, which is a long way from the command that caused it.

Or run the same three commands via the installer, which also installs
and enables the systemd unit from step 4:

```sh
curl -fsSL https://raw.githubusercontent.com/bayerlink/picam2hdmi/main/contrib/install.sh | sudo bash
```

(Read it first; it is short. Piping a URL into root shell is a
convenience for YOUR bench, not a habit.)

## 3. First light: a pattern

With the HDMI cable connected to your receiver:

```sh
sudo picam2hdmi stream --source pattern --pattern counting \
    --width 512 --height 240 --bayer RGGB --mode 1920x1080@30
```

The receiver side judges the link — `bayertap check` on Linux, or the
capture-stick path in the bayerlink guide. `sudo` is for DRM master
(owning the display); the `video` group works too.

With a camera attached:

```sh
sudo picam2hdmi stream --source camera
```

The sensor's own auto-exposure, auto-gain and auto-white-balance are
turned off: a raw source should be deterministic, and the ISP downstream
is what decides those. The VALUES are not set here either — they belong
to the sensor package, where the AE loop lives. Without one installed the
sensor keeps whatever it was last left at, so a near-black or a blown
frame means nobody has set it yet, not that anything is broken.

## 4. Instrument mode

For a permanent bench, run the daemon instead of the CLI:

```sh
sudo cp contrib/picam2hdmi.service /etc/systemd/system/
sudo systemctl enable --now picam2hdmi
```

From power-on the Pi now streams the default pattern and listens on
port 8080:

- **Panel**: http://picam.local:8080 — source tabs, live camera view in
  a draggable crop box (release applies it, no restart), recordings.
- **API**: `GET /status`, `PUT /source`, `GET|PUT|DELETE /recordings/…`,
  `GET /preview.png`, `GET|PUT|DELETE /modules/<slot>`, `PUT /power` —
  same refusals as the CLI, as HTTP 400s; the full list is in
  `serve.py`'s docstring.

```sh
curl http://picam.local:8080/status
curl -X PUT http://picam.local:8080/source \
     -d '{"source":"camera","crop":[600,366,96,240]}'
```

A LAN instrument, not an internet service: it binds 0.0.0.0 and has no
authentication — firewall the port if the LAN is not yours.

The instrument **retains state**: the last accepted source survives a
power cycle, so configure the camera once in the panel and from then on
the Pi boots straight into it — a camera with a power plug. The panel's
**Power off** button shuts the Pi down cleanly (SD cards prefer that to
pulled plugs); switching power back on brings everything back.

## 5. If something misbehaves

- **`bridge: no display connected yet`** — the HDMI sink is not up yet
  (the board downstream may boot after the Pi). The bridge waits five
  minutes for it, then refuses, and the unit ends `failed`.
- **`main.py: DRM master` / modeset errors** — a desktop session owns
  the display. Use OS Lite, or stop the display manager.
- **Near-black or blown camera frames** — the sensor's own AE and AGC are
  off by design, and this package does not set exposure or gain: a sensor
  package owns them. With none installed the sensor keeps whatever it was
  last left at.
- **Refusals** (geometry, rate rule) — the message names
  the limit and the fix; they are the tool working, not failing.
- **Capture stick shows a frozen frame** after the source switches
  modes — the stick replays its last frame while re-locking; wait a
  few seconds or skip the first grabbed frames.
