#!/bin/sh
# picam2hdmi on-Pi installer: packages, tool, instrument unit.
# Run as root on Raspberry Pi OS Lite (Bookworm or later):
#   curl -fsSL https://raw.githubusercontent.com/bayerlink/picam2hdmi/main/contrib/install.sh | sudo bash
# Idempotent: safe to re-run for upgrades.
set -eu

[ "$(id -u)" = 0 ] || { echo "run as root (sudo)"; exit 1; }

echo "== packages (picamera2 via apt: it carries the matching libcamera)"
apt-get update
apt-get install -y --no-install-recommends python3-picamera2 python3-pip curl

echo "== picam2hdmi (system Python: the root-run service imports it)"
pip3 install --break-system-packages --upgrade picam2hdmi

BIN="$(command -v picam2hdmi)"
echo "== instrument unit (ExecStart=$BIN)"
UNIT_URL="https://raw.githubusercontent.com/bayerlink/picam2hdmi/main/contrib/picam2hdmi.service"
curl -fsSL "$UNIT_URL" \
  | sed "s|^ExecStart=.*picam2hdmi |ExecStart=$BIN |" \
  > /etc/systemd/system/picam2hdmi.service
systemctl daemon-reload
systemctl enable --now picam2hdmi

sleep 2
systemctl --no-pager --lines=0 status picam2hdmi || true
echo
echo "Panel:  http://$(hostname).local:8080"
echo "Status: curl http://$(hostname).local:8080/status"
