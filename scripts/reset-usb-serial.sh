#!/usr/bin/env bash
#
# reset-usb-serial.sh — recover the Jetson's USB stack WITHOUT a reboot.
#
# Symptom this fixes: esp32-bridge logs
#   "/dev/ttyCH341USB0 unavailable ([Errno 5] Input/output error)"
# on every retry, lsusb still lists the CH340s, but any open() gets EIO and
# even USBDEVFS_RESET returns ENODEV. That is a wedged tegra-xusb hub state
# (seen after a hot `systemctl restart` overlapped the bridge's serial close
# with the camera streamer's UVC resets on the same hub). Previously the only
# cure was a full reboot.
#
# This unbinds + rebinds the Tegra xHCI controller, which re-enumerates EVERY
# USB device (ESP32 PCBs, cameras, sensor board). The esp32-bridge reconnects
# by itself (3 s retry loop) and the camera streamer's periodic rescan picks
# the cameras back up. If the lidar/ZED stacks are running, restart them after.
#
# Usage (on the Jetson):  sudo bash scripts/reset-usb-serial.sh
#
set -euo pipefail

DRV=/sys/bus/platform/drivers/tegra-xusb
DEV=3610000.usb

log() { printf '\033[1;32m[reset-usb-serial]\033[0m %s\n' "$*"; }
err() { printf '\033[1;31m[reset-usb-serial] ERROR:\033[0m %s\n' "$*" >&2; }

[ "$(id -u)" -eq 0 ] || { err "must run as root (use sudo)"; exit 1; }
[ -e "$DRV/$DEV" ] || { err "$DEV not bound to tegra-xusb (already unbound? wrong platform?)"; exit 1; }

log "unbinding $DEV (all USB devices will drop for a few seconds) ..."
echo "$DEV" > "$DRV/unbind"
sleep 3
log "rebinding $DEV ..."
echo "$DEV" > "$DRV/bind"

log "waiting for devices to re-enumerate ..."
for _ in $(seq 1 15); do
  sleep 1
  ls /dev/ttyCH341USB* >/dev/null 2>&1 && break
done

log "serial devices now present:"
ls -l /dev/ttyCH341USB* /dev/ttyACM* 2>&1 || true
log "DONE. esp32-bridge reconnects on its own within ~5 s; check with:"
log "  journalctl --user-unit=esp32-bridge.service -f"
