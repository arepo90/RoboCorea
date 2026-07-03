#!/usr/bin/env bash
#
# setup-usb-reset.sh — install the USB self-heal helper on the Jetson (run once).
#
# WHY: the out-of-tree WCH ch341 driver (V1.9) corrupts the CH340/tegra-xusb
# state on roughly HALF of all serial port closes (measured 2026-07-03: 4 of 7
# bridge stop/restart cycles wedged, independent of ordering, settle gaps or
# camera traffic). Once wedged, every open() gets EIO and even USBDEVFS_RESET
# says ENODEV — only re-enumerating the USB controller recovers. The bridge
# closes its ports on every service stop, so ANY restart is a coin flip.
#
# FIX (self-heal): esp32_bridge detects the wedge (persistent "[Errno 5]" on
# open) and invokes this root-owned helper via a narrow sudoers NOPASSWD rule.
# The helper re-enumerates the USB controller; the bridge's 3 s retry loop then
# reconnects on its own. The PROPER fix is replacing the WCH driver with the
# mainline ch341 module — until then this keeps restarts deterministic.
#
# Usage (on the Jetson):  sudo bash scripts/setup-usb-reset.sh
#
set -euo pipefail

HELPER_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/reset-usb-serial.sh"
HELPER_DST=/usr/local/bin/robocorea-reset-usb
SUDOERS=/etc/sudoers.d/robocorea-usb-reset
RUN_USER="${SUDO_USER:-susana-akemi}"

log() { printf '\033[1;32m[setup-usb-reset]\033[0m %s\n' "$*"; }
err() { printf '\033[1;31m[setup-usb-reset] ERROR:\033[0m %s\n' "$*" >&2; }

[ "$(id -u)" -eq 0 ] || { err "must run as root (use sudo)"; exit 1; }
[ -f "$HELPER_SRC" ] || { err "helper source not found: $HELPER_SRC"; exit 1; }

# Root-owned copy outside the user-writable repo — the sudoers rule points at
# THIS path, so a compromised user account cannot edit what runs as root.
log "installing $HELPER_DST (root-owned copy of scripts/reset-usb-serial.sh)"
install -o root -g root -m 0755 "$HELPER_SRC" "$HELPER_DST"

log "writing sudoers rule for $RUN_USER -> $HELPER_DST"
cat > "$SUDOERS" <<EOF
# RoboCorea: let the esp32_bridge node self-heal a wedged CH341/tegra-xusb
# stack (WCH driver corrupts device state on ~50%% of port closes; EIO on
# every open until USB re-enumeration). Installed by setup-usb-reset.sh.
$RUN_USER ALL=(root) NOPASSWD: $HELPER_DST
EOF
chmod 0440 "$SUDOERS"
visudo -c -f "$SUDOERS" >/dev/null || { err "sudoers validation failed"; rm -f "$SUDOERS"; exit 1; }

log "verifying: sudo -n $HELPER_DST is allowed for $RUN_USER"
sudo -u "$RUN_USER" sudo -n -l "$HELPER_DST" >/dev/null && log "OK" || err "sudo -n check failed"
log "DONE. The bridge triggers it automatically; manual use: sudo $HELPER_DST"
