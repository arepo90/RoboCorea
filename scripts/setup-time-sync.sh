#!/usr/bin/env bash
#
# setup-time-sync.sh — robust time sync for the RoboCorea hosts (run once per host).
#
# WHY: the Jetson has no RTC battery, so a cold power cycle resets its clock to
# 1969 (epoch). The robot services auto-start seconds after boot — and zenoh
# sessions created while the clock is decades off NEVER stream data across the
# router-router link again, even after NTP later fixes the clock (verified
# 2026-07-03: "topics are there but nothing publishes"). systemd-timesyncd only
# syncs against internet pools, so at a venue with no internet the Jetson's
# clock stays wrong forever and no amount of power cycling helps.
#
# FIX: chrony on both hosts. The laptop (which has a battery-backed RTC and a
# sane clock) serves NTP on the robot LAN even when offline (`local` mode); the
# Jetson syncs from it within seconds of boot and steps the clock immediately
# (`makestep 1 -1`). Internet pools stay configured as a bonus when available.
# On the robot this script also enables a persistent journal (size-capped) so
# boot-time failures survive a reboot and stay diagnosable.
#
# Usage (run once per host):
#   # on the operator laptop (192.168.50.11 — the time SERVER):
#   sudo bash scripts/setup-time-sync.sh server
#   # on the Jetson (192.168.50.10 — the robot, time CLIENT):
#   sudo bash scripts/setup-time-sync.sh robot
#
# Idempotent: safe to re-run.
#
set -euo pipefail

ROLE="${1:-}"
LAPTOP_IP="192.168.50.11"
ROBOT_NET="192.168.50.0/24"
CONF_D="/etc/chrony/conf.d"
CONF="$CONF_D/robocorea.conf"

log() { printf '\033[1;32m[setup-time-sync]\033[0m %s\n' "$*"; }
err() { printf '\033[1;31m[setup-time-sync] ERROR:\033[0m %s\n' "$*" >&2; }

[ "$(id -u)" -eq 0 ] || { err "must run as root (use sudo)"; exit 1; }

# ── role (server = laptop, robot = jetson); auto-detect from the static IP ────
if [ -z "$ROLE" ]; then
  if ip -4 addr show | grep -q "inet ${LAPTOP_IP}/"; then ROLE=server
  elif ip -4 addr show | grep -q "inet 192.168.50.10/"; then ROLE=robot
  else err "cannot auto-detect role (no 192.168.50.10/.11 on this host) — pass 'server' or 'robot'"; exit 2
  fi
  log "auto-detected role: $ROLE"
fi
case "$ROLE" in server|robot) ;; *) err "role must be 'server' or 'robot', got '$ROLE'"; exit 2 ;; esac

# ── 1) install chrony (replaces systemd-timesyncd) ────────────────────────────
if ! dpkg -s chrony >/dev/null 2>&1; then
  log "installing chrony ..."
  apt-get update -qq
  apt-get install -y chrony
else
  log "chrony already installed"
fi
mkdir -p "$CONF_D"

# ── 2) role config drop-in (Ubuntu chrony.conf includes /etc/chrony/conf.d) ───
if [ "$ROLE" = server ]; then
  log "writing $CONF (LAN NTP server, serves even with no internet)"
  cat > "$CONF" <<EOF
# RoboCorea: serve NTP to the robot LAN so the Jetson can sync with no internet.
# Written by scripts/setup-time-sync.sh (role: server).
allow ${ROBOT_NET}
# Keep serving our local clock when the upstream pools are unreachable (arena).
local stratum 10
EOF
else
  log "writing $CONF (sync from the laptop ${LAPTOP_IP}, step immediately)"
  cat > "$CONF" <<EOF
# RoboCorea: the Jetson cold-boots at epoch (no RTC battery). Sync from the
# operator laptop on the robot LAN and STEP the clock at any offset, always
# (not just the first updates) — a slew would take decades to fix 1969.
# Written by scripts/setup-time-sync.sh (role: robot).
server ${LAPTOP_IP} iburst minpoll 2 maxpoll 6 prefer
makestep 1 -1
EOF
fi

systemctl enable chrony >/dev/null 2>&1 || true
systemctl restart chrony

# ── 3) robot only: persistent, size-capped journal (boot failures survive) ────
if [ "$ROLE" = robot ]; then
  log "enabling persistent journal (SystemMaxUse=200M)"
  mkdir -p /etc/systemd/journald.conf.d
  cat > /etc/systemd/journald.conf.d/robocorea.conf <<'EOF'
# RoboCorea: keep logs across reboots (volatile journal made boot-time failures
# undiagnosable) but cap disk use on the eMMC/SD.
[Journal]
Storage=persistent
SystemMaxUse=200M
EOF
  mkdir -p /var/log/journal
  systemd-tmpfiles --create --prefix /var/log/journal >/dev/null 2>&1 || true
  systemctl restart systemd-journald
  journalctl --flush 2>/dev/null || true
fi

# ── 4) report ─────────────────────────────────────────────────────────────────
sleep 2
log "chrony sources:"
chronyc sources || true
log "chrony tracking:"
chronyc tracking | head -6 || true
log "DONE ($ROLE)."
