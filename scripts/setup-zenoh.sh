#!/usr/bin/env bash
#
# setup-zenoh.sh — configure rmw_zenoh as the ROS 2 middleware on this host.
#
# Sets up a per-host Zenoh router (rmw_zenohd) running as a systemd *system*
# service, configured for UNICAST TCP to the peer host (multicast disabled,
# gossip on, auto-retry) — reliable on congested Wi-Fi. ROS nodes on this host
# are Zenoh peers that connect only to the local router (localhost:7447); the
# two routers federate discovery over a single TCP link.
#
# RoboCorea hosts (static IPs, NetworkManager ipv4.method manual):
#   Jetson  192.168.50.10   |   Workstation laptop  192.168.50.11
#
# Usage (run once per host):
#   # on the Jetson  (peer = the laptop):
#   sudo PEER_IP=192.168.50.11 bash scripts/setup-zenoh.sh
#   # on the laptop  (peer = the Jetson):
#   sudo PEER_IP=192.168.50.10 bash scripts/setup-zenoh.sh
#
# Idempotent: safe to re-run (e.g. after re-imaging or to repoint the peer).
#
set -euo pipefail

PEER_IP="${PEER_IP:?set PEER_IP=<other host static IP>}"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
DOMAIN="${ROS_DOMAIN_ID:-20}"

CFG_DIR="/etc/rmw_zenoh"
ROUTER_CFG="${CFG_DIR}/router_config.json5"
WRAPPER="/usr/local/bin/rmw-zenoh-router"
UNIT="/etc/systemd/system/rmw-zenoh-router.service"
PROFILE_D="/etc/profile.d/ros-rmw-zenoh.sh"
DEFAULT_ROUTER_CFG="/opt/ros/${ROS_DISTRO_NAME}/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5"

log() { printf '\033[1;32m[setup-zenoh]\033[0m %s\n' "$*"; }
err() { printf '\033[1;31m[setup-zenoh] ERROR:\033[0m %s\n' "$*" >&2; }

[ "$(id -u)" -eq 0 ] || { err "must run as root (use sudo)"; exit 1; }
[ -f "$ROS_SETUP" ] || { err "no ROS at $ROS_SETUP"; exit 1; }

# ── 1) ensure rmw_zenoh_cpp is installed ──────────────────────────────────────
if ! dpkg -s "ros-${ROS_DISTRO_NAME}-rmw-zenoh-cpp" >/dev/null 2>&1; then
  log "installing ros-${ROS_DISTRO_NAME}-rmw-zenoh-cpp ..."
  apt-get update -qq
  apt-get install -y "ros-${ROS_DISTRO_NAME}-rmw-zenoh-cpp"
else
  log "ros-${ROS_DISTRO_NAME}-rmw-zenoh-cpp already installed"
fi
[ -f "$DEFAULT_ROUTER_CFG" ] || { err "default router config missing: $DEFAULT_ROUTER_CFG"; exit 1; }

# ── 2) router config: default + peer connect endpoint (unicast) ───────────────
log "writing $ROUTER_CFG (peer = tcp/${PEER_IP}:7447)"
mkdir -p "$CFG_DIR"
PEER_IP="$PEER_IP" python3 - "$DEFAULT_ROUTER_CFG" "$ROUTER_CFG" <<'PY'
import os, sys
src_path, dst_path = sys.argv[1], sys.argv[2]
peer = os.environ["PEER_IP"]
s = open(src_path).read()
anchor = '      // "<proto>/<address>"'
if s.count(anchor) != 1:
    sys.exit("anchor for connect.endpoints not found uniquely in default config; aborting")
inject = '      "tcp/%s:7447"   // RoboCorea: peer Zenoh router (unicast, multicast disabled)' % peer
out = s.replace(anchor, inject, 1)
header = ("// RoboCorea rmw_zenoh ROUTER config.\n"
          "// Generated from DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5 by setup-zenoh.sh.\n"
          "// Only change vs default: connect.endpoints points at the peer router for\n"
          "// a unicast TCP link. Multicast is already disabled in the default.\n")
open(dst_path, "w").write(header + out)
PY
chmod 0644 "$ROUTER_CFG"
grep -q "tcp/${PEER_IP}:7447" "$ROUTER_CFG" || { err "peer endpoint not written"; exit 1; }

# ── 3) router launch wrapper ──────────────────────────────────────────────────
log "writing $WRAPPER"
cat > "$WRAPPER" <<EOF
#!/usr/bin/env bash
# RoboCorea: launch the Zenoh router (rmw_zenohd) with the unicast config.
set -uo pipefail
set +u; source "${ROS_SETUP}"; set -u
export ZENOH_ROUTER_CONFIG_URI="\${ZENOH_ROUTER_CONFIG_URI:-${ROUTER_CFG}}"
exec "/opt/ros/${ROS_DISTRO_NAME}/lib/rmw_zenoh_cpp/rmw_zenohd"
EOF
chmod 0755 "$WRAPPER"

# ── 4) systemd system service (autostart at boot) ─────────────────────────────
log "writing $UNIT"
cat > "$UNIT" <<EOF
[Unit]
Description=Zenoh router for ROS 2 (rmw_zenoh) — RoboCorea
Documentation=https://github.com/ros2/rmw_zenoh
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
Environment=ROS_DOMAIN_ID=${DOMAIN}
Environment=RMW_IMPLEMENTATION=rmw_zenoh_cpp
Environment=ZENOH_ROUTER_CONFIG_URI=${ROUTER_CFG}
ExecStart=${WRAPPER}
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable rmw-zenoh-router.service >/dev/null
systemctl restart rmw-zenoh-router.service

# ── 5) make rmw_zenoh the default RMW for every ROS process on this host ───────
log "writing $PROFILE_D (RMW_IMPLEMENTATION for login shells + bash -lc services)"
cat > "$PROFILE_D" <<'EOF'
# RoboCorea: use Zenoh as the ROS 2 middleware (set for all login shells and
# for the systemd --user services that launch via `bash -lc`).
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
EOF
chmod 0644 "$PROFILE_D"

# also add to the invoking user's ~/.bashrc for interactive (non-login) terminals
REAL_USER="${SUDO_USER:-}"
if [ -n "$REAL_USER" ]; then
  USER_HOME="$(getent passwd "$REAL_USER" | cut -d: -f6)"
  BRC="$USER_HOME/.bashrc"
  if [ -f "$BRC" ] && ! grep -q "RMW_IMPLEMENTATION=rmw_zenoh_cpp" "$BRC"; then
    printf '\n# RoboCorea: use Zenoh as the ROS 2 middleware\nexport RMW_IMPLEMENTATION=rmw_zenoh_cpp\n' >> "$BRC"
    log "appended RMW_IMPLEMENTATION export to $BRC"
  else
    log "$BRC already exports RMW_IMPLEMENTATION (or not found) — skipped"
  fi
fi

# ── 6) report ─────────────────────────────────────────────────────────────────
sleep 1
log "service status:"
systemctl --no-pager --lines=8 status rmw-zenoh-router.service || true
log "listening sockets on :7447 ->"
( ss -ltnp 2>/dev/null | grep ':7447' ) || err "nothing listening on 7447 yet"
log "DONE. RMW_IMPLEMENTATION=rmw_zenoh_cpp, router peer = tcp/${PEER_IP}:7447"
