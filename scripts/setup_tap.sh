#!/usr/bin/env bash
# Create and configure a TAP device for xv6 networking tests.
# Usage: sudo scripts/setup_tap.sh [tap-name] [host-ip/cidr]

set -euo pipefail

IFACE=${1:-xv6tap0}
HOST_CIDR=${2:-192.168.76.1/24}

if [[ $(id -u) -ne 0 ]]; then
  echo "This script must be run as root (try again with sudo)." >&2
  exit 1
fi

OWNER=${SUDO_USER:-$(logname)}

if ip link show "$IFACE" >/dev/null 2>&1; then
  echo "[setup_tap] Interface $IFACE already exists, re-configuring..."
else
  echo "[setup_tap] Creating TAP interface $IFACE owned by $OWNER"
  ip tuntap add dev "$IFACE" mode tap user "$OWNER"
fi

# Ensure no stale addresses are lingering.
ip addr flush dev "$IFACE" || true

echo "[setup_tap] Assigning $HOST_CIDR and bringing the link up"
ip addr add "$HOST_CIDR" dev "$IFACE"
ip link set "$IFACE" up

echo "[setup_tap] $IFACE is ready. Run 'make qemu-tap TAP_IFACE=$IFACE' in another shell."
