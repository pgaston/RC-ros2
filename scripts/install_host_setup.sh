#!/bin/bash
# Install configure_system.sh as a boot service, so start_workspace.sh no longer
# asks for a password. Run once, and again after editing configure_system.sh:
#   sudo scripts/install_host_setup.sh
set -euo pipefail
if [ "$(id -u)" -ne 0 ]; then
    echo "run with sudo: sudo $0" >&2
    exit 1
fi
WORKSPACE_DIR="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"

# The service runs a root-owned copy: a root service running a file the pg user
# can edit would hand root to anything running as pg.
install -o root -g root -m 0755 "$WORKSPACE_DIR/configure_system.sh" /usr/local/sbin/rccar-host-setup
install -o root -g root -m 0644 "$WORKSPACE_DIR/rccar-host-setup.service" /etc/systemd/system/rccar-host-setup.service
if [ ! -f /etc/udev/rules.d/99-realsense-libusb.rules ]; then
    install -o root -g root -m 0644 "$WORKSPACE_DIR/99-realsense-libusb.rules" /etc/udev/rules.d/
    udevadm control --reload-rules && udevadm trigger
fi

systemctl daemon-reload
systemctl enable rccar-host-setup.service
systemctl restart rccar-host-setup.service
systemctl --no-pager status rccar-host-setup.service | head -5
