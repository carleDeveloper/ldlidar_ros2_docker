#!/bin/bash
# Backup script for the Arduino UNO Q's Linux side, meant to be run (as root,
# via sudo) BEFORE an App Lab OS re-flash wipes the eMMC.
#
# Captures:
#   - The usb-role-host.service systemd unit (USB host-mode persistence)
#   - The HPS-3D160 verification script in the arduino user's home dir
#   - SSH authorized_keys (so key-based login doesn't need to be re-set-up)
#   - NetworkManager connection profiles (Wi-Fi SSID + PSK -- SENSITIVE)
#   - A snapshot of enabled systemd services and installed packages, for reference
#
# Usage (on the board): sudo ./backup_unoq_config.sh
# Produces /tmp/unoq-backup-<timestamp>.tar.gz
#
# IMPORTANT: the resulting archive contains the Wi-Fi password in plaintext
# (from /etc/NetworkManager/system-connections/). Do NOT commit it to git or
# store it anywhere public. Copy it off the board (scp) and keep it local.

set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "This script must be run as root (sudo ./backup_unoq_config.sh)." >&2
  exit 1
fi

TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
WORKDIR="$(mktemp -d)"
ARCHIVE="/tmp/unoq-backup-${TIMESTAMP}.tar.gz"
ARDUINO_HOME="/home/arduino"

echo "Staging files in ${WORKDIR} ..."

# --- Custom files we installed ---
mkdir -p "${WORKDIR}/systemd"
if [ -f /etc/systemd/system/usb-role-host.service ]; then
  cp /etc/systemd/system/usb-role-host.service "${WORKDIR}/systemd/"
fi

mkdir -p "${WORKDIR}/home"
if [ -f "${ARDUINO_HOME}/verify_hps3d160.py" ]; then
  cp "${ARDUINO_HOME}/verify_hps3d160.py" "${WORKDIR}/home/"
fi

# --- SSH access (so key-based login survives a re-flash) ---
if [ -f "${ARDUINO_HOME}/.ssh/authorized_keys" ]; then
  mkdir -p "${WORKDIR}/ssh"
  cp "${ARDUINO_HOME}/.ssh/authorized_keys" "${WORKDIR}/ssh/"
fi

# --- Wi-Fi connection profiles (SENSITIVE: contains PSK in plaintext) ---
if [ -d /etc/NetworkManager/system-connections ]; then
  mkdir -p "${WORKDIR}/network-manager"
  cp -a /etc/NetworkManager/system-connections/. "${WORKDIR}/network-manager/" 2>/dev/null || true
fi

# --- Hostname / hosts, for reference ---
mkdir -p "${WORKDIR}/etc"
cp /etc/hostname "${WORKDIR}/etc/" 2>/dev/null || true
cp /etc/hosts "${WORKDIR}/etc/" 2>/dev/null || true

# --- Informational snapshots (not restored automatically, just for reference) ---
mkdir -p "${WORKDIR}/info"
systemctl list-unit-files --state=enabled > "${WORKDIR}/info/enabled-services.txt" 2>/dev/null || true
dpkg --get-selections > "${WORKDIR}/info/dpkg-selections.txt" 2>/dev/null || true
id arduino > "${WORKDIR}/info/arduino-user-groups.txt" 2>/dev/null || true

tar -czf "${ARCHIVE}" -C "${WORKDIR}" .
rm -rf "${WORKDIR}"

chmod 600 "${ARCHIVE}"
echo "Backup written to: ${ARCHIVE}"
echo "Copy it off the board before re-flashing, e.g. from your dev machine:"
echo "  scp arduino@<board-ip>:${ARCHIVE} ./"
echo
echo "WARNING: this archive contains your Wi-Fi password in plaintext. Do not commit it to git."
