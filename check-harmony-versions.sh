#!/bin/bash
# Script to check basic version information on Harmony robot

set -e

# Default Harmony connection settings (can be overridden via environment variables)
DEFAULT_HARMONY_IP="${HARMONY_IP:-192.168.2.1}"
DEFAULT_HARMONY_USER="${HARMONY_USER:-root}"
DEFAULT_SSH_PORT="${SSH_PORT:-22}"

HARMONY_IP="${HARMONY_IP:-${DEFAULT_HARMONY_IP}}"
HARMONY_USER="${HARMONY_USER:-${DEFAULT_HARMONY_USER}}"
SSH_PORT="${SSH_PORT:-${DEFAULT_SSH_PORT}}"

echo "Connecting to Harmony at ${HARMONY_USER}@${HARMONY_IP}:${SSH_PORT}..."
echo ""

# Test connectivity first
if ! ping -c 1 -W 2 "${HARMONY_IP}" &> /dev/null; then
    echo "Warning: Cannot ping ${HARMONY_IP}. Check network connectivity."
    echo ""
fi

# Execute version checks on Harmony via SSH
ssh -p "${SSH_PORT}" "${HARMONY_USER}@${HARMONY_IP}" bash << 'REMOTE_EOF'
echo "OS Distribution:"
if command -v lsb_release &> /dev/null; then
    lsb_release -d | cut -f2
elif [ -f /etc/os-release ]; then
    if grep -q PRETTY_NAME /etc/os-release; then
        grep PRETTY_NAME /etc/os-release | cut -d'"' -f2
    else
        echo "$(grep "^NAME=" /etc/os-release | cut -d'=' -f2 | tr -d '"') $(grep "^VERSION=" /etc/os-release | cut -d'=' -f2 | tr -d '"')"
    fi
else
    echo "Unknown"
fi
echo ""

echo "Kernel Version:"
uname -r
echo ""

echo "GLIBC Version:"
ldd --version 2>/dev/null | head -1 || echo "Not available"
echo ""

echo "System Architecture:"
uname -m
echo ""
REMOTE_EOF

echo "Version check complete!"

