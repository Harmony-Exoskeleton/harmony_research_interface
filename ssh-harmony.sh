#!/bin/bash
# Quick SSH connection to Harmony robot
# Default Harmony IP: 192.168.2.2

DEFAULT_HARMONY_IP="192.168.2.1"
DEFAULT_HARMONY_USER="root"
DEFAULT_SSH_PORT="22"

HARMONY_IP="${HARMONY_IP:-${DEFAULT_HARMONY_IP}}"
HARMONY_USER="${HARMONY_USER:-${DEFAULT_HARMONY_USER}}"
SSH_PORT="${SSH_PORT:-${DEFAULT_SSH_PORT}}"

echo "Connecting to Harmony at ${HARMONY_USER}@${HARMONY_IP}:${SSH_PORT}..."

# Test connectivity first
echo "Testing connectivity..."
if ! ping -c 1 -W 2 "${HARMONY_IP}" &> /dev/null; then
    echo "Warning: Cannot ping ${HARMONY_IP}. Check network connectivity."
    echo ""
fi

# Try SSH connection
ssh -p "${SSH_PORT}" "${HARMONY_USER}@${HARMONY_IP}"

