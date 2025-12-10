#!/bin/bash
# Deploy harmony_ros_interface binary to Harmony robot
# Usage: ROSBRIDGE_HOST=<ip> ROSBRIDGE_PORT=<port> ./deploy-to-harmony.sh
#   ROSBRIDGE_HOST: IP address of the rosbridge server (optional, defaults to 192.168.2.2)
#                   Will be set persistently in /root/.bashrc on Harmony
#   ROSBRIDGE_PORT: Port of the rosbridge server (optional, defaults to 9090)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Default Harmony connection settings (can be overridden with environment variables)
DEFAULT_HARMONY_IP="192.168.2.1"
DEFAULT_HARMONY_USER="root"
DEFAULT_SSH_PORT="22"
DEFAULT_HARMONY_DEST_DIR="/opt/hbi/dev/bin/tools"
DEFAULT_ROSBRIDGE_PORT="9090"
DEFAULT_ROSBRIDGE_HOST="192.168.2.2"

HARMONY_IP="${HARMONY_IP:-${DEFAULT_HARMONY_IP}}"
HARMONY_USER="${HARMONY_USER:-${DEFAULT_HARMONY_USER}}"
SSH_PORT="${SSH_PORT:-${DEFAULT_SSH_PORT}}"
HARMONY_DEST_DIR="${HARMONY_DEST_DIR:-${DEFAULT_HARMONY_DEST_DIR}}"
BINARY_NAME="harmony_ros_interface"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-${DEFAULT_ROSBRIDGE_PORT}}"
ROSBRIDGE_HOST="${ROSBRIDGE_HOST:-${DEFAULT_ROSBRIDGE_HOST}}"

# Check for binary in build-docker (Docker build) or build (local build)
if [ -f "${SCRIPT_DIR}/build-docker/application/harmony_ros_interface/${BINARY_NAME}" ]; then
    BINARY_PATH="${SCRIPT_DIR}/build-docker/application/harmony_ros_interface/${BINARY_NAME}"
    echo "Using binary from Docker build: ${BINARY_PATH}"
elif [ -f "${SCRIPT_DIR}/build/application/harmony_ros_interface/${BINARY_NAME}" ]; then
    BINARY_PATH="${SCRIPT_DIR}/build/application/harmony_ros_interface/${BINARY_NAME}"
    echo "Using binary from local build: ${BINARY_PATH}"
else
    echo "Error: Binary not found!"
    echo "Please build first using:"
    echo "  ./build-in-docker.sh  (for Docker build)"
    echo "  or"
    echo "  mkdir -p build && cd build && cmake .. && make  (for local build)"
    exit 1
fi

# Verify binary exists and is executable
if [ ! -f "${BINARY_PATH}" ]; then
    echo "Error: Binary not found at ${BINARY_PATH}"
    exit 1
fi

echo ""
echo "Deploying ${BINARY_NAME} to Harmony..."
echo "  Source: ${BINARY_PATH}"
echo "  Destination: ${HARMONY_USER}@${HARMONY_IP}:${HARMONY_DEST_DIR}/${BINARY_NAME}"
echo ""

# Test connectivity first
echo "Testing connectivity to Harmony..."
if ! ping -c 1 -W 2 "${HARMONY_IP}" &> /dev/null; then
    echo "Warning: Cannot ping ${HARMONY_IP}. Check network connectivity."
    echo ""
fi

# Test connection first
echo "Testing SSH connection to Harmony..."
if ! ssh -p "${SSH_PORT}" -o ConnectTimeout=5 -o BatchMode=yes "${HARMONY_USER}@${HARMONY_IP}" "echo 'Connection successful'" 2>/dev/null; then
    echo "Warning: Could not connect to Harmony automatically."
    echo "You may need to enter your password for SSH."
    echo "If connection fails, check:"
    echo "  - SSH service is running on Harmony"
    echo "  - Firewall allows SSH (port ${SSH_PORT})"
    echo "  - Network connectivity"
    echo ""
fi

# Create destination directory if it doesn't exist
echo "Creating destination directory on Harmony..."
ssh -p "${SSH_PORT}" "${HARMONY_USER}@${HARMONY_IP}" "mkdir -p ${HARMONY_DEST_DIR}"

# Copy binary
echo "Copying binary to Harmony..."
scp -P "${SSH_PORT}" "${BINARY_PATH}" "${HARMONY_USER}@${HARMONY_IP}:${HARMONY_DEST_DIR}/${BINARY_NAME}"

# Set executable permissions
# echo "Setting executable permissions..."
# ssh -p "${SSH_PORT}" "${HARMONY_USER}@${HARMONY_IP}" "chmod +x ${HARMONY_DEST_DIR}/${BINARY_NAME}"

# Set ROSBRIDGE_HOST persistently in /root/.bashrc
echo "Setting ROSBRIDGE_HOST=${ROSBRIDGE_HOST} persistently in /root/.bashrc..."
ssh -p "${SSH_PORT}" "${HARMONY_USER}@${HARMONY_IP}" "bash -c \"
    # Remove existing ROSBRIDGE_HOST and ROSBRIDGE_PORT entries
    sed -i '/^export ROSBRIDGE_HOST=/d' /root/.bashrc
    sed -i '/^export ROSBRIDGE_PORT=/d' /root/.bashrc
    # Add new entries at the end
    echo '' >> /root/.bashrc
    echo '# Harmony ROS Interface - rosbridge connection settings' >> /root/.bashrc
    echo 'export ROSBRIDGE_HOST='${ROSBRIDGE_HOST} >> /root/.bashrc
    echo 'export ROSBRIDGE_PORT='${ROSBRIDGE_PORT} >> /root/.bashrc
\""
echo "✓ ROSBRIDGE_HOST set to ${ROSBRIDGE_HOST} in /root/.bashrc"
echo "  (Will be available in new shell sessions. Run 'source ~/.bashrc' in current session.)"

# Verify deployment
echo "Verifying deployment..."
if ssh -p "${SSH_PORT}" "${HARMONY_USER}@${HARMONY_IP}" "test -x ${HARMONY_DEST_DIR}/${BINARY_NAME}"; then
    echo ""
    echo "✓ Deployment successful!"
    echo ""
    echo "Binary deployed to: ${HARMONY_DEST_DIR}/${BINARY_NAME}"
    echo ""
    echo "ROSBRIDGE_HOST is set to: ${ROSBRIDGE_HOST}"
    echo "  (Persisted in /root/.bashrc - will be available in new shell sessions)"
    echo "  To override, run: ROSBRIDGE_HOST=<ip> ./deploy-to-harmony.sh"
    echo ""
    echo "To run on Harmony:"
    echo "  ssh ${HARMONY_USER}@${HARMONY_IP}"
    echo "  cd ${HARMONY_DEST_DIR}"
    echo "  source ~/.bashrc  # Load ROSBRIDGE_HOST (if not already loaded)"
    echo "  ./${BINARY_NAME}"
else
    echo "Error: Deployment verification failed!"
    exit 1
fi

