#!/bin/bash
# Build script using pre-built Docker image
# Uses Ubuntu 18.04 to match Harmony's OS version for structure size compatibility
#
# First time setup: Run ./build-docker-image.sh to build the Docker image
# Then use this script for fast builds

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER_NAME="harmony-build-$(date +%s)"
IMAGE_NAME="harmony-build:ubuntu18.04"

echo "Building harmony_ros_interface in Docker container..."

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed. Please install Docker first."
    exit 1
fi

# Check if Docker image exists
if ! docker image inspect "${IMAGE_NAME}" &> /dev/null; then
    echo "Docker image ${IMAGE_NAME} not found!"
    echo "Building it now (this is a one-time setup)..."
    "${SCRIPT_DIR}/build-docker-image.sh"
    echo ""
fi

# Get current user ID and group ID to fix permissions after build
USER_ID=$(id -u)
GROUP_ID=$(id -g)

# Build in Docker container using pre-built image
docker run --rm \
    -v "${SCRIPT_DIR}:/workspace" \
    -w /workspace \
    --name "${CONTAINER_NAME}" \
    "${IMAGE_NAME}" \
    bash -c "
        set -e
        # Use separate build directory for Docker to avoid conflicts with local builds
        rm -rf build-docker
        mkdir -p build-docker
        cd build-docker
        cmake ..
        make -j\$(nproc)
        echo ''
        echo 'Build complete! Binary is at: build-docker/application/harmony_ros_interface/harmony_ros_interface'
        echo 'Checking GLIBC requirements...'
        ldd --version | head -1
        # Fix ownership of build directory to match host user
        chown -R ${USER_ID}:${GROUP_ID} /workspace/build-docker
    "

echo ""
echo "Build finished! Binary should be compatible with Harmony"
echo "Binary location: ${SCRIPT_DIR}/build-docker/application/harmony_ros_interface/harmony_ros_interface"
