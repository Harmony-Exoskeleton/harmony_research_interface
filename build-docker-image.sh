#!/bin/bash
# Script to build the Docker image for harmony_ros_interface
# Run this once (or when dependencies change) to create the build image

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="harmony-build"
IMAGE_TAG="glibc2.32"

echo "Building Docker image for harmony_ros_interface..."
echo "Image: ${IMAGE_NAME}:${IMAGE_TAG}"

docker build -t "${IMAGE_NAME}:${IMAGE_TAG}" -f "${SCRIPT_DIR}/Dockerfile" "${SCRIPT_DIR}"

echo ""
echo "Docker image built successfully!"
echo "You can now use ./build-in-docker.sh for faster builds"

