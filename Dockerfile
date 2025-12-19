# Dockerfile for building harmony_ros_interface
# Uses Ubuntu 18.04 to ensure structure size compatibility with Harmony
#
# Harmony Environment:
#   - OS: Buildroot 2021.02
#   - GLIBC: 2.32
#   - Architecture: x86_64
#   - Expected structure size: 184 bytes (ArmControllerState)
#
# Docker Environment (Ubuntu 18.04):
#   - GLIBC: 2.27 (compatible with Harmony's 2.32 - backward compatible)
#   - GCC: 7.5 (produces compatible structure padding)
#   - Architecture: x86_64
#
# This setup ensures:
#   1. Structure sizes match (184 bytes) due to same compiler ABI
#   2. GLIBC compatibility (2.27 <= 2.32, so binaries will run on Harmony)
#   3. Same architecture (x86_64)

FROM ubuntu:18.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies
# Note: We use Ubuntu 18.04's default packages to match compiler behavior
# libboost-system-dev includes both shared and static libraries
RUN apt-get update && \
    apt-get install -y \
        build-essential \
        cmake \
        pkg-config \
        libboost-dev \
        libboost-system-dev \
        libwebsocketpp-dev \
        libjsoncpp-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /workspace

# Default command (can be overridden)
CMD ["/bin/bash"]

