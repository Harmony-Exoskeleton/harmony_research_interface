# Dockerfile for building harmony_ros_interface with GLIBC 2.32 compatibility
# Uses Ubuntu 20.04 which has GLIBC 2.31 (compatible with Harmony's GLIBC 2.32)

FROM ubuntu:20.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies
RUN apt-get update && \
    apt-get install -y \
        build-essential \
        cmake \
        pkg-config \
        libboost-dev \
        libboost-system-dev \
        libwebsocketpp-dev \
        libjsoncpp-dev \
        libbson-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /workspace

# Default command (can be overridden)
CMD ["/bin/bash"]

