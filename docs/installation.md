# Installation and Building

This guide covers building the `harmony_ros_interface` application for the Harmony robot.

## Prerequisites

### Development Machine

```bash
sudo apt update
sudo apt install -y cmake build-essential
sudo apt install -y libwebsocketpp-dev libjsoncpp-dev libbson-dev
sudo apt install -y ros-${ROS_DISTRO}-rosbridge-suite
```

### Harmony Robot

The Harmony robot needs:
- Network connectivity to the development machine (where `rosbridge_server` runs)

**Note:** All libraries are statically linked into the `harmony_ros_interface` binary, so no separate library files need to be deployed to Harmony. Harmony does **not** need ROS2 installed.

## Building with Docker (Recommended)

Docker ensures binary compatibility with Harmony's environment (GLIBC 2.32, x86_64).

### Why Docker?

The Harmony robot runs Buildroot 2021.02 with GLIBC 2.32. Using Docker with Ubuntu 18.04 ensures:
- **GLIBC compatibility**: Ubuntu 18.04's GLIBC 2.27 produces backward-compatible binaries
- **Structure alignment**: GCC 7.5 produces compatible structure padding (184 bytes for ArmControllerState)
- **Consistent builds**: Same compiler across all development machines

### Step 1: Build Docker Image (One-Time)

```bash
./build-docker-image.sh
```

This creates the `harmony-build:ubuntu18.04` image with all build dependencies.

### Step 2: Build the Project

```bash
./build-in-docker.sh
```

The binary is output to: `build-docker/application/harmony_ros_interface/harmony_ros_interface`

### Verifying the Build

Check GLIBC requirements:
```bash
ldd build-docker/application/harmony_ros_interface/harmony_ros_interface
```

## Local Development Build

For testing on your development machine (not for Harmony deployment):

```bash
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

**Note:** Local builds may not be compatible with Harmony due to GLIBC version differences. Always use Docker for builds intended for Harmony.

## Build Output

The build process creates:

| Output | Location |
|--------|----------|
| Main application | `build-docker/application/harmony_ros_interface/harmony_ros_interface` |
| Tools | `build-docker/tools/` |
| Libraries | `build-docker/resources/` |

## Installation Path

By default, binaries are installed to: `$HOME/harmony_research/bin/`

To change this:
```bash
cmake .. -DCMAKE_INSTALL_PREFIX=/your/custom/path
make install
```

## Troubleshooting Build Issues

### Docker image not found

```bash
# Rebuild the Docker image
./build-docker-image.sh
```

### GLIBC version mismatch

If the binary fails on Harmony with GLIBC errors:
1. Use Docker build (recommended)
2. Or ensure your host GLIBC version is <= Harmony's GLIBC (2.32)

Check Harmony's GLIBC version:
```bash
./check-harmony-versions.sh
```

### Linker errors

- Ensure `libharmony_research.a` is compiled for the correct architecture (x86_64)
- Verify all dependencies are available
