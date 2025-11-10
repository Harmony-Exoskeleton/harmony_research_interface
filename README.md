# Harmony Research Interface Library

Code for the research interface library of Harmony. This library provides a ROS2 interface for the Harmony robot using rosbridge, allowing remote control and monitoring of the robot's arms.

## Overview

The `harmony_ros_interface` application runs on the Harmony robot and connects to a rosbridge server, enabling ROS2 communication over WebSocket. The application publishes joint states, arm sizes, and TF transforms, while providing services for state queries and control mode management.

**Architecture:**
- **Harmony Target Machine**: Runs `harmony_ros_interface` (cross-compiled binary)
- **Development Host Machine**: Runs ROS2 with `rosbridge_server`, used for debugging and control

## Dependencies

### On Development Machine (for building and debugging)

```bash
sudo apt update
sudo apt install -y cmake build-essential
sudo apt install -y libwebsocketpp-dev libjsoncpp-dev libbson-dev
sudo apt install -y ros-${ROS_DISTRO}-rosbridge-suite
# or
sudo apt install -y ros-${ROS_DISTRO}-rosbridge-server
```

### On Harmony Robot

The Harmony robot needs:
- Network connectivity to the development machine (where `rosbridge_server` runs)

**Note:** All libraries are statically linked into the `harmony_ros_interface` binary, so no separate library files need to be deployed to Harmony. Harmony does **not** need ROS2 installed - it only needs the binary and network access to connect to the rosbridge server running on your development machine.

## Step-by-Step Guide

### Step 1: Set Up Cross-Compilation Toolchain

The `harmony_ros_interface` application must be cross-compiled for the Harmony robot's architecture (typically ARM-based).

1. **Identify Harmony's architecture:**
   ```bash
   # SSH into Harmony and check
   ssh user@harmony  # (change as needed)
   uname -m  # Typically armv7l or aarch64
   ```

2. **Set up cross-compilation toolchain:**
   
   For ARM (armv7l):
   ```bash
   sudo apt install -y gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf
   ```
   
   For ARM64 (aarch64):
   ```bash
   sudo apt install -y gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
   ```

3. **Create a CMake toolchain file** (e.g., `toolchain-harmony.cmake`):
   
   For ARM:
   ```cmake
   set(CMAKE_SYSTEM_NAME Linux)
   set(CMAKE_SYSTEM_PROCESSOR arm)
   
   set(CMAKE_C_COMPILER arm-linux-gnueabihf-gcc)
   set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)
   
   set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
   set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
   set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
   set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
   ```
   
   For ARM64:
   ```cmake
   set(CMAKE_SYSTEM_NAME Linux)
   set(CMAKE_SYSTEM_PROCESSOR aarch64)
   
   set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
   set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
   
   set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
   set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
   set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
   set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
   ```

### Step 2: Build for Harmony

1. **Create build directory:**
   ```bash
   mkdir -p build-harmony
   cd build-harmony
   ```

2. **Configure CMake with cross-compilation:**
   ```bash
   cmake .. -DCMAKE_TOOLCHAIN_FILE=../toolchain-harmony.cmake
   ```
   
   Or if the toolchain file is in a different location:
   ```bash
   cmake .. -DCMAKE_TOOLCHAIN_FILE=/path/to/toolchain-harmony.cmake
   ```

3. **Verify the build configuration:**
   - Check that `harmony_research` library is found
   - Verify the compiler is the cross-compiler (e.g., `arm-linux-gnueabihf-g++`)

4. **Build the project:**
   ```bash
   make -j$(nproc)
   ```

5. **Install (optional, for local testing):**
   ```bash
   make install
   ```
   
   This installs to `$HOME/harmony_research/bin/` by default.

### Step 3: Deploy to Harmony via SSH

1. **Copy the binary to Harmony:**
   ```bash
   # From your development machine
   scp build-harmony/application/harmony_ros_interface/harmony_ros_interface user@harmony:~/harmony_research/bin/ # (change as needed)
   ```

2. **Make the binary executable:**
   ```bash
   ssh user@harmony  # (change as needed)
   chmod +x ~/harmony_research/bin/harmony_ros_interface
   ```

### Step 4: Start rosbridge Server on Development Machine

Before launching `harmony_ros_interface` on Harmony, start the rosbridge server on your development machine:

1. **On your development machine, start rosbridge_server:**
   ```bash
   # Set up ROS2 environment
   source /opt/ros/${ROS_DISTRO}/setup.bash
   
   # Start rosbridge server
   ros2 run rosbridge_server rosbridge_websocket --port 9090
   ```
   
   Keep this terminal open, or run it in the background:
   ```bash
   ros2 run rosbridge_server rosbridge_websocket --port 9090 &
   ```

2. **Note the development machine's IP address:**
   ```bash
   hostname -I  # or use 'ip addr' to find your IP
   ```
   
   You'll need this IP to configure Harmony to connect to it.

### Step 5: Launch on Harmony

1. **SSH into Harmony:**
   ```bash
   ssh user@harmony # (change as needed)
   ```

2. **Launch harmony_ros_interface:**
   ```bash
   cd ~/harmony_research/bin
   
   # Set connection parameters to point to your development machine
   export ROSBRIDGE_HOST=<development-machine-ip>  # Replace with your dev machine's IP
   export ROSBRIDGE_PORT=9090
   
   # Run with default 100 Hz loop frequency
   ./harmony_ros_interface
   
   # Or specify custom loop frequency (e.g., 50 Hz)
   ./harmony_ros_interface 50
   ```

   The application will:
   - Connect to the rosbridge server on your development machine
   - Start publishing joint states, sizes, and TF transforms
   - Advertise all Harmony services

### Step 6: Debug from Development Machine

From your development machine, you can interact with the Harmony robot using standard ROS2 commands.

1. **Set up ROS2 environment:**
   ```bash
   source /opt/ros/${ROS_DISTRO}/setup.bash
   ```

2. **Set ROS_DOMAIN_ID (if using ROS2 domains):**
   ```bash
   export ROS_DOMAIN_ID=0  # ensure your ROS2 environment uses the correct domain ID
   ```

3. **List available topics:**
   ```bash
   ros2 topic list
   ```
   
   You should see:
   - `/harmony/left/joint_states`
   - `/harmony/right/joint_states`
   - `/harmony/left/sizes`
   - `/harmony/right/sizes`
   - `/tf`
   - `/tf_static`

4. **List available services:**
   ```bash
   ros2 service list
   ```
   
   You should see:
   - `/harmony/get_state`
   - `/harmony/left/get_state`
   - `/harmony/right/get_state`
   - `/harmony/left/enable`
   - `/harmony/right/enable`
   - `/harmony/left/enable_harmony_mode`
   - `/harmony/left/enable_impedance_mode`
   - `/harmony/left/enable_torque_mode`
   - `/harmony/right/enable_harmony_mode`
   - `/harmony/right/enable_impedance_mode`
   - `/harmony/right/enable_torque_mode`

5. **Get robot state:**
   ```bash
   ros2 service call /harmony/get_state std_srvs/srv/Trigger
   ```

6. **Enable an arm:**
   ```bash
   ros2 service call /harmony/left/enable std_srvs/srv/SetBool "{data: true}"
   ```

7. **Set control mode:**
   ```bash
   # Set left arm to impedance mode
   ros2 service call /harmony/left/enable_impedance_mode std_srvs/srv/Trigger
   
   # Set right arm to torque mode
   ros2 service call /harmony/right/enable_torque_mode std_srvs/srv/Trigger
   ```

8. **Monitor joint states:**
   ```bash
   ros2 topic echo /harmony/left/joint_states
   ros2 topic echo /harmony/right/joint_states
   ```

9. **Visualize in RViz2:**
    ```bash
    rviz2
    ```
    
    Add displays for:
    - TF (to see robot structure)
    - Joint States (to see joint positions)
    - Topics (to visualize published data)

## Troubleshooting

### Connection Issues

**Problem:** `harmony_ros_interface` cannot connect to rosbridge.

**Solutions:**
- Verify rosbridge_server is running on your development machine: `ros2 node list` should show `/rosbridge_websocket`
- Check firewall settings on both Harmony and development machine (port 9090 must be open)
- Verify `ROSBRIDGE_HOST` and `ROSBRIDGE_PORT` environment variables point to your development machine
- Test connection from Harmony: `telnet <development-machine-ip> 9090` (should connect)

### Cross-Compilation Issues

**Problem:** Build fails with linker errors.

**Solutions:**
- Ensure `libharmony_research.a` is compiled for the correct architecture
- Verify all dependencies are available for the target architecture
- Check that the toolchain file is correctly configured

### ROS2 Discovery Issues

**Problem:** Cannot see Harmony topics/services from development machine.

**Solutions:**
- Ensure your development machine uses the correct `ROS_DOMAIN_ID` (default is 0)
- Check network connectivity from development machine: `ping harmony`
- Verify firewall allows ROS2 multicast (UDP port 7400-7500) on your development machine
- Try setting `ROS_LOCALHOST_ONLY=0` on your development machine

### Application Crashes

**Problem:** `harmony_ros_interface` crashes on startup.

**Solutions:**
- Check logs in `~/harmony_research/bin/log/harmony_ros_interface_log.csv`
- Verify `harmony_research` library is accessible
- Ensure shared memory is properly set up
- Check that the Research Interface can initialize

## Additional Information

### Application Configuration

The `harmony_ros_interface` application supports:
- **Command-line arguments:**
  - Loop frequency (Hz): `./harmony_ros_interface [frequency]`
  - Help: `./harmony_ros_interface --help`

- **Environment variables:**
  - `ROSBRIDGE_HOST`: rosbridge server hostname/IP (default: `127.0.0.1`). **On Harmony, this must be set to your development machine's IP address**
  - `ROSBRIDGE_PORT`: rosbridge server port (default: `9090`)

### Logging

Logs are written to: `~/harmony_research/bin/log/harmony_ros_interface_log.csv`

### Build Output

The build process creates:
- `harmony_ros_interface`: Main application binary (in `build/application/harmony_ros_interface/`)
- Various tools in `build-harmony/tools/`
- Libraries in `build-harmony/resources/`

### Installation Path

By default, binaries are installed to: `$HOME/harmony_research/bin/`
You can change this by setting `CMAKE_INSTALL_PREFIX` during CMake configuration.
