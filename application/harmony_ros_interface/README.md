# Harmony ROS Interface - Usage Manual

This manual provides a step-by-step guide for using the `harmony_ros_interface` system to control and monitor the Harmony robot arms via ROS2.

## Overview

The `harmony_ros_interface` application runs on the Harmony robot and connects to a rosbridge server on your development machine, enabling ROS2 communication over WebSocket. The system provides comprehensive control and monitoring capabilities for both Harmony robot arms.

## Features

### Control Modes

The system supports three control modes that determine how the robot arms respond to commands:

#### 1. **Harmony Mode** (Default)
- **Description**: The robot operates under Harmony's default control system. Research interface commands are ignored.
- **Use Case**: Normal robot operation, safety mode when not actively controlling the robot.
- **Enable**: This is the default mode. To return to it, disable override mode.
- **Service**: `/harmony/{left|right}/disable_override_mode`

#### 2. **Torque Mode**
- **Description**: Direct torque control mode. Only torque commands are accepted; stiffness and position commands are rejected.
- **Use Case**: Pure torque control for force-based manipulation, impedance learning, or torque-based control algorithms.
- **Enable**: 
  ```bash
  ros2 service call /harmony/{left|right}/enable_torque_mode std_srvs/srv/Trigger
  ```
- **Accepted Commands**: `/harmony/{left|right}/desired_torque` only
- **Rejected Commands**: `/harmony/{left|right}/desired_stiffness`, `/harmony/{left|right}/desired_position`

#### 3. **Impedance Mode**
- **Description**: Full impedance control mode. Allows simultaneous control of torque, stiffness (spring constant), and position. The arm behaves like a spring-damper system.
- **Use Case**: Compliant manipulation, position control with adjustable stiffness, hybrid force/position control.
- **Enable**:
  ```bash
  ros2 service call /harmony/{left|right}/enable_impedance_mode std_srvs/srv/Trigger
  ```
- **Accepted Commands**: All three command topics:
  - `/harmony/{left|right}/desired_torque` - Torque in Nm
  - `/harmony/{left|right}/desired_stiffness` - Stiffness in Nm/rad
  - `/harmony/{left|right}/desired_position` - Position in radians
- **Behavior**: Commands can be updated independently. Updating one field preserves the others.

### Arm Features

These features can be enabled or disabled independently of the control mode:

#### 1. **Gravity Compensation**
- **Description**: Automatically compensates for gravitational forces acting on the arm, reducing the torque required to hold the arm in position.
- **Use Case**: Essential for torque/impedance control to prevent the arm from falling under its own weight.
- **Enable/Disable**:
  ```bash
  # Enable gravity compensation
  ros2 service call /harmony/{left|right}/enable_gravity std_srvs/srv/SetBool "{data: true}"
  
  # Disable gravity compensation
  ros2 service call /harmony/{left|right}/enable_gravity std_srvs/srv/SetBool "{data: false}"
  ```
- **Recommendation**: Always enable gravity compensation when using torque or impedance mode.

#### 2. **Scapulo-Humeral Rhythm (SHR)**
- **Description**: Enables the natural coupling between shoulder blade (scapula) and upper arm (humerus) movement, mimicking human arm kinematics.
- **Use Case**: More natural arm movements, biomechanically accurate motion.
- **Enable/Disable**:
  ```bash
  # Enable SHR
  ros2 service call /harmony/{left|right}/enable_shr std_srvs/srv/SetBool "{data: true}"
  
  # Disable SHR
  ros2 service call /harmony/{left|right}/enable_shr std_srvs/srv/SetBool "{data: false}"
  ```

#### 3. **Constraints**
- **Description**: Enables joint limit constraints and kinematic constraints to prevent unsafe configurations.
- **Use Case**: Safety feature to prevent the arm from reaching joint limits or singular configurations.
- **Enable/Disable**:
  ```bash
  # Enable constraints
  ros2 service call /harmony/{left|right}/enable_constraints std_srvs/srv/SetBool "{data: true}"
  
  # Disable constraints
  ros2 service call /harmony/{left|right}/enable_constraints std_srvs/srv/SetBool "{data: false}"
  ```
- **Note**: Constraints are only relevant when in `jointsOverride` mode (impedance or torque mode).

### Monitoring Features

#### 1. **Joint States**
- **Description**: Real-time joint position, velocity, and effort (torque) for all 7 joints of each arm.
- **Topic**: `/harmony/{left|right}/joint_states`
- **Message Type**: `sensor_msgs/JointState`
- **Update Rate**: Configurable (default 100 Hz)
- **Usage**:
  ```bash
  # Monitor left arm joint states
  ros2 topic echo /harmony/left/joint_states
  
  # Monitor right arm joint states
  ros2 topic echo /harmony/right/joint_states
  ```

#### 2. **Arm Sizes**
- **Description**: Real-time measurements of arm segment lengths (in mm) for kinematic calculations.
- **Topic**: `/harmony/{left|right}/sizes`
- **Message Type**: `std_msgs/Float64MultiArray`
- **Update Rate**: Configurable (default 100 Hz)
- **Usage**:
  ```bash
  ros2 topic echo /harmony/left/sizes
  ros2 topic echo /harmony/right/sizes
  ```

#### 3. **End-Effector Poses (TF)**
- **Description**: Transform frames (TF) for end-effector positions and orientations in 3D space.
- **Topics**: `/tf`, `/tf_static`
- **Frames**:
  - `map` → `base_link` (static transform, identity)
  - `base_link` → `left_end_effector` (dynamic, updated at loop rate)
  - `base_link` → `right_end_effector` (dynamic, updated at loop rate)
- **Usage**: Visualize in RViz2 or query with `tf2_ros`:
  ```bash
  ros2 run tf2_ros tf2_echo base_link left_end_effector
  ```

#### 4. **State Queries**
- **Description**: Query current control mode, feature states, and arm configuration.
- **Services**:
  - `/harmony/get_state` - Get state of both arms
  - `/harmony/left/get_state` - Get state of left arm only
  - `/harmony/right/get_state` - Get state of right arm only
- **Usage**:
  ```bash
  ros2 service call /harmony/left/get_state std_srvs/srv/Trigger
  ```

### Command Topics

#### 1. **Torque Commands**
- **Topic**: `/harmony/{left|right}/desired_torque`
- **Message Type**: `std_msgs/Float64MultiArray`
- **Format**: Array of 7 values (one per joint) in Newton-meters (Nm)
- **Valid Modes**: Torque mode, Impedance mode
- **Example**:
  ```bash
  # Send zero torque to all joints
  ros2 topic pub /harmony/left/desired_torque std_msgs/msg/Float64MultiArray \
    "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
  
  # Continuous control at 100 Hz
  ros2 topic pub -r 100 /harmony/left/desired_torque std_msgs/msg/Float64MultiArray \
    "{data: [1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0]}"
  ```

#### 2. **Stiffness Commands**
- **Topic**: `/harmony/{left|right}/desired_stiffness`
- **Message Type**: `std_msgs/Float64MultiArray`
- **Format**: Array of 7 values (one per joint) in Nm/rad
- **Valid Modes**: Impedance mode only
- **Example**:
  ```bash
  # Set medium stiffness (10 Nm/rad per joint)
  ros2 topic pub /harmony/left/desired_stiffness std_msgs/msg/Float64MultiArray \
    "{data: [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]}"
  ```

#### 3. **Position Commands**
- **Topic**: `/harmony/{left|right}/desired_position`
- **Message Type**: `std_msgs/Float64MultiArray`
- **Format**: Array of 7 values (one per joint) in radians
- **Valid Modes**: Impedance mode only
- **Example**:
  ```bash
  # Move to a specific joint configuration
  ros2 topic pub /harmony/left/desired_position std_msgs/msg/Float64MultiArray \
    "{data: [0.1, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0]}"
  ```

### Utility Services

#### Reset Shared Memory
- **Service**: `/harmony/reset_shared_memory`
- **Description**: Removes all shared memory segments used by the research interface. Useful for troubleshooting or after crashes.
- **Usage**:
  ```bash
  ros2 service call /harmony/reset_shared_memory std_srvs/srv/Trigger
  ```
- **Warning**: This will disconnect any active research interface connections. Use with caution.

## System Architecture

```
┌─────────────────────┐         WebSocket          ┌──────────────────────┐
│  Development        │◄──────────────────────────►│  Harmony Robot       │
│  Machine            │         (Port 9090)        │                      │
│                     │                            │                      │
│  - ROS2             │                            │  - harmony_ros_      │
│  - rosbridge_server │                            │    interface         │
│  - Your ROS nodes   │                            │  - Research Interface│
└─────────────────────┘                            └──────────────────────┘
```

## Prerequisites

Before starting, ensure:

1. **Development Machine:**
   - ROS2 installed and sourced
   - `rosbridge_server` package installed
   - Network connectivity to Harmony robot

2. **Harmony Robot:**
   - `harmony_ros_interface` binary deployed
   - `ROSBRIDGE_HOST` environment variable configured (points to development machine IP)
   - Network connectivity to development machine

## Usage Sequence

### Step 1: Start rosbridge Server on Development Machine

The rosbridge server must be running **before** launching `harmony_ros_interface` on Harmony.

```bash
# On your development machine
source /opt/ros/${ROS_DISTRO}/setup.bash

# Start rosbridge server (default port 9090)
ros2 run rosbridge_server rosbridge_websocket --port 9090
```

**Keep this terminal open** or run it in the background:
```bash
ros2 run rosbridge_server rosbridge_websocket --port 9090 &
```

**Verify it's running:**
```bash
ros2 node list  # Should show /rosbridge_websocket
```

### Step 2: Launch harmony_ros_interface on Harmony

SSH into Harmony and launch the interface:

```bash
# SSH into Harmony
ssh user@harmony

# Navigate to binary location (default deployment path)
cd /opt/hbi/dev/bin/tools

# Load environment variables (if configured during deployment)
source ~/.bashrc

# Or manually set connection parameters
export ROSBRIDGE_HOST=<development-machine-ip>  # e.g., 192.168.2.2
export ROSBRIDGE_PORT=9090

# Launch with default 100 Hz loop frequency
./harmony_ros_interface

# Or specify custom frequency (e.g., 50 Hz)
./harmony_ros_interface 50
```

**Expected output:**
```
Harmony ROS Interface starting...
Loop frequency: 100 Hz
Research Interface initialized successfully
Connecting to ROSBridge server at 192.168.2.2:9090
Successfully connected to ROSBridge server!
Created joint state publishers...
Created size publishers...
Created TF broadcaster...
Setting up services...
Setting up joint command subscribers...
```

### Step 3: Verify Connection from Development Machine

From your development machine, verify the connection:

```bash
# Source ROS2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# List available topics
ros2 topic list

# You should see:
# /harmony/left/joint_states
# /harmony/right/joint_states
# /harmony/left/sizes
# /harmony/right/sizes
# /harmony/left/desired_torque
# /harmony/right/desired_torque
# /harmony/left/desired_stiffness
# /harmony/right/desired_stiffness
# /harmony/left/desired_position
# /harmony/right/desired_position
# /tf
# /tf_static

# List available services
ros2 service list

# You should see all Harmony services (get_state, enable_*, etc.)
```

### Step 4: Monitor Robot State

**View joint states:**
```bash
# Monitor left arm joint states
ros2 topic echo /harmony/left/joint_states

# Monitor right arm joint states
ros2 topic echo /harmony/right/joint_states
```

**Get current robot state:**
```bash
# Get state of both arms
ros2 service call /harmony/get_state std_srvs/srv/Trigger

# Get state of left arm only
ros2 service call /harmony/left/get_state std_srvs/srv/Trigger

# Get state of right arm only
ros2 service call /harmony/right/get_state std_srvs/srv/Trigger
```

**View arm sizes:**
```bash
ros2 topic echo /harmony/left/sizes
ros2 topic echo /harmony/right/sizes
```

### Step 5: Configure Control Mode

Before sending joint commands, you must set the arm to the appropriate control mode.

#### Option A: Enable Impedance Mode

Impedance mode allows control of torque, stiffness, and position simultaneously:

```bash
# Enable impedance mode for left arm
ros2 service call /harmony/left/enable_impedance_mode std_srvs/srv/Trigger

# Enable impedance mode for right arm
ros2 service call /harmony/right/enable_impedance_mode std_srvs/srv/Trigger
```

#### Option B: Enable Torque Mode

Torque mode allows only torque control:

```bash
# Enable torque mode for left arm
ros2 service call /harmony/left/enable_torque_mode std_srvs/srv/Trigger

# Enable torque mode for right arm
ros2 service call /harmony/right/enable_torque_mode std_srvs/srv/Trigger
```

#### Return to Harmony Mode

To return the arm to Harmony's default control mode:

```bash
# Disable override mode (returns to Harmony mode)
ros2 service call /harmony/left/disable_override_mode std_srvs/srv/Trigger
ros2 service call /harmony/right/disable_override_mode std_srvs/srv/Trigger
```

### Step 6: Configure Arm Features (Optional)

Configure additional arm features before or during control. For detailed information about each feature, see the [Arm Features](#arm-features) section above.

**Gravity Compensation** (Recommended for torque/impedance mode):
```bash
# Enable gravity compensation for left arm
ros2 service call /harmony/left/enable_gravity std_srvs/srv/SetBool "{data: true}"

# Disable gravity compensation
ros2 service call /harmony/left/enable_gravity std_srvs/srv/SetBool "{data: false}"
```

**Scapulo-Humeral Rhythm (SHR):**
```bash
# Enable SHR for left arm
ros2 service call /harmony/left/enable_shr std_srvs/srv/SetBool "{data: true}"

# Disable SHR
ros2 service call /harmony/left/enable_shr std_srvs/srv/SetBool "{data: false}"
```

**Constraints:**
```bash
# Enable constraints for left arm
ros2 service call /harmony/left/enable_constraints std_srvs/srv/SetBool "{data: true}"

# Disable constraints
ros2 service call /harmony/left/enable_constraints std_srvs/srv/SetBool "{data: false}"
```

**Note:** Apply the same commands to `/harmony/right/` for the right arm.

### Step 7: Send Joint Commands

Once the arm is in the correct control mode, you can send commands via ROS topics. For detailed information about command topics, see the [Command Topics](#command-topics) section above.

#### Torque Commands

Works in both **torque mode** and **impedance mode**:

```bash
# Send torque commands to left arm (7 values in Nm, one per joint)
ros2 topic pub /harmony/left/desired_torque std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# Example: Apply 1.0 Nm to first joint, 0.5 Nm to second joint
ros2 topic pub /harmony/left/desired_torque std_msgs/msg/Float64MultiArray \
  "{data: [1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

**For continuous control**, publish at a regular rate:
```bash
# Publish torque commands at 100 Hz
ros2 topic pub -r 100 /harmony/left/desired_torque std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

#### Stiffness Commands

**Only works in impedance mode:**

```bash
# Send stiffness commands to left arm (7 values in Nm/rad)
ros2 topic pub /harmony/left/desired_stiffness std_msgs/msg/Float64MultiArray \
  "{data: [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]}"

# For continuous control
ros2 topic pub -r 100 /harmony/left/desired_stiffness std_msgs/msg/Float64MultiArray \
  "{data: [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]}"
```

#### Position Commands

**Only works in impedance mode:**

```bash
# Send position commands to left arm (7 values in radians)
ros2 topic pub /harmony/left/desired_position std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# For continuous control
ros2 topic pub -r 100 /harmony/left/desired_position std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

**Important Notes:**
- In **impedance mode**: Torque, stiffness, and position can be updated independently. Updating one field preserves the others.
- In **torque mode**: Only torque commands are accepted. Stiffness and position commands are rejected.
- All fields are reset when switching control modes.
- Commands must contain exactly 7 values (one per joint).

### Step 8: Visualize in RViz2 (Optional)

Visualize the robot state in RViz2:

```bash
# Launch RViz2
rviz2
```

**Add displays:**
1. **TF**: Shows robot structure and end-effector poses
2. **Joint States**: Visualizes joint positions
3. **Topics**: Monitor published data

**Configure TF:**

The system publishes transforms from `map` → `base_link` → `left_end_effector` / `right_end_effector`

**If you see a "missing map" error in RViz2, you have two options:**

**Option 1: Change RViz2 Fixed Frame (Recommended - Quick Fix)**
1. In RViz2, look at the "Global Options" panel on the left
2. Find the "Fixed Frame" dropdown (usually shows "map" by default)
3. Change it from `map` to `base_link`
4. The error should disappear immediately

**Option 2: Wait for Static Transform (Automatic)**
- The code automatically publishes the `map` → `base_link` static transform via `/tf_static`
- Static transforms are republished every 5 seconds for reliability
- If RViz2 still shows an error, try:
  1. Click "Reset" in RViz2 (or restart RViz2)
  2. Ensure the `harmony_ros_interface` application is running
  3. Check that `/tf_static` topic is available: `ros2 topic list | grep tf_static`
  4. Verify the transform: `ros2 run tf2_ros tf2_echo map base_link`

**Note:** The static transform publishes `map` as the root frame (standard ROS convention). If you prefer to use `base_link` as the fixed frame, Option 1 is the simplest solution.

## Complete Workflow Example

Here's a complete example workflow for controlling the left arm:

```bash
# 1. Start rosbridge (on development machine)
ros2 run rosbridge_server rosbridge_websocket --port 9090 &

# 2. Launch harmony_ros_interface (on Harmony)
# (SSH into Harmony and run)
export ROSBRIDGE_HOST=192.168.2.2
./harmony_ros_interface

# 3. Verify connection (on development machine)
ros2 topic list
ros2 service list

# 4. Get current state
ros2 service call /harmony/left/get_state std_srvs/srv/Trigger

# 5. Enable impedance mode
ros2 service call /harmony/left/enable_impedance_mode std_srvs/srv/Trigger

# 6. Enable gravity compensation
ros2 service call /harmony/left/enable_gravity std_srvs/srv/SetBool "{data: true}"

# 7. Set stiffness
ros2 topic pub -r 100 /harmony/left/desired_stiffness std_msgs/msg/Float64MultiArray \
  "{data: [20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0]}"

# 8. Set position (arm will move to this position)
ros2 topic pub -r 100 /harmony/left/desired_position std_msgs/msg/Float64MultiArray \
  "{data: [0.1, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# 9. Monitor joint states
ros2 topic echo /harmony/left/joint_states

# 10. Return to Harmony mode when done
ros2 service call /harmony/left/disable_override_mode std_srvs/srv/Trigger
```

## Control Mode Behavior

For detailed information about control modes, see the [Features](#control-modes) section above.

### Quick Reference

**Impedance Mode:**
- **Torque commands**: ✅ Accepted (updates torque, preserves stiffness and position)
- **Stiffness commands**: ✅ Accepted (updates stiffness, preserves torque and position)
- **Position commands**: ✅ Accepted (updates position, preserves torque and stiffness)
- **Use case**: Full control with spring-like behavior

**Torque Mode:**
- **Torque commands**: ✅ Accepted
- **Stiffness commands**: ❌ Rejected
- **Position commands**: ❌ Rejected
- **Use case**: Direct torque control without position/stiffness

**Harmony Mode (Default):**
- **All commands**: ❌ Rejected (arm controlled by Harmony's default controller)
- **Use case**: Normal robot operation

## Troubleshooting

### Connection Issues

**Problem:** `harmony_ros_interface` cannot connect to rosbridge.

**Solutions:**
1. Verify rosbridge is running: `ros2 node list` should show `/rosbridge_websocket`
2. Check firewall: Port 9090 must be open on both machines
3. Verify `ROSBRIDGE_HOST` is set correctly on Harmony
4. Test connectivity: From Harmony, run `telnet <dev-machine-ip> 9090`

### Commands Not Working

**Problem:** Joint commands are not affecting the robot.

**Solutions:**
1. Verify arm is in correct control mode:
   ```bash
   ros2 service call /harmony/left/get_state std_srvs/srv/Trigger
   ```
2. Ensure you're using the correct topic for the mode:
   - Torque mode: Only `/desired_torque` works
   - Impedance mode: All three topics work
3. Check that commands contain exactly 7 values
4. Verify `harmony_ros_interface` is running and connected

### ROS2 Discovery Issues

**Problem:** Cannot see Harmony topics/services from development machine.

**Solutions:**
1. Ensure `ROS_DOMAIN_ID` matches (default is 0)
2. Check network connectivity: `ping <harmony-ip>`
3. Verify firewall allows ROS2 multicast (UDP ports 7400-7500)
4. Try setting `ROS_LOCALHOST_ONLY=0` on development machine

### Application Crashes

**Problem:** `harmony_ros_interface` crashes on startup.

**Solutions:**
1. Check logs: `~/harmony_research/bin/log/harmony_ros_interface_log.csv`
2. Verify `harmony_research` library is accessible
3. Ensure shared memory is properly set up
4. Check Research Interface initialization

## Environment Variables

The application uses the following environment variables:

- **`ROSBRIDGE_HOST`**: rosbridge server IP/hostname (default: `127.0.0.1`)
  - **On Harmony**: Must be set to your development machine's IP
  - Can be set during deployment or in `/root/.bashrc`
  
- **`ROSBRIDGE_PORT`**: rosbridge server port (default: `9090`)

**Setting on Harmony:**
```bash
# Temporary (current session only)
export ROSBRIDGE_HOST=192.168.2.2
export ROSBRIDGE_PORT=9090

# Persistent (add to ~/.bashrc)
echo 'export ROSBRIDGE_HOST=192.168.2.2' >> ~/.bashrc
echo 'export ROSBRIDGE_PORT=9090' >> ~/.bashrc
source ~/.bashrc
```

## Command-Line Options

```bash
# Default frequency (100 Hz)
./harmony_ros_interface

# Custom frequency (e.g., 50 Hz)
./harmony_ros_interface 50

# Show help
./harmony_ros_interface --help
./harmony_ros_interface -h
```

## Logging

Logs are written to: `~/harmony_research/bin/log/harmony_ros_interface_log.csv`

The application logs:
- **Info level**: Connection status, mode changes, subscriber status
- **Debug level**: Detailed field updates, message publishing

## Safety Considerations

⚠️ **Important Safety Notes:**

1. **Always verify control mode** before sending commands
2. **Start with small torque values** when testing
3. **Monitor joint states** during operation
4. **Return to Harmony mode** when done with research control
5. **Ensure workspace is clear** before enabling override modes
6. **Test commands incrementally** - don't send large values immediately

## Additional Resources

- Main project README: See root `README.md` for build and deployment instructions
- ROS2 documentation: https://docs.ros.org/
- rosbridge documentation: https://github.com/RobotWebTools/rosbridge_suite

