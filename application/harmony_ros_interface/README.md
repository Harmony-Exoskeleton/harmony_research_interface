# Harmony ROS Interface - Usage Manual

This manual provides a guide for controlling and monitoring the Harmony robot arms via ROS2.

## Overview

The `harmony_ros_interface` application runs on the Harmony robot and connects to a rosbridge server on your development machine, enabling ROS2 communication over WebSocket.

## Control Modes

The system supports three control modes:

### 1. Harmony Mode (Default)

The robot operates under Harmony's default control system. Research interface commands are ignored.

```bash
# Return to Harmony mode
ros2 service call /harmony/{left|right}/disable_override_mode std_srvs/srv/Trigger
```

### 2. Torque Mode

Direct torque control. Only torque commands are accepted.

```bash
# Enable torque mode
ros2 service call /harmony/{left|right}/enable_torque_mode std_srvs/srv/Trigger

# Send torque commands (7 values in Nm)
ros2 topic pub /harmony/left/desired_torque std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

### 3. Impedance Mode

Full impedance control with torque, stiffness, and position. The arm behaves like a spring-damper system.

```bash
# Enable impedance mode
ros2 service call /harmony/{left|right}/enable_impedance_mode std_srvs/srv/Trigger

# Send commands (can update independently)
ros2 topic pub /harmony/left/desired_torque std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

ros2 topic pub /harmony/left/desired_stiffness std_msgs/msg/Float64MultiArray \
  "{data: [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]}"

ros2 topic pub /harmony/left/desired_position std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

**Mode Behavior:**
- In **impedance mode**: All three fields can be updated independently; other fields are preserved
- In **torque mode**: Only torque commands accepted; stiffness/position rejected
- Fields reset when switching modes

## Arm Features

These features can be toggled independently of control mode:

### Gravity Compensation

Compensates for gravitational forces on the arm.

```bash
ros2 service call /harmony/{left|right}/enable_gravity std_srvs/srv/SetBool "{data: true}"
ros2 service call /harmony/{left|right}/enable_gravity std_srvs/srv/SetBool "{data: false}"
```

**Recommendation:** Always enable when using torque or impedance mode.

### Scapulo-Humeral Rhythm (SHR)

Enables natural coupling between shoulder blade and upper arm movement.

```bash
ros2 service call /harmony/{left|right}/enable_shr std_srvs/srv/SetBool "{data: true}"
ros2 service call /harmony/{left|right}/enable_shr std_srvs/srv/SetBool "{data: false}"
```

### Joint Constraints

Enables joint limit and kinematic constraints for safety.

```bash
ros2 service call /harmony/{left|right}/enable_constraints std_srvs/srv/SetBool "{data: true}"
ros2 service call /harmony/{left|right}/enable_constraints std_srvs/srv/SetBool "{data: false}"
```

## Monitoring

### Joint States

Real-time joint position, velocity, and effort for all 7 joints.

```bash
ros2 topic echo /harmony/{left|right}/joint_states
```

### Arm Sizes

Arm segment lengths in mm for kinematic calculations.

```bash
ros2 topic echo /harmony/{left|right}/sizes
```

### End-Effector Poses (TF)

Transform frames for end-effector positions.

```bash
ros2 run tf2_ros tf2_echo base_link left_end_effector
```

### State Queries

```bash
ros2 service call /harmony/get_state std_srvs/srv/Trigger           # Both arms
ros2 service call /harmony/{left|right}/get_state std_srvs/srv/Trigger  # Single arm
```

## Complete Workflow Example

```bash
# 1. Start rosbridge (development machine)
ros2 run rosbridge_server rosbridge_websocket --port 9090 &

# 2. Launch harmony_ros_interface (on Harmony)
export ROSBRIDGE_HOST=192.168.2.2
./harmony_ros_interface

# 3. Verify connection (development machine)
ros2 topic list
ros2 service list

# 4. Get current state
ros2 service call /harmony/left/get_state std_srvs/srv/Trigger

# 5. Enable impedance mode
ros2 service call /harmony/left/enable_impedance_mode std_srvs/srv/Trigger

# 6. Enable gravity compensation
ros2 service call /harmony/left/enable_gravity std_srvs/srv/SetBool "{data: true}"

# 7. Set stiffness and position
ros2 topic pub -r 100 /harmony/left/desired_stiffness std_msgs/msg/Float64MultiArray \
  "{data: [20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0]}"

ros2 topic pub -r 100 /harmony/left/desired_position std_msgs/msg/Float64MultiArray \
  "{data: [0.1, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# 8. Monitor joint states
ros2 topic echo /harmony/left/joint_states

# 9. Return to Harmony mode when done
ros2 service call /harmony/left/disable_override_mode std_srvs/srv/Trigger
```

## Command Reference

### Topics Published

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/harmony/{left,right}/joint_states` | `sensor_msgs/JointState` | Loop rate | Position, velocity, effort |
| `/harmony/{left,right}/sizes` | `std_msgs/Float64MultiArray` | Loop rate | Arm segment lengths (mm) |
| `/tf` | `tf2_msgs/TFMessage` | Loop rate | End-effector transforms |
| `/tf_static` | `tf2_msgs/TFMessage` | 5s | Static transforms |

### Topics Subscribed

| Topic | Type | Modes | Description |
|-------|------|-------|-------------|
| `/harmony/{left,right}/desired_torque` | `std_msgs/Float64MultiArray` | Torque, Impedance | 7 torque values (Nm) |
| `/harmony/{left,right}/desired_stiffness` | `std_msgs/Float64MultiArray` | Impedance only | 7 stiffness values (Nm/rad) |
| `/harmony/{left,right}/desired_position` | `std_msgs/Float64MultiArray` | Impedance only | 7 position values (rad) |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/harmony/get_state` | `std_srvs/Trigger` | Get state of both arms |
| `/harmony/{left,right}/get_state` | `std_srvs/Trigger` | Get state of single arm |
| `/harmony/{left,right}/enable_torque_mode` | `std_srvs/Trigger` | Enable torque mode |
| `/harmony/{left,right}/enable_impedance_mode` | `std_srvs/Trigger` | Enable impedance mode |
| `/harmony/{left,right}/disable_override_mode` | `std_srvs/Trigger` | Return to Harmony mode |
| `/harmony/{left,right}/enable_gravity` | `std_srvs/SetBool` | Toggle gravity compensation |
| `/harmony/{left,right}/enable_shr` | `std_srvs/SetBool` | Toggle SHR |
| `/harmony/{left,right}/enable_constraints` | `std_srvs/SetBool` | Toggle constraints |
| `/harmony/reset_shared_memory` | `std_srvs/Trigger` | Reset shared memory |

## Configuration

### Command-Line Options

```bash
./harmony_ros_interface          # Default 100 Hz
./harmony_ros_interface 50       # Custom frequency
./harmony_ros_interface --help   # Show help
```

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `ROSBRIDGE_HOST` | `127.0.0.1` | rosbridge server IP |
| `ROSBRIDGE_PORT` | `9090` | rosbridge server port |

## RViz2 Visualization

```bash
rviz2
```

Add TF display to visualize robot structure and end-effector poses.

**If "No transform from map" error appears:**
- Change "Fixed Frame" from `map` to `base_link` in Global Options, or
- Wait for static transform (republished every 5 seconds)

## Safety Considerations

1. **Verify control mode** before sending commands
2. **Start with small values** when testing
3. **Monitor joint states** during operation
4. **Return to Harmony mode** when done
5. **Ensure workspace is clear** before enabling override modes

## Logging

Logs: `~/harmony_research/bin/log/harmony_ros_interface_log.csv`

- **Info**: Connection status, mode changes
- **Debug**: Field updates, message publishing
