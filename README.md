# Harmony Research Interface

A ROS2 interface for the Harmony exoskeleton robot using rosbridge, enabling remote control and monitoring through shared memory communication.

## Architecture

```
+-------------------+       WebSocket        +------------------+
|  Dev Machine      |<---------------------->|  Harmony Robot   |
|                   |       (Port 9090)      |                  |
| - ROS2            |                        | - harmony_ros_   |
| - rosbridge_server|                        |   interface      |
| - Your ROS nodes  |                        | - Shared Memory  |
+-------------------+                        +------------------+
```

- **Harmony Robot**: Runs `harmony_ros_interface` (cross-compiled binary)
- **Development Machine**: Runs ROS2 with `rosbridge_server` for debugging and control

## Quick Start

> For detailed build options, see the [Installation Guide](docs/installation.md).

### 1. Install Dependencies (Development Machine)

```bash
sudo apt update
sudo apt install -y docker.io ros-${ROS_DISTRO}-rosbridge-server
```

### 2. Build with Docker

```bash
# Build the Docker image (one-time setup)
./build-docker-image.sh

# Build the project
./build-in-docker.sh
```

### 3. Deploy to Harmony

```bash
# Deploy with your dev machine's IP
ROSBRIDGE_HOST=<your-dev-machine-ip> ./deploy-to-harmony.sh
```

> For more deployment options and configuration, see the [Deployment Guide](docs/deployment.md).

### 4. Start rosbridge Server (Development Machine)

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
ros2 run rosbridge_server rosbridge_websocket --port 9090
```

### 5. Launch on Harmony

```bash
ssh root@192.168.2.1
cd /opt/hbi/dev/bin/tools
./harmony_ros_interface
```

### 6. Verify Connection

```bash
ros2 topic list              # Should show /harmony/* topics
ros2 service list            # Should show /harmony/* services
ros2 topic echo /harmony/left/joint_states
```

> **Next steps:** See the [Usage Manual](application/harmony_ros_interface/README.md) for control modes, commands, and examples.
>
> **Having issues?** Check the [Troubleshooting Guide](docs/troubleshooting.md).

## Documentation

| Document | Description |
|----------|-------------|
| [Installation Guide](docs/installation.md) | Build instructions and Docker setup |
| [Deployment Guide](docs/deployment.md) | Deploying to Harmony, configuration, network setup |
| [Troubleshooting](docs/troubleshooting.md) | Common issues and solutions |
| [Usage Manual](application/harmony_ros_interface/README.md) | Control modes, topics, services, and ROS2 commands |

## Quick Reference

### Topics Published

| Topic | Type | Description |
|-------|------|-------------|
| `/harmony/{left,right}/joint_states` | `sensor_msgs/JointState` | Joint positions, velocities, torques |
| `/harmony/{left,right}/sizes` | `std_msgs/Float64MultiArray` | Arm segment lengths (mm) |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | End-effector transforms |

### Topics Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/harmony/{left,right}/desired_torque` | `std_msgs/Float64MultiArray` | Torque commands (Nm) |
| `/harmony/{left,right}/desired_stiffness` | `std_msgs/Float64MultiArray` | Stiffness commands (Nm/rad) |
| `/harmony/{left,right}/desired_position` | `std_msgs/Float64MultiArray` | Position commands (rad) |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/harmony/get_state` | `std_srvs/Trigger` | Query robot state |
| `/harmony/{left,right}/enable_torque_mode` | `std_srvs/Trigger` | Enable torque control |
| `/harmony/{left,right}/enable_impedance_mode` | `std_srvs/Trigger` | Enable impedance control |
| `/harmony/{left,right}/disable_override_mode` | `std_srvs/Trigger` | Return to Harmony mode |
| `/harmony/{left,right}/enable_gravity` | `std_srvs/SetBool` | Toggle gravity compensation |
| `/harmony/{left,right}/enable_shr` | `std_srvs/SetBool` | Toggle scapulo-humeral rhythm |
| `/harmony/{left,right}/enable_constraints` | `std_srvs/SetBool` | Toggle joint constraints |

## Helper Scripts

| Script | Purpose |
|--------|---------|
| `build-docker-image.sh` | Build the Docker image (one-time) |
| `build-in-docker.sh` | Build the project using Docker |
| `deploy-to-harmony.sh` | Deploy binary to Harmony robot |
| `ssh-harmony.sh` | Quick SSH access to Harmony |
| `check-harmony-versions.sh` | Check Harmony system info |

## Project Structure

```
harmony_research_interface/
├── include/                    # Core interface headers
├── application/
│   ├── harmony_ros_interface/  # Main ROS interface application
│   ├── harmony_perturb_torques/# Torque perturbation testing
│   └── realtime_logger/        # Real-time data logging
├── tools/                      # Utility programs
├── resources/                  # Third-party libraries (Eigen, plog, etc.)
├── lib/                        # Pre-built Harmony research library
├── tests/                      # Unit tests
├── docs/                       # Documentation
├── Dockerfile                  # Build environment definition
└── *.sh                        # Build and deployment scripts
```

## License

See individual library licenses in `resources/` for third-party dependencies.
