# Harmony Research Interface

A ROS2 interface for the Harmony exoskeleton robot using rosbridge, enabling remote control and monitoring through shared memory communication.

## Architecture

```
┌─────────────────────────────────────────┐      ┌─────────────────────────────────────────┐
│         DEVELOPMENT MACHINE             │      │            HARMONY ROBOT                │
│         (e.g. 192.168.2.2)              │      │            (192.168.2.1)                │
│                                         │      │                                         │
│  ┌───────────────────────────────────┐  │      │  ┌───────────────────────────────────┐  │
│  │           ROS2 NODES              │  │      │  │    HARMONY REAL-TIME CONTROL      │  │
│  │  ┌─────────────┐ ┌─────────────┐  │  │      │  │         (Closed-source)           │  │
│  │  │ Your Node 1 │ │ Your Node 2 │  │  │      │  │                                   │  │
│  │  │ (publisher) │ │ (subscriber)│  │  │      │  │  - Real-time control loops        │  │
│  │  └──────┬──────┘ └──────^──────┘  │  │      │  │  - Internal freeform mode         │  │
│  │         │               │         │  │      │  │  - Internal impedance mode        │  │
│  │         v               │         │  │      │  │                                   │  │
│  │  ┌─────────────────────────────┐  │  │      │  └───────────────┬───────────────────┘  │
│  │  │      ROS2 Topics/Services   │  │  │      │                  │                      │
│  │  │  /harmony/left/joint_states │  │  │      │                  │ Shared Memory        │
│  │  │  /harmony/left/desired_*    │  │  │      │                  │ (IPC)                │
│  │  │  /harmony/*/enable_*        │  │  │      │                  │                      │
│  │  └──────────────┬──────────────┘  │  │      │  ┌───────────────v───────────────────┐  │
│  └─────────────────┼─────────────────┘  │      │  │     HARMONY RESEARCH INTERFACE    │  │
│                    │                    │      │  │      (harmony_ros_interface)      │  │
│  ┌─────────────────v─────────────────┐  │      │  │                                   │  │
│  │        ROSBRIDGE SERVER           │  │      │  │  - Reads joint states             │  │
│  │        (Port 9090)                │  │      │  │  - Writes torque/stiffness/pos    │  │
│  │                                   │  │      │  │  - Mode switching                 │  │
│  │  ros2 launch rosbridge_server     │  │      │  └───────────────┬───────────────────┘  │
│  │  rosbridge_websocket_launch.xml   │  │      │                  │                      │
│  └─────────────────┬─────────────────┘  │      │  ┌───────────────v───────────────────┐  │
│                    │                    │      │  │        ROSBRIDGE CLIENT           │  │
└────────────────────┼────────────────────┘      │  │  (built-in harmony_ros_interface) │  │
                     │                           │  └───────────────┬───────────────────┘  │
                     │      WebSocket            │                  │                      │
                     │      Connection           └──────────────────┼──────────────────────┘
                     │      (TCP:9090)                              │
                     └──────────────────────────────────────────────┘
```

**Components:**
- **Development Machine**: Runs ROS2, your custom nodes, and `rosbridge_server`
- **Harmony Robot**: Runs the real-time control system and `harmony_ros_interface`
- **Shared Memory (IPC)**: Communication between Harmony's real-time control and the research interface
- **WebSocket**: Network bridge between rosbridge server and client (JSON-RPC over TCP)

## Quick Start

> For detailed options, see the [Setup Guide](docs/setup.md).

### Docker Build Pipeline

```
[Development Machine]                          [Harmony Robot]
     │                                                │
     V                                                │
┌───────────────────────┐                             │
│ 0. Install Docker &   │                             │
│    Setup Network      │                             │
│    - docs.docker.com/ │                             │
│      engine/install/  │                             │
│    - ufw allow 9090   │                             │
└───────────┬───────────┘                             │
            V                                         │
┌───────────────────────┐                             │
│ 1. Build Docker Image │                             │
│    ./build-docker-    │<── Dockerfile               │
│       image.sh        │    (Ubuntu 18.04)           │
└───────────┬───────────┘                             │
            V                                         │
┌───────────────────────┐                             │
│ 2. Build Project      │                             │
│    ./build-in-        │──> build-docker/            │
│       docker.sh       │    harmony_ros_interface    │
└───────────┬───────────┘                             │
            V                                         │
┌───────────────────────┐     SCP binary        ┌─────┴───────────────┐
│ 3. Deploy to Harmony  │──────────────────────>│ /opt/hbi/dev/bin/   │
│    ./deploy-to-       │     + configure       │ tools/              │
│       harmony.sh      │     ROSBRIDGE_HOST    │ harmony_ros_        │
└───────────────────────┘                       │ interface           │
                                                └─────────────────────┘
```

### 0. Install Docker & Setup Network (Development Machine)

1. Install Docker following the official guide: https://docs.docker.com/engine/install/ubuntu/
2. Install rosbridge: `sudo apt install -y ros-${ROS_DISTRO}-rosbridge-server`
3. Open firewall for rosbridge: `sudo ufw allow 9090/tcp`

### 1. Build with Docker

```bash
# Build the Docker image (one-time setup)
./build-docker-image.sh

# Build the project
./build-in-docker.sh
```

### 2. Deploy to Harmony

```bash
# Deploy with your dev machine's IP
ROSBRIDGE_HOST=<your-dev-machine-ip> ./deploy-to-harmony.sh
```

> For more options and configuration, see the [Setup Guide](docs/setup.md#deployment).

### 3. Start rosbridge Server (Development Machine)

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
ros2 run rosbridge_server rosbridge_websocket --port 9090
```

### 4. Launch on Harmony

```bash
ssh root@192.168.2.1
cd /opt/hbi/dev/bin/tools
./harmony_ros_interface
```

### 5. Verify Connection

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
| [Setup Guide](docs/setup.md) | Building, deployment, and configuration |
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
