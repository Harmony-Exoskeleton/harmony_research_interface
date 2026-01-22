# Setup Guide

This guide covers building, deploying, and running the `harmony_ros_interface` application.

## Prerequisites

### Development Machine

1. **Install Docker** following the official guide: https://docs.docker.com/engine/install/ubuntu/

2. **Install build tools and rosbridge:**
   ```bash
   sudo apt update
   sudo apt install -y cmake build-essential
   sudo apt install -y ros-${ROS_DISTRO}-rosbridge-suite
   ```

3. **Open firewall for rosbridge:**
   ```bash
   sudo ufw allow 9090/tcp
   ```

### Harmony Robot

- Network connectivity to the development machine
- No additional software required (all libraries are statically linked)

## Building

### Building with Docker (Recommended)

Docker ensures binary compatibility with Harmony's environment (GLIBC 2.32, x86_64).

#### Why Docker?

The Harmony robot runs Buildroot 2021.02 with GLIBC 2.32. Using Docker with Ubuntu 18.04 ensures:
- **GLIBC compatibility**: Ubuntu 18.04's GLIBC 2.27 produces backward-compatible binaries
- **Structure alignment**: GCC 7.5 produces compatible structure padding (184 bytes for ArmControllerState)
- **Consistent builds**: Same compiler across all development machines

#### Step 1: Build Docker Image (One-Time)

```bash
./build-docker-image.sh
```

This creates the `harmony-build:ubuntu18.04` image with all build dependencies.

#### Step 2: Build the Project

```bash
./build-in-docker.sh
```

The binary is output to: `build-docker/application/harmony_ros_interface/harmony_ros_interface`

#### Verifying the Build

Check GLIBC requirements:
```bash
ldd build-docker/application/harmony_ros_interface/harmony_ros_interface
```

### Local Development Build

For testing on your development machine (not for Harmony deployment):

```bash
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

**Note:** Local builds may not be compatible with Harmony due to GLIBC version differences. Always use Docker for builds intended for Harmony.

### Build Output

| Output | Location |
|--------|----------|
| Main application | `build-docker/application/harmony_ros_interface/harmony_ros_interface` |
| Tools | `build-docker/tools/` |
| Libraries | `build-docker/resources/` |

## Deployment

### Using the Deploy Script (Recommended)

The `deploy-to-harmony.sh` script handles copying the binary and configuring environment variables.

```bash
# Basic deployment (ROSBRIDGE_HOST defaults to 192.168.2.2)
./deploy-to-harmony.sh

# Deployment with custom ROSBRIDGE_HOST
ROSBRIDGE_HOST=<development-machine-ip> ./deploy-to-harmony.sh

# With custom port
ROSBRIDGE_HOST=<ip> ROSBRIDGE_PORT=9090 ./deploy-to-harmony.sh
```

The script will:
1. Copy the binary to Harmony (default: `/opt/hbi/dev/bin/tools/`)
2. Configure `ROSBRIDGE_HOST` in `/root/.bashrc`
3. Verify the deployment

### Manual Deployment

```bash
# Copy binary to Harmony
scp build-docker/application/harmony_ros_interface/harmony_ros_interface \
    root@192.168.2.1:/opt/hbi/dev/bin/tools/

# SSH into Harmony and configure
ssh root@192.168.2.1
echo 'export ROSBRIDGE_HOST=<your-dev-machine-ip>' >> ~/.bashrc
echo 'export ROSBRIDGE_PORT=9090' >> ~/.bashrc
```

### Deployment Configuration

| Variable | Default | Description |
|----------|---------|-------------|
| `HARMONY_IP` | `192.168.2.1` | Harmony robot IP address |
| `HARMONY_USER` | `root` | SSH username for Harmony |
| `SSH_PORT` | `22` | SSH port |
| `HARMONY_DEST_DIR` | `/opt/hbi/dev/bin/tools` | Deployment directory on Harmony |

Example:
```bash
HARMONY_IP=10.0.0.5 HARMONY_USER=admin ./deploy-to-harmony.sh
```

## Running the System

### Step 1: Start rosbridge Server (Development Machine)

```bash
# Set up ROS2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# Start rosbridge server
ros2 run rosbridge_server rosbridge_websocket --port 9090

# Or run in background
ros2 run rosbridge_server rosbridge_websocket --port 9090 &
```

Note your development machine's IP:
```bash
hostname -I
```

### Step 2: Launch on Harmony

```bash
# SSH into Harmony
ssh root@192.168.2.1

# Navigate to binary location
cd /opt/hbi/dev/bin/tools

# Load environment (if not already loaded)
source ~/.bashrc

# Or manually set connection parameters
export ROSBRIDGE_HOST=<development-machine-ip>
export ROSBRIDGE_PORT=9090

# Run with default 100 Hz loop frequency
./harmony_ros_interface

# Or specify custom frequency
./harmony_ros_interface 50
```

### Step 3: Verify Connection (Development Machine)

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash

# List topics
ros2 topic list

# List services
ros2 service list

# Get robot state
ros2 service call /harmony/get_state std_srvs/srv/Trigger
```

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `ROSBRIDGE_HOST` | `127.0.0.1` | rosbridge server IP. **On Harmony, set to your dev machine's IP** |
| `ROSBRIDGE_PORT` | `9090` | rosbridge server port |

### Persistent Configuration on Harmony

The deploy script automatically configures `ROSBRIDGE_HOST` in `/root/.bashrc`. To change it manually:

```bash
ssh root@192.168.2.1
nano ~/.bashrc  # or vi ~/.bashrc

# Find and update:
export ROSBRIDGE_HOST=<new-ip>
export ROSBRIDGE_PORT=9090

# Apply changes
source ~/.bashrc
```

### Command-Line Options

```bash
# Default frequency (100 Hz)
./harmony_ros_interface

# Custom frequency (e.g., 50 Hz)
./harmony_ros_interface 50

# Show help
./harmony_ros_interface --help
./harmony_ros_interface -h
```

## Network Setup

### Required Ports

| Port | Protocol | Purpose |
|------|----------|---------|
| 9090 | TCP | rosbridge WebSocket |
| 22 | TCP | SSH (deployment) |

### Firewall Configuration

On the development machine, ensure port 9090 is open:

```bash
# Ubuntu/Debian with ufw
sudo ufw allow 9090/tcp

# Or with iptables
sudo iptables -A INPUT -p tcp --dport 9090 -j ACCEPT
```

### Testing Connectivity

From Harmony:
```bash
# Test rosbridge connectivity
telnet <dev-machine-ip> 9090

# Test with netcat
nc -zv <dev-machine-ip> 9090
```

From development machine:
```bash
# Ping Harmony
ping 192.168.2.1

# Test SSH
ssh -o ConnectTimeout=5 root@192.168.2.1 "echo 'Connected'"
```

## Running as a Service (Optional)

To run `harmony_ros_interface` automatically on Harmony boot, create a systemd service:

```bash
# On Harmony, create service file
cat > /etc/systemd/system/harmony-ros.service << 'EOF'
[Unit]
Description=Harmony ROS Interface
After=network.target

[Service]
Type=simple
Environment="ROSBRIDGE_HOST=192.168.2.2"
Environment="ROSBRIDGE_PORT=9090"
ExecStart=/opt/hbi/dev/bin/tools/harmony_ros_interface
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

# Enable and start
systemctl daemon-reload
systemctl enable harmony-ros
systemctl start harmony-ros

# Check status
systemctl status harmony-ros
```

## Helper Scripts

| Script | Purpose |
|--------|---------|
| `build-docker-image.sh` | Build the Docker image (one-time) |
| `build-in-docker.sh` | Build the project using Docker |
| `deploy-to-harmony.sh` | Deploy binary and configure environment |
| `ssh-harmony.sh` | Quick SSH access to Harmony |
| `check-harmony-versions.sh` | Check OS, GLIBC, and architecture on Harmony |

### Examples

```bash
# Quick SSH access
./ssh-harmony.sh

# Check Harmony system info
./check-harmony-versions.sh

# Deploy with custom settings
HARMONY_IP=10.0.0.5 ROSBRIDGE_HOST=10.0.0.1 ./deploy-to-harmony.sh
```

## Logging

Logs are written to: `~/harmony_research/bin/log/harmony_ros_interface_log.csv`

The application logs:
- **Info level**: Connection status, mode changes, subscriber status
- **Debug level**: Detailed field updates, message publishing
