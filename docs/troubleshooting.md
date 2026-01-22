# Troubleshooting

This guide covers common issues and their solutions when working with `harmony_ros_interface`.

## Connection Issues

### Problem: Cannot connect to rosbridge

**Symptoms:**
- `harmony_ros_interface` hangs or fails on startup
- "Connection refused" errors
- No topics/services visible on development machine

**Solutions:**

1. **Verify rosbridge is running:**
   ```bash
   ros2 node list | grep rosbridge
   # Should show: /rosbridge_websocket
   ```

2. **Check firewall settings:**
   ```bash
   # On development machine, allow port 9090
   sudo ufw allow 9090/tcp
   ```

3. **Verify environment variables on Harmony:**
   ```bash
   echo $ROSBRIDGE_HOST  # Should be your dev machine IP
   echo $ROSBRIDGE_PORT  # Should be 9090
   ```

4. **Test connectivity from Harmony:**
   ```bash
   telnet <development-machine-ip> 9090
   # Or
   nc -zv <development-machine-ip> 9090
   ```

5. **Check network connectivity:**
   ```bash
   # From Harmony
   ping <development-machine-ip>

   # From development machine
   ping 192.168.2.1
   ```

### Problem: rosbridge connects but disconnects immediately

**Solutions:**

1. Check rosbridge server logs for errors
2. Verify no other client is connected to the same rosbridge instance
3. Try restarting rosbridge server

## ROS2 Discovery Issues

### Problem: Cannot see Harmony topics/services

**Symptoms:**
- `ros2 topic list` doesn't show Harmony topics
- `ros2 service list` doesn't show Harmony services

**Solutions:**

1. **Check ROS_DOMAIN_ID:**
   ```bash
   echo $ROS_DOMAIN_ID  # Should match on both machines (default: 0)
   export ROS_DOMAIN_ID=0
   ```

2. **Verify ROS2 environment is sourced:**
   ```bash
   source /opt/ros/${ROS_DISTRO}/setup.bash
   ```

3. **Check network connectivity:**
   ```bash
   ping 192.168.2.1
   ```

4. **Allow ROS2 multicast traffic:**
   ```bash
   # Allow UDP ports 7400-7500 for ROS2 discovery
   sudo ufw allow 7400:7500/udp
   ```

5. **Disable localhost-only mode:**
   ```bash
   export ROS_LOCALHOST_ONLY=0
   ```

6. **Verify rosbridge is connected:**
   - Check `harmony_ros_interface` output for "Successfully connected"
   - Check rosbridge server output for connection messages

## Application Crashes

### Problem: harmony_ros_interface crashes on startup

**Solutions:**

1. **Check logs:**
   ```bash
   cat ~/harmony_research/bin/log/harmony_ros_interface_log.csv
   ```

2. **Verify shared memory:**
   ```bash
   # On Harmony, check shared memory segments
   ipcs -m
   ```

3. **Reset shared memory if corrupted:**
   ```bash
   # Via ROS service (if connected)
   ros2 service call /harmony/reset_shared_memory std_srvs/srv/Trigger

   # Or manually on Harmony
   ipcrm -M <key>  # Remove specific segment
   ```

4. **Check Research Interface initialization:**
   - Verify Harmony's research interface is running
   - Check that shared memory IDs match (222, 223, 233, 234)

### Problem: Segmentation fault

**Solutions:**

1. **GLIBC version mismatch** - Rebuild with Docker:
   ```bash
   ./build-in-docker.sh
   ./deploy-to-harmony.sh
   ```

2. **Structure size mismatch** - Check compatibility:
   ```bash
   ./check-harmony-versions.sh
   ```

3. **Corrupted shared memory** - Reset and restart:
   ```bash
   ros2 service call /harmony/reset_shared_memory std_srvs/srv/Trigger
   ```

## Cross-Compilation Issues

### Problem: Build fails with linker errors

**Solutions:**

1. **Ensure correct library architecture:**
   ```bash
   file lib/libharmony_research.a
   # Should match target architecture (x86_64, arm, etc.)
   ```

2. **Verify toolchain configuration:**
   - Check `CMAKE_C_COMPILER` and `CMAKE_CXX_COMPILER` in toolchain file
   - Ensure cross-compiler is installed

3. **Use Docker build (recommended):**
   ```bash
   ./build-in-docker.sh
   ```

### Problem: Binary won't run on Harmony

**Symptoms:**
- "GLIBC_X.XX not found" errors
- "cannot execute binary file" errors

**Solutions:**

1. **Check GLIBC requirements:**
   ```bash
   # On development machine
   ldd build-docker/application/harmony_ros_interface/harmony_ros_interface

   # On Harmony
   ldd --version
   ```

2. **Verify architecture:**
   ```bash
   file build-docker/application/harmony_ros_interface/harmony_ros_interface
   # Should show: ELF 64-bit LSB executable, x86-64
   ```

3. **Rebuild with Docker:**
   ```bash
   rm -rf build-docker
   ./build-in-docker.sh
   ```

## Control Issues

### Problem: Joint commands not working

**Solutions:**

1. **Verify control mode:**
   ```bash
   ros2 service call /harmony/left/get_state std_srvs/srv/Trigger
   ```

2. **Enable correct mode:**
   ```bash
   # For torque commands
   ros2 service call /harmony/left/enable_torque_mode std_srvs/srv/Trigger

   # For torque + stiffness + position
   ros2 service call /harmony/left/enable_impedance_mode std_srvs/srv/Trigger
   ```

3. **Check command format:**
   - Must have exactly 7 values
   - Must use `std_msgs/msg/Float64MultiArray`

4. **Verify topic is subscribed:**
   ```bash
   ros2 topic info /harmony/left/desired_torque
   # Should show 1 subscriber
   ```

### Problem: Commands rejected

**Symptoms:**
- Stiffness/position commands rejected in torque mode
- All commands rejected in Harmony mode

**Solutions:**

1. **Check current mode:**
   ```bash
   ros2 service call /harmony/left/get_state std_srvs/srv/Trigger
   ```

2. **Mode requirements:**
   - **Harmony mode**: All commands rejected (default safe mode)
   - **Torque mode**: Only torque commands accepted
   - **Impedance mode**: All commands accepted

3. **Switch to appropriate mode:**
   ```bash
   ros2 service call /harmony/left/enable_impedance_mode std_srvs/srv/Trigger
   ```

## Docker Issues

### Problem: Docker image build fails

**Solutions:**

1. **Check Docker is installed:**
   ```bash
   docker --version
   ```

2. **Check Docker daemon is running:**
   ```bash
   sudo systemctl status docker
   sudo systemctl start docker
   ```

3. **Add user to docker group:**
   ```bash
   sudo usermod -aG docker $USER
   # Log out and back in
   ```

4. **Rebuild image:**
   ```bash
   docker rmi harmony-build:ubuntu18.04
   ./build-docker-image.sh
   ```

### Problem: Permission denied on build artifacts

**Solutions:**

The build script should handle permissions, but if files are owned by root:

```bash
sudo chown -R $USER:$USER build-docker/
```

## RViz2 Issues

### Problem: "No transform from [map] to [base_link]"

**Solutions:**

1. **Change Fixed Frame in RViz2:**
   - In "Global Options" panel
   - Change "Fixed Frame" from `map` to `base_link`

2. **Or wait for static transform:**
   - The application publishes `map` -> `base_link` on `/tf_static`
   - Try clicking "Reset" in RViz2

3. **Verify transforms are published:**
   ```bash
   ros2 topic echo /tf_static
   ros2 run tf2_ros tf2_echo map base_link
   ```

## Getting Help

If you're still having issues:

1. **Check logs:**
   ```bash
   cat ~/harmony_research/bin/log/harmony_ros_interface_log.csv
   ```

2. **Verify system versions:**
   ```bash
   ./check-harmony-versions.sh
   ```

3. **Collect diagnostic info:**
   ```bash
   # On development machine
   ros2 topic list
   ros2 service list
   ros2 node list

   # On Harmony
   echo $ROSBRIDGE_HOST
   echo $ROSBRIDGE_PORT
   ps aux | grep harmony
   ```
