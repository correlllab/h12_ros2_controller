# Hardware and Networking

## Hardware checklist

Before commanding hardware, confirm:

- Robot is mechanically clear of obstacles and humans.
- Emergency stop access is physically available.
- Robot power state and controller mode are appropriate for external control.
- Wrist/hand areas have clearance for test trajectories.

## Networking checklist

1. Connect workstation and robot controller on the same network segment.
2. Validate link and route:

```bash
ip addr
ip route
ping <robot-ip>
```

3. Confirm that DDS traffic is not blocked by firewall rules.
4. If using multiple NICs, ensure ROS2 and SDK traffic uses the intended interface.

## ROS2 environment checklist

```bash
source /opt/ros/<distro>/setup.bash
source ~/ros2_ws/install/setup.bash
printenv | grep -E 'ROS_DOMAIN_ID|RMW|CYCLONEDDS|FASTRTPS'
```

## Safety model in this package

Runtime safety is enforced in the command handler by checking:

- Joint position limits
- Joint velocity limits
- Joint torque limits

When exceeded, estop is triggered via the publisher path.

See [Safety and Estop](../reference/safety.md) for operational details.

## Recommended bring-up mode order

1. Visualization-only / debug scripts.
2. ROS2 TF + joint state publishing.
3. Action servers with conservative goals.
4. Hand control commands.
5. Higher-rate or larger workspace motions.
