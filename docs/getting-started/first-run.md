# First Run

This sequence validates the full stack with low risk and clear observability.

## 1. Build and source

```bash
cd ~/ros2_ws
colcon build --packages-select h12_ros2_controller
source install/setup.bash
```

## 2. Launch robot state + frame task server + hand controller

```bash
ros2 launch h12_ros2_controller robot_launch.py
```

This launches:

- `robot_state_publisher`
- `joint_state_publisher`
- `frame_task_server` (recommended)
- `hand_controller_node`

## 3. Open RViz (desktop or full launch)

```bash
ros2 launch h12_ros2_controller desktop_launch.py
# or
ros2 launch h12_ros2_controller full_launch.py
```

## 4. Observe wrist pose topics

```bash
ros2 topic echo /left_ee_pose
ros2 topic echo /right_ee_pose
```

## 5. Send a frame task goal

```bash
ros2 run h12_ros2_controller frame_task_client
```

Use one of the named configs when prompted, or input explicit frame names and poses.

For more details on frame tasks, see [Frame Task Architecture Guide](../guides/frame-task-guide.md).

## 6. Move hands (GUI)

```bash
ros2 run h12_ros2_controller hand_cmd_gui
```

In a second terminal, observe states:

```bash
ros2 topic echo /left_hand_state
ros2 topic echo /right_hand_state
```

## 7. Shutdown

Stop clients first, then servers/launch. Verify no control loop remains active.
