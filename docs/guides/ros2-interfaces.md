# ROS2 Interfaces

## Executables

Available console scripts:

- `joint_state_publisher`
- `dual_arm_server` (legacy)
- `dual_arm_client` (legacy)
- `frame_task_server` (recommended)
- `frame_task_client`
- `hand_controller_node`
- `hand_cmd_gui`

## Topics

### Published

- `/joint_states` (`sensor_msgs/msg/JointState`) - published by `joint_state_publisher`
- `/left_ee_pose` (`geometry_msgs/msg/PoseStamped`) - published by `frame_task_server` or `dual_arm_server`
- `/right_ee_pose` (`geometry_msgs/msg/PoseStamped`) - published by `frame_task_server` or `dual_arm_server`
- `/left_ee_target` (`geometry_msgs/msg/PoseStamped`) - published by `frame_task_server` or `dual_arm_server`
- `/right_ee_target` (`geometry_msgs/msg/PoseStamped`) - published by `frame_task_server` or `dual_arm_server`
- `/left_hand_state` (`std_msgs/msg/Float64MultiArray`) - published by `hand_controller_node`
- `/right_hand_state` (`std_msgs/msg/Float64MultiArray`) - published by `hand_controller_node`
- `/frame_names` (`custom_ros_messages/msg/StringArray`) - published by `frame_task_server` (active frame names)
- `/frame_targets` (`geometry_msgs/msg/PoseArray`) - published by `frame_task_server` (target poses)
- `/frame_poses` (`geometry_msgs/msg/PoseArray`) - published by `frame_task_server` (current frame poses)

### Subscribed

- `/left_hand_cmd` (`std_msgs/msg/Float64MultiArray`)
- `/right_hand_cmd` (`std_msgs/msg/Float64MultiArray`)

## Actions

### `dual_arm` (`custom_ros_messages/action/DualArm`) [Legacy]

Used by:

- Server: `dual_arm_server`
- Client: `dual_arm_client`

Purpose:

- Move left/right end-effectors toward target poses with feedback on linear/angular error.
- **Note:** `dual_arm_server` is not active in `robot_launch.py` by default. Use `frame_task_server` instead.

### `named_config` (`custom_ros_messages/action/NamedConfig`)

Used by:

- Servers: `dual_arm_server`, `frame_task_server`
- Clients: `dual_arm_client`, `frame_task_client`

Purpose:

- Move to predefined reduced joint configurations.

### `frame_task` (`custom_ros_messages/action/FrameTask`) [Recommended]

Used by:

- Server: `frame_task_server`
- Client: `frame_task_client`

Purpose:

- Apply one or more named frame pose tasks simultaneously.
- Allows control of any robot frame (end-effectors, intermediate links, body frames).
- Supports multi-frame coordination with IK-based control.

## Introspection commands

```bash
ros2 node list
ros2 topic list
ros2 action list
ros2 action info /dual_arm
ros2 interface show custom_ros_messages/action/DualArm
```
