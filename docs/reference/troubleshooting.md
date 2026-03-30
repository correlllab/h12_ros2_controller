# Troubleshooting

## No robot state updates

Symptoms:

- `/joint_states` stale or empty.
- Wrist pose topics not updating.

Checks:

```bash
ros2 node list
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once
```

## Action server unavailable

Symptoms:

- Client waits indefinitely for action server.

Checks:

```bash
ros2 action list
ros2 node list | grep dual_arm_server
ros2 node list | grep frame_task_server
```

## Hand commands ignored

Symptoms:

- GUI sliders move but fingers do not.

Checks:

```bash
ros2 topic echo /left_hand_cmd
ros2 topic echo /left_hand_state
ros2 node list | grep hand_controller_node
```

## URDF/model path errors

Symptoms:

- Startup fails on model loading.

Checks:

- Verify `h12_ros2_model` is installed and sourced.
- Confirm `ament_index` can resolve package share paths.

## Unexpected estop

Symptoms:

- Motion stops abruptly and commands are dropped.

Checks:

- Inspect logs for limit exceeded message (joint index and value).
- Reduce target magnitude and velocity demand.
- Ensure no competing controller is publishing commands.
