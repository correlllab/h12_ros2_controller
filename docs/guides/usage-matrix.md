# Usage Matrix

Task-oriented map from intent to command path.

| Goal | What to run | Primary interface | Verify | Notes |
|---|---|---|---|---|
| Read wrist pose | `ros2 launch h12_ros2_controller robot_launch.py` | Topics `/left_ee_pose`, `/right_ee_pose` | `ros2 topic echo /left_ee_pose` | Published by `DualArmServer` at 100 Hz. |
| Move both arms to target pose | `ros2 run h12_ros2_controller dual_arm_client` | Action `dual_arm` | Client feedback logs error norms | Uses `DualArmServer` + `ArmController`. |
| Move to named configuration | `ros2 run h12_ros2_controller dual_arm_client` | Action `named_config` | Feedback `joint_error` | Named presets in `utility/named_config.py`. |
| Move arbitrary frames | `ros2 run h12_ros2_controller frame_task_client` | Action `frame_task` | Feedback `errors_linear`, `errors_angular` | Requires frame names available in URDF model. |
| Move hands interactively | `ros2 run h12_ros2_controller hand_cmd_gui` | Topics `/left_hand_cmd`, `/right_hand_cmd` | `ros2 topic echo /left_hand_state` | GUI publishes 6-DoF normalized finger command per hand. |
| Move hands programmatically | `ros2 topic pub /right_hand_cmd std_msgs/msg/Float64MultiArray '{data: [0.0,0.0,0.0,0.0,0.0,0.0]}'` | Topic message | State topics | Values must be in `[0.0, 1.0]`, length 6. |
| Run arm script without ROS actions | `uv run h12_ros2_controller/example/arm_controller_goto.py --sport` | Direct SDK/controller loop | Console error norms + optional visualization | Useful for debugging IK behavior. |
| Publish joint states + TF only | `ros2 launch h12_ros2_controller robot_tf_launch.py` | `/joint_states` + TF tree | `ros2 topic hz /joint_states` | Good for RViz sanity check before active control. |
| Full desktop bring-up | `ros2 launch h12_ros2_controller full_launch.py` | Combined launch graph | RViz + topics + action availability | Includes delayed RViz startup. |

## Minimal run recipes

### Read wrist pose quickly

```bash
ros2 launch h12_ros2_controller robot_launch.py
ros2 topic echo /left_ee_pose
```

### Move hands quickly

```bash
ros2 launch h12_ros2_controller robot_launch.py
ros2 run h12_ros2_controller hand_cmd_gui
```

### Run a specific script

```bash
uv run h12_ros2_controller/example/frame_controller_goto.py --debug
```

See [Scripts and Examples](scripts-and-examples.md) for more script-level workflows.
