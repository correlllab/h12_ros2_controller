# Scripts and Examples

The `h12_ros2_controller/example` directory provides direct control and data tooling outside ROS action clients.

## Common scripts

- `arm_controller_goto.py`: interactive dual-arm target or named-config control.
- `frame_controller_goto.py`: generic frame-task style control outside ROS action layer.
- `gravity_compensation.py`: hold posture with gravity compensation.
- `robot_record.py`: record robot states.
- `robot_replay.py`: replay recorded trajectories.
- `velocity_record.py`: measure wrist twist / velocity traces.

## Example runs

```bash
uv run h12_ros2_controller/example/arm_controller_goto.py --debug
uv run h12_ros2_controller/example/arm_controller_goto.py --sport --save record_session
uv run h12_ros2_controller/example/gravity_compensation.py
```

## Data outputs

Recorded arrays are typically written to:

- `data/control_record`
- `data/motor_record`
- `data/robot_record`
- `data/velocity_record`

Plot scripts in `h12_ros2_controller/plot` can post-process these runs.
