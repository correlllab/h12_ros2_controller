# Sample Frame Commands

## Launch

Run commands from the repository root. IK and planning settings come from the
selected YAML profile; no IK timeout or integration-step CLI flags are needed.

Hardware-free Meshcat visualizer:

```bash
uv run h12_ros2_controller/example/kinematic_plan_goto.py \
    --visualize --config sim_safety_split.yaml
```

Planned frame controller:

```bash
uv run h12_ros2_controller/example/frame_controller_plan_goto.py \
    --config safety_split.yaml \
    --failure-case /tmp/kilo/ik-failure.json
```

For local simulation, use `--config sim_safety_split.yaml` with the controller
and verify the simulation DDS domain and safety relay first. Unlike the offline
visualizer, the controller publishes motor commands.

## Input Conventions

- Positions: pelvis-relative metres.
- Orientation: roll, pitch, and yaw in degrees.
- Multiple targets: comma-separated pairs in one `frame` command, solved together.
- Frame-controller targets: actual robot frames such as `left_grasp_frame`.
- Arm-only targets: `left` and `right` have wrist-target semantics; do not copy
  grasp-frame coordinates into those entry points without accounting for the
  different target point.
- Independent comparisons: execute `home` between targets.
- Continuity testing: submit nearby targets consecutively without returning home.
- Feasibility: these are test inputs, not guarantees from every measured body pose.

The controller waits for Enter before executing a successful plan. In the
offline visualizer, answer `y` to replay a plan and update the simulated start.
The visualizer's `reset` command only redraws the current state; it does not
return the robot home.

## General Reaches

```text
frame left_grasp_frame 0.4 0.4 0.3 0 -30 0
frame right_grasp_frame 0.4 -0.4 0.3 0 -30 0
```

## Downward-Facing Reaches

```text
frame left_grasp_frame 0.47 0.14 0.34 15 60 -15
frame left_grasp_frame 0.51 0.14 0.32 15 60 -15
frame right_grasp_frame 0.47 -0.14 0.34 -15 60 15
frame right_grasp_frame 0.49 -0.18 0.28 -15 70 15
```

## Pure Pitch-Down Targets

```text
frame left_grasp_frame 0.4 0.2 0.2 0 80 0
frame left_grasp_frame 0.4 0.2 0.2 0 70 0
frame left_grasp_frame 0.4 0.3 0.2 0 70 0
frame left_grasp_frame 0.4 0.3 0.2 0 60 0
frame left_grasp_frame 0.4 0.3 0.3 0 60 0
```

## Detour and Nearby-Target Regressions

Inspect the outbound path and return to home for unnecessary rearward sweeps.
For nearby-route reuse, execute the first target, return home, then try the
second target without restarting the planner.

```text
frame left_grasp_frame 0.42 0.16 0.25 15 80 -15
frame left_grasp_frame 0.43 0.16 0.25 15 80 -15
```

## Simultaneous Bimanual Targets

Enter each command as one line. The comma requests both final frame targets in
one solve; it does not prescribe sequential motion. A validated coordinated,
staggered, or sequential trajectory may be selected.

Moderate pair:

```text
frame left_grasp_frame 0.47 0.14 0.34 15 60 -15, right_grasp_frame 0.47 -0.14 0.34 -15 60 15
```

Harder asymmetric pair:

```text
frame left_grasp_frame 0.51 0.14 0.32 15 60 -15, right_grasp_frame 0.49 -0.18 0.28 -15 70 15
```

## Combined Return Regression

Execute these in order, confirming each successful plan before entering the
next command. Inspect whether the return moves both arms together when safe.

```text
frame right_grasp_frame 0.49 -0.18 0.28 -15 70 15
frame left_grasp_frame 0.4 0.2 0.2 0 80 0
home
```

## Named Configuration Checks

```text
home
arms_front_45
home
t_pose
home
```

## Evaluation Notes

- Inspect both planning time and planned execution duration.
- Check endpoint errors, wrist margin, joint detour ratio, hand deviation,
  body-relative excursion, and minimum self-clearance where reported.
- Check both forward and return motions, especially after moving both arms.
- Treat a rejected path as a reported constraint or search failure, not as proof
  that the target is physically unreachable.
- Keep the execution clipping and collision checks enabled; an accepted path
  should already satisfy the publisher's effective position bounds.
- ROS comparisons require the same target frames, profile, installed assets,
  and measured planning start. ROS frame-task clients must select planned mode.
- A ROS `Planning replay case:` warning contains the exact planning start and
  targets for investigating a failure that differs from offline testing.
