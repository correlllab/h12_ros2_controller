# System Overview

## Layered architecture

1. ROS2 layer
- Node wrappers for actions/topics.
- Launch composition for runtime bring-up.

2. Controller layer
- `ArmController`, `FrameController`, `HandController`, `UpperController`.
- High-level commands translated into IK and command targets.

3. IK and model layer
- `IKSolver` task graph and reduced-state solve loop.
- `RobotModel` kinematics, Jacobians, frame transforms, dynamics helpers.

4. Runtime command layer
- `LowCmdHandler` safety checker and recording.
- `ChannelInterface` publishers/subscribers to Unitree DDS topics.

5. Hardware layer
- Unitree control buses and hand interfaces.

## Runtime loops

- Joint and EE state publish loop: typically 100 Hz.
- Command publish loop: typically 500 Hz (`dt=0.002`) in command publisher.
- Safety checker loop: typically 1 kHz (`checker_dt=0.001`).

## Key design choices

- Reduced model for upper-body IK with selective joints.
- Explicit velocity limiting at joint and EE twist levels.
- Separation between ROS2 API and core controller logic.
- Continuous safety checks independent of client call paths.
