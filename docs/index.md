# h12_ros2_controller Documentation

A production-oriented control stack for Unitree H1.2 upper-body manipulation with ROS2 interfaces, SDK-level control, IK/collision pipelines, and operational safety guards.

## What this package provides

- ROS2 action servers for dual-arm Cartesian goals and generic frame tasks.
- ROS2 node for hand command/state integration.
- Real-time publisher/subscriber interfaces over Unitree DDS channels.
- Pinocchio-based kinematics and reduced-model IK workflow.
- Safety supervision with position/velocity/torque estop checks.
- Example scripts for direct (non-ROS) workflows and replay/record pipelines.

## Documentation map

- Start with [Installation](getting-started/installation.md).
- Follow [Hardware and Networking](getting-started/hardware-networking.md) before commanding hardware.
- Use [First Run](getting-started/first-run.md) for an end-to-end bring-up.
- Jump to [Usage Matrix](guides/usage-matrix.md) for task-to-command recipes.
- Use [ROS2 Interfaces](guides/ros2-interfaces.md) to inspect topics/actions.
- Explore [API Reference](api/index.md) for class/function-level docs with source excerpts.

## Search and discoverability

This docs site is built with full-text search enabled (top bar search) across:

- User guides
- Node/interface docs
- API reference pages
- Operations and troubleshooting

## Quick commands

```bash
# build docs dependencies
pip install -r docs/requirements.txt

# run local docs server
python -m mkdocs serve

# build static site
python -m mkdocs build
```
