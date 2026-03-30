# Installation

## Prerequisites

- Linux workstation with Python 3.10+
- ROS2 workspace tooling (`colcon`, sourced ROS2 setup)
- Network access to robot controller when using hardware

## Dependencies

This package depends on:

- Unitree Python SDK in submodule: `submodules/unitree_sdk2_python`
- ROS2 packages:
  - `h12_ros2_model`
  - `custom_ros_messages`

## Option A: `uv` (recommended)

```bash
git submodule update --init --recursive
uv sync
```

Run scripts with:

```bash
uv run h12_ros2_controller/example/arm_controller_goto.py --debug
```

## Option B: `pip`

```bash
git submodule update --init --recursive
pip install -r requirements.txt
```

## ROS2 workspace setup

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# place or clone h12_ros2_controller here

# also place h12_ros2_model and custom_ros_messages under src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Verify package registration

```bash
ros2 pkg list | grep h12_ros2_controller
ros2 pkg executables h12_ros2_controller
```

Expected executables include:

- `joint_state_publisher`
- `dual_arm_server`
- `dual_arm_client`
- `frame_task_server`
- `frame_task_client`
- `hand_controller_node`
- `hand_cmd_gui`
