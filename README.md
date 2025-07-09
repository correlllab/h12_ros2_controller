# h12_ros2_controller

Controller of the h12 robot with ROS2 services

## Installation

- All the dependencies are listed in the `requirements.txt`\
- Version numbers are determined by installing everything through pip in a separated conda environment defined in `environment.yml`.
- Make sure you are using system-level python3, run `pip install -e .` to install all the dependencies.
- Install the [Unitree Python SDK](https://github.com/unitreerobotics/unitree_sdk2_python/tree/master)
  so the code can communicate with the robot.
- This package depends on two additonal ROS2 packages that holds messages and robot description.
    - [h12_ros2_model](https://github.com/correlllab/h12_ros2_model)
    - [custom_ros_messages](https://github.com/correlllab/custom_ros_messages)

## ROS Setup

- Create an empty ros worksapce and place this repo under `src`.
- The program uses customized message definions and that part can only be built by `ament_cmake`.
- Copy the `controller_msgs` into `src` directory as well, so ROS can build the message package separately.
- Run `colcon build` and then `source ./install/setup.bash` to build the project.

## Files

- `assets/` contains robot description files. Only used when running the programs without ros environment.
- `data/` contains saved data such as joint positions.
- `h12_ros2_controller/`:
    - `core/`: contains the core implementaiton of robot kinematics solver, controller and communication interface.
        - `arm_controller.py` solves inverse kinematics and provides functions to control end-effectors and query their states.
        - `channel_interface.py` implements a publisher for motor commands and a subscriber for motor states using the Unitree Python SDK.
        - `hand_controller.py` tracks hand states and commands finger positions.
        - `robot_model.py` tracks robot states and provides useful functions for kinematics, Jacobians, etc., using Pinocchio.
    - `example/` contains example scripts using core functionalities without ros dependencies.
    - `utility/` contains useful scripts to inspect robot descriptions, process collision pairs, and inspect joint velocities.
    - Python files in the folder are ros nodes.
