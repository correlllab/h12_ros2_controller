# h12_ros2_controller

Controller of the h12 robot with ROS2 services

## Installation

- All the dependencies are listed in the `requirements.txt`\
- Version numbers are determined by installing everything through pip in a separated conda environment defined in `environment.yml`.
- Make sure you are using system-level python3, run `pip install -e .` to install all the dependencies.
- Install the Unitree Python SDK from [here](https://github.com/unitreerobotics/unitree_sdk2_python/tree/master)
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

TODO
