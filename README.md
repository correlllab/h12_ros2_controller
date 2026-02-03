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

## Python Dependencies

- If using conda, create a conda environment from `environment.yml`.

    ```bash
    conda env create -f environment.yml
    ```

- If using pip (e.g. for system Python and ROS), install dependencies from `requirements.txt`.

    ```bash
    pip install -e .
    ```

## Files

- `assets/` contains robot description files. Only used when running the programs without ros environment.
- `data/` contains saved data such as joint positions.
- `figures` contains generated figures.
- `h12_ros2_controller/`: contains source code.
    - `core/`: contains the core implementaiton of robot kinematics solver, controller and communication interface.
        - `controller/` contains different controllers.
            - `hand_controller.py` tracks hand states and commands finger positions.
            - `upper_controller.py` is the base class for upper body controllers (ArmController, FrameController, ImpedanceController, GravityCompController).
            - `arm_controller.py` provides functions to control end-effectors and query their states.
            - `frame_controller.py` provides functions to add frame tasks for any frame and move the upper body accordingly.
            - `gravity_comp_controller.py` provides functions to fix upper body in any fixed pose using I-control.
            - `impedance_controller.py` provides functions to control end-effectors with impedance.
        - `channel_interface.py` implements a publisher for motor commands and a subscriber for motor states using the Unitree Python SDK.
        - `ik_solver` solves inverse kinematics with self-collision avoidance.
        - `low_cmd_handler` sends command through `low_cmd_publisher` and monitors robot states from `robot_model` to trigger active e-stop.
        - `robot_model.py` tracks robot states and provides useful functions for kinematics, Jacobians, etc., using Pinocchio.
    - `plot/` contains scripts to plot figures.
    - `example/` contains example scripts using core functionalities without ros dependencies.
    - `utility/` contains useful scripts to inspect robot descriptions, process collision pairs, and inspect joint velocities.
    - Python files in the root folder are ros nodes.

## Usage

### `arm_controller_goto.py`

- Run arm controller that prompt for target end-effector positions in command line.

    ```bash
    # choose --sport or --debug depending on robot states
    python h12_ros2_controller/arm_controller_goto.py --sport
    python h12_ros2_controller/arm_controller_goto.py --debug
    ```

- Change `visualize=True` when initializing the controller to have the meshcat visualization

    ```python
    arm_controller = ArmController('assets/h1_2/h1_2.urdf',
                                   'assets/h1_2/h1_2_sphere.urdf',
                                   'assets/h1_2/h1_2_sphere_collision.srdf',
                                   dt=0.02,
                                   visualize=True, # use this to turn on meshcat visualization
                                   sport_mode=False)
    ```

- Change `arm_controller.control_step_reduced()` to `arm_controller.sim_step_reduced()`
    to directly integrate the control input to the pinocchio model.
    This is useful when checking IK results without running the full dynamics in simulation
    or on real robot.

- Example usage with mujoco sim:
    - Launch [mujoco simulation](https://github.com/HIRO-group/h1_mujoco) using `python h12_mujoco.py`.
    - Launch arm controller in debug mode using `python h12_ros2_controller/arm_controller_goto.py --debug`.

    ![Mujoco Example](./figures/Mujoco_Example.png)
