import os
import sys
import time
import argparse

import numpy as np

sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))

from h12_ros2_controller.core.controller.arm_controller import ArmController
from h12_ros2_controller.utility.controller_config import (
    load_controller_config,
    init_channel_factory_guard,
    maybe_start_controller_logging,
)
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS
from h12_ros2_controller.utility.goto_commands import (
    FrameGoal, GotoConsole, MultiFrameGoal, NamedGoal, Quit,
)
from h12_ros2_controller.utility.path_definition import (
    URDF_MAGPIE_PATH,
    URDF_MAGPIE_SPHERE_PATH,
    SRDF_MAGPIE_SPHERE_PATH,
)


def main(config_name='safety_split.yaml'):
    # parse threshold and timeout from config
    config= load_controller_config(config_name)
    controller_cfg = config['controller']
    timeout = controller_cfg['timeout']
    threshold_ik = controller_cfg['threshold_ik']
    threshold_joint = controller_cfg['threshold_joint']
    threshold_linear = controller_cfg['threshold_linear']
    threshold_angular = controller_cfg['threshold_angular']

    # initialize channel factory
    init_channel_factory_guard(config)
    # initialize arm controller
    arm_controller = ArmController(URDF_MAGPIE_PATH,
                                   URDF_MAGPIE_SPHERE_PATH,
                                   SRDF_MAGPIE_SPHERE_PATH,
                                   init=True,
                                   handless=False,
                                   visualize=True,
                                   config=config)
    maybe_start_controller_logging(arm_controller)
    console = GotoConsole(
        'Arm Goto', NAMED_CONFIGS,
        supported=(NamedGoal, FrameGoal, MultiFrameGoal, Quit),
    )
    console.help()
    console.say('Arm names: left or right (wrist-yaw target poses)')

    try:
        while True:
            # get keyword or target poses
            try:
                command = console.read_command()
            except ValueError as err:
                print(f'Invalid goal: {err}')
                continue
            if isinstance(command, Quit):
                break
            if command is None:
                continue
            keyword = command.name if isinstance(command, NamedGoal) else ''
            tasks = (command.tasks if isinstance(command, MultiFrameGoal) else
                     (command,) if isinstance(command, FrameGoal) else ())
            if tasks and (len(tasks) > 2 or any(
                task.frame_name not in ('left', 'right') for task in tasks
            )):
                print('Arm goto supports only left and right')
                continue

            # handle keyword configuration
            if keyword in NAMED_CONFIGS:
                print(f'Going to named configuration: {keyword}')
                q_reduced = NAMED_CONFIGS[keyword]
                # use goto as step function
                step_function = lambda: arm_controller.goto_reduced_configuration(q_reduced)
                # step_function = lambda: arm_controller.sim_goto_reduced_configuration(q_reduced)
            elif keyword != '':
                print(f'Unknown keyword: {keyword}')
                continue
            else:
                # manual pose input
                print('Going to target end-effector poses')
                # keep the unspecified arm at its current pose
                targets = {task.frame_name: task.pose for task in tasks}
                arm_controller.left_ee_target_pose = targets.get(
                    'left', arm_controller.left_ee_pose,
                )
                arm_controller.right_ee_target_pose = targets.get(
                    'right', arm_controller.right_ee_pose,
                )
                step_function = lambda: arm_controller.control_step_reduced()
                # step_function = lambda: arm_controller.sim_step_reduced()

            # update ik solver with current state
            arm_controller.update_ik_solver()

            # main loop
            start_time = time.time()
            ik_converged = False
            steady_state_converged = False
            while time.time() - start_time < timeout:
                frame_start_time = time.time()
                if not ik_converged:
                    # run IK until the command stops changing
                    vel = step_function()
                    vel_error = np.max(np.abs(vel))

                    # print error for config task
                    if keyword in NAMED_CONFIGS:
                        error = np.max(
                            np.abs(arm_controller.reduced_configuration_error)
                        )
                        print(
                            f'Configuration Error: {error:.4f}, '
                            f'IK Velocity: {vel_error:.4f}'
                        )
                    # print errors for end-effector pose task
                    else:
                        left_error_linear = np.linalg.norm(
                            arm_controller.left_ee_error[:3]
                        )
                        left_error_angular = np.linalg.norm(
                            arm_controller.left_ee_error[3:]
                        )
                        right_error_linear = np.linalg.norm(
                            arm_controller.right_ee_error[:3]
                        )
                        right_error_angular = np.linalg.norm(
                            arm_controller.right_ee_error[3:]
                        )

                        print(f'Left Error Linear: {left_error_linear:.4f}, '
                              f'Left Error Angular: {left_error_angular:.4f}, '
                              f'Right Error Linear: {right_error_linear:.4f}, '
                              f'Right Error Angular: {right_error_angular:.4f}, '
                              f'IK Velocity: {vel_error:.4f}')

                    if vel_error < threshold_ik:
                        ik_converged = True
                        arm_controller.init_steady_state()
                        print('IK converged; entering steady-state hold')
                else:
                    steady_state_converged = arm_controller.steady_state_step(
                        threshold_joint
                    )
                    if keyword in NAMED_CONFIGS:
                        error = np.max(np.abs(arm_controller.reduced_configuration_error))
                        print(f'Configuration Error: {error:.4f}')
                        steady_state_converged = steady_state_converged or error < threshold_joint
                    else:
                        left_error_linear = np.linalg.norm(arm_controller.left_ee_error[:3])
                        left_error_angular = np.linalg.norm(arm_controller.left_ee_error[3:])
                        right_error_linear = np.linalg.norm(arm_controller.right_ee_error[:3])
                        right_error_angular = np.linalg.norm(arm_controller.right_ee_error[3:])
                        print(f'Left Error Linear: {left_error_linear:.4f}, '
                              f'Left Error Angular: {left_error_angular:.4f}, '
                              f'Right Error Linear: {right_error_linear:.4f}, '
                              f'Right Error Angular: {right_error_angular:.4f}')
                        reached = {
                            'left': (
                                left_error_linear < threshold_linear and
                                left_error_angular < threshold_angular
                            ),
                            'right': (
                                right_error_linear < threshold_linear and
                                right_error_angular < threshold_angular
                            ),
                        }
                        steady_state_converged = all(
                            reached[task.frame_name] for task in tasks
                        )

                    if steady_state_converged:
                        print('Steady state reached!')
                        break

                time.sleep(max(0.0, arm_controller.dt - (time.time() - frame_start_time)))

            if not ik_converged:
                print('Timed out before IK convergence')
            elif not steady_state_converged:
                print('Timed out during steady-state hold')
            else:
                print('Target reached!')

            if keyword in NAMED_CONFIGS:
                error = arm_controller.reduced_configuration_error
                print(f'Final Configuration Error: {np.max(np.abs(error)):.4f}')
            else:
                left_error_linear = np.linalg.norm(arm_controller.left_ee_error[:3])
                left_error_angular = np.linalg.norm(arm_controller.left_ee_error[3:])
                right_error_linear = np.linalg.norm(arm_controller.right_ee_error[:3])
                right_error_angular = np.linalg.norm(arm_controller.right_ee_error[3:])
                print(f'Final Left Error Linear: {left_error_linear:.4f}, '
                    f'Final Left Error Angular: {left_error_angular:.4f}, '
                    f'Final Right Error Linear: {right_error_linear:.4f}, '
                    f'Final Right Error Angular: {right_error_angular:.4f}')

            input('Press any key to continue...') # flush the input buffer
            cont = input('Do you want to send another goal? (y/n): ').lower()
            if cont != 'y':
                break
    except Exception as e:
        print(f'Exception occurred: {e}')
    finally:
        print('Shutting down...')
        arm_controller.shutdown()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Arm Controller Goto')
    parser.add_argument('--config', type=str, default='safety_split.yaml', help='YAML file name under config/')
    args = parser.parse_args()
    main(config_name=args.config)
