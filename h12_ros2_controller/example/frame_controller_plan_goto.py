import time
import argparse
import json
from pathlib import Path
import numpy as np

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.controller.frame_controller import FrameController
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


def input_keyword_or_frame_task():
    '''Ask user for a named config or one manual frame task'''
    print(f'Available named configs: {list(NAMED_CONFIGS.keys())}')
    choice = input(
        'Enter named config (or press Enter for frame task): '
    ).strip()
    if choice:
        return choice, None, None

    frame_name, pose = input_frame_task()
    return '', frame_name, pose


def input_frame_task():
    '''Get frame name and pose from user'''
    frame_name = input('Enter frame name: ').strip()
    while True:
        input_pose = input('Enter x y z roll pitch yaw (degrees): ')
        parts = input_pose.strip().split()
        if len(parts) != 6:
            print('Invalid input. Please enter exactly 6 values.')
            continue

        try:
            values = [float(val) for val in parts]
            values[3:] = np.deg2rad(values[3:])
            return frame_name, np.array(values)
        except ValueError:
            print('Invalid input. Make sure all 6 values are numeric.')


def init_frame_controller(config_name):
    '''Initialize controller, DDS channel, and Meshcat visualizer'''
    config = load_controller_config(config_name)
    init_channel_factory_guard(config)
    frame_controller = FrameController(
        URDF_MAGPIE_PATH,
        URDF_MAGPIE_SPHERE_PATH,
        SRDF_MAGPIE_SPHERE_PATH,
        init=True,
        handless=False,
        visualize=True,
        config=config,
    )
    maybe_start_controller_logging(frame_controller)
    return frame_controller


def plan_goal(
    frame_controller,
    keyword,
    frame_name,
    pose,
    ik_timeout,
    ik_alpha,
    tasks=None,
):
    '''Plan one named config or frame-task goal'''
    start_time = time.perf_counter()

    # named configs plan directly in reduced joint space; the planner still
    # applies the global z constraint from config
    if keyword in NAMED_CONFIGS:
        print(f'Planning to named configuration: {keyword}')
        path = frame_controller.plan_to_configuration(
            NAMED_CONFIGS[keyword],
        )
    elif keyword != '':
        raise ValueError(f'Unknown named configuration: {keyword}')
    else:
        # frame goals are configured by FrameController, then solved by IK
        tasks = tasks if tasks is not None else (FrameGoal(frame_name, pose),)
        print('Planning to frame tasks: ' + ', '.join(task.frame_name for task in tasks))
        frame_controller.clear_frame_tasks()
        for task in tasks:
            frame_controller.add_frame_task(
                f'{task.frame_name}_task', task.frame_name, task.pose,
            )
        path = frame_controller.plan_to_ik_target(
            ik_timeout=ik_timeout,
            ik_alpha=ik_alpha,
        )
    planning_time = time.perf_counter() - start_time
    return path, planning_time


def hold_steady_state(frame_controller, timeout, threshold, q_reduced=None):
    '''Hold until the measured requested goal converges, is blocked, or times out'''
    frame_controller.init_steady_state()
    controller_cfg = frame_controller.config['controller']
    start_time = time.monotonic()
    while time.monotonic() - start_time < timeout:
        step_start = time.monotonic()
        frame_controller.steady_state_step(threshold)
        if frame_controller.steady_state_limited_joints:
            print('Hold blocked at a joint position limit')
            return False
        if q_reduced is not None:
            joint_error = np.max(np.abs(
                frame_controller.robot_model.state_reduced['q'] - q_reduced
            ))
            converged = joint_error < threshold
        else:
            errors = frame_controller.get_frame_task_errors()
            converged = bool(errors) and all(
                np.linalg.norm(error[:3]) < controller_cfg['threshold_linear'] and
                np.linalg.norm(error[3:]) < controller_cfg['threshold_angular']
                for error in errors.values()
            )
        if converged:
            print('Steady state reached!')
            return True
        time.sleep(max(0.0, frame_controller.dt - (time.monotonic() - step_start)))
    print('Timed out during steady-state hold')
    return False


def main(config_name='safety_split.yaml', failure_case=None):
    frame_controller = init_frame_controller(config_name)
    controller_cfg = frame_controller.config['controller']
    steady_timeout = controller_cfg['timeout']
    threshold_joint = controller_cfg['threshold_joint']
    console = GotoConsole(
        'Frame Plan Goto', NAMED_CONFIGS,
        supported=(NamedGoal, FrameGoal, MultiFrameGoal, Quit),
    )
    console.help()
    planner_cfg = frame_controller.planner.planner_config
    ik_timeout = planner_cfg.ik_total_timeout
    ik_alpha = planner_cfg.ik_alpha
    console.say(f'IK budget: {ik_timeout:.2f}s total, '
                f'{min(ik_timeout, planner_cfg.ik_seed_timeout):.2f}s per seed; '
                'timeouts do not establish infeasibility')

    try:
        while True:
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
                     (command,) if isinstance(command, FrameGoal) else None)
            frame_controller.last_planning_start = None
            try:
                # plan and visualize first; execution waits for operator input
                path, planning_time = plan_goal(
                    frame_controller,
                    keyword,
                    tasks[0].frame_name if tasks else None,
                    tasks[0].pose if tasks else None,
                    ik_timeout=ik_timeout,
                    ik_alpha=ik_alpha,
                    tasks=tasks,
                )
            except Exception as err:
                print(f'Planning failed: {err}')
                if (failure_case is not None and tasks and
                        frame_controller.last_planning_start is not None):
                    case = {
                        'config': config_name,
                        'start_q': frame_controller.last_planning_start.tolist(),
                        'tasks': [{'frame': task.frame_name, 'pose': task.pose.tolist()}
                                  for task in tasks],
                        'ik_timeout': ik_timeout, 'ik_alpha': ik_alpha,
                    }
                    try:
                        Path(failure_case).write_text(json.dumps(case, indent=2) + '\n')
                        console.say(f'Exact failed planning start saved to {failure_case}')
                    except OSError as save_error:
                        console.say(f'Cannot save failure case: {save_error}')
                continue

            print(f'Planning time: {planning_time:.4f} s')
            if tasks and getattr(frame_controller, 'last_ik_result', None) is not None:
                ik_elapsed = sum(
                    attempt.elapsed_time
                    for attempt in frame_controller.last_ik_result.attempts
                )
                candidate_elapsed = sum(
                    attempt['wall_time']
                    for attempt in getattr(frame_controller, 'last_plan_attempts', ())
                )
                print(f'IK attempts: {ik_elapsed:.3f}s, '
                      f'planner requests: {candidate_elapsed:.3f}s')
            print(f'Path shape: {path.shape}')
            result = getattr(frame_controller, 'last_plan_result', None)
            if result is not None and result.success:
                metadata = result.metadata
                print(f'Plan type: {metadata.get("path_kind", result.reason)}, '
                      f'coordination: {metadata.get("coordination", "not applicable")}, '
                      f'duration: {metadata.get("duration", 0.0):.3f}s')
            frame_controller.visualize_path(path)

            input('Press ENTER to execute the plan...')
            frame_controller.execute_path(path)
            print('Plan execution finished')

            # hold the final config with steady-state I control
            hold_steady_state(
                frame_controller,
                steady_timeout,
                threshold_joint,
                q_reduced=NAMED_CONFIGS[keyword] if keyword else None,
            )

            cont = input('Do you want to send another goal? (y/n): ').lower()
            if cont != 'y':
                break
    except KeyboardInterrupt:
        print('Interrupted')
    finally:
        print('Shutting down...')
        frame_controller.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Frame Controller Plan Goto')
    parser.add_argument(
        '--config',
        type=str,
        default='safety_split.yaml',
        help='YAML file name under config/',
    )
    parser.add_argument('--failure-case', help='Save the latest failed frame request as replayable JSON')
    args = parser.parse_args()
    main(
        config_name=args.config,
        failure_case=args.failure_case,
    )
