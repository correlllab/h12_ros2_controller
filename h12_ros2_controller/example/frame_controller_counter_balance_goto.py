'''Interactive frame goto demo with optional planning and counter-balance

The command relay and lower-body policy are external. Only one upper controller
and one upper-command publisher may run on this DDS domain at a time.
'''

import argparse
import sys
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.append(str(Path(__file__).resolve().parents[2]))

from h12_ros2_controller.core.controller.counter_balance.counter_ddp_velocity_robust_controller import (
    CounterDDPVelocityRobustController,
)
from h12_ros2_controller.core.controller.upper_controller import UpperController
from h12_ros2_controller.core.planner import PlannerClient
from h12_ros2_controller.utility.controller_config import (
    init_channel_factory_guard,
    load_controller_config,
    maybe_start_controller_logging,
)
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS, UPPER_BODY_INDEX
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS
from h12_ros2_controller.utility.path_definition import (
    SRDF_MAGPIE_SPHERE_PATH,
    URDF_MAGPIE_PATH,
    URDF_MAGPIE_SPHERE_PATH,
)


FRAMES = {'left': 'left_grasp_frame', 'right': 'right_grasp_frame'}
OVERHANG_LEFT = (
    np.array([-0.618233536, 0.977270438, -1.599421561, -0.597744255,
              -2.019447514, -0.018350751, 1.166999901]),
    np.array([-0.942310361, 0.800733383, -1.519986439, 0.052423475,
              -2.291783074, 0.359499901, 0.911634444]),
    np.array([-0.882098306, 0.970014968, -1.568081280, 0.253477573,
              -2.205511689, 0.319163905, 0.938871233]),
    np.array([-1.096290573, 1.318556519, -1.489798025, -0.163479871,
              -2.400130986, 0.359499381, 1.042948613]),
)
OVERHANG_MIRROR = np.array([1, -1, -1, 1, -1, 1, -1])
OVERHANG_CONFIGS = {
    **{f'overhang_left_{i}': np.r_[q, np.zeros(7)]
       for i, q in enumerate(OVERHANG_LEFT, start=1)},
    **{f'overhang_right_{i}': np.r_[np.zeros(7), q * OVERHANG_MIRROR]
       for i, q in enumerate(OVERHANG_LEFT, start=1)},
}
DEMO_CONFIGS = {**NAMED_CONFIGS, **OVERHANG_CONFIGS}


class DemoController(CounterDDPVelocityRobustController):
    def __init__(self, *args, **kwargs):
        self.plain_goto = False
        self.balanced_live = False
        self.planned_balance = False
        super().__init__(*args, **kwargs)

    def _apply_velocity_command(self, velocity):
        if not self.balanced_live:
            return super()._apply_velocity_command(velocity)
        self.ik_solver.integrate(velocity)
        self.control_configuration_step(
            self.ik_solver.q[self.arm_ids], velocity[self.arm_ids], balance_scale=1.0,
        )
        self.update_robot_model()
        return velocity

    def _write_arm_command(self, arm_q, arm_dq, tau):
        if self.planned_balance:
            if not valid_arm_command(self, arm_q):
                raise ValueError('Combined planned/counter-arm command is invalid; holding')
        # retain velocity for balance prediction, not moving-arm motor feedforward
        arm_dq = np.copy(arm_dq)
        arm_dq[self.moving_local] = 0.0
        return super()._write_arm_command(arm_q, arm_dq, tau)

    def _record_moving_command_error(self, arm_q, arm_dq):
        motor_dq = np.copy(arm_dq)
        motor_dq[self.moving_local] = 0.0
        super()._record_moving_command_error(arm_q, motor_dq)

    def _publish_position_command(self, q, dq, tau):
        if self.plain_goto:
            if self.balanced_live:
                # the I-stage holds the last counter command while correcting the moving arm
                q_hold = self.low_cmd_handler.q_cmd.copy()
                dq_hold = np.zeros_like(q_hold)
                tau_hold = self.low_cmd_handler.tau_cmd.copy()
                q_hold[self.moving_ids] = q[self.moving_ids]
                tau_hold[self.moving_ids] = tau[self.moving_ids]
                return UpperController._publish_position_command(
                    self, q_hold, dq_hold, tau_hold,
                )
            return UpperController._publish_position_command(self, q, dq, tau)
        return super()._publish_position_command(q, dq, tau)

    def _steady_state_control_ids(self):
        if self.plain_goto:
            if self.balanced_live:
                return self.moving_ids
            return self.upper_ids
        return super()._steady_state_control_ids()


def ask_choice(prompt, choices):
    while True:
        value = input(prompt).strip().lower()
        if value in choices:
            return value
        options = ', '.join('Enter' if not choice else choice for choice in choices)
        print(f'Invalid input. Choose one of: {options}.')


def ask_pose():
    while True:
        parts = input('Enter x y z roll pitch yaw (degrees): ').split()
        if len(parts) != 6:
            print('Invalid input. Please enter exactly 6 values.')
            continue
        try:
            pose = np.asarray([float(part) for part in parts])
            if not np.isfinite(pose).all():
                raise ValueError('values must be finite')
            pose[3:] = np.deg2rad(pose[3:])
            return pose
        except ValueError:
            print('Invalid input. Make sure all 6 values are finite numbers.')


def ask_z_floor(phase):
    enabled = ask_choice(
        f'Enforce grasp-frame minimum z during {phase} planning? (Y/n): ',
        ('y', 'n', ''),
    ) != 'n'
    if not enabled:
        print('Absolute grasp-frame z floor disabled for this plan; '
              'joint limits, self-collision, and endpoint-derived z corridor remain active.')
    return enabled


def select_planner(controller, planners, z_floor):
    if z_floor:
        planner = planners[True]
    else:
        if False not in planners:
            original = planners[True]
            planners[False] = PlannerClient(
                original.urdf_path,
                original.urdf_sphere_path,
                original.srdf_sphere_path,
                planner_config=replace(original.planner_config, frame_z_min=None),
                handless=original.handless,
            )
        planner = planners[False]
    controller.planner = planner


def fresh_state(controller):
    q = np.array(controller.robot_model.state['q'], dtype=float, copy=True)
    if q.shape != (27,) or not np.isfinite(q).all():
        raise ValueError('Measured motor positions must be finite')
    return q


def hold(controller):
    if controller.low_cmd_handler._estopped:
        return
    ids = UPPER_BODY_INDEX
    controller.low_cmd_handler.set_joint_commands(
        q=controller.low_cmd_handler.q_cmd[ids],
        dq=np.zeros(len(ids)),
        tau=controller.low_cmd_handler.tau_cmd[ids],
        joint_ids=ids,
    )


def valid_arm_command(controller, arm_q):
    candidate = fresh_state(controller)
    candidate[controller.arm_ids] = arm_q
    return (controller.robot_model.check_within_limits_reduced(arm_q)
            and controller.robot_model.check_collision_free(candidate))


def send(controller, q, dq, balanced, check_collision=False):
    measured = fresh_state(controller)
    q = np.asarray(q, dtype=float)
    dq = np.asarray(dq, dtype=float)
    if (q.shape != (27,) or dq.shape != (27,)
            or not np.isfinite(q).all() or not np.isfinite(dq).all()):
        raise ValueError('Command must contain finite motor positions and velocities')
    if check_collision:
        reduced = q[controller.robot_model.reduced_mask]
        if not controller.robot_model.check_within_limits_reduced(reduced):
            raise ValueError('Planned command exceeds reduced joint position limits')
        if not controller.robot_model.check_collision_free_reduced(reduced):
            raise ValueError('Planned command fails self-collision check')
    if balanced:
        controller.control_configuration_step(
            q[controller.arm_ids], dq[controller.arm_ids], balance_scale=1.0,
        )
        return
    ids = UPPER_BODY_INDEX
    tau = np.asarray(controller.robot_model.dynamics.get_gravity_compensation(measured))
    if tau.shape != (27,) or not np.isfinite(tau[ids]).all():
        raise ValueError('Invalid gravity compensation')
    controller.low_cmd_handler.set_joint_commands(
        q=q[ids], dq=np.zeros(len(ids)), tau=tau[ids], joint_ids=ids,
    )


def check_path(controller, path):
    path = np.asarray(path, dtype=float)
    start = fresh_state(controller)[controller.robot_model.reduced_mask]
    if (path.ndim != 2 or path.shape[1:] != start.shape
            or len(path) < 2 or not np.isfinite(path).all()):
        raise ValueError('Planner returned an invalid reduced path')
    if np.max(np.abs(path[0] - start)) > 0.05:
        raise RuntimeError('Robot moved since planning; replan from current state')
    step_limit = controller.dq_lim * controller.dt
    if not np.isfinite(step_limit) or step_limit <= 0:
        raise ValueError('Controller joint speed limit and period must be positive')
    retimed = [path[0]]
    for first, last in zip(path[:-1], path[1:]):
        steps = max(1, int(np.ceil(np.max(np.abs(last - first)) / step_limit - 1e-9)))
        retimed.extend(first + (last - first) * (index / steps)
                       for index in range(1, steps + 1))
    path = np.asarray(retimed)
    dq = np.diff(path, axis=0) / controller.dt
    if np.max(np.abs(dq)) > controller.dq_lim + 1e-6:
        raise ValueError('Retimed path exceeds controller joint speed limit')
    for waypoint in path:
        if (not controller.robot_model.check_within_limits_reduced(waypoint)
                or not controller.robot_model.check_collision_free_reduced(waypoint)):
            raise ValueError('Planned path has an invalid waypoint')
    return path, dq


def execute_path(controller, path, balanced=False):
    path, velocities = check_path(controller, path)
    controller.planned_balance = balanced
    try:
        for waypoint, velocity in zip(path[1:], velocities):
            started = time.monotonic()
            q = fresh_state(controller)
            q[controller.robot_model.reduced_mask] = waypoint
            dq = np.zeros(27)
            dq[controller.robot_model.reduced_mask] = velocity
            send(controller, q, dq, balanced, check_collision=True)
            time.sleep(max(0.0, controller.dt - (time.monotonic() - started)))
    finally:
        controller.planned_balance = False
        hold(controller)


def recover_counter_arm(controller):
    fresh_state(controller)
    ids = controller.counter_arm_ids
    start_q = controller.low_cmd_handler.q_cmd[ids].copy()
    duration = max(8.0, 1.875 * np.max(np.abs(start_q)) / 0.25)
    # check the commanded return with the moving arm fixed before starting it
    for progress in np.linspace(0.0, 1.0, max(2, int(np.ceil(duration / controller.dt)) + 1)):
        q = controller.low_cmd_handler.q_cmd.copy()
        fade = progress**3 * (10.0 + progress * (-15.0 + 6.0 * progress))
        q[ids] = (1.0 - fade) * start_q
        if not valid_arm_command(controller, q[controller.arm_ids]):
            print('Counter-arm return blocked by joint/collision checks; holding current posture')
            hold(controller)
            return False
    print(f'Fading counter arm to zero over {duration:.1f} s...')
    started_at = time.monotonic()
    try:
        while time.monotonic() - started_at < duration:
            tick = time.monotonic()
            measured = fresh_state(controller)
            progress = min(1.0, (tick - started_at) / duration)
            fade = progress**3 * (10.0 + progress * (-15.0 + 6.0 * progress))
            q = (1.0 - fade) * start_q
            candidate = controller.low_cmd_handler.q_cmd.copy()
            candidate[ids] = q
            if not valid_arm_command(controller, candidate[controller.arm_ids]):
                print('Counter-arm return blocked by current posture; holding')
                return False
            gravity = np.asarray(controller.robot_model.dynamics.get_gravity_compensation(measured))
            if gravity.shape != (27,) or not np.isfinite(gravity[ids]).all():
                raise ValueError('Invalid gravity compensation during counter-arm fade')
            controller.low_cmd_handler.set_joint_commands(
                q=q, dq=np.zeros(len(ids)), tau=gravity[ids], joint_ids=ids,
            )
            time.sleep(max(0.0, controller.dt - (time.monotonic() - tick)))
        measured = fresh_state(controller)
        candidate = controller.low_cmd_handler.q_cmd.copy()
        candidate[ids] = 0.0
        if not valid_arm_command(controller, candidate[controller.arm_ids]):
            print('Counter-arm return blocked by current posture; holding')
            return False
        gravity = np.asarray(controller.robot_model.dynamics.get_gravity_compensation(measured))
        if gravity.shape != (27,) or not np.isfinite(gravity[ids]).all():
            raise ValueError('Invalid gravity compensation during counter-arm fade')
        controller.low_cmd_handler.set_joint_commands(
            q=np.zeros(len(ids)), dq=np.zeros(len(ids)), tau=gravity[ids], joint_ids=ids,
        )
        error = np.max(np.abs(fresh_state(controller)[controller.counter_arm_ids]))
        print(f'Counter-arm zero-posture error: {error:.4f} rad')
        return error < 0.1
    finally:
        hold(controller)


def wait_for_target(controller, target, balanced=False, frame_task=None):
    controller.plain_goto = True
    controller.balanced_live = balanced
    try:
        controller.init_steady_state()
        return settle_target(controller, target, balanced, frame_task)
    finally:
        controller.plain_goto = False
        controller.balanced_live = False


def settle_target(controller, target, balanced, frame_task):
    timeout = controller.config['controller']['timeout']
    threshold = controller.config['controller']['threshold_joint']
    end = time.monotonic() + timeout
    error_description = 'unknown'
    while time.monotonic() < end:
        started = time.monotonic()
        controller.steady_state_step(threshold)
        fresh_state(controller)
        if frame_task:
            error = controller.get_frame_task_error(frame_task)
            settings = controller.config['controller']
            linear_error = np.linalg.norm(error[:3])
            angular_error = np.linalg.norm(error[3:])
            reached = (linear_error < settings['threshold_linear']
                       and angular_error < settings['threshold_angular'])
            error_description = (f'linear={linear_error:.4f} m, '
                                 f'angular={angular_error:.4f} rad')
        else:
            current = controller.robot_model.state_reduced['q']
            if balanced:
                ids = slice(0, 7) if controller.moving_arm == 'left' else slice(7, 14)
                joint_error = np.max(np.abs(current[ids] - target[ids]))
            else:
                joint_error = np.max(np.abs(current - target))
            reached = joint_error < threshold
            error_description = f'max joint error={joint_error:.4f} rad'
        if reached:
            return True
        time.sleep(max(0.0, controller.dt - (time.monotonic() - started)))
    print(f'Position not fully settled ({error_description}); holding final command')
    return False


def live_goto(controller, target, balanced, frame_task=None):
    settings = controller.config['controller']
    controller.plain_goto = True
    controller.balanced_live = balanced
    tick_count = 0
    late_ticks = 0
    max_compute = 0.0
    statuses = {}
    try:
        controller.update_ik_solver()
        start = time.monotonic()
        ik_converged = False
        while time.monotonic() - start < settings['timeout']:
            tick = time.monotonic()
            reached = False
            if not ik_converged:
                if frame_task:
                    velocity = controller.control_step_reduced(com=False)
                else:
                    velocity = controller.goto_reduced_configuration(target)
                if balanced:
                    status = getattr(controller, 'latest_status', 'unknown')
                    statuses[status] = statuses.get(status, 0) + 1
                if np.max(np.abs(velocity)) < settings['threshold_ik']:
                    ik_converged = True
                    controller.init_steady_state()
                    print('IK converged; entering steady-state hold')
            else:
                steady = controller.steady_state_step(settings['threshold_joint'])
                if frame_task:
                    error = controller.get_frame_task_error(frame_task)
                    steady = steady or (
                        np.linalg.norm(error[:3]) < settings['threshold_linear']
                        and np.linalg.norm(error[3:]) < settings['threshold_angular']
                    )
                else:
                    error = controller.reduced_configuration_error
                    if balanced:
                        ids = slice(0, 7) if controller.moving_arm == 'left' else slice(7, 14)
                        error = error[ids]
                    steady = steady or np.max(np.abs(error)) < settings['threshold_joint']
                if steady:
                    print('Steady state reached!')
                    reached = True
            compute = time.monotonic() - tick
            tick_count += 1
            max_compute = max(max_compute, compute)
            late_ticks += compute > controller.dt
            if reached:
                return True
            time.sleep(max(0.0, controller.dt - compute))
        print('Timed out during steady-state hold' if ik_converged
              else 'Timed out before IK convergence')
        return False
    finally:
        if tick_count:
            mode = 'Balanced' if balanced else 'Plain'
            print(f'{mode} IK timing: max compute {max_compute * 1000:.1f} ms, '
                  f'{late_ticks}/{tick_count} ticks over {controller.dt * 1000:.1f} ms')
        if balanced:
            print(f'Counter statuses during motion: {statuses}')
        controller.balanced_live = False
        controller.plain_goto = False
        if balanced:
            hold(controller)


def prepare_goal(controller, name, side, pose, balanced):
    controller.clear_frame_tasks()
    if name is None:
        frame = FRAMES[side]
        task = f'{side}_grasp_task'
        if balanced:
            controller.set_frame_target(frame, pose, task_name=task)
        else:
            controller.add_frame_task(task, frame, pose)
        return None, task
    if name not in DEMO_CONFIGS:
        raise ValueError(f'Unknown configuration: {name}')
    target = np.asarray(DEMO_CONFIGS[name]).copy()
    if balanced:
        if name in OVERHANG_CONFIGS:
            controller._set_arm_ownership('left' if name.startswith('overhang_left') else 'right')
        else:
            try:
                target = controller.set_named_target(name).copy()
            except ValueError as error:
                raise ValueError(
                    f'{error}. Use a left/right frame target for counter-balance; '
                    'use counter-balance off for two-arm named configurations or home'
                ) from error
        measured = fresh_state(controller)[controller.robot_model.reduced_mask]
        inactive = slice(7, 14) if controller.moving_arm == 'left' else slice(0, 7)
        target[inactive] = measured[inactive]
    return target, None


def plan_goal(controller, target, frame_task, ik_timeout, ik_alpha):
    if frame_task:
        return controller.plan_to_ik_target(
            ik_timeout=ik_timeout, ik_alpha=ik_alpha,
        )
    return controller.plan_to_configuration(target)


def startup_home(controller):
    fresh_state(controller)
    start = controller.robot_model.state_reduced['q']
    started = time.monotonic()
    controller.plan_to_configuration(start, start=start)
    print(f'Planner warm-up: {time.monotonic() - started:.3f} s '
          '(process/model startup and direct-path check)')
    started = time.monotonic()
    path = controller.plan_to_configuration(NAMED_CONFIGS['home'])
    print(f'Direct-home planning: {time.monotonic() - started:.3f} s')
    execute_path(controller, path)
    wait_for_target(controller, NAMED_CONFIGS['home'])
    torso_id = BODY_JOINTS.index('torso_joint')
    torso_init = float(fresh_state(controller)[torso_id])
    controller._move_torso_to_target(torso_init, controller.torso_target)
    print('HOME and torso commands complete. '
          'Start the external lower-body policy and verify standing.')


def main(config_name='real_benchmark_upper.yaml', ik_timeout=1.0, ik_alpha=0.1):
    config = load_controller_config(config_name)
    if config.get('mode') != 'debug' or config['topics']['low_cmd'] != 'rt/safety/lowcmd_upper_in':
        raise ValueError('Demo requires the split upper-body command relay config')
    if not config.get('reactive_counter_balance'):
        raise ValueError('Demo requires an explicit real reactive_counter_balance config')
    if ask_choice('Direct HOME will actuate the upper body. Proceed? (y/N): ', ('y', 'n', '')) != 'y':
        return
    init_channel_factory_guard(config)
    controller = None
    planners = None
    try:
        controller = DemoController(
            URDF_MAGPIE_PATH, URDF_MAGPIE_SPHERE_PATH, SRDF_MAGPIE_SPHERE_PATH,
            moving_arm=None, init=False, handless=False, visualize=False, config=config,
        )
        planners = {True: controller.planner}
        select_planner(controller, planners, False)
        print('Startup HOME planning: absolute grasp-frame z floor disabled')
        maybe_start_controller_logging(controller)
        while True:
            try:
                startup_home(controller)
                break
            except Exception as error:
                hold(controller)
                print(f'Startup HOME stopped: {type(error).__name__}: {error}')
                if ask_choice('Retry startup HOME? (y/N): ', ('y', 'n', '')) != 'y':
                    return
        if ask_choice('Standing and lower-body policy verified? (y/N): ', ('y', 'n', '')) != 'y':
            return
        while True:
            print(f'Available named configs: {list(DEMO_CONFIGS.keys())}')
            name = input(
                'Enter named config (or press Enter for frame task, q to quit): '
            ).strip().lower()
            if name == 'q':
                break
            if name and name not in DEMO_CONFIGS:
                print(f'Unknown named configuration: {name}')
                continue
            side, pose = (None, None)
            if not name:
                side = ask_choice('Enter moving arm (left/right): ', FRAMES)
                pose = ask_pose()
            goal_description = (
                f'named configuration: {name}' if name else f'frame task: {FRAMES[side]}'
            )
            planned = ask_choice('Plan this goto? (y/N): ', ('y', 'n', '')) == 'y'
            balanced = ask_choice('Enable counter-balance? (y/N): ', ('y', 'n', '')) == 'y'
            try:
                if name in OVERHANG_CONFIGS:
                    print('Offline overhang joint target: inspect the path; '
                          'HOME-to-first and counter reaction are not hardware-qualified')
                target, task = prepare_goal(controller, name or None, side, pose, balanced)
                if planned:
                    print(f'Planning to {goal_description}')
                    select_planner(controller, planners, ask_z_floor('this goal'))
                    started = time.monotonic()
                    path = plan_goal(controller, target, task, ik_timeout, ik_alpha)
                    planning_time = time.monotonic() - started
                    planned_steps = len(path)
                    path, _ = check_path(controller, path)
                    length = np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))
                    print(f'Planning time: {planning_time:.4f} s')
                    print(f'Path shape: {path.shape} (retimed from {planned_steps} waypoints)')
                    print(f'Joint path length: {length:.3f} rad, '
                          f'nominal execution: {(len(path) - 1) * controller.dt:.2f} s')
                    if ask_choice('Execute plan? (y/N): ', ('y', 'n', '')) != 'y':
                        continue
                    execute_path(controller, path, balanced)
                    settled = wait_for_target(controller, path[-1], balanced, frame_task=task)
                else:
                    print(f'Going to {goal_description}')
                    settled = live_goto(controller, target, balanced, frame_task=task)
                if balanced:
                    recover_counter_arm(controller)
                    print(f'Counter-balance: {controller.diagnostics()["status"]}')
                print('Target reached' if settled else 'Goal command complete; verify final pose')
                if input('Do you want to send another goal? (y/n): ').strip().lower() != 'y':
                    break
            except (Exception, KeyboardInterrupt) as error:
                try:
                    hold(controller)
                except Exception as hold_error:
                    print(f'Could not update hold command: {hold_error}')
                if planned and isinstance(error, KeyboardInterrupt):
                    # discard an interrupted request so its response cannot serve the next goal
                    try:
                        controller.planner.shutdown()
                    except Exception as planner_error:
                        print(f'Could not stop interrupted planner: {planner_error}')
                print(f'Goal refused or stopped: {type(error).__name__}: {error}')
                print('Returning to goal selection; no automatic retry or fallback motion.')
    except (KeyboardInterrupt, EOFError):
        print('Interrupted; holding upper-body command')
    finally:
        if controller is not None:
            try:
                hold(controller)
            finally:
                try:
                    if planners is not None:
                        for planner in planners.values():
                            if planner is not planners[True]:
                                planner.shutdown()
                finally:
                    if planners is not None:
                        controller.planner = planners[True]
                    controller.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Frame Controller Counter-Balance Goto')
    parser.add_argument(
        '--config', default='real_benchmark_upper.yaml',
        help='YAML file name under config/',
    )
    parser.add_argument('--ik-timeout', type=float, default=1.0)
    parser.add_argument('--ik-alpha', type=float, default=0.1)
    args = parser.parse_args()
    main(args.config, args.ik_timeout, args.ik_alpha)
