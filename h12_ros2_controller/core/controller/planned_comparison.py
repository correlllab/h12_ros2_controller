import time

import numpy as np

from h12_ros2_controller.utility.named_config import NAMED_CONFIGS
from h12_ros2_controller.core.controller.planned_reference import PlanReference
from h12_ros2_controller.core.controller.counter_frame_controller import CounterFrameController
from h12_ros2_controller.core.controller.upper_controller import PathExecutionCancelled
from h12_ros2_controller.core.controller.counter_balance.counter_goto_controller import (
    fresh_state, hold,
)


class PlannedComparison:
    '''Run one immutable single-arm reference with the free-arm overlay on/off'''

    def __init__(self, controller, preparation='prep'):
        if not isinstance(controller, CounterFrameController):
            raise ValueError('Planned comparison requires a CounterFrameController')
        if preparation not in NAMED_CONFIGS:
            raise ValueError(f'Unknown preparation configuration: {preparation}')
        self.controller = controller
        self.preparation = preparation

    def _condition(self):
        '''Seed IK caches from the current command without replacing measured history'''
        controller = self.controller
        controller._prior_ik = None
        controller.update_ik_solver()
        ik = controller.ik_solver
        # acceleration and collision caches belong to this move's control seed
        ik.rebuild_reduced()
        controller.update_ik_solver()
        ik.posture_task.set_target_from_configuration(ik.configuration_reduced)
        ik.config_task.set_target(ik.q_reduced)

    def run(self, frame_name, target_matrix, order, relative=False, slow_mode=False,
            timeout=300.0, cancel_check=None, feedback=None, event=None):
        controller = self.controller
        order = tuple(order)
        target = np.asarray(target_matrix, dtype=float).copy()
        summary = {'success': False, 'cancelled': False, 'trials': [],
                   'hash': None, 'reference': None, 'reason': ''}
        started = time.monotonic()
        deadline = started + float(timeout)
        current = None
        reference = None
        preparation = self.preparation
        preparation_target = NAMED_CONFIGS[preparation].copy()
        hold_steps = max(1, int(np.ceil(controller.timeout / controller.dt)))
        saved = {name: getattr(controller, name, None) for name in (
            'plain_goto', 'balanced_live', 'planned_balance', 'speed_scale',
            '_comparison_mode', '_comparison_playback', '_comparison_pending',
            '_comparison_audit', '_comparison_check', '_path_step_event',
            '_comparison_written', '_comparison_velocity', '_planned_previous',
            '_comparison_max_deviation',
        )}

        def check():
            if cancel_check is not None and cancel_check():
                raise PathExecutionCancelled('Comparison canceled; holding last command')
            if time.monotonic() >= deadline:
                raise TimeoutError('Comparison overall wall budget exceeded')
            controller.check_state_fresh()
            fresh_state(controller)

        def emit(record):
            if current is not None and record['type'] == 'command':
                status = record['counter_status']
                if status is not None:
                    counts = current['counter_status_counts']
                    counts[status] = counts.get(status, 0) + 1
                current['nominal_retry_used'] |= record['nominal_retry_used']
            if event is not None:
                event({**({'index': current['index'], 'condition': current['condition']}
                         if current is not None else {}),
                       'hash': summary['hash'], 'elapsed': time.monotonic() - started,
                       **record})

        def phase(name):
            check()
            emit({'type': 'phase', 'phase': name})
            if feedback is not None:
                feedback(name, len(summary['trials']) / len(order))

        def paced_step(function):
            check()
            tick = time.monotonic()
            result = function()
            elapsed = time.monotonic() - tick
            emit({'type': 'step', 'phase': 'hold', 'work_elapsed': elapsed,
                  'overrun': max(0.0, elapsed - controller.dt)})
            time.sleep(max(0.0, controller.dt - elapsed))
            return result

        def settle(preparation=False):
            controller.init_steady_state()
            # both conditions have the same fixed endpoint-hold tick count
            converged = False
            for _ in range(hold_steps):
                paced_step(controller.steady_state_step)
                if controller.steady_state_limited_joints:
                    raise ValueError('Comparison hold blocked at a joint position limit')
                if preparation:
                    joint_error = float(np.max(np.abs(
                        controller.robot_model.state_reduced['q'] -
                        preparation_target
                    )))
                    converged = joint_error < controller.config['controller']['threshold_joint']
                    if converged:
                        return
            if preparation and not converged:
                raise TimeoutError('Common preparation did not settle')

        try:
            if not order or any(mode not in ('frame', 'counter') for mode in order):
                raise ValueError('Comparison order must contain frame/counter conditions')
            if (not np.isfinite(timeout) or timeout <= 0.0 or
                    target.shape != (4, 4) or not np.isfinite(target).all()):
                raise ValueError('Comparison requires a finite target and positive wall budget')
            moving_arm = controller._arm_for_frame(frame_name)
            controller.speed_scale = 0.25 if slow_mode else 1.0
            for index, mode in enumerate(order):
                current = {'index': index, 'condition': mode, 'success': False,
                           'reason': '', 'hash': summary['hash'],
                           'linear_error': None, 'angular_error': None,
                           'moving_max_deviation': [0.0, 0.0, 0.0],
                           'config_digest': None, 'planned_path_checksum': summary['hash'],
                           'counter_status_counts': {}, 'nominal_retry_used': False}
                trial_started = time.monotonic()
                controller._comparison_mode = None
                controller._comparison_playback = False
                controller._comparison_pending = None
                controller._comparison_audit = None
                controller._comparison_max_deviation = np.zeros(3)
                controller._path_step_event = None
                phase('preparation')
                if reference is not None:
                    reference.check(controller, reference.path)
                controller.prepare_named_goal()
                # reset before plain preparation, never at a main-path waypoint
                if controller.moving_arm != moving_arm:
                    controller._set_arm_ownership(moving_arm)
                controller._tau_bias[:] = 0.0
                self._condition()
                prep = controller.plan_to_configuration(preparation_target)
                check()
                controller.execute_path(prep, cancel_check=lambda: (check() or False))
                controller.set_reduced_configuration_target(preparation_target)
                settle(preparation=True)
                check()
                controller.clear_frame_tasks()
                if reference is None and relative:
                    measured = controller.get_frame_transformation(frame_name, fresh_state(controller))
                    target[:3, 3] += measured[:3, 3]
                    target[:3, :3] = target[:3, :3] @ measured[:3, :3]
                controller.add_frame_task(f'{frame_name}_task', frame_name, target)
                self._condition()
                prepared_bias = controller._tau_bias.copy()
                if reference is None:
                    phase('planning')
                    path = controller.plan_to_ik_target()
                    check()
                    if not np.all(path[:, controller.counter_local] ==
                                  path[0, controller.counter_local]):
                        raise ValueError('Nominal plan moves the free arm')
                    reference = PlanReference.capture(controller, path)
                    summary['hash'] = reference.checksum
                    summary['reference'] = {
                        'created_at': reference.created_at, 'dt': reference.dt,
                        'config_digest': reference.config_digest,
                        'planned_path_checksum': reference.checksum,
                        'joint_order': [str(name) for name in
                                        controller.robot_model.model_body_reduced.names[1:]],
                        'target': target.tolist(), 'waypoints': len(reference.path),
                        'hold_steps': hold_steps,
                        'hold_duration': hold_steps * reference.dt,
                    }
                    emit({'type': 'reference', **summary['reference']})
                # planning may change IK configuration, prior, I bias and solver caches
                controller._tau_bias[:] = prepared_bias
                self._condition()
                controller.capture_reference()
                current['hash'] = reference.checksum
                current['config_digest'] = reference.config_digest
                current['planned_path_checksum'] = reference.checksum
                controller._comparison_mode = mode
                controller._comparison_check = check
                controller._comparison_max_deviation = np.zeros(3)
                controller._comparison_audit = emit
                controller._path_step_event = emit
                phase('revalidation')
                validation = controller.revalidate_plan(reference)
                check()
                phase('playback')
                controller.execute_path(reference.path, validation=validation,
                                        cancel_check=lambda: (check() or False))
                phase('hold')
                settle()
                check()
                error = np.asarray(controller.get_frame_task_error(f'{frame_name}_task'))
                if not np.isfinite(error).all():
                    raise ValueError('Nonfinite comparison endpoint task error')
                linear, angular = float(np.linalg.norm(error[:3])), float(np.linalg.norm(error[3:]))
                if not np.isfinite([linear, angular]).all():
                    raise ValueError('Nonfinite comparison endpoint error norm')
                cfg = controller.config.get('controller', {})
                current.update({
                    'linear_error': linear, 'angular_error': angular,
                    'moving_max_deviation': controller._comparison_max_deviation.tolist(),
                    'elapsed': time.monotonic() - trial_started,
                    'config_digest': reference.config_digest,
                    'planned_path_checksum': reference.checksum,
                    'validated_at': validation.validated_at,
                })
                if (linear >= cfg.get('threshold_linear', 1e-3) or
                        angular >= cfg.get('threshold_angular', 1e-2)):
                    raise RuntimeError('Comparison endpoint task did not converge')
                current['success'] = True
                summary['trials'].append(current)
                emit({'type': 'trial', **current})
                current = None
            summary['success'] = True
            if feedback is not None:
                feedback('complete', 1.0)
        except Exception as error:
            summary['cancelled'] = isinstance(error, PathExecutionCancelled)
            summary['reason'] = str(error)
            if current is not None:
                current.update({'reason': str(error), 'elapsed': time.monotonic() - trial_started,
                                'moving_max_deviation': controller._comparison_max_deviation.tolist()})
                summary['trials'].append(current)
                emit({'type': 'trial', **current})
        finally:
            # no return motion, publisher release or lower-body restart on any outcome
            try:
                hold(controller)
            finally:
                for name, value in saved.items():
                    setattr(controller, name, value)
        summary['elapsed'] = time.monotonic() - started
        return summary
