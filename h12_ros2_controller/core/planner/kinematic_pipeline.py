'''Offline IK and reduced joint-space planning without DDS or motor commands'''

import time
from dataclasses import dataclass, field, replace

import numpy as np
import pinocchio as pin

from h12_ros2_controller.core.ik_solver import IKSolver
from h12_ros2_controller.core.planner.reduced_joint_planner import (
    PlannerConfig, ReducedJointPlanner, active_grasp_frames, moving_grasp_frames,
)
from h12_ros2_controller.core.planner.path_quality import candidate_path_score
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.utility.controller_config import (
    _resolve_config_path, load_controller_config,
)
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS, ENABLED_JOINTS
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS
from h12_ros2_controller.utility.path_definition import resolve_model_paths


@dataclass
class KinematicPlan:
    plan: object
    ik: object = None
    elapsed_time: float = 0.0
    selected_candidate: object = None
    phase_timings: dict = field(default_factory=dict)
    candidate_attempts: list = field(default_factory=list)
    fallback_reason: str = ''


def frame_target_snapshot(frame_tasks):
    '''Copy frame targets for the same seed-reuse policy in every adapter'''
    return tuple(sorted((
        (task.frame, np.asarray(task.transform_target_to_world.translation).copy(),
         np.asarray(task.transform_target_to_world.rotation).copy())
        for task in frame_tasks.values()
    ), key=lambda item: item[0]))


def frame_targets_close(targets, previous, config):
    '''Compare target changes using Cartesian distance and SO(3) rotation'''
    return len(targets) == len(previous) and all(
        name == old_name and
        np.linalg.norm(position - old_position) <= config.ik_reuse_translation and
        np.linalg.norm(pin.log3(old_rotation.T @ rotation)) <= config.ik_reuse_rotation
        for (name, position, rotation), (old_name, old_position, old_rotation)
        in zip(targets, previous)
    )


def plan_ik_candidates(ik_result, plan_goal, reduced_mask, max_candidates=None,
                       planning_budget=None, attempts=None, config=None,
                       deadline=None):
    '''Prefer short direct paths, then bounded local OMPL candidates

    plan_goal accepts goal, direct_only, and timeout keyword arguments, plus
    an absolute deadline when this route is bounded.
    Returns (PlanResult or None, selected IK attempt or None).
    '''
    config = config if config is not None else PlannerConfig()
    if max_candidates is None:
        max_candidates = config.max_ik_candidates
    if planning_budget is None:
        planning_budget = config.candidate_planning_budget
    if max_candidates < 1:
        raise ValueError('max_candidates must be positive')
    if not np.isfinite(planning_budget) or planning_budget <= 0:
        raise ValueError('planning_budget must be finite and positive')
    if deadline is not None and not np.isfinite(deadline):
        raise ValueError('deadline must be finite')
    bounded = deadline is not None
    candidates = ik_result.candidates[:max_candidates]
    phase_deadline = time.perf_counter() + planning_budget
    deadline = min(phase_deadline, deadline) if bounded else phase_deadline
    direct_plans = []
    failures = []
    last_plan = None

    def record(candidate, phase, plan, started):
        if attempts is not None:
            attempts.append({
                'seed': candidate.seed_name,
                'phase': phase,
                'success': bool(plan.success),
                'reason': plan.reason,
                'wall_time': time.perf_counter() - started,
                'planner_time': plan.planning_time,
                'joint_path_ratio': plan.metadata.get('joint_path_ratio'),
                'joint_path_length': plan.metadata.get('joint_path_length'),
                'stage_timings': plan.metadata.get('stage_timings', {}).copy(),
            })

    def score(plan, candidate):
        length = plan.metadata.get('joint_path_length')
        if length is None:
            length = float(np.sum(np.linalg.norm(np.diff(plan.path, axis=0), axis=1)))
        return candidate_path_score(
            {**plan.metadata, 'joint_path_length': length}, candidate.score,
            config,
        )

    for candidate in candidates:
        remaining = deadline - time.perf_counter()
        if remaining <= 0:
            failures.append('planning budget exhausted before direct probe')
            break
        goal = candidate.q[reduced_mask]
        attempt_start = time.perf_counter()
        if bounded:
            remaining = deadline - time.perf_counter()
            if remaining <= 0:
                break
        try:
            plan = plan_goal(
                goal, direct_only=True, timeout=remaining if bounded else None,
                **({'deadline': deadline} if bounded else {}),
            )
        except TimeoutError:
            if not bounded or not direct_plans:
                raise
            break
        record(candidate, 'direct', plan, attempt_start)
        last_plan = plan
        if plan.success:
            direct_plans.append((score(plan, candidate), plan, candidate))
        else:
            failures.append(f'{candidate.seed_name}: {plan.reason}')
    if direct_plans:
        _, plan, candidate = min(direct_plans, key=lambda item: item[0])
        return plan, candidate

    best = None
    for index, candidate in enumerate(candidates):
        remaining = deadline - time.perf_counter()
        if remaining <= 0 or (not bounded and remaining < 0.1):
            break
        goal = candidate.q[reduced_mask]
        timeout = min(config.candidate_ompl_timeout,
                      remaining / (len(candidates) - index))
        attempt_start = time.perf_counter()
        try:
            plan = plan_goal(
                goal, direct_only=False, timeout=timeout,
                **({'deadline': deadline} if bounded else {}),
            )
        except TimeoutError:
            if not bounded or best is None:
                raise
            break
        record(candidate, 'ompl', plan, attempt_start)
        last_plan = plan
        if not plan.success:
            failures.append(f'{candidate.seed_name}: {plan.reason}')
            continue
        quality = score(plan, candidate)
        if best is None or quality < best[0]:
            best = (quality, plan, candidate)
        remaining_candidates = candidates[index + 1:]
        if remaining_candidates and best is not None:
            # endpoint travel and IK cost lower-bound each remaining path score
            lower_bound = min(
                np.linalg.norm(other.q[reduced_mask] - plan.path[0]) +
                config.candidate_ik_weight * (other.score or 0.0)
                for other in remaining_candidates
            )
            if best[0] <= lower_bound:
                break
        elif not remaining_candidates:
            break
    if best is not None:
        return best[1], best[2]
    if last_plan is not None:
        if deadline - time.perf_counter() < 0.1:
            failures.append('planning budget exhausted')
        last_plan.reason = 'No acceptable local path: ' + '; '.join(failures)
    return last_plan, None


def compose_bimanual_seed(ik_solver, start_q, active_mask, plan_goal, config,
                          alpha, linear_threshold, angular_threshold,
                          deadline=None):
    '''Find path-feasible per-arm endpoints without changing final frame tasks'''
    if deadline is not None and not np.isfinite(deadline):
        raise ValueError('deadline must be finite')
    original_tasks = ik_solver.frame_tasks
    grouped = {'left': {}, 'right': {}}
    for name, task in original_tasks.items():
        side = task.frame.split('_', 1)[0]
        if side not in grouped:
            return None, f'unsupported bimanual frame: {task.frame}'
        grouped[side][name] = task
    if not all(grouped.values()):
        return None, 'frame tasks do not involve both arms'

    robot = ik_solver.robot_model
    reduced_mask = robot.reduced_mask
    combined = np.asarray(start_q, dtype=float).copy()
    bounded = deadline is not None
    phase_deadline = time.perf_counter() + config.bimanual_fallback_budget
    deadline = min(phase_deadline, deadline) if bounded else phase_deadline
    try:
        for index, side in enumerate(('left', 'right')):
            arm_mask = np.zeros(robot.model_body_reduced.nq, dtype=bool)
            arm_mask[index * 7:index * 7 + 7] = active_mask[index * 7:index * 7 + 7]
            if not np.any(arm_mask):
                return None, f'{side} arm has no active joints'
            ik_solver.frame_tasks = grouped[side]
            remaining = deadline - time.perf_counter()
            if remaining <= (0 if bounded else 0.2):
                return None, 'bimanual fallback budget exhausted before arm IK'
            arm_ik_budget = min(0.8, remaining / (3 - index))
            arm_ik = ik_solver.solve_ik_reduced(
                alpha=alpha, timeout=min(config.ik_seed_timeout, arm_ik_budget),
                total_timeout=arm_ik_budget, max_seeds=config.ik_max_seeds,
                linear_threshold=linear_threshold,
                angular_threshold=angular_threshold,
                start_q=start_q, active_mask=arm_mask,
            )
            if not arm_ik.success:
                return None, f'{side} arm IK failed'
            remaining = deadline - time.perf_counter()
            if remaining <= (0 if bounded else 0.2):
                return None, 'bimanual fallback budget exhausted before arm planning'
            arm_plan, chosen = plan_ik_candidates(
                arm_ik,
                lambda goal, direct_only, timeout, **limits: plan_goal(
                    goal, arm_mask, (f'{side}_grasp_frame',), direct_only,
                    timeout, **limits,
                ),
                reduced_mask, config=config,
                planning_budget=min(
                    config.candidate_planning_budget, remaining / (3 - index),
                ),
                deadline=deadline if bounded else None,
            )
            if chosen is None:
                return None, f'{side} arm path failed: {arm_plan.reason if arm_plan else "no path"}'
            # combine disjoint arm joints, then check whole-body limits and collision
            q = combined[reduced_mask].copy()
            q[arm_mask] = chosen.q[reduced_mask][arm_mask]
            combined[reduced_mask] = q
        q = combined[reduced_mask]
        model = robot.model_body_reduced
        if (np.any(q < model.lowerPositionLimit) or
                np.any(q > model.upperPositionLimit) or
                not robot.check_collision_free_reduced(
                    q, min_distance=config.self_collision_clearance,
                )):
            return None, 'composed arm endpoints are infeasible together'
        return combined, ''
    finally:
        ik_solver.frame_tasks = original_tasks
        ik_solver.update_configurations(start_q)


def plan_frame_tasks(ik_solver, start_q, active_mask, reduced_mask, local_frames,
                     plan_goal, config, alpha=None, ik_timeout=None,
                     linear_threshold=1e-3, angular_threshold=1e-2,
                     preferred_positions=None, deadline=None):
    '''Use one shared candidate and bimanual fallback route for frame tasks'''
    alpha = config.ik_alpha if alpha is None else alpha
    ik_timeout = config.ik_total_timeout if ik_timeout is None else ik_timeout
    started = time.perf_counter()
    if deadline is not None and not np.isfinite(deadline):
        raise ValueError('deadline must be finite')
    remaining = deadline - started if deadline is not None else ik_timeout
    ik = None
    if deadline is None or remaining > 0:
        budget = min(ik_timeout, remaining)
        ik = ik_solver.solve_ik_reduced(
            alpha=alpha, timeout=min(config.ik_seed_timeout, budget),
            total_timeout=budget, max_seeds=config.ik_max_seeds,
            linear_threshold=linear_threshold,
            angular_threshold=angular_threshold,
            start_q=start_q, active_mask=active_mask,
            preferred_positions=preferred_positions,
        )
    ik_elapsed = time.perf_counter() - started
    attempts = []
    plan = selected = None
    if ik is not None and ik.success and (deadline is None or time.perf_counter() < deadline):
        plan, selected = plan_ik_candidates(
            ik,
            lambda goal, direct_only, timeout, **limits: plan_goal(
                goal, active_mask, local_frames, direct_only, timeout,
                **limits,
            ),
            reduced_mask, config=config, attempts=attempts, deadline=deadline,
        )
    fallback_time = 0.0
    fallback_reason = 'planning deadline exceeded before IK' if ik is None else ''
    if (selected is None and len(local_frames) == 2 and
            (deadline is None or time.perf_counter() < deadline)):
        fallback_started = time.perf_counter()
        fallback_deadline = fallback_started + config.bimanual_fallback_budget
        if deadline is not None:
            fallback_deadline = min(fallback_deadline, deadline)
        composed, fallback_reason = compose_bimanual_seed(
            ik_solver, start_q, active_mask, plan_goal, config,
            alpha, linear_threshold, angular_threshold,
            deadline=fallback_deadline if deadline is not None else None,
        )
        if composed is not None:
            remaining = fallback_deadline - time.perf_counter()
            if remaining > (0 if deadline is not None else 0.2):
                ik = ik_solver.solve_ik_reduced(
                    alpha=alpha, timeout=min(config.ik_seed_timeout, remaining / 2),
                    total_timeout=min(ik_timeout, remaining / 2),
                    max_seeds=config.ik_max_seeds,
                    linear_threshold=linear_threshold,
                    angular_threshold=angular_threshold,
                    start_q=start_q, active_mask=active_mask,
                    preferred_positions=composed,
                )
                if ik.success:
                    remaining = fallback_deadline - time.perf_counter()
                    if remaining > (0 if deadline is not None else 0.1):
                        plan, selected = plan_ik_candidates(
                            ik,
                            lambda goal, direct_only, timeout, **limits: plan_goal(
                                goal, active_mask, local_frames, direct_only,
                                timeout, **limits,
                            ),
                            reduced_mask, config=config, attempts=attempts,
                            planning_budget=min(config.candidate_planning_budget,
                                                remaining),
                            deadline=fallback_deadline if deadline is not None else None,
                        )
            if selected is None:
                fallback_reason = 'composed bimanual seed has no valid joint path'
        fallback_time = time.perf_counter() - fallback_started
    if selected is None and plan is not None and fallback_reason:
        plan.reason += f'; bimanual fallback: {fallback_reason}'
    elapsed = time.perf_counter() - started
    timings = {
        'ik': ik_elapsed, 'candidate_search': elapsed - ik_elapsed,
        'bimanual_fallback': fallback_time, 'total': elapsed,
    }
    return ik, plan, selected, attempts, timings, fallback_reason


def planner_config_from_controller(config, dt=None):
    '''Resolve the same planner options for hardware and offline planning'''
    options = dict(config.get('planner', {}))
    options['motor_position_limits'] = configured_position_limits(
        config, options.get('motor_position_limits'),
    )
    clearance = float(config.get('controller', {}).get('d_min', 0.02))
    if ('self_collision_clearance' in options and
            not np.isclose(options['self_collision_clearance'], clearance,
                           rtol=0.0, atol=1e-9)):
        raise ValueError('Planner self_collision_clearance must match controller d_min')
    options['self_collision_clearance'] = clearance
    limit = float(config.get('controller', {}).get('dq_lim', 1.0))
    motor_limits = config.get('limits', {}).get('dq_clip_limits')
    if motor_limits is not None:
        enabled_ids = [BODY_JOINTS.index(name) for name in ENABLED_JOINTS]
        limit = np.minimum(limit, np.asarray(motor_limits)[enabled_ids])
    # an explicit planner override cannot exceed the publisher's motor caps
    requested = np.asarray(options.get('joint_velocity_limit', limit), dtype=float)
    options['joint_velocity_limit'] = tuple(
        np.minimum(np.broadcast_to(requested, (len(ENABLED_JOINTS),)), limit)
    )
    controller_dt = 1.0 / float(config.get('frequency', {}).get('ctrl_hz', 50.0))
    options.setdefault(
        'dt', dt if dt is not None else controller_dt,
    )
    if not np.isclose(options['dt'], controller_dt, rtol=0.0, atol=1e-9):
        raise ValueError('Planner dt must match controller execution period')
    if 'frame_names' in options:
        options['frame_names'] = tuple(options['frame_names'])
    return PlannerConfig(**options)


def configured_position_limits(config, requested=None):
    '''Keep injected planning bounds inside the command publisher's bounds'''
    command = config.get('limits', {}).get('q_clip_limits')
    if command is None:
        return requested
    limits = np.asarray(command, dtype=float).copy()
    if limits.shape != (len(BODY_JOINTS), 2) or not np.all(np.isfinite(limits)):
        raise ValueError('Motor position limits must contain 27 finite lower/upper pairs')
    if requested is not None:
        proposed = np.asarray(requested, dtype=float)
        if proposed.shape != limits.shape or not np.all(np.isfinite(proposed)):
            raise ValueError('Motor position limits must contain 27 finite lower/upper pairs')
        limits[:, 0] = np.maximum(limits[:, 0], proposed[:, 0])
        limits[:, 1] = np.minimum(limits[:, 1], proposed[:, 1])
    if np.any(limits[:, 0] >= limits[:, 1]):
        raise ValueError('Motor position-limit intersection is empty')
    return tuple(tuple(float(value) for value in pair) for pair in limits)


def candidate_metrics(candidate, start, model, reduced_mask):
    '''Summarize wrist use and remaining joint range for a candidate'''
    q = candidate.q[reduced_mask]
    start = np.asarray(start, dtype=float)[reduced_mask]
    wrist_ids = [joint.idx_q for joint in model.joints[1:]
                 if 'wrist' in model.names[joint.id].lower()]
    lower = model.lowerPositionLimit
    upper = model.upperPositionLimit
    margin = np.minimum(q - lower, upper - q)
    return {
        'score': candidate.score,
        'wrist_travel': float(np.linalg.norm(q[wrist_ids] - start[wrist_ids])),
        'wrist_margin': float(np.min(margin[wrist_ids])) if wrist_ids else float('nan'),
        'joint_margin': float(np.min(margin)),
    }


class KinematicPipeline:
    '''Keep simulated joint state, IK, planner, and optional Meshcat together'''

    def __init__(self, visualize=False, planner_config=None,
                 config_name='sim_safety_split.yaml'):
        config = load_controller_config(config_name)
        if not config.get('planner'):
            raise ValueError('Kinematic planning requires an explicit planner YAML block')
        self.config_name = config_name
        controller_cfg = config.get('controller', {})
        self.ik_linear_threshold = float(controller_cfg.get('threshold_linear', 1e-3))
        self.ik_angular_threshold = float(controller_cfg.get('threshold_angular', 1e-2))
        if (not np.isfinite(self.ik_linear_threshold) or
                self.ik_linear_threshold <= 0.0 or
                not np.isfinite(self.ik_angular_threshold) or
                self.ik_angular_threshold <= 0.0):
            raise ValueError('IK frame thresholds must be finite and positive')
        if planner_config is None:
            planner_config = planner_config_from_controller(config)
        elif not np.isclose(
            planner_config.dt,
            1.0 / float(config.get('frequency', {}).get('ctrl_hz', 50.0)),
            rtol=0.0, atol=1e-9,
        ):
            raise ValueError('Planner dt must match controller execution period')
        if not np.isclose(
            planner_config.self_collision_clearance,
            float(controller_cfg.get('d_min', 0.02)),
            rtol=0.0, atol=1e-9,
        ):
            raise ValueError('Planner self_collision_clearance must match controller d_min')
        self.model_paths = resolve_model_paths()
        urdf, collision_urdf, collision_srdf = self.model_paths
        self.robot_model = RobotModel(urdf, handless=False)
        planner_config = replace(
            planner_config,
            motor_position_limits=configured_position_limits(
                config, planner_config.motor_position_limits,
            ),
        )
        if planner_config.motor_position_limits is not None:
            self.robot_model.apply_position_limits(planner_config.motor_position_limits)
            planner_config = replace(
                planner_config,
                motor_position_limits=tuple(zip(
                    self.robot_model.model_body.lowerPositionLimit.tolist(),
                    self.robot_model.model_body.upperPositionLimit.tolist(),
                )),
            )
        self.robot_model.init_reduced_model(ENABLED_JOINTS)
        self.robot_model.init_collision_model(collision_urdf, collision_srdf)
        if visualize:
            self.robot_model.init_visualizer()
            self.robot_model.update_visualizer()
        self.planner_config = planner_config
        self.ik_solver = IKSolver(
            self.robot_model, dt=planner_config.dt,
            d_min=config['controller']['d_min'],
            wrist_movement_weight=planner_config.ik_wrist_motion_weight,
            wrist_posture_cost=planner_config.ik_wrist_posture_cost,
            shoulder_seed_offset=planner_config.ik_shoulder_seed_offset,
            elbow_seed_offset=planner_config.ik_elbow_seed_offset,
            batch_frame_lm_damping=planner_config.ik_frame_lm_damping,
            batch_stagnation_iterations=planner_config.ik_stagnation_iterations,
            batch_stagnation_relative_improvement=(
                planner_config.ik_stagnation_relative_improvement
            ),
        )
        self.planner = ReducedJointPlanner(self.robot_model, planner_config)
        self._prior_ik = None

    def plan_named(self, name):
        '''Plan from the simulated pose to a named reduced configuration'''
        if name not in NAMED_CONFIGS:
            raise ValueError(f'Unknown named configuration: {name}')
        self._prior_ik = None
        start_time = time.perf_counter()
        deadline = start_time + self.planner_config.named_planning_budget
        start = self.robot_model.state_reduced['q']
        goal = np.asarray(NAMED_CONFIGS[name], dtype=float)
        local_frames = moving_grasp_frames(start, goal, self.planner.frame_names)
        plan = self.planner.plan(
            start, goal, local_frames=local_frames, local_joint_padding=0.0,
            deadline=deadline,
        )
        elapsed = time.perf_counter() - start_time
        return KinematicPlan(
            plan=plan, elapsed_time=elapsed,
            phase_timings={'planning': elapsed, 'total': elapsed},
        )

    def set_start(self, q):
        '''Set a simulated motor pose without sending a command'''
        q = np.asarray(q, dtype=float)
        model = self.robot_model
        if q.shape != model._q.shape or not np.all(np.isfinite(q)):
            raise ValueError(f'start_q must contain {model._q.size} finite values')
        self._prior_ik = None
        frozen = ~model.reduced_mask
        model._q = q.copy()
        model._dq = np.zeros_like(q)
        if not np.array_equal(q[frozen], model.reduced_reference_q[frozen]):
            model.rebuild_reduced_model(reference_q=q)
            self.ik_solver.rebuild_reduced()
            self.planner.rebind_reduced_model()
        self.ik_solver.update_configurations(q)
        model.update_kinematics()
        model.update_visualizer()

    def plan_case(self, case):
        '''Replay a saved start and frame goal through the normal plan route'''
        if _resolve_config_path(case.get('config', self.config_name)) != _resolve_config_path(
            self.config_name
        ):
            raise ValueError('Saved case config differs from the selected YAML profile')
        self.set_start(case['start_q'])
        if 'tasks' in case:
            tasks = [(task['frame'], task['pose']) for task in case['tasks']]
            return self.plan_frames(
                tasks, ik_timeout=case.get('ik_timeout'),
                ik_alpha=case.get('ik_alpha'),
            )
        return self.plan_frame(
            case['frame'], case['pose'],
            ik_timeout=case.get('ik_timeout'),
            ik_alpha=case.get('ik_alpha'),
        )

    def plan_frame(self, frame_name, pose, ik_timeout=None, ik_alpha=None):
        '''Plan one frame goal through the shared multi-frame path'''
        return self.plan_frames(
            ((frame_name, pose),), ik_timeout=ik_timeout, ik_alpha=ik_alpha,
        )

    def plan_frames(self, tasks, ik_timeout=None, ik_alpha=None):
        '''Solve all pelvis-relative frame tasks in one IK and joint plan'''
        model = self.robot_model.model_body_reduced
        normalized = []
        for frame_name, value in tasks:
            pose = np.asarray(value, dtype=float)
            if pose.shape != (6,) or not np.all(np.isfinite(pose)):
                raise ValueError('pose must be six finite xyz/rpy values (radians)')
            frame_id = model.getFrameId(frame_name)
            if frame_id >= len(model.frames):
                raise ValueError(f'Unknown reduced-model frame: {frame_name}')
            normalized.append((frame_name, pose.copy(), frame_id))
        if not normalized or len({name for name, _, _ in normalized}) != len(normalized):
            raise ValueError('Frame goals must be nonempty with distinct frame names')
        start_time = time.perf_counter()
        deadline = start_time + self.planner_config.request_planning_budget
        self.ik_solver.clear_frame_tasks()
        for index, (frame_name, _, _) in enumerate(normalized):
            self.ik_solver.add_frame_task(f'offline_target_{index}', frame_name)
        self.ik_solver.set_from_configuration_reduced()
        active_mask = np.zeros(model.nq, dtype=bool)
        for index, (_, pose, frame_id) in enumerate(normalized):
            self.ik_solver.frame_tasks[f'offline_target_{index}'].transform_target_to_world = (
                pin.SE3(pin.rpy.rpyToMatrix(pose[3:]), pose[:3])
            )
            joint_id = model.frames[frame_id].parentJoint
            while joint_id > 0:
                joint = model.joints[joint_id]
                active_mask[joint.idx_q:joint.idx_q + joint.nq] = True
                joint_id = model.parents[joint_id]
        start_q = np.copy(self.robot_model.state['q'])
        preferred = None
        previous = self._prior_ik
        targets = frame_target_snapshot(self.ik_solver.frame_tasks)
        if previous is not None and frame_targets_close(
            targets, previous['targets'], self.planner_config,
        ):
            preferred = previous['q'][self.robot_model.reduced_mask].copy()
        start = self.robot_model.state_reduced['q']
        local_frames = active_grasp_frames(active_mask, self.planner.frame_names)
        ik, plan, selected, attempts, timings, fallback_reason = plan_frame_tasks(
            self.ik_solver, start_q, active_mask, self.robot_model.reduced_mask,
            local_frames,
            lambda goal, mask, frames, direct_only, timeout, deadline=deadline: self.planner.plan(
                start, goal, active_mask=mask, local_frames=frames,
                direct_only=direct_only, timeout=timeout,
                deadline=deadline,
            ),
            self.planner_config, alpha=ik_alpha, ik_timeout=ik_timeout,
            linear_threshold=self.ik_linear_threshold,
            angular_threshold=self.ik_angular_threshold,
            preferred_positions=preferred,
            deadline=deadline,
        )
        if selected is not None:
            self._prior_ik = {
                'targets': targets,
                'q': selected.q.copy(),
            }
        elapsed = time.perf_counter() - start_time
        timings['setup'] = elapsed - timings['total']
        timings['total'] = elapsed
        return KinematicPlan(
            plan=plan, ik=ik, selected_candidate=selected,
            elapsed_time=elapsed,
            phase_timings=timings,
            candidate_attempts=attempts,
            fallback_reason=fallback_reason,
        )

    def preview_candidate(self, candidate):
        '''Display an IK pose without advancing the simulated start state'''
        if self.robot_model.visualizer is None:
            raise RuntimeError('Start with --visualize to preview IK candidates')
        self.robot_model.visualizer.viz.display(self.robot_model.full_q(candidate.q))

    def replay(self, path, delay=0.0):
        '''Advance the simulated pose over a successful reduced path'''
        path = np.asarray(path, dtype=float)
        nq = self.robot_model.model_body_reduced.nq
        if path.ndim != 2 or path.shape[1] != nq or not len(path) or not np.all(np.isfinite(path)):
            raise ValueError(f'path must have shape (N, {nq}) with finite values')
        for q_reduced in path:
            self.robot_model._q[self.robot_model.reduced_mask] = q_reduced
            self.robot_model.update_kinematics()
            self.robot_model.update_visualizer()
            if delay > 0:
                time.sleep(delay)

    def show_path(self, path):
        '''Draw the path if Meshcat was requested'''
        if self.robot_model.visualizer is not None:
            self.robot_model.visualizer.visualize_reduced_path(path)
