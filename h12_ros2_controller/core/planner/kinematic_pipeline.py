'''Offline IK and reduced joint-space planning without DDS or motor commands'''

import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pinocchio as pin

from h12_ros2_controller.core.ik_solver import IKSolver
from h12_ros2_controller.core.planner.reduced_joint_planner import (
    PlannerConfig, ReducedJointPlanner, moving_grasp_frames,
)
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.utility.controller_config import load_controller_config
from h12_ros2_controller.utility.joint_definition import ENABLED_JOINTS
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS
from h12_ros2_controller.utility.path_definition import (
    SRDF_MAGPIE_SPHERE_PATH,
    URDF_MAGPIE_PATH,
    URDF_MAGPIE_SPHERE_PATH,
)


@dataclass
class KinematicPlan:
    plan: object
    ik: object = None
    elapsed_time: float = 0.0
    selected_candidate: object = None


def plan_ik_candidates(ik_result, plan_goal, reduced_mask, max_candidates=3,
                       planning_budget=2.0):
    '''Prefer short direct paths, then bounded local OMPL candidates

    plan_goal accepts goal, direct_only, and timeout keyword arguments.
    Returns (PlanResult or None, selected IK attempt or None).
    '''
    if max_candidates < 1:
        raise ValueError('max_candidates must be positive')
    if not np.isfinite(planning_budget) or planning_budget <= 0:
        raise ValueError('planning_budget must be finite and positive')
    candidates = ik_result.candidates[:max_candidates]
    deadline = time.perf_counter() + planning_budget
    direct_plans = []
    failures = []
    last_plan = None

    def score(plan, candidate):
        length = plan.metadata.get('joint_path_length')
        if length is None:
            length = float(np.sum(np.linalg.norm(np.diff(plan.path, axis=0), axis=1)))
        # retain IK's wrist and joint-margin preference when paths are similar
        return length + 0.1 * (candidate.score or 0.0)

    for candidate in candidates:
        goal = candidate.q[reduced_mask]
        plan = plan_goal(goal, direct_only=True, timeout=None)
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
        if remaining < 0.1:
            break
        goal = candidate.q[reduced_mask]
        timeout = min(0.75, remaining / (len(candidates) - index))
        plan = plan_goal(goal, direct_only=False, timeout=timeout)
        last_plan = plan
        if not plan.success:
            failures.append(f'{candidate.seed_name}: {plan.reason}')
            continue
        quality = score(plan, candidate)
        if best is None or quality < best[0]:
            best = (quality, plan, candidate)
        if plan.metadata.get('joint_path_ratio', float('inf')) <= 1.3:
            break
    if best is not None:
        return best[1], best[2]
    if last_plan is not None:
        if deadline - time.perf_counter() < 0.1:
            failures.append('planning budget exhausted')
        last_plan.reason = 'No acceptable local path: ' + '; '.join(failures)
    return last_plan, None


def planner_config_from_controller(config, dt=None):
    '''Resolve the same planner options for hardware and offline planning'''
    options = dict(config.get('planner', {}))
    options.setdefault(
        'dt', dt if dt is not None else
        1.0 / config.get('frequency', {}).get('ctrl_hz', 50.0),
    )
    if 'frame_names' in options:
        options['frame_names'] = tuple(options['frame_names'])
    return PlannerConfig(**options)


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

    def __init__(self, visualize=False, planner_config=None, config_name='debug.yaml'):
        config = load_controller_config(config_name)
        self.config_name = config_name
        if planner_config is None:
            planner_config = planner_config_from_controller(config)
        for asset in (URDF_MAGPIE_PATH, URDF_MAGPIE_SPHERE_PATH, SRDF_MAGPIE_SPHERE_PATH):
            if not Path(asset).is_file():
                raise FileNotFoundError(f'missing robot asset: {asset}')
        self.robot_model = RobotModel(URDF_MAGPIE_PATH, handless=False)
        self.robot_model.init_reduced_model(ENABLED_JOINTS)
        self.robot_model.init_collision_model(URDF_MAGPIE_SPHERE_PATH, SRDF_MAGPIE_SPHERE_PATH)
        if visualize:
            self.robot_model.init_visualizer()
            self.robot_model.update_visualizer()
        self.ik_solver = IKSolver(
            self.robot_model, dt=planner_config.dt,
            d_min=config['controller']['d_min'],
        )
        self.planner = ReducedJointPlanner(self.robot_model, planner_config)

    def plan_named(self, name):
        '''Plan from the simulated pose to a named reduced configuration'''
        if name not in NAMED_CONFIGS:
            raise ValueError(f'Unknown named configuration: {name}')
        start_time = time.perf_counter()
        start = self.robot_model.state_reduced['q']
        goal = np.asarray(NAMED_CONFIGS[name], dtype=float)
        local_frames = moving_grasp_frames(start, goal, self.planner.frame_names)
        plan = self.planner.plan(start, goal, local_frames=local_frames)
        return KinematicPlan(plan=plan, elapsed_time=time.perf_counter() - start_time)

    def set_start(self, q):
        '''Set a simulated motor pose without sending a command'''
        q = np.asarray(q, dtype=float)
        model = self.robot_model
        if q.shape != model._q.shape or not np.all(np.isfinite(q)):
            raise ValueError(f'start_q must contain {model._q.size} finite values')
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
        if case.get('config') != self.config_name:
            raise ValueError('Saved case config differs from the selected YAML profile')
        self.set_start(case['start_q'])
        return self.plan_frame(
            case['frame'], case['pose'],
            ik_timeout=case.get('ik_timeout', 1.0),
            ik_alpha=case.get('ik_alpha', 0.1),
        )

    def plan_frame(self, frame_name, pose, ik_timeout=1.0, ik_alpha=0.1):
        '''Solve a pelvis-relative xyz/rpy frame goal, then plan to its IK result'''
        pose = np.asarray(pose, dtype=float)
        if pose.shape != (6,) or not np.all(np.isfinite(pose)):
            raise ValueError('pose must be six finite xyz/rpy values (radians)')
        model = self.robot_model.model_body_reduced
        frame_id = model.getFrameId(frame_name)
        if frame_id >= len(model.frames):
            raise ValueError(f'Unknown reduced-model frame: {frame_name}')
        start_time = time.perf_counter()
        self.ik_solver.clear_frame_tasks()
        self.ik_solver.add_frame_task('offline_target', frame_name)
        self.ik_solver.set_from_configuration_reduced()
        self.ik_solver.frame_tasks['offline_target'].transform_target_to_world = pin.SE3(
            pin.rpy.rpyToMatrix(pose[3:]), pose[:3],
        )
        active_mask = np.zeros(model.nq, dtype=bool)
        joint_id = model.frames[frame_id].parentJoint
        while joint_id > 0:
            joint = model.joints[joint_id]
            active_mask[joint.idx_q:joint.idx_q + joint.nq] = True
            joint_id = model.parents[joint_id]
        ik = self.ik_solver.solve_ik_reduced(
            alpha=ik_alpha, timeout=min(0.3, ik_timeout),
            total_timeout=ik_timeout,
            start_q=self.robot_model.state['q'],
            active_mask=active_mask,
        )
        if not ik.success:
            return KinematicPlan(
                plan=None, ik=ik, elapsed_time=time.perf_counter() - start_time,
            )

        start = self.robot_model.state_reduced['q']
        plan, selected = plan_ik_candidates(
            ik,
            lambda goal, direct_only, timeout: self.planner.plan(
                start, goal, active_mask=active_mask,
                direct_only=direct_only, timeout=timeout,
                local_frames=(frame_name,) if frame_name in self.planner.frame_names else (),
            ),
            self.robot_model.reduced_mask,
        )
        return KinematicPlan(
            plan=plan, ik=ik, selected_candidate=selected,
            elapsed_time=time.perf_counter() - start_time,
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
