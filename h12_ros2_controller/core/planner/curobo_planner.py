'''cuRobo joint-space planning backend.

Drop-in alternative to ReducedJointPlanner that produces the same PlanResult
(an N x nq reduced-joint waypoint array) so nothing downstream changes.

Requires a CUDA GPU: cuRobo builds on NVIDIA Warp + CUDA PyTorch and compiles
kernels at load. It therefore cannot run in the headless CPU sim (Mac/Colima);
the backend factory only selects it when a GPU is present. All cuRobo/CUDA
imports are lazy so this module stays importable on CPU-only hosts.

NOTE: the cuRobo API calls below follow cuRobo's documented MotionGen interface
but have not been exercised in this repo's CPU sim (no GPU here). Validate on
GPU hardware before relying on it; a version bump may need small API tweaks.
The pure-Python glue (endpoint validation, joint reordering, active-mask
projection, final revalidation) mirrors the OMPL backend's guarantees and is
covered by unit tests.

Generating the cuRobo robot config:
    cuRobo needs its own robot YAML (kinematics + collision spheres + cspace).
    The sphere geometry already exists in CL_Assets/ros_assets/
    (h1_2_handless_sphere.urdf + exclude-pair SRDFs); convert it to cuRobo's
    sphere YAML format and point planner.curobo.robot_cfg at the result.
'''

import numpy as np

from h12_ros2_controller.core.planner.planner_types import (
    PlannerConfig,
    PlanResult,
)


class CuroboJointPlanner:
    '''GPU motion planner exposing the ReducedJointPlanner.plan() contract'''

    def __init__(self, robot_model, config: PlannerConfig = None):
        self.robot_model = robot_model
        self.config = config if config is not None else PlannerConfig()
        self._check_robot_model_ready()

        self.nq = int(self.robot_model.model_body_reduced.nq)
        self.curobo_cfg = dict(getattr(self.config, 'curobo', {}) or {})
        self.frame_names = tuple(self.config.frame_names)
        self.frame_min_z = self._make_frame_floor_map(self.config.frame_z_min)

        # reduced-model joint order the rest of the controller speaks in
        self.reduced_joint_names = self._reduced_joint_names()

        self.validity_checks = 0
        # built lazily on first plan() so process startup stays cheap and the
        # expensive cuRobo warmup only happens when planning is actually used
        self._motion_gen = None
        self._plan_config = None

    # -- setup -------------------------------------------------------------

    def _check_robot_model_ready(self):
        if not getattr(self.robot_model, 'init_reduced', False):
            raise ValueError('RobotModel reduced model is not initialized')
        if not getattr(self.robot_model, 'init_collision', False):
            raise ValueError('RobotModel collision model is not initialized')

    def _reduced_joint_names(self):
        # pinocchio Model.names[0] is the 'universe' joint; the rest map 1:1
        # to the nq columns for the revolute-only reduced arm model
        names = [str(n) for n in self.robot_model.model_body_reduced.names[1:]]
        if len(names) != self.nq:
            raise ValueError(
                'reduced model joint-name count does not match nq; a '
                'non-revolute joint would break the cuRobo column mapping'
            )
        return names

    def _make_frame_floor_map(self, min_z):
        if min_z is None:
            return {}
        if isinstance(min_z, dict):
            return {name: float(z) for name, z in min_z.items()}
        return {name: float(min_z) for name in self.frame_names}

    def _ensure_motion_gen(self):
        if self._motion_gen is not None:
            return

        robot_cfg_path = self.curobo_cfg.get('robot_cfg')
        if not robot_cfg_path:
            raise RuntimeError(
                'planner.curobo.robot_cfg is not set; cuRobo needs a robot '
                'config YAML (kinematics + collision spheres + cspace). See '
                'curobo_planner.py for how to generate it from the existing '
                'sphere URDF.'
            )

        # lazy, GPU-only imports kept out of module import
        import torch
        from curobo.types.base import TensorDeviceType
        from curobo.util_file import load_yaml
        from curobo.geom.types import WorldConfig
        from curobo.wrap.reacher.motion_gen import (
            MotionGen,
            MotionGenConfig,
            MotionGenPlanConfig,
        )

        self._torch = torch
        self._MotionGenPlanConfig = MotionGenPlanConfig

        tensor_args = TensorDeviceType()
        robot_cfg = load_yaml(robot_cfg_path)
        robot_cfg = robot_cfg.get('robot_cfg', robot_cfg)

        world_cfg = self._build_world_config(WorldConfig)

        motion_gen_config = MotionGenConfig.load_from_robot_config(
            robot_cfg,
            world_cfg,
            tensor_args,
            # emit waypoints already spaced at the execution rate so the
            # controller can stream them directly (like OMPL's dt spacing)
            interpolation_dt=float(self.config.dt),
            velocity_scale=float(self.curobo_cfg.get('velocity_scale', 1.0)),
        )
        self._motion_gen = MotionGen(motion_gen_config)
        if self.curobo_cfg.get('warmup', True):
            self._motion_gen.warmup()

        self._plan_config = MotionGenPlanConfig(
            max_attempts=int(self.curobo_cfg.get('max_attempts', 4)),
            timeout=float(self.config.timeout),
            enable_graph=bool(self.curobo_cfg.get('enable_graph', True)),
        )

    def _build_world_config(self, WorldConfig):
        '''Optional world collision (obstacles + a ground plane z floor)'''
        # cuRobo checks robot spheres against the world; a ground plane is the
        # closest world-collision analogue to OMPL's grasp-frame z floor. The
        # exact frame-above-floor guarantee is still enforced by _validate_path
        # below, identically to the OMPL backend.
        world_path = self.curobo_cfg.get('world_cfg')
        if world_path:
            from curobo.util_file import load_yaml
            return WorldConfig.from_dict(load_yaml(world_path))
        return WorldConfig()

    # -- planning ----------------------------------------------------------

    def plan(self, start, goal, active_mask=None):
        '''Plan a collision-free reduced joint path (PlanResult contract)'''
        start = self._as_reduced_array(start, 'start')
        goal = self._as_reduced_array(goal, 'goal')
        active_mask = self._resolve_active_mask(active_mask)

        # pin inactive joints to the start value at both endpoints so untasked
        # limbs never move, matching the OMPL backend's active_mask behavior
        goal = self._project_inactive(goal, active_mask, start)

        self.validity_checks = 0
        for name, q in (('start', start), ('goal', goal)):
            reason = self._validity_failure(q, name)
            if reason:
                return self._failure(reason)

        try:
            self._ensure_motion_gen()
            path = self._plan_curobo(start, goal)
        except Exception as exc:
            return self._failure(f'cuRobo planning error: {exc}')

        if path is None or len(path) == 0:
            return self._failure('cuRobo failed to find a solution')

        # force inactive joints exactly to start along the whole path, then
        # revalidate with the SAME predicates execute_path re-checks
        path[:, ~active_mask] = start[~active_mask]
        reason = self._path_validity_failure(path)
        if reason:
            return self._failure(reason, path=path)

        return PlanResult(
            success=True,
            path=path,
            reason='success',
            planner_name='curobo',
            metadata=self._metadata(path),
        )

    def _plan_curobo(self, start, goal):
        torch = self._torch
        tensor_args = self._motion_gen.tensor_args
        from curobo.types.robot import JointState

        start_state = JointState.from_position(
            tensor_args.to_device(np.asarray([start], dtype=np.float32)),
            joint_names=self.reduced_joint_names,
        )
        goal_state = JointState.from_position(
            tensor_args.to_device(np.asarray([goal], dtype=np.float32)),
            joint_names=self.reduced_joint_names,
        )

        result = self._motion_gen.plan_single_js(
            start_state, goal_state, self._plan_config,
        )
        if not bool(result.success.item()):
            return None

        traj = result.get_interpolated_plan()
        positions = traj.position.detach().cpu().numpy()
        # reorder cuRobo's kinematic joint order back to the reduced order
        src_names = list(getattr(traj, 'joint_names', self.reduced_joint_names))
        return self._reorder_columns(positions, src_names)

    def _reorder_columns(self, positions, src_names):
        positions = np.asarray(positions, dtype=float)
        if list(src_names) == self.reduced_joint_names:
            return positions
        index = {name: col for col, name in enumerate(src_names)}
        try:
            cols = [index[name] for name in self.reduced_joint_names]
        except KeyError as exc:
            raise RuntimeError(
                f'cuRobo trajectory is missing reduced joint {exc}'
            )
        return positions[:, cols]

    # -- active-mask + validity (shared semantics with the OMPL backend) ---

    def _resolve_active_mask(self, active_mask):
        if active_mask is None:
            return np.ones(self.nq, dtype=bool)
        active_mask = np.asarray(active_mask, dtype=bool).reshape(-1)
        if active_mask.shape != (self.nq,):
            raise ValueError(f'active_mask must have shape ({self.nq},)')
        return active_mask

    def _project_inactive(self, q, active_mask, start):
        if np.all(active_mask):
            return q
        q = np.array(q, dtype=float)
        q[~active_mask] = start[~active_mask]
        return q

    def _as_reduced_array(self, q, name):
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.shape != (self.nq,):
            raise ValueError(f'{name} must have shape ({self.nq},)')
        if not np.all(np.isfinite(q)):
            raise ValueError(f'{name} must contain only finite values')
        return q

    def _motor_q(self, q_reduced):
        q = np.copy(self.robot_model.state['q'])
        q[self.robot_model.reduced_mask] = q_reduced
        return q

    def _validity_failure(self, q, name):
        self.validity_checks += 1
        if not self.robot_model.check_within_limits_reduced(q):
            return f'{name} is outside reduced joint limits'
        if not self.robot_model.check_collision_free_reduced(q):
            return f'{name} is in self-collision'
        return self._workspace_constraint_failure(q, name)

    def _workspace_constraint_failure(self, q_reduced, name):
        if not self.frame_min_z:
            return ''
        margin = self.config.frame_z_min_margin
        q = self._motor_q(q_reduced)
        for frame_name in self.frame_names:
            min_z = self.frame_min_z.get(frame_name)
            if min_z is None:
                continue
            z = self.robot_model.get_frame_position(frame_name, q)[2]
            if z + margin < min_z:
                return f'{name} moves {frame_name} below z={min_z:.4f}'
        return ''

    def _path_validity_failure(self, path):
        for idx, q in enumerate(path):
            reason = self._validity_failure(q, f'path waypoint {idx}')
            if reason:
                return reason
        return ''

    # -- result helpers ----------------------------------------------------

    def _failure(self, reason, path=None):
        return PlanResult(
            success=False,
            path=path if path is not None else np.empty((0, self.nq)),
            reason=reason,
            planner_name='curobo',
            metadata={'validity_checks': self.validity_checks},
        )

    def _metadata(self, path):
        return {
            'waypoint_count': int(path.shape[0]),
            'duration': max(path.shape[0] - 1, 1) * float(self.config.dt),
            'validity_checks': self.validity_checks,
        }
