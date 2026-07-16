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

The generated robot configurations live in the CL_Assets submodule alongside
the corresponding sphere URDFs. This module selects the handless or Magpie
asset from RobotModel.handless, so controller configuration never contains a
machine-specific robot path.
'''

import os

import numpy as np

from h12_ros2_controller.core.planner.planner_types import (
    PlannerConfig,
    PlanResult,
)
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS
from h12_ros2_controller.utility.path_definition import (
    CUROBO_HANDLESS_CONFIG_PATH,
    CUROBO_MAGPIE_CONFIG_PATH,
    URDF_HANDLESS_SPHERE_PATH,
    URDF_MAGPIE_SPHERE_PATH,
)

_ARM_REDUCED_NQ = 14
_ARM_MOVE_TOL = 2e-2
_LENGTH_ESTIMATE_STEPS = 50
_FRAME_ROTATION_RADIUS = 0.20
_EMPTY_WORLD = {
    'cuboid': {
        'curobo_empty_world': {
            'pose': [100.0, 100.0, 100.0, 1.0, 0.0, 0.0, 0.0],
            'dims': [0.01, 0.01, 0.01],
        },
    },
}


class CuroboJointPlanner:
    '''GPU motion planner exposing the ReducedJointPlanner.plan() contract'''

    def __init__(self, robot_model, config: PlannerConfig = None):
        self.robot_model = robot_model
        self.config = config if config is not None else PlannerConfig()
        self._check_robot_model_ready()
        self._check_planner_config()

        self.nq = int(self.robot_model.model_body_reduced.nq)
        self.curobo_cfg = dict(getattr(self.config, 'curobo', {}) or {})
        self.frame_names = tuple(self.config.frame_names)
        self.frame_min_z = self._make_frame_floor_map(self.config.frame_z_min)
        self.corridor_min_z = {}

        # reduced-model joint order the rest of the controller speaks in
        self.reduced_joint_names = self._reduced_joint_names()

        self.validity_checks = 0
        # built lazily on first plan() so process startup stays cheap and the
        # expensive cuRobo warmup only happens when planning is actually used
        self._motion_gen = None
        self._plan_config = None
        self._curobo_joint_names = ()
        self._motion_gen_key = None

    # -- setup -------------------------------------------------------------

    def _check_robot_model_ready(self):
        if not getattr(self.robot_model, 'init_reduced', False):
            raise ValueError('RobotModel reduced model is not initialized')
        if not getattr(self.robot_model, 'init_collision', False):
            raise ValueError('RobotModel collision model is not initialized')

    def _check_planner_config(self):
        if self.config.moving_speed <= 0.0:
            raise ValueError('Planner moving_speed must be positive')
        if self.config.dt <= 0.0:
            raise ValueError('Planner dt must be positive')
        if self.config.min_interpolation_steps < 2:
            raise ValueError('Planner min_interpolation_steps must be at least 2')
        if self.config.max_interpolation_steps < self.config.min_interpolation_steps:
            raise ValueError(
                'Planner max_interpolation_steps must be at least '
                'min_interpolation_steps'
            )

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

    def _ensure_motion_gen(self, start, active_mask):
        key = self._make_motion_gen_key(start, active_mask)
        if self._motion_gen is not None and key == self._motion_gen_key:
            return

        robot_cfg_path, urdf_path = self._curobo_asset_paths()
        if not os.path.isfile(robot_cfg_path):
            raise RuntimeError(f'cuRobo robot config is missing: {robot_cfg_path}')
        if not os.path.isfile(urdf_path):
            raise RuntimeError(f'cuRobo sphere URDF is missing: {urdf_path}')

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

        tensor_args = TensorDeviceType()
        robot_cfg = load_yaml(robot_cfg_path)
        robot_cfg = robot_cfg.get('robot_cfg', robot_cfg)

        kin = robot_cfg.setdefault('kinematics', {})
        kin['urdf_path'] = urdf_path
        kin['asset_root_path'] = os.path.dirname(urdf_path)
        lock_joints = dict(kin.get('lock_joints', {}))
        lock_joints['torso_joint'] = self._torso_position()
        for idx, name in enumerate(self.reduced_joint_names):
            if not active_mask[idx]:
                lock_joints[name] = float(start[idx])
        kin['lock_joints'] = lock_joints

        world_cfg = self._build_world_config(WorldConfig)

        motion_gen_config = MotionGenConfig.load_from_robot_config(
            robot_cfg,
            world_cfg,
            tensor_args,
            # emit waypoints already spaced at the execution rate so the
            # controller can stream them directly (like OMPL's dt spacing)
            interpolation_dt=float(self.config.dt),
            use_gradient_descent=bool(
                self.curobo_cfg.get('use_gradient_descent', True)
            ),
        )
        self._motion_gen = MotionGen(motion_gen_config)
        if self.curobo_cfg.get('warmup', True):
            self._motion_gen.warmup()

        active_names = self._active_joint_names(active_mask)
        kinematics = getattr(self._motion_gen, 'kinematics', None)
        names = getattr(kinematics, 'joint_names', None)
        if names and set(names) == set(active_names):
            active_names = list(names)
        self._curobo_joint_names = tuple(active_names)
        self._motion_gen_key = key

        self._plan_config = MotionGenPlanConfig(
            max_attempts=int(self.curobo_cfg.get('max_attempts', 4)),
            timeout=float(self.config.timeout),
            enable_graph=bool(self.curobo_cfg.get('enable_graph', True)),
            time_dilation_factor=float(
                self.curobo_cfg.get('time_dilation_factor', 1.0)
            ),
        )

    def _curobo_asset_paths(self):
        if getattr(self.robot_model, 'handless', False):
            return CUROBO_HANDLESS_CONFIG_PATH, URDF_HANDLESS_SPHERE_PATH
        return CUROBO_MAGPIE_CONFIG_PATH, URDF_MAGPIE_SPHERE_PATH

    def _make_motion_gen_key(self, start, active_mask):
        inactive_q = tuple(np.asarray(start)[~active_mask].tolist())
        return self._torso_position(), tuple(active_mask.tolist()), inactive_q

    def _torso_position(self):
        torso_idx = BODY_JOINTS.index('torso_joint')
        q = self.robot_model.state['q']
        if torso_idx >= len(q):
            return 0.0
        return float(q[torso_idx])

    def _active_joint_names(self, active_mask):
        return [
            name for idx, name in enumerate(self.reduced_joint_names)
            if active_mask[idx]
        ]

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
        return WorldConfig.from_dict(_EMPTY_WORLD)

    # -- planning ----------------------------------------------------------

    def plan(self, start, goal, active_mask=None):
        '''Plan a collision-free reduced joint path (PlanResult contract)'''
        start = self._as_reduced_array(start, 'start')
        goal = self._as_reduced_array(goal, 'goal')
        active_mask = self._resolve_active_mask(active_mask, start, goal)

        # pin inactive joints to the start value at both endpoints so untasked
        # limbs never move, matching the OMPL backend's active_mask behavior
        goal = self._project_inactive(goal, active_mask, start)
        self._set_workspace_corridor(start, goal)

        self.validity_checks = 0
        for name, q in (('start', start), ('goal', goal)):
            reason = self._validity_failure(q, name)
            if reason:
                return self._failure(reason)

        if not np.any(active_mask):
            path, metadata = self._retime_path(np.vstack([start, goal]))
        else:
            try:
                self._ensure_motion_gen(start, active_mask)
                path = self._plan_curobo(start, goal)
            except Exception as exc:
                return self._failure(f'cuRobo planning error: {exc}')

            if path is None or len(path) == 0:
                return self._failure('cuRobo failed to find a solution')
            path, metadata = self._retime_path(path)

        reason = self._path_validity_failure(path)
        metadata['validity_checks'] = self.validity_checks
        if reason:
            return self._failure(reason, path=path)

        return PlanResult(
            success=True,
            path=path,
            reason='success',
            planner_name='curobo',
            metadata=metadata,
        )

    def _plan_curobo(self, start, goal):
        tensor_args = self._motion_gen.tensor_args
        from curobo.types.robot import JointState

        start_state = JointState.from_position(
            tensor_args.to_device(self._curobo_position(start)),
            joint_names=list(self._curobo_joint_names),
        )
        goal_state = JointState.from_position(
            tensor_args.to_device(self._curobo_position(goal)),
            joint_names=list(self._curobo_joint_names),
        )

        result = self._motion_gen.plan_single_js(
            start_state, goal_state, self._plan_config,
        )
        if not bool(result.success.item()):
            return None

        traj = result.get_interpolated_plan()
        positions = traj.position.detach().cpu().numpy()
        # reorder cuRobo's kinematic joint order back to the reduced order
        src_names = list(getattr(traj, 'joint_names', self._curobo_joint_names))
        return self._expand_and_reorder(positions, src_names, start)

    def _curobo_position(self, q):
        index = {name: idx for idx, name in enumerate(self.reduced_joint_names)}
        values = [q[index[name]] for name in self._curobo_joint_names]
        return np.asarray([values], dtype=np.float32)

    def _expand_and_reorder(self, positions, src_names, start):
        positions = np.asarray(positions, dtype=float)
        path = np.repeat(np.asarray(start, dtype=float)[None, :], len(positions), axis=0)
        index = {name: col for col, name in enumerate(src_names)}
        try:
            for dst, name in enumerate(self.reduced_joint_names):
                if name in index:
                    path[:, dst] = positions[:, index[name]]
        except IndexError as exc:
            raise RuntimeError('cuRobo trajectory has invalid joint columns') from exc
        if not set(src_names).issubset(set(self.reduced_joint_names)):
            raise RuntimeError('cuRobo trajectory has unknown joint names')
        return path

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

    def _resolve_active_mask(self, active_mask, start, goal):
        if active_mask is None:
            return self._infer_active_mask(start, goal)
        active_mask = np.asarray(active_mask, dtype=bool).reshape(-1)
        if active_mask.shape != (self.nq,):
            raise ValueError(f'active_mask must have shape ({self.nq},)')
        return active_mask

    def _infer_active_mask(self, start, goal):
        if self.nq != _ARM_REDUCED_NQ:
            return np.ones(self.nq, dtype=bool)
        mask = np.zeros(self.nq, dtype=bool)
        if np.max(np.abs(start[:7] - goal[:7])) > _ARM_MOVE_TOL:
            mask[:7] = True
        if np.max(np.abs(start[7:] - goal[7:])) > _ARM_MOVE_TOL:
            mask[7:] = True
        return mask

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
        if not self.frame_min_z and not self.corridor_min_z:
            return ''
        margin = self.config.frame_z_min_margin
        q = self._motor_q(q_reduced)
        for frame_name in self.frame_names:
            min_z = self.frame_min_z.get(frame_name)
            corridor_z = self.corridor_min_z.get(frame_name)
            if corridor_z is not None:
                min_z = corridor_z if min_z is None else max(min_z, corridor_z)
            if min_z is None:
                continue
            z = self.robot_model.get_frame_position(frame_name, q)[2]
            if z + margin < min_z:
                return f'{name} moves {frame_name} below z={min_z:.4f}'
        return ''

    def _set_workspace_corridor(self, start, goal):
        self.corridor_min_z = {}
        margin = float(self.config.frame_z_corridor_margin)
        for frame_name in self.frame_names:
            start_z = self.robot_model.get_frame_position(
                frame_name, self._motor_q(start),
            )[2]
            goal_z = self.robot_model.get_frame_position(
                frame_name, self._motor_q(goal),
            )[2]
            self.corridor_min_z[frame_name] = min(start_z, goal_z) - margin

    def _path_validity_failure(self, path):
        for idx, q in enumerate(path):
            reason = self._validity_failure(q, f'path waypoint {idx}')
            if reason:
                return reason

        steps = max(1, int(self.config.constraint_check_steps))
        for idx in range(len(path) - 1):
            for sub_idx in range(1, steps):
                alpha = sub_idx / steps
                q = (1.0 - alpha) * path[idx] + alpha * path[idx + 1]
                reason = self._workspace_constraint_failure(
                    q, f'path segment {idx}',
                )
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

    def _sample_path_for_length(self, path):
        if len(path) < 2 or len(path) >= _LENGTH_ESTIMATE_STEPS:
            return path

        segment_lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
        distance = np.concatenate([[0.0], np.cumsum(segment_lengths)])
        if distance[-1] <= 0.0:
            return path

        sample_distance = np.linspace(
            0.0, distance[-1], _LENGTH_ESTIMATE_STEPS,
        )
        return np.asarray([
            np.interp(sample_distance, distance, path[:, idx])
            for idx in range(path.shape[1])
        ]).T

    def _frame_path_length(self, path, frame_name):
        if len(path) < 2:
            return 0.0, 0.0, 0.0

        motor_path = [self._motor_q(q) for q in path]
        positions = np.asarray([
            self.robot_model.get_frame_position(frame_name, q)
            for q in motor_path
        ])
        translation = float(np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1)))

        rotation = 0.0
        if hasattr(self.robot_model, 'get_frame_rotation'):
            rotations = [
                self.robot_model.get_frame_rotation(frame_name, q)
                for q in motor_path
            ]
            angle = sum(
                self._rotation_angle(rotations[idx], rotations[idx + 1])
                for idx in range(len(rotations) - 1)
            )
            rotation = _FRAME_ROTATION_RADIUS * angle
        return translation + rotation, translation, rotation

    def _rotation_angle(self, rotation_a, rotation_b):
        rotation_delta = rotation_a.T @ rotation_b
        cos_angle = 0.5 * (np.trace(rotation_delta) - 1.0)
        return float(np.arccos(np.clip(cos_angle, -1.0, 1.0)))

    def _interpolation_metadata(self, path):
        sample_path = self._sample_path_for_length(path)
        frame_lengths = {
            frame_name: self._frame_path_length(sample_path, frame_name)
            for frame_name in self.frame_names
        }
        lengths = {name: values[0] for name, values in frame_lengths.items()}
        translations = {
            name: values[1] for name, values in frame_lengths.items()
        }
        rotations = {name: values[2] for name, values in frame_lengths.items()}
        path_length = max(lengths.values(), default=0.0)
        steps = int(np.ceil(
            path_length / float(self.config.moving_speed) / float(self.config.dt),
        )) + 1
        steps = max(int(self.config.min_interpolation_steps), steps)
        steps = min(int(self.config.max_interpolation_steps), steps)
        duration = max(steps - 1, 1) * float(self.config.dt)
        return {
            'waypoint_count': steps,
            'duration': duration,
            'effective_speed': path_length / duration,
            'path_length': path_length,
            'frame_path_lengths': lengths,
            'frame_translation_lengths': translations,
            'frame_rotation_lengths': rotations,
            'length_estimate_samples': len(sample_path),
        }

    def _retime_path(self, path):
        metadata = self._interpolation_metadata(path)
        steps = max(metadata['waypoint_count'], len(path))
        steps = min(steps, int(self.config.max_interpolation_steps))
        source = np.linspace(0.0, 1.0, len(path))
        target = self._smoothstep(steps)
        path = np.asarray([
            np.interp(target, source, path[:, idx])
            for idx in range(path.shape[1])
        ]).T
        metadata['waypoint_count'] = int(path.shape[0])
        metadata['actual_waypoints'] = int(path.shape[0])
        metadata['duration'] = max(len(path) - 1, 1) * float(self.config.dt)
        metadata['validity_checks'] = self.validity_checks
        return path, metadata

    def _smoothstep(self, steps):
        t = np.linspace(0.0, 1.0, steps)
        return t * t * (3.0 - 2.0 * t)
