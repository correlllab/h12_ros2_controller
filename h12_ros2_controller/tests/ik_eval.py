'''
Frame-controller IK accuracy / repeatability evaluator.

Runs a configurable matrix of (case x sweep-point x trial) frame-task goals
against FrameController. For every trial it records a per-control-step log:
  q_actual, q_cmd, dq_actual, tau_actual, tau_cmd, frame_err, ee_actual,
  target, twist, convergence step.

Output layout (under runs/<timestamp>_<label>/):
  summary.csv            - one row per trial (top-level metrics)
  meta.yaml              - resolved config + joint-limit snapshot
  trials/<case>__<sweep>__t<N>.npz   - per-trial raw timeseries
  plots/                 - produced by plot_ik_eval.py

Modes:
  real   use control_step_reduced and publish to robot
  sim    use sim_step_reduced (pinocchio-only rollout)

Usage:
  python -m h12_ros2_controller.tests.ik_eval --config minimal.yaml --mode sim
  python -m h12_ros2_controller.tests.ik_eval --config sweep.yaml --mode real --yes
'''
import argparse
import csv
import os
import sys
import time
import traceback
from dataclasses import dataclass, field
from datetime import datetime
from itertools import product
from pathlib import Path

import numpy as np
import pinocchio as pin
import yaml

from h12_ros2_controller.core.controller.frame_controller import FrameController
from h12_ros2_controller.utility.controller_config import (
    load_controller_config,
    initialize_channel_factory,
)
from h12_ros2_controller.utility.joint_definition import (
    BODY_JOINTS,
    ENABLED_JOINTS,
)
from h12_ros2_controller.utility.joint_limits import (
    JOINT_POSITION_LIMITS,
    JOINT_TORQUE_LIMITS,
    JOINT_VELOCITY_LIMITS,
)
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS

REPO_ROOT = Path(__file__).resolve().parents[2]
TESTS_CONFIG_DIR = Path(__file__).resolve().parent / 'configs'

REDUCED_JOINT_NAMES = list(ENABLED_JOINTS)
REDUCED_BODY_IDS = np.array(
    [BODY_JOINTS.index(j) for j in ENABLED_JOINTS], dtype=int
)


# ---------------------------- config ----------------------------------------


def _candidate_config_roots() -> list[Path]:
    '''Directories that might contain tests/configs/*.yaml.

    Order matters: first hit wins. Covers source tree, running from repo root,
    running from tests/, and the install-tree fallback that searches up for a
    colcon workspace's src/ sibling.
    '''
    roots: list[Path] = []
    roots.append(TESTS_CONFIG_DIR)                       # next to __file__
    cwd = Path.cwd()
    roots.append(cwd)                                    # bare path from cwd
    roots.append(cwd / 'configs')                        # running from tests/
    roots.append(cwd / 'h12_ros2_controller' / 'tests' / 'configs')  # repo root cwd

    # install-tree fallback: climb to a workspace and look for src/<pkg>
    probe = Path(__file__).resolve()
    for parent in probe.parents:
        src_candidate = (parent.parent / 'src' / 'h12_ros2_controller'
                         / 'h12_ros2_controller' / 'tests' / 'configs')
        if src_candidate.is_dir():
            roots.append(src_candidate)
            break
    # de-dupe preserving order
    seen: set[Path] = set()
    unique: list[Path] = []
    for r in roots:
        rp = r.resolve() if r.exists() else r
        if rp in seen:
            continue
        seen.add(rp)
        unique.append(r)
    return unique


def _resolve_test_config(name: str) -> Path:
    p = Path(name)
    if not p.suffix:
        p = p.with_suffix('.yaml')
    if p.is_absolute():
        if p.exists():
            return p
        raise FileNotFoundError(f'test config not found: {p}')

    tried: list[Path] = []
    for root in _candidate_config_roots():
        candidate = (root / p) if not root.name == p.name else root
        tried.append(candidate)
        if candidate.is_file():
            return candidate.resolve()

    tried_str = '\n  '.join(str(t) for t in tried)
    raise FileNotFoundError(
        f"test config not found: '{name}'\n"
        f"searched:\n  {tried_str}\n"
        f"hint: run from the repo source root, or pass an absolute path."
    )


def _load_test_config(name: str) -> dict:
    path = _resolve_test_config(name)
    with path.open('r', encoding='utf-8') as handle:
        cfg = yaml.safe_load(handle) or {}
    cfg['__path__'] = str(path)
    return cfg


def _resolve_home(home_field) -> np.ndarray:
    if isinstance(home_field, str):
        if home_field not in NAMED_CONFIGS:
            raise ValueError(f'unknown named config: {home_field}')
        return np.asarray(NAMED_CONFIGS[home_field], dtype=float)
    arr = np.asarray(home_field, dtype=float)
    if arr.shape != (len(ENABLED_JOINTS),):
        raise ValueError(
            f'home must be {len(ENABLED_JOINTS)} floats, got shape {arr.shape}'
        )
    return arr


def _expand_sweeps(sweeps: dict) -> list[dict]:
    if not sweeps:
        return [{}]
    keys = list(sweeps.keys())
    grids = [list(sweeps[k]) for k in keys]
    return [dict(zip(keys, combo)) for combo in product(*grids)]


def _sweep_label(sweep: dict) -> str:
    if not sweep:
        return 'default'
    parts = [f'{k}={v}' for k, v in sweep.items()]
    return '_'.join(parts).replace(' ', '')


# ---------------------------- recording -------------------------------------


@dataclass
class TrialRecord:
    case: str = ''
    frame: str = ''
    sweep: dict = field(default_factory=dict)
    trial_index: int = 0
    mode: str = 'sim'
    dt: float = 0.02

    target_pose: np.ndarray = field(default_factory=lambda: np.zeros(6))
    q_ik_optimal: np.ndarray | None = None
    ik_optimal_success: bool = False

    t: list = field(default_factory=list)
    frame_err: list = field(default_factory=list)
    ee_actual: list = field(default_factory=list)
    q_actual: list = field(default_factory=list)
    q_cmd: list = field(default_factory=list)
    dq_actual: list = field(default_factory=list)
    tau_actual: list = field(default_factory=list)
    tau_cmd: list = field(default_factory=list)
    twist: list = field(default_factory=list)

    converged: bool = False
    convergence_step: int = -1
    wall_time_s: float = 0.0

    def push(self, **kwargs):
        for key, value in kwargs.items():
            getattr(self, key).append(np.asarray(value, dtype=float))

    def arrays(self) -> dict:
        return {
            't': np.asarray(self.t),
            'frame_err': np.asarray(self.frame_err),
            'ee_actual': np.asarray(self.ee_actual),
            'q_actual': np.asarray(self.q_actual),
            'q_cmd': np.asarray(self.q_cmd),
            'dq_actual': np.asarray(self.dq_actual),
            'tau_actual': np.asarray(self.tau_actual),
            'tau_cmd': np.asarray(self.tau_cmd),
            'twist': np.asarray(self.twist),
        }

    def save(self, path: Path, limits_cfg: dict):
        arrays = self.arrays()
        np.savez(
            path,
            case=self.case,
            frame=self.frame,
            sweep_keys=np.array(list(self.sweep.keys())),
            sweep_values=np.array(list(self.sweep.values()), dtype=float),
            trial_index=self.trial_index,
            mode=self.mode,
            dt=self.dt,
            target_pose=self.target_pose,
            q_ik_optimal=(
                np.full(len(BODY_JOINTS), np.nan)
                if self.q_ik_optimal is None
                else self.q_ik_optimal
            ),
            ik_optimal_success=self.ik_optimal_success,
            converged=self.converged,
            convergence_step=self.convergence_step,
            wall_time_s=self.wall_time_s,
            reduced_body_ids=REDUCED_BODY_IDS,
            reduced_joint_names=np.array(REDUCED_JOINT_NAMES),
            # limits snapshot (for plotting without reloading controller)
            urdf_q_low=limits_cfg['urdf_q_low'],
            urdf_q_high=limits_cfg['urdf_q_high'],
            urdf_tau_limit=limits_cfg['urdf_tau_limit'],
            urdf_dq_limit=limits_cfg['urdf_dq_limit'],
            clip_q_low=limits_cfg['clip_q_low'],
            clip_q_high=limits_cfg['clip_q_high'],
            clip_tau_limit=limits_cfg['clip_tau_limit'],
            clip_dq_limit=limits_cfg['clip_dq_limit'],
            **arrays,
        )


# ---------------------------- trial runner ----------------------------------


def _pose_to_target(pose_list, units: str) -> np.ndarray:
    pose = np.asarray(pose_list, dtype=float)
    if pose.shape != (6,):
        raise ValueError(f'target_pose must be 6 floats, got {pose.shape}')
    if units == 'deg':
        pose = pose.copy()
        pose[3:] = np.deg2rad(pose[3:])
    elif units != 'rad':
        raise ValueError(f"orientation_units must be 'rad' or 'deg', got {units}")
    return pose


def _frame_err_norms(err6: np.ndarray) -> tuple[float, float]:
    return float(np.linalg.norm(err6[:3])), float(np.linalg.norm(err6[3:]))


def _go_home(controller: FrameController, home_q: np.ndarray,
             timeout: float, tolerance: float, mode: str, dt: float) -> bool:
    '''Drive the reduced configuration toward home_q and return True on success.'''
    step_function = (
        controller.sim_goto_reduced_configuration
        if mode == 'sim'
        else controller.goto_reduced_configuration
    )
    start = time.time()
    last_err = np.inf
    while time.time() - start < timeout:
        frame_start = time.time()
        step_function(home_q)
        err = np.linalg.norm(controller.robot_model.state_reduced['q'] - home_q)
        last_err = err
        if err < tolerance:
            # hold a handful of steps so the PD loop can settle
            for _ in range(20):
                hold_start = time.time()
                step_function(home_q)
                time.sleep(max(0.0, dt - (time.time() - hold_start)))
            return True
        time.sleep(max(0.0, dt - (time.time() - frame_start)))
    print(f'  [home] timeout, residual norm={last_err:.4f}', flush=True)
    return False


def _maybe_solve_ik_optimal(controller: FrameController, task_name: str,
                            trial_cfg: dict) -> tuple[np.ndarray | None, bool]:
    '''Run pink's iterative solve_ik_reduced once as a "reference optimum".

    This resets the IK configuration to zero and runs to convergence; we
    sync back to the real state before returning, so the live control loop
    is unaffected. Failures (collision, infeasible) are swallowed.
    '''
    if not trial_cfg.get('record_ik_optimal', True):
        return None, False
    try:
        result = controller.ik_solver.solve_ik_reduced(
            alpha=float(trial_cfg.get('ik_alpha', 0.1)),
            timeout=float(trial_cfg.get('ik_solve_timeout', 1.0)),
            linear_threshold=float(trial_cfg['linear_threshold']),
            angular_threshold=float(trial_cfg['angular_threshold']),
        )
        return np.asarray(result['q'], dtype=float), bool(result['success'])
    except Exception as exc:
        print(f'  [ik_optimal] skipped: {exc}', flush=True)
        return None, False
    finally:
        controller.update_ik_solver()  # resync to robot state


def _run_single_trial(controller: FrameController, case_cfg: dict,
                      sweep_cfg: dict, trial_cfg: dict, trial_index: int,
                      orientation_units: str, mode: str,
                      limits_cfg: dict) -> TrialRecord:
    frame = case_cfg['frame']
    target_pose = _pose_to_target(case_cfg['target_pose'], orientation_units)
    task_name = f'{frame}_task'

    rec = TrialRecord(
        case=case_cfg['name'],
        frame=frame,
        sweep=dict(sweep_cfg),
        trial_index=trial_index,
        mode=mode,
        dt=controller.dt,
        target_pose=target_pose.copy(),
    )

    # 1) rebuild frame task with this sweep's weights
    controller.ik_solver.clear_frame_tasks()
    controller.ik_solver.add_frame_task(
        task_name, frame,
        position_cost=float(sweep_cfg.get('position_cost', 50.0)),
        orientation_cost=float(sweep_cfg.get('orientation_cost', 30.0)),
        lm_damping=float(sweep_cfg.get('lm_damping', 3.0)),
    )
    controller.set_frame_task_pose(task_name, target_pose)
    controller.update_ik_solver()

    # 2) reference IK-optimal (iterative pink solve_ik, starting from zero)
    rec.q_ik_optimal, rec.ik_optimal_success = _maybe_solve_ik_optimal(
        controller, task_name, trial_cfg
    )

    # 3) control loop
    step_function = (
        (lambda: controller.sim_step_reduced(com=False))
        if mode == 'sim'
        else (lambda: controller.control_step_reduced(com=False))
    )

    lin_threshold = float(trial_cfg['linear_threshold'])
    ang_threshold = float(trial_cfg['angular_threshold'])
    step_timeout = float(trial_cfg['step_timeout'])
    settle_steps = int(trial_cfg['settle_steps'])

    dt = controller.dt
    step = 0
    start_wall = time.time()
    converged = False
    convergence_step = -1

    # Read PD gains once — used to compute actual motor torque each step.
    # dq/tau from rt/lowstate are unreliable in safety_split mode, so we derive
    # them: dq from successive q differences, tau from gravity + PD feedback.
    pub = controller.low_cmd_handler._command_publisher
    with pub.data_lock:
        kp_gains = np.copy(pub.kp)
        kd_gains = np.copy(pub.kd)

    while time.time() - start_wall < step_timeout:
        frame_start = time.time()
        # snapshot BEFORE step (state reflects last commanded)
        pre_state = controller.robot_model.state
        pre_state_reduced = controller.robot_model.state_reduced

        step_function()

        # post-step state — q is updated (from ik_solver in sim, hardware in real)
        post_state = controller.robot_model.state

        # post-step captures
        q_cmd_full = np.copy(controller.ik_solver.q)
        tau_cmd_full = np.copy(
            controller.robot_model.dynamics.get_gravity_compensation(pre_state['q'])
        )
        err = controller.get_frame_task_error(task_name)
        ee_actual = np.concatenate([
            controller.robot_model.get_frame_position_reduced(frame),
            pin.rpy.matrixToRpy(
                controller.robot_model.get_frame_rotation_reduced(frame)
            ),
        ])

        # dq from successive q readings (reliable in both sim and real mode)
        dq_actual = (post_state['q'] - pre_state['q']) / dt
        # full motor torque: gravity feedforward + PD feedback
        tau_actual = (tau_cmd_full
                      + kp_gains * (q_cmd_full - pre_state['q'])
                      - kd_gains * dq_actual)

        twist = controller.robot_model.dynamics.compute_frame_twist(frame, dq_actual)

        rec.push(
            t=np.array([step * dt]),
            frame_err=err,
            ee_actual=ee_actual,
            q_actual=pre_state['q'],
            q_cmd=q_cmd_full,
            dq_actual=dq_actual,
            tau_actual=tau_actual,
            tau_cmd=tau_cmd_full,
            twist=twist,
        )

        lin_err, ang_err = _frame_err_norms(err)
        if not converged and lin_err < lin_threshold and ang_err < ang_threshold:
            converged = True
            convergence_step = step
            print(
                f'    converged at step {step} '
                f'(lin={lin_err:.4f} m, ang={ang_err:.4f} rad)',
                flush=True,
            )
        if converged and step >= convergence_step + settle_steps:
            break
        step += 1
        time.sleep(max(0.0, dt - (time.time() - frame_start)))

    rec.converged = converged
    rec.convergence_step = convergence_step
    rec.wall_time_s = time.time() - start_wall
    if not converged:
        lin_err, ang_err = _frame_err_norms(rec.frame_err[-1])
        print(
            f'    TIMEOUT after {rec.wall_time_s:.2f}s '
            f'(lin={lin_err:.4f} m, ang={ang_err:.4f} rad)',
            flush=True,
        )
    return rec


# ---------------------------- limits snapshot -------------------------------


def _real_mode_ros_env_sanity(controller_cfg_name: str) -> None:
    '''Print and warn about ROS transport env that must match safety node setup.

    This runner evaluates through ROS safety topics in real mode. Keep the same
    ROS overlay / middleware settings as the safety node process to avoid transport
    mismatches and silent non-delivery.
    '''
    rmw = os.environ.get('RMW_IMPLEMENTATION', '')
    uri = os.environ.get('CYCLONEDDS_URI', '')
    domain = os.environ.get('ROS_DOMAIN_ID', '')
    print('[env] RMW_IMPLEMENTATION =', rmw or '(unset)')
    print('[env] ROS_DOMAIN_ID      =', domain or '(unset)')
    print('[env] CYCLONEDDS_URI     =', uri or '(unset)')

    warnings = []
    if not rmw:
        warnings.append(
            'RMW_IMPLEMENTATION is unset. Ensure this shell uses the same ROS '
            'middleware setting and overlay as the running safety node.'
        )
    if not domain:
        warnings.append(
            'ROS_DOMAIN_ID is unset. If your safety node uses a non-default domain, '
            'set the same value here before running.'
        )

    # Also remind about controller/safety suffix matching (launch file enforces this).
    stem = Path(controller_cfg_name).stem
    if stem not in ('safety_full', 'safety_split', 'debug'):
        warnings.append(
            f"controller_config '{controller_cfg_name}' is unusual; expected "
            "safety_full.yaml, safety_split.yaml, or debug.yaml."
        )
    if stem == 'safety_split':
        print(
            '[env] publishing to rt/safety/lowcmd_upper_in — requires a '
            'split-mode safety node subscribed there.'
        )
    elif stem == 'safety_full':
        print(
            '[env] publishing to rt/safety/lowcmd_in — requires a '
            'full-mode safety node subscribed there.'
        )
    elif stem == 'debug':
        warnings.append(
            'debug.yaml bypasses the safety layer (publishes to rt/lowcmd '
            'directly). Use only if you are intentionally sim-testing without '
            'safety.'
        )

    for w in warnings:
        print(f'[env][WARN] {w}')
    if warnings:
        print('[env] continuing in 2s… (Ctrl-C to abort)')
        time.sleep(2.0)


def _limits_snapshot(controller_cfg: dict) -> dict:
    '''Extract URDF + software (clip) limits as plain 1D arrays of length 27.'''
    n = len(BODY_JOINTS)
    urdf_q_low = np.array([JOINT_POSITION_LIMITS[i]['low'] for i in range(n)])
    urdf_q_high = np.array([JOINT_POSITION_LIMITS[i]['high'] for i in range(n)])
    urdf_tau = np.asarray(JOINT_TORQUE_LIMITS, dtype=float)
    urdf_dq = np.asarray(JOINT_VELOCITY_LIMITS, dtype=float)

    limits = controller_cfg.get('limits', {})
    q_clip = limits.get('q_clip_limits')
    dq_clip = limits.get('dq_clip_limits')
    tau_clip = limits.get('tau_clip_limits')

    return {
        'urdf_q_low': urdf_q_low,
        'urdf_q_high': urdf_q_high,
        'urdf_tau_limit': urdf_tau,
        'urdf_dq_limit': urdf_dq,
        'clip_q_low': np.asarray(q_clip[:, 0], dtype=float),
        'clip_q_high': np.asarray(q_clip[:, 1], dtype=float),
        'clip_tau_limit': np.asarray(tau_clip, dtype=float),
        'clip_dq_limit': np.asarray(dq_clip, dtype=float),
    }


# ---------------------------- orchestration ---------------------------------


def _run_fk_roundtrip(controller: FrameController, fk_cfg: dict,
                      trial_cfg: dict, out_dir: Path) -> None:
    '''Offline: sample feasible reduced q, FK -> pose, IK -> q_hat, record residual.

    Pure IK-solver diagnostic. If this has high residuals, the issue is the
    solver or the model, not the robot/calibration.
    '''
    samples = int(fk_cfg.get('samples', 100))
    seed = int(fk_cfg.get('seed', 0))
    frame = fk_cfg.get('frame', 'left_wrist_yaw_link')
    margin = float(fk_cfg.get('joint_limit_margin', 0.05))

    rng = np.random.default_rng(seed)
    # per-joint bounds (reduced)
    low = np.array([
        JOINT_POSITION_LIMITS[i]['low'] for i in REDUCED_BODY_IDS
    ])
    high = np.array([
        JOINT_POSITION_LIMITS[i]['high'] for i in REDUCED_BODY_IDS
    ])
    span = high - low
    low_m = low + margin * span
    high_m = high - margin * span

    results = {
        'q_true': [],
        'q_hat': [],
        'pose_target': [],
        'pose_hat': [],
        'pos_residual': [],
        'ang_residual': [],
        'success': [],
    }

    task_name = f'{frame}_fk_roundtrip'
    controller.ik_solver.clear_frame_tasks()
    controller.ik_solver.add_frame_task(task_name, frame)

    print(f'[fk_roundtrip] frame={frame} samples={samples}', flush=True)
    for _ in range(samples):
        q_true = rng.uniform(low_m, high_m)
        # FK via reduced model
        T = controller.robot_model.get_frame_transformation_reduced(frame, q_true)
        pos = T[:3, 3]
        rpy = pin.rpy.matrixToRpy(T[:3, :3])
        pose_target = np.concatenate([pos, rpy])

        controller.set_frame_task_transformation(task_name, T)
        res = controller.ik_solver.solve_ik_reduced(
            alpha=float(trial_cfg.get('ik_alpha', 0.1)),
            timeout=float(trial_cfg.get('ik_solve_timeout', 1.0)),
            linear_threshold=float(trial_cfg['linear_threshold']),
            angular_threshold=float(trial_cfg['angular_threshold']),
        )
        q_hat_full = np.asarray(res['q'], dtype=float)
        # solve_ik_reduced returns full body-q (27)
        q_hat_reduced = q_hat_full[REDUCED_BODY_IDS]
        T_hat = controller.robot_model.get_frame_transformation_reduced(
            frame, q_hat_reduced
        )
        pose_hat = np.concatenate([
            T_hat[:3, 3], pin.rpy.matrixToRpy(T_hat[:3, :3])
        ])
        pos_res = float(np.linalg.norm(pose_hat[:3] - pose_target[:3]))
        ang_res = float(np.linalg.norm(pose_hat[3:] - pose_target[3:]))

        results['q_true'].append(q_true)
        results['q_hat'].append(q_hat_reduced)
        results['pose_target'].append(pose_target)
        results['pose_hat'].append(pose_hat)
        results['pos_residual'].append(pos_res)
        results['ang_residual'].append(ang_res)
        results['success'].append(bool(res['success']))

    controller.ik_solver.clear_frame_tasks()
    controller.update_ik_solver()

    out = out_dir / 'fk_roundtrip.npz'
    np.savez(out, **{k: np.asarray(v) for k, v in results.items()},
             reduced_joint_names=np.array(REDUCED_JOINT_NAMES),
             frame=frame)
    pos = np.array(results['pos_residual'])
    ang = np.array(results['ang_residual'])
    succ = np.array(results['success'])
    print(
        f'[fk_roundtrip] success={succ.mean()*100:.1f}%  '
        f'pos_residual: mean={pos.mean():.4f} m  p95={np.percentile(pos, 95):.4f} m  '
        f'ang_residual: mean={ang.mean():.4f} rad  p95={np.percentile(ang, 95):.4f} rad',
        flush=True,
    )


def _summary_row(rec: TrialRecord) -> dict:
    arrs = rec.arrays()
    n = arrs['frame_err'].shape[0] if arrs['frame_err'].ndim == 2 else 0
    tail = max(1, n // 10)
    if n > 0:
        lin_ss = np.linalg.norm(arrs['frame_err'][-tail:, :3], axis=1).mean()
        ang_ss = np.linalg.norm(arrs['frame_err'][-tail:, 3:], axis=1).mean()
        ee_mean = arrs['ee_actual'][-tail:].mean(axis=0)
        q_mean = arrs['q_actual'][-tail:].mean(axis=0)
    else:
        lin_ss = ang_ss = float('nan')
        ee_mean = np.full(6, np.nan)
        q_mean = np.full(len(BODY_JOINTS), np.nan)

    row = {
        'case': rec.case,
        'frame': rec.frame,
        'trial_index': rec.trial_index,
        'mode': rec.mode,
        'converged': int(rec.converged),
        'convergence_step': rec.convergence_step,
        'wall_time_s': f'{rec.wall_time_s:.3f}',
        'lin_err_steady_m': f'{lin_ss:.5f}',
        'ang_err_steady_rad': f'{ang_ss:.5f}',
        'ik_optimal_success': int(rec.ik_optimal_success),
    }
    row.update({f'sweep__{k}': v for k, v in rec.sweep.items()})
    row.update({f'ee_ss__{a}': f'{ee_mean[i]:.5f}'
                for i, a in enumerate(['x', 'y', 'z', 'r', 'p', 'yw'])})
    # record which reduced joints we are using so summary readers know the order
    return row


def run(args):
    test_cfg = _load_test_config(args.config)
    label = args.label or Path(test_cfg['__path__']).stem
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir = REPO_ROOT / 'runs' / f'{timestamp}_{label}_{args.mode}'
    trials_dir = run_dir / 'trials'
    trials_dir.mkdir(parents=True, exist_ok=True)
    print(f'[run] output dir: {run_dir}', flush=True)

    # load controller config
    controller_cfg_name = test_cfg.get('controller_config', 'debug.yaml')
    controller_cfg = load_controller_config(controller_cfg_name)
    if args.mode == 'real':
        _real_mode_ros_env_sanity(controller_cfg_name)
        initialize_channel_factory(controller_cfg)
        if not args.yes:
            ans = input(
                '[real] This will command the physical robot. Continue? [y/N]: '
            ).strip().lower()
            if ans != 'y':
                print('[real] aborted by user')
                return 1
    else:
        initialize_channel_factory(controller_cfg)

    urdf = test_cfg['urdf']
    controller = FrameController(
        urdf_path=urdf['body'],
        urdf_sphere_path=urdf['sphere'],
        srdf_sphere_path=urdf['srdf'],
        handless=bool(test_cfg.get('handless', False)),
        visualize=bool(args.visualize),
        config=controller_cfg,
    )

    limits_cfg = _limits_snapshot(controller_cfg)

    # persist resolved config + limits for the plotting step
    meta = {
        'label': label,
        'mode': args.mode,
        'timestamp': timestamp,
        'test_config': test_cfg,
        'controller_config_name': controller_cfg_name,
    }
    with (run_dir / 'meta.yaml').open('w', encoding='utf-8') as fh:
        yaml.safe_dump(meta, fh, sort_keys=False)

    home_q = _resolve_home(test_cfg.get('home', 'home'))
    orientation_units = str(test_cfg.get('orientation_units', 'rad'))
    trial_cfg = test_cfg.get('trial', {})
    sweeps = _expand_sweeps(test_cfg.get('sweeps', {}))
    trials_per_case = int(test_cfg.get('trials_per_case', 3))
    return_home = bool(test_cfg.get('return_home_between', True))

    summary_path = run_dir / 'summary.csv'
    summary_fh = summary_path.open('w', newline='', encoding='utf-8')
    writer: csv.DictWriter | None = None

    exit_code = 0
    try:
        # optional offline diagnostic before anything else
        fk_cfg = test_cfg.get('fk_roundtrip', {}) or {}
        if fk_cfg.get('enabled', False):
            _run_fk_roundtrip(controller, fk_cfg, trial_cfg, run_dir)

        # go home first
        print(f'[home] driving to home configuration (mode={args.mode})',
              flush=True)
        _go_home(
            controller, home_q,
            timeout=float(test_cfg.get('home_settle_timeout', 6.0)),
            tolerance=float(test_cfg.get('home_tolerance', 2e-2)),
            mode=args.mode, dt=controller.dt,
        )

        for case_cfg in test_cfg.get('cases', []):
            for sweep in sweeps:
                sweep_tag = _sweep_label(sweep)
                for trial_index in range(trials_per_case):
                    print(
                        f'[trial] case={case_cfg["name"]} sweep={sweep_tag} '
                        f'trial={trial_index + 1}/{trials_per_case}',
                        flush=True,
                    )
                    if return_home:
                        _go_home(
                            controller, home_q,
                            timeout=float(test_cfg.get('home_settle_timeout', 6.0)),
                            tolerance=float(test_cfg.get('home_tolerance', 2e-2)),
                            mode=args.mode, dt=controller.dt,
                        )
                    rec = _run_single_trial(
                        controller, case_cfg, sweep, trial_cfg,
                        trial_index, orientation_units, args.mode, limits_cfg,
                    )
                    out_name = f'{case_cfg["name"]}__{sweep_tag}__t{trial_index}.npz'
                    rec.save(trials_dir / out_name, limits_cfg)

                    row = _summary_row(rec)
                    if writer is None:
                        writer = csv.DictWriter(summary_fh, fieldnames=list(row.keys()))
                        writer.writeheader()
                    writer.writerow(row)
                    summary_fh.flush()

    except KeyboardInterrupt:
        print('\n[interrupt] user requested stop', flush=True)
        exit_code = 130
    except Exception:
        traceback.print_exc()
        exit_code = 1
    finally:
        summary_fh.close()
        try:
            controller.shutdown()
        except Exception as exc:
            print(f'[shutdown] warning: {exc}', flush=True)

    print(f'[done] summary: {summary_path}', flush=True)
    return exit_code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='minimal.yaml',
                        help='test config name under tests/configs/ or abs path')
    parser.add_argument('--mode', choices=['sim', 'real'], default='sim')
    parser.add_argument('--visualize', action='store_true',
                        help='launch meshcat visualizer')
    parser.add_argument('--yes', action='store_true',
                        help='skip real-mode confirmation prompt')
    parser.add_argument('--label', default=None,
                        help='extra label for the run output dir')
    args = parser.parse_args()
    sys.exit(run(args))


if __name__ == '__main__':
    main()
