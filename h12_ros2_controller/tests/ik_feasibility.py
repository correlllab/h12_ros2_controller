'''
Offline IK feasibility check — no hardware required.

Evaluates a grid of candidate end-effector targets against the pinocchio model
and reports reachability, residual error, required joint motion from home, and
joint-limit margins.  Use this to vet sweep.yaml targets before running on
the real robot.

Usage:
  python -m h12_ros2_controller.tests.ik_feasibility
'''
import sys
import warnings
from pathlib import Path

import numpy as np
import pinocchio as pin

REPO_ROOT = Path(__file__).resolve().parents[2]

from h12_ros2_controller.core.ik_solver import IKSolver
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.utility.joint_definition import ENABLED_JOINTS

# ── model paths (same as sweep.yaml) ─────────────────────────────────────────
_URDF       = str(REPO_ROOT / 'assets/h1_2/h1_2_handless.urdf')
_URDF_SPHERE = str(REPO_ROOT / 'assets/h1_2/h1_2_handless_sphere.urdf')
_SRDF       = str(REPO_ROOT / 'assets/h1_2/h1_2_handless_sphere_collision.srdf')

# ── IK convergence thresholds (match sweep.yaml) ─────────────────────────────
_LIN_THRESH = 5e-3   # m
_ANG_THRESH = 2e-2   # rad
_IK_TIMEOUT = 2.0    # s  (longer than real sweep for offline check)
_IK_ALPHA   = 0.1

# ── candidate targets ─────────────────────────────────────────────────────────
# Format: (name, frame, [x, y, z, roll, pitch, yaw])
# home EE observed at ~[0.237, 0.201, 0.089] for left_wrist_yaw_link
CANDIDATES = [
    # ── existing cases (kept for baseline comparison) ─────────────────────────
    ('left_forward_low',     'left_wrist_yaw_link',  [0.30,  0.20,  0.10, 0, 0, 0]),
    ('left_forward_high',    'left_wrist_yaw_link',  [0.30,  0.20,  0.30, 0, 0, 0]),
    ('left_side_reach',      'left_wrist_yaw_link',  [0.15,  0.35,  0.20, 0, 0, 0]),
    ('right_forward_low',    'right_wrist_yaw_link', [0.30, -0.20,  0.10, 0, 0, 0]),
    ('right_forward_high',   'right_wrist_yaw_link', [0.30, -0.20,  0.30, 0, 0, 0]),
    ('right_side_reach',     'right_wrist_yaw_link', [0.15, -0.35,  0.20, 0, 0, 0]),

    # ── proposed larger-motion targets ────────────────────────────────────────
    # left arm
    ('left_elbow_flex',      'left_wrist_yaw_link',  [0.20,  0.20,  0.25, 0, 0, 0]),
    ('left_forward_mid',     'left_wrist_yaw_link',  [0.30,  0.20,  0.20, 0, 0, 0]),
    ('left_forward_extend',  'left_wrist_yaw_link',  [0.35,  0.20,  0.10, 0, 0, 0]),
    ('left_forward_hi_mid',  'left_wrist_yaw_link',  [0.28,  0.20,  0.25, 0, 0, 0]),
    ('left_inward',          'left_wrist_yaw_link',  [0.30,  0.08,  0.10, 0, 0, 0]),
    ('left_side_up',         'left_wrist_yaw_link',  [0.20,  0.35,  0.20, 0, 0, 0]),
    ('left_side_low',        'left_wrist_yaw_link',  [0.20,  0.35,  0.08, 0, 0, 0]),
    ('left_up_reach',        'left_wrist_yaw_link',  [0.22,  0.20,  0.32, 0, 0, 0]),
    # right arm (mirrored)
    ('right_elbow_flex',     'right_wrist_yaw_link', [0.20, -0.20,  0.25, 0, 0, 0]),
    ('right_forward_mid',    'right_wrist_yaw_link', [0.30, -0.20,  0.20, 0, 0, 0]),
    ('right_forward_extend', 'right_wrist_yaw_link', [0.35, -0.20,  0.10, 0, 0, 0]),
    ('right_forward_hi_mid', 'right_wrist_yaw_link', [0.28, -0.20,  0.25, 0, 0, 0]),
    ('right_inward',         'right_wrist_yaw_link', [0.30, -0.08,  0.10, 0, 0, 0]),
    ('right_side_up',        'right_wrist_yaw_link', [0.20, -0.35,  0.20, 0, 0, 0]),
    ('right_side_low',       'right_wrist_yaw_link', [0.20, -0.35,  0.08, 0, 0, 0]),
    ('right_up_reach',       'right_wrist_yaw_link', [0.22, -0.20,  0.32, 0, 0, 0]),
]


def _build_model():
    rm = RobotModel(_URDF, handless=True)
    rm.init_reduced_model(ENABLED_JOINTS)
    rm.init_collision_model(_URDF_SPHERE, _SRDF)
    return rm


def _home_ee(robot_model: RobotModel, frame: str) -> np.ndarray:
    '''FK from all-zero home config.'''
    q0 = robot_model.zero_q_body_reduced
    pin.forwardKinematics(robot_model.model_body_reduced,
                          robot_model.data_body_reduced, q0)
    pin.updateFramePlacements(robot_model.model_body_reduced,
                              robot_model.data_body_reduced)
    fid = robot_model.model_body_reduced.getFrameId(frame)
    pos = robot_model.data_body_reduced.oMf[fid].translation.copy()
    rot = robot_model.data_body_reduced.oMf[fid].rotation.copy()
    rpy = pin.rpy.matrixToRpy(rot)
    return np.concatenate([pos, rpy])


def _check(robot_model: RobotModel, solver: IKSolver,
           name: str, frame: str, target: list) -> dict:
    target_arr = np.array(target, dtype=float)

    # reset solver config to home
    solver.configuration_reduced.update(robot_model.zero_q_body_reduced)

    # set task
    task_name = 'feasibility_task'
    if task_name not in solver.frame_tasks:
        solver.add_frame_task(task_name, frame,
                              position_cost=20.0,
                              orientation_cost=10.0,
                              lm_damping=1.0)
    else:
        # update frame if needed
        if solver.frame_tasks[task_name].frame != frame:
            solver.remove_frame_task(task_name)
            solver.add_frame_task(task_name, frame,
                                  position_cost=20.0,
                                  orientation_cost=10.0,
                                  lm_damping=1.0)

    rotation = pin.rpy.rpyToMatrix(target_arr[3:])
    solver.frame_tasks[task_name].transform_target_to_world = pin.SE3(
        rotation, target_arr[:3]
    )

    try:
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            result = solver.solve_ik_reduced(
                alpha=_IK_ALPHA,
                timeout=_IK_TIMEOUT,
                linear_threshold=_LIN_THRESH,
                angular_threshold=_ANG_THRESH,
            )
    except Exception:
        return {
            'name': name, 'frame': frame,
            'target': target_arr[:3],
            'success': False,
            'lin_err_m': float('inf'), 'ang_err_rad': float('inf'),
            'home_ee': _home_ee(robot_model, frame)[:3],
            'ee_disp_m': float(np.linalg.norm(target_arr[:3] - _home_ee(robot_model, frame)[:3])),
            'max_joint_motion_rad': 0.0, 'top_joint': 'N/A', 'top_motion_rad': 0.0,
            'min_limit_margin_rad': 0.0,
            'dq': np.zeros(len(ENABLED_JOINTS)), 'joint_names': [],
        }

    q_sol = result['q']  # full 27-dof
    success = result['success']

    # residual error
    q_red = q_sol[robot_model.reduced_mask]
    cfg = solver.configuration_reduced
    cfg.update(q_red)
    err = solver.frame_tasks[task_name].compute_error(cfg)
    lin_err = float(np.linalg.norm(err[:3]))
    ang_err = float(np.linalg.norm(err[3:]))

    # joint motion from home (reduced joints only)
    q0_red = robot_model.zero_q_body_reduced
    dq = q_red - q0_red
    joint_names = [str(n) for n in robot_model.model_body_reduced.names[1:]]

    # joint limit margins
    ql = robot_model.model_body_reduced.lowerPositionLimit
    qu = robot_model.model_body_reduced.upperPositionLimit
    margins_low  = q_red - ql
    margins_high = qu - q_red
    min_margin = float(np.min(np.minimum(margins_low, margins_high)))

    # find the joint with largest motion
    top_joint_idx = int(np.argmax(np.abs(dq)))
    top_joint_name = joint_names[top_joint_idx] if top_joint_idx < len(joint_names) else '?'
    top_joint_motion = float(dq[top_joint_idx])

    # home EE position
    home_ee = _home_ee(robot_model, frame)
    displacement = float(np.linalg.norm(target_arr[:3] - home_ee[:3]))

    return {
        'name': name,
        'frame': frame,
        'target': target_arr[:3],
        'success': success,
        'lin_err_m': lin_err,
        'ang_err_rad': ang_err,
        'home_ee': home_ee[:3],
        'ee_disp_m': displacement,
        'max_joint_motion_rad': float(np.max(np.abs(dq))),
        'top_joint': top_joint_name,
        'top_motion_rad': top_joint_motion,
        'min_limit_margin_rad': min_margin,
        'dq': dq,
        'joint_names': joint_names,
    }


def _print_table(results: list[dict]):
    header = (
        f"{'Name':<26} {'Frame':<22} {'Target (x,y,z)':<22}"
        f"{'Disp':>6} {'OK':>3} {'LinErr':>7} {'MaxJt':>7} {'Margin':>7}"
        f"  TopJoint"
    )
    sep = '-' * len(header)
    print(sep)
    print(header)
    print(sep)
    for r in results:
        tgt = '[{:.2f},{:.2f},{:.2f}]'.format(*r['target'])
        ok  = 'YES' if r['success'] else 'NO '
        print(
            f"{r['name']:<26} {r['frame']:<22} {tgt:<22}"
            f"{r['ee_disp_m']:>6.3f} {ok:>3} {r['lin_err_m']:>7.4f}"
            f" {r['max_joint_motion_rad']:>7.3f} {r['min_limit_margin_rad']:>7.3f}"
            f"  {r['top_joint']} ({np.degrees(r['top_motion_rad']):.1f}°)"
        )
    print(sep)


def _print_joint_breakdown(result: dict):
    print(f"\n  Joint breakdown for '{result['name']}':")
    for name, dq in zip(result['joint_names'], result['dq']):
        if abs(dq) > 0.01:
            print(f"    {str(name):35s}: {np.degrees(dq):+7.2f}°  ({dq:+.4f} rad)")


def main():
    print(f'[feasibility] building model from {_URDF}')
    rm = _build_model()
    solver = IKSolver(rm, dt=0.02, d_min=0.02)

    # print home EE positions
    for frame in ('left_wrist_yaw_link', 'right_wrist_yaw_link'):
        ee = _home_ee(rm, frame)
        print(f'[feasibility] home EE {frame}: '
              f'x={ee[0]:.4f} y={ee[1]:.4f} z={ee[2]:.4f}')

    print(f'\n[feasibility] checking {len(CANDIDATES)} candidates...\n')

    results = []
    for name, frame, target in CANDIDATES:
        r = _check(rm, solver, name, frame, target)
        tag = 'OK ' if r['success'] else 'FAIL'
        print(f'  [{tag}] {name:<26}  lin={r["lin_err_m"]:.4f}m  '
              f'ang={r["ang_err_rad"]:.4f}rad  '
              f'disp={r["ee_disp_m"]:.3f}m  '
              f'max_jt={np.degrees(r["max_joint_motion_rad"]):.1f}°  '
              f'margin={r["min_limit_margin_rad"]:.3f}rad')
        results.append(r)

    print('\n\n=== SUMMARY TABLE ===\n')
    _print_table(results)

    # detailed joint breakdown for passing targets with significant motion
    print('\n=== JOINT BREAKDOWN (passing, displacement > 0.10 m) ===')
    for r in results:
        if r['success'] and r['ee_disp_m'] > 0.10:
            _print_joint_breakdown(r)

    # recommendation
    passing = [r for r in results if r['success'] and r['lin_err_m'] < _LIN_THRESH]
    significant = [r for r in passing if r['ee_disp_m'] > 0.10]
    print(f'\n[feasibility] {len(passing)}/{len(results)} targets converge; '
          f'{len(significant)} with displacement > 0.10 m')

    # separate by arm
    left_ok  = [r for r in significant if 'left'  in r['name']]
    right_ok = [r for r in significant if 'right' in r['name']]

    print(f'\nRecommended left-arm cases  ({len(left_ok)}):')
    for r in left_ok:
        print(f'  {r["name"]:<26}  disp={r["ee_disp_m"]:.3f}m  '
              f'max_jt={np.degrees(r["max_joint_motion_rad"]):.1f}°  '
              f'margin={r["min_limit_margin_rad"]:.3f}rad')

    print(f'\nRecommended right-arm cases ({len(right_ok)}):')
    for r in right_ok:
        print(f'  {r["name"]:<26}  disp={r["ee_disp_m"]:.3f}m  '
              f'max_jt={np.degrees(r["max_joint_motion_rad"]):.1f}°  '
              f'margin={r["min_limit_margin_rad"]:.3f}rad')

    return results


if __name__ == '__main__':
    main()
