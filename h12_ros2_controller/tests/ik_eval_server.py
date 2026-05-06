'''
ROS2-server-based IK evaluation harness.

Delegates all robot control to a running frame_task_server.  Records data
exclusively via ROS2 topics — no Unitree SDK / ChannelFactory needed here.

Required nodes already running:
  ros2 run h12_ros2_controller frame_task_server
  ros2 run h12_ros2_controller joint_state_publisher   # for joint positions

Produces the same NPZ / summary.csv / meta.yaml layout as ik_eval.py so that
plot_ik_eval.py works without changes.  Joint velocity and torque fields are
filled with zeros (not available from ROS2 topics without a dedicated
publisher); position and EE data are fully recorded.

Usage:
  python -m h12_ros2_controller.tests.ik_eval_server --config sweep.yaml
  python -m h12_ros2_controller.tests.ik_eval_server --config sweep.yaml --yes
'''
import argparse
import csv
import sys
import time
from datetime import datetime
from itertools import product
from pathlib import Path

import numpy as np
import pinocchio as pin
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from custom_ros_messages.action import FrameTask, NamedConfig
from h12_ros2_controller.ros2.utility import list_to_pose
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS, ENABLED_JOINTS
from h12_ros2_controller.utility.joint_limits import (
    JOINT_POSITION_LIMITS,
    JOINT_TORQUE_LIMITS,
    JOINT_VELOCITY_LIMITS,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
TESTS_CONFIG_DIR = Path(__file__).resolve().parent / 'configs'

REDUCED_JOINT_NAMES = list(ENABLED_JOINTS)
REDUCED_BODY_IDS = np.array(
    [BODY_JOINTS.index(j) for j in ENABLED_JOINTS], dtype=int
)
N_BODY = len(BODY_JOINTS)


# ── config helpers ────────────────────────────────────────────────────────────

def _load_test_config(name: str) -> dict:
    for d in [TESTS_CONFIG_DIR, Path.cwd(), Path.cwd() / 'configs']:
        p = d / (name if name.endswith('.yaml') else name)
        if p.exists():
            cfg = yaml.safe_load(p.read_text()) or {}
            cfg['__path__'] = str(p)
            return cfg
    raise FileNotFoundError(f'config not found: {name}')


def _expand_sweeps(sweeps: dict) -> list[dict]:
    if not sweeps:
        return [{}]
    keys = list(sweeps.keys())
    return [dict(zip(keys, c)) for c in product(*[list(sweeps[k]) for k in keys])]


def _sweep_label(sweep: dict) -> str:
    return '_'.join(f'{k}={v}' for k, v in sweep.items()) if sweep else 'default'


def _pose_to_target(raw: list, units: str) -> np.ndarray:
    arr = np.asarray(raw, dtype=float)
    if units == 'deg':
        arr[3:] = np.radians(arr[3:])
    return arr


def _rpy_from_pose(pose_msg) -> np.ndarray:
    from pyquaternion import Quaternion
    o = pose_msg.orientation
    rot = Quaternion(o.w, o.x, o.y, o.z).rotation_matrix
    return pin.rpy.matrixToRpy(rot)


def _pose_error(current: np.ndarray, target: np.ndarray) -> tuple[float, float]:
    """Return (linear_error_m, angular_error_rad) between two 6D poses."""
    lin = float(np.linalg.norm(current[:3] - target[:3]))
    r_cur = pin.rpy.rpyToMatrix(current[3:])
    r_tgt = pin.rpy.rpyToMatrix(target[3:])
    r_err = r_tgt.T @ r_cur
    ang = float(np.linalg.norm(pin.log3(r_err)))
    return lin, ang


def _limits_snapshot() -> dict:
    return {
        'urdf_q_low':     np.array([JOINT_POSITION_LIMITS[i]['low']  for i in range(N_BODY)]),
        'urdf_q_high':    np.array([JOINT_POSITION_LIMITS[i]['high'] for i in range(N_BODY)]),
        'urdf_tau_limit': np.asarray(JOINT_TORQUE_LIMITS,   dtype=float),
        'urdf_dq_limit':  np.asarray(JOINT_VELOCITY_LIMITS, dtype=float),
        'clip_q_low':     np.array([JOINT_POSITION_LIMITS[i]['low']  for i in range(N_BODY)]),
        'clip_q_high':    np.array([JOINT_POSITION_LIMITS[i]['high'] for i in range(N_BODY)]),
        'clip_tau_limit': np.asarray(JOINT_TORQUE_LIMITS,   dtype=float),
        'clip_dq_limit':  np.asarray(JOINT_VELOCITY_LIMITS, dtype=float),
    }


# ── node ──────────────────────────────────────────────────────────────────────

class IKEvalNode(Node):
    '''
    Pure ROS2 node.  No background threads — all spinning done by the caller
    via spin_until_future_complete / spin_once, exactly like frame_task_client.
    '''

    def __init__(self):
        super().__init__('ik_eval_server')

        # action clients
        self._ft_client = ActionClient(self, FrameTask,   'frame_task')
        self._nc_client = ActionClient(self, NamedConfig, 'named_config')

        # latest data from subscriptions (updated by callbacks)
        self._q   = np.zeros(N_BODY)   # joint positions (27)
        self._left_ee  = np.zeros(6)   # x y z r p yaw
        self._right_ee = np.zeros(6)

        self.create_subscription(JointState,  'joint_states',  self._js_cb,       10)
        self.create_subscription(PoseStamped, 'left_ee_pose',  self._left_ee_cb,  10)
        self.create_subscription(PoseStamped, 'right_ee_pose', self._right_ee_cb, 10)

        # recording state (written during feedback callbacks + settle loop)
        self._recording = False
        self._rec_t:        list = []
        self._rec_q:        list = []
        self._rec_ee:       list = []
        self._rec_frame_err: list = []
        self._rec_t0:       float = 0.0
        self._active_ee:    str  = 'left'   # 'left' or 'right'
        self._active_target: np.ndarray | None = None
        self._feedback_seen: bool = False
        self._active_goal_handle = None

    # ── subscription callbacks ────────────────────────────────────────────────

    def _js_cb(self, msg: JointState):
        positions = np.asarray(msg.position, dtype=float)
        # JointState may include all joints in BODY_JOINTS order
        if len(positions) >= N_BODY:
            self._q = positions[:N_BODY].copy()

    def _left_ee_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self._left_ee = np.array([p.x, p.y, p.z, *_rpy_from_pose(msg.pose)])

    def _right_ee_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self._right_ee = np.array([p.x, p.y, p.z, *_rpy_from_pose(msg.pose)])

    def _snapshot(self, lin_err: float | None = None, ang_err: float | None = None):
        '''Record one timestep; call from feedback cb or settle loop.'''
        if not self._recording:
            return
        ee = self._left_ee.copy() if self._active_ee == 'left' else self._right_ee.copy()
        if lin_err is None or ang_err is None:
            if self._active_target is not None:
                lin_err, ang_err = _pose_error(ee, self._active_target)
            else:
                lin_err, ang_err = 0.0, 0.0
        self._rec_t.append(time.monotonic() - self._rec_t0)
        self._rec_q.append(self._q.copy())
        self._rec_ee.append(ee)
        self._rec_frame_err.append([lin_err, 0., 0., ang_err, 0., 0.])

    def _start_recording(self, active_ee: str, target_pose: np.ndarray | None = None):
        self._active_ee = active_ee
        self._rec_t.clear(); self._rec_q.clear()
        self._rec_ee.clear(); self._rec_frame_err.clear()
        self._rec_t0 = time.monotonic()
        self._recording = True
        self._active_target = target_pose.copy() if target_pose is not None else None
        self._feedback_seen = False

    def _stop_recording(self):
        self._recording = False
        n = len(self._rec_t)
        if n == 0:
            return None
        t   = np.asarray(self._rec_t).reshape(-1, 1)
        q   = np.asarray(self._rec_q)          # (N, 27)
        ee  = np.asarray(self._rec_ee)          # (N, 6)
        err = np.asarray(self._rec_frame_err)   # (N, 6)
        return dict(t=t, q_actual=q, ee_actual=ee, frame_err=err)

    # ── blocking action helpers (same pattern as frame_task_client.py) ────────

    def send_named_config(self, config_name: str, timeout: float = 12.0) -> bool:
        if not self._nc_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('named_config server not available')
            return False
        goal = NamedConfig.Goal()
        goal.config_name = config_name

        send_future = self._nc_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout)
        gh = send_future.result()
        if not gh or not gh.accepted:
            self.get_logger().warn(f'named_config "{config_name}" rejected')
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        res = result_future.result()
        return bool(res.result.success) if res else False

    def send_frame_task(self, frame_name: str, target_pose: np.ndarray,
                        timeout: float = 12.0) -> tuple[bool, int, float, float]:
        '''
        Returns (success, convergence_step, final_lin_err, final_ang_err).
        Records data via snapshot() on every feedback message.
        '''
        if not self._ft_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('frame_task server not available')
            return False, -1, 1.0, 1.0

        goal = FrameTask.Goal()
        goal.frame_names   = [frame_name]
        goal.frame_targets = [list_to_pose(list(target_pose))]
        goal.duration.sec = int(timeout)
        goal.duration.nanosec = int((timeout - int(timeout)) * 1e9)

        lin_thresh = 5e-3
        ang_thresh = 2e-2
        convergence_step = -1
        step_counter = [0]
        last_lin = [1.0]
        last_ang = [1.0]

        def _feedback_cb(fb_msg):
            lin_errs = list(fb_msg.feedback.errors_linear)
            ang_errs = list(fb_msg.feedback.errors_angular)
            lin = float(lin_errs[0]) if lin_errs else 0.0
            ang = float(ang_errs[0]) if ang_errs else 0.0
            last_lin[0] = lin
            last_ang[0] = ang
            step_counter[0] += 1
            nonlocal convergence_step
            if convergence_step < 0 and lin < lin_thresh and ang < ang_thresh:
                convergence_step = step_counter[0]
            self._feedback_seen = True
            self._snapshot(lin, ang)

        send_future = self._ft_client.send_goal_async(
            goal, feedback_callback=_feedback_cb
        )
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout)
        gh = send_future.result()
        if not gh or not gh.accepted:
            self.get_logger().warn('frame_task goal rejected')
            return False, -1, last_lin[0], last_ang[0]

        self._active_goal_handle = gh

        result_future = gh.get_result_async()
        result_timeout = timeout + 2.0
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=result_timeout)
        if not result_future.done():
            self.get_logger().warn(
                f'frame_task result timeout after {result_timeout:.1f}s'
            )
            success = False
        else:
            res = result_future.result()
            success = bool(res.result.success) if res else False
        if not self._feedback_seen and self._active_target is not None:
            ee = self._left_ee if 'left' in frame_name else self._right_ee
            last_lin[0], last_ang[0] = _pose_error(ee, self._active_target)
        self._active_goal_handle = None
        return success, convergence_step, last_lin[0], last_ang[0]

    def cancel_active_goal(self):
        if self._active_goal_handle is None:
            return
        self._active_goal_handle.cancel_goal_async()
        self._active_goal_handle = None


# ── orchestration ─────────────────────────────────────────────────────────────

def run(args):
    test_cfg = _load_test_config(args.config)
    label    = args.label or Path(test_cfg['__path__']).stem
    ts       = datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir  = REPO_ROOT / 'runs' / f'{ts}_{label}_real'
    trials_dir = run_dir / 'trials'
    trials_dir.mkdir(parents=True, exist_ok=True)
    print(f'[run] output dir: {run_dir}', flush=True)

    if not args.yes:
        ans = input(
            '[server] Commands will be sent to the running frame_task_server. '
            'Continue? [y/N]: '
        ).strip().lower()
        if ans != 'y':
            print('[aborted]'); return 1

    rclpy.init(args=None)
    node = IKEvalNode()

    limits_cfg        = _limits_snapshot()
    orientation_units = str(test_cfg.get('orientation_units', 'rad'))
    trial_cfg         = test_cfg.get('trial', {})
    sweeps            = _expand_sweeps(test_cfg.get('sweeps', {}))
    trials_per_case   = int(test_cfg.get('trials_per_case', 3))
    settle_steps      = int(trial_cfg.get('settle_steps', 50))
    ctrl_hz           = 50.0
    dt                = 1.0 / ctrl_hz
    home_timeout      = float(test_cfg.get('home_settle_timeout', 12.0))

    meta = {
        'label': label, 'mode': 'real', 'timestamp': ts,
        'test_config': test_cfg,
        'controller_config_name': test_cfg.get('controller_config', 'safety_split.yaml'),
    }
    (run_dir / 'meta.yaml').write_text(yaml.safe_dump(meta, sort_keys=False))

    summary_fh = (run_dir / 'summary.csv').open('w', newline='', encoding='utf-8')
    writer = None

    try:
        for case_cfg in test_cfg.get('cases', []):
            frame      = case_cfg['frame']
            target     = _pose_to_target(case_cfg['target_pose'], orientation_units)
            case_name  = case_cfg['name']
            active_ee  = 'left' if 'left' in frame else 'right'

            for sweep in sweeps:
                sweep_tag = _sweep_label(sweep)

                for trial_index in range(1, trials_per_case + 1):
                    print(f'[home] {case_name} sweep={sweep_tag} trial={trial_index}/{trials_per_case}',
                          flush=True)

                    # ── go home ───────────────────────────────────────────────
                    ok = node.send_named_config('home', timeout=home_timeout)
                    if not ok:
                        print('  [home] timeout / failed', flush=True)

                    # ── run trial ─────────────────────────────────────────────
                    node._start_recording(active_ee, target_pose=target)
                    t_wall_start = time.monotonic()

                    success, conv_step, lin_ss, ang_ss = node.send_frame_task(
                        frame, target,
                        timeout=float(trial_cfg.get('step_timeout', 6.0)) + 2.0,
                    )

                    # settle: spin_once at ctrl_hz for settle_steps more samples
                    for _ in range(settle_steps):
                        frame_start = time.monotonic()
                        rclpy.spin_once(node, timeout_sec=dt)
                        node._snapshot()
                        elapsed = time.monotonic() - frame_start
                        if elapsed < dt:
                            time.sleep(dt - elapsed)

                    wall_time = time.monotonic() - t_wall_start
                    arrs = node._stop_recording()

                    if arrs is None or arrs['t'].shape[0] < 2:
                        print('  [warn] too few samples, skipping', flush=True)
                        continue

                    converged = bool(success) and conv_step >= 0
                    n = arrs['t'].shape[0]

                    # EE steady-state: mean of last 10%
                    tail  = max(1, n // 10)
                    ee_ss = arrs['ee_actual'][-tail:].mean(axis=0)

                    # Override reported steady-state errors from recorded data.
                    lin_ss = float(arrs['frame_err'][-tail:, 0].mean())
                    ang_ss = float(arrs['frame_err'][-tail:, 3].mean())

                    # q_cmd not available externally — use q_actual as placeholder
                    q_actual = arrs['q_actual']
                    zeros_n  = np.zeros_like(q_actual)

                    # ── save NPZ ──────────────────────────────────────────────
                    trial_stem = f'{case_name}__{sweep_tag}__t{trial_index}'
                    np.savez(
                        trials_dir / f'{trial_stem}.npz',
                        # identity fields
                        case=case_name, frame=frame,
                        sweep_keys=np.array(list(sweep.keys())),
                        sweep_values=np.array(list(sweep.values())),
                        trial_index=trial_index,
                        mode='real',
                        dt=dt,
                        target_pose=target,
                        q_ik_optimal=np.full(N_BODY, np.nan),
                        ik_optimal_success=False,
                        converged=converged,
                        convergence_step=conv_step,
                        wall_time_s=wall_time,
                        reduced_body_ids=REDUCED_BODY_IDS,
                        reduced_joint_names=np.array(REDUCED_JOINT_NAMES),
                        # limits
                        **limits_cfg,
                        # timeseries
                        t=arrs['t'],
                        frame_err=arrs['frame_err'],
                        ee_actual=arrs['ee_actual'],
                        q_actual=q_actual,
                        q_cmd=q_actual,        # placeholder
                        dq_actual=zeros_n,     # not published
                        tau_actual=zeros_n,    # not published
                        tau_cmd=zeros_n,
                        twist=np.zeros((n, 6)),
                    )

                    # ── summary row ───────────────────────────────────────────
                    sweep_cols = {f'sweep__{k}': v for k, v in sweep.items()}
                    row = {
                        'case':               case_name,
                        'frame':              frame,
                        'trial_index':        trial_index,
                        'mode':               'real',
                        'converged':          int(converged),
                        'convergence_step':   conv_step,
                        'wall_time_s':        f'{wall_time:.3f}',
                        'lin_err_steady_m':   f'{abs(lin_ss):.5f}',
                        'ang_err_steady_rad': f'{abs(ang_ss):.5f}',
                        'ik_optimal_success': 0,
                        **sweep_cols,
                        'ee_ss__x':  f'{ee_ss[0]:.5f}',
                        'ee_ss__y':  f'{ee_ss[1]:.5f}',
                        'ee_ss__z':  f'{ee_ss[2]:.5f}',
                        'ee_ss__r':  f'{ee_ss[3]:.5f}',
                        'ee_ss__p':  f'{ee_ss[4]:.5f}',
                        'ee_ss__yw': f'{ee_ss[5]:.5f}',
                    }
                    if writer is None:
                        writer = csv.DictWriter(summary_fh, fieldnames=list(row.keys()))
                        writer.writeheader()
                    writer.writerow(row)
                    summary_fh.flush()

                    tag = 'OK ' if converged else 'TO '
                    print(f'  [{tag}] step={conv_step}  '
                          f'lin={abs(lin_ss):.4f}m  ang={abs(ang_ss):.4f}rad  '
                          f't={wall_time:.2f}s  n={n}', flush=True)

    except KeyboardInterrupt:
        print('\n[interrupted]', flush=True)
    finally:
        node.cancel_active_goal()
        summary_fh.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f'[done] {run_dir}', flush=True)

    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', default='sweep.yaml')
    p.add_argument('--label',  default='')
    p.add_argument('--yes', '-y', action='store_true')
    args = p.parse_args()
    sys.exit(run(args) or 0)


if __name__ == '__main__':
    main()
