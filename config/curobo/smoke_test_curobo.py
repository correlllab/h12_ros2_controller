#!/usr/bin/env python3
'''First-GPU-load smoke test for the cuRobo robot config.

Loads h1_2_handless_curobo.yml into cuRobo MotionGen and plans one joint-space
motion (home -> a small arm pose). Run this on a CUDA GPU host BEFORE wiring the
cuRobo backend into the controller: it isolates config-schema issues (the
cspace / lock_joints convention, sphere format, URDF resolution) from the rest
of the stack.

    python3 config/curobo/smoke_test_curobo.py \
        --urdf /abs/path/CL_Assets/ros_assets/h1_2_handless_sphere.urdf

Requires cuRobo + CUDA + torch. Exits 0 on success, 1 on planning failure,
2 if the environment can't run cuRobo. Some accessors below are guarded with
getattr fallbacks because minor cuRobo API names vary by version; adjust if a
newer/older cuRobo reports differently (same caveat as curobo_planner.py).
'''
import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_CFG = os.path.join(_HERE, 'h1_2_handless_curobo.yml')


def _active_joint_names(motion_gen, robot_cfg):
    # cuRobo's active (non-locked) joints, in the order it controls them
    kin = getattr(motion_gen, 'kinematics', None)
    for holder in (kin, motion_gen):
        names = getattr(holder, 'joint_names', None)
        if names:
            return list(names)
    return list(robot_cfg['cspace']['joint_names'])


def _scalar(value):
    # solve_time/status may be python floats or 0-d tensors
    try:
        return float(value.item())
    except Exception:
        try:
            return float(value)
        except Exception:
            return value


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--robot-cfg', default=_DEFAULT_CFG)
    ap.add_argument('--urdf', default=None,
                    help='absolute kinematics URDF path (overrides robot_cfg)')
    ap.add_argument('--timeout', type=float, default=5.0)
    ap.add_argument('--dt', type=float, default=1.0 / 30.0)
    args = ap.parse_args()

    try:
        import numpy as np
        import torch
        from curobo.types.base import TensorDeviceType
        from curobo.types.robot import JointState
        from curobo.util_file import load_yaml
        from curobo.geom.types import WorldConfig
        from curobo.wrap.reacher.motion_gen import (
            MotionGen, MotionGenConfig, MotionGenPlanConfig,
        )
    except Exception as exc:
        print(f'[smoke] cuRobo/torch import failed: {exc}')
        print('[smoke] run on a CUDA GPU host with cuRobo installed')
        return 2

    if not torch.cuda.is_available():
        print('[smoke] no CUDA device available')
        return 2

    robot_cfg = load_yaml(args.robot_cfg)
    robot_cfg = robot_cfg.get('robot_cfg', robot_cfg)
    if args.urdf:
        kin = robot_cfg.setdefault('kinematics', {})
        kin['urdf_path'] = args.urdf
        kin['asset_root_path'] = os.path.dirname(args.urdf)
    print(f'[smoke] robot_cfg={args.robot_cfg}')
    print(f"[smoke] urdf_path={robot_cfg['kinematics'].get('urdf_path')}")

    tensor_args = TensorDeviceType()
    print('[smoke] building MotionGen ...')
    motion_gen_config = MotionGenConfig.load_from_robot_config(
        robot_cfg, WorldConfig(), tensor_args, interpolation_dt=args.dt,
    )
    motion_gen = MotionGen(motion_gen_config)
    print('[smoke] warmup (compiles kernels; may take a while) ...')
    motion_gen.warmup()

    joint_names = _active_joint_names(motion_gen, robot_cfg)
    dof = len(joint_names)
    print(f'[smoke] active DOF = {dof}')
    print(f'[smoke] active joints = {joint_names}')
    if dof != 14:
        print(f'[smoke] WARNING: expected 14 arm DOF, got {dof} -- check the '
              'cspace / lock_joints convention in the robot_cfg')

    start_q = np.zeros(dof, dtype=np.float32)   # home
    goal_q = start_q.copy()
    # nudge one joint to force a non-trivial plan
    if 'left_elbow_joint' in joint_names:
        goal_q[joint_names.index('left_elbow_joint')] = -0.6
    else:
        goal_q[0] = 0.3

    def js(q):
        return JointState.from_position(
            tensor_args.to_device(q.reshape(1, -1)), joint_names=joint_names,
        )

    print('[smoke] planning home -> goal ...')
    result = motion_gen.plan_single_js(
        js(start_q), js(goal_q),
        MotionGenPlanConfig(max_attempts=4, timeout=args.timeout),
    )

    ok = bool(_scalar(getattr(result, 'success')))
    solve_time = _scalar(getattr(result, 'solve_time', float('nan')))
    status = getattr(result, 'status', None)
    print(f'[smoke] success={ok} solve_time={solve_time} status={status}')

    if not ok:
        print('[smoke] FAIL: planner found no solution')
        return 1

    traj = result.get_interpolated_plan()
    n = traj.position.shape[0]
    print(f'[smoke] interpolated waypoints={n} (dt={args.dt:.4f}s)')
    print('[smoke] PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
