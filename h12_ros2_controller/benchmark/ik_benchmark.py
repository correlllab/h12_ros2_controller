'''Finite headless benchmark of the offline IK-to-planner route'''

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.append(str(Path(__file__).resolve().parents[2]))

from h12_ros2_controller.core.planner.kinematic_pipeline import (
    KinematicPipeline, candidate_metrics,
)


def main(samples=3, ik_timeout=1.0, seed=0, visualize=False,
         config_name='debug.yaml', start_name='arms_front_45', cases_path=None):
    pipeline = KinematicPipeline(visualize=visualize, config_name=config_name)
    cases = None
    if cases_path is not None:
        cases = json.loads(Path(cases_path).read_text())
        if isinstance(cases, dict):
            cases = [cases]
        if not isinstance(cases, list) or not cases:
            raise ValueError('cases must contain a saved case or nonempty list')
        samples = len(cases)
    else:
        staging = pipeline.plan_named(start_name)
        if not staging.plan.success:
            raise RuntimeError(f'Cannot reach benchmark starting pose: {staging.plan.reason}')
        pipeline.replay(staging.plan.path)
    rng = np.random.default_rng(seed)
    successes = 0
    elapsed = []
    wrist_travel = []
    wrist_margin = []
    for index in range(samples):
        if cases is not None:
            case = cases[index]
            start = np.asarray(case['start_q'], dtype=float)
            result = pipeline.plan_case(case)
        else:
            # perturb the current grasp pose to keep random goals reachable
            transform = pipeline.robot_model.get_frame_transformation_reduced(
                'left_grasp_frame', pipeline.robot_model.state_reduced['q'],
            )
            pose = np.r_[transform[:3, 3] + rng.uniform(-0.025, 0.025, 3),
                         pin.rpy.matrixToRpy(transform[:3, :3])]
            start = pipeline.robot_model.state['q'].copy()
            result = pipeline.plan_frame(
                'left_grasp_frame', pose, ik_timeout=ik_timeout,
            )
        elapsed.append(result.elapsed_time)
        planned = result.plan is not None and result.plan.success
        print(f'{index + 1}: IK={result.ik.success}, plan={planned}, '
              f'elapsed={result.elapsed_time:.4f}s, '
              f'reason={result.plan.reason if result.plan is not None else "IK failed"}')
        if planned:
            successes += 1
            metrics = candidate_metrics(
                result.selected_candidate, start,
                pipeline.robot_model.model_body_reduced,
                pipeline.robot_model.reduced_mask,
            )
            wrist_travel.append(metrics['wrist_travel'])
            wrist_margin.append(metrics['wrist_margin'])
            print(f'   score={metrics["score"]:.3f}, '
                  f'wrist travel={metrics["wrist_travel"]:.3f} rad, '
                  f'wrist margin={metrics["wrist_margin"]:.3f} rad, '
                  f'planner stages={result.plan.metadata.get("stage_timings", {})}')
            pipeline.show_path(result.plan.path)
            pipeline.replay(result.plan.path)
    print(f'Completed {samples} attempts, {successes} planned')
    if elapsed:
        print(f'End-to-end p50={np.percentile(elapsed, 50):.3f}s, '
              f'p95={np.percentile(elapsed, 95):.3f}s, '
              f'max={max(elapsed):.3f}s (warm offline planning, excludes setup)')
    if wrist_travel:
        print(f'Successful poses: mean wrist travel={np.mean(wrist_travel):.3f} rad, '
              f'min wrist margin={min(wrist_margin):.3f} rad')
    return successes


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', type=int, default=3)
    parser.add_argument('--ik-timeout', type=float, default=1.0)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--visualize', action='store_true')
    parser.add_argument('--config', default='debug.yaml', help='Controller YAML profile')
    parser.add_argument('--start', default='arms_front_45',
                        help='Named starting configuration')
    parser.add_argument('--cases', help='Saved frame case JSON (single case or list)')
    args = parser.parse_args()
    main(args.samples, args.ik_timeout, args.seed, args.visualize, args.config,
         args.start, args.cases)
