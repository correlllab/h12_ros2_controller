'''Interactive hardware-free plan-goto using simulated state and optional Meshcat'''

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.append(str(Path(__file__).resolve().parents[2]))

from h12_ros2_controller.core.planner.kinematic_pipeline import (
    KinematicPipeline, candidate_metrics,
)
from h12_ros2_controller.utility.goto_commands import (
    FrameGoal, GotoConsole, LoadCase, MultiFrameGoal, NamedGoal, Quit, Replay, Reset,
    SaveCase, ShowCandidate,
)
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS


def main(visualize=False, replay_delay=0.03, config_name='sim_safety_split.yaml'):
    pipeline = KinematicPipeline(visualize=visualize, config_name=config_name)
    console = GotoConsole(
        title='Offline IK', named_goals=NAMED_CONFIGS,
        supported=(NamedGoal, FrameGoal, MultiFrameGoal, ShowCandidate, Reset, Replay,
                   SaveCase, LoadCase, Quit),
    )
    console.say('Offline simulation only: no DDS subscriber or hardware controller')
    console.help()
    last_path = None
    last_ik = None
    last_start = None
    last_case = None
    while True:
        try:
            command = console.read_command()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        except ValueError as err:
            console.say(f'Failed: {err}')
            continue
        if command is None:
            continue
        if isinstance(command, Quit):
            break
        try:
            if isinstance(command, SaveCase):
                if last_case is None:
                    raise ValueError('Plan a frame goal, then enter save <file>')
                Path(command.path).write_text(json.dumps(last_case, indent=2) + '\n')
                console.say(f'Saved case to {command.path}')
                continue
            if isinstance(command, ShowCandidate):
                if last_ik is None:
                    raise ValueError('Plan a frame goal, then enter show <candidate index>')
                pipeline.preview_candidate(last_ik.candidates[command.index])
                continue
            if isinstance(command, Reset):
                pipeline.robot_model.update_visualizer()
                continue
            if isinstance(command, Replay):
                if last_path is None:
                    console.say('No successful path to replay')
                    continue
                pipeline.replay(last_path, delay=replay_delay if visualize else 0)
                console.say('Simulated pose updated')
                continue
            last_path = None
            if isinstance(command, LoadCase):
                last_case = json.loads(Path(command.path).read_text())
                result = pipeline.plan_case(last_case)
                last_start = np.asarray(last_case['start_q'], dtype=float)
                last_ik = result.ik
            elif isinstance(command, (FrameGoal, MultiFrameGoal)):
                planner_cfg = pipeline.planner_config
                ik_timeout = planner_cfg.ik_total_timeout
                ik_alpha = planner_cfg.ik_alpha
                last_start = pipeline.robot_model.state['q'].copy()
                tasks = command.tasks if isinstance(command, MultiFrameGoal) else (command,)
                last_case = {
                    'config': config_name, 'start_q': last_start.tolist(),
                    'ik_timeout': ik_timeout, 'ik_alpha': ik_alpha,
                }
                if isinstance(command, MultiFrameGoal):
                    last_case['tasks'] = [
                        {'frame': task.frame_name, 'pose': task.pose.tolist()}
                        for task in tasks
                    ]
                else:
                    last_case.update(frame=command.frame_name, pose=command.pose.tolist())
                result = pipeline.plan_frames(
                    [(task.frame_name, task.pose) for task in tasks],
                    ik_timeout=ik_timeout, ik_alpha=ik_alpha,
                )
                last_ik = result.ik
            elif isinstance(command, NamedGoal):
                result = pipeline.plan_named(command.name)
                last_ik = None
                last_case = None
            else:
                raise ValueError('Unsupported goto command')
            metrics_fn = None
            if result.ik is not None:
                metrics_fn = lambda candidate: candidate_metrics(
                    candidate, last_start, pipeline.robot_model.model_body_reduced,
                    pipeline.robot_model.reduced_mask,
                )
            console.report(result, metrics_fn=metrics_fn)
            if result.plan is None:
                continue
            if result.plan.success:
                last_path = result.plan.path.copy()
                pipeline.show_path(last_path)
                if console.confirm('Replay simulated path now? [y/N] '):
                    pipeline.replay(last_path, delay=replay_delay if visualize else 0)
                    console.say('Simulated pose updated')
        except (ValueError, RuntimeError, IndexError, KeyError, OSError) as err:
            console.say(f'Failed: {err}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--visualize', action='store_true', help='Start Meshcat')
    parser.add_argument('--replay-delay', type=float, default=0.03)
    parser.add_argument('--config', default='sim_safety_split.yaml', help='Controller YAML profile')
    args = parser.parse_args()
    main(args.visualize, args.replay_delay, args.config)
