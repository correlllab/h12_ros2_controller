'''Shared interactive goto commands and planning diagnostics'''

import shlex
import sys
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class NamedGoal:
    name: str


@dataclass(frozen=True)
class FrameGoal:
    frame_name: str
    pose: np.ndarray


@dataclass(frozen=True)
class ShowCandidate:
    index: int


@dataclass(frozen=True)
class SaveCase:
    path: str


@dataclass(frozen=True)
class LoadCase:
    path: str


@dataclass(frozen=True)
class Replay:
    pass


@dataclass(frozen=True)
class Reset:
    pass


@dataclass(frozen=True)
class Quit:
    pass


def parse_goto_command(line, named_goals=None, supported=None):
    '''Parse a named or Cartesian target and optional console actions'''
    parts = shlex.split(line)
    if not parts:
        return None
    name = parts[0].lower()
    if name in ('quit', 'exit'):
        if len(parts) != 1:
            raise ValueError(f'Expected {name} without arguments')
        command = Quit()
    elif name in ('replay', 'reset'):
        if len(parts) != 1:
            raise ValueError(f'Expected {name} without arguments')
        command = Replay() if name == 'replay' else Reset()
    elif name == 'show':
        if len(parts) != 2:
            raise ValueError('Expected show <candidate index>')
        index = int(parts[1])
        if index < 0:
            raise ValueError('Candidate index must be nonnegative')
        command = ShowCandidate(index)
    elif name in ('save', 'load'):
        if len(parts) != 2:
            raise ValueError(f'Expected {name} <file>')
        command = SaveCase(parts[1]) if name == 'save' else LoadCase(parts[1])
    elif name == 'frame':
        if len(parts) != 8:
            raise ValueError('Expected frame <name> x y z roll pitch yaw (degrees)')
        pose = np.asarray([float(value) for value in parts[2:]], dtype=float)
        if not np.all(np.isfinite(pose)):
            raise ValueError('Frame pose must contain only finite values')
        pose[3:] = np.deg2rad(pose[3:])
        command = FrameGoal(parts[1], pose)
    elif len(parts) == 1:
        if named_goals is not None and parts[0] not in named_goals:
            raise ValueError(f'Unknown named configuration: {parts[0]}')
        command = NamedGoal(parts[0])
    else:
        raise ValueError('Expected a named goal or frame command')
    if supported is not None and not isinstance(command, tuple(supported)):
        raise ValueError(f'{name} is not supported by this entry point')
    return command


class GotoConsole:
    '''Reusable editable prompt, help, and planning-result presentation'''

    def __init__(self, title='Goto', named_goals=(), supported=None,
                 input_fn=None, output_fn=None):
        self.title = title
        self.named_goals = tuple(named_goals)
        self.supported = supported
        self._input = input_fn or input
        self._output = output_fn or print
        if sys.stdin.isatty():
            try:
                import readline  # noqa: F401 - enables arrow-key editing for input()
            except ImportError:
                pass

    def say(self, message):
        self._output(f'[{self.title}] {message}')

    def help(self):
        if self.supported is None:
            available = (NamedGoal, FrameGoal, ShowCandidate, Reset, Replay,
                         SaveCase, LoadCase, Quit)
        else:
            available = self.supported
        commands = []
        if NamedGoal in available:
            self.say('Named goals: ' + ', '.join(sorted(self.named_goals)))
            commands.append('<name>')
        if FrameGoal in available:
            commands.append('frame <frame_name> x y z roll pitch yaw (degrees)')
        self.say('Commands: ' + ' | '.join(commands))
        actions = []
        for kind, label in ((ShowCandidate, 'show <index>'), (Reset, 'reset'),
                            (Replay, 'replay'), (SaveCase, 'save <file>'),
                            (LoadCase, 'load <file>'), (Quit, 'quit')):
            if kind in available:
                actions.append(label)
        if actions:
            self.say('          ' + ' | '.join(actions))

    def read_command(self):
        line = self._input('goto> ')
        return parse_goto_command(line, self.named_goals, self.supported)

    def confirm(self, prompt):
        return self._input(prompt).strip().lower() == 'y'

    def report(self, result, metrics_fn=None):
        '''Display IK attempts, scored candidates, and path diagnostics'''
        if result.ik is not None:
            ik = result.ik
            seed_label = 'seed' if ik.success else 'best failed seed'
            self.say(f'IK: success={ik.success}, {seed_label}={ik.seed_name}, '
                     f'linear={ik.linear_error:.4g}, angular={ik.angular_error:.4g}')
            for attempt in ik.attempts:
                if not attempt.success:
                    self._output(
                        f'  {attempt.seed_name}: {attempt.reason or "not converged"}, '
                        f'linear={attempt.linear_error:.4g}, '
                        f'angular={attempt.angular_error:.4g}, '
                        f'elapsed={attempt.elapsed_time:.3f}s'
                    )
            for index, candidate in enumerate(ik.candidates):
                metrics = metrics_fn(candidate) if metrics_fn is not None else None
                score = candidate.score if metrics is None else metrics['score']
                detail = f'score={score:.3f}'
                if metrics is not None:
                    detail += (f', wrist travel={metrics["wrist_travel"]:.3f} rad, '
                               f'wrist margin={metrics["wrist_margin"]:.3f} rad')
                selected = ' selected' if candidate is result.selected_candidate else ''
                self._output(f'  [{index}] {candidate.seed_name}: {detail}{selected}')
        plan = result.plan
        if plan is None:
            self.say(f'IK failed after {result.elapsed_time:.3f}s; pose unchanged')
            return
        self.say(f'Plan: success={plan.success}, reason={plan.reason}, '
                 f'waypoints={len(plan.path)}, elapsed={result.elapsed_time:.3f}s')
        for message in plan.metadata.get('diagnostics', ()):
            self.say(message)
        metadata = plan.metadata
        if 'joint_path_ratio' in metadata:
            self.say(
                f'Joint path: {metadata["joint_path_length"]:.3f} rad, '
                f'detour ratio={metadata["joint_path_ratio"]:.2f}'
            )
            for name, x in metadata.get('min_frame_x', {}).items():
                self.say(f'{name}: minimum x={x:.3f}m '
                         f'(local bound={metadata["local_x_bounds"][name]:.3f}m)')
        if 'path_kind' in metadata:
            frame_lengths = ', '.join(
                f'{name}={length:.4f}m '
                f'(pos={metadata["frame_translation_lengths"][name]:.4f}, '
                f'rot={metadata["frame_rotation_lengths"][name]:.4f})'
                for name, length in metadata['frame_path_lengths'].items()
            )
            self.say(
                f'{metadata["path_kind"]}: waypoints={metadata["waypoint_count"]}, '
                f'duration={metadata["duration"]:.3f}s, '
                f'effective_speed={metadata["effective_speed"]:.4f}m/s, '
                f'path_length={metadata["path_length"]:.4f}m, '
                f'samples={metadata["length_estimate_samples"]}, {frame_lengths}'
            )
