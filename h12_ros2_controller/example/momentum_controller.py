import os
import sys
import time
import argparse

import numpy as np

sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))

from h12_ros2_controller.core.controller.momentum_controller import MomentumController
from h12_ros2_controller.utility.controller_config import (
    load_controller_config,
    initialize_channel_factory,
    maybe_start_controller_logging,
)
from h12_ros2_controller.utility.path_definition import (
    SRDF_HANDLESS_SPHERE_PATH,
    URDF_HANDLESS_PATH,
    URDF_HANDLESS_SPHERE_PATH,
)


def print_plan_status(plan):
    if plan is None:
        return

    print('Momentum trajectory generated')
    print(f'Solved: {plan.solved}')
    print(f'Cost: {plan.solver.cost:.6f}')
    print(f'Iterations: {plan.solver.iter}')
    print(f'Target: {plan.target_momentum}')
    print(f'Peak useful momentum: {plan.peak_momentum}')
    print(f'Final posture error: {plan.final_posture_error:.4f} rad')


def parse_momentum(text):
    parts = text.strip().split()
    if len(parts) != 3:
        raise ValueError('Enter exactly three numbers')
    return np.array(
        [float(parts[0]), float(parts[1]), float(parts[2])],
        dtype=np.float64,
    )


def main(config_name='balance_debug.yaml'):
    config = load_controller_config(config_name)
    config.setdefault('momentum_ddp', {})['enabled'] = True

    initialize_channel_factory(config)

    controller = MomentumController(
        URDF_HANDLESS_PATH,
        URDF_HANDLESS_SPHERE_PATH,
        SRDF_HANDLESS_SPHERE_PATH,
        handless=True,
        visualize=False,
        config=config,
    )
    maybe_start_controller_logging(controller)

    print('Momentum controller ready')
    print('enter hx hy hz to solve and execute one trajectory')
    print('enter q to quit')

    try:
        while True:
            text = input('target momentum: ').strip()
            if text.lower() in {'q', 'quit', 'exit'}:
                break
            if not text:
                continue

            try:
                target = parse_momentum(text)
            except ValueError as exc:
                print(f'invalid input: {exc}')
                continue

            # solve once, then stream the optimized velocity sequence
            start = time.time()
            plan = controller.plan_momentum(target)
            print(f'solve time: {time.time() - start:.3f}s')
            print_plan_status(plan)

            while not controller.plan_done:
                start_time = time.time()
                controller.execute_plan_step()
                time.sleep(max(
                    0.0,
                    controller.dt - (time.time() - start_time),
                ))
            print()
    except KeyboardInterrupt:
        print()
        print('interrupted')
    except Exception as exc:
        print(f'error: {exc}')
    finally:
        print('shutting down')
        controller.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Interactive momentum controller demo'
    )
    parser.add_argument('--config',
                        type=str,
                        default='balance_debug.yaml',
                        help='YAML file name under config/')
    args = parser.parse_args()

    main(config_name=args.config)
