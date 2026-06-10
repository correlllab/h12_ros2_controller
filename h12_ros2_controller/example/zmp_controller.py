import os
import sys
import time
import argparse

sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))

from h12_ros2_controller.core.controller.zmp_controller import ZmpController
from h12_ros2_controller.utility.controller_config import (
    load_controller_config,
    initialize_channel_factory,
    maybe_start_controller_logging,
)


def print_zmp_status(controller):
    print(f'zmp: {controller.latest_zmp}')
    print(f'zmp target: {controller.latest_zmp_target}')
    print(f'zmp error: {controller.latest_zmp_error}')
    print(f'momentum target: {controller.latest_momentum_target}')
    plan = controller.latest_plan
    if plan is None:
        print('no momentum plan generated')
        return
    print(f'solved: {plan.solved}')
    print(f'cost: {plan.solver.cost:.6f}')
    print(f'iterations: {plan.solver.iter}')


def main(config_name='balance_debug.yaml', status_interval=1.0):
    config = load_controller_config(config_name)
    config.setdefault('momentum_ddp', {})['enabled'] = True
    config.setdefault('zmp', {})['enabled'] = True

    initialize_channel_factory(config)
    controller = ZmpController(
        'assets/h1_2/h1_2_handless.urdf',
        'assets/h1_2/h1_2_handless_sphere.urdf',
        'assets/h1_2/h1_2_handless_sphere_collision.srdf',
        handless=True,
        visualize=False,
        config=config,
    )
    maybe_start_controller_logging(controller)

    print('ZMP controller ready')
    print('running continuously; press ctrl-c to stop')

    try:
        next_status_time = 0.0
        while True:
            start_time = time.time()
            previous_plan = controller.latest_plan
            controller.execute_zmp_step()
            if controller.latest_plan is not previous_plan:
                print_zmp_status(controller)
                print()
                next_status_time = start_time + status_interval
            elif start_time >= next_status_time:
                print_zmp_status(controller)
                print()
                next_status_time = start_time + status_interval
            time.sleep(max(
                0.0,
                controller.dt - (time.time() - start_time),
            ))
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
        description='Interactive ZMP controller demo'
    )
    parser.add_argument('--config',
                        type=str,
                        default='balance_debug.yaml',
                        help='YAML file name under config/')
    parser.add_argument('--status-interval',
                        type=float,
                        default=1.0,
                        help='Seconds between status prints')
    args = parser.parse_args()

    main(config_name=args.config, status_interval=args.status_interval)
