import os
import sys
import time
import argparse

REPO_ROOT = os.path.abspath(
    os.path.join(__file__, '../../..')
)
sys.path.append(REPO_ROOT)

from h12_ros2_controller.core.controller.zmp import (  # noqa: E402
    format_vector,
)
from h12_ros2_controller.core.controller.zmp_controller import (  # noqa: E402
    ZmpController,
)
from h12_ros2_controller.utility.controller_config import (  # noqa: E402
    load_controller_config,
    initialize_channel_factory,
    maybe_start_controller_logging,
)


def format_array(value):
    if value is None:
        return 'None'
    return format_vector(value)


def print_status(controller, last_summary):
    perturbation = controller.latest_perturbation_state

    active = False if perturbation is None else perturbation.active
    reasons = [] if perturbation is None else perturbation.reasons
    reason_text = ','.join(reasons) if reasons else '-'
    target = format_array(controller.latest_target_momentum)
    print(
        f'active={active} reasons={reason_text} target={target} '
        f'actuator={controller.latest_actuator_state} '
        f'plan_idx={controller.latest_actuator_plan_index} '
        f'response={controller.latest_response_status} '
        f'raw_norm={controller.latest_raw_command_norm:.3f} '
        f'applied_norm={controller.latest_applied_command_norm:.3f} '
        f'plan_dt={controller.latest_plan_duration:.3f}s'
    )

    summary = controller.latest_response_summary
    if summary and summary != last_summary:
        print(f'  {summary}')
        return summary
    return last_summary


def main(config_name='balance_safety_split.yaml', status_interval=1.0):
    config = load_controller_config(config_name)
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

    print('ZMP balance controller ready')
    print(f'config: {config_name}')
    print('press Ctrl-C to stop')

    next_status_time = 0.0
    last_summary = ''
    try:
        while True:
            start_time = time.time()
            controller.control_step_reduced()

            if start_time >= next_status_time:
                last_summary = print_status(controller, last_summary)
                next_status_time = start_time + status_interval

            time.sleep(max(0.0, controller.dt - (time.time() - start_time)))
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
        description='ZMP balance controller runner'
    )
    parser.add_argument('--config',
                        type=str,
                        default='balance_safety_split.yaml',
                        help='YAML file name under config/')
    parser.add_argument('--status-interval',
                        type=float,
                        default=1.0,
                        help='Seconds between status prints')
    args = parser.parse_args()

    main(
        config_name=args.config,
        status_interval=args.status_interval,
    )
