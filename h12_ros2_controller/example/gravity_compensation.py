import time
import argparse

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.gravity_comp_controller import GravityCompController

def main(use_sport_mode=False):
    print('Initializing GravityCompController...')
    ChannelFactoryInitialize()
    gravity_comp_controller = GravityCompController('assets/h1_2/h1_2.urdf',
                                                    'assets/h1_2/h1_2_sphere.urdf',
                                                    'assets/h1_2/h1_2_sphere_collision.srdf',
                                                    dt=0.01,
                                                    visualize=False,
                                                    use_sport_mode=use_sport_mode)
    # set gain for damp mode
    gravity_comp_controller.damp_mode(6.0)

    try:
        while True:
            start_time = time.time()
            gravity_comp_controller.gravity_compensation_step()
            time.sleep(max(0.0, gravity_comp_controller.dt - (time.time() - start_time)))
    except Exception as e:
        print(f'Exception occurred: {e}')
    finally:
        print('Shutting down...')
        gravity_comp_controller.shutdown()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Arm Controller Goto')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--debug', action='store_true', help='Run in debug mode (use_sport_mode=False)')
    group.add_argument('--sport', action='store_true', help='Run in sport mode (use_sport_mode=True)')
    args = parser.parse_args()

    if args.sport:
        main(use_sport_mode=True)
    elif args.debug:
        main(use_sport_mode=False)
    else:
        print('Invalid argument! Use --debug or --sport')
