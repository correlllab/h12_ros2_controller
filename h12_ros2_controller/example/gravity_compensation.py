import time
import argparse

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.arm_controller import ArmController

def main(use_sport_mode=False):
    print('Initializing ArmController...')
    ChannelFactoryInitialize()
    arm_controller = ArmController('assets/h1_2/h1_2.urdf',
                                   'assets/h1_2/h1_2_sphere.urdf',
                                   'assets/h1_2/h1_2_sphere_collision.srdf',
                                   dt=0.01,
                                   visualize=False,
                                   use_sport_mode=use_sport_mode)
    # set gain for damp mode
    arm_controller.damp_mode(6.0)


    try:
        while True:
            start_time = time.time()
            arm_controller.gravity_compensation_step()
            time.sleep(max(0.0, arm_controller.dt - (time.time() - start_time)))
    except Exception as e:
        print(f'Exception occurred: {e}')
    finally:
        print('Shutting down...')
        arm_controller.shutdown()

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
