import time
import argparse

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.controller.gravity_comp_controller import GravityCompController
from h12_ros2_controller.utility.controller_config import load_controller_config, initialize_channel_factory

def main(config='default.yaml'):
    print('Initializing GravityCompController...')
    config_data = load_controller_config(config)
    initialize_channel_factory(config_data)
    gravity_comp_controller = GravityCompController('assets/h1_2/h1_2.urdf',
                                                    'assets/h1_2/h1_2_sphere.urdf',
                                                    'assets/h1_2/h1_2_sphere_collision.srdf',
                                                    dt=0.01,
                                                    visualize=False,
                                                    config=config)

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
    parser.add_argument('--config', type=str, default='default.yaml', help='YAML file name under config/')
    args = parser.parse_args()
    main(config=args.config)
