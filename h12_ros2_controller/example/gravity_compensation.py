import time
import argparse

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.controller.gravity_comp_controller import GravityCompController
from h12_ros2_controller.utility.controller_config import load_controller_config, initialize_channel_factory
from h12_ros2_controller.utility.path_definition import (
    URDF_HANDLESS_PATH,
    URDF_HANDLESS_SPHERE_PATH,
    SRDF_HANDLESS_SPHERE_PATH,
)

def main(config_name='debug.yaml'):
    print('Initializing GravityCompController...')
    config= load_controller_config(config_name)
    initialize_channel_factory(config)
    gravity_comp_controller = GravityCompController(URDF_HANDLESS_PATH,
                                                    URDF_HANDLESS_SPHERE_PATH,
                                                    SRDF_HANDLESS_SPHERE_PATH,
                                                    handless=True,
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
    parser = argparse.ArgumentParser(description='Gravity Compensation Controller')
    parser.add_argument('--config', type=str, default='debug.yaml', help='YAML file name under config/')
    args = parser.parse_args()
    main(config_name=args.config)
