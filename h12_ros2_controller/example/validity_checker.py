import time
import numpy as np

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.utility.path_definition import (
    URDF_MAGPIE_PATH,
    URDF_MAGPIE_SPHERE_PATH,
    SRDF_MAGPIE_SPHERE_PATH,
)

def main():
    robot_model = RobotModel(URDF_MAGPIE_PATH, handless=False)
    robot_model.init_visualizer()
    robot_model.init_collision_model(URDF_MAGPIE_SPHERE_PATH,
                                     SRDF_MAGPIE_SPHERE_PATH)

    while True:
        # sample random q (motor-only, 27)
        q = np.random.uniform(low=robot_model.model_body.lowerPositionLimit,
                              high=robot_model.model_body.upperPositionLimit)
        robot_model._q = q
        robot_model.update_kinematics()
        robot_model.update_visualizer()

        t = time.perf_counter()
        valid = robot_model.check_valid(q)
        print(f'Validity checked in {time.perf_counter() - t:.4f} seconds')
        print(f'Configuration is {"valid" if valid else "invalid"}')
        input('Press Enter to sample another random configuration...')

if __name__ == '__main__':
    main()
