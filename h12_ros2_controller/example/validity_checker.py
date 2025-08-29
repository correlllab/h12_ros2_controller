import time
import numpy as np

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.robot_model import RobotModel

def main():
    robot_model = RobotModel('./assets/h1_2/h1_2.urdf')
    robot_model.init_visualizer()
    robot_model.init_collision_model('./assets/h1_2/h1_2_sphere.urdf',
                                     './assets/h1_2/h1_2_sphere_collision.srdf')

    while True:
        # sample random q
        q = np.random.uniform(low=robot_model.model.lowerPositionLimit,
                              high=robot_model.model.upperPositionLimit)
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
