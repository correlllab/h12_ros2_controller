import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.robot_model import RobotModel

def main():
    ChannelFactoryInitialize()
    # robot_model = RobotModel('./assets/h1_2/h1_2_sphere.urdf')
    robot_model = RobotModel('./assets/h1_2/h1_2.urdf')
    # robot_model = RobotModel('./assets/h1_2/h1_2.xml')
    robot_model.init_visualizer()
    robot_model.config_visualizer(show_sensors=True, show_com=True, show_zmp=True)
    robot_model.init_subscriber()

    # main loop shadowing robot states
    while True:
        robot_model.update_kinematics()
        robot_model.update_visualizer()
        time.sleep(0.01)

if __name__ == '__main__':
    main()
