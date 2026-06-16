import time
import argparse

from h12_ros2_controller.utility.dds_init import init_channel_factory_from_env
from h12_ros2_controller.utility.path_definition import URDF_HANDLESS_PATH

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.plot.plot_robot_record import create_com_zmp_plot, update_com_zmp_plot

def main(show_plot):
    init_channel_factory_from_env()
    # robot_model = RobotModel('./assets/h1_2/h1_2_sphere.urdf')
    # robot_model = RobotModel('./assets/h1_2/h1_2.urdf')
    # robot_model = RobotModel('./assets/h1_2/h1_2.xml')
    robot_model = RobotModel(URDF_HANDLESS_PATH, handless=True)
    robot_model.init_visualizer()
    robot_model.config_visualizer(show_sensors=True, show_com=True, show_zmp=True)
    robot_model.init_subscriber()

    # create plot if requested
    plot_elements = None
    if show_plot:
        plot_elements = create_com_zmp_plot()

    # main loop shadowing robot states
    frame_idx = 0
    while True:
        robot_model.update_kinematics()
        robot_model.update_visualizer()

        # update plot if enabled
        if plot_elements is not None:
            com_pos = robot_model.get_com()
            zmp_pos = robot_model.get_zmp()
            left_foot_pos = robot_model.get_frame_position('left_ankle_roll_link')
            right_foot_pos = robot_model.get_frame_position('right_ankle_roll_link')
            update_com_zmp_plot(plot_elements, frame_idx, com_pos, zmp_pos, left_foot_pos, right_foot_pos)
            frame_idx += 1

        time.sleep(0.01)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='View robot state in real-time')
    parser.add_argument('--plot', action='store_true', help='Show ZMP plot')
    args = parser.parse_args()

    main(args.plot)
