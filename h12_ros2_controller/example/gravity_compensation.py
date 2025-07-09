import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.arm_controller import ArmController

def main():
    print('Initializing ArmController...')
    arm_controller = ArmController('assets/h1_2/h1_2.urdf',
                                   dt=0.01,
                                   vlim=1.0,
                                   visualize=False)
    # set gain for damp mode
    arm_controller.damp_mode(6.0)

    while True:
        start_time = time.time()
        arm_controller.gravity_compensation_step()
        time.sleep(max(0, arm_controller.dt - (time.time() - start_time)))

if __name__ == '__main__':
    main()
