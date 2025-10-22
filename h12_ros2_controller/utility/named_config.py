import numpy as np

from h12_ros2_controller.utility.joint_definition import ENABLED_JOINTS

NAMED_CONFIGS = {
    'home': np.zeros(len(ENABLED_JOINTS))
}
