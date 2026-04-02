import numpy as np

from h12_ros2_controller.utility.joint_definition import BODY_JOINTS

# Joint limits use URDF values directly
# Functions for conservative scaling will be added later

JOINT_POSITION_LIMITS = [
    {'low': -0.43, 'high': 0.43},      # left_hip_yaw_joint
    {'low': -3.14, 'high': 2.5},       # left_hip_pitch_joint
    {'low': -0.43, 'high': 3.14},      # left_hip_roll_joint
    {'low': -0.12, 'high': 2.19},      # left_knee_joint
    {'low': -0.897334, 'high': 0.523598}, # left_ankle_pitch_joint
    {'low': -0.261799, 'high': 0.261799}, # left_ankle_roll_joint

    {'low': -0.43, 'high': 0.43},      # right_hip_yaw_joint
    {'low': -3.14, 'high': 2.5},       # right_hip_pitch_joint
    {'low': -3.14, 'high': 0.43},      # right_hip_roll_joint
    {'low': -0.12, 'high': 2.19},      # right_knee_joint
    {'low': -0.897334, 'high': 0.523598}, # right_ankle_pitch_joint
    {'low': -0.261799, 'high': 0.261799}, # right_ankle_roll_joint

    {'low': -2.35, 'high': 2.35},      # torso_joint

    {'low': -3.14, 'high': 1.57},      # left_shoulder_pitch_joint
    {'low': -0.38, 'high': 3.4},       # left_shoulder_roll_joint
    {'low': -2.66, 'high': 3.01},      # left_shoulder_yaw_joint
    {'low': -0.95, 'high': 3.18},      # left_elbow_joint
    {'low': -3.01, 'high': 2.75},      # left_wrist_roll_joint
    {'low': -0.4625, 'high': 0.4625},  # left_wrist_pitch_joint
    {'low': -1.27, 'high': 1.27},      # left_wrist_yaw_joint

    {'low': -3.14, 'high': 1.57},      # right_shoulder_pitch_joint
    {'low': -3.4, 'high': 0.38},       # right_shoulder_roll_joint
    {'low': -3.01, 'high': 2.66},      # right_shoulder_yaw_joint
    {'low': -0.95, 'high': 3.18},      # right_elbow_joint
    {'low': -2.75, 'high': 3.01},      # right_wrist_roll_joint
    {'low': -0.4625, 'high': 0.4625},  # right_wrist_pitch_joint
    {'low': -1.27, 'high': 1.27},      # right_wrist_yaw_joint
]

JOINT_VELOCITY_LIMITS = [
    23.0,  # left_hip_yaw_joint
    23.0,  # left_hip_pitch_joint
    23.0,  # left_hip_roll_joint
    14.0,  # left_knee_joint
    9.0,   # left_ankle_pitch_joint
    9.0,   # left_ankle_roll_joint

    23.0,  # right_hip_yaw_joint
    23.0,  # right_hip_pitch_joint
    23.0,  # right_hip_roll_joint
    14.0,  # right_knee_joint
    9.0,   # right_ankle_pitch_joint
    9.0,   # right_ankle_roll_joint

    23.0,  # torso_joint

    9.0,   # left_shoulder_pitch_joint
    9.0,   # left_shoulder_roll_joint
    20.0,  # left_shoulder_yaw_joint
    20.0,  # left_elbow_joint
    31.4,  # left_wrist_roll_joint
    31.4,  # left_wrist_pitch_joint
    31.4,  # left_wrist_yaw_joint

    9.0,   # right_shoulder_pitch_joint
    9.0,   # right_shoulder_roll_joint
    20.0,  # right_shoulder_yaw_joint
    20.0,  # right_elbow_joint
    31.4,  # right_wrist_roll_joint
    31.4,  # right_wrist_pitch_joint
    31.4,  # right_wrist_yaw_joint
]

JOINT_TORQUE_LIMITS = [
    200.0,  # left_hip_yaw_joint
    200.0,  # left_hip_pitch_joint
    200.0,  # left_hip_roll_joint
    300.0,  # left_knee_joint
    60.0,   # left_ankle_pitch_joint
    40.0,   # left_ankle_roll_joint

    200.0,  # right_hip_yaw_joint
    200.0,  # right_hip_pitch_joint
    200.0,  # right_hip_roll_joint
    300.0,  # right_knee_joint
    60.0,   # right_ankle_pitch_joint
    40.0,   # right_ankle_roll_joint

    200.0,  # torso_joint

    40.0,   # left_shoulder_pitch_joint
    40.0,   # left_shoulder_roll_joint
    18.0,   # left_shoulder_yaw_joint
    18.0,   # left_elbow_joint
    19.0,   # left_wrist_roll_joint
    19.0,   # left_wrist_pitch_joint
    19.0,   # left_wrist_yaw_joint

    40.0,   # right_shoulder_pitch_joint
    40.0,   # right_shoulder_roll_joint
    18.0,   # right_shoulder_yaw_joint
    18.0,   # right_elbow_joint
    19.0,   # right_wrist_roll_joint
    19.0,   # right_wrist_pitch_joint
    19.0,   # right_wrist_yaw_joint
]

JOINT_TORQUE_ESTOP_SCALERS = [
    0.90, # left_hip_yaw_joint
    0.90, # left_hip_pitch_joint
    1.00, # left_hip_roll_joint
    0.90, # left_knee_joint
    0.90, # left_ankle_pitch_joint
    0.90, # left_ankle_roll_joint

    0.90, # right_hip_yaw_joint
    0.90, # right_hip_pitch_joint
    1.00, # right_hip_roll_joint
    0.90, # right_knee_joint
    0.90, # right_ankle_pitch_joint
    0.90, # right_ankle_roll_joint

    0.50, # torso_joint

    0.50, # left_shoulder_pitch_joint
    0.50, # left_shoulder_roll_joint
    0.50, # left_shoulder_yaw_joint
    0.80, # left_elbow_joint
    0.40, # left_wrist_roll_joint
    0.40, # left_wrist_pitch_joint
    0.40, # left_wrist_yaw_joint

    0.50, # right_shoulder_pitch_joint
    0.40, # right_shoulder_roll_joint
    0.40, # right_shoulder_yaw_joint
    0.80, # right_elbow_joint
    0.40, # right_wrist_roll_joint
    0.40, # right_wrist_pitch_joint
    0.40, # right_wrist_yaw_joint
]

def get_default_gains():
    kp = np.full(len(BODY_JOINTS), 50.0, dtype=np.float32)
    kd = np.full(len(BODY_JOINTS), 5.0, dtype=np.float32)

    kp[BODY_JOINTS.index('left_wrist_roll_joint')] = 20.0
    kp[BODY_JOINTS.index('left_wrist_pitch_joint')] = 20.0
    kp[BODY_JOINTS.index('left_wrist_yaw_joint')] = 20.0
    kp[BODY_JOINTS.index('right_wrist_roll_joint')] = 20.0
    kp[BODY_JOINTS.index('right_wrist_pitch_joint')] = 20.0
    kp[BODY_JOINTS.index('right_wrist_yaw_joint')] = 20.0

    kd[BODY_JOINTS.index('left_hip_yaw_joint')] = 8.0
    kd[BODY_JOINTS.index('right_hip_yaw_joint')] = 8.0
    kd[BODY_JOINTS.index('left_hip_pitch_joint')] = 15.0
    kd[BODY_JOINTS.index('right_hip_pitch_joint')] = 15.0
    kd[BODY_JOINTS.index('left_hip_roll_joint')] = 15.0
    kd[BODY_JOINTS.index('right_hip_roll_joint')] = 15.0
    kd[BODY_JOINTS.index('left_knee_joint')] = 10.0
    kd[BODY_JOINTS.index('right_knee_joint')] = 10.0
    kd[BODY_JOINTS.index('left_ankle_pitch_joint')] = 5.0
    kd[BODY_JOINTS.index('right_ankle_pitch_joint')] = 5.0
    kd[BODY_JOINTS.index('left_ankle_roll_joint')] = 5.0
    kd[BODY_JOINTS.index('right_ankle_roll_joint')] = 5.0
    kd[BODY_JOINTS.index('torso_joint')] = 10.0
    kd[BODY_JOINTS.index('left_shoulder_pitch_joint')] = 10.0
    kd[BODY_JOINTS.index('right_shoulder_pitch_joint')] = 10.0
    kd[BODY_JOINTS.index('left_shoulder_roll_joint')] = 10.0
    kd[BODY_JOINTS.index('right_shoulder_roll_joint')] = 10.0
    kd[BODY_JOINTS.index('left_shoulder_yaw_joint')] = 4.0
    kd[BODY_JOINTS.index('right_shoulder_yaw_joint')] = 4.0
    kd[BODY_JOINTS.index('left_elbow_joint')] = 4.0
    kd[BODY_JOINTS.index('right_elbow_joint')] = 4.0
    kd[BODY_JOINTS.index('left_wrist_roll_joint')] = 4.0
    kd[BODY_JOINTS.index('right_wrist_roll_joint')] = 4.0
    kd[BODY_JOINTS.index('left_wrist_pitch_joint')] = 4.0
    kd[BODY_JOINTS.index('right_wrist_pitch_joint')] = 4.0
    kd[BODY_JOINTS.index('left_wrist_yaw_joint')] = 4.0
    kd[BODY_JOINTS.index('right_wrist_yaw_joint')] = 4.0

    return kp, kd


def setup_gains(command_publisher, gains_config=None):
    '''Configure motor gains for all joints - optimized for real robot'''
    if gains_config is not None and gains_config.get('kp') is not None and gains_config.get('kd') is not None:
        command_publisher.kp[:] = np.asarray(gains_config['kp'], dtype=np.float32)
        command_publisher.kd[:] = np.asarray(gains_config['kd'], dtype=np.float32)
        return

    default_kp, default_kd = get_default_gains()
    command_publisher.kp[:] = default_kp
    command_publisher.kd[:] = default_kd
