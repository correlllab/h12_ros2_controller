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

JOINT_POSITION_CLIP_LIMITS = [
    {'low': limit['low'] + 0.02, 'high': limit['high'] - 0.02} for limit in JOINT_POSITION_LIMITS
]

JOINT_POSITION_ESTOP_LIMITS = [
    {'low': limit['low'] + 0.01, 'high': limit['high'] - 0.01} for limit in JOINT_POSITION_LIMITS
]

JOINT_VELOCITY_CLIP_LIMITS = [
    limit * 0.08 for limit in JOINT_VELOCITY_LIMITS
]

JOINT_VELOCITY_ESTOP_LIMITS = [
    limit * 0.5 for limit in JOINT_VELOCITY_LIMITS
]

JOINT_TORQUE_CLIP_LIMITS = [
    limit * 0.25 for limit in JOINT_TORQUE_LIMITS
]

JOINT_TORQUE_ESTOP_LIMITS = [
    limit * scaler for limit, scaler in zip(JOINT_TORQUE_LIMITS, JOINT_TORQUE_ESTOP_SCALERS)
]

def setup_gains(command_publisher):
    '''Configure motor gains for all joints - optimized for real robot'''
    # initialize all gains to default values
    command_publisher.kp.fill(50.0)
    command_publisher.kd.fill(5.0)

    # gain for hip yaw - real robot settings
    command_publisher.kp[BODY_JOINTS.index('left_hip_yaw_joint')] = 50.0
    command_publisher.kd[BODY_JOINTS.index('left_hip_yaw_joint')] = 8.0
    command_publisher.kp[BODY_JOINTS.index('right_hip_yaw_joint')] = 50.0
    command_publisher.kd[BODY_JOINTS.index('right_hip_yaw_joint')] = 8.0

    # gain for hip pitch
    command_publisher.kp[BODY_JOINTS.index('left_hip_pitch_joint')] = 50.0
    command_publisher.kd[BODY_JOINTS.index('left_hip_pitch_joint')] = 15.0
    command_publisher.kp[BODY_JOINTS.index('right_hip_pitch_joint')] = 50.0
    command_publisher.kd[BODY_JOINTS.index('right_hip_pitch_joint')] = 15.0
    # gain for hip roll
    command_publisher.kp[BODY_JOINTS.index('left_hip_roll_joint')] = 50.0
    command_publisher.kd[BODY_JOINTS.index('left_hip_roll_joint')] = 15.0
    command_publisher.kp[BODY_JOINTS.index('right_hip_roll_joint')] = 50.0
    command_publisher.kd[BODY_JOINTS.index('right_hip_roll_joint')] = 15.0
    # gain for knee
    command_publisher.kp[BODY_JOINTS.index('left_knee_joint')] = 50.0
    command_publisher.kd[BODY_JOINTS.index('left_knee_joint')] = 10.0
    command_publisher.kp[BODY_JOINTS.index('right_knee_joint')] = 50.0
    command_publisher.kd[BODY_JOINTS.index('right_knee_joint')] = 10.0
    # gain for ankle pitch
    command_publisher.kp[BODY_JOINTS.index('left_ankle_pitch_joint')] = 50.0
    command_publisher.kd[BODY_JOINTS.index('left_ankle_pitch_joint')] = 5.0
    command_publisher.kp[BODY_JOINTS.index('right_ankle_pitch_joint')] = 50.0
    command_publisher.kd[BODY_JOINTS.index('right_ankle_pitch_joint')] = 5.0
    # gain for ankle roll
    command_publisher.kp[BODY_JOINTS.index('left_ankle_roll_joint')] = 50.0
    command_publisher.kd[BODY_JOINTS.index('left_ankle_roll_joint')] = 5.0
    command_publisher.kp[BODY_JOINTS.index('right_ankle_roll_joint')] = 50.0
    command_publisher.kd[BODY_JOINTS.index('right_ankle_roll_joint')] = 5.0

    # gain for torso (12)
    command_publisher.kp[BODY_JOINTS.index('torso_joint')] = 50.0
    command_publisher.kd[BODY_JOINTS.index('torso_joint')] = 10.0

    # gain for shoulder pitch
    command_publisher.kp[BODY_JOINTS.index('left_shoulder_pitch_joint')] = 50.0
    command_publisher.kd[BODY_JOINTS.index('left_shoulder_pitch_joint')] = 10.0
    command_publisher.kp[BODY_JOINTS.index('right_shoulder_pitch_joint')] = 50.0
    command_publisher.kd[BODY_JOINTS.index('right_shoulder_pitch_joint')] = 10.0
    # gain for shoulder roll
    command_publisher.kp[BODY_JOINTS.index('left_shoulder_roll_joint')] = 50.0
    command_publisher.kd[BODY_JOINTS.index('left_shoulder_roll_joint')] = 10.0
    command_publisher.kp[BODY_JOINTS.index('right_shoulder_roll_joint')] = 50.0
    command_publisher.kd[BODY_JOINTS.index('right_shoulder_roll_joint')] = 10.0
    # gain for shoulder yaw
    command_publisher.kp[BODY_JOINTS.index('left_shoulder_yaw_joint')] = 50.0
    command_publisher.kd[BODY_JOINTS.index('left_shoulder_yaw_joint')] = 4.0
    command_publisher.kp[BODY_JOINTS.index('right_shoulder_yaw_joint')] = 50.0
    command_publisher.kd[BODY_JOINTS.index('right_shoulder_yaw_joint')] = 4.0
    # gain for elbow
    command_publisher.kp[BODY_JOINTS.index('left_elbow_joint')] = 50.0
    command_publisher.kd[BODY_JOINTS.index('left_elbow_joint')] = 4.0
    command_publisher.kp[BODY_JOINTS.index('right_elbow_joint')] = 50.0
    command_publisher.kd[BODY_JOINTS.index('right_elbow_joint')] = 4.0
    # gain for wrist roll
    command_publisher.kp[BODY_JOINTS.index('left_wrist_roll_joint')] = 20.0
    command_publisher.kd[BODY_JOINTS.index('left_wrist_roll_joint')] = 4.0
    command_publisher.kp[BODY_JOINTS.index('right_wrist_roll_joint')] = 20.0
    command_publisher.kd[BODY_JOINTS.index('right_wrist_roll_joint')] = 4.0
    # gain for wrist pitch
    command_publisher.kp[BODY_JOINTS.index('left_wrist_pitch_joint')] = 20.0
    command_publisher.kd[BODY_JOINTS.index('left_wrist_pitch_joint')] = 4.0
    command_publisher.kp[BODY_JOINTS.index('right_wrist_pitch_joint')] = 20.0
    command_publisher.kd[BODY_JOINTS.index('right_wrist_pitch_joint')] = 4.0
    # gain for wrist yaw
    command_publisher.kp[BODY_JOINTS.index('left_wrist_yaw_joint')] = 20.0
    command_publisher.kd[BODY_JOINTS.index('left_wrist_yaw_joint')] = 4.0
    command_publisher.kp[BODY_JOINTS.index('right_wrist_yaw_joint')] = 20.0
    command_publisher.kd[BODY_JOINTS.index('right_wrist_yaw_joint')] = 4.0
