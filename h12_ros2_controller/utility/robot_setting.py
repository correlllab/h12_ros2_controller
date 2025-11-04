from h12_ros2_controller.utility.joint_definition import BODY_JOINTS

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
    5.0,  # left_hip_yaw_joint (URDF: 23, conservative: 5.0)
    5.0,  # left_hip_pitch_joint (URDF: 23, conservative: 5.0)
    5.0,  # left_hip_roll_joint (URDF: 23, conservative: 5.0)
    3.0,  # left_knee_joint (URDF: 14, conservative: 3.0)
    2.0,  # left_ankle_pitch_joint (URDF: 9, conservative: 2.0)
    2.0,  # left_ankle_roll_joint (URDF: 9, conservative: 2.0)

    5.0,  # right_hip_yaw_joint (URDF: 23, conservative: 5.0)
    5.0,  # right_hip_pitch_joint (URDF: 23, conservative: 5.0)
    5.0,  # right_hip_roll_joint (URDF: 23, conservative: 5.0)
    3.0,  # right_knee_joint (URDF: 14, conservative: 3.0)
    2.0,  # right_ankle_pitch_joint (URDF: 9, conservative: 2.0)
    2.0,  # right_ankle_roll_joint (URDF: 9, conservative: 2.0)

    5.0,  # torso_joint (URDF: 23, conservative: 5.0)

    2.0,  # left_shoulder_pitch_joint (URDF: 9, conservative: 2.0)
    2.0,  # left_shoulder_roll_joint (URDF: 9, conservative: 2.0)
    4.0,  # left_shoulder_yaw_joint (URDF: 20, conservative: 4.0)
    4.0,  # left_elbow_joint (URDF: 20, conservative: 4.0)
    6.0,  # left_wrist_roll_joint (URDF: 31.4, conservative: 6.0)
    6.0,  # left_wrist_pitch_joint (URDF: 31.4, conservative: 6.0)
    6.0,  # left_wrist_yaw_joint (URDF: 31.4, conservative: 6.0)

    2.0,  # right_shoulder_pitch_joint (URDF: 9, conservative: 2.0)
    2.0,  # right_shoulder_roll_joint (URDF: 9, conservative: 2.0)
    4.0,  # right_shoulder_yaw_joint (URDF: 20, conservative: 4.0)
    4.0,  # right_elbow_joint (URDF: 20, conservative: 4.0)
    6.0,  # right_wrist_roll_joint (URDF: 31.4, conservative: 6.0)
    6.0,  # right_wrist_pitch_joint (URDF: 31.4, conservative: 6.0)
    6.0,  # right_wrist_yaw_joint (URDF: 31.4, conservative: 6.0)
]

JOINT_TORQUE_LIMITS = [
    40.0,  # left_hip_yaw_joint (URDF: 200, conservative: 40.0)
    40.0,  # left_hip_pitch_joint (URDF: 200, conservative: 40.0)
    40.0,  # left_hip_roll_joint (URDF: 200, conservative: 40.0)
    60.0,  # left_knee_joint (URDF: 300, conservative: 60.0)
    12.0,  # left_ankle_pitch_joint (URDF: 60, conservative: 12.0)
    8.0,   # left_ankle_roll_joint (URDF: 40, conservative: 8.0)

    40.0,  # right_hip_yaw_joint (URDF: 200, conservative: 40.0)
    40.0,  # right_hip_pitch_joint (URDF: 200, conservative: 40.0)
    40.0,  # right_hip_roll_joint (URDF: 200, conservative: 40.0)
    60.0,  # right_knee_joint (URDF: 300, conservative: 60.0)
    12.0,  # right_ankle_pitch_joint (URDF: 60, conservative: 12.0)
    8.0,   # right_ankle_roll_joint (URDF: 40, conservative: 8.0)

    40.0,  # torso_joint (URDF: 200, conservative: 40.0)

    8.0,   # left_shoulder_pitch_joint (URDF: 40, conservative: 8.0)
    8.0,   # left_shoulder_roll_joint (URDF: 40, conservative: 8.0)
    3.6,   # left_shoulder_yaw_joint (URDF: 18, conservative: 3.6)
    3.6,   # left_elbow_joint (URDF: 18, conservative: 3.6)
    3.8,   # left_wrist_roll_joint (URDF: 19, conservative: 3.8)
    3.8,   # left_wrist_pitch_joint (URDF: 19, conservative: 3.8)
    3.8,   # left_wrist_yaw_joint (URDF: 19, conservative: 3.8)

    8.0,   # right_shoulder_pitch_joint (URDF: 40, conservative: 8.0)
    8.0,   # right_shoulder_roll_joint (URDF: 40, conservative: 8.0)
    3.6,   # right_shoulder_yaw_joint (URDF: 18, conservative: 3.6)
    3.6,   # right_elbow_joint (URDF: 18, conservative: 3.6)
    3.8,   # right_wrist_roll_joint (URDF: 19, conservative: 3.8)
    3.8,   # right_wrist_pitch_joint (URDF: 19, conservative: 3.8)
    3.8,   # right_wrist_yaw_joint (URDF: 19, conservative: 3.8)
]

def setup_gains(command_publisher):
    '''Configure motor gains for all joints - optimized for real robot'''
    # initialize all gains to default values
    command_publisher.kp.fill(100.0)
    command_publisher.kd.fill(5.0)

    # gain for hip yaw - real robot settings
    command_publisher.kp[BODY_JOINTS.index('left_hip_yaw_joint')] = 200.0
    command_publisher.kd[BODY_JOINTS.index('left_hip_yaw_joint')] = 8.0
    command_publisher.kp[BODY_JOINTS.index('right_hip_yaw_joint')] = 200.0
    command_publisher.kd[BODY_JOINTS.index('right_hip_yaw_joint')] = 8.0

    # gain for hip pitch
    command_publisher.kp[BODY_JOINTS.index('left_hip_pitch_joint')] = 350.0
    command_publisher.kd[BODY_JOINTS.index('left_hip_pitch_joint')] = 35.0
    command_publisher.kp[BODY_JOINTS.index('right_hip_pitch_joint')] = 350.0
    command_publisher.kd[BODY_JOINTS.index('right_hip_pitch_joint')] = 35.0
    # gain for hip roll
    command_publisher.kp[BODY_JOINTS.index('left_hip_roll_joint')] = 250.0
    command_publisher.kd[BODY_JOINTS.index('left_hip_roll_joint')] = 25.0
    command_publisher.kp[BODY_JOINTS.index('right_hip_roll_joint')] = 250.0
    command_publisher.kd[BODY_JOINTS.index('right_hip_roll_joint')] = 25.0
    # gain for knee
    command_publisher.kp[BODY_JOINTS.index('left_knee_joint')] = 300.0
    command_publisher.kd[BODY_JOINTS.index('left_knee_joint')] = 10.0
    command_publisher.kp[BODY_JOINTS.index('right_knee_joint')] = 300.0
    command_publisher.kd[BODY_JOINTS.index('right_knee_joint')] = 10.0
    # gain for ankle pitch
    command_publisher.kp[BODY_JOINTS.index('left_ankle_pitch_joint')] = 150.0
    command_publisher.kd[BODY_JOINTS.index('left_ankle_pitch_joint')] = 5.0
    command_publisher.kp[BODY_JOINTS.index('right_ankle_pitch_joint')] = 150.0
    command_publisher.kd[BODY_JOINTS.index('right_ankle_pitch_joint')] = 5.0
    # gain for ankle roll
    command_publisher.kp[BODY_JOINTS.index('left_ankle_roll_joint')] = 150.0
    command_publisher.kd[BODY_JOINTS.index('left_ankle_roll_joint')] = 5.0
    command_publisher.kp[BODY_JOINTS.index('right_ankle_roll_joint')] = 150.0
    command_publisher.kd[BODY_JOINTS.index('right_ankle_roll_joint')] = 5.0

    # gain for torso (12)
    command_publisher.kp[BODY_JOINTS.index('torso_joint')] = 200.0
    command_publisher.kd[BODY_JOINTS.index('torso_joint')] = 10.0

    # gain for shoulder pitch
    command_publisher.kp[BODY_JOINTS.index('left_shoulder_pitch_joint')] = 200.0
    command_publisher.kd[BODY_JOINTS.index('left_shoulder_pitch_joint')] = 10.0
    command_publisher.kp[BODY_JOINTS.index('right_shoulder_pitch_joint')] = 200.0
    command_publisher.kd[BODY_JOINTS.index('right_shoulder_pitch_joint')] = 10.0
    # gain for shoulder roll
    command_publisher.kp[BODY_JOINTS.index('left_shoulder_roll_joint')] = 200.0
    command_publisher.kd[BODY_JOINTS.index('left_shoulder_roll_joint')] = 10.0
    command_publisher.kp[BODY_JOINTS.index('right_shoulder_roll_joint')] = 200.0
    command_publisher.kd[BODY_JOINTS.index('right_shoulder_roll_joint')] = 10.0
    # gain for shoulder yaw
    command_publisher.kp[BODY_JOINTS.index('left_shoulder_yaw_joint')] = 150.0
    command_publisher.kd[BODY_JOINTS.index('left_shoulder_yaw_joint')] = 4.0
    command_publisher.kp[BODY_JOINTS.index('right_shoulder_yaw_joint')] = 150.0
    command_publisher.kd[BODY_JOINTS.index('right_shoulder_yaw_joint')] = 4.0
    # gain for elbow
    command_publisher.kp[BODY_JOINTS.index('left_elbow_joint')] = 150.0
    command_publisher.kd[BODY_JOINTS.index('left_elbow_joint')] = 4.0
    command_publisher.kp[BODY_JOINTS.index('right_elbow_joint')] = 150.0
    command_publisher.kd[BODY_JOINTS.index('right_elbow_joint')] = 4.0
    # gain for wrist roll
    command_publisher.kp[BODY_JOINTS.index('left_wrist_roll_joint')] = 120.0
    command_publisher.kd[BODY_JOINTS.index('left_wrist_roll_joint')] = 4.0
    command_publisher.kp[BODY_JOINTS.index('right_wrist_roll_joint')] = 120.0
    command_publisher.kd[BODY_JOINTS.index('right_wrist_roll_joint')] = 4.0
    # gain for wrist pitch
    command_publisher.kp[BODY_JOINTS.index('left_wrist_pitch_joint')] = 120.0
    command_publisher.kd[BODY_JOINTS.index('left_wrist_pitch_joint')] = 4.0
    command_publisher.kp[BODY_JOINTS.index('right_wrist_pitch_joint')] = 120.0
    command_publisher.kd[BODY_JOINTS.index('right_wrist_pitch_joint')] = 4.0
    # gain for wrist yaw
    command_publisher.kp[BODY_JOINTS.index('left_wrist_yaw_joint')] = 120.0
    command_publisher.kd[BODY_JOINTS.index('left_wrist_yaw_joint')] = 4.0
    command_publisher.kp[BODY_JOINTS.index('right_wrist_yaw_joint')] = 120.0
    command_publisher.kd[BODY_JOINTS.index('right_wrist_yaw_joint')] = 4.0
