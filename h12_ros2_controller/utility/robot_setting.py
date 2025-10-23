from h12_ros2_controller.utility.joint_definition import BODY_JOINTS

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
