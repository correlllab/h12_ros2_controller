# free-flyer base offsets
# q vector: [x, y, z, qx, qy, qz, qw] then motor joints
# v vector: [vx, vy, vz, wx, wy, wz] then motor joints
FREEFLYER_NQ = 7  # position (3) + quaternion xyzw (4)
FREEFLYER_NV = 6  # linear vel (3) + angular vel (3)

# slice indices for free-flyer q vector (Pinocchio xyzw convention)
FREEFLYER_POS = slice(0, 3)   # [x, y, z]
FREEFLYER_QUAT = slice(3, 7)  # [qx, qy, qz, qw]

# motor degrees of freedom on the Unitree DDS lowstate/cmd path.
# magpie grippers are not on this path.
NUM_MOTOR = 27
NUM_HAND_DOF = 0

# All joints the controller observes. Magpie gripper joints are included so
# they can be frozen out during model initialization.
ALL_JOINTS = [
    'left_hip_yaw_joint',
    'left_hip_pitch_joint',
    'left_hip_roll_joint',
    'left_knee_joint',
    'left_ankle_pitch_joint',
    'left_ankle_roll_joint',
    'right_hip_yaw_joint',
    'right_hip_pitch_joint',
    'right_hip_roll_joint',
    'right_knee_joint',
    'right_ankle_pitch_joint',
    'right_ankle_roll_joint',
    'torso_joint',
    'left_shoulder_pitch_joint',
    'left_shoulder_roll_joint',
    'left_shoulder_yaw_joint',
    'left_elbow_joint',
    'left_wrist_roll_joint',
    'left_wrist_pitch_joint',
    'left_wrist_yaw_joint',
    'right_shoulder_pitch_joint',
    'right_shoulder_roll_joint',
    'right_shoulder_yaw_joint',
    'right_elbow_joint',
    'right_wrist_roll_joint',
    'right_wrist_pitch_joint',
    'right_wrist_yaw_joint',
    'left_grasp_frame_joint',
    'left_graspgenx_frame_joint',
    'right_grasp_frame_joint',
    'right_graspgenx_frame_joint',
    'lg_mount_joint',
    'lg_base_bot_joint',
    'lg_base_top_joint',
    'lg_left_hinge_1',
    'lg_left_hinge_2',
    'lg_left_hinge_3',
    'lg_right_hinge_1',
    'lg_right_hinge_2',
    'lg_right_hinge_3',
    'rg_mount_joint',
    'rg_base_bot_joint',
    'rg_base_top_joint',
    'rg_left_hinge_1',
    'rg_left_hinge_2',
    'rg_left_hinge_3',
    'rg_right_hinge_1',
    'rg_right_hinge_2',
    'rg_right_hinge_3',
]

ALL_HANDLESS_JOINTS = [
    'left_hip_yaw_joint',
    'left_hip_pitch_joint',
    'left_hip_roll_joint',
    'left_knee_joint',
    'left_ankle_pitch_joint',
    'left_ankle_roll_joint',
    'right_hip_yaw_joint',
    'right_hip_pitch_joint',
    'right_hip_roll_joint',
    'right_knee_joint',
    'right_ankle_pitch_joint',
    'right_ankle_roll_joint',
    'torso_joint',
    'left_shoulder_pitch_joint',
    'left_shoulder_roll_joint',
    'left_shoulder_yaw_joint',
    'left_elbow_joint',
    'left_wrist_roll_joint',
    'left_wrist_pitch_joint',
    'left_wrist_yaw_joint',
    'right_shoulder_pitch_joint',
    'right_shoulder_roll_joint',
    'right_shoulder_yaw_joint',
    'right_elbow_joint',
    'right_wrist_roll_joint',
    'right_wrist_pitch_joint',
    'right_wrist_yaw_joint',
]

# body joints without hand joints
BODY_JOINTS = [
    'left_hip_yaw_joint',
    'left_hip_pitch_joint',
    'left_hip_roll_joint',
    'left_knee_joint',
    'left_ankle_pitch_joint',
    'left_ankle_roll_joint',
    'right_hip_yaw_joint',
    'right_hip_pitch_joint',
    'right_hip_roll_joint',
    'right_knee_joint',
    'right_ankle_pitch_joint',
    'right_ankle_roll_joint',
    'torso_joint',
    'left_shoulder_pitch_joint',
    'left_shoulder_roll_joint',
    'left_shoulder_yaw_joint',
    'left_elbow_joint',
    'left_wrist_roll_joint',
    'left_wrist_pitch_joint',
    'left_wrist_yaw_joint',
    'right_shoulder_pitch_joint',
    'right_shoulder_roll_joint',
    'right_shoulder_yaw_joint',
    'right_elbow_joint',
    'right_wrist_roll_joint',
    'right_wrist_pitch_joint',
    'right_wrist_yaw_joint',
]

LOWER_BODY_JOINTS = [
    'left_hip_yaw_joint',
    'left_hip_pitch_joint',
    'left_hip_roll_joint',
    'left_knee_joint',
    'left_ankle_pitch_joint',
    'left_ankle_roll_joint',
    'right_hip_yaw_joint',
    'right_hip_pitch_joint',
    'right_hip_roll_joint',
    'right_knee_joint',
    'right_ankle_pitch_joint',
    'right_ankle_roll_joint'
]

UPPER_BODY_JOINTS = [
    'torso_joint',
    'left_shoulder_pitch_joint',
    'left_shoulder_roll_joint',
    'left_shoulder_yaw_joint',
    'left_elbow_joint',
    'left_wrist_roll_joint',
    'left_wrist_pitch_joint',
    'left_wrist_yaw_joint',
    'right_shoulder_pitch_joint',
    'right_shoulder_roll_joint',
    'right_shoulder_yaw_joint',
    'right_elbow_joint',
    'right_wrist_roll_joint',
    'right_wrist_pitch_joint',
    'right_wrist_yaw_joint'
]

HAND_JOINTS = []

ENABLED_JOINTS = [
    # 'torso_joint',
    # left arm
    'left_shoulder_pitch_joint',
    'left_shoulder_roll_joint',
    'left_shoulder_yaw_joint',
    'left_elbow_joint',
    'left_wrist_roll_joint',
    'left_wrist_pitch_joint',
    'left_wrist_yaw_joint',
    # right arm
    'right_shoulder_pitch_joint',
    'right_shoulder_roll_joint',
    'right_shoulder_yaw_joint',
    'right_elbow_joint',
    'right_wrist_roll_joint',
    'right_wrist_pitch_joint',
    'right_wrist_yaw_joint'
]

UPPER_BODY_INDEX = [BODY_JOINTS.index(joint) for joint in UPPER_BODY_JOINTS]
LOWER_BODY_INDEX = [BODY_JOINTS.index(joint) for joint in LOWER_BODY_JOINTS]
LEFT_ARM_INDEX = [i for i in range(13, 20)]
RIGHT_ARM_INDEX = [i for i in range(20, 27)]
