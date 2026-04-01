# free-flyer base offsets
# q vector: [x, y, z, qx, qy, qz, qw] then motor joints
# v vector: [vx, vy, vz, wx, wy, wz] then motor joints
FREEFLYER_NQ = 7  # position (3) + quaternion xyzw (4)
FREEFLYER_NV = 6  # linear vel (3) + angular vel (3)

# slice indices for free-flyer q vector (Pinocchio xyzw convention)
FREEFLYER_POS = slice(0, 3)   # [x, y, z]
FREEFLYER_QUAT = slice(3, 7)  # [qx, qy, qz, qw]

# motor and hand degrees of freedom
NUM_MOTOR = 27
NUM_HAND_DOF = 12

# all joints defined in the URDF
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
    'L_index_proximal_joint',
    'L_index_intermediate_joint',
    'L_middle_proximal_joint',
    'L_middle_intermediate_joint',
    'L_pinky_proximal_joint',
    'L_pinky_intermediate_joint',
    'L_ring_proximal_joint',
    'L_ring_intermediate_joint',
    'L_thumb_proximal_yaw_joint',
    'L_thumb_proximal_pitch_joint',
    'L_thumb_intermediate_joint',
    'L_thumb_distal_joint',
    'right_shoulder_pitch_joint',
    'right_shoulder_roll_joint',
    'right_shoulder_yaw_joint',
    'right_elbow_joint',
    'right_wrist_roll_joint',
    'right_wrist_pitch_joint',
    'right_wrist_yaw_joint',
    'R_index_proximal_joint',
    'R_index_intermediate_joint',
    'R_middle_proximal_joint',
    'R_middle_intermediate_joint',
    'R_pinky_proximal_joint',
    'R_pinky_intermediate_joint',
    'R_ring_proximal_joint',
    'R_ring_intermediate_joint',
    'R_thumb_proximal_yaw_joint',
    'R_thumb_proximal_pitch_joint',
    'R_thumb_intermediate_joint',
    'R_thumb_distal_joint'
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

HAND_JOINTS = [
    'L_index_proximal_joint',
    'L_index_intermediate_joint',
    'L_middle_proximal_joint',
    'L_middle_intermediate_joint',
    'L_pinky_proximal_joint',
    'L_pinky_intermediate_joint',
    'L_ring_proximal_joint',
    'L_ring_intermediate_joint',
    'L_thumb_proximal_yaw_joint',
    'L_thumb_proximal_pitch_joint',
    'L_thumb_intermediate_joint',
    'L_thumb_distal_joint',
    'R_index_proximal_joint',
    'R_index_intermediate_joint',
    'R_middle_proximal_joint',
    'R_middle_intermediate_joint',
    'R_pinky_proximal_joint',
    'R_pinky_intermediate_joint',
    'R_ring_proximal_joint',
    'R_ring_intermediate_joint',
    'R_thumb_proximal_yaw_joint',
    'R_thumb_proximal_pitch_joint',
    'R_thumb_intermediate_joint',
    'R_thumb_distal_joint'
]

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
LEFT_ARM_INDEX = [i for i in range(13, 20)]
RIGHT_ARM_INDEX = [i for i in range(20, 27)]
