import numpy as np

from h12_ros2_controller.utility.joint_definition import ENABLED_JOINTS

# ENABLED_JOINTS is UPPER-BODY (ARMS) ONLY in debug mode (safety_split.yaml: lower-body
# gains = 0, cmd -> rt/safety/lowcmd_upper_in). So every config below poses the 14 ARM
# joints; legs/ankles are NOT commandable here. These poses are sysid excitation for
# identifying ARM link mass/CoM (P2): hold the robot suspended (gantry, upright) and cycle
# through them while dds_live_recorder runs. Poses with arms HORIZONTAL load the shoulders/
# elbows maximally (max gravity signal); elbow-bent variants separate the forearm CoM.
#
# index: 0 L_sh_pitch 1 L_sh_roll 2 L_sh_yaw 3 L_elbow 4 L_wr_roll 5 L_wr_pitch 6 L_wr_yaw
#        7 R_sh_pitch 8 R_sh_roll 9 R_sh_yaw 10 R_elbow 11 R_wr_roll 12 R_wr_pitch 13 R_wr_yaw
# (all values rad, validated in joint range)
NAMED_CONFIGS = {
    'home':  # arms down (low load baseline)
        np.zeros(len(ENABLED_JOINTS)),
    't_pose':  # arms out to sides, horizontal
        np.array([0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, -1.5, 0.0, 0.0, 0.0, 0.0, 0.0]),
    # 'arms_front':  # arms straight forward, horizontal
    #     np.array([-1.45, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    #               -1.45, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    # 'arms_front_elbow':  # forward + elbow bent (forearm CoM); elbow 1.0 = collision-free
    #     np.array([-1.45, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0,
    #               -1.45, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]),
    # 't_pose_elbow':  # sides + elbow bent
    #     np.array([0.0, 1.5, 0.0, -1.0, 0.0, 0.0, 0.0,
    #               0.0, -1.5, 0.0, 1.0, 0.0, 0.0, 0.0]),
    'arms_front_45':  # forward 45 deg (mid load, conditioning)
        np.array([-0.75, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  -0.75, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    'arms_overhead':  # arms overhead (different gravity dir)
        np.array([0.0, 2.9, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, -2.9, 0.0, 0.0, 0.0, 0.0, 0.0]),
    'arms_asym':  # L forward / R to side (excitation diversity)
        np.array([-1.45, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, -1.5, 0.0, 0.0, 0.0, 0.0, 0.0]),
    'arms_front_yaw':  # forward + shoulder-yaw twist (upper-arm axial)
        np.array([-1.45, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                  -1.45, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0]),
    'elbow_only':  # arms down, elbows bent (loads elbow); elbow 1.0 = collision-free
        np.array([0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]),
    'init_1':  # move arm sideways
        np.array([0.0, 0.35, 0.0, 1.3, 0.0, 0.0, 0.0,
                  0.0, -0.35, 0.0, 1.3, 0.0, 0.0, 0.0]),
    'init_2':
        np.array([0.7, 0.35, 0.0, 1.3, 0.0, 0.0, 0.0,
                  0.7, -0.35, 0.0, 1.3, 0.0, 0.0, 0.0]),
    'init_3':
        np.array([0.7, 0.35, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.7, -0.35, 0.0, 0.0, 0.0, 0.0, 0.0]),
    'prep':  # init_3 with elbow bend for level, raised grasp frames
        np.array([0.7, 0.35, 0.0, -0.7, 0.0, 0.0, 0.0,
                  0.7, -0.35, 0.0, -0.7, 0.0, 0.0, 0.0]),
    'staging_right':  # right arm RE-RECORDED live from the real robot's /lowstate
                      # 2026-07-24 (arm motors 20-26) at the grasp staging pose;
                      # left (idle) arm held at 'prep'. NOTE wrist_yaw -1.2632 sits
                      # ~0.007 rad off its -1.27 limit — leave headroom if retuning.
        np.array([0.7, 0.35, 0.0, -0.7, 0.0, 0.0, 0.0,
                  -1.0597, -0.8747, 1.5802, -0.5551, 2.4575, -0.0036, -1.2632]),
    'staging_left':  # MIRROR of 'staging_right' across the sagittal plane: left
                     # arm staged, right (idle) arm at 'prep' ('prep' is itself
                     # L/R symmetric, so the mirror still holds). Mirror rule (arm
                     # joint axes are identical L/R in the URDF, so a XZ reflection
                     # negates the roll/yaw axes only): pitch same, roll FLIP, yaw
                     # FLIP, elbow same, wrist_roll FLIP, wrist_pitch same,
                     # wrist_yaw FLIP. DERIVED, not recorded live — sanity-check
                     # against the LEFT joint limits before trusting.
        np.array([-1.0597, 0.8747, -1.5802, -0.5551, -2.4575, -0.0036, 1.2632,
                  0.7, -0.35, 0.0, -0.7, 0.0, 0.0, 0.0]),
}
