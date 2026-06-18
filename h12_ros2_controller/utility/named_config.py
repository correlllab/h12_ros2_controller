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
    'home':              np.zeros(len(ENABLED_JOINTS)),                       # arms down (low load baseline)
    't_pose':            np.array([ 0.0, 1.5, 0,  0.0, 0,0,0,   0.0,-1.5, 0,  0.0, 0,0,0]),  # arms out to sides, horizontal
    'arms_front':        np.array([-1.45,0.0, 0,  0.0, 0,0,0,  -1.45,0.0, 0,  0.0, 0,0,0]),  # arms straight forward, horizontal
    'arms_front_elbow':  np.array([-1.45,0.0, 0, -1.0, 0,0,0,  -1.45,0.0, 0,  1.0, 0,0,0]),  # forward + elbow bent (forearm CoM); elbow 1.0 = collision-free
    't_pose_elbow':      np.array([ 0.0, 1.5, 0, -1.0, 0,0,0,   0.0,-1.5, 0,  1.0, 0,0,0]),  # sides + elbow bent
    'arms_front_45':     np.array([-0.75,0.0, 0,  0.0, 0,0,0,  -0.75,0.0, 0,  0.0, 0,0,0]),  # forward 45 deg (mid load, conditioning)
    'arms_overhead':     np.array([ 0.0, 2.9, 0,  0.0, 0,0,0,   0.0,-2.9, 0,  0.0, 0,0,0]),  # arms overhead (different gravity dir)
    'arms_asym':         np.array([-1.45,0.0, 0,  0.0, 0,0,0,   0.0,-1.5, 0,  0.0, 0,0,0]),  # L forward / R to side (excitation diversity)
    'arms_front_yaw':    np.array([-1.45,0.0, 1.0,0.0, 0,0,0,  -1.45,0.0,-1.0,0.0, 0,0,0]),  # forward + shoulder-yaw twist (upper-arm axial)
    'elbow_only':        np.array([ 0.0, 0.0, 0, -1.0, 0,0,0,   0.0, 0.0, 0,  1.0, 0,0,0]),  # arms down, elbows bent (loads elbow); elbow 1.0 = collision-free
}
