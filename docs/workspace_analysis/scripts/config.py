'''Shared paths, robot limits, camera calibration and pose definitions'''
import os

import numpy as np

# single-threaded math per worker so the process pool does not oversubscribe the cpu
for _var in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_var, '1')

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPORT_DIR = os.path.dirname(SCRIPT_DIR)
REPO_DIR = os.path.dirname(os.path.dirname(REPORT_DIR))
DATA_DIR = os.environ.get('WS_DATA_DIR', os.path.join(REPORT_DIR, 'data'))
ASSETS = os.path.join(REPO_DIR, 'submodules', 'CL_Assets', 'ros_assets')
URDF = os.path.join(ASSETS, 'h1_2_magpie.urdf')
URDF_SPHERE = os.path.join(ASSETS, 'h1_2_magpie_sphere.urdf')
SRDF_SPHERE = os.path.join(ASSETS, 'h1_2_magpie_sphere_collision.srdf')

ARM_JOINTS = ['shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow',
              'wrist_roll', 'wrist_pitch', 'wrist_yaw']
# software clip limits from config/safety_split.yaml
CLIP = {
    'left': [(-3.139, 1.569), (-0.379, 3.399), (-2.659, 3.009), (-0.949, 3.179),
             (-3.009, 2.749), (-0.4615, 0.4615), (-1.269, 1.269)],
    'right': [(-3.139, 1.569), (-3.399, 0.379), (-3.009, 2.659), (-0.949, 3.179),
              (-2.749, 3.009), (-0.4615, 0.4615), (-1.269, 1.269)],
}
# a point is good when clearance >= MARGIN, no self-collision and min singular value >= SMIN
MARGIN = 0.05
SMIN = 0.06
# joint sign flip that maps a left-arm configuration onto the right arm
MIRROR = np.array([1, -1, -1, 1, -1, 1, -1])
# diverse left-arm IK seeds covering the elbow-down and elbow-out branches
SEEDS = [
    [-0.6, 0.25, 0.0, 1.0, 0.0, 0.0, 0.0],
    [-1.0, 0.2, 0.2, 2.6, 0.1, 0.1, 0.0],
    [-1.0, 0.8, -1.4, -0.3, -2.5, 0.2, 1.0],
    [-1.6, 0.6, -2.4, -0.4, -2.0, 0.3, 0.4],
    [-1.5, 0.5, -2.5, -0.3, 2.3, -0.3, 0.1],
    [-1.4, 0.3, 0.4, 3.0, -0.8, -0.2, 0.0],
    [-1.4, 0.2, 0.2, 3.0, 0.9, -0.1, 0.0],
    [-1.2, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0],
]
# posture the IK is weakly pulled toward: elbow bent, arm forward
NOMINAL = {
    'left': np.array([-0.6, 0.25, 0.0, 1.0, 0.0, 0.0, 0.0]),
    'right': np.array([-0.6, -0.25, 0.0, 1.0, 0.0, 0.0, 0.0]),
}

# left-arm grid in the pelvis frame; the right arm uses the mirrored y range
STEP = 0.025
COARSE_STEP = 0.05
GRID = dict(x=(0.0, 0.80), y=(-0.25, 0.80), z=(-0.35, 0.55))
# only the table floor constrains the boxes
Z_FLOOR = -0.05

# head RealSense D435i, captured from the real robot in
# GOLEM h1_robocasa/realsense_profile.yaml (hand-to-eye calibrated mount)
# chain: head_camera_link -> head_link -> head_color_frame -> head_color_optical_frame
CAMERA_CHAIN = [
    ([0.000985, 0.014977, -0.018805], [-0.510111, 0.523885, -0.486398, -0.478277]),
    ([-0.00022629060549661517, 0.014902372844517231, 3.6659606848843396e-05],
     [0.007132303901016712, -0.0024601402692496777, -0.0033503943122923374, 0.9999659061431885]),
    ([0.0, 0.0, 0.0], [-0.5, 0.5, -0.5, 0.5]),
]
COLOR_K = dict(w=1280, h=720, fx=909.6611328125, fy=909.5230102539062,
               cx=647.69384765625, cy=369.2969970703125)
DEPTH_K = dict(w=1280, h=720, fx=642.6455078125, fy=642.6455078125,
               cx=651.2221069335938, cy=358.1368103027344)
# d435 minimum depth at 1280x720
DEPTH_MIN = 0.28

# gripper orientations, left arm; the right arm negates roll and yaw
# free: the angle chosen by the search ('yaw' for a vertical gripper, else 'roll')
# box: pinned (left, right) boxes of the published report; set to None to recompute
POSES = [
    dict(key='pitch90', title='Gripper pointing down', label='Gripper Pointing Down',
         pitch=90, yaw=-60, roll=0, free='yaw',
         box=((0.40, 0.50, 0.125, 0.325, 0.05, 0.225), (0.40, 0.50, -0.325, -0.125, 0.05, 0.225))),
    dict(key='pitch60', title='Tilted forward, pitch 60°', label='Pitch 60°',
         pitch=60, yaw=0, roll=-90, free='roll',
         box=((0.40, 0.625, 0.175, 0.375, 0.075, 0.325), (0.45, 0.625, -0.35, -0.175, 0.05, 0.25))),
    dict(key='pitch45', title='Tilted forward, pitch 45°', label='Pitch 45°',
         pitch=45, yaw=0, roll=0, free='roll',
         box=((0.425, 0.60, 0.275, 0.45, 0.025, 0.20), (0.50, 0.65, -0.35, -0.20, 0.05, 0.30))),
    dict(key='pitch0', title='Horizontal, pointing forward', label='Horizontal Forward',
         pitch=0, yaw=0, roll=-60, free='roll',
         note=('The horizontal grasp is the most fragmented pose: in the visible forward region '
               'about 30% of grid points fail (no IK solution, shoulder roll or wrist pitch near '
               'its limit, or self-collision), which caps any fully reachable box.'),
         box=((0.45, 0.65, 0.10, 0.40, 0.125, 0.325), (0.425, 0.625, -0.325, -0.125, 0.075, 0.275))),
]
# candidate free angles scanned for the explanation tables
YAWS = (-90, -75, -60, -45, -30, 0, 30, 60)
ROLLS = (-90, -60, -30, 0, 30, 60, 90)


def data_path(kind, side, pitch, yaw, roll):
    '''Path of a sweep result: kind is fine, scan or yaw'''
    return os.path.join(DATA_DIR, kind, f'{side}_p{pitch}_y{yaw}_r{roll}.npy')


def mirrored(pose):
    '''Right-arm (pitch, yaw, roll) of a left-arm pose'''
    return pose['pitch'], -pose['yaw'], -pose['roll']
