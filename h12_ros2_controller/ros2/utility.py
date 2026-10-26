import warnings

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose

import numpy as np
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R


def _project_to_so3(rotation):
    u, _, vh = np.linalg.svd(rotation)
    projected = u @ vh
    if np.linalg.det(projected) < 0:
        u[:, -1] *= -1
        projected = u @ vh
    return projected

def pose_to_matrix(pose):
    position = pose.position
    orientation = pose.orientation
    rotation = Quaternion(
        [orientation.w, orientation.x, orientation.y, orientation.z]
    ).rotation_matrix
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = [position.x, position.y, position.z]
    return matrix

def matrix_to_pose(matrix):
    pose = Pose()
    translation = np.asarray(matrix[0:3, 3], dtype=float)
    if not np.isfinite(translation).all():
        translation = np.zeros(3)
    pose.position.x = translation[0]
    pose.position.y = translation[1]
    pose.position.z = translation[2]
    rotation_matrix = np.asarray(matrix[:3, :3], dtype=float)

    if not np.isfinite(rotation_matrix).all():
        warnings.warn(
            "matrix_to_pose: rotation matrix contains non-finite values; "
            "falling back to identity rotation. This may indicate an upstream issue.",
            RuntimeWarning,
            stacklevel=2,
        )
        rotation_matrix = np.eye(3)
    else:
        orthogonality_error = np.linalg.norm(rotation_matrix.T @ rotation_matrix - np.eye(3), ord='fro')
        determinant = np.linalg.det(rotation_matrix)
        if orthogonality_error > 1e-6 or not np.isfinite(determinant) or determinant <= 0:
            rotation_matrix = _project_to_so3(rotation_matrix)

    rotation = Quaternion(matrix=rotation_matrix)
    pose.orientation.w = rotation.w
    pose.orientation.x = rotation.x
    pose.orientation.y = rotation.y
    pose.orientation.z = rotation.z
    return pose

def list_to_pose(values):
    assert(len(values) == 6), 'Please enter a pose of 6 elements'
    x, y, z, roll, pitch, yaw = values
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    quat = R.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_quat()
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    return pose

def action_result_summary(response):
    '''Format the ROS action terminal status and result together'''
    labels = {
        GoalStatus.STATUS_SUCCEEDED: 'succeeded',
        GoalStatus.STATUS_CANCELED: 'canceled',
        GoalStatus.STATUS_ABORTED: 'aborted',
    }
    status = labels.get(response.status, f'unknown({response.status})')
    return f'status={status}, success={response.result.success}'
