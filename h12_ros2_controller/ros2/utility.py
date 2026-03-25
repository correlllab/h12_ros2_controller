from geometry_msgs.msg import Pose

import numpy as np
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R


_MATRIX_TO_POSE_WARN_COUNT = 0


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
    global _MATRIX_TO_POSE_WARN_COUNT
    pose = Pose()
    pose.position.x = matrix[0, 3]
    pose.position.y = matrix[1, 3]
    pose.position.z = matrix[2, 3]
    rotation_matrix = np.asarray(matrix[:3, :3], dtype=float)

    if not np.isfinite(rotation_matrix).all():
        _MATRIX_TO_POSE_WARN_COUNT += 1
        if _MATRIX_TO_POSE_WARN_COUNT <= 10 or _MATRIX_TO_POSE_WARN_COUNT % 100 == 0:
            print(
                '[WARN][matrix_to_pose] non-finite rotation block detected; '
                f'count={_MATRIX_TO_POSE_WARN_COUNT} '
                f'raw={np.array2string(rotation_matrix, precision=4, suppress_small=False)}',
                flush=True,
            )
        rotation_matrix = np.eye(3)
    else:
        orthogonality_error = np.linalg.norm(rotation_matrix.T @ rotation_matrix - np.eye(3), ord='fro')
        determinant = np.linalg.det(rotation_matrix)

        severe_outlier = orthogonality_error > 1e-3 or determinant <= 0.0 or abs(determinant - 1.0) > 1e-3
        if severe_outlier:
            _MATRIX_TO_POSE_WARN_COUNT += 1
            if _MATRIX_TO_POSE_WARN_COUNT <= 10 or _MATRIX_TO_POSE_WARN_COUNT % 100 == 0:
                print(
                    '[WARN][matrix_to_pose] severe SO(3) outlier before projection; '
                    f'count={_MATRIX_TO_POSE_WARN_COUNT} '
                    f'ortho_fro={orthogonality_error:.3e} '
                    f'det={determinant:.8f} '
                    f'raw={np.array2string(rotation_matrix, precision=4, suppress_small=False)}',
                    flush=True,
                )

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

def input_pose():
    while True:
        input_pose = input('Enter x y z roll pitch yaw (separated by space): ')
        parts = input_pose.strip().split()

        if len(parts) != 6:
            print('Invalid input. Please enter exactly 6 values.')
            continue
        try:
            values = [float(val) for val in parts]
            # degrees to radians for roll, pitch, yaw
            values[3:] = np.deg2rad(values[3:])
            return list_to_pose(values)
        except ValueError:
            print('Invalid input. Make sure all 6 values are numeric.')
