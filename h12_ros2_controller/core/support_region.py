from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class SupportRectangle:
    '''Oriented planar support rectangle'''

    center: np.ndarray
    forward_axis: np.ndarray
    lateral_axis: np.ndarray
    half_extents: np.ndarray
    valid: bool
    invalid_reason: str = ''


def _invalid_rectangle(reason):
    nan2 = np.full(2, np.nan, dtype=np.float64)
    return SupportRectangle(
        center=nan2.copy(),
        forward_axis=nan2.copy(),
        lateral_axis=nan2.copy(),
        half_extents=nan2.copy(),
        valid=False,
        invalid_reason=reason,
    )


def _as_transform(transform, name):
    if transform is None:
        raise ValueError(f'missing {name} foot frame')
    array = np.asarray(transform, dtype=np.float64)
    if array.shape != (4, 4):
        raise ValueError(f'{name} foot transform must have shape (4, 4)')
    if not np.all(np.isfinite(array)):
        raise ValueError(f'{name} foot transform is nonfinite')
    return array


def compute_support_rectangle(left_foot_transform, right_foot_transform,
                              front=0.174, rear=0.086, half_width=0.043,
                              max_yaw_divergence=0.349):
    '''Build an oriented rectangle enclosing both sole rectangles'''
    try:
        left = _as_transform(left_foot_transform, 'left')
        right = _as_transform(right_foot_transform, 'right')
        geometry = np.asarray(
            [front, rear, half_width, max_yaw_divergence],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(geometry)):
            raise ValueError('support geometry is nonfinite')
        if front <= 0.0 or rear <= 0.0 or half_width <= 0.0:
            raise ValueError('sole dimensions must be positive')
        if max_yaw_divergence < 0.0 or max_yaw_divergence > np.pi:
            raise ValueError('max yaw divergence must be in [0, pi]')

        forward = []
        for name, transform in (('left', left), ('right', right)):
            axis = np.asarray(transform[:2, 0], dtype=np.float64)
            norm = np.linalg.norm(axis)
            if not np.isfinite(norm) or norm <= 1e-9:
                raise ValueError(f'{name} foot forward axis is degenerate')
            forward.append(axis / norm)

        left_forward, right_forward = forward
        if np.dot(left_forward, right_forward) < 0.0:
            right_forward = -right_forward
        yaw_divergence = np.arccos(np.clip(
            np.dot(left_forward, right_forward),
            -1.0,
            1.0,
        ))
        if yaw_divergence > max_yaw_divergence:
            raise ValueError('foot yaw divergence exceeds configured maximum')

        forward_axis = left_forward + right_forward
        forward_norm = np.linalg.norm(forward_axis)
        if not np.isfinite(forward_norm) or forward_norm <= 1e-9:
            raise ValueError('averaged support forward axis is degenerate')
        forward_axis /= forward_norm
        lateral_axis = np.array(
            [-forward_axis[1], forward_axis[0]],
            dtype=np.float64,
        )

        local_corners = np.array([
            [front, half_width, 0.0, 1.0],
            [front, -half_width, 0.0, 1.0],
            [-rear, half_width, 0.0, 1.0],
            [-rear, -half_width, 0.0, 1.0],
        ], dtype=np.float64)
        corners = np.vstack([
            (left @ local_corners.T).T[:, :2],
            (right @ local_corners.T).T[:, :2],
        ])
        if not np.all(np.isfinite(corners)):
            raise ValueError('transformed sole geometry is nonfinite')

        forward_projection = corners @ forward_axis
        lateral_projection = corners @ lateral_axis
        projection_min = np.array([
            np.min(forward_projection),
            np.min(lateral_projection),
        ])
        projection_max = np.array([
            np.max(forward_projection),
            np.max(lateral_projection),
        ])
        projection_center = 0.5 * (projection_min + projection_max)
        half_extents = 0.5 * (projection_max - projection_min)
        if not np.all(np.isfinite(half_extents)) or np.any(half_extents <= 0.0):
            raise ValueError('support half-extents are degenerate')
        center = (
            projection_center[0] * forward_axis
            + projection_center[1] * lateral_axis
        )
        return SupportRectangle(
            center=center,
            forward_axis=forward_axis,
            lateral_axis=lateral_axis,
            half_extents=half_extents,
            valid=True,
        )
    except (TypeError, ValueError) as err:
        return _invalid_rectangle(str(err))


support_rectangle = compute_support_rectangle


def signed_support_margin(point, rectangle):
    '''Return positive-inside signed margin to an oriented rectangle'''
    if not rectangle.valid:
        return np.nan
    point = np.asarray(point, dtype=np.float64).reshape(-1)
    if point.size < 2 or not np.all(np.isfinite(point[:2])):
        return np.nan
    delta = point[:2] - rectangle.center
    distances = np.abs(np.array([
        np.dot(delta, rectangle.forward_axis),
        np.dot(delta, rectangle.lateral_axis),
    ]))
    return float(np.min(rectangle.half_extents - distances))
