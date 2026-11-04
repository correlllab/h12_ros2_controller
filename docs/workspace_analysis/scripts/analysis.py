'''Grid helpers, box search and head camera projection'''
from functools import lru_cache

import numpy as np
import pinocchio as pin

import config as cfg
import kinematics as kin


def to_grid(arr):
    '''Axes, good mask and joint solutions of a result array on its regular grid'''
    xs, ys, zs = (np.unique(np.round(arr[:, i], 3)) for i in range(3))
    good = np.zeros((len(xs), len(ys), len(zs)), bool)
    q = np.full(good.shape + (7,), np.nan)
    idx = tuple(np.searchsorted(a, np.round(arr[:, i], 3)) for i, a in enumerate((xs, ys, zs)))
    good[idx] = kin.good_mask(arr)
    q[idx] = arr[:, 7:]
    return xs, ys, zs, good, q


def box_slices(axes, box):
    return tuple(slice(np.searchsorted(a, lo - 1e-9), np.searchsorted(a, hi + 1e-9))
                 for a, lo, hi in zip(axes, box[0::2], box[1::2]))


def box_coverage(arr, box):
    '''Fraction of grid points inside box that are good'''
    xs, ys, zs, good, _ = to_grid(arr)
    return good[box_slices((xs, ys, zs), box)].mean()


def center_solution(arr, box):
    '''Joint solution at the grid point nearest the box center'''
    xs, ys, zs, _, q = to_grid(arr)
    center = [(box[2 * a] + box[2 * a + 1]) / 2 for a in range(3)]
    return q[tuple(np.argmin(np.abs(ax - v)) for ax, v in zip((xs, ys, zs), center))]


def largest_box(good, min_cells=(2, 2, 3)):
    '''Most balanced all-good axis-aligned box: longest shortest side, then volume'''
    nx, ny, nz = good.shape
    P = np.zeros((nx + 1, ny + 1, nz + 1), int)
    P[1:, 1:, 1:] = good.astype(int).cumsum(0).cumsum(1).cumsum(2)
    za, zb = np.triu_indices(nz + 1, k=min_cells[2])
    best = (0, None)
    for x0 in range(nx):
        for x1 in range(x0 + min_cells[0], nx + 1):
            for y0 in range(ny):
                for y1 in range(y0 + min_cells[1], ny + 1):
                    s = (P[x1, y1, zb] - P[x0, y1, zb] - P[x1, y0, zb] + P[x0, y0, zb]
                         - P[x1, y1, za] + P[x0, y1, za] + P[x1, y0, za] - P[x0, y0, za])
                    vol = (x1 - x0) * (y1 - y0) * (zb - za)
                    full = s == vol
                    if not full.any():
                        continue
                    side = np.minimum(min(x1 - x0, y1 - y0), zb - za)
                    score = np.where(full, side * 100000 + vol, 0)
                    k = np.argmax(score)
                    if score[k] > best[0]:
                        best = (score[k], (x0, x1 - 1, y0, y1 - 1, za[k], zb[k] - 1))
    return best[1]


def best_box(arr, in_view=True):
    '''Largest balanced box above the floor, optionally inside the head camera view'''
    xs, ys, zs, good, _ = to_grid(arr)
    good[:, :, zs < cfg.Z_FLOOR - 1e-9] = False
    if in_view:
        P = np.stack(np.meshgrid(xs, ys, zs, indexing='ij'), -1).reshape(-1, 3)
        good &= project(camera_pose(), P)[3].reshape(good.shape)
    b = largest_box(good)
    if b is None:
        return None
    return tuple(float(v) for v in (xs[b[0]], xs[b[1]], ys[b[2]], ys[b[3]], zs[b[4]], zs[b[5]]))


@lru_cache(maxsize=None)
def camera_pose():
    '''Pelvis-frame pose of the head color optical frame at zero torso'''
    model = pin.buildModelFromUrdf(cfg.URDF)
    data = model.createData()
    pin.framesForwardKinematics(model, data, pin.neutral(model))
    M = data.oMf[model.getFrameId('head_camera_link')]
    for t, (x, y, z, w) in cfg.CAMERA_CHAIN:
        M = M * pin.SE3(pin.Quaternion(w, x, y, z).normalized().matrix(), np.array(t))
    return M


def project(M, pts, K=cfg.COLOR_K):
    '''Pixel u, v, depth and in-view flag of pelvis-frame points'''
    pc = (np.asarray(pts) - M.translation) @ M.rotation
    z = pc[:, 2]
    u = K['fx'] * pc[:, 0] / z + K['cx']
    v = K['fy'] * pc[:, 1] / z + K['cy']
    inside = (z > cfg.DEPTH_MIN) & (u >= 0) & (u < K['w']) & (v >= 0) & (v < K['h'])
    return u, v, z, inside


def camera_stats(box, n=9):
    '''In-view fraction and depth range of points sampled through the box'''
    g = np.stack(np.meshgrid(*[np.linspace(box[2 * a], box[2 * a + 1], n) for a in range(3)],
                             indexing='ij'), -1).reshape(-1, 3)
    _, _, z, inside = project(camera_pose(), g)
    return inside.mean(), z.min(), z.max()


def fov(K):
    '''Horizontal and vertical field of view in degrees'''
    return (np.rad2deg(2 * np.arctan(K['w'] / 2 / K['fx'])),
            np.rad2deg(2 * np.arctan(K['h'] / 2 / K['fy'])))


def frustum_corners(far):
    '''Camera origin and the four image-corner rays scaled to depth far'''
    M = camera_pose()
    K = cfg.COLOR_K
    corners = [M.translation + M.rotation @ (np.array([(u - K['cx']) / K['fx'],
                                                      (v - K['cy']) / K['fy'], 1.0]) * far)
               for u, v in [(0, 0), (K['w'], 0), (K['w'], K['h']), (0, K['h'])]]
    return M.translation, corners
