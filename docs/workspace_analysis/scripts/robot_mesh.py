'''Posed robot mesh triangles from the URDF visual meshes'''
import struct

import numpy as np
import pinocchio as pin

import config as cfg

_STL = {}


def load_stl(path):
    '''Read a binary or ascii stl into (n, 3, 3) triangles'''
    if path not in _STL:
        with open(path, 'rb') as f:
            raw = f.read()
        n = struct.unpack('<I', raw[80:84])[0]
        if len(raw) == 84 + 50 * n:
            dt = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
            tris = np.frombuffer(raw, dtype=dt, count=n, offset=84)['v'].astype(float)
        else:
            verts = [list(map(float, l.split()[1:4])) for l in raw.decode().splitlines()
                     if l.strip().startswith('vertex')]
            tris = np.array(verts).reshape(-1, 3, 3)
        _STL[path] = tris
    return _STL[path]


def decimate(tris, cell):
    '''Vertex-clustering decimation on a cubic cell grid'''
    pts = tris.reshape(-1, 3)
    _, inv = np.unique(np.round(pts / cell).astype(np.int64), axis=0, return_inverse=True)
    inv = inv.ravel()
    centers = np.zeros((inv.max() + 1, 3))
    np.add.at(centers, inv, pts)
    centers /= np.bincount(inv)[:, None]
    faces = inv.reshape(-1, 3)
    keep = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    faces = faces[keep]
    _, first = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)
    return centers[faces[np.sort(first)]]


def is_arm(name, side):
    '''True for the working arm links and its gripper'''
    arm_link = name.startswith(side + '_') and any(k in name for k in ('shoulder', 'elbow', 'wrist'))
    return arm_link or name.startswith(side[0] + 'g_')


class RobotMesh:
    def __init__(self):
        self.model, _, self.visual = pin.buildModelsFromUrdf(cfg.URDF, package_dirs=cfg.ASSETS)
        self.data = self.model.createData()
        self.vdata = pin.GeometryData(self.visual)

    def configuration(self, arms):
        '''Full q with the given arm joints from {side: q7}, everything else zero'''
        q = pin.neutral(self.model)
        for side, qa in arms.items():
            for name, v in zip(cfg.ARM_JOINTS, qa):
                q[self.model.joints[self.model.getJointId(f'{side}_{name}_joint')].idx_q] = v
        return q

    def triangles(self, q, cell=0.008, skip=('ankle', 'knee')):
        '''List of (geometry name, pelvis-frame triangles) at q'''
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateGeometryPlacements(self.model, self.data, self.visual, self.vdata)
        out = []
        for g, M in zip(self.visual.geometryObjects, self.vdata.oMg):
            if not g.meshPath or any(s in g.name for s in skip):
                continue
            tris = decimate(load_stl(g.meshPath) * np.asarray(g.meshScale), cell)
            out.append((g.name, (tris.reshape(-1, 3) @ M.rotation.T + M.translation).reshape(-1, 3, 3)))
        return out

    def posed(self, side, q_arm, cell=0.008):
        '''Triangles with the working arm at q_arm and the idle arm at zero'''
        other = 'right' if side == 'left' else 'left'
        return self.triangles(self.configuration({side: q_arm, other: np.zeros(7)}), cell)
