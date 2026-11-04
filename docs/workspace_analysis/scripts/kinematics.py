'''Single-arm kinematics: model, IK and solution quality'''
import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares

import config as cfg

_CONTEXT = {}


def build(side):
    '''Build a 7-dof arm model with everything else locked at zero, plus sphere collision'''
    model, _, _ = pin.buildModelsFromUrdf(cfg.URDF, package_dirs=cfg.ASSETS)
    keep = [f'{side}_{j}_joint' for j in cfg.ARM_JOINTS]
    lock = [i for i in range(1, model.njoints) if model.names[i] not in keep]
    arm = pin.buildReducedModel(model, lock, pin.neutral(model))
    cmodel, cgeom, _ = pin.buildModelsFromUrdf(cfg.URDF_SPHERE, package_dirs=cfg.ASSETS)
    lock = [i for i in range(1, cmodel.njoints) if cmodel.names[i] not in keep]
    cmodel, cgeom = pin.buildReducedModel(cmodel, cgeom, lock, pin.neutral(cmodel))
    cgeom.addAllCollisionPairs()
    pin.removeCollisionPairs(cmodel, cgeom, cfg.SRDF_SPHERE)
    return arm, cmodel, cgeom


def context(side):
    '''Cached per-process models, limits and a quick reach bound'''
    if side in _CONTEXT:
        return _CONTEXT[side]
    model, cmodel, cgeom = build(side)
    data = model.createData()
    fid = model.getFrameId(f'{side}_grasp_frame')
    wid = model.getFrameId(f'{side}_wrist_yaw_link')
    sid = model.getFrameId(f'{side}_shoulder_pitch_link')
    lo = np.array([l for l, _ in cfg.CLIP[side]])
    hi = np.array([h for _, h in cfg.CLIP[side]])
    # wrist reach from the shoulder over random samples, used to skip far points
    reach = 0.0
    for q in np.random.default_rng(0).uniform(lo, hi, (4000, 7)):
        pin.framesForwardKinematics(model, data, q)
        reach = max(reach, np.linalg.norm(data.oMf[wid].translation - data.oMf[sid].translation))
    _CONTEXT[side] = dict(
        model=model, data=data, fid=fid, lo=lo, hi=hi, reach=reach + 0.01,
        cmodel=cmodel, cdata=cmodel.createData(), cgeom=cgeom, gdata=pin.GeometryData(cgeom),
        shoulder=data.oMf[sid].translation.copy(),
        offset=data.oMf[wid].actInv(data.oMf[fid]).translation.copy(),
    )
    return _CONTEXT[side]


def rotation(pitch, yaw, roll=0.0):
    '''Grasp-frame rotation from rpy in radians'''
    return pin.rpy.rpyToMatrix(roll, pitch, yaw)


def solve(c, target, q0, nominal, iters=60):
    '''Bounded least squares on the 6d pose error with a small posture term'''
    model, data, fid = c['model'], c['data'], c['fid']
    w_reg = 1e-3

    def residual(q):
        pin.framesForwardKinematics(model, data, q)
        err = pin.log6(target.actInv(data.oMf[fid])).vector
        return np.concatenate([err, w_reg * (q - nominal)])

    def jacobian(q):
        pin.computeJointJacobians(model, data, q)
        pin.framesForwardKinematics(model, data, q)
        J = pin.Jlog6(target.actInv(data.oMf[fid])) @ pin.getFrameJacobian(model, data, fid, pin.LOCAL)
        return np.vstack([J, w_reg * np.eye(7)])

    q0 = np.clip(q0, c['lo'] + 1e-6, c['hi'] - 1e-6)
    q = least_squares(residual, q0, jac=jacobian, bounds=(c['lo'], c['hi']), method='trf',
                      max_nfev=iters, xtol=1e-10, ftol=1e-10, gtol=1e-10).x
    return q, reaches(c, target, q)


def reaches(c, target, q):
    '''True when the grasp frame at q matches target within 1 mm and 5 mrad'''
    pin.framesForwardKinematics(c['model'], c['data'], q)
    err = pin.log6(target.actInv(c['data'].oMf[c['fid']])).vector
    return np.linalg.norm(err[:3]) < 1e-3 and np.linalg.norm(err[3:]) < 5e-3


def quality(c, q):
    '''Return (clearance, collision, min translational singular value) of q'''
    clearance = np.min(np.minimum(q - c['lo'], c['hi'] - q))
    pin.updateGeometryPlacements(c['cmodel'], c['cdata'], c['cgeom'], c['gdata'], q)
    collision = pin.computeCollisions(c['cgeom'], c['gdata'], True)
    pin.framesForwardKinematics(c['model'], c['data'], q)
    J = pin.computeFrameJacobian(c['model'], c['data'], q, c['fid'], pin.LOCAL_WORLD_ALIGNED)
    return clearance, collision, np.linalg.svd(J[:3], compute_uv=False)[-1]


def is_good(clearance, collision, smin):
    return clearance >= cfg.MARGIN and not collision and smin >= cfg.SMIN


def row(p, q=None, quality_=None):
    '''Result row: x y z ok clearance collision smin q1..q7'''
    if q is None:
        return [*p, 0, -1, 1, 0] + [np.nan] * 7
    return [*p, 1, quality_[0], int(quality_[1]), quality_[2]] + list(q)


def good_mask(arr):
    '''Boolean mask of good rows in a result array'''
    return ((arr[:, 3] > 0) & (arr[:, 4] >= cfg.MARGIN) & (arr[:, 5] == 0)
            & (arr[:, 6] >= cfg.SMIN))


def sweep_line(args):
    '''Solve a list of points with fixed seeds and warm starts along the list'''
    side, rpy, points = args
    c = context(side)
    R = rotation(*rpy)
    sign = np.ones(7) if side == 'left' else cfg.MIRROR
    seeds = [sign * np.array(s) for s in cfg.SEEDS]
    seeds += list(np.random.default_rng(7).uniform(c['lo'], c['hi'], (8, 7)))
    rows, prev = [], None
    for p in points:
        target = pin.SE3(R, np.asarray(p))
        wrist = np.asarray(p) - R @ c['offset']
        best = None
        if np.linalg.norm(wrist - c['shoulder']) <= c['reach']:
            for s in ([prev] if prev is not None else []) + seeds:
                q, ok = solve(c, target, s, cfg.NOMINAL[side])
                if not ok:
                    continue
                qa = quality(c, q)
                score = (not qa[1], qa[0] >= cfg.MARGIN, qa[0] + 0.5 * qa[2])
                if best is None or score > best[0]:
                    best = (score, q, qa)
                if is_good(*qa):
                    break
        if best is None:
            rows.append(row(p))
            prev = None
        else:
            rows.append(row(p, best[1], best[2]))
            prev = best[1]
    return np.array(rows)


def try_seeds(args):
    '''Return the first good result row at p from the given seeds, else None'''
    side, rpy, p, seeds = args
    c = context(side)
    target = pin.SE3(rotation(*rpy), np.asarray(p))
    for s in seeds:
        q, ok = solve(c, target, s, cfg.NOMINAL[side])
        if ok:
            qa = quality(c, q)
            if is_good(*qa):
                return np.array(row(p, q, qa)[3:])
    return None


def check_mirror(args):
    '''Accept a mirrored left solution on the right arm if it reaches and is good'''
    side, rpy, p, q = args
    c = context(side)
    q = np.clip(q, c['lo'], c['hi'])
    if not reaches(c, pin.SE3(rotation(*rpy), np.asarray(p)), q):
        return None
    qa = quality(c, q)
    return np.array(row(p, q, qa)[3:]) if is_good(*qa) else None
