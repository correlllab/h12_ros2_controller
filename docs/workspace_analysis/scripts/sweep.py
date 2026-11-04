'''Reachability sweep: grid IK, region growing, gap filling and right-arm mirroring'''
import os
import argparse
from multiprocessing import Pool

import numpy as np

import config as cfg
import kinematics as kin

NEIGHBORS = [(i, j, k) for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1)
             if (i, j, k) != (0, 0, 0)]


def grid(side, step):
    '''Grid axes in the pelvis frame; the right arm mirrors the y range'''
    xs, ys, zs = (np.round(np.arange(lo, hi + step / 2, step), 3) for lo, hi in cfg.GRID.values())
    if side == 'right':
        ys = -ys[::-1]
    return xs, ys, zs


def _good(R):
    return ((R[..., 3] > 0) & (R[..., 4] >= cfg.MARGIN) & (R[..., 5] == 0)
            & (R[..., 6] >= cfg.SMIN))


def _propagate(R, good, axes, frontier, side, rpy, pool, label):
    '''Retry non-good neighbors of the frontier, seeded from the good neighbors'''
    shape = good.shape
    while frontier:
        cand = {}
        for g in frontier:
            for o in NEIGHBORS:
                n = (g[0] + o[0], g[1] + o[1], g[2] + o[2])
                if all(0 <= n[a] < shape[a] for a in range(3)) and not good[n]:
                    cand.setdefault(n, []).append(R[g][7:])
        keys = list(cand)
        jobs = [(side, rpy, tuple(axes[a][n[a]] for a in range(3)), cand[n][:6]) for n in keys]
        frontier = []
        for n, r in zip(keys, pool.map(kin.try_seeds, jobs, chunksize=8)):
            if r is not None:
                R[n][3:] = r
                good[n] = True
                frontier.append(n)
        print(f'  {label} grow: {len(keys)} tried, {len(frontier)} added, good {good.sum()}',
              flush=True)


def _fill(R, good, axes, side, rpy, pool, label, n_random=24):
    '''Random-restart IK for non-good points surrounded by good ones'''
    shape = good.shape
    pad = np.pad(good, 1).astype(int)
    count = sum(pad[1 + i:1 + i + shape[0], 1 + j:1 + j + shape[1], 1 + k:1 + k + shape[2]]
                for i, j, k in NEIGHBORS)
    holes = list(zip(*np.nonzero(~good & (count >= 6))))
    c = kin.context(side)
    rng = np.random.default_rng(3)
    jobs = [(side, rpy, tuple(axes[a][n[a]] for a in range(3)),
             list(rng.uniform(c['lo'], c['hi'], (n_random, 7)))) for n in holes]
    added = []
    for n, r in zip(holes, pool.map(kin.try_seeds, jobs, chunksize=4)):
        if r is not None:
            R[n][3:] = r
            good[n] = True
            added.append(n)
    print(f'  {label} fill: {len(holes)} holes tried, {len(added)} filled, good {good.sum()}',
          flush=True)
    return added


def sweep(side, pitch, yaw, roll, kind, pool, step=cfg.STEP, fill_rounds=2):
    '''Sweep one orientation (degrees) and save the result; skip if it exists'''
    path = cfg.data_path(kind, side, pitch, yaw, roll)
    if os.path.exists(path):
        return path
    os.makedirs(os.path.dirname(path), exist_ok=True)
    label = os.path.basename(path)[:-4]
    rpy = tuple(np.deg2rad(v) for v in (pitch, yaw, roll))
    axes = grid(side, step)
    shape = tuple(len(a) for a in axes)
    # fixed seeds with warm starts along z, one job per x slice
    jobs = [(side, rpy, [(x, y, z) for y in axes[1] for z in axes[2]]) for x in axes[0]]
    R = np.concatenate(pool.map(kin.sweep_line, jobs, chunksize=1)).reshape(*shape, -1)
    good = _good(R)
    _propagate(R, good, axes, list(zip(*np.nonzero(good))), side, rpy, pool, label)
    for _ in range(fill_rounds):
        added = _fill(R, good, axes, side, rpy, pool, label)
        if not added:
            break
        _propagate(R, good, axes, added, side, rpy, pool, label)
    np.save(path, R.reshape(-1, R.shape[-1]))
    print(f'saved {path}', flush=True)
    return path


def mirror(pose, pool, n_random=40):
    '''Build the right-arm result from verified mirrored left solutions'''
    p, y, r = cfg.mirrored(pose)
    path = cfg.data_path('fine', 'right', p, y, r)
    if os.path.exists(path):
        return path
    L = np.load(cfg.data_path('fine', 'left', pose['pitch'], pose['yaw'], pose['roll']))
    rpy = tuple(np.deg2rad(v) for v in (p, y, r))
    R = L.copy()
    R[:, 1] *= -1
    R[:, 3:] = kin.row((0, 0, 0))[3:]
    idx = np.nonzero(kin.good_mask(L))[0]
    jobs = [('right', rpy, tuple(R[i, :3]), cfg.MIRROR * L[i, 7:]) for i in idx]
    failed = []
    for i, res in zip(idx, pool.map(kin.check_mirror, jobs, chunksize=16)):
        if res is None:
            failed.append(i)
        else:
            R[i, 3:] = res
    # the few mirrored solutions that fail are re-solved on the right arm
    c = kin.context('right')
    rng = np.random.default_rng(5)
    seeds = [cfg.MIRROR * np.array(s) for s in cfg.SEEDS]
    jobs = [('right', rpy, tuple(R[i, :3]),
             [cfg.MIRROR * L[i, 7:]] + seeds + list(rng.uniform(c['lo'], c['hi'], (n_random, 7))))
            for i in failed]
    solved = 0
    for i, res in zip(failed, pool.map(kin.try_seeds, jobs, chunksize=1)):
        if res is not None:
            R[i, 3:] = res
            solved += 1
    os.makedirs(os.path.dirname(path), exist_ok=True)
    np.save(path, R)
    print(f'saved {path}: {len(idx) - len(failed)} mirrored, {solved} of {len(failed)} re-solved',
          flush=True)
    return path


def main(args):
    poses = [p for p in cfg.POSES if args.pose in (None, p['key'])]
    with Pool(args.workers, maxtasksperchild=500) as pool:
        for pose in poses:
            sweep('left', pose['pitch'], pose['yaw'], pose['roll'], 'fine', pool)
            mirror(pose, pool)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Sweep reachable grasp poses for each orientation')
    parser.add_argument('--pose', choices=[p['key'] for p in cfg.POSES], default=None)
    parser.add_argument('--workers', type=int, default=12)
    main(parser.parse_args())
