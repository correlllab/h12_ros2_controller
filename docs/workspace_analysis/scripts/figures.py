'''Static figures: robot with working box and camera cone, and projected views'''
import os
import argparse

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.colors import LinearSegmentedColormap
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import config as cfg
import analysis as an
from robot_mesh import RobotMesh, is_arm
from report import load_pose, fmt_range, fmt_rpy

ORANGE, AQUA, INK, MUTED = '#eb6834', '#1baf7a', '#0b0b0b', '#52514e'
CMAP = LinearSegmentedColormap.from_list('cov', ['#ffffff', '#cfe0f6', '#2a78d6', '#0d3b73'])
# poses that get a static robot figure
ROBOT_POSES = ('pitch90', 'pitch60', 'pitch45')
# pose that gets the projected-views figure
VIEWS_POSE = 'pitch90'


def shade(tris, base):
    '''Lambert shading per triangle'''
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    light = np.array([0.4, -0.3, 0.85]) / np.linalg.norm([0.4, -0.3, 0.85])
    return np.clip(np.asarray(base)[None, :] * (0.45 + 0.55 * np.abs(n @ light))[:, None], 0, 1)


def draw_box(ax, b, **kw):
    c = np.array([[x, y, z] for x in b[0:2] for y in b[2:4] for z in b[4:6]])
    for i in range(8):
        for j in range(i + 1, 8):
            if np.sum(c[i] != c[j]) == 1:
                ax.plot(*zip(c[i], c[j]), **kw)


def draw_frustum(ax, far=0.72):
    o, c = an.frustum_corners(far)
    sides = [[o, c[i], c[(i + 1) % 4]] for i in range(4)]
    ax.add_collection3d(Poly3DCollection(sides, facecolors=(0.106, 0.686, 0.478, 0.10),
                                         edgecolors='none', zorder=2))
    for i in range(4):
        ax.plot(*zip(o, c[i]), color=AQUA, lw=1.2, zorder=3)
        ax.plot(*zip(c[i], c[(i + 1) % 4]), color=AQUA, lw=1.2, zorder=3)


def title(pose, side, box, rpy):
    return (f'{side.capitalize()} Arm, {pose["label"]}    X {fmt_range(box[0], box[1])}    '
            f'Y {fmt_range(box[2], box[3])}    Z {fmt_range(box[4], box[5])}    RPY {fmt_rpy(rpy)}')


def robot_figure(pose, side, tris, out_dir):
    info = pose[side]
    fig = plt.figure(figsize=(12, 11), dpi=150)
    ax = fig.add_subplot(projection='3d', computed_zorder=False)
    fig.subplots_adjust(left=0.03, right=0.97, bottom=0.05, top=0.94)
    for name, t in tris:
        hl = is_arm(name, side)
        base = (0.50, 0.53, 0.60) if hl else (0.80, 0.80, 0.80)
        ax.add_collection3d(Poly3DCollection(t, facecolors=shade(t, base), edgecolors='none',
                                             alpha=1.0 if hl else 0.6, zorder=1))
    draw_frustum(ax)
    draw_box(ax, info['box'], color=ORANGE, lw=2.6, zorder=10)
    # equal metric scale with room for the camera cone
    ax.set_xlim(-0.2, 0.8)
    ax.set_ylim(-0.5, 0.5)
    ax.set_zlim(-0.45, 0.75)
    ax.set_box_aspect((1.0, 1.0, 1.2))
    ax.view_init(elev=20, azim=35 if side == 'left' else -35)
    for name, axis in zip('xyz', (ax.xaxis, ax.yaxis, ax.zaxis)):
        axis.set_label_text(f'{name} [m]')
        axis.pane.set_facecolor((0.97, 0.97, 0.97, 1))
        axis.pane.set_edgecolor((0.85, 0.85, 0.85, 1))
    fig.suptitle(title(pose, side, info['box'], info['rpy']), fontsize=14, weight='bold',
                 color=INK, y=0.97)
    path = os.path.join(out_dir, f'h12_{side}_{pose["key"]}_robot.png')
    fig.savefig(path, facecolor='white')
    plt.close(fig)
    print('wrote', path)


def views_figure(pose, side, tris, out_dir):
    '''Top, side and front views: share of the box range that is reachable'''
    info = pose[side]
    b = info['box']
    xs, ys, zs, good, _ = an.to_grid(info['arr'])
    sl = an.box_slices((xs, ys, zs), b)
    step = xs[1] - xs[0]
    extent = lambda a: (a[0] - step / 2, a[-1] + step / 2)
    panels = [
        (f'Top view (x-y), color over heights z {fmt_range(b[4], b[5])}',
         good[:, :, sl[2]].mean(2).T, xs, ys, (0, 1)),
        (f'Side view (x-z), color over widths y {fmt_range(b[2], b[3])}',
         good[:, sl[1], :].mean(1).T, xs, zs, (0, 2)),
        (f'Front view (y-z), color over depths x {fmt_range(b[0], b[1])}',
         good[sl[0], :, :].mean(0).T, ys, zs, (1, 2)),
    ]
    # every axis spans 0.95 m so all panels share one scale and size
    lims = {0: (-0.25, 0.70), 1: (-0.30, 0.65) if side == 'left' else (-0.65, 0.30),
            2: (-0.40, 0.55)}
    verts = np.concatenate([t.reshape(-1, 3) for _, t in tris])[::6]
    fig = plt.figure(figsize=(21, 8.6), dpi=130)
    gs = fig.add_gridspec(1, 3, wspace=0.16, left=0.04, right=0.99, top=0.86, bottom=0.27)
    for k, (name, img, ha, va, dims) in enumerate(panels):
        ax = fig.add_subplot(gs[0, k])
        ax.scatter(verts[:, dims[0]], verts[:, dims[1]], s=0.3, c='#cccccc', linewidths=0, zorder=1)
        im = ax.imshow(np.ma.masked_equal(img, 0), origin='lower', extent=(*extent(ha), *extent(va)),
                       cmap=CMAP, vmin=0, vmax=1, alpha=0.92, zorder=2, interpolation='nearest')
        r = [b[2 * d] for d in dims] + [b[2 * d + 1] for d in dims]
        ax.add_patch(Rectangle(r[:2], r[2] - r[0], r[3] - r[1], fill=False, ec=ORANGE, lw=2.6,
                               zorder=3))
        ax.set_title(name, fontsize=12, color=INK, loc='left', pad=10)
        ax.set_aspect('equal')
        ax.set_xlabel('xyz'[dims[0]] + ' [m]', fontsize=10)
        ax.set_ylabel('xyz'[dims[1]] + ' [m]', fontsize=10)
        ax.set_xlim(*lims[dims[0]])
        ax.set_ylim(*lims[dims[1]])
        ax.tick_params(labelsize=9, colors=MUTED)
        ax.grid(color='#e8e8e8', lw=0.6, zorder=0)
        for s in ax.spines.values():
            s.set_color('#cccccc')
    cb = fig.colorbar(im, cax=fig.add_axes([0.3, 0.12, 0.4, 0.03]), orientation='horizontal')
    cb.set_ticks([0, 0.25, 0.5, 0.75, 1.0])
    cb.ax.tick_params(labelsize=10)
    cb.set_label('Each 2D cell looks through the box along the hidden axis: color = fraction of '
                 'those points the gripper can reach in this orientation.\n'
                 '1.0 = reachable all the way through the box range;  '
                 'blank = not reachable;  gray = robot.', fontsize=11, labelpad=8)
    fig.suptitle(title(pose, side, b, info['rpy']), fontsize=15, weight='bold', color=INK, y=0.965)
    path = os.path.join(out_dir, f'h12_{side}_{pose["key"]}_views.png')
    fig.savefig(path, facecolor='white')
    plt.close(fig)
    print('wrote', path)


def main(args):
    rm = RobotMesh()
    for pose in cfg.POSES:
        if pose['key'] not in ROBOT_POSES and pose['key'] != VIEWS_POSE:
            continue
        load_pose(pose)
        for side in ('left', 'right'):
            tris = rm.posed(side, an.center_solution(pose[side]['arr'], pose[side]['box']))
            if pose['key'] in ROBOT_POSES:
                robot_figure(pose, side, tris, args.out)
            if pose['key'] == VIEWS_POSE:
                views_figure(pose, side, tris, args.out)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Render static workspace figures')
    parser.add_argument('--out', default=cfg.REPORT_DIR)
    main(parser.parse_args())
