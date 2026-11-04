'''Self-contained interactive HTML report of the gripper workspaces'''
import os
import json
import argparse

import numpy as np
import plotly.graph_objects as go
from plotly.offline import get_plotlyjs

import config as cfg
import analysis as an
import kinematics as kin
from robot_mesh import RobotMesh, is_arm

ORANGE, AQUA = '#eb6834', '#1baf7a'
BLUES = [[0.0, '#dbe8f8'], [0.35, '#8db8ec'], [0.7, '#2a78d6'], [1.0, '#0d3b73']]
MINUS = '−'
HTML_NAME = 'h12_workspace_analysis.html'


def fmt(v, nd=3):
    s = f'{v:.{nd}f}'.rstrip('0').rstrip('.') if nd else f'{v:.0f}'
    return ('0' if s in ('-0', '') else s).replace('-', MINUS)


def fmt_range(a, b):
    return f'{fmt(a)} – {fmt(b)}'


def fmt_rpy(rpy):
    return ' '.join(fmt(v, 0) for v in rpy)


def load_pose(pose):
    '''Attach left/right results, rpy and boxes (pinned or searched) to a pose'''
    if 'left' in pose:
        return pose
    for k, side in enumerate(('left', 'right')):
        p, y, r = (pose['pitch'], pose['yaw'], pose['roll']) if side == 'left' else cfg.mirrored(pose)
        arr = np.load(cfg.data_path('fine', side, p, y, r))
        box = pose['box'][k] if pose.get('box') else an.best_box(arr)
        pose[side] = dict(arr=arr, rpy=(r, p, y), box=box)
        print(pose['key'], side, 'box', box, f'good {an.box_coverage(arr, box):.0%}')
    return pose


def mesh_trace(tris, color, name):
    v = np.concatenate(tris).reshape(-1, 3)
    _, first, inv = np.unique(np.round(v / 1e-4).astype(np.int64), axis=0,
                              return_index=True, return_inverse=True)
    v, f = v[first], inv.reshape(-1, 3)
    return go.Mesh3d(x=v[:, 0].round(4), y=v[:, 1].round(4), z=v[:, 2].round(4),
                     i=f[:, 0], j=f[:, 1], k=f[:, 2], color=color, flatshading=True, name=name,
                     showlegend=True, legendgroup='robot', hoverinfo='skip',
                     lighting=dict(ambient=0.75, diffuse=0.55, specular=0.05, roughness=0.9,
                                   fresnel=0.1),
                     lightposition=dict(x=2, y=-1, z=3))


def lines(segments):
    xs, ys, zs = [], [], []
    for a, b in segments:
        xs += [a[0], b[0], None]
        ys += [a[1], b[1], None]
        zs += [a[2], b[2], None]
    return xs, ys, zs


def box_trace(b):
    c = [(x, y, z) for x in b[0:2] for y in b[2:4] for z in b[4:6]]
    seg = [(c[i], c[j]) for i in range(8) for j in range(i + 1, 8)
           if sum(p != q for p, q in zip(c[i], c[j])) == 1]
    x, y, z = lines(seg)
    return go.Scatter3d(x=x, y=y, z=z, mode='lines', name='Working box',
                        line=dict(color=ORANGE, width=11), hoverinfo='skip')


def frustum_traces(far=1.0):
    o, c = an.frustum_corners(far)
    v = np.array([o] + c)
    cone = go.Mesh3d(x=v[:, 0], y=v[:, 1], z=v[:, 2], i=[0, 0, 0, 0], j=[1, 2, 3, 4],
                     k=[2, 3, 4, 1], color=AQUA, opacity=0.12, name='Head camera view',
                     legendgroup='cam', showlegend=True, hoverinfo='skip', flatshading=True)
    x, y, z = lines([(o, p) for p in c] + [(c[i], c[(i + 1) % 4]) for i in range(4)])
    edges = go.Scatter3d(x=x, y=y, z=z, mode='lines', line=dict(color=AQUA, width=3),
                         legendgroup='cam', showlegend=False, hoverinfo='skip')
    return [cone, edges]


def arm_figure(side, info, rm):
    arr, b = info['arr'], info['box']
    ok = kin.good_mask(arr)
    pts, clearance = arr[ok, :3], arr[ok, 4].round(3)
    tris = rm.posed(side, an.center_solution(arr, b), cell=0.012)
    inside = np.all([(pts[:, a] >= b[2 * a] - 1e-9) & (pts[:, a] <= b[2 * a + 1] + 1e-9)
                     for a in range(3)], axis=0)
    hover = ('x %{x:.3f}  y %{y:.3f}  z %{z:.3f}<br>'
             'joint-limit clearance %{marker.color:.2f} rad<extra></extra>')
    marker = dict(colorscale=BLUES, cmin=0.05, cmax=0.4, line=dict(width=0))
    fig = go.Figure([
        mesh_trace([t for n, t in tris if not is_arm(n, side)], '#d4d6da', 'Robot body'),
        mesh_trace([t for n, t in tris if is_arm(n, side)], '#7d8596', f'{side.capitalize()} arm'),
        go.Scatter3d(x=pts[:, 0], y=pts[:, 1], z=pts[:, 2], mode='markers', name='Reachable points',
                     hovertemplate=hover,
                     marker=dict(size=3.2, color=clearance, opacity=0.2, **marker,
                                 colorbar=dict(title=dict(text='clearance<br>[rad]', side='top'),
                                               thickness=14, len=0.55, x=1.0,
                                               tickvals=[0.05, 0.1, 0.2, 0.3, 0.4],
                                               ticktext=['0.05', '0.1', '0.2', '0.3', '≥0.4']))),
        go.Scatter3d(x=pts[inside, 0], y=pts[inside, 1], z=pts[inside, 2], mode='markers',
                     name='Points inside box', hovertemplate=hover,
                     marker=dict(size=3.6, color=clearance[inside], opacity=0.95, **marker)),
        box_trace(b),
        *frustum_traces(),
    ])
    s = 1 if side == 'left' else -1
    axis = dict(backgroundcolor='#f6f6f4', gridcolor='#e2e2df')
    fig.update_layout(
        height=720, margin=dict(l=0, r=0, t=6, b=0), paper_bgcolor='rgba(0,0,0,0)',
        font=dict(family='Inter, system-ui, sans-serif', size=13, color='#2b2b2b'),
        legend=dict(x=0.01, y=0.99, bgcolor='rgba(255,255,255,0.75)', itemsizing='constant',
                    font=dict(size=12)),
        scene=dict(
            # equal metric scale on x, y, z (ranges 1.05, 1.6, 1.2 m)
            aspectmode='manual', aspectratio=dict(x=0.656, y=1.0, z=0.75),
            xaxis=dict(title='x [m]', range=[-0.25, 0.80], **axis),
            yaxis=dict(title='y [m]', range=[-0.80, 0.80], **axis),
            zaxis=dict(title='z [m]', range=[-0.45, 0.75], **axis),
            camera=dict(eye=dict(x=1.15, y=0.8 * s, z=0.42),
                        center=dict(x=0.03, y=0.08 * s, z=-0.08)),
        ),
    )
    return fig


def yaw_rows(pose):
    '''Coverage and clearance of each scanned yaw inside the chosen left box'''
    b = pose['left']['box']
    rows = []
    for yaw in cfg.YAWS:
        path = cfg.data_path('yaw', 'left', pose['pitch'], yaw, pose['roll'])
        if yaw == pose['yaw']:
            path = cfg.data_path('fine', 'left', pose['pitch'], yaw, pose['roll'])
        if not os.path.exists(path):
            continue
        arr = np.load(path)
        xs, ys, zs, good, _ = an.to_grid(arr)
        sl = an.box_slices((xs, ys, zs), b)
        g = good[sl]
        c = arr[:, 4].reshape(good.shape)[sl][g] if g.any() else []
        cls = ' class="pick"' if yaw == pose['yaw'] else ''
        rows.append(f'<tr{cls}><td>{fmt(yaw, 0)}</td>'
                    f'<td><span class="bar" style="width:{g.mean() * 120:.0f}px"></span>{g.mean():.0%}</td>'
                    f'<td>{np.min(c):.3f}</td><td>{np.median(c):.3f}</td></tr>' if g.any() else
                    f'<tr{cls}><td>{fmt(yaw, 0)}</td><td>0%</td><td>–</td><td>–</td></tr>')
    return '\n'.join(rows)


def roll_rows(pose):
    '''Largest balanced box per scanned roll on the coarse grid'''
    rows = []
    for roll in cfg.ROLLS:
        path = cfg.data_path('scan', 'left', pose['pitch'], pose['yaw'], roll)
        if not os.path.exists(path):
            continue
        b = an.best_box(np.load(path), in_view=False)
        size = '–' if b is None else ' × '.join(f'{(b[2 * a + 1] - b[2 * a]) * 100:.0f}' for a in range(3))
        vol = 0 if b is None else np.prod([b[2 * a + 1] - b[2 * a] for a in range(3)])
        cls = ' class="pick"' if roll == pose['roll'] else ''
        rows.append(f'<tr{cls}><td>{fmt(roll, 0)}</td><td>{size} cm</td>'
                    f'<td><span class="bar" style="width:{vol / 0.02 * 120:.0f}px"></span>'
                    f'{vol * 1000:.1f} L</td></tr>')
    return '\n'.join(rows)


def free_angle_html(pose):
    note = f'<p class="note">{pose["note"]}</p>' if 'note' in pose else ''
    if pose['free'] == 'yaw':
        rows = yaw_rows(pose)
        table = (f'''<div class="wide"><table>
<tr><th>Yaw [deg]</th><th>Box points reachable</th><th>Min clearance [rad]</th><th>Median clearance [rad]</th></tr>
{rows}
</table></div>''' if rows else '')
        return f'''<details><summary>Why yaw {fmt(pose['yaw'], 0)}°</summary>
<p class="note">With the gripper vertical, roll and yaw both spin it about the vertical axis, so
yaw is the only free angle. Each tested yaw is checked inside the chosen left-arm box (the right
arm mirrors it). Clearance is the distance from the nearest joint limit.</p>
{table}
<p class="note">−60° covers the whole box with the most clearance; −75° also covers it with less
margin. Yaw 0 almost fails. The wrist pitch joint (±0.46 rad) is always the first to run out,
so margin is smallest near the box edges. If the fingers are interchangeable, −60° and +120°
give the same grasp.</p>{note}</details>'''
    rows = roll_rows(pose)
    table = (f'''<div class="wide"><table>
<tr><th>Roll [deg]</th><th>Largest box (5 cm scan, any view)</th><th>Volume</th></tr>
{rows}
</table></div>''' if rows else '')
    return f'''<details><summary>Why roll {fmt(pose['roll'], 0)}°</summary>
<p class="note">With the gripper tilted toward +x (yaw 0), the free angle is roll, the spin of
the gripper about its own pointing axis. Each roll was scanned on a 5 cm grid for the largest
box where every point is reachable. The top two were refined on the 2.5 cm grid, and the one
with the longer shortest side won.</p>{note}
{table}</details>'''


def pose_section(pose):
    rows = ''
    for side in ('left', 'right'):
        info = pose[side]
        b = info['box']
        cov, d0, d1 = an.camera_stats(b)
        rows += (f'<tr><td>{side.capitalize()}</td><td>{fmt_range(b[0], b[1])}</td>'
                 f'<td>{fmt_range(b[2], b[3])}</td><td>{fmt_range(b[4], b[5])}</td>'
                 f'<td>{fmt_rpy(info["rpy"])}</td><td>{cov:.0%} at {d0:.2f} – {d1:.2f} m</td></tr>')
    return f'''<h2 id="{pose['key']}">{pose['title']}</h2>
<div class="wide"><table>
<tr><th>Arm</th><th>X [m]</th><th>Y [m]</th><th>Z [m]</th><th>RPY [deg]</th><th>Box in head camera view</th></tr>
{rows}
</table></div>
{free_angle_html(pose)}
<div class="plots">
<div><h3>Left arm</h3><div id="{pose['key']}_left" class="plot"></div></div>
<div><h3>Right arm</h3><div id="{pose['key']}_right" class="plot"></div></div>
</div>'''


def summary_table(poses):
    rows = ''
    for pose in poses:
        for side in ('left', 'right'):
            b, r = pose[side]['box'], pose[side]['rpy']
            name = f'<a href="#{pose["key"]}">{pose["title"]}</a>' if side == 'left' else ''
            rows += (f'<tr><td>{name}</td><td>{side.capitalize()}</td><td>{fmt_range(b[0], b[1])}</td>'
                     f'<td>{fmt_range(b[2], b[3])}</td><td>{fmt_range(b[4], b[5])}</td>'
                     f'<td>{fmt_rpy(r)}</td></tr>')
    return f'''<div class="wide"><table>
<tr><th>Pose</th><th>Arm</th><th>X [m]</th><th>Y [m]</th><th>Z [m]</th><th>RPY [deg]</th></tr>
{rows}</table></div>'''


CSS = '''
:root { --bg: #fcfcfb; --card: #ffffff; --ink: #0b0b0b; --ink2: #52514e; --line: #e6e5e1;
  --orange: #eb6834; --blue: #2a78d6; }
* { box-sizing: border-box; }
body { margin: 0; background: var(--bg); color: var(--ink);
  font: 15px/1.55 Inter, system-ui, -apple-system, Segoe UI, sans-serif; }
main { max-width: 1320px; margin: 0 auto; padding: 32px 16px 64px; }
h1 { font-size: 28px; margin: 0 0 6px; }
h2 { font-size: 22px; margin: 48px 0 8px; padding-top: 8px; border-top: 1px solid var(--line); }
h3 { font-size: 15px; margin: 0 0 4px; color: var(--ink2); font-weight: 600; }
p.lead { color: var(--ink2); margin: 0 0 22px; max-width: 860px; }
table { border-collapse: collapse; background: var(--card); border: 1px solid var(--line);
  border-radius: 8px; overflow: hidden; }
th, td { text-align: left; padding: 8px 14px; border-bottom: 1px solid var(--line);
  font-variant-numeric: tabular-nums; white-space: nowrap; }
th { font-size: 13px; color: var(--ink2); font-weight: 600; background: #f6f6f4; white-space: normal; }
tr:last-child td { border-bottom: 0; }
.pick td { background: #fdf0ea; font-weight: 600; }
.bar { display: inline-block; height: 9px; border-radius: 2px; background: var(--blue);
  vertical-align: 0; margin-right: 8px; }
.card { background: var(--card); border: 1px solid var(--line); border-radius: 10px;
  padding: 14px 16px; margin-top: 12px; }
.note { color: var(--ink2); font-size: 14px; max-width: 920px; }
.key { display: flex; flex-wrap: wrap; gap: 18px; margin: 6px 0 0; color: var(--ink2); font-size: 14px; }
.sw { display: inline-block; width: 12px; height: 12px; border-radius: 50%; vertical-align: -1px;
  margin-right: 6px; }
.ramp { display: inline-block; width: 90px; height: 10px; border-radius: 3px; vertical-align: 0;
  margin: 0 6px; background: linear-gradient(90deg, #dbe8f8, #8db8ec, #2a78d6, #0d3b73); }
.wide { overflow-x: auto; }
.plots { display: grid; grid-template-columns: 1fr; gap: 14px; margin-top: 14px; }
.plots > div { background: var(--card); border: 1px solid var(--line); border-radius: 10px;
  padding: 10px 10px 0; }
.plot { width: 100%; height: 720px; }
details { margin-top: 10px; }
summary { cursor: pointer; color: var(--ink); font-weight: 600; font-size: 14px; }
nav a { color: var(--blue); text-decoration: none; margin-right: 16px; font-size: 14px; }
@media (max-width: 760px) { .plot { height: 520px; } }
'''


def camera_json(poses):
    '''Camera pose, intrinsics and box coverage for downstream use'''
    M = an.camera_pose()
    out = dict(frame='pelvis', torso=0.0, position=M.translation.round(4).tolist(),
               rotation=M.rotation.round(5).tolist(), color=cfg.COLOR_K, depth=cfg.DEPTH_K,
               depth_min=cfg.DEPTH_MIN, color_fov_deg=[round(v, 1) for v in an.fov(cfg.COLOR_K)],
               depth_fov_deg=[round(v, 1) for v in an.fov(cfg.DEPTH_K)], boxes={})
    for pose in poses:
        for side in ('left', 'right'):
            cov, d0, d1 = an.camera_stats(pose[side]['box'])
            out['boxes'][f'{pose["key"]}_{side}'] = dict(
                box=[round(v, 3) for v in pose[side]['box']], rpy_deg=list(pose[side]['rpy']),
                in_view=round(float(cov), 3), depth=[round(float(d0), 3), round(float(d1), 3)])
    return out


def main(args):
    poses = [load_pose(p) for p in cfg.POSES]
    rm = RobotMesh()
    figs = {f'{p["key"]}_{s}': json.loads(arm_figure(s, p[s], rm).to_json())
            for p in poses for s in ('left', 'right')}
    hfov, vfov = an.fov(cfg.COLOR_K)
    M = an.camera_pose()
    pos = ', '.join(f'{v:.2f}' for v in M.translation)
    tilt = np.rad2deg(np.arcsin(-M.rotation[2, 2]))
    body = f'''<main>
<h1>H12 Gripper Workspace</h1>
<p class="lead">Where each gripper can work for {len(poses)} fixed gripper orientations, for
pick-and-place and peg-in-hole tasks. Positions are the grasp frame (fingertip point) in the pelvis
frame, with the torso at 0. Each box is the most balanced box in which every point is reachable and
inside the head camera view, with z ≥ {fmt(cfg.Z_FLOOR)} and no other x/y limits. Right boxes are a
bit smaller than left ones because the head camera sits slightly to the left.</p>
<nav>{''.join(f'<a href="#{p["key"]}">{p["title"]}</a>' for p in poses)}</nav>
<h2 style="border:0;margin-top:20px">Summary</h2>
{summary_table(poses)}
<div class="card">
<p class="note" style="margin:0 0 8px"><b>Reading the 3D views.</b> Each sphere is a point on a
{cfg.STEP * 100:.1f} cm grid where the gripper can hold that orientation: the inverse kinematics
solves exactly, every joint stays inside its limits, and the arm doesn't hit the body or stretch into
a near-singular pose. The shade shows its clearance, how much room is left before the nearest joint
limit: <span class="ramp"></span>pale = just inside a limit ({cfg.MARGIN} rad), dark = comfortable
(0.4 rad or more). Empty space is unreachable with that orientation. Drag to rotate, scroll to zoom,
and click legend items to hide them.</p>
<p class="note" style="margin:0 0 8px"><b>Head camera.</b> The green pyramid is the color view of
the head RealSense D435i ({hfov:.0f}° wide, {vfov:.0f}° tall), from the hand-to-eye calibrated mount
in GOLEM (<code>h1_robocasa/realsense_profile.yaml</code>). It sits at ({pos}) m, looks forward and
down about {tilt:.0f}°, and is drawn 1 m long.</p>
<div class="key"><span><span class="sw" style="background:var(--orange)"></span>Working box</span>
<span><span class="sw" style="background:#7d8596"></span>Working arm, posed at box center</span>
<span><span class="sw" style="background:#d4d6da"></span>Rest of robot, idle arm at zero</span>
<span><span class="sw" style="background:#1baf7a"></span>Head camera view</span></div>
</div>
{''.join(pose_section(p) for p in poses)}
</main>'''
    html = f'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>H12 Workspace Analysis</title>
<style>{CSS}</style>
<script>{get_plotlyjs()}</script>
</head>
<body>
{body}
<script>
const figs = {json.dumps(figs, separators=(',', ':'))};
const cfg = {{responsive: true, displaylogo: false}};
for (const [id, f] of Object.entries(figs)) Plotly.newPlot(id, f.data, f.layout, cfg);
</script>
</body>
</html>
'''
    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, HTML_NAME)
    with open(path, 'w') as f:
        f.write(html)
    print('wrote', path, f'{os.path.getsize(path) / 1e6:.1f} MB')
    path = os.path.join(args.out, 'head_camera.json')
    with open(path, 'w') as f:
        json.dump(camera_json(poses), f, indent=1)
    print('wrote', path)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Build the interactive workspace report')
    parser.add_argument('--out', default=cfg.REPORT_DIR)
    main(parser.parse_args())
