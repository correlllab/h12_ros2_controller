'''Export and independently verify workspace training targets and an offline report'''
import json
import hashlib
from pathlib import Path
from itertools import product

import yaml
import numpy as np
import pinocchio as pin
import plotly.graph_objects as go
from plotly.offline import get_plotlyjs

import config as cfg
import analysis as an
import kinematics as kin
from report import mesh_trace, frustum_traces
from robot_mesh import RobotMesh, is_arm

ROOT = Path(cfg.REPO_DIR)
OUT = Path(cfg.REPORT_DIR) / 'arm_targets'
REPORT = Path(cfg.REPORT_DIR) / 'h12_workspace_targets.html'
NAMES = {'pitch90': 'pitch_90', 'pitch60': 'pitch_60',
         'pitch45': 'pitch_45', 'pitch0': 'horizontal'}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def select(points, box):
    '''Select nearest feasible corners, face centers, center and ten maximin points'''
    bounds = np.array(box).reshape(3, 2)
    unit = (points - bounds[:, 0]) / np.diff(bounds, axis=1).ravel()
    anchors = [(f'corner_{i:02}', p) for i, p in enumerate(product((0, 1), repeat=3), 1)]
    for axis, name in enumerate('xyz'):
        for value, end in ((0, 'min'), (1, 'max')):
            p = np.full(3, 0.5)
            p[axis] = value
            anchors.append((f'face_{name}_{end}', p))
    anchors.append(('center', (0.5, 0.5, 0.5)))
    chosen, labels = [], []
    for label, anchor in anchors:
        distance = np.linalg.norm(unit - anchor, axis=1)
        distance[chosen] = np.inf
        chosen.append(int(np.argmin(distance)))
        labels.append(label)
    for i in range(10):
        distance = np.min(np.linalg.norm(unit[:, None] - unit[chosen], axis=2), axis=1)
        distance[chosen] = -np.inf
        chosen.append(int(np.argmax(distance)))
        labels.append(f'coverage_{i + 1:02}')
    radius = np.min(np.linalg.norm(unit[:, None] - unit[chosen], axis=2), axis=1).max()
    return chosen, labels, float(radius)


def export(pose):
    '''Recheck source solutions against current models before writing a catalog'''
    name = f'arm_{NAMES[pose["key"]]}_targets'
    catalog = dict(
        schema_version=1, description=f'{pose["title"]}: 25 targets per arm, 50 total',
        role='policy_fine_tuning', generated_model='magpie',
        reference_frame='pelvis', position_units='meters', joint_units='radians',
        orientation_convention='R = Rz(yaw) Ry(pitch) Rx(roll); grasp-local to pelvis; degrees',
        joint_names_order14=[f'{side}_{joint}_joint' for side in ('left', 'right')
                             for joint in cfg.ARM_JOINTS],
        software_joint_bounds_rad={side: [list(pair) for pair in cfg.CLIP[side]]
                                   for side in ('left', 'right')},
        joint_limit_policy='Intersection of analyzed software clips and current URDF limits, '
                           'with 0.05 rad clearance; filter incompatible source witnesses',
        authoritative_representation='target_position and target_rpy_degrees for the named grasp frame; '
                                     'arm_configuration is a verified IK witness',
        non_arm_configuration='All other joints, including torso and opposite arm, locked at zero',
        training_requirement='Use all seven joints per active arm and a grasp-frame pose objective. '
                             'The current mjlab ALMI manipulation parser uses only four proximal joints '
                             'and does not enforce these Cartesian poses or wrist orientations.',
        screening=dict(position_tolerance_m=0.001, angular_tolerance_rad=0.005,
                       joint_limit_clearance_rad=cfg.MARGIN,
                       minimum_translational_jacobian_singular_value=cfg.SMIN,
                       collision_model='Full-body sphere URDF with SRDF exclusions, single active arm',
                       scope='Static endpoints only; transitions and simultaneous arm pairs not checked',
                       camera='Head color optical frustum and minimum depth; no occlusion test'),
        selection=dict(method='8 nearest feasible corners + 6 nearest grid face centers + nearest grid center + '
                              '10 greedy farthest points in box-normalized coordinates',
                       grid_step_m=cfg.STEP, targets_per_arm=25,
                       coverage_scope='Published camera-visible working boxes, not the full reachable cloud',
                       tie_break='Lexicographic x, y, z; nearest anchors can differ from ideal by half a grid cell'),
        source_sha256={str(Path(p).relative_to(ROOT)): sha256(p) for p in
                       (cfg.URDF, cfg.URDF_SPHERE, cfg.SRDF_SPHERE,
                        ROOT / 'config/safety_split.yaml', Path(cfg.SCRIPT_DIR) / 'config.py')},
        workspaces={}, candidates=[])
    for index, side in enumerate(('left', 'right')):
        pitch, yaw, roll = ((pose['pitch'], pose['yaw'], pose['roll']) if side == 'left'
                            else cfg.mirrored(pose))
        source = cfg.data_path('fine', side, pitch, yaw, roll)
        arr = np.load(source)
        box = pose['box'][index]
        inside = np.all([(arr[:, a] >= box[2 * a] - 1e-9)
                         & (arr[:, a] <= box[2 * a + 1] + 1e-9) for a in range(3)], axis=0)
        rows = arr[inside]
        rows = rows[np.lexsort((rows[:, 2], rows[:, 1], rows[:, 0]))]
        assert len(rows) >= 25 and kin.good_mask(rows).all(), (side, pose['key'])
        c = kin.context(side)
        c['lo'] = np.maximum(c['lo'], c['model'].lowerPositionLimit)
        c['hi'] = np.minimum(c['hi'], c['model'].upperPositionLimit)
        source_count = len(rows)
        limit_clearance = np.minimum(rows[:, 7:] - c['lo'], c['hi'] - rows[:, 7:]).min(axis=1)
        rows = rows[limit_clearance >= cfg.MARGIN]
        assert len(rows) >= 25
        chosen, labels, radius = select(rows[:, :3], box)
        assert list(c['model'].names)[1:] == [f'{side}_{j}_joint' for j in cfg.ARM_JOINTS]
        catalog['workspaces'][side] = dict(
            frame=f'{side}_grasp_frame', bounds_m=dict(zip('xyz', np.array(box).reshape(3, 2).tolist())),
            rpy_degrees=[roll, pitch, yaw], source_grid_points=source_count,
            compatible_grid_points=len(rows),
            source_sweep_sha256=sha256(source), source_sweep=str(Path(source).relative_to(ROOT)),
            normalized_grid_cover_radius=radius)
        catalog['workspaces'][side]['effective_joint_bounds_rad'] = np.column_stack(
            (c['lo'], c['hi'])).tolist()
        for rank, (row_index, label) in enumerate(zip(chosen, labels), 1):
            row = rows[row_index]
            p, q = row[:3].copy(), row[7:].copy()
            target = pin.SE3(kin.rotation(*np.deg2rad([pitch, yaw, roll])), p)
            assert kin.reaches(c, target, q)
            clearance, collision, smin = kin.quality(c, q)
            assert kin.is_good(clearance, collision, smin)
            achieved = c['data'].oMf[c['fid']]
            position_error = float(np.linalg.norm(achieved.translation - p))
            angular_error = float(np.linalg.norm(pin.log3(target.rotation.T @ achieved.rotation)))
            u, v, depth, visible = an.project(an.camera_pose(), p[None])
            assert visible[0]
            q14 = np.zeros(14)
            q14[0:7] = q if side == 'left' else 0
            q14[7:14] = q if side == 'right' else 0
            catalog['candidates'].append(dict(
                id=f'{side}_{NAMES[pose["key"]]}_{label}', direction_id=f'{side}_{NAMES[pose["key"]]}',
                group=name, family=NAMES[pose['key']], arm=side, rank=rank, kind=label,
                frame=f'{side}_grasp_frame', reference_frame='pelvis',
                target_position=np.round(p, 6).tolist(), target_rpy_degrees=[roll, pitch, yaw],
                arm_configuration=q14.tolist(), success=True, collision_free=True, within_limits=True,
                position_error=position_error, angular_error=angular_error,
                metrics=dict(joint_limit_clearance_rad=float(clearance),
                             minimum_translational_jacobian_singular_value=float(smin),
                             head_color_in_view=True, head_color_pixel_uv=[float(u[0]), float(v[0])],
                             head_optical_depth_m=float(depth[0]))))
    path = OUT / f'{name}.yaml'
    path.write_text(yaml.safe_dump(catalog, sort_keys=False, width=100, indent=4))
    assert yaml.safe_load(path.read_text()) == catalog
    print(f'{path.name}: {len(catalog["candidates"])} verified targets', flush=True)
    return name, catalog


def figure(catalog, robot_traces):
    '''Plot targets, robot and camera frustum with equal meter scales'''
    fig = go.Figure(list(robot_traces) + frustum_traces(far=1.0))
    for side, color in (('left', '#cf552a'), ('right', '#1274a0')):
        entries = [c for c in catalog['candidates'] if c['arm'] == side]
        points = np.array([c['target_position'] for c in entries])
        hover = [f'{c["id"]}<br>RPY {c["target_rpy_degrees"]} deg<br>'
                 f'clearance {c["metrics"]["joint_limit_clearance_rad"]:.3f} rad'
                 for c in entries]
        marker = dict(size=5, color=color)
        fig.add_trace(go.Scatter3d(x=points[:, 0], y=points[:, 1], z=points[:, 2],
                                  mode='markers', marker=marker, name=side, legendgroup=side,
                                  text=hover, hovertemplate='%{text}<br>(%{x:.3f}, %{y:.3f}, %{z:.3f}) m<extra></extra>'))
        bounds = catalog['workspaces'][side]['bounds_m']
        corners = list(product(*[bounds[a] for a in 'xyz']))
        edges = [(a, b) for i, a in enumerate(corners) for b in corners[i + 1:]
                 if sum(x != y for x, y in zip(a, b)) == 1]
        xyz = [[v for a, b in edges for v in (a[k], b[k], None)] for k in range(3)]
        fig.add_trace(go.Scatter3d(x=xyz[0], y=xyz[1], z=xyz[2], mode='lines',
                                  line=dict(color=color, width=3), showlegend=False, hoverinfo='skip'))
    axis = dict(backgroundcolor='#f6f6f4', gridcolor='#e2e2df')
    # scale the scene dimensions by the displayed meter ranges, preserving equal units
    fig.update_layout(height=760, scene=dict(
        aspectmode='manual', aspectratio=dict(x=1.05 / 1.6, y=1, z=1.2 / 1.6),
        xaxis=dict(title='X [m]', range=[-0.25, 0.80], **axis),
        yaxis=dict(title='Y [m]', range=[-0.80, 0.80], **axis),
        zaxis=dict(title='Z [m]', range=[-0.45, 0.75], **axis),
        camera=dict(eye=dict(x=1.15, y=0.8, z=0.42),
                    center=dict(x=0.03, y=0.08, z=-0.08),
                    projection=dict(type='orthographic'))),
                      legend=dict(orientation='h', x=0, y=1.03),
                      margin=dict(l=10, r=10, t=65, b=15), template='plotly_white')
    return fig


def report(catalogs):
    '''Write an offline 3D target atlas with downloads and concise training notes'''
    robot = RobotMesh()
    triangles = robot.triangles(robot.configuration({}), cell=0.012)
    body, arms = [], []
    for name, tris in triangles:
        (arms if any(is_arm(name, side) for side in ('left', 'right')) else body).append(tris)
    robot_traces = [mesh_trace(body, '#d4d6da', 'Robot body'),
                    mesh_trace(arms, '#7d8596', 'Robot arms (zero posture)')]
    titles = {'pitch_90': 'Pitch 90° · Pointing down', 'pitch_60': 'Pitch 60°',
              'pitch_45': 'Pitch 45°', 'horizontal': 'Horizontal · Pointing forward'}
    parts = ['''<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>H12 workspace targets</title>
<style>
    :root { color-scheme: light; }
    * { box-sizing: border-box; }
    html { scroll-behavior: smooth; }
    body { margin: 0; background: #f4f6f8; color: #253340; font: 16px system-ui, sans-serif; }
    main { max-width: 1500px; margin: auto; padding: 48px 28px; }
    h1 { font-size: clamp(30px, 4vw, 46px); letter-spacing: -.04em; margin: 8px 0 12px; }
    h2 { font-size: 25px; letter-spacing: -.025em; margin: 0; }
    h3 { margin: 0 0 10px; }
    p { line-height: 1.65; margin: 10px 0; }
    a { color: #165f89; text-underline-offset: 3px; }
    .eyebrow { color: #637381; font-size: 12px; font-weight: 700; letter-spacing: .14em; }
    .intro { max-width: 870px; color: #536372; }
    .stats, nav, .section-head { display: flex; gap: 12px; flex-wrap: wrap; align-items: center; }
    .stats { margin: 24px 0; }
    .stats span { padding: 10px 16px; background: #e8eef3; border-radius: 10px; }
    nav { margin: 24px 0 32px; }
    nav a { background: white; border: 1px solid #dce3e9; border-radius: 8px;
            padding: 10px 16px; text-decoration: none; }
    section, .guide { background: white; border: 1px solid #dce3e9; border-radius: 16px;
                     padding: 26px; margin: 24px 0; scroll-margin-top: 20px; }
    .section-head { justify-content: space-between; }
    .download { background: #1a526f; color: white; padding: 10px 16px;
                border-radius: 8px; text-decoration: none; font-size: 14px; }
    .hint { color: #637381; font-size: 14px; }
    .table-wrap { overflow-x: auto; }
    table { border-collapse: collapse; width: 100%; font-size: 14px; }
    th, td { padding: 11px 12px; border-bottom: 1px solid #e4e9ed; text-align: left; }
    th { color: #637381; font-weight: 600; background: #f7f9fb; white-space: nowrap; }
    td:first-child { font-weight: 500; }
    details { margin-top: 18px; }
    summary { cursor: pointer; font-weight: 600; padding: 10px 0; }
    code { background: #edf2f6; border-radius: 4px; padding: 2px 5px; font-size: .88em; }
    .note { border-left: 3px solid #d5a449; padding-left: 18px; }
    footer { color: #637381; font-size: 14px; padding: 12px 0; }
    @media (max-width: 700px) {
        main { padding: 24px 12px; }
        section, .guide { padding: 16px 10px; }
        td, th { padding: 9px; }
    }
</style></head><body><main>
<header><div class="eyebrow">H12 · MAGPIE · POLICY FINE-TUNING</div>
<h1>Workspace targets</h1>
<p class="intro">Four grasp orientations, sampled across the left and right camera-visible
working boxes. Explore the targets against the robot and calibrated head-camera view,
then download the self-contained training catalogs.</p>
<div class="stats"><span><strong>200</strong> targets</span><span><strong>4</strong> catalogs</span>
<span><strong>25</strong> positions per arm / orientation</span><span><strong>25 mm</strong> source grid</span></div>
<p class="hint">Based on the <a href="h12_workspace_analysis.html">H12 workspace analysis</a>.
Each arm uses its own workspace bounds and orientation.</p></header>''']
    parts.append('<nav aria-label="Target sets">' + ''.join(
        f'<a href="#{name}">{titles[catalog["candidates"][0]["family"]]}</a>'
        for name, catalog in catalogs) + '</nav>')
    parts.append('''<div class="guide"><h3>Reading the plots</h3>
<p>Orange targets belong to the left arm; blue targets belong to the right. The wire boxes
show the published working bounds, and the green cone shows the calibrated head-camera
color view, drawn to 1 m optical depth as in the workspace analysis. The gray robot is at
zero posture as a spatial reference.</p>
<p class="hint">Drag to rotate · Scroll to zoom · Hover for target IDs and poses · Click legend
entries to hide the robot, camera or targets. All three axes use the same meter scale;
orthographic projection preserves proportions.</p>
<p>Positions are grasp fingertip coordinates relative to <code>pelvis</code>:
X forward, Y left, Z up. Left targets use <code>left_grasp_frame</code>; right targets use
<code>right_grasp_frame</code>. RPY is in degrees with
<code>R = Rz(yaw) Ry(pitch) Rx(roll)</code>. Preserve the full RPY, including roll and yaw.</p></div>''')
    parts.append('<script>' + get_plotlyjs() + '</script>')
    for name, catalog in catalogs:
        title = titles[catalog['candidates'][0]['family']]
        parts.append(f'<section id="{name}"><div class="section-head"><h2>{title}</h2>'
                     f'<a class="download" href="arm_targets/{name}.yaml" download>Download YAML</a></div>'
                     f'<p class="hint">50 targets · 25 left + 25 right · {name}.yaml</p>')
        parts.append('<div class="table-wrap"><table><thead><tr><th>Arm</th><th>X [m]</th>'
                     '<th>Y [m]</th><th>Z [m]</th><th>Roll / pitch / yaw [deg]</th></tr></thead><tbody>')
        for side, info in catalog['workspaces'].items():
            cells = ''.join(f'<td>{lo:.3f}–{hi:.3f}</td>' for lo, hi in info['bounds_m'].values())
            rpy = ' / '.join(str(value) for value in info['rpy_degrees'])
            parts.append(f'<tr><td>{side.capitalize()}</td>{cells}<td>{rpy}</td></tr>')
        parts.append('</tbody></table></div>')
        parts.append(figure(catalog, robot_traces).to_html(
            full_html=False, include_plotlyjs=False,
            config=dict(responsive=True, displaylogo=False, scrollZoom=True)))
        parts.append('<details><summary>Inspect all 50 target poses</summary><div class="table-wrap">'
                     '<table><thead><tr><th>ID</th><th>Frame</th><th>XYZ [m]</th>'
                     '<th>RPY [deg]</th></tr></thead><tbody>')
        for c in catalog['candidates']:
            parts.append(f'<tr><td>{c["id"]}</td><td>{c["frame"]}</td>'
                         f'<td>{c["target_position"]}</td><td>{c["target_rpy_degrees"]}</td></tr>')
        parts.append('</tbody></table></div></details></section>')
    parts.append('''<div class="guide"><h2>Using the catalogs</h2>
<p>Copy the four YAMLs to <code>h12_mjlab/data/arm_targets/</code>. Each follows the
schema-v1 <code>candidates</code> format and includes pelvis-relative grasp poses,
14-joint IK witnesses, explicit joint order, effective limits and provenance hashes.</p>
<p class="note"><strong>Use a full-arm pose task.</strong> Control all seven joints per active arm
and train against the named grasp-frame position and orientation. The current mjlab ALMI
manipulation parser retains only four proximal joints, so loading these files alone does
not enforce their wrist orientations.</p>
<details><summary>Selection and verification</summary>
<p>Each box contributes eight nearest feasible corner samples, six nearest grid face centers,
one nearest grid center and ten greedily spaced coverage points. Distances are normalized
by the box dimensions to cover short axes as well as long ones. These targets cover the
published working boxes, not the entire irregular reachable cloud.</p>
<p>The controller allows inward shoulder roll to −0.38 rad on the left and +0.38 rad on the
right; the URDF uses −0.19 and +0.19 rad. Source witnesses are filtered to the intersection
of both limits with 0.05 rad clearance. Some horizontal corner anchors consequently move
to the nearest compatible grid point; their labels describe the intended anchor.</p>
<p>All 200 stored witnesses pass position error &lt;1 mm, angular error &lt;5 mrad, joint-limit
clearance ≥0.05 rad, minimum translational Jacobian singular value ≥0.06, no sphere-model
self-collision and head color-frustum inclusion. The torso, opposite arm and all other
joints are zero. The camera cone visualizes the image bounds; visibility checks do not
test occlusion. Transitions, simultaneous arm pairs and dynamics require validation in
the training environment.</p></details>
<details><summary>Suggested training prompt</summary>
<p>Load these four schema-v1 candidates catalogs. Fine-tune a full seven-DOF-per-arm Magpie
policy on the named pelvis-relative grasp-frame positions and orientations. Use
arm_configuration as an IK witness, preserve each arm’s full RPY, balance sampling across
orientations and arms, and validate transitions and arm pairs.</p></details></div>
<footer>Self-contained offline report · <a href="h12_workspace_analysis.html">Workspace analysis</a>
· <a href="README.md">Documentation</a></footer></main></body></html>''')
    REPORT.write_text('\n'.join(parts))


def main():
    '''Generate exactly four independent catalogs and their report'''
    OUT.mkdir(exist_ok=True)
    catalogs = [export(pose) for pose in cfg.POSES]
    entries = [c for _, catalog in catalogs for c in catalog['candidates']]
    assert len(entries) == len({c['id'] for c in entries}) == 200
    report(catalogs)
    print(json.dumps(dict(targets=len(entries),
                         max_position_error_m=max(c['position_error'] for c in entries),
                         max_angular_error_rad=max(c['angular_error'] for c in entries)), indent=4))


if __name__ == '__main__':
    main()
