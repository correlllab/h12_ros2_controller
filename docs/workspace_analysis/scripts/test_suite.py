'''Build a copyable 40-command frame-controller test suite from the target catalogs'''
import json
from html import escape
from pathlib import Path

import yaml
import numpy as np
from plotly.offline import get_plotlyjs

from robot_mesh import RobotMesh, is_arm
from training_targets import figure, mesh_trace

ROOT = Path(__file__).resolve().parents[1]
SETS = [('pitch_90', 'Pitch 90° · Pointing down'), ('pitch_60', 'Pitch 60°'),
        ('pitch_45', 'Pitch 45°'), ('horizontal', 'Horizontal · Pointing forward')]


def choose(catalog, side):
    '''Keep the source center and four maximin samples in normalized box coordinates'''
    entries = [c for c in catalog['candidates'] if c['arm'] == side]
    entries.sort(key=lambda c: c['id'])
    bounds = np.array(list(catalog['workspaces'][side]['bounds_m'].values()))
    points = np.array([c['target_position'] for c in entries])
    unit = (points - bounds[:, 0]) / np.diff(bounds, axis=1).ravel()
    chosen = [next(i for i, c in enumerate(entries) if c['kind'] == 'center')]
    for _ in range(4):
        distance = np.min(np.linalg.norm(unit[:, None] - unit[chosen], axis=2), axis=1)
        distance[chosen] = -np.inf
        chosen.append(int(np.argmax(distance)))
    return [entries[i] for i in chosen]


def command(entry):
    '''Format an exact catalog pose with console position and degree units'''
    values = entry['target_position'] + entry['target_rpy_degrees']
    numbers = ' '.join(f'{value:.6f}'.rstrip('0').rstrip('.') if value else '0' for value in values)
    return f'frame {entry["frame"]} {numbers}'


def test_plot(catalog, selected, robot_traces):
    '''Show the ten test poses with labels matching their command fields'''
    subset = dict(catalog, candidates=[entry for entries in selected.values() for entry in entries])
    fig = figure(subset, robot_traces)
    for side, entries in selected.items():
        family = entries[0]['family']
        trace = next(trace for trace in fig.data if trace.name == side)
        trace.hovertext = trace.text
        trace.text = [f'{side[0].upper()}{number:02d}' for number in range(1, 6)]
        trace.customdata = [f'{family}_{side}_{number}' for number in range(1, 6)]
        trace.mode = 'markers+text'
        trace.textposition = 'top center'
        trace.textfont = dict(size=13, color=trace.marker.color)
        trace.marker.size = 7
        trace.hovertemplate = ('%{text} · %{hovertext}<br>'
                               'XYZ (%{x:.3f}, %{y:.3f}, %{z:.3f}) m<br>'
                               'Click to select command<extra></extra>')
    return fig.to_html(
        full_html=False, include_plotlyjs=False,
        config=dict(responsive=True, displaylogo=False, scrollZoom=True),
        post_script='''
document.getElementById('{plot_id}').on('plotly_click', event => {
    const field = document.getElementById(event.points[0].customdata);
    if (field) {
        field.scrollIntoView({behavior: 'smooth', block: 'center'});
        field.focus({preventScroll: true});
        field.select();
    }
});''')


def main():
    '''Write the offline suite with per-command and per-group copy controls'''
    robot = RobotMesh()
    body, arms = [], []
    for name, triangles in robot.triangles(robot.configuration({}), cell=0.012):
        (arms if any(is_arm(name, side) for side in ('left', 'right')) else body).append(triangles)
    robot_traces = [mesh_trace(body, '#d4d6da', 'Robot body'),
                    mesh_trace(arms, '#7d8596', 'Robot arms (zero posture)')]
    records, sections = [], []
    for family, title in SETS:
        path = ROOT / 'arm_targets' / f'arm_{family}_targets.yaml'
        catalog = yaml.safe_load(path.read_text())
        selected = {side: choose(catalog, side) for side in ('left', 'right')}
        cards = []
        for side in ('left', 'right'):
            group = f'{family}_{side}'
            rows = []
            for number, entry in enumerate(selected[side], 1):
                text = command(entry)
                key = f'{group}_{number}'
                records.append(dict(key=key, group=group, command=text, source_id=entry['id']))
                boundary = entry['target_position'][0] > 0.60 + 1e-9
                tag = '<span class="tag">X &gt; 0.60 m</span>' if boundary else ''
                label = f'{side[0].upper()}{number:02d}'
                rows.append(f'<div class="test"><div class="meta"><strong>{label}</strong> '
                            f'<span>{escape(entry["id"])}</span>{tag}</div>'
                            f'<div class="command"><input id="{key}" readonly spellcheck="false" '
                            f'aria-label="{side} {family} test {number} command" value="{escape(text)}">'
                            f'<button data-copy="{key}">Copy</button></div></div>')
            cards.append(f'<article><div class="card-head"><h3>{side.capitalize()} arm</h3>'
                         f'<button data-group="{group}">Copy 5 commands</button></div>'
                         + ''.join(rows) + '</article>')
        sections.append(f'<section id="{family}"><div class="section-head"><h2>{title}</h2>'
                        f'<a href="arm_targets/{path.name}">Source YAML ↗</a></div>'
                        '<p class="muted">Orange L01–L05: left tests · Blue R01–R05: right tests. '
                        'Click a target to select its command below.</p>'
                        + test_plot(catalog, selected, robot_traces)
                        + ''.join(cards) + '</section>')
    header = '''<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>H12 workspace test suite</title>
<style>
    * { box-sizing: border-box; }
    html { scroll-behavior: smooth; }
    body { margin: 0; background: #f3f6f8; color: #263745; font: 16px system-ui, sans-serif; }
    main { max-width: 1250px; margin: auto; padding: 40px 24px; }
    h1 { font-size: clamp(30px, 4vw, 44px); letter-spacing: -.04em; margin: 10px 0; }
    h2 { font-size: 25px; margin: 0; letter-spacing: -.02em; }
    h3 { font-size: 18px; margin: 0; }
    p { line-height: 1.65; }
    a { color: #1a6287; text-underline-offset: 3px; }
    .eyebrow { color: #607483; font-size: 12px; font-weight: 700; letter-spacing: .13em; }
    .muted, .meta { color: #607483; font-size: 14px; }
    nav, .toolbar, .section-head, .card-head { display: flex; flex-wrap: wrap;
                                           gap: 12px; align-items: center; }
    nav { margin: 24px 0; }
    nav a, button { border: 1px solid #cad7df; background: white; border-radius: 7px;
                   padding: 9px 14px; color: #1e5774; font: inherit; text-decoration: none; }
    button { cursor: pointer; font-size: 14px; white-space: nowrap; }
    button:hover, nav a:hover { background: #eaf2f7; }
    button:focus-visible, input:focus-visible { outline: 3px solid #70afd1; outline-offset: 2px; }
    .primary { background: #1e5774; color: white; }
    .primary:hover { background: #153e53; }
    .guide, section { border: 1px solid #dce5eb; background: white; border-radius: 14px;
                      padding: 24px; margin: 24px 0; scroll-margin-top: 18px; }
    .section-head, .card-head { justify-content: space-between; }
    article { margin-top: 22px; border-top: 3px solid #d1774b; padding-top: 18px; }
    article + article { border-top-color: #3c84a8; }
    .test { padding: 14px 0; border-bottom: 1px solid #e5ebef; }
    .test:focus-within { background: #eaf2f7; border-radius: 6px; }
    .meta { display: flex; gap: 10px; flex-wrap: wrap; align-items: center; margin-bottom: 8px; }
    .meta strong { color: #263745; }
    .command { display: flex; gap: 10px; }
    input { flex: 1; min-width: 0; border: 1px solid #d8e2e9; background: #f7f9fb;
            padding: 12px; border-radius: 6px; color: #223b4c; font: 14px ui-monospace, monospace; }
    code { background: #edf2f6; padding: 2px 5px; border-radius: 4px; font-size: .9em; }
    .tag { background: #fff1d7; color: #805711; font-size: 12px; padding: 3px 7px; border-radius: 4px; }
    .note { border-left: 3px solid #d3a14a; padding-left: 16px; }
    details { margin-top: 15px; }
    summary { cursor: pointer; font-weight: 600; }
    #status { min-height: 24px; color: #256347; font-size: 14px; }
    footer { font-size: 14px; color: #607483; padding: 16px 0; }
    @media (max-width: 650px) { main { padding: 24px 12px; } section, .guide { padding: 16px 12px; } }
    @media print { button, nav, .toolbar { display: none; } main { max-width: none; padding: 0; }
                   section, article { break-inside: avoid; } input { font-size: 11px; } }
</style></head><body><main>
<header><div class="eyebrow">H12 · FRAME CONTROLLER · MANUAL TEST SUITE</div>
<h1>Workspace test suite</h1>
<p>40 commands · Four orientations · Five targets per arm and orientation.</p>
<p class="muted">A representative subset of the
<a href="h12_workspace_targets.html">workspace targets</a>, retaining each source pose exactly.
Start at the center sample, then test four spread-out reaches in each workspace.</p></header>
<div class="guide"><h3>Run a test</h3>
<p>Copy a command into the frame controller’s interactive goto prompt, one target at a time.
Commands use <code>frame frame_name x y z roll pitch yaw</code>:
pelvis-relative positions in meters and orientation in degrees. Use the complete RPY;
left and right poses may have different roll and yaw.</p>
<p>For independent comparisons, return to <code>home</code> between targets and wait for each
motion to settle. Keep the torso at zero and the opposite arm at home to match the catalog
endpoint checks. Copying a block gives one command per line; it does not execute the suite.</p>
<p>Each 3D view shows only that section’s ten test targets, labeled to match the commands.
The gray robot is at zero posture, the wire boxes show the published workspace bounds,
and the green cone is the calibrated head-camera color view drawn to 1 m optical depth.
Drag to rotate, scroll to zoom, and use the legend to hide scene elements. All three
axes use equal meter scales with orthographic projection.</p>
<p class="note">Targets marked <span class="tag">X &gt; 0.60 m</span> exceed the current
<code>safety_split.yaml</code> setting <code>planner.frame_x_max: 0.60</code>.
Testing those published workspace boundaries requires a selected profile whose X limit
includes the target. This document does not change controller settings.</p>
<details><summary>How these targets were selected</summary>
<p>Each group takes the catalog’s center sample plus four greedy farthest-point samples
from its 25 targets, using distance normalized by the three workspace dimensions.
Ties follow source ID order. IDs below map directly to the source YAML.</p>
<p>These are static endpoint tests. Their positions, orientations and IK witnesses were
screened in the source catalogs; they do not guarantee a transition from every measured
posture. The left and right groups are independent, not simultaneous arm commands.</p>
<p>See <a href="../sample_frame_commands.md">sample frame commands</a> for launch details
and <a href="h12_workspace_analysis.html">workspace analysis</a> for the underlying bounds.</p>
</details></div>
'''
    navigation = '<nav aria-label="Orientations">' + ''.join(
        f'<a href="#{family}">{title}</a>' for family, title in SETS) + '</nav>'
    toolbar = ('<div class="toolbar"><button class="primary" data-all>Copy all 40 commands</button>'
               '<span class="muted">Individual command fields can also be selected and copied manually.</span>'
               '</div><p id="status" role="status" aria-live="polite"></p>')
    script = '''<script>
const records = RECORDS;
async function copyText(text, button) {
    const status = document.getElementById('status');
    try {
        try {
            if (!navigator.clipboard) throw new Error('Clipboard unavailable');
            await navigator.clipboard.writeText(text);
        } catch (error) {
            const field = document.createElement('textarea');
            field.value = text;
            field.style.position = 'fixed';
            field.style.opacity = '0';
            document.body.appendChild(field);
            field.select();
            let copied;
            try { copied = document.execCommand('copy'); }
            finally { field.remove(); button.focus(); }
            if (!copied) throw new Error('Copy blocked');
        }
        const count = text.split('\\n').length;
        status.textContent = `Copied ${count} command${count === 1 ? '' : 's'}.`;
    } catch (error) {
        status.textContent = 'Clipboard blocked. Select a command field and copy it manually.';
    }
}
document.addEventListener('click', event => {
    const button = event.target.closest('button');
    if (!button) return;
    let subset;
    if (button.hasAttribute('data-copy')) subset = records.filter(r => r.key === button.dataset.copy);
    else if (button.hasAttribute('data-group')) subset = records.filter(r => r.group === button.dataset.group);
    else if (button.hasAttribute('data-all')) subset = records;
    else return;
    copyText(subset.map(r => r.command).join('\\n'), button);
});
document.querySelectorAll('input[readonly]').forEach(field => {
    field.addEventListener('click', () => field.select());
});
</script>'''.replace('RECORDS', json.dumps(records))
    output = ROOT / 'h12_workspace_test.html'
    output.write_text(header + navigation + toolbar + '<script>' + get_plotlyjs() + '</script>'
                      + '\n'.join(sections)
                      + '<footer>Offline document · '
                      '<a href="h12_workspace_targets.html">All 200 workspace targets</a>'
                      '</footer></main>' + script + '</body></html>')
    print(f'{output}: {len(records)} commands')


if __name__ == '__main__':
    main()
