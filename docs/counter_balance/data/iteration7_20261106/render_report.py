'''Refresh numeric tables and an inline SVG from the frozen iteration-7 results'''
import json
import sys
from html import escape
from pathlib import Path

import numpy as np


LABELS = {'B1': 'FAME diagonal', 'B2': 'ALMI diagonal', 'B3': 'FAME sideways',
          'R3': 'FAME sideways repeat', 'B4': 'ALMI inner-forward',
          'R4': 'ALMI inner-forward repeat'}
ORDER = ('B1', 'B2', 'B3', 'R3', 'B4', 'R4')


def table(records, columns):
    rows = ['<div class="scroll"><table><thead><tr><th>Pair</th>']
    rows.extend(f'<th>{escape(label)}</th>' for label, _, _, _ in columns)
    rows.append('</tr></thead><tbody>')
    for case in ORDER:
        mark = '*' if case in ('B3', 'R3') else ('†' if case == 'B4' else '')
        rows.append(f'<tr><td><strong>{case}{mark}</strong><br>{LABELS[case]}</td>')
        for _, key, scale, precision in columns:
            f = records[case + '_frame'][key] * scale
            c = records[case + '_counter'][key] * scale
            rows.append(f'<td>{f:.{precision}f} → {c:.{precision}f}</td>')
        rows.append('</tr>')
    rows.append('</tbody></table></div>')
    return ''.join(rows)


def figure(records):
    parts = ['<figure><svg viewBox="0 0 1050 385" role="img" aria-labelledby="effects-title effects-desc">',
             '<title id="effects-title">Counter effects differ by task despite consistent timed-tracking penalties</title>',
             '<desc id="effects-desc">Six pair differences, Counter minus Frame. Timed tracking is worse in all six. Tilt is better in FAME diagonal and both ALMI inner-forward pairs, worse in ALMI diagonal and both FAME sideways pairs. Each dot is one paired comparison, not a confidence interval.</desc>',
             '<g font-family="system-ui,sans-serif" font-size="13" fill="#172d38">',
             '<text x="385" y="25" text-anchor="middle" font-size="17">Timed tracking Δ (mm)</text>',
             '<text x="840" y="25" text-anchor="middle" font-size="17">Move tilt RMS Δ (deg)</text>']
    panels = [('timed_position_error_rms_m', 1000, 240, 560, 0, 3.2, [0, 1, 2, 3]),
              ('move_rms_tilt_deviation_deg', 1, 680, 1000, -.3, 1.5, [0, .5, 1])]
    for key, factor, left, right, low, high, ticks in panels:
        def coordinate(value):
            return left + (value - low) * (right - left) / (high - low)
        zero = coordinate(0)
        parts.append(f'<line x1="{zero:.2f}" y1="43" x2="{zero:.2f}" y2="318" stroke="#536875" stroke-dasharray="4 4"/>')
        for tick in ticks:
            xx = coordinate(tick)
            parts.append(f'<text x="{xx:.2f}" y="342" text-anchor="middle">{tick:g}</text>')
        for index, case in enumerate(ORDER):
            y = 65 + index * 45
            delta = (records[case + '_counter'][key] - records[case + '_frame'][key]) * factor
            xx = coordinate(delta)
            color = '#b05235' if delta > 0 else '#16716c'
            parts.append(f'<line x1="{zero:.2f}" y1="{y}" x2="{xx:.2f}" y2="{y}" stroke="{color}" stroke-width="2"/>')
            parts.append(f'<circle cx="{xx:.2f}" cy="{y}" r="5" fill="{color}"/>')
            parts.append(f'<text x="{xx + 10:.2f}" y="{y-9}" fill="{color}">{delta:+.3f}</text>')
    for index, case in enumerate(ORDER):
        parts.append(f'<text x="0" y="{70+index*45}">{case} · {LABELS[case]}</text>')
    parts.append('<text x="525" y="375" text-anchor="middle" fill="#536875">Negative: Counter better. Positive: Counter worse.</text></g></svg>')
    parts.append('<figcaption>Observed pair differences, not causal policy effects or confidence intervals. Repeats stay separate; B3/R3 onset-warning records remain included.</figcaption></figure>')
    return ''.join(parts)


def phase_figure(records):
    '''Plot observed onset-relative rotation without filling unobserved onset'''
    root = Path.cwd()
    sys.path.insert(0, str(root))
    from h12_zmp_benchmark.experiment import analyze_real_runs as reducer
    parts = ['<figure><svg viewBox="0 0 1050 375" role="img" aria-labelledby="phase-title phase-desc">',
             '<title id="phase-title">Rotation grows before posture-only recovery</title>',
             '<desc id="phase-desc">Observed onset-relative orientation traces for B2 ALMI diagonal and R3 FAME sideways. Counter rotation already exceeds Frame during movement. Shading marks the half-second fade; posture-only recovery follows.</desc>',
             '<g font-family="system-ui,sans-serif" font-size="12" fill="#172d38">']
    for panel, case in enumerate(('B2', 'R3')):
        left, top, width, height = 65 + panel * 520, 55, 400, 230
        duration = records[case + '_counter']['move_duration_s']
        x = lambda t: left + np.asarray(t) / 3 * width
        y = lambda angle: top + height - np.asarray(angle) / 5 * height
        parts.append(f'<text x="{left + width/2}" y="24" text-anchor="middle" font-size="17">{case} · {LABELS[case]}</text>')
        parts.append(f'<rect x="{x(duration):.2f}" y="{top}" width="{width/6:.2f}" height="{height}" fill="#e7efeb"/>')
        for value in (0, 1, 2, 3, 4, 5):
            parts.append(f'<line x1="{left}" x2="{left+width}" y1="{y(value):.2f}" y2="{y(value):.2f}" stroke="#d7ddd9"/>')
            parts.append(f'<text x="{left-10}" y="{y(value)+4:.2f}" text-anchor="end">{value}</text>')
        for value in (0, 1, 2, 3):
            parts.append(f'<text x="{x(value):.2f}" y="{top+height+22}" text-anchor="middle">{value}</text>')
        parts.append(f'<text x="{x(duration+.25):.2f}" y="{top+15}" text-anchor="middle">Fade</text>')
        for controller, color in (('frame', '#3567a0'), ('counter', '#b05235')):
            row = records[case + '_' + controller]
            samples = reducer.read_samples(root / row['raw_id'])
            times = samples['time']
            rotations = samples['pelvis']
            angles = np.degrees([reducer.rotation_angle(rotations[0].T @ r) for r in rotations])
            mask = times <= 3
            points = ' '.join(f'{xx:.2f},{yy:.2f}' for xx, yy in zip(x(times[mask]), y(angles[mask])))
            parts.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2"/>')
        parts.append(f'<text x="{left+width/2}" y="{top+height+45}" text-anchor="middle">Time since execution start (s)</text>')
    parts.extend(['<text x="18" y="178" transform="rotate(-90 18 178)" text-anchor="middle">Onset-relative orientation (deg)</text>',
                  '<line x1="380" x2="405" y1="360" y2="360" stroke="#3567a0" stroke-width="3"/><text x="415" y="364">Frame</text>',
                  '<line x1="540" x2="565" y1="360" y2="360" stroke="#b05235" stroke-width="3"/><text x="575" y="364">Counter</text>',
                  '</g></svg><figcaption>Raw recorded pose samples, re-referenced to each run\'s first observed task orientation; not fixture-relative pose. No pre-onset extrapolation. No gap above 50 ms occurs in these displayed intervals. The recovery candidate planned for 7b begins only after the shaded fade ends.</figcaption></figure>'])
    return ''.join(parts)


def main():
    here = Path(__file__).resolve().parent
    data = json.loads((here / 'results.json').read_text())
    records = {row['run']: row for row in data['records']}
    assert len(records) == 12 and len(data['pairs']) == 6
    body = figure(records) + phase_figure(records)
    body += '<h3>Tracking</h3>' + table(records, [
        ('Frozen uniform RMS, mm', 'frozen_uniform_rms_m', 1000, 3),
        ('Timed RMS, mm', 'timed_position_error_rms_m', 1000, 3),
        ('Joint-vector RMS, rad', 'configuration_error_rms_rad', 1, 5),
        ('Hold endpoint RMS, mm', 'frozen_hold_error_rms_m', 1000, 3),
    ])
    body += '<h3>Balance and recovery</h3>' + table(records, [
        ('Move peak drift, deg', 'motion_peak_drift_deg', 1, 3),
        ('Move tilt RMS, deg', 'move_rms_tilt_deviation_deg', 1, 3),
        ('Full peak drift, deg', 'peak_drift_deg', 1, 3),
        ('Terminal drift, deg', 'residual_drift_deg', 1, 3),
        ('Peak gyro, rad/s', 'peak_rate_rad_s', 1, 4),
    ])
    body += '<h3>Preparation and task-added orientation</h3>' + table(records, [
        ('Initial drift from standing, deg', 'initial_drift_deg', 1, 3),
        ('Onset SO(3) move RMS, deg', 'onset_so3_motion_rms_deg', 1, 3),
        ('Onset gravity move RMS, deg', 'onset_gravity_motion_rms_deg', 1, 3),
    ])
    body += '<p class="small">* Included onset-only warning. † B4 Frame has an 889 ms late gap; its official terminal estimate contains six samples versus 31 for Counter. No interpolation repairs that evidence. Supplementary clean-tail comparisons below avoid the gap.</p>'
    output = here.parents[1] / 'counter_balance_analysis_iteration_7_hardware.html'
    text = output.read_text()
    start, end = '<!-- PAIR_RESULTS_START -->', '<!-- PAIR_RESULTS_END -->'
    assert text.count(start) == text.count(end) == 1
    text = text.split(start)[0] + start + '\n' + body + '\n' + end + text.split(end)[1]
    output.write_text(text)
    print(f'Updated six-pair tables and inline SVG: {output}')


if __name__ == '__main__':
    main()
