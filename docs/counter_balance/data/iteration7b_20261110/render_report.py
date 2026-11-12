'''Render paired hardware tables and conditional model figures from retained evidence'''
import json
from html import escape
from pathlib import Path


ORDER = ('F01', 'F02', 'F03', 'F03r', 'F04', 'F04r')
LABELS = ('Right diagonal', 'Left sideways', 'Right sideways', 'Right sideways repeat',
          'Right inner-forward', 'Right inner-forward repeat')


def value(record, key):
    for part in key.split('.'):
        record = record[part]
    return record


def table(records, columns):
    out = ['<div class="scroll"><table><thead><tr><th>Pair</th>']
    out.extend(f'<th>{escape(label)}</th>' for label, _, _, _ in columns)
    out.append('</tr></thead><tbody>')
    for case, label in zip(ORDER, LABELS):
        out.append(f'<tr><td><strong>{case}</strong><br>{label}</td>')
        for _, key, scale, digits in columns:
            f = value(records[case + '_frame'], key) * scale
            c = value(records[case + '_counter'], key) * scale
            out.append(f'<td>{f:.{digits}f} → {c:.{digits}f}</td>')
        out.append('</tr>')
    out.append('</tbody></table></div>')
    return ''.join(out)


def motion_figure(records):
    out = ['<figure><svg viewBox="0 0 1000 370" role="img" aria-labelledby="movement-title movement-desc">',
           '<title id="movement-title">Directional benefit, mirrored overhang regressions</title>',
           '<desc id="movement-desc">Each row is one Frame and Counter pair. Move tilt RMS decreases in the right diagonal case but increases in both sideways arms, both right repeats, and both inner-forward repeats.</desc>',
           '<g font-family="system-ui,sans-serif" font-size="14" fill="#172d38">']
    x = lambda v: 310 + v / .65 * 570
    for tick in (0, .1, .2, .3, .4, .5, .6):
        out.append(f'<line x1="{x(tick):.2f}" y1="30" x2="{x(tick):.2f}" y2="302" stroke="#d7ddd9"/>')
        out.append(f'<text x="{x(tick):.2f}" y="326" text-anchor="middle">{tick:g}</text>')
    for index, (case, label) in enumerate(zip(ORDER, LABELS)):
        y = 51 + index * 47
        f = records[case + '_frame']['official']['move_rms_tilt_deviation_deg']
        c = records[case + '_counter']['official']['move_rms_tilt_deviation_deg']
        out.append(f'<text x="0" y="{y+5}">{case} · {label}</text>')
        out.append(f'<line x1="{x(f):.2f}" y1="{y}" x2="{x(c):.2f}" y2="{y}" stroke="#8a9c9b"/>')
        out.append(f'<circle cx="{x(f):.2f}" cy="{y}" r="5" fill="#3567a0"/>')
        out.append(f'<circle cx="{x(c):.2f}" cy="{y}" r="5" fill="#b05235"/>')
    out.extend(['<text x="595" y="353" text-anchor="middle">Movement tilt-deviation RMS (degrees; lower is better)</text>',
                '<circle cx="715" cy="12" r="5" fill="#3567a0"/><text x="728" y="17">Frame</text>',
                '<circle cx="820" cy="12" r="5" fill="#b05235"/><text x="833" y="17">Counter</text>',
                '</g></svg><figcaption>Same policy declaration and unchanged controller configuration within each pair. Repeats remain separate. Direction-aware onset SO(3) and gravity-direction checks preserve these signs; this is not a yaw-only or reference-choice result.</figcaption></figure>'])
    return ''.join(out)


def cam_figure(mechanism):
    out = ['<figure><svg viewBox="0 0 950 440" role="img" aria-labelledby="cam-title cam-desc">',
           '<title id="cam-title">Lower modeled angular momentum is not sufficient for lower torso rate</title>',
           '<desc id="cam-desc">Raw-sensor model branch: all pairs reduce modeled horizontal whole-body CAM. Only diagonal reduces observed torso XY gyro RMS. Five overhang pairs have higher torso rate despite lower modeled momentum. This is conditional model evidence, not a measured wrench or stability certificate.</desc>',
           '<g font-family="system-ui,sans-serif" font-size="13" fill="#172d38">']
    x = lambda v: 100 + (v + 40) / 50 * 700
    y = lambda v: 35 + (80 - v) / 140 * 310
    for tick in (-40, -30, -20, -10, 0, 10):
        out.append(f'<line x1="{x(tick):.2f}" y1="35" x2="{x(tick):.2f}" y2="345" stroke="#d7ddd9"/>')
        out.append(f'<text x="{x(tick):.2f}" y="369" text-anchor="middle">{tick}%</text>')
    for tick in (-60, -40, -20, 0, 20, 40, 60, 80):
        out.append(f'<line x1="100" y1="{y(tick):.2f}" x2="800" y2="{y(tick):.2f}" stroke="#d7ddd9"/>')
        out.append(f'<text x="85" y="{y(tick)+4:.2f}" text-anchor="end">{tick}%</text>')
    out.append(f'<line x1="{x(0):.2f}" y1="35" x2="{x(0):.2f}" y2="345" stroke="#536875" stroke-dasharray="4 4"/>')
    out.append(f'<line x1="100" y1="{y(0):.2f}" x2="800" y2="{y(0):.2f}" stroke="#536875" stroke-dasharray="4 4"/>')
    numbers = []
    for case in ORDER:
        f = mechanism['runs'][case + '_frame']['phases']['move']
        c = mechanism['runs'][case + '_counter']['phases']['move']
        dx = (c['total_rms_xy'] / f['total_rms_xy'] - 1) * 100
        dy = (c['gyro_rms_xy'] / f['gyro_rms_xy'] - 1) * 100
        color = '#16716c' if case == 'F01' else '#b05235'
        offset_y = -11 if case != 'F02' else 18
        out.append(f'<circle cx="{x(dx):.2f}" cy="{y(dy):.2f}" r="6" fill="{color}"/>')
        out.append(f'<text x="{x(dx)+10:.2f}" y="{y(dy)+offset_y:.2f}">{case}</text>')
        numbers.append((case, f, c))
    out.append('<text x="450" y="407" text-anchor="middle">Change in modeled whole-body horizontal CAM RMS (Counter vs Frame)</text>')
    out.append('<text x="22" y="195" transform="rotate(-90 22 195)" text-anchor="middle">Change in recorded torso XY gyro RMS</text>')
    out.append('</g></svg><figcaption>Model-derived world-horizontal CAM versus measured torso-frame XY gyro norm. Raw-sensor reconstruction; small inner-forward model effects are sensitive to signal reconstruction. This does not equate these frames, estimate contact forces or prove causality.</figcaption></figure>')
    out.append('<div class="scroll"><table><thead><tr><th>Pair</th><th>Full horizontal CAM RMS, kg m²/s</th><th>Torso XY gyro RMS, rad/s</th></tr></thead><tbody>')
    for case, f, c in numbers:
        out.append(f'<tr><td>{case}</td><td>{f["total_rms_xy"]:.4f} → {c["total_rms_xy"]:.4f}</td><td>{f["gyro_rms_xy"]:.5f} → {c["gyro_rms_xy"]:.5f}</td></tr>')
    out.append('</tbody></table></div>')
    return ''.join(out)


def replace_block(text, name, body):
    start, end = f'<!-- {name}_START -->', f'<!-- {name}_END -->'
    assert text.count(start) == text.count(end) == 1
    return text.split(start)[0] + start + '\n' + body + '\n' + end + text.split(end)[1]


def main():
    here = Path(__file__).resolve().parent
    data = json.loads((here / 'results.json').read_text())
    records = {row['run']: row for row in data['records']}
    assert len(records) == 12 and len(data['pairs']) == 6
    body = '<p>Table cells are <strong>Frame → Counter</strong>. Each pair is one comparison; repeats are not six independent trajectory families.</p>'
    body += motion_figure(records)
    body += '<h3>Movement and tracking</h3>' + table(records, [
        ('Move tilt RMS, deg', 'official.move_rms_tilt_deviation_deg', 1, 3),
        ('Onset SO(3) move RMS, deg', 'phases.move.metrics.onset_so3_deg.rms_time', 1, 3),
        ('Timed RMS (native samples), mm', 'official.timed_position_error_rms_m', 1000, 3),
        ('Frozen spatial RMS, mm', 'official.frozen_uniform_rms_m', 1000, 3),
        ('Joint RMS, rad', 'official.configuration_error_rms_rad', 1, 5),
    ])
    body += '<h3>Full record and terminal recovery</h3>' + table(records, [
        ('Full peak drift, deg', 'official.peak_drift_deg', 1, 3),
        ('Terminal standing drift, deg', 'official.residual_drift_deg', 1, 3),
        ('Terminal onset SO(3), deg', 'phases.tail.metrics.onset_so3_deg.mean', 1, 3),
        ('Full-record peak gyro, rad/s', 'official.peak_rate_rad_s', 1, 4),
        ('Hold endpoint RMS, mm', 'official.frozen_hold_error_rms_m', 1000, 3),
    ])
    body += '<h3>Where the disturbance grows</h3>' + table(records, [
        ('Move onset peak, deg', 'phases.move.metrics.onset_so3_deg.peak_abs', 1, 3),
        ('Fade onset peak, deg', 'phases.fade.metrics.onset_so3_deg.peak_abs', 1, 3),
        ('Early postfade onset peak, deg', 'phases.postfade.metrics.onset_so3_deg.peak_abs', 1, 3),
    ])
    body += '<p class="small">Move: observed onset to T; fade: T to T+0.5; early postfade: T+0.5 to T+2; terminal: final second. Timed RMS uses the executed reference clock. Frozen/live FK omit unmeasured base translation, so they are not fixture-relative or world tracking.</p>'
    report = here.parents[1] / 'counter_balance_analysis_iteration_7b.html'
    text = replace_block(report.read_text(), 'HARDWARE_RESULTS', body)
    mechanism = json.loads((here / 'mechanism_summary.json').read_text())
    text = replace_block(text, 'CAM_RESULTS', cam_figure(mechanism))
    report.write_text(text)
    print(f'Rendered six-pair hardware and conditional model figures: {report}')


if __name__ == '__main__':
    main()
