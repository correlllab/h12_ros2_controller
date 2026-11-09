'''Reproduce the November 6 timing audit offline from the benchmark root

Run with the benchmark .venv Python; --output defaults to /tmp/kilo.
The neighboring supplement uses saved trajectory payloads and fake-clock tests.
No hardware interfaces are instantiated and no production files are changed.
'''

import json
import argparse
from pathlib import Path
from collections import defaultdict

import numpy as np

from audit_timing_supplement import supplement

ROOT = Path.cwd()
CAMPAIGN = Path('runs/real_experiments/iter7_baseline_20261106_190035')


def stats(values, scale=1000):
    a = np.asarray([v for v in values if v is not None], dtype=float) * scale
    return dict(n=len(a), **({k: float(v) for k, v in zip(
        ['min', 'mean', 'p50', 'p95', 'p99', 'max'],
        [a.min(), a.mean(), *np.quantile(a, [.5, .95, .99]), a.max()])} if len(a) else {}))


def audit(path):
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    meta = json.loads(path.with_name('metadata.json').read_text())
    summary = json.loads(path.with_name('summary.json').read_text())
    events = [json.loads(line) for line in
              (path.parent.parent / 'events.jsonl').read_text().splitlines()]
    cov = summary['coverage']
    start = cov['execution_start_monotonic']
    # retain the original audit's coverage-clock subtraction exactly
    duration = cov['required_end_monotonic'] - start
    arrival = np.array([r['snapshot']['arrival_monotonic'] for r in rows])
    command = np.array([r['command_monotonic'] for r in rows])
    elapsed = arrival - start
    ctime = command - start
    commit = np.array([r['buffer_commit_monotonic'] for r in rows])
    control = np.array([r['timing']['control_s'] for r in rows])
    seq = np.array([r['snapshot']['sequence'] for r in rows])
    ticks = np.array([r['snapshot']['unwrapped_tick'] for r in rows])
    phases = [('move', 0, duration), ('fade', duration, duration + .5),
              ('early_postfade', duration + .5, duration + 2),
              ('late_hold', duration + 2, float('inf'))]
    gaps = []
    for i in np.flatnonzero(np.diff(arrival) > .05) + 1:
        gaps.append(dict(
            row=int(i + 1), prev_elapsed=float(elapsed[i-1]), elapsed=float(elapsed[i]),
            gap_ms=float((arrival[i]-arrival[i-1])*1000),
            command_gap_ms=float((command[i]-command[i-1])*1000),
            arrival_to_command_ms=float((command[i]-arrival[i])*1000),
            control_ms=float(control[i]*1000), prev_control_ms=float(control[i-1]*1000),
            skipped=rows[i]['control_slots_skipped'],
            prev_skipped=rows[i-1]['control_slots_skipped'],
            tick_delta=int(ticks[i]-ticks[i-1]), sequence_delta=int(seq[i]-seq[i-1]),
            phase=next(name for name, lo, hi in phases if lo <= elapsed[i] < hi)))
    data = dict(
        run=path.parent.parent.name, target=path.parent.name, rows=len(rows),
        controller=meta['controller'], policy=meta.get('policy'), duration=duration,
        complete=summary['run_complete'], status=summary['status'],
        reason=summary['finish_reason'], interrupted=summary['interrupted'],
        final_valid=summary['final_state_valid'], clock_fault=summary['clock_fault'],
        coverage=cov, first_observation_ms=float(elapsed[0]*1000),
        first_command_ms=float(ctime[0]*1000),
        first_buffer_commit_ms=float((commit[0]-start)*1000),
        start_authorized_ms=next((e['arrival_monotonic']-start)*1000
                                for e in events if e.get('event') == 'start_authorized'),
        skipped_measurements=summary['skipped_measurements'],
        unknown_publish_rows=sum(r['timing'].get('publish_s') is None for r in rows),
        unknown_lateness_rows=sum(r['timing'].get('publication_lateness_s') is None
                                  for r in rows),
        gap_events=gaps, phases={})
    arrays = dict(
        observation_gap=np.r_[np.nan, np.diff(arrival)],
        command_gap=np.r_[np.nan, np.diff(command)], control=control,
        arrival_to_command=command-arrival,
        arrival_to_record=np.array([r['state_age_s'] for r in rows]),
        command_to_commit=commit-command,
        commit_to_host=np.array([r['host_monotonic'] for r in rows])-commit,
        tick_gap=np.r_[np.nan, np.diff(ticks)/1000],
        sequence_gap=np.r_[np.nan, np.diff(seq)])
    for name, lo, hi in [('all', 0, float('inf'))] + phases:
        mask = (elapsed >= lo) & (elapsed < hi)
        # require both endpoints within the phase for interval statistics
        interval = mask & np.r_[False, mask[:-1]]
        d = {}
        for key in ('observation_gap', 'command_gap', 'control'):
            values = arrays[key][interval if key.endswith('_gap') else mask]
            d[key] = stats(values[np.isfinite(values)])
        d.update(
            rows=int(mask.sum()),
            skipped_slots=sum(r['control_slots_skipped'] for r, m in zip(rows, mask) if m),
            skip_rows=sum(r['control_slots_skipped'] > 0 for r, m in zip(rows, mask) if m),
            observation_gaps_gt50=int(np.sum(arrays['observation_gap'][interval] > .05)),
            command_gaps_gt50=int(np.sum(arrays['command_gap'][interval] > .05)))
        data['phases'][name] = d
    data['skip_events'] = [dict(
        row=i+1, elapsed=float(elapsed[i]), command_elapsed=float(ctime[i]),
        slots=r['control_slots_skipped'], control_ms=float(control[i]*1000),
        command_gap_ms=None if i == 0 else float((command[i]-command[i-1])*1000))
        for i, r in enumerate(rows) if r['control_slots_skipped']]
    counters = summary['publisher_counter_deltas']
    assert all(counters[k] is None for k in
               ('application_late_count', 'write_attempts', 'write_successes', 'write_late_count'))
    assert all(r['command'].get('last_published') is None for r in rows)
    return data, arrays, elapsed


def main(args):
    results = []
    pooled = defaultdict(lambda: defaultdict(list))
    paths = sorted((ROOT / CAMPAIGN).rglob('arm_target.jsonl'))
    if len(paths) != 12:
        raise ValueError('Run from benchmark root with all 12 campaign trials available')
    for path in paths:
        data, arrays, elapsed = audit(path)
        results.append(data)
        for name, lo, hi in [('all', 0, float('inf')), ('move', 0, data['duration'])]:
            mask = (elapsed >= lo) & (elapsed < hi)
            interval = mask & np.r_[False, mask[:-1]]
            for key, values in arrays.items():
                values = values[interval if key.endswith('_gap') else mask]
                pooled[data['controller']+'_'+name][key].extend(
                    values[np.isfinite(values)].tolist())
    output = dict(
        campaign=str(CAMPAIGN),
        conventions={
            'statistics': 'milliseconds except sequence_gap (consumptions); no rounding',
            'phases': 'observation clock: [0,T), [T,T+.5), [T+.5,T+2), [T+2,end]',
            'intervals': 'both endpoints inside phase; boundary-straddling intervals excluded',
            'duration': 'coverage end minus epoch; onset geometry uses exact saved task duration',
            'timestamps': 'arrival is local take_latest consumption, not packet/source timestamp',
            'control': 'foreground adapter application, not pure solver or asynchronous publication',
            'publisher': 'all rows lack publish/lateness/last_published; endpoint writer counters unknown',
            'interpretation': 'data-quality evidence only; no physical safety or network causality claim',
        },
        runs=results,
        pooled={group: {k: stats(v, 1 if k == 'sequence_gap' else 1000)
                        for k, v in values.items()} for group, values in pooled.items()},
        **supplement(ROOT, paths, stats))
    assert sum(r['rows'] for r in results) == 4035
    assert sum(not r['complete'] for r in results) == 2
    assert all(r['coverage']['terminal_covered'] and r['final_valid']
               and r['reason'] == 'auto_stop' and r['clock_fault'] is None for r in results)
    assert all(r['skipped_measurements'] == {'duplicate': 0, 'prestart': 0} for r in results)
    assert sum(len(r['gap_events']) for r in results) == 1
    assert all(r['unknown_publish_rows'] == r['unknown_lateness_rows'] == r['rows']
               for r in results)
    args.output.write_text(json.dumps(output, indent=2, allow_nan=False)+'\n')
    print(f'Wrote {args.output}: 12 trials, 4035 rows, 2 onset warnings, 1 late-hold gap')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('/tmp/kilo/timing_summary.json'))
    main(parser.parse_args())
