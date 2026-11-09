'''Onset geometry, separate historical cadence and nonactuating clock checks'''

import sys
import json
import runpy
import hashlib
from collections import defaultdict

import numpy as np


def supplement(root, paths, stats):
    sys.path.insert(0, str(root))
    from h12_zmp_benchmark.utility.straight_line_trajectory import (
        SavedLineTrajectory, smoothstep,
    )
    out = {'onset_details': [], 'warning_severity': [], 'deterministic_checks': []}
    new_baseline = defaultdict(lambda: defaultdict(list))
    for path in paths:
        rows = [json.loads(s) for s in path.read_text().splitlines()]
        meta = json.loads(path.with_name('metadata.json').read_text())
        summary = json.loads(path.with_name('summary.json').read_text())
        task = meta['task']
        payload = root / 'data/trajectory_banks' / meta['bank'] / task['npz_path']
        digest = hashlib.sha256(payload.read_bytes()).hexdigest()
        assert digest == task['npz_sha256']
        with np.load(payload, allow_pickle=False) as arrays:
            trajectory = SavedLineTrajectory(task, arrays['s'], arrays['q14'])
        duration = task['motion_duration_s']
        # evaluate the saved analytic reference, not unobserved physical movement
        onset = rows[0]['elapsed_s']
        progress = float(smoothstep(onset / duration)[0])
        q, _, _, pos, _ = trajectory.sample_geometry(progress)
        q0, _, _, pos0, _ = trajectory.sample_geometry(0)
        offsets = np.array([r['snapshot']['arrival_monotonic'] -
                            r['snapshot']['unwrapped_tick']/1000 for r in rows])
        arrival = np.array([r['snapshot']['arrival_monotonic'] for r in rows])
        command = np.array([r['command_monotonic'] for r in rows])
        elapsed = np.array([r['elapsed_s'] for r in rows])
        sequence = np.array([r['control_sequence'] for r in rows])
        if path.parent.parent.name.startswith('B'):
            move = elapsed <= duration
            new_baseline[meta['controller']]['observation_gap'].extend(np.diff(arrival[move]))
            new_baseline[meta['controller']]['command_gap'].extend(np.diff(command[move]))
            new_baseline[meta['controller']]['control'].extend(
                [r['timing']['control_s'] for r, m in zip(rows, move) if m])
        saved_sources = json.loads(path.with_name('provenance.json').read_text())['sources']
        source_checks = {}
        for label, source in saved_sources.items():
            source_path = source.get('path') or ''
            if '/h12_zmp_benchmark/' in source_path:
                local = root / source_path.split('/h12_zmp_benchmark/', 1)[1]
                if local.is_file():
                    source_checks[label] = (
                        hashlib.sha256(local.read_bytes()).hexdigest() == source.get('sha256'))
        d = dict(
            run=path.parent.parent.name, duration=duration, onset_ms=onset*1000,
            onset_motion_time_percent=onset/duration*100,
            onset_reference_progress_percent=progress*100,
            onset_reference_distance_mm=float(np.linalg.norm(pos-pos0)*1000),
            onset_reference_joint_norm_rad=float(np.linalg.norm(q-q0)),
            onset_reference_joint_max_rad=float(np.max(np.abs(q-q0))),
            payload_sha256=digest,
            tick_relative_offset_range_ms=float(np.ptp(offsets)*1000),
            tick_offset_step_ms=stats(np.diff(offsets)),
            all_state_valid=all(r['state_valid'] and r['snapshot']['tick_valid'] for r in rows),
            control_sequence_diff=stats(np.diff(sequence), 1), source_checks=source_checks,
            first_control_loop_gap_ms=rows[0]['control_loop_gap_s']*1000,
            max_row_elapsed_difference_ms=max(abs(r['elapsed']-r['elapsed_s'])*1000
                                              for r in rows), phase_endpoints={})
        for label, end in [('move', duration), ('fade', duration+.5), ('postfade', duration+2)]:
            before = np.flatnonzero(elapsed <= end)[-1]
            after = np.flatnonzero(elapsed > end)[0]
            d['phase_endpoints'][label] = dict(
                previous_s=float(elapsed[before]), next_s=float(elapsed[after]),
                straddle_gap_ms=float((arrival[after]-arrival[before])*1000))
        assert d['all_state_valid']
        assert d['control_sequence_diff']['min'] == d['control_sequence_diff']['max'] == 1
        assert all(source_checks.values())
        out['onset_details'].append(d)
        if not summary['run_complete']:
            assert not summary['coverage']['onset_covered']
            onset_tolerance = task.get('onset_tolerance_s', .05)
            out['warning_severity'].append(dict(
                run=d['run'], reason='onset_only',
                threshold_ms=onset_tolerance*1000,
                excess_ms=(onset-onset_tolerance)*1000,
                onset_reference_progress_percent=d['onset_reference_progress_percent'],
                onset_reference_distance_mm=d['onset_reference_distance_mm'],
                onset_reference_joint_max_rad=d['onset_reference_joint_max_rad']))
    historical = defaultdict(lambda: defaultdict(list))
    selected = []
    selectors = {
        ('fame_finetune_directional', 'right_diagonal_outward_up'),
        ('almi_finetune_directional', 'right_diagonal_outward_up'),
        ('fame_finetune_overhang', 'left_overhang_sideways'),
        ('almi_finetune_overhang', 'right_overhang_inner_forward'),
    }
    for group, target in sorted(selectors):
        for path in sorted((root / 'runs/real_experiments/traj' / group).rglob('arm_target.jsonl')):
            if path.parent.name != target:
                continue
            meta = json.loads(path.with_name('metadata.json').read_text())
            summary = json.loads(path.with_name('summary.json').read_text())
            duration = meta['task']['motion_duration_s']
            rows = [json.loads(s) for s in path.read_text().splitlines()]
            rows = [r for r in rows if 0 <= r['elapsed_s'] <= duration]
            arrival = np.array([r['snapshot']['arrival_monotonic'] for r in rows])
            command = np.array([r['command_monotonic'] for r in rows])
            control = np.array([r['timing']['control_s'] for r in rows])
            controller = meta['controller']
            historical[controller]['observation_gap'].extend(np.diff(arrival))
            historical[controller]['command_gap'].extend(np.diff(command))
            historical[controller]['control'].extend(control)
            selected.append(dict(
                path=str(path.relative_to(root)), controller=controller,
                complete=summary['run_complete'], rows=len(rows),
                onset_ms=rows[0]['elapsed_s']*1000,
                observation_gap_ms=stats(np.diff(arrival)),
                command_gap_ms=stats(np.diff(command)), control_ms=stats(control)))
    assert len(selected) == 8
    out['historical_separate'] = dict(
        runs=selected,
        pooled={g: {k: stats(v) for k, v in d.items()} for g, d in historical.items()})
    out['new_B1_to_B4_only'] = {
        g: {k: stats(v) for k, v in d.items()} for g, d in new_baseline.items()}
    tests = runpy.run_path(str(root / 'tests/test_real_experiment.py'))
    for validate_delay, start_delay in [(0, 0), (.2, 0), (0, .051), (.2, .051)]:
        trial, clock, adapter, _, recorder = tests['make_trial'](controller='counter')
        tests['home'](trial, clock)
        validate, start = adapter.validate_motion, recorder.start
        def delayed_validate(*args):
            value = validate(*args)
            clock.now += validate_delay
            return value
        def delayed_start(*args, **kwargs):
            value = start(*args, **kwargs)
            clock.now += start_delay
            return value
        adapter.validate_motion, recorder.start = delayed_validate, delayed_start
        trial.command('START')
        clock.now = max(clock.now+.002, trial._next_apply_at)
        trial.tick()
        row = recorder.samples[0]
        out['deterministic_checks'].append(dict(
            validation_delay_ms=validate_delay*1000, recorder_start_delay_ms=start_delay*1000,
            first_observation_ms=(row['snapshot']['arrival_monotonic']-trial.started_at)*1000,
            skips=row['control_slots_skipped']))
    return out
