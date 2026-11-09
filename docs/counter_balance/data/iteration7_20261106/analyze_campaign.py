'''Reproduce the twelve-run Nov 6 campaign and save sibling result evidence'''
import csv
import sys
import json
import hashlib
import argparse
from pathlib import Path

import numpy as np


def read_json(path):
    return json.loads(path.read_text())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(args):
    root = args.root.resolve()
    sys.path.insert(0, str(root))
    from h12_zmp_benchmark.experiment import analyze_real_runs as a
    from h12_zmp_benchmark.utility.straight_line_trajectory import load_trajectory

    campaign = root / args.campaign
    cache = {}

    def reduce(trial, cohort):
        meta = read_json(trial / 'metadata.json')
        summary = read_json(trial / 'summary.json')
        provenance = read_json(trial / 'provenance.json')
        task = meta['task']
        bank = root / 'data/trajectory_banks' / task['bank']
        reference_path = bank / (task['id'] + '.json')
        trajectory = load_trajectory(reference_path)
        expected = provenance['sources'].get('trajectory', {}).get('sha256')
        assert expected is None or sha(reference_path) == expected, (trial, 'trajectory hash')
        assert task['npz_sha256'] == trajectory.metadata['npz_sha256']
        for key in ('line', 'nominal_duration', 'joint_names', 'moving_arm'):
            assert task[key] == trajectory.metadata[key], (trial, key)
        duration = float(task['move_duration'])
        assert np.isclose(duration, trajectory.nominal_duration * .8)
        trajectory.set_acceleration_limit(task['speed_acceleration_limit'])
        raw = trial / 'arm_target.jsonl'
        rows = [json.loads(line) for line in raw.read_text().splitlines() if line.strip()]
        samples = a.read_samples(trial)
        time = samples['time']
        motion = time <= duration
        terminal = time >= time[-1] - 1
        start = meta['execution_start_monotonic']
        elapsed = np.array([r['snapshot']['arrival_monotonic'] - start for r in rows])
        assert np.max(np.abs(elapsed - time)) < 1e-9
        assert np.all(np.diff(time) > 0)
        assert np.allclose(samples['standing'], read_json(trial / 'standing_capture.json')['R0'])
        result = a.flatten(a.reduce_run(trial.parent))
        # interrupted also means onset-incomplete here, not necessarily operator stop
        result['operator_stopped'] = bool(summary['interrupted'] and summary['finish_reason'] != 'auto_stop')
        result['record_truncated_by_operator'] = result['operator_stopped']
        result['raw_id'] = str(trial.relative_to(root))
        result['cohort'] = cohort
        result['case'] = trial.parent.name.split('_')[0] if cohort == 'new' else None
        result['created_utc'] = meta['created_utc']
        result['raw_flags'] = {key: summary.get(key) for key in (
            'run_complete', 'window_complete', 'interrupted', 'finish_reason', 'status',
            'clock_fault', 'final_state_valid', 'capture_available', 'recording_started',
            'actuation_acknowledged', 'dds_delivery_confirmed', 'physical_classification_available')}
        result['coverage'] = summary.get('coverage')
        result['first_elapsed_s'] = float(time[0])
        result['observed_motion_window_s'] = [float(time[motion][0]), float(time[motion][-1])]
        result['initial_drift_deg'] = float(np.degrees(samples['drift'][0]))
        result['motion_peak_tilt_deg'] = float(np.degrees(samples['tilt'][motion].max()))
        result['motion_peak_tilt_deviation_deg'] = float(np.degrees(
            np.abs(samples['tilt'][motion] - samples['tilt'][0]).max()))
        result['motion_peak_rate_rad_s'] = float(samples['rate'][motion].max())
        result['motion_rms_rate_rad_s'] = float(np.sqrt(a.time_weighted(
            time[motion], samples['rate'][motion], 2)))
        result['final_drift_deg'] = float(np.degrees(samples['drift'][-1]))
        result['motion_rms_standing_drift_deg'] = float(np.degrees(np.sqrt(a.time_weighted(
            time[motion], samples['drift'][motion], 2))))
        rotations = samples['pelvis']
        onset = np.array([a.rotation_angle(rotations[0].T @ r) for r in rotations])
        gravity = rotations[:, 2, :]
        gravity_angle = np.arccos(np.clip(gravity @ gravity[0], -1, 1))
        result['phase_peaks'] = {}
        for phase, mask in (
                ('move', motion), ('fade', (time > duration) & (time <= duration+.5)),
                ('early_postfade', (time > duration+.5) & (time <= duration+2)),
                ('late_hold', time > duration+2)):
            entry = {}
            for name, signal, scale in (
                    ('standing_drift_deg', samples['drift'], 180/np.pi),
                    ('onset_so3_deg', onset, 180/np.pi),
                    ('onset_gravity_deg', gravity_angle, 180/np.pi),
                    ('tilt_deg', samples['tilt'], 180/np.pi),
                    ('gyro_rad_s', samples['rate'], 1)):
                indices = np.flatnonzero(mask)
                index = indices[np.argmax(signal[mask])]
                entry[name] = {'peak': float(signal[index]*scale),
                               'elapsed_s': float(time[index]), 'after_T_s': float(time[index]-duration)}
            result['phase_peaks'][phase] = entry
        for name, signal in (('onset_so3', onset), ('onset_gravity', gravity_angle)):
            result[name + '_motion_rms_deg'] = float(np.degrees(np.sqrt(
                a.time_weighted(time[motion], signal[motion], 2))))
            result[name + '_motion_peak_deg'] = float(np.degrees(signal[motion].max()))
            result[name + '_full_peak_deg'] = float(np.degrees(signal.max()))
            result[name + '_terminal_mean_deg'] = float(np.degrees(signal[terminal].mean()))
        q14, _, _, reference_p, _ = trajectory.sample(time, duration=duration)
        arm_offset = 0 if trajectory.moving_arm == 'left' else 7
        qerror = np.linalg.norm(samples['q'][:, 13+arm_offset:20+arm_offset]
                                - q14[:, arm_offset:arm_offset+7], axis=1)
        wrist = samples[trajectory.moving_arm + '_wrist']
        perror = np.linalg.norm(wrist - reference_p, axis=1)
        assert np.isclose(np.sqrt(np.mean(qerror[motion]**2)), result['configuration_error_rms_rad'])
        assert np.isclose(np.sqrt(np.mean(perror[motion]**2)), result['timed_position_error_rms_m'])
        command_time = np.array([r['command_monotonic'] - start for r in rows])
        desired_q = trajectory.sample(command_time, duration=duration)[0]
        result['command_clock_q_max_abs_error_rad'] = float(np.max(np.abs(
            desired_q[:, arm_offset:arm_offset+7] - samples['desired_q'][:, arm_offset:arm_offset+7])))
        result['online_timed_rms_m'] = summary['diagnostics']['frozen_pelvis_fk']['timed_position_error_m']['rms']
        result['online_q_rms_rad'] = summary['diagnostics']['moving_arm_configuration_error_rad']['rms']
        result['online_sample_timed_max_recompute_error_m'] = float(max(
            abs(r['model_metrics']['frozen_pelvis_fk']['timed_position_error_m'] - perror[i])
            for i, r in enumerate(rows)))
        checks = {}
        for name, info in provenance['sources'].items():
            path = info.get('path')
            local = root / path.split('/h12_zmp_benchmark/', 1)[-1] if path and '/h12_zmp_benchmark/' in path else None
            checks[name] = {'saved_sha256': info.get('sha256'),
                            'current_sha256': sha(local) if local and local.is_file() else None}
            checks[name]['matches'] = (checks[name]['saved_sha256'] == checks[name]['current_sha256']
                                      if checks[name]['current_sha256'] is not None else None)
        result['provenance'] = {
            'raw_sha256': sha(raw), 'sources': checks, 'git': provenance['git'],
            'resolved_config_sha256': provenance['resolved_config_sha256'],
            'pipeline_revision': meta.get('pipeline_revision'),
            'measurement_clock': meta.get('measurement_clock'),
            'external_policy_declaration_only': meta.get('external_policy_declaration_only'),
            'external_policy_version': meta.get('external_policy_version'),
            'task_duration_s': duration, 'nominal_duration_s': trajectory.nominal_duration,
            'config_target_duration_s': meta['config']['target_profile']['move_duration'],
            'saved_controller': meta['resolved_config']['controller'],
            'payload_sha256': trajectory.metadata['npz_sha256'],
        }
        result['quality'] = {
            'state_valid_false_count': sum(r.get('state_valid') is False for r in rows),
            'all_zero_q_rows': int(np.sum(np.all(samples['q'] == 0, axis=1))),
            'all_zero_joint_columns': np.where(np.all(samples['q'] == 0, axis=0))[0].tolist(),
            'max_observation_gap_ms': float(np.diff(time).max()*1000),
            'motion_max_observation_gap_ms': float(np.diff(time[motion]).max()*1000),
            'elapsed_clock_max_error_s': float(np.max(np.abs(time-elapsed))),
            'terminal_sample_count': int(terminal.sum()),
            'terminal_max_gap_ms': float(np.diff(time[terminal]).max()*1000),
            'terminal_continuity_warning': bool(np.diff(time[terminal]).max() > .05),
        }
        result['directory_inventory'] = {p.name: p.stat().st_size for p in sorted(trial.iterdir()) if p.is_file()}
        cache[result['raw_id']] = (samples, trajectory, onset, gravity_angle)
        return result

    records = [reduce(raw.parent, 'new') for raw in sorted(campaign.rglob('arm_target.jsonl'))]
    assert len(records) == 12
    metric_keys = [
        'frozen_uniform_rms_m', 'live_uniform_rms_m', 'frozen_segment_max_m',
        'frozen_endpoint_error_m', 'frozen_hold_error_rms_m', 'frozen_hold_creep_m',
        'timed_position_error_rms_m', 'configuration_error_rms_rad',
        'wrist_orientation_error_rms_rad', 'motion_peak_drift_deg',
        'move_rms_tilt_deviation_deg', 'motion_peak_tilt_deviation_deg',
        'motion_peak_tilt_deg', 'peak_drift_deg', 'residual_drift_deg',
        'final_drift_deg', 'peak_rate_rad_s', 'motion_peak_rate_rad_s',
        'motion_rms_rate_rad_s', 'terminal_rate_rad_s',
        'initial_drift_deg', 'initial_tilt_deg', 'onset_so3_motion_rms_deg',
        'onset_gravity_motion_rms_deg', 'onset_so3_full_peak_deg',
        'onset_so3_terminal_mean_deg', 'onset_gravity_terminal_mean_deg']

    def pair(frame, counter, label):
        for key in ('policy', 'bank', 'target', 'speed', 'move_duration_s'):
            assert frame[key] == counter[key], (label, key)
        result = {'pair': label, 'policy': frame['policy'], 'bank': frame['bank'],
                  'target': frame['target'], 'frame_raw_id': frame['raw_id'],
                  'counter_raw_id': counter['raw_id'],
                  'order': 'F-C' if frame['created_utc'] < counter['created_utc'] else 'C-F',
                  'strict_complete': frame['run_complete'] and counter['run_complete'],
                  'classification': [frame['classification'], counter['classification']],
                  'metrics': {key: {'frame': frame[key], 'counter': counter[key],
                                    'delta': counter[key]-frame[key]} for key in metric_keys}}
        # common observed interval; no extrapolation over the missing onset
        sf, trajectory, _, _ = cache[frame['raw_id']]
        sc, _, _, _ = cache[counter['raw_id']]
        grid = np.linspace(max(sf['time'][0], sc['time'][0]), frame['move_duration_s'], 401)
        qo, _, _, po, _ = trajectory.sample(grid, duration=frame['move_duration_s'])
        offset = 0 if trajectory.moving_arm == 'left' else 7
        common = {}
        for controller, s in (('frame', sf), ('counter', sc)):
            q = np.stack([np.interp(grid, s['time'], s['q'][:, i]) for i in range(13+offset, 20+offset)], axis=1)
            p = np.stack([np.interp(grid, s['time'], s[trajectory.moving_arm+'_wrist'][:, i]) for i in range(3)], axis=1)
            common[controller] = {
                'q_rms_rad': float(np.sqrt(a.time_weighted(grid, np.linalg.norm(q-qo[:, offset:offset+7], axis=1), 2))),
                'timed_rms_m': float(np.sqrt(a.time_weighted(grid, np.linalg.norm(p-po, axis=1), 2))),
                'tilt_rms_deg': float(np.degrees(np.sqrt(a.time_weighted(grid,
                    np.interp(grid, s['time'], s['tilt']-s['tilt'][0]), 2))))}
        result['common_grid_sensitivity'] = {'window_s': [float(grid[0]), float(grid[-1])], **common}
        grid = np.linspace(frame['move_duration_s']+7, frame['move_duration_s']+9, 401)
        clean_tail = {'window_s': [float(grid[0]), float(grid[-1])]}
        for controller, record in (('frame', frame), ('counter', counter)):
            s, trajectory, onset, gravity_angle = cache[record['raw_id']]
            t = s['time']
            covering = (t >= t[max(0, np.searchsorted(t, grid[0])-1)]) & (t <= t[min(len(t)-1, np.searchsorted(t, grid[-1]))])
            max_gap = float(np.diff(t[covering]).max())
            endpoint = trajectory.sample(record['move_duration_s'], duration=record['move_duration_s'])[3]
            wrist = s[trajectory.moving_arm+'_wrist']
            clean_tail[controller] = {
                'max_gap_ms': max_gap*1000, 'continuous_at_50ms': max_gap <= .05,
                'drift_mean_deg': float(np.degrees(a.time_weighted(grid, np.interp(grid, t, s['drift'])))),
                'onset_so3_mean_deg': float(np.degrees(a.time_weighted(grid, np.interp(grid, t, onset)))),
                'onset_gravity_mean_deg': float(np.degrees(a.time_weighted(grid, np.interp(grid, t, gravity_angle)))),
                'gyro_mean_rad_s': a.time_weighted(grid, np.interp(grid, t, s['rate'])),
                'frozen_endpoint_rms_m': float(np.sqrt(a.time_weighted(grid, np.interp(grid, t, np.linalg.norm(wrist-endpoint, axis=1)), 2))),
            }
        result['continuous_late_hold_sensitivity'] = clean_tail
        return result

    pairs = []
    for case in ('B1', 'B2', 'B3', 'B4', 'R3', 'R4'):
        selected = [r for r in records if r['case'] == case]
        assert len(selected) == 2
        frame = next(r for r in selected if r['controller'] == 'frame')
        counter = next(r for r in selected if r['controller'] == 'counter')
        pairs.append(pair(frame, counter, case))
    history = []
    historical_pairs = []
    for base in pairs[:4]:
        group = ('fame' if base['policy'] == 'fame' else 'almi') + '_finetune_' + ('directional' if base['bank'] == 'directional_traj' else 'overhang')
        parent = root / 'runs/real_experiments/traj' / group
        matches = sorted(parent.glob('*/' + base['target'] + '/arm_target.jsonl'))
        assert len(matches) == 2, (group, base['target'], len(matches))
        subset = [reduce(raw.parent, 'historical_matching_only') for raw in matches]
        history.extend(subset)
        historical_pairs.append(pair(next(r for r in subset if r['controller'] == 'frame'),
                                     next(r for r in subset if r['controller'] == 'counter'), 'historical_'+base['pair']))

    def aggregate(selected):
        return {'pairs': [p['pair'] for p in selected], 'n': len(selected), 'metrics': {
            key: {'frame_mean': float(np.mean([p['metrics'][key]['frame'] for p in selected])),
                  'counter_mean': float(np.mean([p['metrics'][key]['counter'] for p in selected])),
                  'mean_delta': float(np.mean([p['metrics'][key]['delta'] for p in selected])),
                  'delta_range': [float(min(p['metrics'][key]['delta'] for p in selected)),
                                  float(max(p['metrics'][key]['delta'] for p in selected))],
                  'counter_lower': sum(p['metrics'][key]['delta'] < 0 for p in selected)}
            for key in metric_keys}}
    groups = {
        'baseline_four_cases': aggregate(pairs[:4]),
        'repeat_overhang_two_cases': aggregate(pairs[4:]),
        'all_six_pairs_repeat_weighted_descriptive_only': aggregate(pairs),
        'strict_complete_sensitivity': aggregate([p for p in pairs if p['strict_complete']]),
        'fame_sideways_two_repeats': aggregate([p for p in pairs if p['pair'] in ('B3', 'R3')]),
        'almi_inner_forward_two_repeats': aggregate([p for p in pairs if p['pair'] in ('B4', 'R4')]),
        'exclude_gap_pair_terminal_sensitivity': aggregate([p for p in pairs if p['pair'] != 'B4']),
    }
    report = {'campaign': args.campaign, 'rules': {
        'pairing': 'same policy/bank/target/speed and explicit B1,B2,B3,B4,R3,R4; never collapse duplicate targets',
        'primary_inclusion': 'all12, including B3_counter and R3_counter onset warnings, flags unchanged',
        'strict_sensitivity': 'only pairs with both original run_complete true',
        'balance': 'official reducer; standing R0 drift; scalar tilt deviation from first observed task sample',
        'spatial': '1001 uniform arc-length samples of observed move wrist path; frozen and live rotation gauges omit world translation',
        'timed': 'trajectory sampled on observation arrival clock at saved executed duration; official sample RMS',
        'supplementary': 'onset SO3/gravity reference first recorded task sample; observed-window trapezoid RMS',
        'inference': 'none; no pooling new with historical, no policy causal effect under blocked FAME then ALMI order',
    }, 'records': records, 'pairs': pairs, 'aggregates': groups,
        'historical_matching_records': history, 'historical_matching_pairs': historical_pairs}
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False))
    csv_path = args.output.with_suffix('.csv')
    flattened = [{k: v for k, v in r.items() if not isinstance(v, (dict, list))} for r in records+history]
    fields = list(dict.fromkeys(k for row in flattened for k in row))
    with csv_path.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(flattened)
    for p in pairs+historical_pairs:
        print(p['pair'], p['order'], p['classification'])
    print('WROTE', args.output, csv_path)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path.cwd())
    parser.add_argument('--campaign', default='runs/real_experiments/iter7_baseline_20261106_190035')
    parser.add_argument('--output', type=Path, default=Path(__file__).with_name('results.json'))
    main(parser.parse_args())
