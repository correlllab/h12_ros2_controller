import json
import argparse
import hashlib
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from collections import Counter

import numpy as np
from scipy.optimize import lsq_linear


def rms(values):
    values = np.asarray(values)
    return float(np.sqrt(np.mean(values ** 2))) if values.size else None


def summarize(samples):
    if not samples:
        return {'n': 0}
    result = {'n': len(samples)}
    for key in ('status', 'backend'):
        result[key] = dict(Counter(s[key] for s in samples))
    for key in ('published', 'accepted', 'retry', 'collision_flag', 'clipped',
                'bound_active', 'rate_bound_active', 'excursion_bound_active', 'requested_nonzero', 'applied_nonzero',
                'lower_limit_invalid', 'all_scales_limit_invalid',
                'moving_dq_clipped'):
        result[key] = sum(s[key] for s in samples)
    result['limit_causes'] = dict(Counter(j for s in samples for j in s['limit_causes']))
    result['remaining_invalid_not_proven_limit'] = sum(
        s['status'] == 'counter_candidate_invalid' and not s['all_scales_limit_invalid']
        for s in samples)
    for key in ('requested', 'applied', 'measured'):
        result[key + '_rms'] = rms([s[key] for s in samples])
        result[key + '_peak_by_joint'] = np.max(np.abs([s[key] for s in samples]), axis=0).tolist()
    for key in ('buffer_dq_error', 'buffer_q_law_error', 'moving_q_buffer_error',
                'moving_dq_clip_peak', 'limit_violation_peak'):
        result[key] = max(s[key] for s in samples)
    result['excursion_peak_by_joint'] = np.max(np.abs([s['excursion'] for s in samples]), axis=0).tolist()
    result['excursion_mean_by_joint'] = np.mean([s['excursion'] for s in samples], axis=0).tolist()
    result['applied_mean_by_joint'] = np.mean([s['applied'] for s in samples], axis=0).tolist()
    result['measured_mean_by_joint'] = np.mean([s['measured'] for s in samples], axis=0).tolist()
    result['counter_feedforward_tau_cap_hits'] = sum(s['tau_cap_hit'] for s in samples)
    result['counter_feedforward_tau_peak'] = np.max(np.abs([s['tau_ff'] for s in samples]), axis=0).tolist()
    result['com_error_rms'] = rms([s['com_error'] for s in samples])
    for which in ('requested', 'applied', 'measured', 'task_only', 'posture005'):
        denom = sum(s['task_target_ss'] for s in samples)
        result['task_residual_ratio_' + which] = (
            float(np.sqrt(sum(s['task_ss_' + which] for s in samples) / denom))
            if denom > 1e-15 else None)
        result['full_cost_' + which] = sum(s['full_ss_' + which] for s in samples)
    for key in ('com', 'momentum'):
        for which in ('requested', 'applied', 'measured'):
            result[key + '_residual_rms_' + which] = rms([s[key + '_residual_' + which] for s in samples])
    actual_denom = sum(s['actual_task_target_ss'] for s in samples)
    result['measured_task_residual_ratio_measured_moving_rhs'] = (
        float(np.sqrt(sum(s['actual_task_ss_measured'] for s in samples) / actual_denom))
        if actual_denom > 1e-15 else None)
    return result


def main(args):
    root = args.root.resolve()
    run = root / 'runs/real_experiments/iter7_baseline_20261106_190035'
    controller = root / 'submodules/h12_ros2_controller'
    sys.path.insert(0, str(controller))
    from h12_ros2_controller.utility.controller_config import load_controller_config
    current_config = json.loads(json.dumps(
        load_controller_config('real_benchmark_upper.yaml', config_dir=controller / 'config'),
        default=lambda value: value.tolist()))
    names = [side + '_' + name + '_joint' for side in ('left', 'right')
             for name in ('hip_yaw', 'hip_pitch', 'hip_roll', 'knee', 'ankle_pitch', 'ankle_roll')]
    names += ['torso_joint'] + [side + '_' + name + '_joint' for side in ('left', 'right')
             for name in ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')]
    urdf = controller / 'submodules/CL_Assets/ros_assets/h1_2_magpie.urdf'
    joint_limits = {j.attrib['name']: j.find('limit').attrib for j in ET.parse(urdf).getroot().findall('joint')
                    if j.find('limit') is not None}
    urdf_limits = np.array([[float(joint_limits[name]['lower']), float(joint_limits[name]['upper'])] for name in names])
    results = {'definition': 'raw observed rows, no extrapolation; nonzero threshold 1e-6 rad/s; task residual uses weighted com+momentum blocks, pooled sum of squared residuals/target; balance_scale=0 has no task residual ratio; measured is simultaneous pre-command state NOT causal response; bounded alternatives are same-state algebra only', 'runs': {}}
    for path in sorted(run.glob('*counter/*/arm_target.jsonl')):
        rows = [json.loads(line) for line in path.read_text().splitlines()]
        meta = json.loads(path.with_name('metadata.json').read_text())
        resolved = json.loads(path.with_name('resolved_config.yaml').read_text())['controller']
        provenance = json.loads(path.with_name('provenance.json').read_text())
        ident = path.parent.parent.name
        duration = meta['task']['move_duration']
        counter_start = 13 if meta['task']['moving_arm'] == 'right' else 20
        ci = np.arange(counter_start, counter_start + 4)
        cw = np.arange(counter_start + 4, counter_start + 7)
        mi = np.arange(20, 27) if counter_start == 13 else np.arange(13, 20)
        samples = []
        for i, row in enumerate(rows):
            diag = row['diagnostics']['controller_diagnostics']
            prepared = diag['nominal_prepared_context']
            context = prepared['context']
            pub = prepared['publication']
            ref = prepared['references']
            q = np.array(context['motor_q'])
            limits = np.array(pub['limits']['q_clip_limits'])
            limits[:, 0] = np.maximum(limits[:, 0], urdf_limits[:, 0])
            limits[:, 1] = np.minimum(limits[:, 1], urdf_limits[:, 1])
            req = np.array(diag['requested_counter_dq'])
            app = np.array(diag['applied_counter_dq'])
            measured = np.array(prepared['observation']['dq'])[ci]
            buf = row['command']['buffer']
            bq, bdq = np.array(buf['q']), np.array(buf['dq'])
            full_candidate = q.copy()
            full_candidate[13:] = context['arm_q']
            violations = []
            for scale in pub['backtrack_scales']:
                candidate = full_candidate.copy()
                candidate[ci] = np.array(context['counter_q']) + scale * pub['dt'] * req
                candidate[cw] = ref['counter_wrist_ref']
                bad = (candidate < limits[:, 0]) | (candidate > limits[:, 1])
                violations.append(bad)
            lower_bad = (q[:13] < limits[:13, 0]) | (q[:13] > limits[:13, 1])
            lo = np.array(diag['nominal_problem_lower'])
            hi = np.array(diag['nominal_problem_upper'])
            rate_cap = np.minimum(np.array(pub['limits']['dq_clip_limits'])[ci], pub['dq_lim'])
            max_excursion = np.array(resolved['reactive_counter_balance']['max_excursion'])
            excursion = np.array(context['counter_q']) - ref['q_counter_ref']
            excursion_lower = (-max_excursion - excursion) / pub['dt']
            excursion_upper = (max_excursion - excursion) / pub['dt']
            a = np.array(diag['nominal_problem_matrix'])
            b = np.array(diag['nominal_problem_target'])
            independent = lsq_linear(a, b, bounds=(lo, hi), method='bvls', tol=1e-12).x
            assert np.max(abs(independent - req)) < 1e-9
            terms = diag['nominal_problem_terms']
            s = terms['balance_scale']
            task_a = np.vstack((np.sqrt(s * terms['com_weight']) * np.array(terms['com_counter']) / terms['com_velocity_scale'],
                                np.sqrt(s * terms['momentum_weight']) * np.array(terms['momentum_counter']) / terms['momentum_scale']))
            task_b = np.r_[np.sqrt(s * terms['com_weight']) * np.array(terms['com_rhs']) / terms['com_velocity_scale'],
                           np.sqrt(s * terms['momentum_weight']) * np.array(terms['momentum_rhs']) / terms['momentum_scale']]
            if s > 0:
                assert np.max(abs(task_a - a[:4])) < 1e-14
                assert np.max(abs(task_b - b[:4])) < 1e-14
            task_u = lsq_linear(task_a, task_b, bounds=(lo, hi), method='bvls', tol=1e-12).x
            a005, b005 = a.copy(), b.copy()
            posture_start = 4 if s > 0 else 0
            a005[posture_start:posture_start + 4] *= .5
            b005[posture_start:posture_start + 4] *= .5
            u005 = lsq_linear(a005, b005, bounds=(lo, hi), method='bvls', tol=1e-12).x
            observed_moving_dq = np.array(prepared['observation']['dq'])[mi]
            dq_difference = observed_moving_dq - np.array(context['moving_dq'])
            actual_task_b = task_b - np.r_[
                np.sqrt(s * terms['com_weight']) * s * np.array(context['com_moving']) @ dq_difference / terms['com_velocity_scale'],
                np.sqrt(s * terms['momentum_weight']) * s * np.array(context['momentum_moving']) @ dq_difference / terms['momentum_scale']]
            desired_q, desired_dq = np.array(row['desired']['q']), np.array(row['desired']['dq'])
            mi_local = mi - 13
            if desired_q.size == 27:
                mi_local = mi
            sample = {
                'row': i + 1, 'elapsed': row['elapsed'], 'command_elapsed': row['command_monotonic'] - meta['execution_start_monotonic'],
                'observation_elapsed': prepared['observation']['arrival_monotonic'] - meta['execution_start_monotonic'],
                'status': diag['status'], 'backend': diag['nominal_backend'],
                'published': bool(diag['published']), 'accepted': bool(diag['nominal_primary_accepted']),
                'retry': bool(diag['nominal_retry_used']), 'collision_flag': bool(diag['collision_rejection']),
                'clipped': bool(diag['clipped']), 'scale': diag['balance_scale'],
                'bound_active': bool(np.any(np.minimum(abs(req - lo), abs(req - hi)) < 1e-6)),
                'rate_bound_active': bool(np.any(abs(abs(req) - rate_cap) < 1e-6)),
                'excursion_bound_active': bool(np.any(np.minimum(abs(req - excursion_lower), abs(req - excursion_upper)) < 1e-6)),
                'requested_nonzero': bool(np.max(abs(req)) > 1e-6),
                'applied_nonzero': bool(np.max(abs(app)) > 1e-6),
                'lower_limit_invalid': bool(np.any(lower_bad)),
                'all_scales_limit_invalid': all(np.any(v) for v in violations),
                'limit_causes': [names[j] for j in np.flatnonzero(np.logical_and.reduce(violations))],
                'limit_violation_peak': float(np.max(np.maximum(limits[:, 0] - full_candidate, full_candidate - limits[:, 1]))),
                'requested': req.tolist(), 'applied': app.tolist(), 'measured': measured.tolist(),
                'counter_q': np.array(context['counter_q']).tolist(),
                'excursion': excursion.tolist(),
                'tau_ff': np.array(buf['tau'])[ci].tolist(),
                'tau_cap_hit': bool(np.any(abs(abs(np.array(buf['tau'])[ci]) - np.array(pub['limits']['tau_clip_limits'])[ci]) < 1e-6)),
                'com_error': context['com_error'],
                'buffer_dq_error': float(np.max(abs(bdq[ci] - app))),
                'buffer_q_law_error': float(np.max(abs(bq[ci] - np.array(context['counter_q']) - pub['dt'] * app))),
                'moving_q_buffer_error': float(np.max(abs(bq[mi] - desired_q[mi_local]))),
                'moving_dq_clip_peak': float(np.max(abs(bdq[mi] - desired_dq[mi_local]))),
                'moving_dq_clipped': bool(np.max(abs(bdq[mi] - desired_dq[mi_local])) > 1e-6),
                'task_target_ss': float(task_b @ task_b),
                'actual_task_target_ss': float(actual_task_b @ actual_task_b),
                'actual_task_ss_measured': float(np.sum((task_a @ measured - actual_task_b) ** 2)),
            }
            for which, u in [('requested', req), ('applied', app), ('measured', measured), ('task_only', task_u), ('posture005', u005)]:
                sample['task_ss_' + which] = float(np.sum((task_a @ u - task_b) ** 2))
                sample['full_ss_' + which] = float(np.sum((a @ u - b) ** 2))
                if which in ('requested', 'applied', 'measured'):
                    for term in ('com', 'momentum'):
                        sample[term + '_residual_' + which] = (np.array(terms[term + '_counter']) @ u - terms[term + '_rhs']).tolist()
            samples.append(sample)
        phases = {
            'move': [s for s in samples if s['elapsed'] <= duration],
            'fade': [s for s in samples if duration < s['elapsed'] <= duration + .5],
            'postfade': [s for s in samples if s['elapsed'] > duration + .5],
            'late_hold': [s for s in samples if duration + 7 <= s['elapsed'] <= duration + 9],
            'all': samples,
        }
        run_result = {
            'target': meta['task']['id'], 'policy': meta['policy'], 'variant': meta['controller_variant'],
            'duration': duration, 'first_elapsed': rows[0]['elapsed'],
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'urdf_hash_matches': hashlib.sha256(urdf.read_bytes()).hexdigest() == provenance['sources']['robot_urdf']['sha256'],
            'controller_yaml_hash_matches': hashlib.sha256((controller / 'config/real_benchmark_upper.yaml').read_bytes()).hexdigest() == provenance['sources']['controller_config']['sha256'],
            'current_resolved_config_matches': current_config == resolved,
            'current_resolved_config_different_sections': [key for key in set(current_config) | set(resolved) if current_config.get(key) != resolved.get(key)],
            'first_references': rows[0]['diagnostics']['controller_diagnostics']['nominal_prepared_context']['references'],
            'reference_changes': sum(rows[0]['diagnostics']['controller_diagnostics']['nominal_prepared_context']['references'] != r['diagnostics']['controller_diagnostics']['nominal_prepared_context']['references'] for r in rows),
            'transport_evidence': {
                'dds_delivery_confirmed': sum(r['dds_delivery_confirmed'] is True for r in rows),
                'actuation_acknowledged': sum(r['actuation_acknowledged'] is True for r in rows),
                'last_published_available': sum(r['command']['last_published'] is not None for r in rows),
            },
            'verification': 'independent full BVLS solve equals requested to <1e-9 on every row; logged task blocks reconstructed to <1e-14 when active',
            'phases': {k: summarize(v) for k, v in phases.items()},
            'samples': samples,
        }
        ct = np.array([s['command_elapsed'] for s in samples])
        ot = np.array([s['observation_elapsed'] for s in samples])
        uu = np.array([s['applied'] for s in samples])
        yy = np.array([s['measured'] for s in samples])
        keep = (ot >= .15) & (ot <= duration)
        scans = []
        for lag in np.arange(0, .101, .001):
            # descriptive smooth-command alignment only, not a transport estimator
            xx = np.column_stack([np.interp(ot[keep] - lag, ct, uu[:, j]) for j in range(4)])
            gain = np.sum(xx * yy[keep]) / np.sum(xx * xx)
            scans.append({'lag_ms': round(lag * 1000), 'gain': float(gain),
                          'rms_residual': rms(yy[keep] - gain * xx),
                          'cosine': float(np.sum(xx * yy[keep]) / np.sqrt(np.sum(xx * xx) * np.sum(yy[keep] ** 2)))})
        run_result['descriptive_counter_alignment'] = {'best': min(scans, key=lambda x: x['rms_residual']), 'zero': scans[0], 'all': scans}
        run_result['reference_and_return'] = {
            'q_ref': samples[0]['counter_q'],
            'last_counter_q': samples[-1]['counter_q'],
            'last_excursion': samples[-1]['excursion'],
            'last_applied': samples[-1]['applied'],
            'last_measured': samples[-1]['measured'],
            'first_scale_below_one': next((s['command_elapsed'] for s in samples if s['scale'] < 1), None),
            'last_scale_above_zero': max(s['command_elapsed'] for s in samples if s['scale'] > 0),
        }
        results['runs'][ident] = run_result
        print(ident, json.dumps({'n': len(samples), 'resolved_match': run_result['current_resolved_config_matches'], 'resolved_difference': run_result['current_resolved_config_different_sections'], 'refs_changed': run_result['reference_changes'], 'move_rate_bounds': run_result['phases']['move']['rate_bound_active'], 'move_excursion_bounds': run_result['phases']['move']['excursion_bound_active'], 'late_hold': {k: run_result['phases']['late_hold'][k] for k in ('excursion_mean_by_joint', 'applied_mean_by_joint', 'measured_mean_by_joint')}, 'tau_hits': run_result['phases']['all']['counter_feedforward_tau_cap_hits'], 'tau_peak': run_result['phases']['all']['counter_feedforward_tau_peak']}))
    args.output.write_text(json.dumps(results, indent=2) + '\n')
    print('Wrote', args.output)
    if args.summary_output is not None:
        compact = {'definition': results['definition'], 'runs': {}}
        for ident, result in results['runs'].items():
            compact_run = {key: value for key, value in result.items() if key != 'samples'}
            compact_run['descriptive_counter_alignment'] = {
                key: value for key, value in result['descriptive_counter_alignment'].items()
                if key != 'all'
            }
            compact['runs'][ident] = compact_run
        args.summary_output.write_text(json.dumps(compact, indent=2) + '\n')
        print('Wrote', args.summary_output)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=Path.cwd())
    parser.add_argument('--output', type=Path, default=Path('/tmp/kilo/iter7b_counter_audit.json'))
    parser.add_argument('--summary-output', type=Path, default=None)
    main(parser.parse_args())
