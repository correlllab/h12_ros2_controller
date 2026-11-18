'''Solve stored unloaded nominal problems, without controller or plant imports'''
import sys
import ast
import csv
import json
import argparse
import hashlib
from pathlib import Path

import numpy as np
from scipy.optimize import lsq_linear


VARIANTS = ('c0', 'feedback_off', 'com_feedback_off', 'gyro_feedback_off')


def rms(x):
    return float(np.sqrt(np.mean(np.asarray(x) ** 2)))


def norm_rms(x):
    return float(np.sqrt(np.mean(np.sum(np.asarray(x) ** 2, axis=-1))))


def solve(a, b, lo, hi):
    fixed = lo == hi
    u = lo.copy()
    if (~fixed).any():
        u[~fixed] = lsq_linear(a[:, ~fixed], b - a[:, fixed] @ lo[fixed],
                              bounds=(lo[~fixed], hi[~fixed]), method='bvls',
                              tol=1e-12, max_iter=100).x
    return u


def summarize(samples):
    result = {'n': len(samples)}
    if not samples:
        return result
    for key in ('ff_com', 'fb_com', 'ff_momentum', 'fb_momentum', 'com_error', 'gyro'):
        result[key + '_norm_rms'] = norm_rms([s[key] for s in samples])
    result['variants'] = {}
    for variant in VARIANTS:
        vs = [s['variants'][variant] for s in samples]
        out = {'delta_u_rms_rad_s': rms([v['du'] for v in vs]),
               'delta_u_peak_rad_s': float(np.max(np.abs([v['du'] for v in vs]))),
               'u_rms_rad_s': rms([v['u'] for v in vs]),
               'bound_active_rows': sum(v['bound_active'] for v in vs),
               'bound_active_fraction': float(np.mean([v['bound_active'] for v in vs])),
               'exact_equal_c0_rows': sum(v['exact_equal'] for v in vs),
               'delta_q_command_rms_mrad': 1000 * rms([v['dq_command'] for v in vs])}
        for key in ('ff_com_residual', 'original_com_residual',
                    'ff_momentum_residual', 'original_momentum_residual'):
            out[key + '_norm_rms'] = norm_rms([v[key] for v in vs])
        for key in ('u_slew', 'q_command_slew', 'delta_u_slew', 'delta_q_command_slew'):
            vals = [v[key] for v in vs if key in v]
            out[key + '_rms'] = rms(vals) if vals else None
        out['original_weighted_task_residual_norm_rms'] = norm_rms([
            v['weighted_original_residual'] for v in vs])
        result['variants'][variant] = out
    return result


def main(args):
    root = args.root.resolve()
    almi = json.loads(Path('/tmp/kilo/lastweek_almi_figures.json').read_text())
    paired = {p[k + '_run_id'] for p in almi['pairs'] for k in ('counter', 'frame')}
    sessions = ['iter7b_fame_validation_20261110_184722',
                'iter7b_almi_manip_stand_session_20261113_190215',
                'iter7b_almi_manip_stand_session_20261113_191535']
    result = {'scope': 'unloaded stored-state allocation only; no plant rollout',
              'definitions': {
                  'rows': 'all_rows, no outcome/threshold selection; absent Frame matrices not fabricated',
                  'rms': 'sample RMS; action RMS per scalar joint, residual RMS Euclidean norm per 2D task',
                  'ff': 's*(-Jmoving*dqdesired); original=FF+FB; feedback norms reported separately',
                  'residual': 'Jcounter*u-original_target or Jcounter*u-FF; original target never changed for scoring',
                  'bounds': 'exact stored lower/upper; active tolerance 1e-10 from existing solver',
                  'slew': 'adjacent frozen-row finite difference, divided by actual command_monotonic interval; not plant acceleration',
                  'q_command': 'q_measured+publication_dt*u; no backtrack/collision/actuation claim',
                  'postfade': 'elapsed>T+.5; s=0 leaves exact same posture+damping problem',
                  'body': 'unknown tilt/yaw/translation/contact/load/leg-policy counterfactual outcomes',
              }, 'cases': [], 'verification': {}, 'inventory': []}
    all_samples = []
    max_solve_error = max_target_error = max_matrix_error = 0.
    max_applied_error = 0.
    max_class_error = 0.
    source = root / 'submodules/h12_ros2_controller/h12_ros2_controller/core/controller/counter_balance/diagnostic_controller.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'CounterBalanceDiagnosticController')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_reaction_target_offsets')
    # execute only the pure target-offset method, never import hardware or a controller
    base = type('Base', (), {'_reaction_target_offsets': lambda self, *args: (np.zeros(2), np.zeros(2))})
    test_cls = ast.ClassDef(name='PureOffsets', bases=[ast.Name(id='Base', ctx=ast.Load())],
                           keywords=[], body=[method], decorator_list=[])
    namespace = {'np': np, 'Base': base}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[test_cls], type_ignores=[])), str(source), 'exec'), namespace)
    checker = namespace['PureOffsets']()
    n_s0 = n_missing = 0
    for session in sessions:
        for path in sorted((root / 'runs/real_experiments' / session).rglob('arm_target.jsonl')):
            meta = json.loads(path.with_name('metadata.json').read_text())
            summary = json.loads(path.with_name('summary.json').read_text())
            provenance = json.loads(path.with_name('provenance.json').read_text())
            resolved = json.loads(path.with_name('resolved_config.yaml').read_text())
            cfg = resolved['controller']['reactive_counter_balance']
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            rel = str(path.parent.parent.relative_to(root))
            item = {'run_id': rel, 'raw_path': str(path), 'raw_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                    'policy': meta['policy'], 'external_policy_version': meta.get('external_policy_version'),
                    'source_git': provenance['git'], 'config_sha256': provenance['resolved_config_sha256'],
                    'controller_source_sha256': provenance['sources']['controller_config']['sha256'],
                    'real_adapter_sha256': provenance['sources']['real_adapter']['sha256'],
                    'samples': len(rows), 'run_complete': summary.get('run_complete'),
                    'window_complete': summary.get('window_complete'),
                    'paired': True if session == sessions[0] else rel in paired}
            result['inventory'].append(item)
            samples = []
            previous = None
            for row in rows:
                d = row['diagnostics']['controller_diagnostics']
                if not d or d.get('nominal_problem_matrix') is None:
                    n_missing += 1
                    continue
                p = d['nominal_prepared_context']
                c, terms = p['context'], d['nominal_problem_terms']
                a, b, lo, hi = [np.asarray(d['nominal_problem_' + k], dtype=float)
                                for k in ('matrix', 'target', 'lower', 'upper')]
                scale = terms['balance_scale']
                ff = [scale * (-np.asarray(c[k + '_moving']) @ c['moving_dq'])
                      for k in ('com', 'momentum')]
                fb = [-scale * cfg['gains']['com'] * np.asarray(c['com_error']),
                      scale * cfg['gains']['gyro'] * np.asarray(c['gyro'][:2])]
                rhs = [np.asarray(terms[k + '_rhs']) for k in ('com', 'momentum')]
                max_target_error = max(max_target_error, *(np.max(abs(x+y-z)) for x,y,z in zip(ff,fb,rhs)))
                factor = [np.sqrt(scale * terms[k + '_weight']) / terms[k + '_scale']
                          if k == 'momentum' else np.sqrt(scale * terms['com_weight']) / terms['com_velocity_scale']
                          for k in ('com', 'momentum')]
                if scale:
                    aa = np.vstack([f * np.asarray(terms[k + '_counter'])
                                    for f,k in zip(factor, ('com', 'momentum'))])
                    max_matrix_error = max(max_matrix_error, float(np.max(abs(aa-a[:4]))))
                    assert np.allclose(b[:4], np.concatenate([f*x for f,x in zip(factor,rhs)]), atol=1e-14, rtol=0)
                u0 = solve(a,b,lo,hi)
                max_solve_error = max(max_solve_error, float(np.max(abs(u0 - d['requested_counter_dq']))))
                max_applied_error = max(max_applied_error, float(np.max(abs(u0 - d['applied_counter_dq']))))
                elapsed = row['elapsed_s']
                duration = meta['task']['move_duration']
                phase = 'move' if elapsed <= duration else 'fade' if elapsed <= duration+.5 else 'postfade'
                sample = {'phase': phase, 'ff_com': ff[0], 'fb_com': fb[0],
                          'ff_momentum': ff[1], 'fb_momentum': fb[1],
                          'com_error': c['com_error'], 'gyro': c['gyro'][:2], 'variants': {}}
                time = row['command_monotonic']
                dt = p['publication']['dt']
                for variant in VARIANTS:
                    checker.com_gain = cfg['gains']['com']
                    checker.gyro_gain = cfg['gains']['gyro']
                    checker.diagnostic_variant = variant
                    checker._diagnostic_record = {}
                    offsets = checker._reaction_target_offsets(
                        np.asarray(c['com_moving']), np.asarray(c['momentum_moving']),
                        np.asarray(c['moving_dq']), np.asarray(c['com_error']),
                        np.asarray(c['gyro']), scale)
                    expected = (-(fb[0]) if variant in ('feedback_off', 'com_feedback_off') else np.zeros(2),
                                -(fb[1]) if variant in ('feedback_off', 'gyro_feedback_off') else np.zeros(2))
                    max_class_error = max(max_class_error, *(float(np.max(abs(x-y))) for x,y in zip(offsets,expected)))
                    bb = b.copy()
                    if scale:
                        if variant in ('feedback_off', 'com_feedback_off'):
                            bb[:2] = factor[0] * ff[0]
                        if variant in ('feedback_off', 'gyro_feedback_off'):
                            bb[2:4] = factor[1] * ff[1]
                    u = u0.copy() if variant == 'c0' or np.array_equal(bb,b) else solve(a,bb,lo,hi)
                    du = u-u0
                    residual = [np.asarray(terms[k + '_counter']) @ u for k in ('com', 'momentum')]
                    qcmd = np.asarray(c['counter_q']) + dt*u
                    v = {'u': u, 'du': du, 'qcmd': qcmd, 'dq_command': dt*du,
                         'exact_equal': bool(np.array_equal(u,u0)),
                         'bound_active': bool(np.any(np.minimum(abs(u-lo),abs(u-hi)) <= 1e-10)),
                         'ff_com_residual': residual[0]-ff[0],
                         'original_com_residual': residual[0]-rhs[0],
                         'ff_momentum_residual': residual[1]-ff[1],
                         'original_momentum_residual': residual[1]-rhs[1],
                         'weighted_original_residual': np.concatenate([f*(x-y) for f,x,y in zip(factor,residual,rhs)])}
                    if previous is not None:
                        gap = time-previous['time']
                        assert gap > 0
                        pv = previous['sample']['variants'][variant]
                        v['u_slew'] = (u-pv['u'])/gap
                        v['q_command_slew'] = (qcmd-pv['qcmd'])/gap
                        v['delta_u_slew'] = (du-pv['du'])/gap
                        v['delta_q_command_slew'] = (v['dq_command']-pv['dq_command'])/gap
                    sample['variants'][variant] = v
                if scale == 0:
                    n_s0 += 1
                    assert all(v['exact_equal'] for v in sample['variants'].values())
                previous = {'time': time, 'sample': sample}
                samples.append(sample)
            if samples:
                first = samples[0]
                case = {**item, 'target': meta['task']['id'], 'duration_s': duration,
                        'first_com_error': first['com_error'], 'first_gyro': first['gyro'],
                        'first_delta_u': {k:v['du'].tolist() for k,v in first['variants'].items()},
                        'phases': {phase:summarize([s for s in samples if s['phase']==phase])
                                   for phase in ('move','fade','postfade')},
                        'all_rows': summarize(samples)}
                result['cases'].append(case)
                for sample in samples:
                    sample['stratum'] = session
                    sample['paired'] = item['paired']
                all_samples.extend(samples)
                print(session[-6:], path.parent.parent.name, 'paired',item['paired'], 'rows',len(samples))
    result['all_rows'] = summarize(all_samples)
    result['phases'] = {phase:summarize([s for s in all_samples if s['phase']==phase])
                        for phase in ('move','fade','postfade')}
    result['session_phases'] = {session:{phase:summarize([
        s for s in all_samples if s['stratum']==session and s['phase']==phase])
        for phase in ('move','fade','postfade')} for session in sessions}
    result['paired_state_phases'] = {phase:summarize([
        s for s in all_samples if s['paired'] and s['phase']==phase])
        for phase in ('move','fade','postfade')}
    result['verification'] = {'c0_max_abs_vs_requested': max_solve_error,
                              'target_decomposition_max_abs': max_target_error,
                              'weighted_task_matrix_max_abs': max_matrix_error,
                              'c0_max_abs_vs_applied': max_applied_error,
                              'pure_diagnostic_class_offsets_max_abs': max_class_error,
                              'diagnostic_class_source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
                              'class_validation_scope': 'AST extracted pure offset method only, not full controller replay',
                              's0_identical_rows': n_s0, 'missing_problem_rows': n_missing,
                              'repository_edits': False, 'actuation': False, 'simulation': False}
    assert max_solve_error < 1e-8 and max_target_error < 1e-12 and max_matrix_error < 1e-12 and max_class_error == 0
    args.output.write_text(json.dumps(result, separators=(',', ':'))+'\n')
    table = []
    for case in result['cases']:
        for phase, stats in case['phases'].items():
            for variant, values in stats.get('variants', {}).items():
                table.append({'run_id': case['run_id'], 'target': case['target'],
                              'paired': case['paired'], 'phase': phase,
                              'variant': variant, 'n': stats['n'],
                              **{k:v for k,v in stats.items() if k.endswith('_norm_rms')},
                              **values})
    with args.output.with_suffix('.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(table[0]))
        writer.writeheader()
        writer.writerows(table)
    print('VERIFIED', result['verification'])
    print('OUTPUT',args.output,args.output.stat().st_size)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=Path.cwd())
    parser.add_argument('--output', type=Path, default=Path('/tmp/kilo/frozen_feedback_ablation.json'))
    main(parser.parse_args())
