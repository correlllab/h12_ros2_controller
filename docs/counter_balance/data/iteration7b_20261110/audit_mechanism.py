"""Reproduce the offline mechanism audit from the benchmark repository root

Run with the benchmark .venv Python and --verify-frozen to compare every saved
value exactly. Outputs and intermediate traces default to /tmp/kilo. No robot,
controller, transport, simulator, or raw-data writes are performed. The frozen
campaign has six pairs; the positional mapping and model assumptions are not a
general-purpose dataset interface. Exact equality requires the frozen numeric
environment; disagreements must be inspected, not silently rounded away.
"""

import sys
import json
import hashlib
import argparse
from pathlib import Path
from collections import Counter

import scipy
import numpy as np
import pinocchio as pin
from scipy.signal import savgol_filter
from scipy.optimize import lsq_linear
from scipy.spatial.transform import Rotation


def rms(x):
    return float(np.sqrt(np.mean(np.asarray(x) ** 2)))


def vrms(x):
    return float(np.sqrt(np.mean(np.sum(np.asarray(x) ** 2, axis=-1))))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(args):
    root = args.root.resolve()
    controller = root / 'submodules/h12_ros2_controller'
    sys.path.insert(0, str(controller))
    from h12_ros2_controller.utility.joint_definition import BODY_JOINTS
    urdf = controller / 'submodules/CL_Assets/ros_assets/h1_2_magpie.urdf'
    model = pin.buildModelFromUrdf(str(urdf), pin.JointModelFreeFlyer())
    frozen = [i for i, name in enumerate(model.names)
              if i > 1 and name not in BODY_JOINTS]
    frozen_names = [model.names[i] for i in frozen]
    model = pin.buildReducedModel(model, frozen, pin.neutral(model))
    data = model.createData()
    qi = np.array([model.joints[model.getJointId(n)].idx_q for n in BODY_JOINTS])
    vi = np.array([model.joints[model.getJointId(n)].idx_v for n in BODY_JOINTS])
    torso = model.getFrameId('torso_link')
    results = {
        'definition': 'CMM generalized-velocity column decomposition, not physical link ownership. Angular momentum about whole-body CoM. SI units. All model quantities conditional on URDF inertias, rigid joints, torso-aligned IMU, q/dq/IMU synchrony. No world translation, contact truth, CP, ZMP or causal counterfactual.',
        'root_reconstruction': 'R_WP=R_WT R_PT.T; omega_P=R_PT gyro_T - [0,0,dq_torso]; vroot linear=0; root angular local=omega_P; all 27 measured dq included. R_PT=Rz(q_torso), verified against frame FK.',
        'urdf_sha256': digest(urdf), 'pinocchio_version': pin.__version__,
        'unmeasured_joints_frozen_at_neutral': frozen_names,
        'model_mass_kg': float(sum(inertia.mass for inertia in model.inertias)),
        'runs': {},
    }
    campaign = root / 'runs/real_experiments/iter7b_fame_validation_20261110_184722'
    paths = sorted(campaign.glob('*/*/arm_target.jsonl'))
    if len(paths) != 12:
        raise ValueError(f'Expected 12 frozen trials under {campaign}')
    traces = {}
    for path in paths:
        ident = path.parent.parent.name
        rows = [json.loads(line) for line in path.read_text().splitlines()]
        meta = json.loads(path.with_name('metadata.json').read_text())
        provenance = json.loads(path.with_name('provenance.json').read_text())
        cfg = json.loads(path.with_name('resolved_config.yaml').read_text())['controller']
        moving = np.arange(13, 20) if meta['task']['moving_arm'] == 'left' else np.arange(20, 27)
        counter = np.arange(20, 27) if moving[0] == 13 else np.arange(13, 20)
        active = counter[:4]
        duration = meta['task']['move_duration']
        t = np.array([r['snapshot']['arrival_monotonic'] - meta['execution_start_monotonic'] for r in rows])
        ct = np.array([r['command_monotonic'] - meta['execution_start_monotonic'] for r in rows])
        signals = {k: [] for k in (
            'moving', 'counter', 'base', 'legs', 'torso', 'total', 'partial',
            'rest', 'omega', 'gyro', 'total_body', 'rotation', 'q', 'dq', 'u',
            'qbuffer', 'dqbuffer', 'excursion', 'requested', 'context_q',
            'context_dq', 'context_time',
        )}
        checks, errors = Counter(), Counter()
        task_ss, bound, rate_bound, excursion_bound, tau_hits = [], [], [], [], []
        for i, row in enumerate(rows):
            snap = row['snapshot']
            qj, dqj = np.array(snap['q']), np.array(snap['dq'])
            quat = np.array(snap['imu_quaternion_wxyz'])
            rwt = Rotation.from_quat(quat[[1, 2, 3, 0]]).as_matrix()
            rpt = Rotation.from_rotvec([0, 0, qj[12]]).as_matrix()
            rwp = rwt @ rpt.T
            gyro = np.array(snap['imu_gyroscope'])
            omega = rpt @ gyro - np.array([0, 0, dqj[12]])
            q = pin.neutral(model)
            q[qi] = qj
            v = np.zeros(model.nv)
            v[3:6], v[vi] = omega, dqj
            pin.computeCentroidalMap(model, data, q)
            hbody = np.array(data.Ag[3:]).copy()
            errors['translation_block'] = max(errors['translation_block'], float(np.max(abs(hbody[:, :3]))))
            components = {
                'moving': hbody[:, vi[moving]] @ dqj[moving],
                'counter': hbody[:, vi[counter]] @ dqj[counter],
                'base': hbody[:, 3:6] @ omega,
                'legs': hbody[:, vi[:12]] @ dqj[:12],
                'torso': hbody[:, vi[12]] * dqj[12],
            }
            body_total = hbody @ v
            components['total'] = body_total
            components['partial'] = components['moving'] + components['counter']
            components['rest'] = components['base'] + components['legs'] + components['torso']
            errors['component_sum'] = max(errors['component_sum'], float(np.max(abs(components['partial'] + components['rest'] - body_total))))
            q[3:7] = Rotation.from_matrix(rwp).as_quat()
            pin.computeCentroidalMap(model, data, q)
            world_h = np.array(data.Ag[3:]).copy()
            errors['frame_rotation'] = max(errors['frame_rotation'], float(np.max(abs(world_h @ v - rwp @ body_total))))
            if i == 0:
                pin.computeCentroidalMomentum(model, data, q, v)
                errors['direct_momentum'] = float(np.max(abs(data.hg.angular - world_h @ v)))
                shifted = q.copy()
                shifted[:3] = [1, -2, .5]
                translated = v.copy()
                translated[:3] = [.3, -.2, .1]
                pin.computeCentroidalMomentum(model, data, shifted, translated)
                errors['translation_invariance'] = float(np.max(abs(data.hg.angular - world_h @ v)))
            pin.forwardKinematics(model, data, q, v)
            pin.updateFramePlacements(model, data)
            errors['torso_orientation'] = max(errors['torso_orientation'], float(np.max(abs(data.oMf[torso].rotation - rwt))))
            wtorso = pin.getFrameVelocity(model, data, torso, pin.LOCAL).angular
            errors['torso_gyro_reconstruction'] = max(errors['torso_gyro_reconstruction'], float(np.max(abs(wtorso - gyro))))
            for key, value in components.items():
                signals[key].append(rwp @ value)
            for key, value in [('omega', rwp @ omega), ('gyro', gyro), ('total_body', body_total), ('rotation', rwp), ('q', qj), ('dq', dqj)]:
                signals[key].append(value)
            checks['nonzero_ddq'] += int(np.count_nonzero(snap['ddq']))
            checks['dds_confirmed'] += row.get('dds_delivery_confirmed') is True
            checks['actuation_acknowledged'] += row.get('actuation_acknowledged') is True
            if 'counter' not in ident:
                continue
            diag = row['diagnostics']['controller_diagnostics']
            prepared = diag['nominal_prepared_context']
            context, pub, ref = prepared['context'], prepared['publication'], prepared['references']
            req, u = np.array(diag['requested_counter_dq']), np.array(diag['applied_counter_dq'])
            qb = np.array(row['command']['buffer']['q'])[active]
            ub = np.array(row['command']['buffer']['dq'])[active]
            cq = np.array(context['counter_q'])
            ex = cq - ref['q_counter_ref']
            cdq = np.array(prepared['observation']['dq'])[active]
            for key, value in [('u', u), ('qbuffer', qb), ('dqbuffer', ub), ('excursion', ex), ('requested', req), ('context_q', cq), ('context_dq', cdq), ('context_time', prepared['observation']['arrival_monotonic'] - meta['execution_start_monotonic'])]:
                signals[key].append(value)
            errors['buffer_dq'] = max(errors['buffer_dq'], float(np.max(abs(ub - u))))
            errors['buffer_q_law'] = max(errors['buffer_q_law'], float(np.max(abs(qb - cq - pub['dt'] * u))))
            errors['snapshot_context_q'] = max(errors['snapshot_context_q'], float(np.max(abs(qj - np.array(prepared['observation']['q'])))))
            lo, hi = np.array(diag['nominal_problem_lower']), np.array(diag['nominal_problem_upper'])
            a, b = np.array(diag['nominal_problem_matrix']), np.array(diag['nominal_problem_target'])
            solved = lsq_linear(a, b, bounds=(lo, hi), method='bvls', tol=1e-12).x
            errors['independent_solve'] = max(errors['independent_solve'], float(np.max(abs(solved - req))))
            caps = np.minimum(np.array(pub['limits']['dq_clip_limits'])[active], pub['dq_lim'])
            maxex = np.array(cfg['reactive_counter_balance']['max_excursion'])
            bound.append(np.any(np.minimum(abs(req - lo), abs(req - hi)) < 1e-6))
            rate_bound.append(np.any(abs(abs(req) - caps) < 1e-6))
            excursion_bound.append(np.any(np.minimum(abs(req - (-maxex - ex) / pub['dt']), abs(req - (maxex - ex) / pub['dt'])) < 1e-6))
            terms = diag['nominal_problem_terms']
            if terms['balance_scale'] > 0:
                task_ss.append([np.sum((a[:4] @ val - b[:4]) ** 2) for val in (u, cdq)] + [np.sum(b[:4] ** 2)])
            else:
                task_ss.append([0, 0, 0])
                errors['postfade_law'] = max(errors['postfade_law'], float(np.max(abs(u + ex / 1.005))))
            checks[diag['status']] += 1
            checks['retry'] += bool(diag['nominal_retry_used'])
            checks['backtrack'] += diag['backtrack_scale'] < 1
            tau = np.array(row['command']['buffer']['tau'])[active]
            tau_hits.append(np.any(abs(tau) >= np.array(pub['limits']['tau_clip_limits'])[active] - 1e-6))
        signals = {k: np.array(v) for k, v in signals.items() if len(v)}
        phases = {}
        for name, mask in {
                'move': t <= duration,
                'fade': (t > duration) & (t <= duration + .5),
                'postfade': t > duration + .5,
                'late': (t >= duration + 7) & (t <= duration + 9),
        }.items():
            p = {'n': int(mask.sum()), 'observed_start': float(t[mask][0]), 'observed_end': float(t[mask][-1]), 'max_gap': float(np.max(np.diff(t[mask])))}
            for key in ('moving', 'counter', 'base', 'legs', 'torso', 'partial', 'rest', 'total', 'omega', 'gyro'):
                p[key + '_rms_xyz'] = vrms(signals[key][mask])
                p[key + '_rms_xy'] = vrms(signals[key][mask, :2])
            p['partial_to_moving_xy'] = p['partial_rms_xy'] / p['moving_rms_xy']
            p['total_to_moving_xy'] = p['total_rms_xy'] / p['moving_rms_xy']
            if 'counter' in ident:
                for key in ('u', 'dqbuffer', 'requested'):
                    p[key + '_rms'] = rms(signals[key][mask])
                    p[key + '_mean'] = signals[key][mask].mean(axis=0).tolist()
                    p[key + '_peak'] = np.max(abs(signals[key][mask]), axis=0).tolist()
                p['measured_rms'] = rms(signals['dq'][mask][:, active])
                p['measured_mean'] = signals['dq'][mask][:, active].mean(axis=0).tolist()
                p['measured_peak'] = np.max(abs(signals['dq'][mask][:, active]), axis=0).tolist()
                p['excursion_mean'] = signals['excursion'][mask].mean(axis=0).tolist()
                p['excursion_peak'] = np.max(abs(signals['excursion'][mask]), axis=0).tolist()
                p['qlead_mean'] = (signals['qbuffer'][mask] - signals['context_q'][mask]).mean(axis=0).tolist()
                p['context_measured_rms'] = rms(signals['context_dq'][mask])
                p['context_measured_mean'] = signals['context_dq'][mask].mean(axis=0).tolist()
                p['context_q_slope'] = np.polyfit(signals['context_time'][mask], signals['context_q'][mask], 1)[0].tolist()
                p['context_q_end_minus_start'] = (signals['context_q'][mask][-1] - signals['context_q'][mask][0]).tolist()
                p['context_minus_snapshot_ms_max'] = float(np.max(signals['context_time'][mask] - t[mask]) * 1000)
                for key, values in [('bound', bound), ('rate_bound', rate_bound), ('excursion_bound', excursion_bound), ('tau_ff_cap_hit', tau_hits)]:
                    p[key] = int(np.array(values)[mask].sum())
                ss = np.array(task_ss)[mask].sum(axis=0)
                p['task_residual_applied_measured'] = np.sqrt(ss[:2] / ss[2]).tolist() if ss[2] > 0 else None
                valid = mask[1:] & mask[:-1] & (np.diff(ct) < .1)
                slew = np.diff(signals['u'], axis=0)[valid] / np.diff(ct)[valid, None]
                p['command_slew_rms'] = rms(slew)
                p['command_slew_peak'] = float(np.max(abs(slew)))
            phases[name] = p
        # derivatives are resampled signal diagnostics, never recorded ddq
        grid = np.arange(t[0], t[-1], 1 / 30)
        interp = {k: np.column_stack([np.interp(grid, t, signals[k][:, j]) for j in range(3)]) for k in ('moving', 'counter', 'partial', 'rest', 'total', 'total_body', 'omega')}
        deriv = {}
        for width in (3, 5, 9):
            d = {k: savgol_filter(v, width, min(2, width - 1), deriv=1, delta=1 / 30, axis=0) for k, v in interp.items()}
            good = np.ones(len(grid), dtype=bool)
            edge = width // 2
            good[:edge] = good[-edge:] = False
            for left, right in zip(t[:-1], t[1:]):
                if right - left > .1:
                    good &= (grid < left - edge / 30) | (grid > right + edge / 30)
            out = {}
            for name, mask in {'move': grid <= duration, 'fade': (grid > duration) & (grid <= duration + .5), 'postfade': grid > duration + .5, 'late': (grid >= duration + 7) & (grid <= duration + 9)}.items():
                mask &= good
                out[name] = {k + '_rate_rms_xy': vrms(v[mask, :2]) for k, v in d.items() if k != 'total_body'}
                out[name]['partial_rate_ratio'] = out[name]['partial_rate_rms_xy'] / out[name]['moving_rate_rms_xy']
                out[name]['n'] = int(mask.sum())
            deriv[str(width)] = out
        rotations = Rotation.from_matrix(signals['rotation'])
        orient_rate = (rotations[1:] * rotations[:-1].inv()).as_rotvec() / np.diff(t)[:, None]
        midomega = .5 * (signals['omega'][1:] + signals['omega'][:-1])
        good = np.diff(t) < .1
        result = {
            'path': str(path.relative_to(root)), 'raw_sha256': digest(path),
            'provenance': provenance, 'target': meta['task']['id'],
            'duration': duration, 'checks': dict(checks), 'errors': dict(errors),
            'phases': phases, 'derivative_window_samples': deriv,
            'orientation_vs_gyro_rate_rms_error': vrms(orient_rate[good] - midomega[good]),
            'orientation_rate_rms': vrms(orient_rate[good]),
            'gyro_root_rate_rms': vrms(midomega[good]),
            'orientation_vs_gyro_rate_cosine': float(np.sum(orient_rate[good] * midomega[good]) / np.linalg.norm(orient_rate[good]) / np.linalg.norm(midomega[good])),
        }
        if 'counter' in ident:
            buf = rows[-1]['command']['buffer']
            kp, kd = np.array(buf['kp'])[active], np.array(buf['kd'])[active]
            result['effective_postfade_stiffness_PD_model'] = ((kp / 30 + kd) / 1.005).tolist()
            result['kp'], result['kd'] = kp.tolist(), kd.tolist()
        results['runs'][ident] = result
        traces[ident] = {'t': t.tolist(), **{k: v.tolist() for k, v in signals.items()}}
        print(ident, 'rows', len(rows), 'mapping error', errors.get('buffer_q_law'))
    # one-time migration check against the original temporary audit
    if args.baseline:
        assert results == json.loads(args.baseline.read_text()), 'Original audit differs'
    source_checks = {}
    input_hashes = {}
    for ident, run in results['runs'].items():
        source_checks[ident] = {}
        for key, src in run['provenance']['sources'].items():
            if not src['path'] or not src['sha256']:
                continue
            relative = src['path'].split('h12_zmp_benchmark/', 1)[-1]
            source = root / relative
            current = digest(source) if source.is_file() else None
            source_checks[ident][key] = {
                'path': relative, 'current': current,
                'recorded': src['sha256'], 'matches': current == src['sha256'],
            }
        directory = (root / run['path']).parent
        input_hashes[ident] = {name: digest(directory / name) for name in (
            'arm_target.jsonl', 'metadata.json', 'provenance.json', 'resolved_config.yaml',
        )}
    verification = {
        'all_rows': sum(r['phases'][p]['n'] for r in results['runs'].values() for p in ('move', 'fade', 'postfade')),
        'counter_rows': sum(r['checks'].get('solved', 0) for r in results['runs'].values()),
        'max_errors': {key: max(r['errors'].get(key, 0) for r in results['runs'].values()) for key in results['runs']['F01_counter']['errors']},
    }
    assert verification['all_rows'] == 4057
    assert verification['counter_rows'] == 2029
    assert verification['max_errors']['buffer_q_law'] < 3e-8
    assert verification['max_errors']['translation_invariance'] < 2e-14
    results.update({
        'verification': verification, 'source_checks': source_checks,
        'input_hashes': input_hashes,
        'numeric_environment': {'numpy': np.__version__, 'scipy': scipy.__version__},
        'pairs': {key: [key + '_frame', key + '_counter'] for key in ('F01', 'F02', 'F03', 'F03r', 'F04', 'F04r')},
    })
    if args.verify_frozen:
        frozen_path = Path(__file__).with_name('mechanism_summary.json')
        assert results == json.loads(frozen_path.read_text()), 'Frozen mechanism summary differs'
        print('Exact frozen mechanism comparison passed')
    args.output.write_text(json.dumps(results, indent=2, allow_nan=False) + '\n')
    args.traces.write_text(json.dumps(traces, allow_nan=False) + '\n')
    print('Verification:', verification)
    print('Summary:', args.output, 'Traces:', args.traces)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path.cwd())
    parser.add_argument('--output', type=Path, default=Path('/tmp/kilo/mechanism_summary.json'))
    parser.add_argument('--traces', type=Path, default=Path('/tmp/kilo/iter7b_retained_traces.json'))
    parser.add_argument('--baseline', type=Path, help='Optional original audit JSON for exact migration check')
    parser.add_argument('--verify-frozen', action='store_true')
    main(parser.parse_args())
