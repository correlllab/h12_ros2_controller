'''Replay the full diagnostic command path against three unloaded recordings'''
import sys
import json
import argparse
import threading
from copy import deepcopy
from pathlib import Path
from hashlib import sha256
from types import SimpleNamespace
from collections import Counter
from unittest.mock import patch

import numpy as np


VARIANTS = ('c0', 'feedback_off', 'com_feedback_off', 'gyro_feedback_off')


def maximum(a, b):
    return float(np.max(np.abs(np.asarray(a, dtype=float)-np.asarray(b, dtype=float))))


def main(args):
    root = args.root.resolve()
    sub = root / 'submodules/h12_ros2_controller'
    fixture = sub / 'docs/counter_balance/data/iteration7b_20261110/recovery_prototype'
    sys.path[:0] = [str(sub), str(fixture)]
    from iter7b_recovery_fixture import make_model
    from h12_ros2_controller.core.low_cmd_handler import LowCmdHandler
    from h12_ros2_controller.core.controller.counter_balance.diagnostic_controller import CounterBalanceDiagnosticController as Controller
    from h12_ros2_controller.core.channel_interface import StateSubscriber, LowCmdPublisher

    # constructors are denied, not simulated; only memory transport is supplied
    def denied(*unused, **kwargs):
        raise AssertionError('DDS/controller/handler constructor forbidden')

    base = root / 'runs/real_experiments'
    specs = [
        ('iter7b_fame_validation_20261110_184722', 'F02_counter'),
        ('iter7b_fame_validation_20261110_184722', 'F03_counter'),
        ('iter7b_almi_manip_stand_session_20261113_190215',
         'trial_0006_counter_right_overhang_sideways'),
    ]
    old_artifacts = {str(p): sha256(p.read_bytes()).hexdigest() for p in (
        Path('/tmp/kilo/frozen_feedback_ablation.py'),
        Path('/tmp/kilo/frozen_feedback_ablation.json'),
        Path('/tmp/kilo/frozen_feedback_ablation.csv'))}
    result = {
        'scope': 'unloaded frozen-observation full command-path replay, no plant/body counterfactual',
        'construction': '__new__ controller and LowCmdHandler; real RobotModel and Pin/FCL; memory publisher only',
        'not_mocked': ['control_configuration_step', '_model_terms', '_counter_velocity_bounds',
                       '_counter_excursion_bounds', '_plan_counter_velocity',
                       '_finalize_counter_velocity', '_backtrack_counter',
                       '_candidate_status', '_publish_arm_command', '_write_arm_command',
                       'LowCmdHandler.set_joint_commands', 'RobotDynamics.get_gravity_compensation'],
        'references': 'configure_diagnostic_trial resets first; restore all captured refs per row before full step',
        'limits': 'recorded publication limits and dt; actual model URDF intersection, dq_lim, excursion, publisher clips',
        'tolerance': 'one float32 epsilon times max(1,abs(original)); report exact equality separately',
        'reset_scope': 'each variant/row gets original pre-command buffer and same recorded observation; not an evolving rollout',
        'historical_tau_limit': 'q/dq parity is required; historical tau parity is measured, not assumed. Gravity uses live model.state IMU, not the nominal captured snapshot. Exact historical gravity-time IMU is not recorded.',
        'prior_artifact_sha256': old_artifacts, 'sources': {}, 'runs': [],
        'simulation': False, 'hardware': False, 'repository_edits': False,
    }
    for rel in ('h12_ros2_controller/core/controller/counter_balance/diagnostic_controller.py',
                'h12_ros2_controller/core/controller/counter_balance/counter_balance_controller.py',
                'h12_ros2_controller/core/controller/counter_balance/scipy_nominal_planner.py',
                'h12_ros2_controller/core/low_cmd_handler.py',
                'h12_ros2_controller/core/robot_model.py'):
        result['sources'][str(sub / rel)] = sha256((sub / rel).read_bytes()).hexdigest()
    with patch.object(Controller, '__init__', denied), patch.object(LowCmdHandler, '__init__', denied), \
            patch.object(StateSubscriber, '__init__', denied), patch.object(LowCmdPublisher, '__init__', denied):
        for session, name in specs:
            path = next((base / session / name).glob('*/arm_target.jsonl'))
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            if args.max_rows:
                rows = rows[:args.max_rows]
            cfg = json.loads(path.with_name('resolved_config.yaml').read_text())['controller']
            meta = json.loads(path.with_name('metadata.json').read_text())
            provenance = json.loads(path.with_name('provenance.json').read_text())
            first = rows[0]['diagnostics']['controller_diagnostics']['nominal_prepared_context']
            model = make_model(first['publication']['limits']['q_clip_limits'])
            assets = sub / 'submodules/CL_Assets/ros_assets'
            for source, filename in (('robot_urdf', 'h1_2_magpie.urdf'),
                                     ('collision_urdf', 'h1_2_magpie_sphere.urdf'),
                                     ('collision_srdf', 'h1_2_magpie_sphere_collision.srdf')):
                assert sha256((assets / filename).read_bytes()).hexdigest() == provenance['sources'][source]['sha256']
            controllers = {}
            for variant in VARIANTS:
                c = Controller.__new__(Controller)
                c.robot_model = model
                c.visualize = False
                c.arm_ids = list(range(13, 27))
                c.motor_q_indices, c.motor_v_indices = c._body_indices()
                c._load_reactive_config(cfg['reactive_counter_balance'])
                c._reactive_data = model.model_body.createData()
                c.dt = first['publication']['dt']
                c.dq_lim = first['publication']['dq_lim']
                c.backtrack_scales = first['publication']['backtrack_scales']
                c.config = deepcopy(cfg)
                c.config['limits'] = {k: np.asarray(v) for k,v in first['publication']['limits'].items()}
                c.moving_arm = meta['task']['moving_arm']
                c.velocity_ocp = None
                c.latest_velocity_ocp_result = None
                h = LowCmdHandler.__new__(LowCmdHandler)
                h._recording = False
                h._estopped = False
                h._q_clip_limits = c.config['limits']['q_clip_limits']
                h._dq_clip_limits = c.config['limits']['dq_clip_limits']
                h._tau_clip_limits = c.config['limits']['tau_clip_limits']
                h._command_publisher = SimpleNamespace(data_lock=threading.Lock(),
                    q=np.zeros(27, np.float32), dq=np.zeros(27, np.float32),
                    tau=np.zeros(27, np.float32), mode=np.ones(27, np.int32))
                c.low_cmd_handler = h
                c.configure_diagnostic_trial(variant)
                controllers[variant] = c
            out = {'path': str(path), 'raw_sha256': sha256(path.read_bytes()).hexdigest(),
                   'verified_model_sha256': {k:provenance['sources'][k]['sha256'] for k in ('robot_urdf','collision_urdf','collision_srdf')},
                   'moving_arm': meta['task']['moving_arm'], 'rows': len(rows),
                   'source_git': provenance['git'], 'phases': {},
                   'c0_buffer_max_abs': dict(q=0., dq=0., tau=0.),
                   'c0_buffer_exact_rows': dict(q=0, dq=0, tau=0),
                   'c0_buffer_within_float32_rows': dict(q=0, dq=0, tau=0),
                   'c0_problem_max_abs': 0., 'c0_prepared_context_max_abs': 0.,
                   'c0_requested_max_abs': 0., 'c0_offset_max_abs': 0.,
                   'variant_counts': {}, 'errors': []}
            for variant in VARIANTS:
                out['variant_counts'][variant] = {'executions': 0, 'statuses': Counter(),
                    'finite': 0, 'published': 0, 'valid_candidate_pin_fcl': 0,
                    'request_within_actual_bounds': 0, 'buffers_within_publisher_limits': 0,
                    'noncounter_buffers_exact_c0': 0, 'tau_exact_c0': 0,
                    'zero_scale_buffers_exact_c0': 0, 'zero_scale_rows': 0,
                    'delta_counter_q_peak_rad': 0., 'delta_counter_dq_peak_rad_s': 0.,
                    'bounds_max_abs_vs_recorded': 0., 'backtrack_scales': Counter()}
            for index, row in enumerate(rows):
                d = row['diagnostics']['controller_diagnostics']
                p = d['nominal_prepared_context']
                obs = deepcopy(p['observation'])
                obs['q'], obs['dq'] = np.asarray(obs['q']), np.asarray(obs['dq'])
                obs['imu_state'] = SimpleNamespace(**obs['imu_state'])
                model.state_subscriber = SimpleNamespace(state=obs)
                phase = ('move' if row['elapsed_s'] <= meta['task']['move_duration'] else
                         'fade' if row['elapsed_s'] <= meta['task']['move_duration']+.5 else 'postfade')
                out['phases'][phase] = out['phases'].get(phase, 0)+1
                outputs = {}
                for variant, c in controllers.items():
                    c.configure_diagnostic_trial(variant)
                    for key, value in p['references'].items():
                        setattr(c, key, np.asarray(value) if isinstance(value, list) else value)
                    for key in ('q', 'dq', 'tau'):
                        getattr(c.low_cmd_handler._command_publisher, key)[:] = p['publication'][key+'_cmd']
                    c.set_diagnostic_sample(row['elapsed_s'], meta['task']['move_duration'])
                    returned = c.control_configuration_step(
                        np.asarray(row['desired']['q']), np.asarray(row['desired']['dq']),
                        balance_scale=d['balance_scale'])
                    diag = c.diagnostics()
                    counts = out['variant_counts'][variant]
                    counts['executions'] += 1
                    counts['statuses'][diag['status']] += 1
                    counts['published'] += bool(diag['published'])
                    plan = c._latest_nominal_plan
                    if plan is None:
                        raise AssertionError((name,index,variant,diag['status']))
                    bounds_error = max(maximum(plan.lower,d['nominal_problem_lower']),
                                       maximum(plan.upper,d['nominal_problem_upper']))
                    counts['bounds_max_abs_vs_recorded'] = max(counts['bounds_max_abs_vs_recorded'],bounds_error)
                    counts['request_within_actual_bounds'] += bool(np.all(
                        plan.requested_counter_dq >= plan.lower-1e-12) and np.all(
                        plan.requested_counter_dq <= plan.upper+1e-12))
                    buffers = {k: getattr(c.low_cmd_handler, k+'_cmd') for k in ('q','dq','tau')}
                    outputs[variant] = buffers
                    counts['finite'] += bool(all(np.isfinite(v).all() for v in buffers.values()) and np.isfinite(returned).all())
                    # check the full candidate with recorded non-arm positions, not zero lower-body command fields
                    candidate = c._full_candidate(obs['q'], buffers['q'][c.arm_ids].astype(float))
                    counts['valid_candidate_pin_fcl'] += c._candidate_status(candidate, 'replay') is None
                    qi, vi, ti = (c.config['limits'][k] for k in ('q_clip_limits','dq_clip_limits','tau_clip_limits'))
                    ids = c.arm_ids
                    counts['buffers_within_publisher_limits'] += bool(
                        np.all(buffers['q'][ids] >= qi[ids,0]-2e-7) and np.all(buffers['q'][ids] <= qi[ids,1]+2e-7)
                        and np.all(abs(buffers['dq'][ids]) <= vi[ids]+2e-7)
                        and np.all(abs(buffers['tau'][ids]) <= ti[ids]+2e-6))
                    counts['backtrack_scales'][str(diag['backtrack_scale'])] += 1
                    if variant == 'c0':
                        for key, value in buffers.items():
                            original = np.asarray(row['command']['buffer'][key],dtype=np.float32)
                            out['c0_buffer_max_abs'][key] = max(out['c0_buffer_max_abs'][key],maximum(value,original))
                            out['c0_buffer_exact_rows'][key] += np.array_equal(value,original)
                            tol = np.finfo(np.float32).eps * np.maximum(1.,abs(original))
                            out['c0_buffer_within_float32_rows'][key] += bool(np.all(abs(value.astype(float)-original.astype(float)) <= tol))
                        for current, stored in ((plan.objective_matrix,d['nominal_problem_matrix']),
                                                (plan.objective_target,d['nominal_problem_target']),
                                                (plan.lower,d['nominal_problem_lower']),
                                                (plan.upper,d['nominal_problem_upper'])):
                            out['c0_problem_max_abs'] = max(out['c0_problem_max_abs'],maximum(current,stored))
                        for key, value in p['context'].items():
                            out['c0_prepared_context_max_abs'] = max(out['c0_prepared_context_max_abs'],
                                maximum(getattr(c._latest_nominal_context,key),value))
                        out['c0_requested_max_abs'] = max(out['c0_requested_max_abs'],maximum(plan.requested_counter_dq,d['requested_counter_dq']))
                        offsets = diag['counter_balance_diagnostic']['target_offsets']
                        out['c0_offset_max_abs'] = max(out['c0_offset_max_abs'],max(abs(x) for v in offsets.values() for x in v))
                    c0 = outputs['c0']
                    protected = np.setdiff1d(np.arange(27), c.counter_ids)
                    counts['noncounter_buffers_exact_c0'] += all(np.array_equal(buffers[k][protected],c0[k][protected]) for k in buffers)
                    counts['tau_exact_c0'] += np.array_equal(buffers['tau'],c0['tau'])
                    counts['delta_counter_q_peak_rad'] = max(counts['delta_counter_q_peak_rad'],maximum(buffers['q'][c.counter_ids],c0['q'][c.counter_ids]))
                    counts['delta_counter_dq_peak_rad_s'] = max(counts['delta_counter_dq_peak_rad_s'],maximum(buffers['dq'][c.counter_ids],c0['dq'][c.counter_ids]))
                    if d['balance_scale'] == 0:
                        counts['zero_scale_rows'] += 1
                        counts['zero_scale_buffers_exact_c0'] += all(np.array_equal(buffers[k],c0[k]) for k in buffers)
                if args.max_rows:
                    print(name,index,'C0',out['c0_buffer_max_abs'],'problem',out['c0_problem_max_abs'])
            result['runs'].append(out)
            print(name,'rows',len(rows),'C0',out['c0_buffer_max_abs'],'problem',out['c0_problem_max_abs'])
    result['prior_artifacts_unchanged'] = all(sha256(Path(k).read_bytes()).hexdigest()==v for k,v in old_artifacts.items())
    result['total_observations'] = sum(r['rows'] for r in result['runs'])
    result['full_controller_steps'] = result['total_observations'] * len(VARIANTS)
    result['source_files_unchanged_during_replay'] = all(sha256(Path(k).read_bytes()).hexdigest()==v for k,v in result['sources'].items())
    assert result['prior_artifacts_unchanged']
    assert result['source_files_unchanged_during_replay']
    for run in result['runs']:
        assert all(run['c0_buffer_within_float32_rows'][key]==run['rows'] for key in ('q','dq')),run
        assert run['c0_problem_max_abs'] < 1e-12 and run['c0_requested_max_abs'] < 1e-10,run
        assert run['c0_offset_max_abs'] == 0
        for counts in run['variant_counts'].values():
            for key in ('finite','published','valid_candidate_pin_fcl','request_within_actual_bounds',
                        'buffers_within_publisher_limits','noncounter_buffers_exact_c0','tau_exact_c0'):
                assert counts[key] == run['rows'],(run['path'],key,counts)
            assert counts['zero_scale_buffers_exact_c0']==counts['zero_scale_rows']
            assert counts['bounds_max_abs_vs_recorded'] == 0
    result['q_dq_and_variant_command_validation_passed'] = True
    result['historical_tau_bitwise_parity_passed'] = all(run['c0_buffer_exact_rows']['tau']==run['rows'] for run in result['runs'])
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print('OUTPUT',args.output,args.output.stat().st_size)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,default=Path.cwd())
    parser.add_argument('--max-rows',type=int,default=0)
    parser.add_argument('--output',type=Path,default=Path('/tmp/kilo/diagnostic_recorded_command_replay.json'))
    main(parser.parse_args())
