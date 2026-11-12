"""Replay from benchmark root cwd; outputs stay in /tmp, no DDS initialized

Frozen prototype behavior: fallback checks endpoints, not command slew, and
ignores its write result. Retained for reproduction, not hardware readiness.
"""

import sys
import json
import argparse
import threading
from copy import deepcopy
from pathlib import Path
from hashlib import sha256
from types import SimpleNamespace
from collections import Counter

import numpy as np

ROOT = Path.cwd().resolve()
OUTPUT = Path('/tmp/kilo/iteration7b_recovery_reproduction')
if not (ROOT / 'h12_zmp_benchmark').is_dir():
    raise RuntimeError('Run from the h12_zmp_benchmark repository root')
sys.path.insert(0, str(ROOT / 'submodules/h12_ros2_controller'))

from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.core.low_cmd_handler import LowCmdHandler
from h12_ros2_controller.core.controller.counter_balance.counter_ddp_velocity_robust_controller import CounterDDPVelocityRobustController


PARAMETERS = {
    'phase': 'strictly after T+0.5 s; baseline during movement and fade',
    'desired_speed_rad_s': 0.20,
    'acceleration_rad_s2': 1.0,
    'lead_rad': 0.04,
    'estimated_total_effort_nm': [24.0, 24.0, 10.8, 10.8],
    'dt_s': 1 / 30,
    'scope': 'engineering fixture limits, not commissioned hardware safety limits',
}


def arrays(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def sample(row):
    diag = row['diagnostics']['controller_diagnostics']
    prepared = diag['nominal_prepared_context']
    ctx = prepared['context']
    return (diag, prepared, np.array(ctx['motor_q']),
            np.array(prepared['observation']['dq']),
            np.array(ctx['arm_q']), np.array(ctx['arm_dq']),
            np.array(row['command']['buffer']['tau'])[13:])


class Harness:
    def __init__(self, row, config, model, moving_arm):
        diag, prepared, q, dq, aq, av, tau = sample(row)
        pub = prepared['publication']
        refs = prepared['references']
        c = CounterDDPVelocityRobustController.__new__(CounterDDPVelocityRobustController)
        c.arm_ids = np.arange(13, 27)
        c.moving_ids = np.arange(13, 20) if moving_arm == 'left' else np.arange(20, 27)
        c.counter_arm_ids = np.arange(20, 27) if moving_arm == 'left' else np.arange(13, 20)
        c.counter_ids = c.counter_arm_ids[:4]
        c.counter_local = c.counter_arm_ids - 13
        c.counter_active_local = c.counter_ids - 13
        c.counter_wrist_local = c.counter_local[4:]
        c.moving_local = c.moving_ids - 13
        c.motor_q_indices = np.arange(27)
        c.motor_v_indices = np.arange(27)
        c.dt = pub['dt']
        c.dq_lim = pub['dq_lim']
        c.max_velocity = np.array(config['reactive_counter_balance']['max_velocity'])
        c.max_excursion = np.array(config['reactive_counter_balance']['max_excursion'])
        c.q_counter_ref = np.array(refs['q_counter_ref'])
        c.counter_wrist_ref = np.array(refs['counter_wrist_ref'])
        c.backtrack_scales = pub['backtrack_scales']
        c.robot_model = model
        c.config = {'limits': {k: np.array(v) for k, v in pub['limits'].items()}}
        handler = LowCmdHandler.__new__(LowCmdHandler)
        handler._q_clip_limits = c.config['limits']['q_clip_limits']
        handler._dq_clip_limits = c.config['limits']['dq_clip_limits']
        handler._tau_clip_limits = c.config['limits']['tau_clip_limits']
        handler._recording = False
        # memory-only transport boundary; actual production clipping/writer methods run
        handler._command_publisher = SimpleNamespace(
            data_lock=threading.Lock(), q=q.astype(np.float32),
            dq=np.zeros(27, dtype=np.float32), tau=np.zeros(27, dtype=np.float32),
            mode=np.ones(27, dtype=np.int32),
        )
        c.low_cmd_handler = handler
        self.c = c
        assert len(config['gains']['kp']) == len(config['gains']['kd']) == 27
        self.kp = np.array(config['gains']['kp'])[c.counter_ids]
        self.kd = np.array(config['gains']['kd'])[c.counter_ids]
        assert np.array_equal(self.kp, [240, 200, 150, 150])
        assert np.array_equal(self.kd, [12, 12, 12, 12])
        self.stop_requests = 0
        self.writes = 0
        self.proposals = []

    def write(self, aq, av, tau):
        self.writes += 1
        return self.c._write_arm_command(aq.copy(), av.copy(), tau.copy())

    def baseline(self, row):
        diag, prepared, q, dq, aq, av, tau = sample(row)
        result, reason = self.c._backtrack_counter(
            q, aq, av, np.array(prepared['context']['counter_q']),
            np.array(diag['requested_counter_dq']),
        )
        if result is None:
            raise AssertionError(f'baseline replay rejected: {reason}')
        out_q, out_v, scale = result
        return out_q, out_v, tau

    def check(self, q, aq):
        full = self.c._full_candidate(q, aq)
        self.proposals.append(full.copy())
        return self.c._candidate_status(full, 'recovery_candidate')

    def stop(self):
        # explicit fixture supervisor, not verification of any real relay/lease
        self.stop_requests += 1
        pub = self.c.low_cmd_handler._command_publisher
        pub.mode[self.c.counter_arm_ids] = 0
        pub.dq[self.c.counter_arm_ids] = 0
        pub.tau[self.c.counter_arm_ids] = 0


class Recovery:
    def __init__(self, harness, token):
        self.h = harness
        self.token = token
        self.reference = None
        self.velocity = None
        self.last_q = None
        self.last_v = None
        self.entered = False
        self.latched = False
        self.stop_latched = False
        self.accepted = 0

    def invalidate(self):
        self.reference = None
        self.velocity = None
        self.latched = True

    def rearm(self, token):
        self.token = token
        self.reference = None
        self.velocity = None
        self.entered = False
        self.latched = False
        self.stop_latched = False
        c = self.h.c
        self.last_q = c.low_cmd_handler.q_cmd[13:]
        self.last_v = c.low_cmd_handler.dq_cmd[13:]

    def effort(self, q, dq, aq, av, tau):
        c = self.h.c
        ci, local = c.counter_ids, c.counter_active_local
        clipped_tau = np.clip(tau[local], -c.config['limits']['tau_clip_limits'][ci],
                              c.config['limits']['tau_clip_limits'][ci])
        return self.h.kp * (aq[local] - q[ci]) + self.h.kd * (av[local] - dq[ci]) + clipped_tau

    def validate(self, q, dq, aq, av, tau):
        c = self.h.c
        ci, local = c.counter_ids, c.counter_active_local
        reason = self.h.check(q, aq)
        if reason is not None:
            return reason
        if np.any(abs(aq[local] - c.q_counter_ref) > c.max_excursion + 1e-12):
            return 'excursion'
        if np.any(abs(aq[local] - q[ci]) > PARAMETERS['lead_rad'] + 1e-12):
            return 'lead'
        if np.any(abs(av[local]) > c._arm_velocity_limits()[local] + 1e-12):
            return 'velocity'
        if np.any(abs(self.effort(q, dq, aq, av, tau)) > np.array(PARAMETERS['estimated_total_effort_nm']) + 1e-12):
            return 'effort'
        return None

    def fallback(self, q, dq, aq, av, tau, reason):
        self.invalidate()
        c = self.h.c
        aq = aq.copy()
        av = av.copy()
        aq[c.counter_local] = q[c.counter_arm_ids]
        av[c.counter_local] = 0
        invalid = self.validate(q, dq, aq, av, tau)
        if invalid is not None:
            self.stop_latched = True
            self.h.stop()
            return 'stop_required:' + reason + ':' + invalid
        self.h.write(aq, av, tau)
        self.last_q = c.low_cmd_handler.q_cmd[13:].astype(float)
        self.last_v = c.low_cmd_handler.dq_cmd[13:].astype(float)
        return 'validated_hold:' + reason

    def step(self, q, dq, aq, av, tau, postfade, *, fresh=True,
             eligible=True, stopped=False, token=None):
        c = self.h.c
        if not fresh or stopped:
            self.invalidate()
            self.stop_latched = True
            self.h.stop()
            return 'stop_required:stale' if not fresh else 'stop_required:operator'
        if self.stop_latched:
            return 'stop_latched_no_write'
        if token is not None and token != self.token:
            return self.fallback(q, dq, aq, av, tau, 'new_target')
        if not eligible:
            return 'skipped_no_write'
        if self.latched:
            return self.fallback(q, dq, aq, av, tau, 'latched')
        if not postfade:
            self.h.write(aq, av, tau)
            self.last_q = c.low_cmd_handler.q_cmd[13:].astype(float)
            self.last_v = c.low_cmd_handler.dq_cmd[13:].astype(float)
            return 'baseline'
        if not self.entered:
            if self.last_q is None:
                return self.fallback(q, dq, aq, av, tau, 'missing_seed')
            self.reference = self.last_q[c.counter_active_local].copy()
            self.velocity = self.last_v[c.counter_active_local].copy()
            self.entered = True
        old_ref, old_v = self.reference.copy(), self.velocity.copy()
        error = c.q_counter_ref - old_ref
        desired_v = np.clip(error / c.dt, -PARAMETERS['desired_speed_rad_s'], PARAMETERS['desired_speed_rad_s'])
        new_v = old_v + np.clip(desired_v - old_v, -PARAMETERS['acceleration_rad_s2'] * c.dt,
                               PARAMETERS['acceleration_rad_s2'] * c.dt)
        new_ref = (old_ref + c.dt * new_v).astype(np.float32).astype(float)
        new_v = new_v.astype(np.float32).astype(float)
        proposal_q, proposal_v = aq.copy(), av.copy()
        proposal_q[c.counter_active_local] = new_ref
        proposal_v[c.counter_active_local] = new_v
        proposal_q[c.counter_wrist_local] = c.counter_wrist_ref
        proposal_v[c.counter_wrist_local] = 0
        reason = self.validate(q, dq, proposal_q, proposal_v, tau)
        if reason is not None:
            return self.fallback(q, dq, aq, av, tau, reason)
        if not self.h.write(proposal_q, proposal_v, tau):
            self.invalidate()
            self.stop_latched = True
            self.h.stop()
            return 'stop_required:publication'
        actual_q = c.low_cmd_handler.q_cmd[c.counter_ids]
        actual_v = c.low_cmd_handler.dq_cmd[c.counter_ids]
        if (not np.array_equal(actual_q, new_ref.astype(np.float32))
                or not np.array_equal(actual_v, new_v.astype(np.float32))):
            self.invalidate()
            self.stop_latched = True
            self.h.stop()
            return 'stop_required:clipped'
        self.reference, self.velocity = actual_q.astype(float), actual_v.astype(float)
        self.last_q = c.low_cmd_handler.q_cmd[13:].astype(float)
        self.last_v = c.low_cmd_handler.dq_cmd[13:].astype(float)
        self.accepted += 1
        assert np.max(abs((self.reference - old_ref) / c.dt - self.velocity)) < 3e-6
        assert np.max(abs(self.velocity - old_v)) <= PARAMETERS['acceleration_rad_s2'] * c.dt + 1e-7
        return 'accepted'


def make_model(limits):
    assets = ROOT / 'submodules/h12_ros2_controller/submodules/CL_Assets/ros_assets'
    model = RobotModel(str(assets / 'h1_2_magpie.urdf'))
    model.apply_position_limits(np.array(limits))
    model.init_collision_model(str(assets / 'h1_2_magpie_sphere.urdf'),
                               str(assets / 'h1_2_magpie_sphere_collision.srdf'))
    return model


def replay(path):
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    meta = json.loads(path.with_name('metadata.json').read_text())
    config = json.loads(path.with_name('resolved_config.yaml').read_text())['controller']
    limits = rows[0]['diagnostics']['controller_diagnostics']['nominal_prepared_context']['publication']['limits']['q_clip_limits']
    model = make_model(limits)
    h = Harness(rows[0], config, model, meta['task']['moving_arm'])
    recovery = Recovery(h, path.parent.name)
    T = meta['task']['move_duration']
    records = []
    baseline_error = 0.0
    for i, row in enumerate(rows):
        diag, prepared, q, dq, aq, av, tau = sample(row)
        bq, bv, tau = h.baseline(row)
        published_bq = np.clip(bq, h.c.config['limits']['q_clip_limits'][13:, 0],
                               h.c.config['limits']['q_clip_limits'][13:, 1]).astype(np.float32)
        baseline_error = max(baseline_error, float(np.max(abs(published_bq - np.array(row['command']['buffer']['q'])[13:]))))
        postfade = row['command_monotonic'] - meta['execution_start_monotonic'] > T + .5
        status = recovery.step(q, dq, bq, bv, tau, postfade)
        actual_q = h.c.low_cmd_handler.q_cmd
        actual_v = h.c.low_cmd_handler.dq_cmd
        local = h.c.counter_active_local
        records.append({
            'row': i, 'elapsed': row['elapsed'], 'postfade': postfade,
            'status': status, 'command_change_rad': float(np.max(abs(actual_q[13:] - published_bq))),
            'moving_command_change_rad': float(np.max(abs(actual_q[h.c.moving_ids] - published_bq[h.c.moving_local]))),
            'actual_counter_q': actual_q[h.c.counter_ids],
            'actual_counter_dq': actual_v[h.c.counter_ids],
            'lead_rad': actual_q[h.c.counter_ids] - q[h.c.counter_ids],
            'estimated_total_effort_nm': recovery.effort(q, dq, actual_q[13:], actual_v[13:], tau),
            'reference': None if recovery.reference is None else recovery.reference.copy(),
        })
        if not postfade:
            assert records[-1]['command_change_rad'] == 0
    accepted = [r for r in records if r['status'] == 'accepted']
    return {
        'source': str(path), 'source_sha256': sha256(path.read_bytes()).hexdigest(),
        'records': records, 'status_counts': dict(Counter(r['status'] for r in records)),
        'baseline_q_reproduction_max_error_rad': baseline_error,
        'pretransition_max_change_rad': max(r['command_change_rad'] for r in records if not r['postfade']),
        'moving_arm_max_change_rad': max(r['moving_command_change_rad'] for r in records),
        'accepted_peak_lead_rad': max((float(np.max(abs(r['lead_rad']))) for r in accepted), default=None),
        'accepted_peak_effort_nm': np.max(np.abs([r['estimated_total_effort_nm'] for r in accepted]), axis=0) if accepted else None,
        'stop_requests': h.stop_requests,
        'physical_recovery_result': None,
    }, (h, recovery, rows, meta, config)


def main(args):
    campaign = ROOT / 'runs/real_experiments/iter7b_fame_validation_20261110_184722'
    results = {
        'scope': 'recorded-state command replay; measured states frozen, no physical recovery or whole-body prediction',
        'development_failures': [
            'Initial fixture indexed resolved 27-joint gains as raw 15-joint gains; rejected output retained at iter7b_recovery_fixture_invalid_gain_index_results.json. Fixed to motor indices; no candidate parameters changed.',
            'Initial memory buffers used float64 unlike deployed float32. Replaced with actual publisher dtypes and seed/commit from emitted buffer; prior output retained at iter7b_recovery_fixture_pre_float32_results.json. Quantization tolerances explicit, no candidate parameters changed.',
        ],
        'parameters': PARAMETERS,
        'config_diff': {'production': {}, 'temporary': PARAMETERS},
        'transport': 'in-memory publisher only; LowCmdHandler clipping and controller writer are actual deployed methods; no DDS initialized',
        'supervisor': 'fixture callback only; real command lease/relay/stale output is NOT verified',
        'runs': {},
    }
    for name in ('F02_counter', 'F03_counter', 'F03r_counter'):
        path = next((campaign / name).glob('*/arm_target.jsonl'))
        results['runs'][name], _ = replay(path)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(results, indent=2, default=arrays) + '\n')
    print(json.dumps({name: {k: v for k, v in result.items() if k != 'records'}
                      for name, result in results['runs'].items()}, indent=2, default=arrays))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=OUTPUT / 'fixture_results.json')
    main(parser.parse_args())
