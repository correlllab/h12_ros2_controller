"""Run fixed-rest ideal-PD hypotheses from root cwd; not robot validation

Uses only in-memory command buffers. Outputs default to a separate /tmp folder.
"""

import json
import argparse
from pathlib import Path
from hashlib import sha256
from collections import Counter
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin

from iter7b_recovery_fixture import ROOT, OUTPUT, PARAMETERS, Harness, Recovery, make_model, sample, arrays


def run_cell(path, sign, candidate):
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    meta = json.loads(path.with_name('metadata.json').read_text())
    config = json.loads(path.with_name('resolved_config.yaml').read_text())['controller']
    T = meta['task']['move_duration']
    row = next(row for row in rows if row['elapsed'] >= T + 8)
    diag, prepared, anchor_q, _, aq, _, _ = sample(row)
    model = make_model(prepared['publication']['limits']['q_clip_limits'])
    h = Harness(row, config, model, meta['task']['moving_arm'])
    c = h.c
    ci, local = c.counter_ids, c.counter_active_local
    xml = ROOT / 'submodules/CL_Assets/mujoco_assets/h1_2_magpie.xml'
    default_joint = ET.parse(xml).getroot().find("default/default[@class='h1_2']/joint")
    damping = float(default_joint.attrib['damping'])
    armature = float(default_joint.attrib['armature'])
    mass = pin.crba(model.model_body, model.data_body, anchor_q)[np.ix_(ci, ci)].copy()
    mass += np.eye(4) * armature
    inverse_mass = np.linalg.inv(mass)
    assert np.min(np.linalg.eigvalsh(mass)) > 0
    q, dq = anchor_q.copy(), np.zeros(27)
    q[ci] = c.q_counter_ref + sign * .10
    aq = aq.copy()
    aq[c.counter_local] = q[c.counter_arm_ids]
    aq[c.counter_wrist_local] = c.counter_wrist_ref
    av, tau = np.zeros(14), np.zeros(14)
    recovery = Recovery(h, 'fixed_rest')
    coeff = config['reactive_counter_balance']['weights']['posture']
    coeff /= coeff + config['reactive_counter_balance']['damping']
    records = []
    control_records = []
    next_control = 0.0
    integration_dt = .001
    seeded = False
    max_interface_error = 0.0
    for tick in range(10001):
        time = tick * integration_dt
        if time >= next_control - 1e-12:
            next_control += c.dt
            lo, hi = c._counter_velocity_bounds(q[ci])
            lo, hi = c._counter_excursion_bounds(q[ci], lo, hi)
            requested = np.clip(-coeff * (q[ci] - c.q_counter_ref), lo, hi)
            accepted, reason = c._backtrack_counter(q, aq, av, q[ci], requested)
            if accepted is None:
                h.stop()
                status = 'baseline_invalid:' + str(reason)
            else:
                bq, bv, _ = accepted
                if candidate:
                    if not seeded:
                        recovery.step(q, dq, bq, bv, tau, False)
                        seeded = True
                    status = recovery.step(q, dq, bq, bv, tau, True)
                else:
                    h.write(bq, bv, tau)
                    status = 'baseline'
            control_records.append({'time': time, 'status': status})
        received_q = h.c.low_cmd_handler.q_cmd[ci].astype(float)
        received_v = h.c.low_cmd_handler.dq_cmd[ci].astype(float)
        max_interface_error = max(max_interface_error,
                                  float(np.max(abs(received_q - h.c.low_cmd_handler.q_cmd[ci]))))
        effort = h.kp * (received_q - q[ci]) + h.kd * (received_v - dq[ci])
        enabled = h.c.low_cmd_handler._command_publisher.mode[ci] == 1
        effort = np.where(enabled, effort, 0)
        records.append({'time': time, 'error': q[ci] - c.q_counter_ref,
                        'q': q[ci].copy(), 'dq': dq[ci].copy(),
                        'lead': received_q - q[ci], 'effort': effort.copy(),
                        'received_q': received_q, 'received_dq': received_v})
        acceleration = inverse_mass @ (effort + sign * np.ones(4) - damping * dq[ci])
        dq[ci] += integration_dt * acceleration
        q[ci] += integration_dt * dq[ci]
    tail = [r for r in records if 8 <= r['time'] <= 10]
    result = {
        'candidate': candidate, 'load_sign': sign,
        'source': str(path), 'source_sha256': sha256(path.read_bytes()).hexdigest(),
        'anchor_elapsed': row['elapsed'], 'inertia_matrix_kg_m2': mass,
        'inertia_eigenvalues': np.linalg.eigvalsh(mass),
        'xml': str(xml), 'xml_sha256': sha256(xml.read_bytes()).hexdigest(),
        'xml_damping': damping, 'xml_armature': armature,
        'xml_frictionloss_omitted': float(default_joint.attrib['frictionloss']),
        'terminal_mean_abs_elbow_error_rad': float(np.mean([abs(r['error'][3]) for r in tail])),
        'terminal_rms_fourjoint_error_rad': float(np.sqrt(np.mean([np.dot(r['error'], r['error']) for r in tail]))),
        'peak_abs_lead_rad': np.max(np.abs([r['lead'] for r in records]), axis=0),
        'peak_abs_effort_nm': np.max(np.abs([r['effort'] for r in records]), axis=0),
        'peak_measured_speed_rad_s': np.max(np.abs([r['dq'] for r in records]), axis=0),
        'control_status': dict(Counter(r['status'] for r in control_records)),
        'buffer_to_plant_position_error_rad': max_interface_error,
        'stop_requests': h.stop_requests,
        'trace_at_control_rate': records[::33],
        'control_trace': control_records,
    }
    return result


def main(args):
    campaign = ROOT / 'runs/real_experiments/iter7b_fame_validation_20261110_184722'
    result = {
        'scope': 'synthetic linear four-joint actuator; all other coordinates fixed, no base/policy/contact dynamics',
        'not_physical_validation': True,
        'development_failures': ['First invocation stopped before any plant cell: XML joint default is nested under class h1_2, not default/joint. Corrected selector, no physical values changed.'],
        'equation': 'Mcc*qdd + damping*dq = kp*(buffer_q-q) + kd*(buffer_dq-dq) + sign*[1,1,1,1]',
        'assumptions': [
            'Mcc frozen at T+8 recorded state, plus unchanged XML armature; not fitted',
            'Gravity exactly compensated; zero FF residual except arbitrary predeclared +/-1 Nm disturbance',
            'XML frictionloss omitted; no Coulomb friction, Coriolis, gravity gradient or coupling to prescribed rest',
            'Actual float32 buffer drives plant, 30 Hz references, 1 ms semi-implicit integration',
            'Plant assumes ideal immediate PD realization; no claim about real motor/relay authority or latency',
        ],
        'parameters': PARAMETERS,
        'conditions': {},
        'comparisons': [],
    }
    for name in ('F02_counter', 'F03_counter'):
        path = next((campaign / name).glob('*/arm_target.jsonl'))
        for sign in (-1, 1):
            base = run_cell(path, sign, False)
            candidate = run_cell(path, sign, True)
            key = f'{name}_sign{sign:+d}'
            result['conditions'][key] = {'baseline': base, 'candidate': candidate}
            b = base['terminal_mean_abs_elbow_error_rad']
            c = candidate['terminal_mean_abs_elbow_error_rad']
            comparison = {'condition': key, 'baseline_elbow_rad': b, 'candidate_elbow_rad': c,
                          'reduction_rad': b - c, 'reduction_fraction': 1 - c / b,
                          'mechanism_threshold_met': b - c >= .02 and c <= .75 * b,
                          'candidate_only_accepted': set(candidate['control_status']) == {'accepted'}}
            result['comparisons'].append(comparison)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, default=arrays) + '\n')
    print(json.dumps(result['comparisons'], indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=OUTPUT / 'actuator_results.json')
    main(parser.parse_args())
