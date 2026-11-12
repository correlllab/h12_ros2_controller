"""Validate regenerated outputs against frozen compact evidence from root cwd

Run fixture, actuator and pytest first; defaults read/write only the separate
/tmp reproduction folder. Historical simulator failure provenance is retained
in sibling summary.json, so the failed raw simulator directory is not needed.
Original artifact hashes identify original /tmp files, not these portable copies.
"""

import json
import argparse
from pathlib import Path
from hashlib import sha256
from copy import deepcopy
import xml.etree.ElementTree as ET

import numpy as np

from iter7b_recovery_fixture import ROOT, OUTPUT, PARAMETERS


def digest(path):
    return sha256(Path(path).read_bytes()).hexdigest()


def compact_actuator(plant):
    result = deepcopy(plant)
    for condition in result['conditions'].values():
        for run in condition.values():
            run.pop('trace_at_control_rate', None)
            run.pop('control_trace', None)
    return result


def compact_fixture(fixture):
    result = {}
    for name, run in fixture['runs'].items():
        accepted = [r for r in run['records'] if r['status'] == 'accepted']
        rejected = next(r for r in run['records'] if r['status'] == 'validated_hold:lead')
        result[name] = {k: v for k, v in run.items() if k != 'records'}
        result[name]['accepted_peak_command_change_rad'] = max(r['command_change_rad'] for r in accepted)
        result[name]['lead_rejection_elapsed'] = rejected['elapsed']
        result[name]['replayed_rows'] = len(run['records'])
    return result


def fallback_audit(fixture):
    result = {}
    for name, run in fixture['runs'].items():
        records = run['records']
        i = next(i for i, row in enumerate(records) if row['status'] == 'validated_hold:lead')
        delta = np.array(records[i]['actual_counter_q']) - records[i - 1]['actual_counter_q']
        dv = np.array(records[i]['actual_counter_dq']) - records[i - 1]['actual_counter_dq']
        result[name] = {
            'accepted_to_fallback_max_position_jump_rad': float(np.max(abs(delta))),
            'jump_over_nominal_dt_rad_s': float(np.max(abs(delta)) / PARAMETERS['dt_s']),
            'accepted_to_fallback_max_dq_change_rad_s': float(np.max(abs(dv))),
            'normal_dq_slew_allowance_rad_s': PARAMETERS['acceleration_rad_s2'] * PARAMETERS['dt_s'],
        }
    return result


def normalize_paths(value, old_root):
    if isinstance(value, dict):
        return {k: normalize_paths(v, old_root) for k, v in value.items()}
    if isinstance(value, list):
        return [normalize_paths(v, old_root) for v in value]
    if isinstance(value, str):
        return value.replace(old_root + '/', '<ROOT>/').replace(str(ROOT) + '/', '<ROOT>/')
    return value


def main(args):
    archive = Path(__file__).resolve().parent
    frozen = json.loads((archive / 'summary.json').read_text())
    expected_plant = json.loads((archive / 'actuator_results.json').read_text())
    fixture = json.loads((args.inputs / 'fixture_results.json').read_text())
    plant = json.loads((args.inputs / 'actuator_results.json').read_text())
    tests = ET.parse(args.inputs / 'tests.xml').getroot().find('testsuite').attrib
    assert int(tests['failures']) == int(tests['errors']) == 0 and int(tests['tests']) == 26
    assert PARAMETERS == frozen['parameters'] == fixture['parameters'] == plant['parameters']
    old_root = frozen['command_cwd']
    for name, run in fixture['runs'].items():
        assert digest(run['source']) == run['source_sha256']
        assert run['baseline_q_reproduction_max_error_rad'] == 0
        assert run['pretransition_max_change_rad'] == run['moving_arm_max_change_rad'] == 0
        assert run['accepted_peak_lead_rad'] <= PARAMETERS['lead_rad']
        assert np.all(np.array(run['accepted_peak_effort_nm']) <= PARAMETERS['estimated_total_effort_nm'])
        rejected = next(r for r in run['records'] if r['status'] == 'validated_hold:lead')
        assert rejected['reference'] is None
        assert all(r['status'] == 'validated_hold:latched' and r['reference'] is None
                   for r in run['records'][rejected['row'] + 1:])
    actual_fixture = compact_fixture(fixture)
    actual_plant = compact_actuator(plant)
    assert normalize_paths(actual_fixture, old_root) == normalize_paths(frozen['recorded_state_fixture'], old_root)
    assert normalize_paths(actual_plant, old_root) == normalize_paths(expected_plant, old_root)
    assert fallback_audit(fixture) == frozen['archive']['fallback_audit']
    expected_baseline = 1 / ((150 / 30 + 12) * .02 / .0201)
    for condition in plant['conditions'].values():
        for kind, run in condition.items():
            assert digest(run['source']) == run['source_sha256']
            assert digest(run['xml']) == run['xml_sha256']
            assert run['buffer_to_plant_position_error_rad'] == run['stop_requests'] == 0
            expected = 1 / 150 if kind == 'candidate' else expected_baseline
            assert abs(run['terminal_mean_abs_elbow_error_rad'] - expected) < .0001
            if kind == 'candidate':
                assert set(run['control_status']) == {'accepted'}
                assert max(run['peak_abs_lead_rad']) < PARAMETERS['lead_rad']
                assert np.all(np.array(run['peak_abs_effort_nm']) < PARAMETERS['estimated_total_effort_nm'])
    original_exact = None
    if args.original_results is not None:
        original_fixture = json.loads((args.original_results / 'iter7b_recovery_fixture_results.json').read_text())
        original_plant = json.loads((args.original_results / 'iter7b_recovery_actuator_results.json').read_text())
        assert normalize_paths(fixture, old_root) == normalize_paths(original_fixture, old_root)
        assert normalize_paths(plant, old_root) == normalize_paths(original_plant, old_root)
        original_exact = True
    result = deepcopy(frozen)
    result['reproduction'] = {
        'all_compact_values_exact': True,
        'full_original_results_exact_when_requested': original_exact,
        'original_test_count': 26, 'rerun_tests': tests,
        'source_hashes_checked': True,
        'fallback_audit_exact': True,
        'readiness': False,
        'portable_script_hashes': {p.name: digest(p) for p in sorted(archive.glob('*.py'))},
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'output': str(args.output), **result['reproduction']}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--inputs', type=Path, default=OUTPUT)
    parser.add_argument('--output', type=Path, default=OUTPUT / 'summary.json')
    parser.add_argument('--original-results', type=Path)
    main(parser.parse_args())
