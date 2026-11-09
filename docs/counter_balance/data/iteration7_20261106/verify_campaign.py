'''Verify sibling frozen results against benchmark-root raw and source files'''
import json
import hashlib
from pathlib import Path


def main():
    root = Path.cwd()
    data = json.loads(Path(__file__).with_name('results.json').read_text())
    records, pairs = data['records'], data['pairs']
    assert len(records) == 12 and len(pairs) == 6
    assert len({r['raw_id'] for r in records}) == 12
    assert sum(r['sample_count'] for r in records) == 4035
    assert {r['case'] for r in records if not r['run_complete']} == {'B3', 'R3'}
    assert sum(r['quality']['terminal_continuity_warning'] for r in records) == 1
    assert len({r['provenance']['resolved_config_sha256'] for r in records}) == 1
    for r in records:
        assert r['quality']['state_valid_false_count'] == 0
        assert r['quality']['all_zero_q_rows'] == 0
        assert r['command_clock_q_max_abs_error_rad'] == 0
        assert r['online_sample_timed_max_recompute_error_m'] < 1e-12
        assert abs(r['online_timed_rms_m'] - r['timed_position_error_rms_m']) < 1e-12
        assert all(v['matches'] is not False for v in r['provenance']['sources'].values())
    for r in records + data['historical_matching_records']:
        trial = root / r['raw_id']
        raw = trial / 'arm_target.jsonl'
        assert hashlib.sha256(raw.read_bytes()).hexdigest() == r['provenance']['raw_sha256']
        summary = json.loads((trial / 'summary.json').read_text())
        assert r['raw_flags'] == {key: summary.get(key) for key in r['raw_flags']}
        provenance = json.loads((trial / 'provenance.json').read_text())
        for name, frozen in r['provenance']['sources'].items():
            source = provenance['sources'][name]
            assert frozen['saved_sha256'] == source.get('sha256')
            if frozen['current_sha256'] is not None:
                path = root / source['path'].split('/h12_zmp_benchmark/', 1)[-1]
                assert hashlib.sha256(path.read_bytes()).hexdigest() == frozen['current_sha256']
        metadata = json.loads((trial / 'metadata.json').read_text())
        task = metadata['task']
        payload = root / 'data/trajectory_banks' / task['bank'] / task['npz_path']
        assert hashlib.sha256(payload.read_bytes()).hexdigest() == r['provenance']['payload_sha256']
    for p in pairs:
        common = p['common_grid_sensitivity']
        assert common['counter']['q_rms_rad'] > common['frame']['q_rms_rad']
        assert common['counter']['timed_rms_m'] > common['frame']['timed_rms_m']
        tail = p['continuous_late_hold_sensitivity']
        assert tail['counter']['continuous_at_50ms'] and tail['frame']['continuous_at_50ms']
        assert tail['counter']['drift_mean_deg'] > tail['frame']['drift_mean_deg']
    for r in records:
        if r['case'] in ('B2', 'B3', 'R3'):
            print(r['case'], r['controller'], json.dumps(r['phase_peaks']))
    print('PASS: 20 raw/payload hashes and available frozen source hashes unchanged; raw flags preserved; 12 campaign runs, 4035 samples, 6 explicit pairs; matching clocks and 6/6 common-grid timed/q regressions and continuous-tail standing drift regressions')


if __name__ == '__main__':
    main()
