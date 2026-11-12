'''Print exact paired tables and verify the November 10 analysis ledger'''
import io
import json
import hashlib
import argparse
import contextlib
from pathlib import Path

import numpy as np


def main(args):
    root=args.root.resolve()
    path=args.results
    d=json.loads(path.read_text())
    if args.frozen_results:
        assert path.read_bytes()==args.frozen_results.read_bytes()
    records={r['run']:r for r in d['records']}
    assert len(records)==12
    assert sum(r['inventory']['samples'] for r in records.values())==4057
    mismatches=[]
    for r in records.values():
        assert r['official']['run_complete'] and r['official']['finish_reason']=='auto_stop'
        assert not r['inventory']['gaps_gt50ms']
        for name,digest in r['provenance']['input_sha256'].items():
            assert hashlib.sha256((root/r['raw_id']/name).read_bytes()).hexdigest()==digest
        mismatches.extend(dict(run=r['run'],source=k,**c) for k,c in r['provenance']['sources'].items() if c['matches'] is False)
        assert r['validation']['clock_max_error_s']==0
        assert r['validation']['desired_q_clock_max_error_rad']<1e-12
        assert r['validation']['online_timed_max_error_m']<1e-12
        assert r['validation']['invalid_state_count']==r['validation']['invalid_tick_count']==0
    assert len(mismatches)==10 and all(c['source']=='recording_code' for c in mismatches)
    assert all(c['saved']=='2a0cfb3c092f925f74e75e3e8c1f677de480bda6636e33298af540cb1aec1faa' for c in mismatches)
    print('VERIFIED 12runs 4057rows; raw hashes, clocks, commands and online timed metrics; 10 recorder-source mismatches retained')
    print('RECORDER_MISMATCHES',json.dumps(mismatches))
    print('SOURCE_HASHES',json.dumps({k:v['saved'] for k,v in d['records'][0]['provenance']['sources'].items()}))
    print('MAX_ONLINE_ERROR_M',max(r['validation']['online_timed_max_error_m'] for r in records.values()))
    print('CONFIG_HASHES',set(r['inventory']['resolved_config_sha256'] for r in records.values()))
    print('GIT',json.dumps(d['records'][0]['inventory']['git']))
    print('INITIAL | run | tilt | standing drift | standing capture tilt | standing roll,pitch,yaw | gyro | stance width | stance stagger | com-ahead | prepared idle s')
    for name in d['chronological_runs']:
        r=records[name]
        x=r['initial']
        event={e['event']:e['relative_s'] for e in r['inventory']['events_relative_epoch']}
        print(name,*(f'{x[k]:.9f}' for k in ('tilt_deg','standing_deg','standing_capture_tilt_deg')),
            [round(v,9) for v in x['relative_roll_pitch_yaw_deg']],
            *(f'{x[k]:.9f}' for k in ('gyro_rad_s','stance_width_m','stance_stagger_m','com_ahead_of_midfoot_m')),
            x['start_after_preparation_s'],r['official']['classification'])
    for title, keys, source in (
        ('MOVE BALANCE RMS',('standing_deg','onset_so3_deg','onset_gravity_deg','tilt_deviation_deg','gyro_rad_s'),'phase'),
        ('TRACKING',('timed_position_error_rms_m','configuration_error_rms_rad','frozen_uniform_rms_m','live_uniform_rms_m','frozen_endpoint_error_m','frozen_hold_error_rms_m'),'official'),
        ('TERMINAL',('standing_deg','onset_so3_deg','onset_gravity_deg','tilt_deg','gyro_rad_s','endpoint_mm','live_endpoint_mm'),'tail')):
        print(title,keys)
        for p in d['pairs']:
            f,c=records[p['case']+'_frame'],records[p['case']+'_counter']
            vals=[]
            for key in keys:
                if source=='official':
                    scale=1000 if key.endswith('_m') else 1
                    x,y=[r['official'][key]*scale for r in (f,c)]
                else:
                    phase='move' if source=='phase' else 'tail'
                    field='rms_time' if source=='phase' else 'mean'
                    x,y=[r['phases'][phase]['metrics'][key][field] for r in (f,c)]
                vals.append(f'{x:.9f}/{y:.9f}')
            print(p['case'],' | '.join(vals))
    print('PHASE PEAKS onsetSO3/standing/gyro; all F/C')
    for p in d['pairs']:
        f,c=records[p['case']+'_frame'],records[p['case']+'_counter']
        for phase in ('move','fade','postfade','late','tail'):
            print(p['case'],phase,' | '.join('/'.join(f"{r['phases'][phase]['metrics'][key]['peak_abs']:.9f}" for r in (f,c))
                for key in ('onset_so3_deg','standing_deg','gyro_rad_s')))
    print('COMMON MOVE and100ms trim: timed q spatial tilt onset gravity gyro (delta C-F)')
    for p in d['pairs']:
        for phase in ('move','onset_trim100ms'):
            w=p['common_windows'][phase]
            grid={k:w['counter']['exact_reference_grid'][k]-w['frame']['exact_reference_grid'][k]
                  for k in ('timed_mm','q_rad','frozen_uniform_mm','live_uniform_mm','endpoint_at_window_end_mm')}
            print(p['case'],phase,'window',w['window_s'],grid,
                  {k:w['deltas'][k] for k in ('tilt_deviation_deg_rms','onset_so3_deg_rms','onset_gravity_deg_rms','gyro_rad_s_rms')})
    print('CLEAN_LATE/Tail deltas standing onset gravity gyro endpoint live_endpoint')
    for p in d['pairs']:
        for phase in ('clean_late','common_tail'):
            w=p['common_windows'][phase]
            print(p['case'],phase,w['max_support_gap_ms'],
                  {k:w['deltas'][k] for k in ('standing_deg_mean','onset_so3_deg_mean','onset_gravity_deg_mean','gyro_rad_s_mean','endpoint_mm_rms','live_endpoint_mm_rms')})
    print('COMMON END_HOLD frozen/live endpoint RMS F/C and deltas')
    for p in d['pairs']:
        w=p['common_windows']['end_hold']
        print(p['case'],{key:{'frame':w['frame'][key],'counter':w['counter'][key], 'delta':w['deltas'][key]}
                        for key in ('endpoint_mm_rms','live_endpoint_mm_rms')})
    print('TIMING run N duration onset max phase gap; move command mean/p95; apply mean; skips')
    for name in d['chronological_runs']:
        r=records[name]
        timing=r['timing']
        print(name,r['inventory']['samples'],r['official']['move_duration_s'],r['inventory']['first_s']*1000,
            max(p['max_gap_ms'] for p in r['phases'].values()),
            timing['move_command_interval_ms']['mean'],timing['move_command_interval_ms']['p95'],
            timing['move_control_ms']['mean'],timing['first_skipped_slots'],timing['later_skipped_slots'])
    print('NOV6 SAME CELLS ONLY tilt timed frozen residual onset_so3')
    for p in d['old_nov6_separate']['matched_pairs']:
        print(p['pair'],{k:p['metrics'][k] for k in ('move_rms_tilt_deviation_deg','timed_position_error_rms_m','frozen_uniform_rms_m','residual_drift_deg','onset_so3_motion_rms_deg')})
    out=args.output
    out.write_text(json.dumps(dict(result_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        verified=True,rows=4057,runs=12,pairs=6,all_source_hashes_match=False,
        source_mismatches=mismatches),indent=2)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path.cwd())
    parser.add_argument('--results',type=Path,default=Path(__file__).resolve().with_name('results.json'))
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().with_name('verification.json'))
    parser.add_argument('--frozen-results',type=Path)
    parser.add_argument('--tables-output',type=Path,default=Path(__file__).resolve().with_name('tables.txt'))
    args=parser.parse_args()
    if args.tables_output:
        buffer=io.StringIO()
        with contextlib.redirect_stdout(buffer):
            main(args)
        args.tables_output.write_text(buffer.getvalue())
        print('VERIFIED; tables written to',args.tables_output)
    else:
        main(args)
