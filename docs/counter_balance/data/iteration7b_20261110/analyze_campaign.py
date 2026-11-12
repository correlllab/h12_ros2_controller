'''Analyze the explicit November 10 FAME campaign without changing source data

The ledger's script_sha256 preserves the original analysis source identity.
Only retention paths and that provenance reference differ from the frozen script.
'''
import sys
import json
import hashlib
import argparse
from pathlib import Path

import numpy as np


def load(path):
    return json.loads(path.read_text())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stats(values):
    x = np.asarray(values)
    return {k: float(v) for k, v in zip(('min', 'mean', 'p50', 'p95', 'max'),
            (x.min(), x.mean(), np.median(x), np.percentile(x, 95), x.max()))} if len(x) else None


def main(args):
    root = args.root.resolve()
    sys.path.insert(0, str(root))
    from h12_zmp_benchmark.experiment import analyze_real_runs as a
    from h12_zmp_benchmark.utility.straight_line_trajectory import load_trajectory

    campaign = root / args.campaign
    cases = ('F01', 'F02', 'F03', 'F04', 'F03r', 'F04r')
    raw_paths = sorted(campaign.rglob('arm_target.jsonl'))
    assert len(raw_paths) == 12
    inventory = []
    for raw in raw_paths:
        trial = raw.parent
        m, su, prov = [load(trial / n) for n in ('metadata.json', 'summary.json', 'provenance.json')]
        rows = [json.loads(line) for line in raw.read_text().splitlines() if line.strip()]
        t = np.array([r['elapsed_s'] for r in rows])
        events = [json.loads(line) for line in (trial.parent / 'events.jsonl').read_text().splitlines()]
        item = dict(run=trial.parent.name, target=trial.name, policy=m['policy'], controller=m['controller'],
                    created_utc=m['created_utc'], samples=len(rows), first_s=float(t[0]), last_s=float(t[-1]),
                    move_duration_s=m['task']['move_duration'], raw_sha256=sha(raw),
                    flags={k: su.get(k) for k in ('run_complete', 'window_complete', 'interrupted',
                       'finish_reason', 'clock_fault', 'final_state_valid', 'physical_classification_available',
                       'actuation_acknowledged', 'dds_delivery_confirmed')}, coverage=su['coverage'],
                    external_policy_version=m.get('external_policy_version'), git=prov['git'],
                    resolved_config_sha256=prov['resolved_config_sha256'],
                    gaps_gt50ms=[dict(before_s=float(t[i]), after_s=float(t[i+1]), gap_ms=float(dt*1000))
                                for i, dt in enumerate(np.diff(t)) if dt > .05],
                    events_relative_epoch=[dict(event=e.get('event', e.get('kind')),
                        relative_s=e.get('arrival_monotonic', e.get('execution_start_monotonic',
                            e.get('first_command_monotonic', m['execution_start_monotonic'])))
                            - m['execution_start_monotonic']) for e in events])
        inventory.append(item)
    inventory.sort(key=lambda r: r['created_utc'])
    if not args.quiet:
        for r in inventory:
            print('INVENTORY', r['run'], r['created_utc'], r['samples'], r['first_s'], r['flags'], r['gaps_gt50ms'], flush=True)
    if args.inventory_only:
        args.output.write_text(json.dumps(inventory, indent=2))
        return

    cache = {}
    records = []
    for raw in raw_paths:
        trial = raw.parent
        m, su, prov = [load(trial / n) for n in ('metadata.json', 'summary.json', 'provenance.json')]
        task = m['task']
        trajectory_path = root / 'data/trajectory_banks' / task['bank'] / (task['id'] + '.json')
        trajectory = load_trajectory(trajectory_path)
        assert sha(trajectory_path) == prov['sources']['trajectory']['sha256']
        assert task['npz_sha256'] == trajectory.metadata['npz_sha256']
        for key in ('line', 'nominal_duration', 'joint_names', 'moving_arm'):
            assert task[key] == trajectory.metadata[key]
        trajectory.set_acceleration_limit(task['speed_acceleration_limit'])
        s = a.read_samples(trial)
        t, T = s['time'], float(task['move_duration'])
        assert np.isclose(T, trajectory.nominal_duration * .8)
        assert np.all(np.diff(t) > 0)
        assert np.allclose(s['standing'], load(trial / 'standing_capture.json')['R0'])
        rows = [json.loads(line) for line in raw.read_text().splitlines() if line.strip()]
        qref, _, _, pref, _ = trajectory.sample(t, duration=T)
        arm = trajectory.moving_arm
        off = 0 if arm == 'left' else 7
        wrist = s[arm + '_wrist']
        pelvis = s['pelvis']
        onset = np.array([a.rotation_angle(pelvis[0].T @ r) for r in pelvis])
        gravity = np.arccos(np.clip(pelvis[:, 2, :] @ pelvis[0, 2, :], -1, 1))
        qerr = np.linalg.norm(s['q'][:, 13+off:20+off] - qref[:, off:off+7], axis=1)
        perr = np.linalg.norm(wrist - pref, axis=1)
        endpoint = np.asarray(task['line']['end_pelvis'])
        live = np.einsum('nij,nj->ni', pelvis, wrist)
        live_end = s['standing'] @ endpoint
        signals = dict(standing_deg=np.degrees(s['drift']), onset_so3_deg=np.degrees(onset),
                       onset_gravity_deg=np.degrees(gravity), tilt_deg=np.degrees(s['tilt']),
                       tilt_deviation_deg=np.degrees(s['tilt']-s['tilt'][0]), gyro_rad_s=s['rate'],
                       gyro_torso_xy_rad_s=np.linalg.norm(np.array([r['snapshot']['imu_gyroscope'][:2] for r in rows]),axis=1),
                       gyro_torso_z_abs_rad_s=np.abs(np.array([r['snapshot']['imu_gyroscope'][2] for r in rows])),
                       timed_mm=perr*1000, q_rad=qerr,
                       endpoint_mm=np.linalg.norm(wrist-endpoint, axis=1)*1000,
                       live_endpoint_mm=np.linalg.norm(live-live_end, axis=1)*1000)
        official = a.flatten(a.reduce_run(trial.parent))
        official['operator_stopped'] = bool(su['interrupted'] and su['finish_reason'] != 'auto_stop')
        official['record_truncated_by_operator'] = official['operator_stopped']
        assert np.isclose(official['configuration_error_rms_rad'], np.sqrt(np.mean(qerr[t<=T]**2)))
        assert np.isclose(official['timed_position_error_rms_m'], np.sqrt(np.mean(perr[t<=T]**2)))
        source_checks = {}
        for name, info in prov['sources'].items():
            source = info.get('path')
            p = root / source.split('/h12_zmp_benchmark/', 1)[-1] if source else None
            digest = sha(p) if p and p.is_file() else None
            source_checks[name] = dict(saved=info.get('sha256'), current=digest,
                matches=(info.get('sha256') == digest) if digest else None)
        cmdtime = np.array([r['command_monotonic']-m['execution_start_monotonic'] for r in rows])
        arrival = np.array([r['snapshot']['arrival_monotonic']-m['execution_start_monotonic'] for r in rows])
        phase_masks = dict(move=t<=T, fade=(t>T)&(t<=T+.5), postfade=(t>T+.5)&(t<=T+2),
                           late=(t>T+2), tail=t>=t[-1]-1)
        phases = {}
        for phase, mask in phase_masks.items():
            tt = t[mask]
            phases[phase] = dict(n=int(mask.sum()), window_s=[float(tt[0]),float(tt[-1])],
                               max_gap_ms=float(np.diff(tt).max()*1000), metrics={})
            for key, values in signals.items():
                x = values[mask]
                peak_i = np.flatnonzero(mask)[np.argmax(np.abs(x))]
                phases[phase]['metrics'][key] = dict(mean=float(x.mean()),
                    rms_time=float(np.sqrt(a.time_weighted(tt, x, 2))),
                    peak_abs=float(abs(values[peak_i])), peak_s=float(t[peak_i]),
                    peak_after_T_s=float(t[peak_i]-T))
        motion = t<=T
        event_times = {e['event']: e['relative_s'] for e in next(x for x in inventory if x['run']==trial.parent.name)['events_relative_epoch']}
        ref0 = trajectory.sample(0., duration=T)
        firstref = trajectory.sample(t[0], duration=T)
        record = dict(run=trial.parent.name, case=trial.parent.name.split('_')[0],
            raw_id=str(trial.relative_to(root)), official=official,
            inventory=next(x for x in inventory if x['run']==trial.parent.name),
            phases=phases, initial=dict(standing_deg=float(np.degrees(s['drift'][0])),
              standing_capture_tilt_deg=float(np.degrees(np.arccos(np.clip(s['standing'][2,2],-1,1)))),
              tilt_deg=float(np.degrees(s['tilt'][0])), gyro_rad_s=float(s['rate'][0]),
              relative_roll_pitch_yaw_deg=np.degrees([s[k][0] for k in ('roll','pitch','yaw')]).tolist(),
              lean_xy=s['lean'][0].tolist(), stance_width_m=official['stance_width_m'],
              stance_stagger_m=official['stance_stagger_m'],
              com_ahead_of_midfoot_m=official['com_ahead_of_midfoot_m'],
              start_after_standing_s=-event_times.get('operator_confirmed_standing_translation_unknown', 0),
              start_after_preparation_s=-next((e['relative_s'] for e in next(x for x in inventory if x['run']==trial.parent.name)['events_relative_epoch']
                   if e['event']=='motion_finished' and e['relative_s']<0),0),
              first_reference_distance_mm=float(np.linalg.norm(firstref[3]-ref0[3])*1000)),
            provenance=dict(sources=source_checks, payload_sha256=task['npz_sha256'],
                measurement_clock=m.get('measurement_clock'), pipeline_revision=m.get('pipeline_revision'),
                resolved_controller=m['resolved_config']['controller'],
                input_sha256={p.name:sha(p) for p in trial.iterdir() if p.is_file()}),
            validation=dict(clock_max_error_s=float(np.max(np.abs(arrival-t))),
                desired_q_clock_max_error_rad=float(np.max(np.abs(
                    trajectory.sample(cmdtime, duration=T)[0][:,off:off+7]-s['desired_q'][:,off:off+7]))),
                online_timed_max_error_m=float(max(abs(r['model_metrics']['frozen_pelvis_fk']['timed_position_error_m']-perr[i])
                                                    for i,r in enumerate(rows))),
                invalid_state_count=sum(not r['state_valid'] for r in rows),
                invalid_tick_count=sum(not r['snapshot']['tick_valid'] for r in rows),
                zero_q_rows=int(np.sum(np.all(s['q']==0,axis=1))),
                control_sequence_diff=stats(np.diff([r['control_sequence'] for r in rows]))),
            timing=dict(move_observation_interval_ms=stats(np.diff(t[motion])*1000),
                move_command_interval_ms=stats(np.diff(cmdtime[motion])*1000),
                move_control_ms=stats([r['timing']['control_s']*1000 for r,ok in zip(rows,motion) if ok]),
                first_skipped_slots=rows[0].get('control_slots_skipped'),
                later_skipped_slots=sum(r.get('control_slots_skipped',0) for r in rows[1:]),
                unknown_publish_rows=sum(r['command'].get('last_published') is None for r in rows)))
        records.append(record)
        cache[record['run']] = (s, trajectory, signals)
        if not args.quiet:
            print('REDUCED', record['run'], flush=True)

    metric_keys = ('timed_position_error_rms_m', 'configuration_error_rms_rad', 'frozen_uniform_rms_m',
        'live_uniform_rms_m', 'frozen_endpoint_error_m', 'frozen_hold_error_rms_m', 'live_hold_error_rms_m',
        'move_rms_tilt_deviation_deg', 'motion_peak_drift_deg', 'peak_drift_deg', 'residual_drift_deg',
        'terminal_rate_rad_s', 'peak_rate_rad_s', 'frozen_hold_creep_m')
    pairs = []
    for case in cases:
        rr = {r['official']['controller']:r for r in records if r['case']==case}
        assert set(rr)=={'frame','counter'}
        f,c = rr['frame'],rr['counter']
        for key in ('policy','bank','target','speed','move_duration_s'):
            assert f['official'][key]==c['official'][key]
        T = f['official']['move_duration_s']
        sf, tr, yf = cache[f['run']]
        sc, _, yc = cache[c['run']]
        common_start=max(sf['time'][0],sc['time'][0])
        common_end=min(sf['time'][-1],sc['time'][-1])
        p=dict(case=case, target=f['official']['target'],
            order='F-C' if f['inventory']['created_utc']<c['inventory']['created_utc'] else 'C-F',
            strict_complete=bool(f['official']['run_complete'] and c['official']['run_complete']),
            metrics={k:dict(frame=f['official'][k],counter=c['official'][k],
                        delta=c['official'][k]-f['official'][k]) for k in metric_keys}, common_windows={})
        windows=dict(move=(common_start,T), onset_trim100ms=(max(.1,common_start),T),
                     fade=(T,T+.5), postfade=(T+.5,T+2),
                     end_hold=(T,common_end), clean_late=(T+7,T+9), common_tail=(common_end-1,common_end))
        for phase,(lo,hi) in windows.items():
            entry=dict(window_s=[float(lo),float(hi)], frame={}, counter={})
            for ctrl, s, y in (('frame',sf,yf),('counter',sc,yc)):
                t=s['time']
                # use identical integration support for both runs; never bridge >50 ms gaps
                other=sc['time'] if ctrl=='frame' else sf['time']
                knots=np.unique(np.concatenate(([lo,hi],t[(t>lo)&(t<hi)],other[(other>lo)&(other<hi)])))
                mid=(knots[:-1]+knots[1:])/2
                valid=np.ones(len(mid),dtype=bool)
                maxgap=0.
                for tt in (t,other):
                    index=np.searchsorted(tt,mid)-1
                    dt=tt[index+1]-tt[index]
                    valid &= dt<=.05
                    maxgap=max(maxgap,float(dt.max()))
                widths=np.diff(knots)
                entry['retained_duration_s']=float(widths[valid].sum())
                entry['excluded_gap_duration_s']=float(widths[~valid].sum())
                entry['max_support_gap_ms']=maxgap*1000
                for key, values in y.items():
                    x=np.interp(knots,t,values)
                    # piecewise linear signal RMS integral, exact on each retained interval
                    integral=widths*(x[:-1]**2+x[:-1]*x[1:]+x[1:]**2)/3
                    entry[ctrl][key+'_rms']=float(np.sqrt(integral[valid].sum()/widths[valid].sum()))
                    entry[ctrl][key+'_mean']=float((widths*(x[:-1]+x[1:])/2)[valid].sum()/widths[valid].sum())
                    valid_knots=np.concatenate((valid,[False])) | np.concatenate(([False],valid))
                    entry[ctrl][key+'_peak_abs']=float(np.max(np.abs(x[valid_knots])))
                # timed sensitivity samples measured vectors then the exact reference, not interpolated errors
                grid=np.linspace(lo,hi,1001)
                if not np.all(valid):
                    entry[ctrl]['exact_reference_grid']=None
                else:
                    off=0 if tr.moving_arm=='left' else 7
                    q=np.stack([np.interp(grid,t,s['q'][:,i]) for i in range(13+off,20+off)],axis=1)
                    wrist=np.stack([np.interp(grid,t,s[tr.moving_arm+'_wrist'][:,i]) for i in range(3)],axis=1)
                    qr,_,_,pr,_=tr.sample(grid,duration=T)
                    endpoint=np.asarray(tr.metadata['line']['end_pelvis'])
                    start=np.asarray(tr.metadata['line']['start_pelvis'])
                    uniform=a.uniform_resample(wrist)
                    live=np.einsum('nij,nj->ni',s['pelvis'],s[tr.moving_arm+'_wrist'])
                    live_grid=np.stack([np.interp(grid,t,live[:,i]) for i in range(3)],axis=1)
                    live_uniform=a.uniform_resample(live_grid)
                    live_start,live_end=s['standing']@start,s['standing']@endpoint
                    entry[ctrl]['exact_reference_grid']=dict(
                        timed_mm=float(np.sqrt(a.time_weighted(grid,np.linalg.norm(wrist-pr,axis=1),2))*1000),
                        q_rad=float(np.sqrt(a.time_weighted(grid,np.linalg.norm(q-qr[:,off:off+7],axis=1),2))),
                        frozen_uniform_mm=float(np.sqrt(np.mean(a.segment_distance(uniform,start,endpoint)[0]**2))*1000),
                        live_uniform_mm=float(np.sqrt(np.mean(a.segment_distance(live_uniform,live_start,live_end)[0]**2))*1000),
                        endpoint_at_window_end_mm=float(np.linalg.norm(wrist[-1]-endpoint)*1000),
                        live_endpoint_at_window_end_mm=float(np.linalg.norm(live_grid[-1]-live_end)*1000))
            entry['deltas']={k:entry['counter'][k]-entry['frame'][k] for k in entry['frame'] if k!='exact_reference_grid'}
            p['common_windows'][phase]=entry
        pairs.append(p)

    old_path=root/'submodules/h12_ros2_controller/docs/counter_balance/data/iteration7_20261106/results.json'
    old=load(old_path)
    history=dict(source=str(old_path.relative_to(root)),sha256=sha(old_path),campaign=old['campaign'],
                 matched_pairs=[p for p in old['pairs'] if p['pair'] in ('B1','B3','R3')],
                 matched_records=[r for r in old['records'] if r['case'] in ('B1','B3','R3')],
                 rules='Nov6 only FAME same cells: F01 vs B1, F02 vs B3 and R3 separately. No ALMI-to-FAME comparison or historical pooling.')
    history['comparability']=[]
    for old_case,new_case in (('B1','F01'),('B3','F02'),('R3','F02')):
        for ctrl in ('frame','counter'):
            before=next(r for r in history['matched_records'] if r['case']==old_case and r['controller']==ctrl)
            after=next(r for r in records if r['run']==new_case+'_'+ctrl)
            old_raw=root/before['raw_id']/'arm_target.jsonl'
            assert sha(old_raw)==before['provenance']['raw_sha256']
            assert before['target']==after['official']['target'] and before['policy']==after['official']['policy']
            assert before['move_duration_s']==after['official']['move_duration_s']
            assert before['provenance']['payload_sha256']==after['provenance']['payload_sha256']
            history['comparability'].append(dict(old_case=old_case,new_case=new_case,controller=ctrl,
                same_trajectory_payload=True,same_duration=True,
                same_resolved_config=before['provenance']['resolved_config_sha256']==after['inventory']['resolved_config_sha256'],
                old_git=before['provenance']['git'],new_git=after['inventory']['git'],
                old_raw_sha256=sha(old_raw),new_raw_sha256=after['inventory']['raw_sha256'],
                checkpoint_identity_unverified=True))
    result=dict(campaign=args.campaign, records=records, pairs=pairs, chronological_runs=[r['run'] for r in inventory],
                old_nov6_separate=history, analysis=dict(script_sha256='e1dffe7d4a55972a153bb89dc3e43e72be992bced6e880b0521dda96e660c6e4',
                    reducer_sha256=sha(Path(a.__file__)), numpy_version=np.__version__,
                    policy='FAME intentional zero-input fixed; ALMI pending new checkpoint; no policy-specific exploitation',
                    definitions='Official metrics retain historical definitions. Phase RMS is observed-window trapezoid. Common-window scalar RMS integrates piecewise linear signals exactly; omit >50ms support intervals for BOTH members. Exact-reference grid samples measured q/FK at 1001 times and saved trajectory at actual executed duration. No pre-onset extrapolation. Frozen/live FK omit unmeasured base translation.',
                    phase_windows='move observed onset..T; fade T..T+.5; postfade T+.5..T+2; late T+2..end; tail last1s'))
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    if not args.quiet:
        for p in pairs:
            print('PAIR',p['case'],p['order'],json.dumps(p['metrics']),flush=True)
    print('WROTE',args.output)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path.cwd())
    parser.add_argument('--campaign',default='runs/real_experiments/iter7b_fame_validation_20261110_184722')
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().with_name('results.json'))
    parser.add_argument('--inventory-only',action='store_true')
    parser.add_argument('--quiet',action='store_true')
    main(parser.parse_args())
