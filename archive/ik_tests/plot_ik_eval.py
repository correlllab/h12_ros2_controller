'''
Plot a run directory produced by ik_eval.py.

For each trial.npz it writes:
  <run>/plots/<trial_stem>/<joint>.png     q + tau, with URDF and clip limits dashed
  <run>/plots/<trial_stem>/cartesian.png   lin/ang frame error vs time

Aggregates:
  <run>/plots/repeatability/<case>.png     steady-state EE scatter across trials
  <run>/plots/sweep_summary.png            steady-state lin/ang error vs sweep combo
  <run>/plots/fk_roundtrip.png             offline IK residuals histogram (if present)
  <run>/plots/deep_analysis/figN_*.png     deep characterization (analyze_deep.py)

Usage:
  python -m h12_ros2_controller.tests.plot_ik_eval <run_dir>
  # default = most recent run dir under runs/
'''
import argparse
import csv
import os
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]

# ---- styling constants ------------------------------------------------------

URDF_LIMIT_STYLE = dict(color='tab:red', linestyle='--', linewidth=1.2, alpha=0.6,
                        label='URDF limit')
CLIP_LIMIT_STYLE = dict(color='tab:orange', linestyle=':', linewidth=1.4, alpha=0.9,
                        label='clip (software) limit')


def _latest_run_dir() -> Path:
    root = REPO_ROOT / 'runs'
    if not root.exists():
        raise FileNotFoundError(f'{root} does not exist')
    candidates = sorted([p for p in root.iterdir() if p.is_dir()])
    if not candidates:
        raise FileNotFoundError('no run directories found under runs/')
    return candidates[-1]


def _steady_stats(arr: np.ndarray, frac: float = 0.1) -> np.ndarray:
    '''Mean over the last `frac` of timeseries along axis 0.'''
    if arr.ndim == 1:
        n = arr.shape[0]
        tail = max(1, int(n * frac))
        return arr[-tail:].mean()
    n = arr.shape[0]
    tail = max(1, int(n * frac))
    return arr[-tail:].mean(axis=0)


# ---- per-trial plots --------------------------------------------------------


def _plot_joint_panel(joint_name: str, body_idx: int, data: dict, out_path: Path):
    t = data['t'].flatten() if data['t'].ndim > 1 else data['t']
    q_actual = data['q_actual'][:, body_idx]
    q_cmd = data['q_cmd'][:, body_idx]
    tau_actual = data['tau_actual'][:, body_idx]
    tau_cmd = data['tau_cmd'][:, body_idx]
    dq_actual = data['dq_actual'][:, body_idx]

    urdf_q_low = float(data['urdf_q_low'][body_idx])
    urdf_q_high = float(data['urdf_q_high'][body_idx])
    urdf_tau = float(data['urdf_tau_limit'][body_idx])
    urdf_dq = float(data['urdf_dq_limit'][body_idx])
    clip_q_low = float(data['clip_q_low'][body_idx])
    clip_q_high = float(data['clip_q_high'][body_idx])
    clip_tau = float(data['clip_tau_limit'][body_idx])
    clip_dq = float(data['clip_dq_limit'][body_idx])

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    ax_q, ax_dq, ax_tau = axes

    # --- position
    ax_q.plot(t, q_actual, label='q_actual', linewidth=1.8)
    ax_q.plot(t, q_cmd, linestyle='--', linewidth=1.6, label='q_cmd')
    # IK optimal (if present) as a horizontal reference line
    q_opt_full = data.get('q_ik_optimal')
    if q_opt_full is not None and np.isfinite(q_opt_full[body_idx]):
        ax_q.axhline(q_opt_full[body_idx], color='tab:green',
                     linestyle='-.', linewidth=1.2, alpha=0.8,
                     label='q_ik_optimal (ref)')
    ax_q.axhline(urdf_q_low, **URDF_LIMIT_STYLE)
    ax_q.axhline(urdf_q_high, color=URDF_LIMIT_STYLE['color'],
                 linestyle=URDF_LIMIT_STYLE['linestyle'],
                 linewidth=URDF_LIMIT_STYLE['linewidth'],
                 alpha=URDF_LIMIT_STYLE['alpha'])
    ax_q.axhline(clip_q_low, **CLIP_LIMIT_STYLE)
    ax_q.axhline(clip_q_high, color=CLIP_LIMIT_STYLE['color'],
                 linestyle=CLIP_LIMIT_STYLE['linestyle'],
                 linewidth=CLIP_LIMIT_STYLE['linewidth'],
                 alpha=CLIP_LIMIT_STYLE['alpha'])
    ax_q.set_ylabel('position (rad)')
    ax_q.set_title(f'{joint_name}  (body_idx={body_idx})')
    ax_q.grid(True, alpha=0.3)
    ax_q.legend(loc='best', fontsize=8)

    # --- velocity
    ax_dq.plot(t, dq_actual, label='dq_actual', linewidth=1.6)
    ax_dq.axhline(urdf_dq, **URDF_LIMIT_STYLE)
    ax_dq.axhline(-urdf_dq, color=URDF_LIMIT_STYLE['color'],
                  linestyle=URDF_LIMIT_STYLE['linestyle'],
                  linewidth=URDF_LIMIT_STYLE['linewidth'],
                  alpha=URDF_LIMIT_STYLE['alpha'])
    ax_dq.axhline(clip_dq, **CLIP_LIMIT_STYLE)
    ax_dq.axhline(-clip_dq, color=CLIP_LIMIT_STYLE['color'],
                  linestyle=CLIP_LIMIT_STYLE['linestyle'],
                  linewidth=CLIP_LIMIT_STYLE['linewidth'],
                  alpha=CLIP_LIMIT_STYLE['alpha'])
    ax_dq.set_ylabel('velocity (rad/s)')
    ax_dq.grid(True, alpha=0.3)
    ax_dq.legend(loc='best', fontsize=8)

    # --- torque
    ax_tau.plot(t, tau_actual, label='tau_actual', linewidth=1.6)
    ax_tau.plot(t, tau_cmd, linestyle='--', linewidth=1.4, label='tau_cmd (grav. ff)')
    ax_tau.axhline(urdf_tau, **URDF_LIMIT_STYLE)
    ax_tau.axhline(-urdf_tau, color=URDF_LIMIT_STYLE['color'],
                   linestyle=URDF_LIMIT_STYLE['linestyle'],
                   linewidth=URDF_LIMIT_STYLE['linewidth'],
                   alpha=URDF_LIMIT_STYLE['alpha'])
    ax_tau.axhline(clip_tau, **CLIP_LIMIT_STYLE)
    ax_tau.axhline(-clip_tau, color=CLIP_LIMIT_STYLE['color'],
                   linestyle=CLIP_LIMIT_STYLE['linestyle'],
                   linewidth=CLIP_LIMIT_STYLE['linewidth'],
                   alpha=CLIP_LIMIT_STYLE['alpha'])
    ax_tau.set_ylabel('torque (N m)')
    ax_tau.set_xlabel('time (s)')
    ax_tau.grid(True, alpha=0.3)
    ax_tau.legend(loc='best', fontsize=8)

    plt.tight_layout()
    plt.savefig(out_path, dpi=110)
    plt.close(fig)


def _plot_cartesian(data: dict, out_path: Path):
    t = data['t'].flatten() if data['t'].ndim > 1 else data['t']
    err = data['frame_err']
    lin = np.linalg.norm(err[:, :3], axis=1)
    ang = np.linalg.norm(err[:, 3:], axis=1)

    fig, (ax_lin, ax_ang) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    ax_lin.plot(t, lin, linewidth=1.8)
    ax_lin.set_ylabel('linear err (m)')
    ax_lin.grid(True, alpha=0.3)
    ax_lin.set_title(f'{str(data["case"])}  trial={int(data["trial_index"])}  '
                     f'frame={str(data["frame"])}  converged={bool(data["converged"])}')
    ax_ang.plot(t, ang, linewidth=1.8, color='tab:purple')
    ax_ang.set_ylabel('angular err (rad)')
    ax_ang.set_xlabel('time (s)')
    ax_ang.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_path, dpi=110)
    plt.close(fig)


def _plot_trial(trial_path: Path, out_dir: Path):
    with np.load(trial_path, allow_pickle=False) as bundle:
        data = {k: bundle[k] for k in bundle.files}

    reduced_body_ids = data['reduced_body_ids']
    reduced_names = data['reduced_joint_names']

    trial_out = out_dir / trial_path.stem
    trial_out.mkdir(parents=True, exist_ok=True)

    for i, body_idx in enumerate(reduced_body_ids):
        joint_name = str(reduced_names[i])
        _plot_joint_panel(joint_name, int(body_idx), data,
                          trial_out / f'{joint_name}.png')
    _plot_cartesian(data, trial_out / 'cartesian.png')


# ---- aggregate plots --------------------------------------------------------


def _plot_repeatability(run_dir: Path, out_dir: Path):
    '''For each (case, sweep) group, scatter final EE position across trials.'''
    trials_dir = run_dir / 'trials'
    groups: dict[tuple, list[dict]] = defaultdict(list)
    for trial_path in sorted(trials_dir.glob('*.npz')):
        with np.load(trial_path, allow_pickle=False) as bundle:
            case = str(bundle['case'])
            sweep_keys = bundle['sweep_keys']
            sweep_vals = bundle['sweep_values']
            sweep_tag = '_'.join(
                f'{k}={v}' for k, v in zip(sweep_keys, sweep_vals)
            ) or 'default'
            ee = bundle['ee_actual']
            target = bundle['target_pose']
            groups[(case, sweep_tag)].append({
                'ee_final': _steady_stats(ee),
                'target': target,
                'trial_index': int(bundle['trial_index']),
            })

    rep_dir = out_dir / 'repeatability'
    rep_dir.mkdir(parents=True, exist_ok=True)
    for (case, sweep_tag), entries in groups.items():
        if len(entries) < 2:
            continue
        ee_finals = np.stack([e['ee_final'] for e in entries])
        target = entries[0]['target']
        fig, axes = plt.subplots(1, 3, figsize=(12, 4))
        axes[0].scatter(ee_finals[:, 0], ee_finals[:, 1], c='tab:blue')
        axes[0].scatter([target[0]], [target[1]], marker='x', c='tab:red',
                        s=80, label='target')
        axes[0].set_xlabel('x (m)'); axes[0].set_ylabel('y (m)')
        axes[0].set_title('XY')
        axes[0].legend(); axes[0].grid(True, alpha=0.3)

        axes[1].scatter(ee_finals[:, 0], ee_finals[:, 2], c='tab:blue')
        axes[1].scatter([target[0]], [target[2]], marker='x', c='tab:red', s=80)
        axes[1].set_xlabel('x (m)'); axes[1].set_ylabel('z (m)')
        axes[1].set_title('XZ')
        axes[1].grid(True, alpha=0.3)

        axes[2].scatter(ee_finals[:, 1], ee_finals[:, 2], c='tab:blue')
        axes[2].scatter([target[1]], [target[2]], marker='x', c='tab:red', s=80)
        axes[2].set_xlabel('y (m)'); axes[2].set_ylabel('z (m)')
        axes[2].set_title('YZ')
        axes[2].grid(True, alpha=0.3)

        std_pos = ee_finals[:, :3].std(axis=0)
        fig.suptitle(
            f'{case}  sweep={sweep_tag}  n={len(entries)}  '
            f'std(x,y,z) = ({std_pos[0]:.4f}, {std_pos[1]:.4f}, {std_pos[2]:.4f}) m'
        )
        plt.tight_layout()
        plt.savefig(rep_dir / f'{case}__{sweep_tag}.png', dpi=110)
        plt.close(fig)


def _plot_sweep_summary(run_dir: Path, out_dir: Path):
    '''Bar plot of steady-state error grouped by sweep-tag across all cases.'''
    summary_path = run_dir / 'summary.csv'
    if not summary_path.exists():
        return
    rows = []
    with summary_path.open('r', encoding='utf-8') as fh:
        for row in csv.DictReader(fh):
            rows.append(row)
    if not rows:
        return

    # build sweep label per row
    def row_sweep(r):
        items = [(k[len('sweep__'):], v) for k, v in r.items() if k.startswith('sweep__')]
        return '_'.join(f'{k}={v}' for k, v in items) or 'default'

    grouped: dict[str, list[float]] = defaultdict(list)
    grouped_ang: dict[str, list[float]] = defaultdict(list)
    for r in rows:
        tag = row_sweep(r)
        try:
            grouped[tag].append(float(r['lin_err_steady_m']))
            grouped_ang[tag].append(float(r['ang_err_steady_rad']))
        except (KeyError, ValueError):
            continue

    tags = list(grouped.keys())
    if not tags:
        return
    lin_mean = np.array([np.mean(grouped[t]) for t in tags])
    lin_std = np.array([np.std(grouped[t]) for t in tags])
    ang_mean = np.array([np.mean(grouped_ang[t]) for t in tags])
    ang_std = np.array([np.std(grouped_ang[t]) for t in tags])

    x = np.arange(len(tags))
    fig, (ax_lin, ax_ang) = plt.subplots(2, 1, figsize=(max(8, len(tags) * 0.6), 7))
    ax_lin.bar(x, lin_mean, yerr=lin_std, color='tab:blue')
    ax_lin.set_ylabel('steady-state lin err (m)')
    ax_lin.set_xticks(x); ax_lin.set_xticklabels(tags, rotation=60, ha='right', fontsize=8)
    ax_lin.grid(True, alpha=0.3)
    ax_lin.set_title('sweep summary (mean ± std across all cases & trials)')

    ax_ang.bar(x, ang_mean, yerr=ang_std, color='tab:purple')
    ax_ang.set_ylabel('steady-state ang err (rad)')
    ax_ang.set_xticks(x); ax_ang.set_xticklabels(tags, rotation=60, ha='right', fontsize=8)
    ax_ang.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_dir / 'sweep_summary.png', dpi=110)
    plt.close(fig)


def _plot_fk_roundtrip(run_dir: Path, out_dir: Path):
    path = run_dir / 'fk_roundtrip.npz'
    if not path.exists():
        return
    with np.load(path, allow_pickle=False) as bundle:
        pos = bundle['pos_residual']
        ang = bundle['ang_residual']
        succ = bundle['success']
        frame = str(bundle['frame'])
    fig, (ax_pos, ax_ang) = plt.subplots(1, 2, figsize=(11, 4))
    ax_pos.hist(pos, bins=40, color='tab:blue', alpha=0.8)
    ax_pos.set_xlabel('pos residual (m)'); ax_pos.set_ylabel('count')
    ax_pos.set_title(
        f'FK->IK pos residual  (frame={frame}, n={len(pos)}, '
        f'success={100*succ.mean():.1f}%)'
    )
    ax_pos.grid(True, alpha=0.3)
    ax_ang.hist(ang, bins=40, color='tab:purple', alpha=0.8)
    ax_ang.set_xlabel('ang residual (rad)'); ax_ang.set_ylabel('count')
    ax_ang.set_title('FK->IK ang residual')
    ax_ang.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_dir / 'fk_roundtrip.png', dpi=110)
    plt.close(fig)


# ---- entry point ------------------------------------------------------------


def _plot_deep_analysis(run_dir: Path):
    '''Run the deep-analysis figure set (analyze_deep.run_deep_analysis).

    Imported lazily because it pulls in pandas, which we don't want to require
    for the basic per-trial plots.
    '''
    try:
        from archive.tests.analyze_deep import run_deep_analysis
    except ImportError:
        try:
            from .analyze_deep import run_deep_analysis  # type: ignore
        except ImportError as e:
            print(f'[plot] skipping deep analysis (import failed: {e})')
            return
    try:
        run_deep_analysis(run_dir)
    except Exception as e:
        print(f'[plot] deep analysis failed: {e}')


def plot_run(run_dir: Path):
    out_dir = run_dir / 'plots'
    out_dir.mkdir(parents=True, exist_ok=True)
    _plot_deep_analysis(run_dir)
    trials_dir = run_dir / 'trials'
    trial_files = sorted(trials_dir.glob('*.npz'))
    if not trial_files:
        print(f'[plot] no trial .npz files under {trials_dir}')
    for trial_path in trial_files:
        print(f'[plot] trial: {trial_path.name}', flush=True)
        _plot_trial(trial_path, out_dir)
    _plot_repeatability(run_dir, out_dir)
    _plot_sweep_summary(run_dir, out_dir)
    _plot_fk_roundtrip(run_dir, out_dir)
    print(f'[plot] done -> {out_dir}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', nargs='?', default=None,
                        help='run directory (default: latest under runs/)')
    args = parser.parse_args()
    run_dir = Path(args.run_dir).resolve() if args.run_dir else _latest_run_dir()
    if not run_dir.exists():
        print(f'[plot] missing: {run_dir}', file=sys.stderr)
        sys.exit(1)
    plot_run(run_dir)


if __name__ == '__main__':
    main()
