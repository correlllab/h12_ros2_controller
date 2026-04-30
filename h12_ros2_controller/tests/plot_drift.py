'''
Encoder drift analysis: concatenate every trial in a run into a single
wall-clock timeline and plot per-joint position + torque.

Useful for diagnosing encoder drift when the robot is near-static — any
slow monotonic creep in q_actual (or growing torque offset) is immediately
visible across the full run.

Usage:
  python -m h12_ros2_controller.tests.plot_drift [run_dir]
  # default = most recent run dir under runs/
'''
import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]

URDF_STYLE = dict(color='tab:red',    linestyle='--', linewidth=1.0, alpha=0.55)
CLIP_STYLE = dict(color='tab:orange', linestyle=':',  linewidth=1.2, alpha=0.85)
DRIFT_WARN_FRAC = 0.85   # flag joints whose q_actual reaches this fraction of a clip limit


def _latest_run_dir() -> Path:
    root = REPO_ROOT / 'runs'
    if not root.exists():
        raise FileNotFoundError(root)
    candidates = sorted(p for p in root.iterdir() if p.is_dir())
    if not candidates:
        raise FileNotFoundError('no run directories under runs/')
    return candidates[-1]


def _load_run(run_dir: Path):
    '''Return sorted trial paths and assembled timeline arrays.

    Returns
    -------
    joint_names : list[str]                   length J
    body_ids    : np.ndarray shape (J,)
    wall_t      : np.ndarray shape (T,)       seconds, monotonically increasing
    q_actual    : np.ndarray shape (T, 27)
    tau_actual  : np.ndarray shape (T, 27)
    q_cmd       : np.ndarray shape (T, 27)
    tau_cmd     : np.ndarray shape (T, 27)
    trial_boundaries : list[float]            wall_t values at each trial start
    limits      : dict with keys urdf_q_low/high, urdf_tau_limit,
                                 clip_q_low/high, clip_tau_limit  (shape 27)
    '''
    trial_paths = sorted((run_dir / 'trials').glob('*.npz'))
    if not trial_paths:
        raise FileNotFoundError(f'no trial .npz files under {run_dir}/trials/')

    chunks_t, chunks_q, chunks_tau, chunks_qcmd, chunks_taucmd = [], [], [], [], []
    wall_offset = 0.0
    trial_boundaries = []
    joint_names = None
    body_ids = None
    limits = None

    for tp in trial_paths:
        with np.load(tp, allow_pickle=False) as f:
            if joint_names is None:
                joint_names = [str(s) for s in f['reduced_joint_names']]
                body_ids    = f['reduced_body_ids'].copy()
                limits = {k: f[k].copy() for k in
                          ('urdf_q_low', 'urdf_q_high', 'urdf_tau_limit',
                           'clip_q_low', 'clip_q_high', 'clip_tau_limit')}

            t_raw = f['t'].flatten()
            t_shifted = t_raw - t_raw[0] + wall_offset
            trial_boundaries.append(float(t_shifted[0]))
            wall_offset = float(t_shifted[-1]) + float(f['dt'])

            chunks_t.append(t_shifted)
            chunks_q.append(f['q_actual'].copy())
            chunks_tau.append(f['tau_actual'].copy())
            chunks_qcmd.append(f['q_cmd'].copy())
            chunks_taucmd.append(f['tau_cmd'].copy())

    wall_t   = np.concatenate(chunks_t)
    q_actual = np.concatenate(chunks_q,      axis=0)
    tau_actual = np.concatenate(chunks_tau,  axis=0)
    q_cmd    = np.concatenate(chunks_qcmd,   axis=0)
    tau_cmd  = np.concatenate(chunks_taucmd, axis=0)

    return (joint_names, body_ids, wall_t,
            q_actual, tau_actual, q_cmd, tau_cmd,
            trial_boundaries, limits)


def _drift_summary(joint_names, body_ids, wall_t, q_actual, limits):
    '''Print a quick text summary of per-joint drift statistics.'''
    print('\n--- drift summary (across full run) ---')
    print(f'  total wall time : {wall_t[-1]:.1f} s   ({len(wall_t)} samples)')
    print(f'  {"joint":<36} {"q_start":>9} {"q_end":>9} {"delta":>9} '
          f'{"clip_lo":>9} {"clip_hi":>9}  warn')
    for i, (name, bid) in enumerate(zip(joint_names, body_ids)):
        q0   = q_actual[0,  bid]
        qf   = q_actual[-1, bid]
        d    = qf - q0
        clo  = limits['clip_q_low'][bid]
        chi  = limits['clip_q_high'][bid]
        # warn if q_end is within DRIFT_WARN_FRAC of either clip limit
        near_lo = qf <= clo + DRIFT_WARN_FRAC * (chi - clo) * 0 + abs(clo) * (1 - DRIFT_WARN_FRAC)
        near_hi = qf >= chi - abs(chi) * (1 - DRIFT_WARN_FRAC)
        near_lo = qf <= clo + abs(clo - chi) * (1 - DRIFT_WARN_FRAC)
        near_hi = qf >= chi - abs(clo - chi) * (1 - DRIFT_WARN_FRAC)
        flag = ' <<< NEAR LIMIT' if (near_lo or near_hi) else ''
        print(f'  {name:<36} {q0:+9.4f} {qf:+9.4f} {d:+9.4f} '
              f'{clo:+9.4f} {chi:+9.4f}{flag}')
    print()


def _plot_joint(ax_q, ax_tau, name, bid, wall_t, q_actual, tau_actual,
                q_cmd, tau_cmd, trial_boundaries, limits):
    # position
    ax_q.plot(wall_t, q_actual[:, bid],  color='tab:blue',   lw=1.4, label='q_actual')
    ax_q.plot(wall_t, q_cmd[:, bid],     color='tab:blue',   lw=1.0,
              linestyle='--', alpha=0.5, label='q_cmd')
    ax_q.axhline(limits['urdf_q_low'][bid],  **URDF_STYLE, label='URDF lim')
    ax_q.axhline(limits['urdf_q_high'][bid], **URDF_STYLE)
    ax_q.axhline(limits['clip_q_low'][bid],  **CLIP_STYLE, label='clip lim')
    ax_q.axhline(limits['clip_q_high'][bid], **CLIP_STYLE)
    for tb in trial_boundaries:
        ax_q.axvline(tb, color='gray', lw=0.4, alpha=0.4)
    ax_q.set_ylabel('q (rad)', fontsize=8)
    ax_q.set_title(name, fontsize=9, fontweight='bold')
    ax_q.legend(fontsize=7, loc='upper right')
    ax_q.grid(True, alpha=0.25)

    # torque
    ax_tau.plot(wall_t, tau_actual[:, bid], color='tab:red',  lw=1.4, label='tau_actual')
    ax_tau.plot(wall_t, tau_cmd[:, bid],    color='tab:red',  lw=1.0,
                linestyle='--', alpha=0.5, label='tau_cmd')
    ax_tau.axhline( limits['clip_tau_limit'][bid], **CLIP_STYLE, label='clip tau lim')
    ax_tau.axhline(-limits['clip_tau_limit'][bid], **CLIP_STYLE)
    for tb in trial_boundaries:
        ax_tau.axvline(tb, color='gray', lw=0.4, alpha=0.4)
    ax_tau.set_ylabel('tau (Nm)', fontsize=8)
    ax_tau.set_xlabel('wall time (s)', fontsize=8)
    ax_tau.legend(fontsize=7, loc='upper right')
    ax_tau.grid(True, alpha=0.25)


def plot_drift(run_dir: Path):
    out_dir = run_dir / 'plots' / 'drift'
    out_dir.mkdir(parents=True, exist_ok=True)

    (joint_names, body_ids, wall_t,
     q_actual, tau_actual, q_cmd, tau_cmd,
     trial_boundaries, limits) = _load_run(run_dir)

    _drift_summary(joint_names, body_ids, wall_t, q_actual, limits)

    J = len(joint_names)
    # one figure per joint — easier to open individually
    for i, (name, bid) in enumerate(zip(joint_names, body_ids)):
        fig, (ax_q, ax_tau) = plt.subplots(2, 1, figsize=(14, 6), sharex=True)
        _plot_joint(ax_q, ax_tau, name, int(bid),
                    wall_t, q_actual, tau_actual, q_cmd, tau_cmd,
                    trial_boundaries, limits)
        fig.suptitle(
            f'Drift analysis — {name}  (body_idx={bid})  '
            f'run: {run_dir.name}',
            fontsize=10
        )
        plt.tight_layout()
        out_path = out_dir / f'{name}.png'
        plt.savefig(out_path, dpi=120)
        plt.close(fig)
        print(f'[drift] wrote {out_path}')

    # combined overview: all joints on one tall figure
    ncols = 2
    nrows = J
    fig, axes = plt.subplots(nrows, ncols * 2, figsize=(22, 3.5 * J))
    for i, (name, bid) in enumerate(zip(joint_names, body_ids)):
        ax_q   = axes[i, 0] if ncols == 1 else axes[i, 0]
        ax_tau = axes[i, 1] if ncols == 1 else axes[i, 1]
        _plot_joint(ax_q, ax_tau, name, int(bid),
                    wall_t, q_actual, tau_actual, q_cmd, tau_cmd,
                    trial_boundaries, limits)
    # hide unused axes
    for row in axes:
        for j in range(2, ncols * 2):
            row[j].set_visible(False)

    fig.suptitle(f'Drift overview — all joints — {run_dir.name}', fontsize=11)
    plt.tight_layout()
    overview_path = out_dir / '_overview.png'
    plt.savefig(overview_path, dpi=100)
    plt.close(fig)
    print(f'[drift] overview -> {overview_path}')
    print(f'[drift] done, all plots in {out_dir}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', nargs='?', default=None)
    args = parser.parse_args()
    run_dir = Path(args.run_dir).resolve() if args.run_dir else _latest_run_dir()
    if not run_dir.exists():
        print(f'[drift] missing: {run_dir}', file=sys.stderr)
        sys.exit(1)
    plot_drift(run_dir)


if __name__ == '__main__':
    main()
