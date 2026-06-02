'''
Deep characterization analysis for an ik_eval run directory.

Generates figures covering: absolute error, systematic bias, repeatability,
bimodal IK solutions, orientation instability, and spatial error structure.

Required case-name format:
    {arm}_{...descriptive tokens...}_x{XX}_y{YY}_z{ZZ}

The trailing three tokens encode the target xyz in centimetres (the parser
uses negative indexing so any number of descriptive tokens in the middle is
fine). Example: ``left_pick_x30_y20_z06`` or ``left_forward_extend_x35_y20_z10``.

Usage as a module:
    from h12_ros2_controller.tests.analyze_deep import run_deep_analysis
    run_deep_analysis(Path('runs/20260506_154822_sweep_tilt_real'))

Usage as a script:
    python -m h12_ros2_controller.tests.analyze_deep <run_dir>
'''
import argparse
import itertools
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.lines import Line2D


PALETTE = {'left': '#2196F3', 'right': '#F44336'}
LIGHT = {'left': '#BBDEFB', 'right': '#FFCDD2'}
TRIAL_MARKERS = ['o', 's', '^', 'D', 'v', 'P']
TRIAL_COLORS = ['#1565C0', '#C62828', '#2E7D32', '#6A1B9A', '#EF6C00', '#00838F']


def _parse_case(name: str):
    '''Parse {arm}_..._x{XX}_y{YY}_z{ZZ} → (arm, x, y, z) in metres.

    Uses negative indexing so descriptive tokens of any length are allowed
    between the arm and the trailing x/y/z triple.
    '''
    parts = name.split('_')
    if len(parts) < 4:
        raise ValueError(f'case name {name!r} has fewer than 4 underscore-separated tokens')
    arm = parts[0]
    xt, yt, zt = parts[-3], parts[-2], parts[-1]
    if not (xt.startswith('x') and yt.startswith('y') and zt.startswith('z')):
        raise ValueError(
            f'case name {name!r} does not end in x..y..z.. tokens '
            f'(got {xt!r}, {yt!r}, {zt!r})')
    return arm, int(xt[1:]) / 100, int(yt[1:]) / 100, int(zt[1:]) / 100


def _find_trial(trials_dir: Path, case: str, trial: int):
    '''Find a trial npz for (case, trial) regardless of sweep-param suffix.'''
    matches = sorted(trials_dir.glob(f'{case}__*__t{trial}.npz'))
    if not matches:
        return None
    return np.load(matches[0])


def run_deep_analysis(run_dir: Path) -> Path | None:
    '''Generate the deep-analysis figure set for a single run directory.

    Returns the figure output directory, or None if the run can't be analysed
    (missing summary.csv, unparsable case names, etc.).
    '''
    run_dir = Path(run_dir)
    trials_dir = run_dir / 'trials'
    plots_dir = run_dir / 'plots'
    plots_dir.mkdir(parents=True, exist_ok=True)
    figdir = plots_dir / 'deep_analysis'
    figdir.mkdir(parents=True, exist_ok=True)

    summary_path = run_dir / 'summary.csv'
    if not summary_path.exists():
        print(f'[deep] no summary.csv at {summary_path}, skipping')
        return None

    df = pd.read_csv(summary_path)
    if df.empty:
        print(f'[deep] summary.csv at {summary_path} is empty, skipping')
        return None

    try:
        df[['arm', 'tx', 'ty', 'tz']] = pd.DataFrame(
            [_parse_case(c) for c in df['case']], index=df.index)
    except ValueError as e:
        print(f'[deep] cannot parse case names (need ..._x##_y##_z##): {e}')
        return None

    # World-frame signed errors. Left target y = +ty, right target y = -ty.
    df['target_y_world'] = np.where(df['arm'] == 'left', df['ty'], -df['ty'])
    df['ex'] = df['ee_ss__x'] - df['tx']
    df['ey'] = df['ee_ss__y'] - df['target_y_world']
    df['ez'] = df['ee_ss__z'] - df['tz']
    df['pos_err_3d'] = np.sqrt(df['ex'] ** 2 + df['ey'] ** 2 + df['ez'] ** 2)
    df['ey_abs'] = df['ee_ss__y'].abs() - df['ty']

    grp = df.groupby(['case', 'arm', 'tx', 'ty', 'tz'])
    case_stats = grp.agg(
        pos_err_mean=('pos_err_3d', 'mean'),
        pos_err_std=('pos_err_3d', 'std'),
        ik_resid_mean=('lin_err_steady_m', 'mean'),
        ang_err_mean=('ang_err_steady_rad', 'mean'),
        ang_err_std=('ang_err_steady_rad', 'std'),
        ex_mean=('ex', 'mean'), ex_std=('ex', 'std'),
        ey_mean=('ey', 'mean'), ey_std=('ey', 'std'),
        ey_abs_mean=('ey_abs', 'mean'), ey_abs_std=('ey_abs', 'std'),
        ez_mean=('ez', 'mean'), ez_std=('ez', 'std'),
        x_mean=('ee_ss__x', 'mean'), x_std=('ee_ss__x', 'std'),
        y_mean=('ee_ss__y', 'mean'), y_std=('ee_ss__y', 'std'),
        z_mean=('ee_ss__z', 'mean'), z_std=('ee_ss__z', 'std'),
        r_std=('ee_ss__r', 'std'),
        p_std=('ee_ss__p', 'std'),
        yw_std=('ee_ss__yw', 'std'),
        n=('pos_err_3d', 'count'),
    ).reset_index()

    left = case_stats[case_stats['arm'] == 'left'].copy()
    right = case_stats[case_stats['arm'] == 'right'].copy()
    arms = [a for a, sub in (('left', left), ('right', right)) if not sub.empty]
    if not arms:
        print('[deep] no left/right cases parsed, skipping')
        return None

    _fig1_heatmaps(figdir, left, right, arms)
    _fig2_systematic_bias(figdir, left, right, arms)
    _fig3_signed_errors(figdir, left, right, arms)
    spread_df = _fig4_repeatability(figdir, df, arms)
    _fig5_endpoint_scatter(figdir, df, spread_df, arms)
    _fig6_orientation_instability(figdir, spread_df, arms)
    _fig7_convergence_traces(figdir, trials_dir, case_stats, arms)
    _fig8_error_vs_coord(figdir, left, right, arms)
    _fig9_absolute_error_ranked(figdir, df, left, right, arms)
    _fig10_z_undershoot(figdir, df, arms)
    _print_summary(df, case_stats, spread_df, figdir)
    return figdir


# ── Figure 1 ───────────────────────────────────────────────────────────────────


def _fig1_heatmaps(figdir, left, right, arms):
    fig, axes = plt.subplots(len(arms), 3, figsize=(16, 5 * len(arms)),
                             squeeze=False)
    fig.suptitle('End-Effector Error Summary',
                 fontsize=14, fontweight='bold', y=1.01)
    metrics = [
        ('pos_err_mean', 'Mean 3-D position error (m)', 0, 0.06),
        ('ang_err_mean', 'Mean orientation error (rad)', 0, 0.6),
        ('ez_mean',      'Signed Z error (m) [neg=low]', -0.04, 0.005),
    ]
    arm_dfs = {'left': left, 'right': right}

    for col, (metric, label, vmin, vmax) in enumerate(metrics):
        for row, arm in enumerate(arms):
            ax = axes[row, col]
            arm_df = arm_dfs[arm]
            cmap = 'RdYlGn_r' if 'lin' in metric or 'ang' in metric else 'RdBu_r'

            xy_combos = sorted(arm_df[['tx', 'ty']].drop_duplicates().apply(tuple, axis=1))
            zs = sorted(arm_df['tz'].unique())
            grid = np.full((len(zs), len(xy_combos)), np.nan)
            for i, tz in enumerate(zs):
                for j, (tx, ty) in enumerate(xy_combos):
                    mask = ((arm_df['tx'] == tx) & (arm_df['ty'] == ty)
                            & (arm_df['tz'] == tz))
                    if mask.any():
                        grid[i, j] = arm_df.loc[mask, metric].values[0]

            im = ax.imshow(grid, aspect='auto', cmap=cmap, vmin=vmin, vmax=vmax,
                           origin='lower', interpolation='nearest')
            plt.colorbar(im, ax=ax, shrink=0.8)
            ax.set_xticks(range(len(xy_combos)))
            ax.set_xticklabels([f'x={x:.2f}\ny={y:.2f}' for x, y in xy_combos],
                               fontsize=7)
            ax.set_yticks(range(len(zs)))
            ax.set_yticklabels([f'z={z:.2f}' for z in zs], fontsize=8)
            for i in range(len(zs)):
                for j in range(len(xy_combos)):
                    if not np.isnan(grid[i, j]):
                        v = grid[i, j]
                        txt = f'{v * 1000:.0f}' if 'lin' in metric or 'ez' in metric else f'{v:.2f}'
                        ax.text(j, i, txt, ha='center', va='center', fontsize=7,
                                color='white' if v > (vmax * 0.6) else 'black')
            ax.set_title(f'{arm.upper()} arm — {label}', fontsize=9)
            ax.set_xlabel('(x, y) target', fontsize=8)
            ax.set_ylabel('z target', fontsize=8)

    fig.tight_layout()
    fig.savefig(figdir / 'fig1_error_heatmaps.png', dpi=150, bbox_inches='tight')
    plt.close(fig)
    print('[deep] fig1 heatmaps')


# ── Figure 2 ───────────────────────────────────────────────────────────────────


def _fig2_systematic_bias(figdir, left, right, arms):
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle('Systematic Bias: Target vs. Mean Actual Position (m)',
                 fontsize=13, fontweight='bold')

    proj_pairs = [('tx', 'ty', 'x_mean', 'y_mean', 'X', 'Y'),
                  ('tx', 'tz', 'x_mean', 'z_mean', 'X', 'Z'),
                  ('ty', 'tz', 'y_mean', 'z_mean', 'Y', 'Z')]
    arm_dfs = {'left': left, 'right': right}

    for ax, (ta, tb, aa, ab, la, lb) in zip(axes, proj_pairs):
        for arm in arms:
            arm_df = arm_dfs[arm]
            mult_b = -1 if arm == 'right' and lb == 'Y' else 1
            mult_a = -1 if arm == 'right' and la == 'Y' else 1
            ta_vals = arm_df[ta].values * mult_a
            tb_vals = arm_df[tb].values * mult_b
            aa_vals = arm_df[aa].values * mult_a
            ab_vals = arm_df[ab].values * mult_b
            ax.scatter(ta_vals, tb_vals, marker='o', s=60,
                       color=PALETTE[arm], alpha=0.9, zorder=5)
            ax.scatter(aa_vals, ab_vals, marker='x', s=60,
                       color=PALETTE[arm], alpha=0.5, zorder=4)
            for i in range(len(arm_df)):
                ax.annotate('', xy=(aa_vals[i], ab_vals[i]),
                            xytext=(ta_vals[i], tb_vals[i]),
                            arrowprops=dict(arrowstyle='->', color=PALETTE[arm],
                                            lw=1.2, alpha=0.7))
        ax.set_xlabel(f'{la} (m)', fontsize=11)
        ax.set_ylabel(f'{lb} (m)', fontsize=11)
        ax.set_title(f'{la}–{lb} plane', fontsize=11)
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')

    legend_els = []
    for arm in arms:
        legend_els += [
            Line2D([0], [0], marker='o', color=PALETTE[arm], ls='', ms=8,
                   label=f'{arm.capitalize()} target'),
            Line2D([0], [0], marker='x', color=PALETTE[arm], ls='', ms=8,
                   label=f'{arm.capitalize()} actual (mean)'),
        ]
    fig.legend(handles=legend_els, loc='lower center', ncol=len(legend_els),
               fontsize=10, bbox_to_anchor=(0.5, -0.04))
    fig.tight_layout()
    fig.savefig(figdir / 'fig2_systematic_bias.png', dpi=150, bbox_inches='tight')
    plt.close(fig)
    print('[deep] fig2 systematic bias')


# ── Figure 3 ───────────────────────────────────────────────────────────────────


def _fig3_signed_errors(figdir, left, right, arms):
    fig, axes = plt.subplots(len(arms), 3, figsize=(18, 5 * len(arms)),
                             squeeze=False)
    fig.suptitle('Signed Positional Error per Axis (actual − target)',
                 fontsize=13, fontweight='bold')

    axis_info = [('ex_mean', 'ex_std', 'X error (m)'),
                 ('ey_abs_mean', 'ey_abs_std', '|Y| error (m)'),
                 ('ez_mean', 'ez_std', 'Z error (m)')]
    arm_dfs = {'left': left, 'right': right}

    for col, (err_col, err_std_col, ylabel) in enumerate(axis_info):
        for row, arm in enumerate(arms):
            ax = axes[row, col]
            cases_sorted = arm_dfs[arm].sort_values(['tx', 'ty', 'tz'])
            x_pos = np.arange(len(cases_sorted))
            means = cases_sorted[err_col].values
            stds = (cases_sorted[err_std_col].values
                    if err_std_col in cases_sorted.columns
                    else np.zeros_like(means))
            colors = ['#E53935' if abs(m) > 0.02 else '#FFA000' if abs(m) > 0.01
                      else '#43A047' for m in means]
            ax.bar(x_pos, means, color=colors, alpha=0.8, width=0.6, zorder=3)
            ax.errorbar(x_pos, means, yerr=stds, fmt='none', color='black',
                        capsize=3, lw=1.2, zorder=4)
            ax.axhline(0, color='black', lw=1)
            ax.axhline(0.005, color='gray', lw=0.8, ls='--', label='±5mm')
            ax.axhline(-0.005, color='gray', lw=0.8, ls='--')
            ax.axhline(0.01, color='orange', lw=0.8, ls=':', label='±10mm')
            ax.axhline(-0.01, color='orange', lw=0.8, ls=':')
            short_labels = [f"x{int(r.tx*100)}\ny{int(r.ty*100)}\nz{int(r.tz*100)}"
                            for _, r in cases_sorted.iterrows()]
            ax.set_xticks(x_pos)
            ax.set_xticklabels(short_labels, fontsize=6)
            ax.set_ylabel(ylabel, fontsize=10)
            ax.set_title(f'{arm.upper()} — {ylabel}', fontsize=10)
            ax.grid(axis='y', alpha=0.3, zorder=0)
            if col == 0 and row == 0:
                ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(figdir / 'fig3_signed_errors_per_axis.png', dpi=150,
                bbox_inches='tight')
    plt.close(fig)
    print('[deep] fig3 signed errors')


# ── Figure 4 ───────────────────────────────────────────────────────────────────


def _fig4_repeatability(figdir, df, arms):
    spread_rows = []
    for case_name, grp_df in df.groupby('case'):
        pts = grp_df[['ee_ss__x', 'ee_ss__y', 'ee_ss__z']].values
        centroid = pts.mean(axis=0)
        radii = np.linalg.norm(pts - centroid, axis=1)
        spread_rows.append({
            'case': case_name,
            'arm': grp_df['arm'].iloc[0],
            'tx': grp_df['tx'].iloc[0],
            'ty': grp_df['ty'].iloc[0],
            'tz': grp_df['tz'].iloc[0],
            'pos_spread_m': radii.max(),
            'pos_spread_mean': radii.mean(),
            'x_spread': pts[:, 0].max() - pts[:, 0].min(),
            'y_spread': pts[:, 1].max() - pts[:, 1].min(),
            'z_spread': pts[:, 2].max() - pts[:, 2].min(),
            'roll_spread': grp_df['ee_ss__r'].std(),
            'yaw_spread': grp_df['ee_ss__yw'].std(),
        })
    spread_df = pd.DataFrame(spread_rows).sort_values('pos_spread_m', ascending=False)

    fig, axes = plt.subplots(len(arms), 1, figsize=(16, 5 * len(arms)),
                             squeeze=False)
    fig.suptitle('Repeatability: Trial-to-Trial Position Spread',
                 fontsize=13, fontweight='bold')
    for row, arm in enumerate(arms):
        ax = axes[row, 0]
        sub = spread_df[spread_df['arm'] == arm].copy()
        x_pos = np.arange(len(sub))
        ax.bar(x_pos, sub['x_spread'] * 1000, label='X spread', color='#EF5350', alpha=0.85)
        ax.bar(x_pos, sub['y_spread'] * 1000, bottom=sub['x_spread'] * 1000,
               label='Y spread', color='#42A5F5', alpha=0.85)
        ax.bar(x_pos, sub['z_spread'] * 1000,
               bottom=(sub['x_spread'] + sub['y_spread']) * 1000,
               label='Z spread', color='#66BB6A', alpha=0.85)
        ax.plot(x_pos, sub['pos_spread_m'] * 1000, 'k^', ms=7,
                label='Max radius spread (mm)', zorder=5)
        ax.axhline(1.0, color='green', ls='--', lw=1.2, label='1 mm threshold')
        ax.axhline(5.0, color='orange', ls='--', lw=1.2, label='5 mm threshold')
        labels = [f"x{int(r.tx*100)}_y{int(r.ty*100)}_z{int(r.tz*100)}"
                  for _, r in sub.iterrows()]
        ax.set_xticks(x_pos)
        ax.set_xticklabels(labels, rotation=45, ha='right', fontsize=8)
        ax.set_ylabel('Spread (mm)', fontsize=11)
        ax.set_title(f'{arm.upper()} arm — sorted by worst repeatability', fontsize=11)
        ax.grid(axis='y', alpha=0.3)
        ax.legend(fontsize=9, ncol=3)
    fig.tight_layout()
    fig.savefig(figdir / 'fig4_repeatability_spread.png', dpi=150, bbox_inches='tight')
    plt.close(fig)
    print('[deep] fig4 repeatability')
    return spread_df


# ── Figure 5 ───────────────────────────────────────────────────────────────────


def _fig5_endpoint_scatter(figdir, df, spread_df, arms):
    for arm in arms:
        arm_df_full = df[df['arm'] == arm]
        zs_u = sorted(arm_df_full['tz'].unique())
        xy_combos = sorted(
            arm_df_full[['tx', 'ty']].drop_duplicates().apply(tuple, axis=1))
        if not zs_u or not xy_combos:
            continue

        nrows, ncols = len(zs_u), len(xy_combos)
        fig, axes = plt.subplots(nrows, ncols,
                                 figsize=(4 * ncols, 4 * nrows),
                                 squeeze=False)
        fig.suptitle(
            f'{arm.upper()} ARM — Endpoint Scatter per Case (X–Z plane)\n'
            '● ■ ▲ = trials 1/2/3 | + = target | yellow bg = spread >10mm',
            fontsize=12, fontweight='bold')

        for zi, tz in enumerate(zs_u):
            for ci, (tx, ty) in enumerate(xy_combos):
                ax = axes[zi, ci]
                sub = arm_df_full[(arm_df_full['tx'] == tx)
                                  & (arm_df_full['ty'] == ty)
                                  & (arm_df_full['tz'] == tz)].sort_values('trial_index')
                if sub.empty:
                    ax.set_visible(False)
                    continue
                for i, r in enumerate(sub.itertuples()):
                    m_idx = min(i, len(TRIAL_MARKERS) - 1)
                    ax.scatter(r.ee_ss__x, r.ee_ss__z,
                               marker=TRIAL_MARKERS[m_idx],
                               c=TRIAL_COLORS[m_idx], s=80, zorder=5)
                ax.scatter(tx, tz, marker='+', c='black', s=120, zorder=6,
                           linewidths=2.5)
                theta = np.linspace(0, 2 * np.pi, 100)
                ax.plot(tx + 0.005 * np.cos(theta), tz + 0.005 * np.sin(theta),
                        'k--', lw=0.8, alpha=0.5)
                case_name = sub['case'].iloc[0]
                row_s = spread_df[spread_df['case'] == case_name]
                if not row_s.empty and row_s['pos_spread_m'].values[0] > 0.01:
                    ax.set_facecolor('#FFF8E1')
                margin = 0.035
                ax.set_xlim(tx - margin, tx + margin)
                ax.set_ylim(tz - margin, tz + margin)
                ax.set_aspect('equal')
                ax.tick_params(labelsize=6)
                ax.set_title(f'x{int(tx*100)}_y{int(ty*100)}_z{int(tz*100):02d}',
                             fontsize=8, pad=2)
                if ci == 0:
                    ax.set_ylabel('Z (m)', fontsize=8)
                if zi == nrows - 1:
                    ax.set_xlabel('X (m)', fontsize=8)

        leg = [Line2D([0], [0], marker=m, color=c, ls='', ms=9,
                      label=f'Trial {i+1}')
               for i, (m, c) in enumerate(zip(TRIAL_MARKERS[:3], TRIAL_COLORS[:3]))]
        leg.append(Line2D([0], [0], marker='+', color='black', ls='', ms=11,
                          markeredgewidth=2.5, label='Target'))
        fig.legend(handles=leg, loc='lower center', ncol=5,
                   bbox_to_anchor=(0.5, -0.01), fontsize=10)
        fig.tight_layout()
        fig.savefig(figdir / f'fig5_endpoint_scatter_{arm}.png', dpi=150,
                    bbox_inches='tight')
        plt.close(fig)
    print('[deep] fig5 endpoint scatter')


# ── Figure 6 ───────────────────────────────────────────────────────────────────


def _fig6_orientation_instability(figdir, spread_df, arms):
    fig, axes = plt.subplots(len(arms), 2, figsize=(18, 5 * len(arms)),
                             squeeze=False)
    fig.suptitle('Orientation Instability: Roll & Yaw Std Dev across Trials',
                 fontsize=13, fontweight='bold')
    angle_metrics = [('roll_spread', 'Roll σ (rad)'),
                     ('yaw_spread', 'Yaw σ (rad)')]
    for col, (metric, ylabel) in enumerate(angle_metrics):
        for row, arm in enumerate(arms):
            ax = axes[row, col]
            sub = spread_df[spread_df['arm'] == arm].sort_values('tx')
            x_pos = np.arange(len(sub))
            vals = sub[metric].values
            colors = ['#B71C1C' if v > 0.3 else '#EF6C00' if v > 0.05
                      else '#388E3C' for v in vals]
            ax.bar(x_pos, vals, color=colors, alpha=0.85, width=0.7, zorder=3)
            ax.axhline(0.05, color='orange', ls='--', lw=1.2, label='50 mrad (≈3°)')
            ax.axhline(0.3, color='red', ls='--', lw=1.2, label='300 mrad (≈17°)')
            ax.axhline(0.01, color='green', ls='--', lw=0.8, label='10 mrad (≈0.6°)')
            labels = [f"x{int(r.tx*100)}_y{int(r.ty*100)}_z{int(r.tz*100)}"
                      for _, r in sub.iterrows()]
            ax.set_xticks(x_pos)
            ax.set_xticklabels(labels, rotation=45, ha='right', fontsize=8)
            ax.set_ylabel(ylabel, fontsize=11)
            ax.set_title(f'{arm.upper()} — {ylabel}', fontsize=11)
            ax.grid(axis='y', alpha=0.3)
            if len(vals):
                for idx in np.argsort(vals)[-3:]:
                    ax.text(idx, vals[idx] + 0.01, f'{vals[idx]:.2f}', ha='center',
                            fontsize=8, color='black', fontweight='bold')
            if col == 1 and row == 0:
                ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(figdir / 'fig6_orientation_instability.png', dpi=150,
                bbox_inches='tight')
    plt.close(fig)
    print('[deep] fig6 orientation instability')


# ── Figure 7 ───────────────────────────────────────────────────────────────────


def _fig7_convergence_traces(figdir, trials_dir, case_stats, arms):
    if not trials_dir.exists():
        print('[deep] fig7 skipped (no trials/ directory)')
        return
    rep_rows = case_stats.sort_values('pos_err_mean', ascending=False).head(6)
    if rep_rows.empty:
        return
    rep_cases = [(r['case'], f"{r['arm'].upper()} {r['case']}")
                 for _, r in rep_rows.iterrows()]

    fig, axes = plt.subplots(len(rep_cases), 2,
                             figsize=(16, 3.5 * len(rep_cases)),
                             squeeze=False)
    fig.suptitle('IK Convergence Time Series — Position & Orientation Error vs Step',
                 fontsize=13, fontweight='bold')
    for row, (case, label) in enumerate(rep_cases):
        ax_pos = axes[row, 0]
        ax_ang = axes[row, 1]
        for trial in range(1, 4):
            d = _find_trial(trials_dir, case, trial)
            if d is None:
                continue
            t = d['t'].flatten()
            ferr = d['frame_err']
            lin = np.linalg.norm(ferr[:, :3], axis=1) * 100
            ang = np.linalg.norm(ferr[:, 3:], axis=1)
            c = TRIAL_COLORS[trial - 1]
            ax_pos.plot(t, lin, color=c, lw=1.5, label=f't{trial}', alpha=0.9)
            ax_ang.plot(t, ang, color=c, lw=1.5, label=f't{trial}', alpha=0.9)
        ax_pos.axhline(0.5, color='orange', ls='--', lw=1, label='5mm')
        ax_pos.axhline(1.0, color='red', ls=':', lw=1, label='10mm')
        ax_pos.set_ylabel('Lin error (cm)', fontsize=9)
        ax_pos.set_title(label + ' — Position', fontsize=9)
        ax_pos.grid(alpha=0.3)
        ax_pos.legend(fontsize=8)
        ax_ang.axhline(0.02, color='orange', ls='--', lw=1, label='0.02 rad')
        ax_ang.set_ylabel('Ang error (rad)', fontsize=9)
        ax_ang.set_title(label + ' — Orientation', fontsize=9)
        ax_ang.grid(alpha=0.3)
        if row == len(rep_cases) - 1:
            ax_pos.set_xlabel('Time (s)', fontsize=9)
            ax_ang.set_xlabel('Time (s)', fontsize=9)
    fig.tight_layout()
    fig.savefig(figdir / 'fig7_convergence_traces.png', dpi=150, bbox_inches='tight')
    plt.close(fig)
    print('[deep] fig7 convergence traces')


# ── Figure 8 ───────────────────────────────────────────────────────────────────


def _fig8_error_vs_coord(figdir, left, right, arms):
    fig, axes = plt.subplots(len(arms), 3, figsize=(18, 5 * len(arms)),
                             squeeze=False)
    fig.suptitle('Error vs. Target Coordinate — Structural Analysis of Position Bias',
                 fontsize=13, fontweight='bold')
    coords = [('tx', 'Target X (m)'), ('ty', 'Target |Y| (m)'), ('tz', 'Target Z (m)')]
    err_labels = [('ex_mean', 'ex_std', 'Signed X error (m)'),
                  ('ey_mean', 'ey_std', 'Signed |Y| error (m)'),
                  ('ez_mean', 'ez_std', 'Signed Z error (m)')]
    arm_dfs = {'left': left, 'right': right}

    for row, arm in enumerate(arms):
        arm_df = arm_dfs[arm]
        for col, ((coord, coord_label), (err, err_std_col, err_label)) in \
                enumerate(zip(coords, err_labels)):
            ax = axes[row, col]
            x_vals = arm_df[coord].values
            y_vals = arm_df[err].values
            y_err = (arm_df[err_std_col].values
                     if err_std_col in arm_df else None)
            scatter = ax.scatter(x_vals, y_vals, c=arm_df['pos_err_mean'],
                                 cmap='hot_r', s=80, vmin=0, vmax=0.06,
                                 zorder=5, edgecolors='gray', lw=0.5)
            if y_err is not None:
                ax.errorbar(x_vals, y_vals, yerr=y_err, fmt='none',
                            color='gray', alpha=0.5, capsize=3)
            plt.colorbar(scatter, ax=ax, label='3-D error (m)', shrink=0.8)
            ax.axhline(0, color='black', lw=1)
            ax.axhline(0.005, color='gray', ls='--', lw=0.8)
            ax.axhline(-0.005, color='gray', ls='--', lw=0.8)
            if len(np.unique(x_vals)) >= 2:
                coeffs = np.polyfit(x_vals, y_vals, 1)
                xfit = np.linspace(x_vals.min(), x_vals.max(), 50)
                ax.plot(xfit, np.polyval(coeffs, xfit), 'b-', lw=1.5,
                        label=f'slope={coeffs[0]:.2f}')
                ax.legend(fontsize=8)
            ax.set_xlabel(coord_label, fontsize=10)
            ax.set_ylabel(err_label, fontsize=10)
            ax.set_title(f'{arm.upper()} — {err_label} vs {coord_label}', fontsize=9)
            ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(figdir / 'fig8_error_vs_coord.png', dpi=150, bbox_inches='tight')
    plt.close(fig)
    print('[deep] fig8 error vs coordinate')


# ── Figure 9 ───────────────────────────────────────────────────────────────────


def _fig9_absolute_error_ranked(figdir, df, left, right, arms):
    fig, axes = plt.subplots(1, len(arms), figsize=(9 * len(arms), 8),
                             squeeze=False)
    fig.suptitle('Absolute 3-D Position Error: Cases Ranked by Mean Error',
                 fontsize=13, fontweight='bold')
    arm_dfs = {'left': left, 'right': right}
    for col, arm in enumerate(arms):
        ax = axes[0, col]
        sub = arm_dfs[arm].sort_values('pos_err_mean', ascending=False)
        x_pos = np.arange(len(sub))
        colors = ['#B71C1C' if v > 0.04 else '#EF6C00' if v > 0.025
                  else '#F9A825' if v > 0.015 else '#388E3C'
                  for v in sub['pos_err_mean']]
        ax.bar(x_pos, sub['pos_err_mean'] * 1000,
               yerr=sub['pos_err_std'] * 1000,
               color=colors, alpha=0.85, capsize=4, width=0.7, zorder=3,
               error_kw={'lw': 1.5})
        ax.axhline(10, color='green', ls='--', lw=1.5, label='10mm')
        ax.axhline(20, color='orange', ls='--', lw=1.5, label='20mm')
        ax.axhline(40, color='red', ls='--', lw=1.5, label='40mm')
        overall_mean = df['pos_err_3d'].mean()
        ax.axhline(overall_mean * 1000, color='purple', ls=':', lw=2,
                   label=f'Overall mean = {overall_mean*1000:.1f}mm')
        labels = [f"x{int(r.tx*100)}\ny{int(r.ty*100)}\nz{int(r.tz*100)}"
                  for _, r in sub.iterrows()]
        ax.set_xticks(x_pos)
        ax.set_xticklabels(labels, fontsize=8)
        ax.set_ylabel('3-D position error to target (mm)', fontsize=12)
        ax.set_title(f'{arm.upper()} arm — sorted worst→best', fontsize=12)
        ax.grid(axis='y', alpha=0.3)
        ax.legend(fontsize=9)
        for i, (_, r) in enumerate(sub.iterrows()):
            ax.text(i, r.pos_err_mean * 1000 + r.pos_err_std * 1000 + 0.5,
                    f"{r.pos_err_mean*1000:.1f}", ha='center', va='bottom',
                    fontsize=7.5, fontweight='bold')
    fig.tight_layout()
    fig.savefig(figdir / 'fig9_absolute_error_ranked.png', dpi=150,
                bbox_inches='tight')
    plt.close(fig)
    print('[deep] fig9 absolute error ranked')


# ── Figure 10 ──────────────────────────────────────────────────────────────────


def _fig10_z_undershoot(figdir, df, arms):
    fig, axes = plt.subplots(1, len(arms), figsize=(7 * len(arms), 7),
                             squeeze=False)
    fig.suptitle('Z-Axis: Actual Z vs Target Z (dashed = perfect)',
                 fontsize=13, fontweight='bold')
    zs_target = sorted(df['tz'].unique())
    for col, arm in enumerate(arms):
        ax = axes[0, col]
        arm_df_full = df[df['arm'] == arm]
        trial_idx = arm_df_full['trial_index'].astype(int).clip(1, len(TRIAL_COLORS))
        ax.scatter(arm_df_full['tz'], arm_df_full['ee_ss__z'],
                   c=[TRIAL_COLORS[i - 1] for i in trial_idx],
                   s=30, alpha=0.6, zorder=4, label='Individual trials')
        for tz in zs_target:
            sub = arm_df_full[arm_df_full['tz'] == tz]
            if sub.empty:
                continue
            ax.errorbar(tz, sub['ee_ss__z'].mean(), yerr=sub['ee_ss__z'].std(),
                        fmt='kD', ms=8, capsize=5, lw=2, zorder=5)
        zmin, zmax = df['tz'].min() - 0.005, df['tz'].max() + 0.005
        ax.plot([zmin, zmax], [zmin, zmax], 'k--', lw=1.5,
                label='Perfect (actual=target)')
        if len(arm_df_full) >= 2 and arm_df_full['tz'].nunique() >= 2:
            coeffs = np.polyfit(arm_df_full['tz'], arm_df_full['ee_ss__z'], 1)
            xfit = np.linspace(zmin, zmax, 50)
            ax.plot(xfit, np.polyval(coeffs, xfit), color=PALETTE[arm], lw=2,
                    label=f'Fit: slope={coeffs[0]:.2f}, offset={coeffs[1]*100:.1f}cm')
        mean_undershoot = (arm_df_full['ee_ss__z'] - arm_df_full['tz']).mean()
        ax.text(0.05, 0.95, f'Mean undershoot: {mean_undershoot*1000:.1f} mm',
                transform=ax.transAxes, fontsize=11, va='top', color='red',
                bbox=dict(boxstyle='round', fc='white', ec='red', alpha=0.8))
        ax.set_xlabel('Target Z (m)', fontsize=12)
        ax.set_ylabel('Actual Z (m)', fontsize=12)
        ax.set_title(f'{arm.upper()} arm', fontsize=12)
        ax.set_aspect('equal')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(figdir / 'fig10_z_undershoot.png', dpi=150, bbox_inches='tight')
    plt.close(fig)
    print('[deep] fig10 z undershoot')


# ── Console summary ────────────────────────────────────────────────────────────


def _print_summary(df, case_stats, spread_df, figdir):
    print('\n' + '=' * 60)
    print('KEY STATISTICS SUMMARY')
    print('=' * 60)
    print(f'Total trials: {len(df)}')
    print(f'Total cases: {df["case"].nunique()}')
    print()
    print('POSITION ERROR to target (true world-frame, all trials):')
    print(f'  Mean 3-D: {df["pos_err_3d"].mean()*1000:.1f} mm')
    worst = case_stats.loc[case_stats['pos_err_mean'].idxmax()]
    best = case_stats.loc[case_stats['pos_err_mean'].idxmin()]
    print(f'  Worst case: {worst["pos_err_mean"]*1000:.1f} mm ({worst["case"]})')
    print(f'  Best case:  {best["pos_err_mean"]*1000:.1f} mm ({best["case"]})')
    print(f'  IK residual mean (NOT accuracy): '
          f'{df["lin_err_steady_m"].mean()*1000:.1f} mm')
    print()
    print('Z SYSTEMATIC UNDERSHOOT:')
    for arm in df['arm'].unique():
        sub = df[df['arm'] == arm]
        undershoot = (sub['ee_ss__z'] - sub['tz']).mean()
        print(f'  {arm}: {undershoot*1000:.1f} mm (negative=undershoots)')
    print()
    print('REPEATABILITY (position spread across trials):')
    if not spread_df.empty:
        print(f'  Worst case: {spread_df["pos_spread_m"].max()*1000:.1f} mm '
              f'({spread_df.iloc[0]["case"]})')
        print(f'  Mean spread: {spread_df["pos_spread_m"].mean()*1000:.1f} mm')
        worst_yaw = spread_df.sort_values('yaw_spread', ascending=False).iloc[0]
        print(f'  Worst yaw spread: {worst_yaw["yaw_spread"]:.3f} rad = '
              f'{np.degrees(worst_yaw["yaw_spread"]):.1f}° ({worst_yaw["case"]})')
    print()
    print(f'Plots saved to: {figdir}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', help='run directory containing summary.csv and trials/')
    args = parser.parse_args()
    out = run_deep_analysis(Path(args.run_dir).resolve())
    if out is None:
        sys.exit(1)


if __name__ == '__main__':
    main()
