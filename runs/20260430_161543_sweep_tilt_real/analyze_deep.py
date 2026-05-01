"""
Deep characterization analysis for sweep_tilt_real run.
Generates 8 figures covering: absolute error, systematic bias, repeatability,
bimodal IK solutions, orientation instability, and spatial error structure.
"""

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import FancyArrowPatch
from matplotlib.colors import Normalize
import matplotlib.cm as cm
from pathlib import Path
import itertools

RUN_DIR = Path(__file__).parent
TRIALS_DIR = RUN_DIR / 'trials'
PLOTS_DIR = RUN_DIR / 'plots'
PLOTS_DIR.mkdir(exist_ok=True)

FIGDIR = PLOTS_DIR / 'deep_analysis'
FIGDIR.mkdir(exist_ok=True)

# ── Load summary CSV ──────────────────────────────────────────────────────────
df = pd.read_csv(RUN_DIR / 'summary.csv')

# Parse target from case name (also available in npz target_pose)
def parse_case(name):
    # e.g. left_pick_x30_y20_z06
    parts = name.split('_')
    arm = parts[0]
    x = float(parts[2][1:]) / 100
    y = float(parts[3][1:]) / 100
    z = float(parts[4][1:]) / 100
    return arm, x, y, z

df[['arm','tx','ty','tz']] = pd.DataFrame(
    [parse_case(c) for c in df['case']], index=df.index)

# True world-frame signed errors:
#   - left arm: target_y = +ty, right arm: target_y = -ty
df['target_y_world'] = np.where(df['arm']=='left', df['ty'], -df['ty'])
df['ex'] = df['ee_ss__x'] - df['tx']
df['ey'] = df['ee_ss__y'] - df['target_y_world']  # signed: + = too far lateral
df['ez'] = df['ee_ss__z'] - df['tz']
df['pos_err_3d'] = np.sqrt(df['ex']**2 + df['ey']**2 + df['ez']**2)
# |Y| signed error for plotting both arms on same scale
df['ey_abs'] = df['ee_ss__y'].abs() - df['ty']

# ── Per-case aggregation ──────────────────────────────────────────────────────
grp = df.groupby(['case','arm','tx','ty','tz'])

case_stats = grp.agg(
    # True world-frame position error (target vs actual)
    pos_err_mean=('pos_err_3d','mean'),
    pos_err_std=('pos_err_3d','std'),
    # IK convergence residual (NOT the accuracy metric)
    ik_resid_mean=('lin_err_steady_m','mean'),
    ang_err_mean=('ang_err_steady_rad','mean'),
    ang_err_std=('ang_err_steady_rad','std'),
    ex_mean=('ex','mean'), ex_std=('ex','std'),
    ey_mean=('ey','mean'), ey_std=('ey','std'),
    ey_abs_mean=('ey_abs','mean'), ey_abs_std=('ey_abs','std'),
    ez_mean=('ez','mean'), ez_std=('ez','std'),
    x_mean=('ee_ss__x','mean'), x_std=('ee_ss__x','std'),
    y_mean=('ee_ss__y','mean'), y_std=('ee_ss__y','std'),
    z_mean=('ee_ss__z','mean'), z_std=('ee_ss__z','std'),
    r_std=('ee_ss__r','std'),
    p_std=('ee_ss__p','std'),
    yw_std=('ee_ss__yw','std'),
    n=('pos_err_3d','count'),
).reset_index()

left = case_stats[case_stats['arm']=='left'].copy()
right = case_stats[case_stats['arm']=='right'].copy()

PALETTE = {'left': '#2196F3', 'right': '#F44336'}
LIGHT = {'left': '#BBDEFB', 'right': '#FFCDD2'}

# ═══════════════════════════════════════════════════════════════════════════════
# FIGURE 1 — Grand error summary: heatmap grids over workspace
# ═══════════════════════════════════════════════════════════════════════════════
fig1, axes = plt.subplots(2, 3, figsize=(16, 10))
fig1.suptitle('Sweep Tilt — End-Effector Error Summary (real robot)',
              fontsize=14, fontweight='bold', y=1.01)

metrics = [
    ('pos_err_mean', 'Mean 3-D position error (m)', 0, 0.06),
    ('ang_err_mean', 'Mean orientation error (rad)', 0, 0.6),
    ('ez_mean',      'Signed Z error (m) [neg=low]', -0.04, 0.005),
]

for col, (metric, label, vmin, vmax) in enumerate(metrics):
    for row, (arm_df, arm) in enumerate([(left,'left'),(right,'right')]):
        ax = axes[row, col]
        cmap = 'RdYlGn_r' if 'lin' in metric or 'ang' in metric else 'RdBu_r'

        # Build pivot: rows=tz, cols=(tx,ty)
        xy_combos = sorted(arm_df[['tx','ty']].drop_duplicates().apply(tuple,axis=1))
        zs = sorted(arm_df['tz'].unique())

        grid = np.full((len(zs), len(xy_combos)), np.nan)
        for i, tz in enumerate(zs):
            for j, (tx,ty) in enumerate(xy_combos):
                mask = (arm_df['tx']==tx) & (arm_df['ty']==ty) & (arm_df['tz']==tz)
                if mask.any():
                    grid[i,j] = arm_df.loc[mask, metric].values[0]

        im = ax.imshow(grid, aspect='auto', cmap=cmap, vmin=vmin, vmax=vmax,
                       origin='lower', interpolation='nearest')
        plt.colorbar(im, ax=ax, shrink=0.8)

        ax.set_xticks(range(len(xy_combos)))
        ax.set_xticklabels([f'x={x:.2f}\ny={y:.2f}' for x,y in xy_combos],
                           fontsize=7)
        ax.set_yticks(range(len(zs)))
        ax.set_yticklabels([f'z={z:.2f}' for z in zs], fontsize=8)

        # Annotate cells with values
        for i in range(len(zs)):
            for j in range(len(xy_combos)):
                if not np.isnan(grid[i,j]):
                    ax.text(j, i, f'{grid[i,j]*1000:.0f}'
                            if 'lin' in metric or 'ez' in metric
                            else f'{grid[i,j]:.2f}',
                            ha='center', va='center', fontsize=7,
                            color='white' if grid[i,j] > (vmax*0.6) else 'black')

        ax.set_title(f'{arm.upper()} arm — {label}', fontsize=9)
        ax.set_xlabel('(x, y) target', fontsize=8)
        ax.set_ylabel('z target', fontsize=8)

fig1.tight_layout()
fig1.savefig(FIGDIR / 'fig1_error_heatmaps.png', dpi=150, bbox_inches='tight')
plt.close(fig1)
print('Fig 1 done')

# ═══════════════════════════════════════════════════════════════════════════════
# FIGURE 2 — Systematic bias vectors (target → actual, mean of 3 trials)
# ═══════════════════════════════════════════════════════════════════════════════
fig2, axes = plt.subplots(1, 3, figsize=(18, 6))
fig2.suptitle('Systematic Bias: Target vs. Mean Actual Position (in meters)',
              fontsize=13, fontweight='bold')

proj_pairs = [('tx','ty','x_mean','y_mean','X','Y'),
              ('tx','tz','x_mean','z_mean','X','Z'),
              ('ty','tz','y_mean','z_mean','Y','Z')]

for ax, (ta, tb, aa, ab, la, lb) in zip(axes, proj_pairs):
    for arm, arm_df in [('left',left),('right',right)]:
        # Mirror right arm y back to negative for display
        mult_b = -1 if arm=='right' and lb=='Y' else 1
        mult_a = -1 if arm=='right' and la=='Y' else 1

        ta_vals = arm_df[ta].values * mult_a
        tb_vals = arm_df[tb].values * mult_b
        aa_vals = arm_df[aa].values * mult_a
        ab_vals = arm_df[ab].values * mult_b

        ax.scatter(ta_vals, tb_vals, marker='o', s=60,
                   color=PALETTE[arm], alpha=0.9, zorder=5,
                   label=f'{arm} target' if ax == axes[0] else '')
        ax.scatter(aa_vals, ab_vals, marker='x', s=60,
                   color=PALETTE[arm], alpha=0.5, zorder=4)

        for i in range(len(arm_df)):
            dx = aa_vals[i] - ta_vals[i]
            dy = ab_vals[i] - tb_vals[i]
            ax.annotate('', xy=(aa_vals[i], ab_vals[i]),
                        xytext=(ta_vals[i], tb_vals[i]),
                        arrowprops=dict(arrowstyle='->', color=PALETTE[arm],
                                        lw=1.2, alpha=0.7))

    ax.set_xlabel(f'{la} (m)', fontsize=11)
    ax.set_ylabel(f'{lb} (m)', fontsize=11)
    ax.set_title(f'{la}–{lb} plane', fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

# Legend
from matplotlib.lines import Line2D
legend_els = [
    Line2D([0],[0], marker='o', color=PALETTE['left'],  ls='', ms=8, label='Left target'),
    Line2D([0],[0], marker='x', color=PALETTE['left'],  ls='', ms=8, label='Left actual (mean)'),
    Line2D([0],[0], marker='o', color=PALETTE['right'], ls='', ms=8, label='Right target'),
    Line2D([0],[0], marker='x', color=PALETTE['right'], ls='', ms=8, label='Right actual (mean)'),
]
fig2.legend(handles=legend_els, loc='lower center', ncol=4, fontsize=10,
            bbox_to_anchor=(0.5, -0.04))
fig2.tight_layout()
fig2.savefig(FIGDIR / 'fig2_systematic_bias.png', dpi=150, bbox_inches='tight')
plt.close(fig2)
print('Fig 2 done')

# ═══════════════════════════════════════════════════════════════════════════════
# FIGURE 3 — Per-axis signed error: shows the structure of the bias
# ═══════════════════════════════════════════════════════════════════════════════
fig3, axes = plt.subplots(2, 3, figsize=(18, 10))
fig3.suptitle('Signed Positional Error per Axis (target − actual)',
              fontsize=13, fontweight='bold')

axis_info = [('ex_mean','ex_std','X error (m)'), ('ey_abs_mean','ey_abs_std','|Y| error (m)'), ('ez_mean','ez_std','Z error (m)')]

for col, (err_col, err_std_col, ylabel) in enumerate(axis_info):
    for row, (arm, arm_df_cs) in enumerate([('left',left),('right',right)]):
        ax = axes[row, col]

        cases_sorted = arm_df_cs.sort_values(['tx','ty','tz'])
        x_pos = np.arange(len(cases_sorted))
        means = cases_sorted[err_col].values
        stds = cases_sorted[err_std_col].values if err_std_col in cases_sorted.columns else np.zeros_like(means)

        colors = ['#E53935' if abs(m) > 0.02 else '#FFA000' if abs(m) > 0.01
                  else '#43A047' for m in means]

        bars = ax.bar(x_pos, means, color=colors, alpha=0.8, width=0.6, zorder=3)
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

fig3.tight_layout()
fig3.savefig(FIGDIR / 'fig3_signed_errors_per_axis.png', dpi=150, bbox_inches='tight')
plt.close(fig3)
print('Fig 3 done')

# ═══════════════════════════════════════════════════════════════════════════════
# FIGURE 4 — Repeatability: trial-to-trial spread, all cases sorted by spread
# ═══════════════════════════════════════════════════════════════════════════════
# Compute 3D position spread per case (radius of enclosing sphere)
spread_rows = []
for case_name, grp_df in df.groupby('case'):
    pts = grp_df[['ee_ss__x','ee_ss__y','ee_ss__z']].values
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
        'x_spread': pts[:,0].max() - pts[:,0].min(),
        'y_spread': pts[:,1].max() - pts[:,1].min(),
        'z_spread': pts[:,2].max() - pts[:,2].min(),
        'roll_spread': grp_df['ee_ss__r'].std(),
        'yaw_spread': grp_df['ee_ss__yw'].std(),
    })
spread_df = pd.DataFrame(spread_rows).sort_values('pos_spread_m', ascending=False)

fig4, axes = plt.subplots(2, 1, figsize=(16, 10))
fig4.suptitle('Repeatability: Trial-to-Trial Position Spread (3 trials per case)',
              fontsize=13, fontweight='bold')

for row, arm in enumerate(['left','right']):
    ax = axes[row]
    sub = spread_df[spread_df['arm']==arm].copy()
    x_pos = np.arange(len(sub))

    # Stacked bars: x, y, z spread components
    ax.bar(x_pos, sub['x_spread']*1000, label='X spread', color='#EF5350', alpha=0.85)
    ax.bar(x_pos, sub['y_spread']*1000, bottom=sub['x_spread']*1000,
           label='Y spread', color='#42A5F5', alpha=0.85)
    ax.bar(x_pos, sub['z_spread']*1000,
           bottom=(sub['x_spread']+sub['y_spread'])*1000,
           label='Z spread', color='#66BB6A', alpha=0.85)

    ax.plot(x_pos, sub['pos_spread_m']*1000, 'k^', ms=7,
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

fig4.tight_layout()
fig4.savefig(FIGDIR / 'fig4_repeatability_spread.png', dpi=150, bbox_inches='tight')
plt.close(fig4)
print('Fig 4 done')

# ═══════════════════════════════════════════════════════════════════════════════
# FIGURE 5 — Bimodal IK: scatter of all trial endpoints per case (X-Z plane)
#            4 rows (z values) × 4 cols (x,y combos), one grid per arm
# ═══════════════════════════════════════════════════════════════════════════════
markers = ['o', 's', '^']
trial_colors = ['#1565C0', '#C62828', '#2E7D32']

from matplotlib.lines import Line2D

for arm in ['left', 'right']:
    arm_df_full = df[df['arm']==arm]
    xs = sorted(arm_df_full['tx'].unique())
    ys = sorted(arm_df_full['ty'].unique())
    zs_u = sorted(arm_df_full['tz'].unique())
    xy_combos = list(itertools.product(xs, ys))  # 4 combos

    fig5, axes5 = plt.subplots(len(zs_u), len(xy_combos),
                               figsize=(4*len(xy_combos), 4*len(zs_u)))
    fig5.suptitle(
        f'{arm.upper()} ARM — Endpoint Scatter per Case (X–Z plane)\n'
        '●=t1 ■=t2 ▲=t3 | + = target | yellow bg = spread >10mm',
        fontsize=12, fontweight='bold')

    for zi, tz in enumerate(zs_u):
        for ci, (tx, ty) in enumerate(xy_combos):
            ax = axes5[zi, ci]
            sub = arm_df_full[(arm_df_full['tx']==tx) &
                               (arm_df_full['ty']==ty) &
                               (arm_df_full['tz']==tz)].sort_values('trial_index')
            if sub.empty:
                ax.set_visible(False)
                continue

            for i, r in enumerate(sub.itertuples()):
                ax.scatter(r.ee_ss__x, r.ee_ss__z,
                           marker=markers[i], c=trial_colors[i], s=80, zorder=5)

            ax.scatter(tx, tz, marker='+', c='black', s=120, zorder=6,
                       linewidths=2.5)

            # 5mm tolerance circle
            theta = np.linspace(0, 2*np.pi, 100)
            ax.plot(tx + 0.005*np.cos(theta), tz + 0.005*np.sin(theta),
                    'k--', lw=0.8, alpha=0.5)

            # Yellow background if high spread
            case_name = f'{arm}_pick_x{int(tx*100)}_y{int(ty*100)}_z{int(tz*100):02d}'
            row_s = spread_df[spread_df['case']==case_name]
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
            if zi == len(zs_u)-1:
                ax.set_xlabel('X (m)', fontsize=8)

    leg = [Line2D([0],[0], marker=m, color=c, ls='', ms=9, label=f'Trial {i+1}')
           for i,(m,c) in enumerate(zip(markers, trial_colors))]
    leg.append(Line2D([0],[0], marker='+', color='black', ls='', ms=11,
                      markeredgewidth=2.5, label='Target'))
    fig5.legend(handles=leg, loc='lower center', ncol=5,
                bbox_to_anchor=(0.5, -0.01), fontsize=10)
    fig5.tight_layout()
    fig5.savefig(FIGDIR / f'fig5_endpoint_scatter_{arm}.png', dpi=150,
                 bbox_inches='tight')
    plt.close(fig5)

print('Fig 5 done')

# ═══════════════════════════════════════════════════════════════════════════════
# FIGURE 6 — Orientation instability: roll and yaw spread per case
# ═══════════════════════════════════════════════════════════════════════════════
fig6, axes = plt.subplots(2, 2, figsize=(18, 10))
fig6.suptitle('Orientation Instability: Roll & Yaw Std Dev across 3 Trials\n'
              '(Target orientation is fixed: pitch≈π/2, roll=yaw=0)',
              fontsize=13, fontweight='bold')

angle_metrics = [('roll_spread','Roll σ (rad)'), ('yaw_spread','Yaw σ (rad)')]

for col, (metric, ylabel) in enumerate(angle_metrics):
    for row, arm in enumerate(['left','right']):
        ax = axes[row, col]
        sub = spread_df[spread_df['arm']==arm].sort_values('tx')
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

        # Annotate worst cases
        worst_idx = np.argsort(vals)[-3:]
        for idx in worst_idx:
            ax.text(idx, vals[idx]+0.01, f'{vals[idx]:.2f}', ha='center',
                    fontsize=8, color='black', fontweight='bold')

        if col == 1 and row == 0:
            ax.legend(fontsize=9)

fig6.tight_layout()
fig6.savefig(FIGDIR / 'fig6_orientation_instability.png', dpi=150, bbox_inches='tight')
plt.close(fig6)
print('Fig 6 done')

# ═══════════════════════════════════════════════════════════════════════════════
# FIGURE 7 — Error convergence time series (selected representative cases)
#            Shows both good and catastrophically bad convergence
# ═══════════════════════════════════════════════════════════════════════════════

def load_trial(case, trial=1):
    stem = f'{case}__position_cost=20.0_orientation_cost=10.0_lm_damping=1.0__t{trial}'
    p = TRIALS_DIR / f'{stem}.npz'
    return np.load(p) if p.exists() else None

# Pick representative cases: best, worst, bimodal
rep_cases = [
    ('left_pick_x30_y30_z14', 'LEFT best (x30,y30,z14)'),
    ('left_pick_x35_y30_z06', 'LEFT worst abs error (x35,y30,z06)'),
    ('left_pick_x35_y20_z10', 'LEFT bimodal IK (x35,y20,z10)'),
    ('right_pick_x35_y30_z06', 'RIGHT worst abs error (x35,y30,z06)'),
    ('right_pick_x35_y30_z14', 'RIGHT orientation chaos (x35,y30,z14)'),
    ('right_pick_x30_y20_z06', 'RIGHT worst bias (x30,y20,z06)'),
]

fig7, axes = plt.subplots(len(rep_cases), 2, figsize=(16, 3.5*len(rep_cases)))
fig7.suptitle('IK Convergence Time Series — Position & Orientation Error vs Step',
              fontsize=13, fontweight='bold')

for row, (case, label) in enumerate(rep_cases):
    ax_pos = axes[row, 0]
    ax_ang = axes[row, 1]

    for trial in [1, 2, 3]:
        d = load_trial(case, trial)
        if d is None:
            continue
        t = d['t'].flatten()
        ferr = d['frame_err']  # (T, 6): [dx,dy,dz, drx,dry,drz]
        lin = np.linalg.norm(ferr[:, :3], axis=1) * 100  # cm
        ang = np.linalg.norm(ferr[:, 3:], axis=1)  # rad

        c = trial_colors[trial-1]
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

    if row == len(rep_cases)-1:
        ax_pos.set_xlabel('Time (s)', fontsize=9)
        ax_ang.set_xlabel('Time (s)', fontsize=9)

fig7.tight_layout()
fig7.savefig(FIGDIR / 'fig7_convergence_traces.png', dpi=150, bbox_inches='tight')
plt.close(fig7)
print('Fig 7 done')

# ═══════════════════════════════════════════════════════════════════════════════
# FIGURE 8 — Error vs. spatial coordinate: reveals structure of bias
# ═══════════════════════════════════════════════════════════════════════════════
fig8, axes = plt.subplots(2, 3, figsize=(18, 10))
fig8.suptitle('Error vs. Target Coordinate — Structural Analysis of Position Bias',
              fontsize=13, fontweight='bold')

coords = [('tx', 'Target X (m)'), ('ty', 'Target |Y| (m)'), ('tz', 'Target Z (m)')]
err_labels = [('ex_mean', 'ex_std', 'Signed X error (m)'),
              ('ey_mean', 'ey_std', 'Signed |Y| error (m)'),
              ('ez_mean', 'ez_std', 'Signed Z error (m)')]

for row, (arm, arm_df_cs) in enumerate([('left',left),('right',right)]):
    for col, ((coord, coord_label), (err, err_std_col, err_label)) in \
            enumerate(zip(coords, err_labels)):
        ax = axes[row, col]

        x_vals = arm_df_cs[coord].values
        y_vals = arm_df_cs[err].values
        y_err  = arm_df_cs[err_std_col].values if err_std_col in arm_df_cs else None

        # Color by the OTHER two coordinates to show interaction
        scatter = ax.scatter(x_vals, y_vals, c=arm_df_cs['pos_err_mean'],
                             cmap='hot_r', s=80, vmin=0, vmax=0.06,
                             zorder=5, edgecolors='gray', lw=0.5)
        if y_err is not None:
            ax.errorbar(x_vals, y_vals, yerr=y_err, fmt='none',
                        color='gray', alpha=0.5, capsize=3)

        plt.colorbar(scatter, ax=ax, label='3-D error (m)', shrink=0.8)
        ax.axhline(0, color='black', lw=1)
        ax.axhline(0.005, color='gray', ls='--', lw=0.8)
        ax.axhline(-0.005, color='gray', ls='--', lw=0.8)

        # Fit linear trend
        coeffs = np.polyfit(x_vals, y_vals, 1)
        xfit = np.linspace(x_vals.min(), x_vals.max(), 50)
        ax.plot(xfit, np.polyval(coeffs, xfit), 'b-', lw=1.5,
                label=f'slope={coeffs[0]:.2f}')

        ax.set_xlabel(coord_label, fontsize=10)
        ax.set_ylabel(err_label, fontsize=10)
        ax.set_title(f'{arm.upper()} — {err_label} vs {coord_label}', fontsize=9)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)

fig8.tight_layout()
fig8.savefig(FIGDIR / 'fig8_error_vs_coord.png', dpi=150, bbox_inches='tight')
plt.close(fig8)
print('Fig 8 done')

# ═══════════════════════════════════════════════════════════════════════════════
# FIGURE 9 — The big picture: real vs sim comparison (using results.md numbers)
#            and per-case error magnitude ranked
# ═══════════════════════════════════════════════════════════════════════════════
fig9, axes = plt.subplots(1, 2, figsize=(18, 8))
fig9.suptitle('Absolute 3-D Position Error: All Cases, Both Arms\n'
              'Error bars = std dev across 3 trials', fontsize=13, fontweight='bold')

for row, (arm, arm_df_cs) in enumerate([('left',left),('right',right)]):
    ax = axes[row]
    sub = arm_df_cs.sort_values('pos_err_mean', ascending=False)
    x_pos = np.arange(len(sub))

    colors = ['#B71C1C' if v > 0.04 else '#EF6C00' if v > 0.025
              else '#F9A825' if v > 0.015 else '#388E3C'
              for v in sub['pos_err_mean']]

    ax.bar(x_pos, sub['pos_err_mean']*1000, yerr=sub['pos_err_std']*1000,
           color=colors, alpha=0.85, capsize=4, width=0.7, zorder=3,
           error_kw={'lw':1.5})

    # Horizontal thresholds
    ax.axhline(10, color='green', ls='--', lw=1.5, label='10mm')
    ax.axhline(20, color='orange', ls='--', lw=1.5, label='20mm')
    ax.axhline(40, color='red', ls='--', lw=1.5, label='40mm')
    overall_mean = df['pos_err_3d'].mean()
    ax.axhline(overall_mean*1000, color='purple',
               ls=':', lw=2, label=f'Overall mean = {overall_mean*1000:.1f}mm')

    labels = [f"x{int(r.tx*100)}\ny{int(r.ty*100)}\nz{int(r.tz*100)}"
              for _, r in sub.iterrows()]
    ax.set_xticks(x_pos)
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylabel('3-D position error to target (mm)', fontsize=12)
    ax.set_title(f'{arm.upper()} arm — sorted worst→best', fontsize=12)
    ax.grid(axis='y', alpha=0.3)
    ax.legend(fontsize=9)

    # Annotate each bar
    for i, (_, r) in enumerate(sub.iterrows()):
        ax.text(i, r.pos_err_mean*1000 + r.pos_err_std*1000 + 0.5,
                f"{r.pos_err_mean*1000:.1f}", ha='center', va='bottom',
                fontsize=7.5, fontweight='bold')

fig9.tight_layout()
fig9.savefig(FIGDIR / 'fig9_absolute_error_ranked.png', dpi=150, bbox_inches='tight')
plt.close(fig9)
print('Fig 9 done')

# ═══════════════════════════════════════════════════════════════════════════════
# FIGURE 10 — Z bias deep dive: actual Z vs target Z, all cases
#             Shows that robot systematically undershoots Z
# ═══════════════════════════════════════════════════════════════════════════════
fig10, axes = plt.subplots(1, 2, figsize=(14, 7))
fig10.suptitle('Z-Axis Systematic Undershoot — Actual Z vs Target Z\n'
               'Dashed = perfect accuracy', fontsize=13, fontweight='bold')

zs_target = sorted(df['tz'].unique())

for col, (arm, arm_df_full) in enumerate([('left', df[df['arm']=='left']),
                                           ('right', df[df['arm']=='right'])]):
    ax = axes[col]

    # All individual trials as scatter
    ax.scatter(arm_df_full['tz'], arm_df_full['ee_ss__z'],
               c=[trial_colors[i-1] for i in arm_df_full['trial_index']],
               s=30, alpha=0.6, zorder=4, label='Individual trials')

    # Per-case means with error bars
    for tz in zs_target:
        sub = arm_df_full[arm_df_full['tz']==tz]
        mean_z = sub['ee_ss__z'].mean()
        std_z = sub['ee_ss__z'].std()
        ax.errorbar(tz, mean_z, yerr=std_z, fmt='kD', ms=8,
                    capsize=5, lw=2, zorder=5)

    # Perfect line
    zmin, zmax = df['tz'].min()-0.005, df['tz'].max()+0.005
    ax.plot([zmin,zmax],[zmin,zmax],'k--',lw=1.5,label='Perfect (actual=target)')

    # Fit
    coeffs = np.polyfit(arm_df_full['tz'], arm_df_full['ee_ss__z'], 1)
    xfit = np.linspace(zmin, zmax, 50)
    ax.plot(xfit, np.polyval(coeffs, xfit), color=PALETTE[arm], lw=2,
            label=f'Linear fit: slope={coeffs[0]:.2f}, offset={coeffs[1]*100:.1f}cm')

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

fig10.tight_layout()
fig10.savefig(FIGDIR / 'fig10_z_undershoot.png', dpi=150, bbox_inches='tight')
plt.close(fig10)
print('Fig 10 done')

# ═══════════════════════════════════════════════════════════════════════════════
# Print key statistics summary
# ═══════════════════════════════════════════════════════════════════════════════
print('\n' + '='*60)
print('KEY STATISTICS SUMMARY')
print('='*60)
print(f'Total trials: {len(df)}')
print(f'Total cases: {df["case"].nunique()}')
print()
print('POSITION ERROR to target (true world-frame, all trials):')
print(f'  Mean 3-D: {df["pos_err_3d"].mean()*1000:.1f} mm')
print(f'  Worst case: {case_stats["pos_err_mean"].max()*1000:.1f} mm '
      f'({case_stats.loc[case_stats["pos_err_mean"].idxmax(),"case"]})')
print(f'  Best case:  {case_stats["pos_err_mean"].min()*1000:.1f} mm '
      f'({case_stats.loc[case_stats["pos_err_mean"].idxmin(),"case"]})')
print(f'  IK residual mean (NOT accuracy): {df["lin_err_steady_m"].mean()*1000:.1f} mm')
print()
print('Z SYSTEMATIC UNDERSHOOT:')
for arm in ['left','right']:
    sub = df[df['arm']==arm]
    undershoot = (sub['ee_ss__z'] - sub['tz']).mean()
    print(f'  {arm}: {undershoot*1000:.1f} mm (negative=undershoots)')
print()
print('REPEATABILITY (position spread across 3 trials):')
print(f'  Worst case: {spread_df["pos_spread_m"].max()*1000:.1f} mm '
      f'({spread_df.iloc[0]["case"]})')
print(f'  Mean spread: {spread_df["pos_spread_m"].mean()*1000:.1f} mm')
print()
print('ORIENTATION INSTABILITY (yaw std across trials):')
worst_yaw = spread_df.sort_values('yaw_spread', ascending=False).iloc[0]
print(f'  Worst yaw spread: {worst_yaw["yaw_spread"]:.3f} rad = '
      f'{np.degrees(worst_yaw["yaw_spread"]):.1f}° ({worst_yaw["case"]})')
print()
print(f'Plots saved to: {FIGDIR}')
