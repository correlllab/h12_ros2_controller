# Counter-Balance Analysis Iteration 6B

## Status

Iteration 6B's rebuilt-bank straight-line speed study is **executed**: the full ten
`directional` and ten `overhang` entries were swept at `1.0/0.8/0.6/0.5/0.4/0.3x` on both
the ALMI-Manip-v2 and FAME policies, Frame versus frozen 3C-robust, and compared against the
existing `new_challenge` endpoint sweeps. The [protocol](counter_balance_iteration_6b.md)
freezes controllers and defines this primary straight-line study and the
still-queued endpoint-duration study.

The path here: the first-shortening rejection was diagnosed as a defect in the
override timing gate (it paired the geometry spline's endpoint derivative spikes
with the mid-move clock peak, over-stating acceleration by up to ~3800x), which was
replaced with an exact continuous timed-derivative check that changes no geometry,
controller, or metadata. The `25 rad/s^2` acceleration figure was then shown to be
a conservative bank-generation constant, not a runtime limit -- the runtime gates
velocity and torque, not joint acceleration -- so speed exploration raises it as an
explicit runtime override (`45` at `0.8x`, `100` at `0.5x`, `160 rad/s^2` at `0.4x`)
while `6 rad/s` velocity stays the binding real limit and the immutable bank is
untouched.

**Superseded timed-only headline (retained for comparison):** after rebuilding
the banks -- FAME `overhang_forward` start-pose reworked (largely fixed; 2 marginal
init cells remain, see below), and `directional` redesigned into extended-arm/
natural-wrist straight lines -- the full `1.0/0.8/0.6/0.5/0.4/0.3x` grid was swept
on both banks and policies (`97.9%` stable, `230/240` complete pairs). The counter-balance advantage
grows monotonically with speed and is now a **significant absolute-level win at fast
speed on both banks**: overhang fast-pooled `+1.62 mm`, Wilcoxon `p=0.003`;
directional fast-pooled `+0.82 mm`, `p=0.040`. At true nominal (`1.0x`) 3C is
net-negative (overhang `-2.13 mm`, `p=0.045`) -- the crossover as base disturbance
grows with speed. The redesigned directional bank now reveals the advantage (the old
near-body bank was flat, `p >= 0.22`). On the FAME challenge endpoints 3C also gives
6 paired improvements and 0 regressions (`12 -> 7` falls). Endpoint speed evaluation
remains queued behind an applicable moving-arm acceleration contract. Missing
experiments are not null results.

The [finalized spatial analysis](#finalized-spatial-analysis) below is the
authoritative current-bank metric reduction and supersedes the timed-only ranking,
the claim of monotonic improvement across the full speed grid, and the pooled-speed
significance claims in the historical sections. Segment Spatial RMS and Uniform
Spatial RMS are now the primary spatial-accuracy measures. Ideal / Timed RMS is
retained unchanged as the temporal-plus-spatial task metric. No simulation, run
repair, reclassification, trajectory redesign or threshold change was performed
for this metric finalization.

## Evidence Index

All evidence uses standard sweep directories directly under `runs/traj_sweep/`,
one per policy/bank/speed, mirroring the nominal `frame_vs_3c` comparison sweeps.
The speed sweeps always cover the full ten `directional_traj` and ten
`overhang_traj` entries for general coverage, never a favourable subset.

- Straight-line trajectory sweeps (Frame vs 3C, full 20 each), rebuilt-bank grid:
    `*_iter6b_speed{10,08,06,05,04,03}_<bank>_frame_vs_3c_<policy>` for
    `directional`/`overhang` and `almi`/`fame` (`speed10` = `1.0x` nominal). These are
    the canonical iter6b run set (dated `20260912`); the pre-rebuild overhang-only
    campaign (old banks, dated `20260910/11`) was superseded and archived, npz-stripped,
    to `runs/archive/traj_sweep_prerebuild_iter6b/`.
- Challenge-group endpoint sweeps: `runs/challenge_sweep/*new_challenge40_frame_vs_3c_{almi,fame}`.
- Visual inspection set: `runs/traj_sweep/iter6b_speed_visuals/` (three-view
    world-path overlay plots and side-by-side Frame|3C replay videos, with a
    `README.md` carrying the summary tables).
- Analysis artifacts: `runs/traj_sweep/iter6b_speed08_analysis/`
    (`speed08_comparison.json`, feasibility audit `speed_audit_env45.json`, and the
    retained 3-repetition confirmation of the headline cell); tuning regression
    record `runs/challenge_sweep/tuning_regression/regression_results.json`.

Current spatial ranking uses signed Frame-minus-3C Segment Spatial RMS and Uniform
Spatial RMS, with Ideal / Timed RMS alongside (positive favours 3C). Earlier sections
ranked timed world-position RMS alone. The superseded quasi-static overhang-only campaign under the old
`iteration6b/` tree, and the exploratory `iter6b_tuning*` sweeps, were removed;
their conclusions are retained in this document.

The first audit's ALMI checkpoint-path resolution was corrected and the audit
rerun; original artifacts remain under `health_checks/initial_preflight/`. This
was provenance repair, not a change to the checkpoint or physical experiment.

## Fresh Preflight Results

Historical evidence begins here. These preflight and pre-rebuild results explain
the campaign's development; they are not rows in the finalized current-bank tables.

| Check | Directional | Overhang | Total |
| --- | --- | --- | --- |
| Manifest entries | 10 | 10 | 20 |
| Full nominal sampled validation passed | 10 | 10 | 20 |
| Nominal validation exceptions | 0 | 0 | 0 |
| `0.8 * nominal` timing rejected | 10 | 10 | 20 |
| `0.65/0.5 * nominal` not attempted | 20 | 20 | 40 |
| New physical trials | 0 | 0 | 0 |

The nominal recheck used the existing offline Magpie model and
`validate_trajectory`, including dense execution and adaptive geometry checks,
position limits, velocity, acceleration, original orientation cone, saved-schedule
FK, self-collision, clearance, and initialized-start validation. All input hashes
were unchanged across the audit. Passing is sampled geometric validity, not
standing feasibility or continuous collision/FK certification.

All first-shortening exceptions were
`ValueError: duration override exceeds conservative derivative bounds`.

## Speed Validation Diagnosis And Fix

The blanket rejection was a defect in the override gate, not evidence of physical
infeasibility. The superseded gate bounded each joint's timed acceleration by
`(max_s |q_ss| * 1.875^2 + max_s |q_s| * 10/sqrt(3)) / T^2`, taking the spatial
maxima of the geometry derivatives and the temporal maxima of the quintic clock
independently and multiplying them. Those maxima do not co-occur: the geometry
spline's largest slope sits at the segment endpoints, exactly where the quintic
clock is stationary (`s' = s'' = 0`). For `left_diagonal` the elbow reaches
`|q_s| = 59.5` rad at `s = 1`, contributing zero timed velocity there, yet the
bound paired it with the mid-move clock peak `s' = 1.875`. The bound therefore
over-stated the exact timed acceleration by factors of 3x to ~3815x
(`left_diagonal` 3815x, `forward_outward_high` 1382x, most overhang entries
13-25x), so every `0.8x` request was rejected.

The gate was replaced with the exact continuous timed-derivative check. The timed
velocity `q_s(s(u)) s'(u) / T` and acceleration
`(q_ss(s(u)) s'(u)^2 + q_s(s(u)) s''(u)) / T^2` factor into duration-independent
shape terms scaled by `1/T` and `1/T^2`; because each geometry piece is cubic and
the clock is a fixed quintic, each shape term is an exact low-degree polynomial in
`u` whose continuous maximum comes from its per-piece stationary points, matching
dense sampling to `~1e-7`. A duration is admitted only when every joint's exact
velocity and acceleration stay within the retained effective limits. This is a
tighter mathematically valid check, not a sampled substitute, and it still rejects
genuinely infeasible durations. The bank geometry, nominal metadata and hashes,
controllers, and the `25 rad/s^2` / `6 rad/s` limits are unchanged.

`experiment/iteration6b_speed_audit.py` then re-validated each admitted factor
through the full sampled contract (dense and adaptive FK, self-collision,
clearance, orientation cone, velocity, acceleration) re-timed at the shortened
duration. Result (`health_checks/speed_audit.json`):

| Family (left and right) | Min feasible factor | Admitted factors |
| --- | --- | --- |
| `overhang_forward` | 0.383 | 0.8, 0.65, 0.5 |
| `overhang_inner_upward` | 0.36-0.44 | 0.8, 0.65, 0.5 |
| `overhang_upward` | 0.70 | 0.8 |
| `overhang_sideways` | 0.68-0.69 | 0.8 |
| `overhang_inner_forward` | 1.00 | none |
| all 10 directional | 0.91-1.00 | none |

The directional and `overhang_inner_forward` entries were stretched to their
acceleration ceiling at nominal generation, so even the exact acceleration exceeds
`25 rad/s^2` below roughly `0.9x`; their exclusion is a validation outcome, not a
physical fall. Eight overhang entries admit shortening, giving 16 shortened cells.

## Endpoint Dynamic Comparison (Context)

The endpoint runtime is dynamically harsher than this bank but is not its gate.
It moves each arm with a cubic Hermite blend from the measured release state:
`max|dq| = 1.5 |dq_target| / T`, `sup|ddq| = 6 |dq_target| / T^2`, with
discontinuous acceleration steps at the start and hold transitions and no
moving-arm acceleration gate at runtime (the publisher clips only position,
velocity at `3x` URDF then `6 rad/s`, and torque at `3x` effort). At nominal
`T = 1.5 s` the executed challenge runs realized only about `1.6 rad/s` and
`8-12 rad/s^2`. The `25 rad/s^2` figure is a conservative offline bank-generation
constant with no URDF/publisher/actuator provenance; the runtime gates no
moving-arm acceleration at all. For the straight-line speed study the envelope is
therefore raised as an explicit runtime override (`45 rad/s^2` at `0.8x`,
`100 rad/s^2` at `0.5x`) with velocity remaining the binding real limit -- see the
straight-line contract. That override is not carried into the endpoint path: the
endpoint cubic blend still lacks an applicable moving-arm acceleration gate, so the
accelerated `T <= 0.8 s` endpoint cells remain unrun and must not borrow the
straight-line envelope or the counter-controller constant.

## Runtime Propagation

The duration override (`--duration-factor`) and the acceleration envelope
(`--acceleration-envelope` / `arm_target.speed_acceleration_limit` /
`set_acceleration_limit`) flow into `move_duration` and the trajectory's runtime
acceleration limit, and are propagated through saved-line sampling, the line/hold
phase boundary, `line_end_tick`, total run length, controller coverage, the frozen
world reference re-timed at the executed duration, and the cell slug/resume
identity. A one-time exact-gate warm-up at setup prevents the first line command
from stalling. The saved-line summary re-derives the phase with the runtime's own
release-time formula (not the divided `line_end_tick`) so the fractional boundary
tick cannot be misread as a schedule error. Nominal execution, reference
registration, and the immutable bank metadata are unchanged, and the full test
suite passes.

## Dynamic Results (Full 20-Trajectory Speed Sweeps)

The evaluation sweeps all ten `directional_traj` and ten `overhang_traj` entries
at `0.8x` (envelope `45 rad/s^2`), `frame_task` versus frozen
`counter_ddp_velocity_robust`, on both the `mjlab_almi_manip_2` and FAME policies:
four standard sweeps, 80 trials, with nominal sweeps as the `1.0x` baseline. All
trials stood except one FAME overhang standing-initialization failure; there were
no falls and no infrastructure or phase failures. Comparison in
`iter6b_speed08_analysis/speed08_comparison.json`.

Aggregate timed world-position RMS at `0.8x` (mm):

| Policy | Frame mean ± sd | 3C mean ± sd | 3C gain mean ± sd | 3C better | best gain |
| --- | --- | --- | --- | --- | --- |
| ALMI | 17.16 ± 5.86 | 17.58 ± 6.16 | -0.42 ± 2.45 | 10/20 | +3.90 |
| FAME | 19.63 ± 6.96 | 19.76 ± 6.69 | -0.13 ± 2.08 | 9/19 | +3.71 |

At `0.8x`, **3C-robust is net-neutral versus Frame**: the mean gain (well under
`0.5 mm`) is dwarfed by the between-trajectory spread (`~2.5 mm`) and by the sign
flipping per geometry. 3C helps about half the trajectories (best about `+3.9 mm`)
and regresses on the others. The effect is direction-dependent, not a uniform
benefit; the best ALMI cell (`right_overhang_upward`, `+3.90`) is FAME's worst
(`-4.92`), so the only cell positive on both policies is `right_overhang_sideways`
(`+2.46` ALMI, `+1.93` FAME).

### Mechanism: The Difference Is Base Orientation, Not Arm Tracking

The world reference is locked once from the standing-pelvis SE(3) capture
(`p0 + R0 @ p_line(tau)`, `R0 @ R_line(tau)`; `register_reference` uses only that
capture, never a later measured pose). Both controllers command the identical
moving-arm joint trajectory, so decomposing the world error against the frozen-base
counterfactual (`frozen_pelvis_fk`) isolates the base contribution. For the
strongest ALMI case, `right_overhang_upward`:

| | world RMS | arm-only (base frozen) | base contribution |
| --- | --- | --- | --- |
| Frame | 18.06 mm | 14.51 mm | +3.55 mm |
| 3C-robust | 14.15 mm | 15.12 mm | -0.97 mm |

The arm-only tracking is nearly identical; the entire advantage is the base term.
The driver is pelvis **orientation (tilt)**, not translation: here 3C had *more*
base translation drift (`21` versus `9.5 mm`) yet *less* end-effector error,
because for a raised arm a small pelvis tilt swings the wrist far more than
translation does. 3C helps exactly when its counter-arm net-reduces the
tilt-induced wrist swing, and regresses when the counter-arm's own reaction tilts
the base more than it saves -- which is why the sign is geometry- and
direction-dependent.

### Bank Rebuilds (6B bank rebuild)

Two bank changes preceded the clean grid above:

- **Overhang `overhang_forward` fix.** On FAME the original entry started in an
    abducted pose (`sh_roll ~1.4`) that toppled the robot during the readiness hold.
    The direction was brought inward (`y 0.40 -> 0.30`) and only that left/right pair
    regenerated (the other 18 overhang entries are byte-identical); FAME now stands
    and executes it, giving a clean 10/10 overhang set.
- **Directional redesign.** The old near-body bank (`cross_body`, `forward`,
    `diagonal`, `forward_outward_high`, `upward_arc`) is retired to
    `directional_traj_legacy`. The new `directional_traj` is five extended-arm,
    shoulder-driven straight lines with the **wrist held at natural extension** (joint
    positions near zero, global orientation free to follow the arm -- minimising
    wrist-joint travel), `0.30 m` each, base durations unified to `1.5-1.73 s`. This
    both keeps them feasible fast (no elbow saturation, see the ceiling section below,
    which describes the *legacy* bank) and loads the base enough to expose 3C.
- **Target-catalog cleanup.** `new_challenge.yaml -> challenge_targets.yaml`; the
    per-scan `arm_*_targets.yaml` catalogs archived under `data/archive/`; a
    deduplicated `challenge_targets_legacy.yaml` records the old challenge definition.

### Fast-Speed Arm-Tracking Ceiling (Legacy Directional Bank, `0.5x`/`0.4x`)

This section documents the *legacy* near-body directional bank and motivated the
redesign above. Visual inspection of `left_diagonal` and `left_forward_outward_high`
at `0.5x`/`0.4x` showed the wrist bowing off the commanded line: the elbow lags, the
shoulder swings first, then the elbow pitches down late. This was investigated as a possible joint
limit masking the controller comparison. It is **not** a relaxable limit -- it is the
arm's own dynamic capability, and it is **identical for Frame and 3C**, so it adds a
controller-independent artifact rather than hiding a 3C advantage.

Per-joint completion of the commanded travel over the line window (left arm,
`left_diagonal`, ALMI; execution = `0.4x * 1.5s = 0.6s`) isolates the elbow:

| Speed | cmd elbow peak vel | elbow completion (Frame / 3C) |
| --- | --- | --- |
| `0.8x` | `2.44 rad/s` | `90% / 91%` |
| `0.5x` | `3.90 rad/s` | `67% / 65%` |
| `0.4x` | `4.88 rad/s` | `55% / 56%` |

Shoulders track `92-93%` at `0.4x`; only the elbow (largest excursion, `1.52 rad`,
highest commanded velocity) saturates, and it plateaus at a **`~2.2-2.5 rad/s`
velocity ceiling** regardless of speed. `left_cross_body` (small elbow motion,
`<1.4 rad/s`) tracks `95-99%` at every speed, so the ceiling is trajectory-specific,
driven by elbow excursion, not a global cap.

The ceiling is coupled multibody dynamics, not actuator authority or gain:

- **Not a publisher clip.** Published `q`/`dq` reach full travel (`published_q ==
    bank_desired_q`, peak `dq = 4.88 rad/s`); the command is faithful.
- **Not gain-limited.** Tripling the left-arm `gains.kp` (elbow `150 -> 450`, verified
    propagated to `received_command_kp`) left the motion unchanged (`55% -> 55%`,
    peak `dq 2.29 -> 2.35`, wrist deviation `11.1% -> 11.0%`).
- **Not a torque clip.** The higher gain raised the applied elbow torque from
    `133` to `334 Nm` with no change in acceleration -- the excess is absorbed by
    velocity-dependent reaction. During the velocity plateau (`0.35-0.55s`) roughly
    `100 Nm` net torque sustains `2.1 rad/s` at ~zero acceleration; the effective
    opposing load is `~42 Nm.s/rad`, of which joint damping (`10`) is only a fifth,
    the rest Coriolis/centrifugal plus floating-base reaction. Both scale with arm
    speed, so pushing harder fights the reaction rather than the trajectory.
- **Already over-powered.** The elbow's rated motor is `+/-18 Nm`
    (`actuatorfrcrange` in `h1_2_magpie.xml`); the unbounded sim torque motor applied
    `7-18x` that and still could not execute the line, so relaxing physical damping
    or armature to force it would be unphysical and would not lift the ceiling.

Consequently the wrist bow at `0.5x`/`0.4x` is an arm-tracking artifact, identical
across controllers, and on these **undisturbed** reaches there is no base-stability
signal beneath it to recover: base tilt is `~1.2-1.35 deg` and pelvis drift
`~3-6 mm` with Frame and 3C indistinguishable at every speed. The fastest directional
speed where the arm actually executes a near-straight line is `0.8x` (elbow `>=90%`).
The 3C advantage is exercised by the disturbance/impulse challenge sweeps below, not
by faster undisturbed reaching.

### Headline Case

`right_overhang_sideways` at `0.8x` is the most defensible improvement: positive
on both policies (`+2.46` ALMI, `+1.93` FAME) and independently confirmed over
three fresh interleaved repetitions (Frame `22.97 ± 0.27`, 3C `20.16 ± 0.27`,
gain `+2.81 mm`, about 12 percent) in
`iter6b_speed08_analysis/sideways_0p8x_confirmation_3reps.json`. Three-view
world-path overlays for this cell, the largest ALMI gain, and the largest
regression are in `iter6b_speed08_analysis/`.

### Speed Progression (1.0x to 0.3x) -- 6B Rebuilt Banks

The banks were rebuilt before this clean sweep (see *Bank Rebuilds*, above): the
`overhang` bank now stands cleanly on both policies at all 10 targets (the FAME
`overhang_forward` start-pose failure was fixed), and `directional` was redesigned
from near-body reaches into **extended-arm, shoulder-driven straight lines at natural
wrist extension** (`outward_horizontal`, `front_to_back`, `diagonal_outward_up`,
`push_sideway_forward`, `push_forward_up`; all `0.30 m`, `1.5-1.73 s` base). The
full `1.0/0.8/0.6/0.5/0.4/0.3x` grid was then swept on both banks and both policies
-- 24 sweeps, 480 trials, **470 stable / 8 infrastructure / 2 init** (`97.9%`
stable; `230/240` complete Frame-vs-3C pairs), with swap free and no orphaned
processes (the process-group teardown fix held). Every run produced `sim.npz` (no
hard crashes); the 8 `infrastructure` trials are scattered 3C DDP deadline
singletons (window truncated, execution-window metrics still valid), not the
swap-thrash contamination of the earlier v2 attempt. The 2 remaining `init` cells
are both `left_overhang_forward` under FAME (`1.0x` 3C, `0.6x` Frame): that
forward+overhead start pose sits at FAME's **standing-stability boundary** and the
robot tips during the pre-trajectory standing phase (base collapses to tilt `~pi`
at `t~19.7 s` -- an active fall, not a timeout), reliably across three retries, so
those two cells have `n=9`. The same pose stands fine under FAME at `0.8/0.4x` and
under ALMI everywhere; it is a marginal-pose physical limit, not an infrastructure
defect.

Frame-minus-3C timed world RMS (mm); positive favours 3C. Pooled over both policies:

| Speed | overhang gain (win) | directional gain (win) |
| --- | --- | --- |
| `1.0x` | -2.13 (6/17) | -0.50 (9/19) |
| `0.8x` | +0.07 (10/19) | -0.48 (9/19) |
| `0.5x` | +0.75 (12/19) | +0.55 (9/19) |
| `0.4x` | **+2.55 (12/18)** | **+1.07 (14/20)** |

The advantage grows **monotonically with speed on both banks**. At true nominal
(`1.0x`) 3C is net-negative -- significantly so on overhang (`-2.13`, `p=0.045`):
when the base is barely disturbed, the counter-arm's own hold activity adds error.
As speed rises the base disturbance grows and 3C crosses over, reaching its widest
lead at `0.4x`.

### Statistics

**Historical, superseded inference:** the pooled-speed tests below reuse the same
trajectory at different speeds without a justified independent sampling unit.
They are not accepted significance evidence for the finalized spatial comparison.
The new report keeps policy, bank, trajectory, speed and attempt explicit and
reports descriptive paired means and win counts without a new significance test.

Frame versus 3C is paired per trajectory; the between-trajectory spread is removed
by a one-sample Wilcoxon on the paired gains. The defensible headline is the
**absolute-level win at fast speed, now significant on both banks**:

- **Overhang, fast pooled** (`0.5x`+`0.4x`, both policies, `n=37`): mean `+1.62 mm`,
    `24/37`, Wilcoxon **`p=0.003`** -- significant; both policies positive.
- **Directional, fast pooled** (`n=39`): mean `+0.82 mm`, `23/39`, Wilcoxon
    **`p=0.040`** -- significant.
- **Per-speed pooled level**: overhang `0.4x` `+2.55`, `p=0.018`; directional `0.4x`
    `+1.07`, `p=0.036`. The `1.0x` overhang cell is significantly *negative*
    (`p=0.045`), which is the crossover story, not noise.

Both banks retain a left/right and policy bimodality (e.g. directional `0.4x` is
ALMI `+2.15`, `9/10`, `p=0.01` but FAME `~0`), so per-cell results vary; the pooled
fast tests are the robust claim.

### The Directional Redesign Now Reveals the Advantage (Both Banks Positive)

This is the key change from the earlier 6B result. The *old* directional bank was
flat and non-significant at every speed (`p >= 0.22`) because near-body reaches
barely disturb the base and their large elbow excursions saturated the arm-tracking
ceiling. The rebuilt directional bank -- extended arm, wrist at natural extension,
shoulder-driven -- both stays feasible fast (no elbow saturation) and actually
loads the base, so it now shows a **significant fast-speed 3C win** (`p=0.040`)
alongside overhang. Overhang remains the stronger showcase, but the advantage is no
longer overhang-only.

Strongest individual 3C wins (timed world RMS, mm):

| Policy | Speed | Bank | Target | Frame | 3C | gain |
| --- | --- | --- | --- | --- | --- | --- |
| FAME | `0.4x` | overhang | `right_overhang_upward` | 39.0 | 29.6 | **+9.39** (24%) |
| ALMI | `0.4x` | overhang | `right_overhang_sideways` | 48.6 | 39.8 | +8.79 |
| ALMI | `0.4x` | overhang | `left_overhang_sideways` | 54.1 | 46.6 | +7.59 |
| FAME | `0.8x` | overhang | `right_overhang_forward` | 21.4 | 14.0 | +7.37 |
| ALMI | `0.5x` | directional | `right_push_sideway_forward` | 42.3 | 35.9 | +6.37 |
| ALMI | `0.4x` | directional | `left_push_sideway_forward` | 55.4 | 50.4 | +4.99 |

Overhang `sideways`/`upward` dominate, but the redesigned directional
`push_sideway_forward` now contributes real wins -- confirming the redesign worked.

### Base-Isolated Metric (What 3C Actually Controls)

This historical decomposition remains a mechanism diagnostic, not the primary
spatial path reference. The finalized metrics use the desired segment registered
from the original stable-standing capture, never the base-fixed executed path.
Identical commanded joints do not guarantee identical measured arm tracking;
the diagnostic must not be treated as exact cancellation of all servo error.

`timed_position_rms` is dominated by the arm's own PD/gain tracking error, which is
**common to both controllers** (they command the identical arm path), so the 3C gain
reads as a small `2-7%` of the total. To isolate the part 3C can change, decompose the
measured wrist:

```
world_wrist = frozen_pelvis_fk (arm-only, pelvis held at the stable capture)
            + base-induced displacement (world - frozen_pelvis_fk)
```

`frozen_pelvis_fk` uses the trial's actual measured arm joints with the pelvis frozen,
so `|world - frozen_pelvis_fk|` is purely the pelvis motion's effect on the wrist --
the only thing counter-balancing acts on. Everything below is measured over the
**trajectory-execution window only** (line start to line start + execution duration;
**not** the 10 s hold, which is the separate `hold_position_rms` where 3C's counter-arm
settling costs it). Reported **separately per policy** because the two emphasise
opposite parts of the one base disturbance (FAME trades tilt down / translation up;
ALMI cuts translation) -- see "How to read these tables" below. Cells are
`Frame->3C (%↓)`, positive % favours 3C:

- **Ideal RMS** (`timed_position_rms`): measured wrist vs the *ideal* planned straight
    line. The task metric; contains arm-tracking error (common to both) plus base effect.
- **Execution RMS** (`|world - frozen_pelvis_fk|`): measured wrist vs the *base-fixed
    executed* arm path -- i.e. deviation from what the arm actually achieves with the
    pelvis held still. This isolates the base-induced error, the part 3C controls. (Read
    it as "deviation from the base-fixed execution reference", not "total execution error".)
- **max tilt** = peak pelvis tilt; **pelvis drift** = peak pelvis translation.

#### FAME -- 3C reduces pelvis tilt (trades some translation)

| Bank | Speed | Ideal RMS (F->3C) | win | Execution RMS | max tilt (deg) | pelvis drift |
| --- | --- | --- | --- | --- | --- | --- |
| overhang | 1.0x | 15.7->17.7 (-13%) | 3/9 | 10.5->12.7 (-21%) | 2.82->2.76 (+2%) | 15.6->19.4 (-25%) |
| overhang | 0.8x | 17.4->16.9 (+3%) | 4/9 | 8.8->9.2 (-4%) | 2.52->2.36 (+6%) | 14.2->17.0 (-20%) |
| overhang | 0.6x | 21.9->20.9 (+4%) | 5/9 | 6.8->7.3 (-9%) | 2.24->2.17 (+3%) | 11.1->14.1 (-26%) |
| overhang | 0.5x | 25.9->25.2 (+3%) | 7/10 | 5.1->5.2 (-2%) | 2.24->2.16 (+3%) | 9.5->11.3 (-19%) |
| overhang | 0.4x | 34.4->31.3 (**+9%**) | 5/8 | 5.6->5.0 (+11%) | 2.16->2.12 (+2%) | 8.6->10.0 (-16%) |
| overhang | 0.3x | 45.1->43.5 (+4%) | 6/10 | 3.6->3.9 (-9%) | 2.25->2.05 (+9%) | 5.6->6.6 (-19%) |
| directional | 1.0x | 20.2->21.1 (-4%) | 5/10 | 10.9->11.2 (-2%) | 2.77->2.59 (+6%) | 16.1->15.2 (+5%) |
| directional | 0.8x | 21.8->22.7 (-4%) | 3/10 | 8.7->9.4 (-7%) | 2.66->2.35 (**+12%**) | 14.1->13.3 (+6%) |
| directional | 0.6x | 28.8->29.4 (-2%) | 4/10 | 7.6->8.5 (-11%) | 2.46->2.17 (**+12%**) | 11.6->11.6 (+1%) |
| directional | 0.5x | 35.8->35.7 (+0%) | 4/9 | 7.0->6.9 (+1%) | 2.37->2.03 (**+14%**) | 9.7->9.9 (-2%) |
| directional | 0.4x | 47.4->47.4 (-0%) | 5/10 | 7.5->7.9 (-5%) | 2.18->2.01 (+8%) | 8.4->9.2 (-9%) |
| directional | 0.3x | 66.0->65.8 (+0%) | 4/10 | 8.4->7.2 (+14%) | 2.05->1.94 (+5%) | 6.8->7.1 (-4%) |

FAME 3C **reduces peak pelvis tilt at every speed** (`+5-14%`), most strongly on
directional, but **adds pelvis translation drift** (overhang `-16..-26%`). Execution
RMS is the net of the two so it reads mixed. Because tilt is what swings a
raised/extended arm off the line, the tilt reduction is what carries FAME's Ideal
RMS gain (clearest on fast overhang, `+9%`). This is the same mechanism as the FAME
challenge-group fall-prevention.

#### ALMI -- 3C reduces base motion (translation-dominated)

| Bank | Speed | Ideal RMS (F->3C) | win | Execution RMS | max tilt (deg) | pelvis drift |
| --- | --- | --- | --- | --- | --- | --- |
| overhang | 1.0x | 15.1->17.3 (-15%) | 3/8 | 8.4->9.9 (-18%) | 1.46->1.57 (-8%) | 10.0->11.6 (-17%) |
| overhang | 0.8x | 19.2->19.5 (-1%) | 6/10 | 7.4->8.2 (-11%) | 1.36->1.41 (-4%) | 8.7->9.8 (-12%) |
| overhang | 0.6x | 24.4->23.3 (+5%) | 7/9 | 6.7->6.7 (-1%) | 1.19->1.24 (-4%) | 7.4->7.9 (-7%) |
| overhang | 0.5x | 27.8->27.0 (+3%) | 5/9 | 6.6->6.8 (-3%) | 1.12->1.16 (-4%) | 7.4->7.2 (+3%) |
| overhang | 0.4x | 36.0->34.0 (**+6%**) | 7/10 | 6.7->5.6 (+16%) | 1.08->1.08 (-0%) | 7.2->6.0 (+16%) |
| overhang | 0.3x | 45.6->43.9 (+4%) | 7/9 | 7.0->6.2 (+11%) | 1.06->1.13 (-6%) | 7.3->6.4 (+12%) |
| directional | 1.0x | 18.1->18.2 (-0%) | 4/9 | 10.9->12.4 (-14%) | 1.21->1.20 (+0%) | 12.8->14.2 (-10%) |
| directional | 0.8x | 22.7->22.6 (+0%) | 6/9 | 9.2->10.5 (-14%) | 1.14->1.14 (-1%) | 10.9->12.7 (-16%) |
| directional | 0.6x | 30.4->29.4 (+3%) | 5/10 | 7.7->7.8 (-1%) | 1.16->1.15 (+0%) | 8.5->9.1 (-7%) |
| directional | 0.5x | 37.6->36.6 (+3%) | 5/10 | 8.0->7.0 (+13%) | 1.19->1.16 (+3%) | 7.9->7.9 (-0%) |
| directional | 0.4x | 50.3->48.1 (**+4%**) | 9/10 | 9.2->7.1 (**+23%**) | 1.25->1.16 (+7%) | 7.7->6.8 (+11%) |
| directional | 0.3x | 69.1->67.6 (+2%) | 7/10 | 11.1->8.3 (**+25%**) | 1.33->1.21 (+9%) | 8.0->6.2 (+23%) |

ALMI's tilt is small (`~1.1-1.5 deg`) and roughly neutral; ALMI 3C's benefit is
**cutting Execution RMS / pelvis drift at fast speed** (crossover: worse below `0.6x`,
`+15-25%` better at `0.4x`/`0.3x`).

**How to read these tables:**

- **One base disturbance, two coupled parts.** The robot is a floating-base humanoid
    (the pelvis is a free joint, `qpos[0:7]`); it stands on foot contact and never steps,
    but the pelvis body itself does move in world. There is a *single* SE3 base
    disturbance -- the pelvis deviating from its captured pose `(R0, p0)` -- with a
    **rotational part** (tilt = `norm(roll, pitch)` of the pelvis quaternion, yaw
    excluded) and a **translational part** (`pelvis_world_position`, i.e. MuJoCo
    `data.xpos[pelvis]`, the pelvis-origin translation, measured directly -- not derived
    from tilt). Both are logged independently.
- **Tilt and translation are coupled, not identical.** Because the base pivots roughly
    about the feet, most pelvis translation is tilt re-expressed through a lever:
    empirically `corr(horiz drift, tilt) ~ 0.6-0.9` on fast overhang. But the coupling
    is not a rigid single pivot -- the *effective* pivot sits only `~0.2-0.4 m` below the
    pelvis (a pure ankle pivot ~0.9 m would give `~2x` more horizontal drift than
    observed), and there is genuine `6-12 mm` vertical drift and `~0.5-0.9 deg` yaw, both
    invisible to `tilt`. So translation carries real information beyond tilt: it is the
    ankle+hip balancing strategy and foot compliance, not tilt alone.
- **The two are independently controllable -- that is why we split the policies.** FAME
    3C *reduces tilt* (`+5-14%`) while *increasing* translation (overhang `-16..-26%`);
    if tilt and translation were one rigid rotation you could not push them in opposite
    directions, so FAME is genuinely trading tilt for a more hip/translation strategy.
    ALMI 3C instead leaves tilt flat and cuts translation. FAME attacks the rotational
    part; ALMI attacks the translational part. (Pooling the policies -- an earlier
    version of this section -- averaged the two opposing trades and made Execution RMS
    look flat.) These are two emphases of one controller on one disturbance, not two
    separate physical mechanisms.
- **Execution RMS lumps both parts (no double-count).** Execution RMS
    (`|world - frozen_pelvis_fk|`) decomposes exactly as `(p(t) - p0)` (pelvis
    translation) `+ (R(t) - R0) . r_arm` (tilt+yaw through the arm lever), so it is the
    *total* base-induced wrist error. `pelvis drift` is the translation term only (no arm
    lever); `max tilt` is the rotation magnitude only. 3C can therefore cut tilt while
    Execution RMS worsens (FAME `0.6-0.8x`) -- it traded tilt for translation, not a
    contradiction.
- **Window matters.** Over the full line+hold window 3C's base motion is larger
    (counter-arm settling) -- the real, separate `hold_position_rms` cost below.
- **Magnitude != task gain.** Execution RMS is a vector deviation; its magnitude does
    not map 1:1 to line error. On overhang a small tilt reduction swings the long
    raised-arm lever far *perpendicular* to the line, coupling into Ideal RMS at ~2.8x
    vs ~1.2x on directional -- which is why overhang leads the task metric even where
    directional shows a larger raw Execution RMS %. Headline Ideal RMS
    (`timed_position_rms`, the task metric); use these as the mechanism diagnostic.

### Accuracy Metrics

Reconstructed per run over the line window and the 10 s hold (Frame / 3C mean, mm;
`cum` is the time-integral in mm*s):

| Metric | ALMI 1.0x | ALMI 0.5x | ALMI 0.4x | FAME 1.0x | FAME 0.5x | FAME 0.4x |
| --- | --- | --- | --- | --- | --- | --- |
| traj mean | 12.8/13.8 | 25.4/25.0 | 31.1/30.5 | 16.2/15.6 | 26.4/25.6 | 32.4/31.4 |
| traj RMS | 14.2/15.4 | 29.8/29.3 | 36.8/36.0 | 18.6/18.1 | 31.5/30.4 | 38.5/37.4 |
| traj cum | 20.7/22.3 | 20.2/20.0 | 19.8/19.5 | 28.3/27.4 | 21.8/21.0 | 21.1/20.5 |
| endpoint | 15.0/18.1 | 38.3/38.7 | 47.8/47.0 | 30.4/30.8 | 46.4/46.1 | 53.0/53.1 |
| hold RMS | 15.8/15.9 | 16.4/17.1 | 17.7/18.2 | 32.1/31.2 | 33.3/32.6 | 32.4/33.5 |
| traj+hold RMS | 15.7/16.0 | 18.0/18.5 | 19.6/20.0 | 30.7/29.9 | 33.3/32.6 | 32.9/33.9 |
| traj+hold cum | 178/179 | 175/179 | 180/182 | 348/337 | 349/339 | 334/343 |

On **FAME 3C wins nearly every trajectory-accuracy metric at nearly every speed**;
on ALMI it wins only the line metrics at fast speeds and is marginally worse on the
hold (ALMI stands so still that the counter arm's continued hold activity adds a
little error). This mirrors the challenge-group result below: FAME is the solid
case.

## Challenge-Group Comparison (Endpoint Task, Strongest Evidence)

The existing `new_challenge` endpoint sweeps (40 targets, cubic blend, Frame versus
frozen 3C, `runs/challenge_sweep/*new_challenge40_frame_vs_3c_{almi,fame}`) apply a
much larger, faster base disturbance than the saved lines, and this is where 3C's
benefit is clearest and safety-relevant.

Physical classification counts:

| Policy / Controller | stable | drift | fall |
| --- | --- | --- | --- |
| ALMI / Frame | 37 | 3 | 0 |
| ALMI / 3C | 38 | 2 | 0 |
| FAME / Frame | 24 | 4 | 12 |
| FAME / 3C | 26 | 7 | 7 |

Paired Frame->3C transition on the same endpoint (the decisive view):

- **FAME: 6 improvements, 0 regressions** -- `fall->drift` x4, `fall->stable` x1,
    `drift->stable` x1; no `stable->drift`, `drift->fall`, or `stable->fall`. 3C
    strictly improves or holds every endpoint, cutting Frame's falls `12 -> 7` and
    reducing base drift on `31/40`.
- **ALMI: 1 improvement (`drift->stable`), 0 regressions**, everything else
    unchanged; both controllers keep all 40 up, so there is little to rescue.

The "six up, zero down" paired FAME result, combined with the significant
speed-effect on the saved lines, is the solid dynamic-envelope evidence for the
counter-balance controller. ALMI is intrinsically so stable that its advantage is
marginal; a matched endpoint speed-up study on ALMI (see Remaining Work) is the
natural way to expose disturbance there.

## Endpoint Gate

The three existing endpoint catalogs retain 44 named IDs and 40 unique geometries.
Saved endpoint success/limit/collision flags and corrected nominal ALMI stability
do not certify shortened cubic paths from the measured release pose. Current
configuration validation lacks a duration-dependent full moving-arm acceleration
contract. The counter-controller's acceleration parameter and the straight-line
bank's 25 rad/s-squared limit must not be silently repurposed as one.

Consequently, no `1.25/1.0/0.8/0.6 s` endpoint cells were launched, and no nominal
endpoint reruns were presented as a dynamic failure search. A physical boundary,
an H2-specific benefit, and repeated practical equivalence remain untested.

The current SciPy-primary robust implementation freeze establishes numerical
reliability while retaining the unresolved ALMI collision/hold-entry limitation.
That limitation must remain visible in a future dynamic panel: zero nominal
solver failures does not show useful counter-arm authority on every tick.

## Controller Tuning Exploration

At explicit user request (a departure from the 6B freeze), 3C-robust was tuned by
parameters only to try to improve tracking. Mechanism: 3C reads the
`reactive_counter_balance` block (not `counter_ddp`), already runs the counter arm
at full authority during the move (no throttle), and its objective effectively
weights CoM-centering `~50x` above the momentum (tilt) term (row scales enter
squared: CoM `~100`, momentum `~2`). The one config-only lever is to raise
`weights.momentum` and `gains.gyro`, run as overlay controller ids that write only
`reactive_counter_balance`, leaving the frozen base config untouched.

Screened at `0.5x` ALMI over all 20 trajectories, no variant improved tracking
beyond single-run noise: gain deltas versus baseline were `+0.05` (gyro 0.5),
`+0.13` (momentum 8), `0.00` (combo), and negative for the aggressive ones
(`-0.37` momentum 20, `-0.06` momentum 40, `-0.25` strong), all against a `~2 mm`
spread. Worse, the endpoint regression against the challenge rescues showed that
**any tilt-weight increase costs a fall rescue**: baseline 3C rescues all five
FAME falls, `momentum=8` breaks one (`right_extended_up_rear_03`) and `momentum=40`
breaks two, and neither improves the ALMI base-drift improvement endpoints.
Conclusion: CoM-centering and tilt-cancellation are coupled through the same
4-DOF, excursion-limited counter arm, so no config-only point tracks better without
sacrificing rescue. **Keep the frozen baseline 3C**; a real gain needs architecture
(feedforward tilt compensation from the planned trajectory, an explicit
pelvis-orientation term, or more counter DOF). Evidence: the `iter6b_tuning*`
sweeps were pruned after this negative result; the challenge regression record is
`runs/challenge_sweep/tuning_regression/regression_results.json`.

## Method Selection

1. **Does 3C improve dynamic manipulation tracking over Frame?** On the saved
    lines, only marginally and only with speed: 3C is net-neutral at `1.0-0.8x`
    and moves positive by `0.4x` (gain `+0.84` ALMI, `+0.95` FAME; win `13/20`,
    `11/19`). The absolute level is not significant (pooled `0.4x` t `p=0.055`), but
    the **speed effect is significant** (paired `0.4x` vs `1.0x`, Wilcoxon
    `p=0.012`): faster execution reliably shifts the balance to 3C. The clearest,
    safety-relevant benefit is on the **FAME challenge endpoints**: `6` paired
    improvements and `0` regressions, cutting falls `12 -> 7`. On the very stable
    ALMI the benefit is marginal. Parameter tuning cannot amplify this without
    breaking rescues.
2. **Does H2 add repeatable benefit beyond 3C?** Undetermined by 6B. No challenging
    repeated endpoint panel was executed. The absence of such evidence must not
    be reported as measured equivalence or a confirmed lack of benefit.
3. **Does this evidence support choosing 3C-robust as the final controller?** Not
    yet on the requested dynamic-envelope criterion. It remains the designated
    simpler primary method; a final comparative selection is deferred rather
    than inferred from timing exclusions.

## Remaining Work

Done in this iteration: the admissible speed-evaluation path (exact timed gate plus
configurable acceleration envelope, no bypass, bank immutable); duration propagation
with registration and metrics preserved and `723` tests passing; full 20-trajectory
Frame-versus-3C sweeps at `1.0/0.8/0.5/0.4x` on both policies; the paired speed and
level statistics; the accuracy-metric family (traj mean/RMS/cum, endpoint, hold,
traj+hold); the challenge-group classification and paired phase-change; a negative,
regression-guarded controller-tuning study; and three-view overlays plus
side-by-side replay videos under `runs/traj_sweep/iter6b_speed_visuals/`.

Still open:

**Active priorities (queued 2026-09-12, do the metric first, before any fix):**

- **[METRIC COMPLETE] Spatial metric finalization.** The complete current-bank
    reduction is in [Finalized Spatial Analysis](#finalized-spatial-analysis).
    Segment Spatial RMS is sample/dwell weighted, not timing-invariant. Uniform
    Spatial RMS removes that weighting by resampling chronological measured arc
    length. Both retain the original fixed desired segment and are accompanied by
    Ideal / Timed RMS, completion, endpoint, progress, path length and outcome.
    No base-fixed executed-path substitution or run repair was performed.
- **[OVERHANG RETUNE] Re-explore the `left_overhang_forward` target so the overhang
    group is fully executable.** That forward+overhead reach sits at FAME's
    standing-stability boundary: the robot reliably tips during the *pre-trajectory
    standing phase* (base collapses to tilt `~pi` at `t~19.7 s`, an active fall not a
    timeout), reproduced across three retries, leaving 2 init cells at `n=9` (`1.0x` 3C,
    `0.6x` Frame). The earlier start-pose rework helped but did not fully stabilise it.
    Re-run a targeted reachability/standing exploration for this one target (start-pose
    CoM offset, arm pre-load, or a slightly less forward endpoint) to find a pose that
    stands on FAME at all speeds without disturbing the other 9 overhang entries or
    changing the immutable bank semantics.
- **[INFRA FIX, DEFERRED] The 8 latest-attempt `infrastructure` trials are a 3C real-time-jitter artifact, not
    crashes -- recover them.** All 8 are the `counter_ddp_velocity_robust` (3C) variant
    (7 ALMI, 1 FAME). In every case the trajectory line executed fully
    (`line_complete=True`) and `sim.npz` holds the complete, gap-free (5 ms) line+10 s
    hold; several actually *pass* on the real metrics (e.g. `left_overhang_inner_forward`
    `1.0x`: timed 12.9 mm, hold 9.9 mm, endpoint 10.6 mm -- all inside tol). They are
    flagged only because `window_complete=False`: the online evaluator saw a `>50 ms` gap
    in the *live command/record stream* (a DDP compute spike / DDS delivery hiccup on the
    heavier 3C controller) that the regular physics clock never contains. No data is lost.
    Offline physics metrics have now been recovered without changing those flags.
    There are nine infrastructure attempts when the superseded attempt before a
    successful retry is retained. Physics coverage alone does not certify live
    command continuity, so the prior proposal to automatically reclassify from
    `sim.npz` is not an accepted repair. Investigation of timing/coverage and any
    proposed runtime change belongs to a separately authorized task; the 50 ms
    gate and all recorded classifications remain unchanged.

- **Matched endpoint speed-up on ALMI** (and FAME): ALMI stands so stably that the
    saved-line and nominal-endpoint tasks barely disturb it, so a duration-scaled
    challenge-endpoint sweep is the natural way to expose disturbance and test 3C
    there -- but it first needs a supported duration-dependent moving-arm
    acceleration contract for the cubic blend (the straight-line bank and
    counter-controller constants must not be reused).
- Add repetitions at `0.4x` to try to push the absolute level under `p<0.05`
    (single-run noise, not window or metric choice, is what keeps it at `p=0.055`).
    Note going *faster* than `0.4x` on the directional bank will not help: the elbow
    saturates at `~2.2-2.5 rad/s`, so beyond `0.4x` the wrist-error metric is
    dominated by a controller-independent arm-tracking artifact (see *Fast-Speed
    Arm-Tracking Ceiling*), not by base stability. `0.8x` is the fastest directional
    speed with a faithfully executed straight line.
- Optionally compare H2-robust against 3C-robust on the strongest cases.
- Capture higher-resolution rendered video if publication-quality figures are needed.

The progress record now reports a completed straight-line speed stage with a
real, bounded, direction-dependent result. No dynamic-envelope conclusion beyond
the measured evidence has been manufactured, and no final controller has been
selected.
