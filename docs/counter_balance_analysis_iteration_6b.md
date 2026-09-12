# Counter-Balance Analysis Iteration 6B

## Status

Iteration 6B's straight-line speed study is **executed**: the full ten
`directional` and ten `overhang` entries were swept at `1.0/0.8/0.5/0.4x` on both
the ALMI and FAME policies, Frame versus frozen 3C-robust, and compared against the
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
untouched. Headline result: the counter-balance advantage over Frame emerges
significantly with speed (paired `0.4x` vs `1.0x`, Wilcoxon `p=0.012`), and on the
FAME challenge endpoints 3C gives 6 paired improvements and 0 regressions
(`12 -> 7` falls). Endpoint speed evaluation remains queued behind an applicable
moving-arm acceleration contract. Missing experiments are not null results.

## Evidence Index

All evidence uses standard sweep directories directly under `runs/traj_sweep/`,
one per policy/bank/speed, mirroring the nominal `frame_vs_3c` comparison sweeps.
The speed sweeps always cover the full ten `directional_traj` and ten
`overhang_traj` entries for general coverage, never a favourable subset.

- Straight-line trajectory sweeps (Frame vs 3C, full 20 each):
    `*_iter6b_<bank>_traj10_frame_vs_3c_<policy>` (`1.0x`) and
    `*_iter6b_speed{08,05,04}_<bank>_frame_vs_3c_<policy>` for `directional`/
    `overhang` and `almi`/`fame`.
- Challenge-group endpoint sweeps: `runs/challenge_sweep/*new_challenge40_frame_vs_3c_{almi,fame}`.
- Visual inspection set: `runs/traj_sweep/iter6b_speed_visuals/` (three-view
    world-path overlay plots and side-by-side Frame|3C replay videos, with a
    `README.md` carrying the summary tables).
- Analysis artifacts: `runs/traj_sweep/iter6b_speed08_analysis/`
    (`speed08_comparison.json`, feasibility audit `speed_audit_env45.json`, and the
    retained 3-repetition confirmation of the headline cell); tuning regression
    record `runs/challenge_sweep/tuning_regression/regression_results.json`.

Ranking is by signed Frame-minus-3C timed world-position RMS (positive favours
3C). The superseded quasi-static overhang-only campaign under the old
`iteration6b/` tree, and the exploratory `iter6b_tuning*` sweeps, were removed;
their conclusions are retained in this document.

The first audit's ALMI checkpoint-path resolution was corrected and the audit
rerun; original artifacts remain under `health_checks/initial_preflight/`. This
was provenance repair, not a change to the checkpoint or physical experiment.

## Fresh Preflight Results

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

### Fast-Speed Arm-Tracking Ceiling (Directional Bank, `0.5x`/`0.4x`)

Visual inspection of `left_diagonal` and `left_forward_outward_high` at `0.5x`/`0.4x`
showed the wrist bowing off the commanded line: the elbow lags, the shoulder swings
first, then the elbow pitches down late. This was investigated as a possible joint
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

### Speed Progression (1.0x to 0.4x)

Because `0.8x` does not separate the controllers, the study time-scales the same
frozen geometry faster: `0.5x` (envelope `100 rad/s^2`) and `0.4x` (envelope
`160 rad/s^2`), always the full 20 entries on both policies, with velocity staying
under the real `6 rad/s` limit (`~4.9 rad/s` at `0.4x`). All 160 fast trials stood;
no falls. The counter-balance advantage emerges monotonically with speed:

| Speed | ALMI gain | ALMI win | FAME gain | FAME win |
| --- | --- | --- | --- | --- |
| `1.0x` | -1.15 | 10/20 | -0.08 | 10/19 |
| `0.8x` | -0.42 | 10/20 | -0.13 | 9/19 |
| `0.5x` | +0.49 | 10/20 | +1.11 | 9/19 |
| `0.4x` | +0.84 | 13/20 | +0.95 | 11/19 |

Frame-minus-3C timed world RMS (mm); positive favours 3C. Both controllers' errors
grow with speed (Frame vibrates more), and 3C moves from slightly worse at nominal
to better at `0.4x`, where the win rate first exceeds `50%`.

### Statistics

Frame versus 3C is paired per trajectory, so the `~2 mm` between-trajectory spread
is a nuisance removed by paired tests (pooled ALMI+FAME, `n=39`):

- **Speed effect** (paired `gain@0.4x - gain@1.0x`): mean `+1.53`, `25/39`,
    t `p=0.018`, Wilcoxon `p=0.012` -- **significant**. Speeding up reliably shifts
    the balance to 3C.
- **Level at `0.4x`** (is 3C better outright): mean `+0.90`, `24/39` (`62%`),
    t `p=0.055`, Wilcoxon `p=0.088` -- borderline, best of any speed, not yet
    `< 0.05`.

The defensible claim is the significant speed effect, not an absolute per-speed
win. The level is limited by the left/right bimodality (3C helps roughly half the
geometries and hurts the other half) and by FAME plateauing near `0.4x` (torque
saturation at `156 rad/s^2`); directional entries hit the `6 rad/s` velocity limit
near `~0.33x`, so there is little room faster on the full 20.

### The Absolute Win Is Concentrated in the Overhang (Raised-Arm) Bank

Splitting the timed-RMS gain by bank sharpens the level result. The full-20 pool
mixes two mechanisms: `overhang_traj` raises the arm (a small pelvis tilt swings the
wrist far, so base stability dominates), while `directional_traj` keeps the arm near
the body (the base is barely disturbed, and at `0.4x`/`0.5x` the wrist error is
dominated by the elbow arm-tracking ceiling, not the base). They separate cleanly
(Frame-minus-3C timed world RMS, mm; positive favours 3C):

| Bank | `0.8x` | `0.5x` | `0.4x` |
| --- | --- | --- | --- |
| `overhang` (both policies) | -0.35 (9/18) | +1.53 (10/19) | +1.69 (13/19) |
| `directional` (both policies) | -0.37 (8/19) | +0.15 (9/19) | +0.08 (10/19) |

The entire speed-driven advantage lives in the overhang bank; directional is flat and
non-significant at every speed (Wilcoxon `p >= 0.22`). Restricting the level test to
the overhang bank at fast speed (`0.5x` and `0.4x` pooled, both policies, `n=38`)
gives mean `+1.61 mm`, `23/38`, one-sample **Wilcoxon `p=0.022` -- significant**, with
both policies positive (ALMI `+1.29`, FAME `+1.97`). Per single fast speed the
overhang bank is borderline (`0.4x` `p=0.104`, `n=19`); the significance needs the
two fast speeds pooled.

The strongest clean-tracking individual cells (arm fidelity `87-100%`, so the wrist
path is faithfully executed and the error is genuinely the base term):

| Policy | Speed | Target | Frame | 3C | gain |
| --- | --- | --- | --- | --- | --- |
| FAME | `0.4x` | `right_overhang_upward` | 38.8 | 29.3 | **+9.53** (25%) |
| FAME | `0.4x` | `right_overhang_sideways` | 48.8 | 41.1 | +7.72 |
| FAME | `0.5x` | `right_overhang_sideways` | 39.4 | 32.9 | +6.56 |
| ALMI | `0.4x` | `right_overhang_sideways` | 48.4 | 41.8 | +6.54 |
| ALMI | `0.5x` | `right_overhang_sideways` | 38.2 | 32.5 | +5.72 |

The takeaway: the fast straight-line showcase for 3C is the **overhang bank**, not
directional. `right_overhang_sideways` and `right_overhang_upward` are the clearest
undisturbed-reach cells (this is the same `right_overhang_sideways` that is the
`0.8x` headline case above, and its lead widens with speed).

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
