# Counter-Balance Iteration 3B

## Status

This document defines the next controlled experiments for the finite-horizon
counter-arm controller. Iteration 3B keeps the iteration-3 architecture and
tests two unresolved design choices:

- Whether a longer horizon improves acceleration and braking decisions.
- Whether arm momentum rate provides the missing reaction-torque objective for
  preventing ALMI steps.

Iteration 3B is not a whole-body or contact-aware MPC redesign. A controller
that adds base dynamics or contact state to the optimized state belongs in
iteration 4.

## Objective

The primary goal is to convert repeated ALMI `stumble` outcomes to `drift` or
`stable` while preserving the previously observed FAME improvements:

- Fall to drift or stable.
- Drift to stable.
- No stable-to-drift, stable-to-stumble, or stable-to-fall regressions.
- No stumble-to-fall regression.

The manipulation arm remains immutable. Only the four active proximal joints of
the counter arm may be optimized.

## Current Baseline

The current simplified controller uses:

| Item | Value |
|---|---:|
| Control period | `0.02 s` |
| Horizon | 3 intervals, `0.06 s` |
| FDDP iterations | 1 |
| State | Counter position and velocity, 8 values |
| Control | Counter acceleration, 4 values |
| Activation entry | Planar gyro magnitude `0.10 rad/s` |
| Full authority | Planar gyro magnitude `0.15 rad/s` |
| Moving-speed gate | `0.001 rad/s` |
| Moving-phase feedforward authority | `0.25` |
| Post-motion gyro feedback | Enabled |

The running objective contains normalized CoM velocity, angular momentum,
posture, acceleration, velocity, and near-limit terms. It does not optimize arm
momentum rate or predicted base response.

When inactive, the complete upstream arm sample passes through. The DDP solver
does not run.

The selected iteration-3 controller now has paired 44-target FAME and ALMI
panels. Baseline `B0` is:

- Three intervals and one FDDP iteration.
- `0.25` moving-phase feedforward authority.
- Scalar gyro feedback during motion and after motion completion.
- The existing iteration-3 objective and limits.

Its paired artifacts are:

- `runs/challenge_sweep/20260828_014310_iter3_candidate_vs_frame_fame`.
- `runs/challenge_sweep/20260828_021447_iter3_candidate_vs_frame_almi`.

`B0` FAME produced five improvements with no regression, including one
stumble-to-stable conversion, but no fall rescue. `B0` ALMI produced one
single-panel drift-to-stumble change on `left_fast_fall_search_09_scale_78`;
five fresh repetitions classified both frame task and `B0` as stumble in all
five trials. It is not a majority regression.

The later momentum-risk candidate is mechanism evidence, not `B0`: it rescued
one FAME fall in a full panel without a repeated ALMI regression, but failed the
repeated-rescue and real-time promotion gates.

Freeze `B0` with the current controller code and configuration hashes. Run five
repetitions for every stumble, improvement sentinel, and stochastic boundary
case. Run three repetitions for every remaining cell in the 44-target FAME and
ALMI focused panels. This defines majority severity and majority survival for
every later acceptance comparison.

## Evidence

### ALMI Stumbles

The ungated DDP panel retained five ALMI stumble outcomes:

`runs/challenge_sweep/20260827_041156_counter_ddp_almi_hard_groups_final`.

Relative to frame task, ungated DDP changed the peak metrics as follows:

| Case | Base drift delta | Foot displacement delta | Foot lift delta |
|---|---:|---:|---:|
| `left_arm_overhead` | `+0.012` | `-0.415 m` | `+0.003 m` |
| `left_fast_fall_search_09_scale_78` | `-0.013` | `-0.416 m` | `-0.022 m` |
| `left_fast_fall_search_11_scale_78` | `-0.002` | `-0.127 m` | `-0.001 m` |
| `left_manual_grasp_pitch_minus` | `-0.009` | `-0.455 m` | `-0.004 m` |
| `left_manual_grasp_pitch_plus` | `-0.019` | `-0.339 m` | `-0.005 m` |

The counter arm has useful authority: foot displacement decreased in all five
cases and base drift decreased in four. However, no stumble classification was
converted.

Ungated DDP already used substantial motion:

- Approximately `0.2` to `0.48 rad` counter displacement.
- Approximately `1.0` to `1.4 rad/s` counter velocity.
- Active solves through most of the target hold.

Therefore, more authority alone is not the next hypothesis.

### Stumble Timing

The manipulation trajectory lasts `1.5 s`. First measured foot lift occurs at:

| Family | Frame task | Ungated DDP |
|---|---:|---:|
| Manual grasp | `1.32–1.34 s` | `1.35–1.38 s` |
| Arm overhead | `1.44 s` | `1.59 s` |
| Boundary 09 | `2.36 s` | `2.33 s` |
| Boundary 11 | `2.21 s` | `2.50 s` |

Some steps begin during the final part of manipulation. Others begin well after
the moving arm stops. Activation tied only to moving-arm speed cannot address
the complete stumble panel.

### Retained Improvements

Historical FAME panels identified these important improvement sentinels:

- `left_fast_fall_search_06_scale_74`: fall to drift.
- `right_inner_upward_overhang_pitch_plus`: fall to drift.
- `left_fast_fall_search_09_scale_78`: drift to stable in the gated panel.
- `left_lateral_high_reach`: drift to stable.

Additional stochastic authority checks are:

- `right_fast_fall_search_11_scale_78`.
- `right_upward_arc_rank6`.

Iteration 3B must not gain ALMI performance by discarding these FAME benefits.

## Hypotheses

### H1: Three Steps Are Too Short

A three-step horizon sees only `60 ms`. It cannot represent a complete sequence
of acceleration, useful arm momentum, and braking.

A `100–160 ms` horizon may improve:

- Reaction-torque buildup.
- Counter-arm stopping distance.
- Excursion preservation.
- Braking timing near the end of manipulation.

A longer horizon alone is not expected to predict a foot step because the OCP
does not model base or contact dynamics.

Iteration-3 screening already rejected direct five- and eight-step activation
with the current frozen-map implementation: both lost FAME rescues and exceeded
the timing target. Iteration 3B must not repeat that screen unchanged. Longer
horizons are reconsidered only after nominal-trajectory linearization, complete
validation, and inactive pre-map gating are implemented.

### H2: Momentum Rate Is A Better Empirical Reaction Proxy

The whole-body centroidal map used by iteration 3 is expressed about the
whole-body CoM. The time derivative of its arm-column contribution is not an
isolated base torque: contact wrench, gravity, moving reference-point transport,
and lower-policy torques also contribute.

Iteration 3B therefore treats:

\[
p_{reaction}=-\dot H_{arm}
\]

as an empirical reaction proxy, not a conservation identity. It may correlate
with the arm-induced component of base angular acceleration during double
support. The frame, reference point, sign, gain, and delay must be identified
and validated before it receives objective weight.

The current objective shapes arm momentum `H`, not its rate. It can reduce total
motion without producing the useful reaction proxy at the time ALMI decides to
step.

Iteration 3B tests one momentum-rate residual without adding a learned base
model.

### H3: Recovery Must Follow Measured Response

Boundary stumble onset occurs after manipulation completion. Once an episode has
started, counter control must remain available while measured base response is
unsafe, even after moving-arm speed reaches zero.

Recovery must end from measured quietness, not a backend name or target-specific
timer.

## Scope

### Included

- Horizon ablation at fixed objective and solver settings.
- Momentum-rate diagnostic and one weighted momentum-rate residual.
- A minimal event-based recovery state.
- Contact transition as a safety gate when a reliable signal is available.
- Repeated paired simulation on compact development and guard panels.

### Excluded

- Base state in Crocoddyl.
- Learned base dynamics inside the OCP.
- Foot position or step suppression as an OCP cost.
- Whole-body or lower-policy optimization.
- More counter velocity or excursion as the primary experiment.
- Simultaneous tuning of horizon, momentum-rate weight, activation, and all
  existing objective weights.

## Experiment 1: Horizon Ablation

Keep the current objective, activation, acceleration limits, and one FDDP
iteration fixed.

| Candidate | Steps | Horizon |
|---|---:|---:|
| `H1` | 1 | `0.02 s` |
| `H3` | 3 | `0.06 s` |
| `H5` | 5 | `0.10 s` |
| `H8` | 8 | `0.16 s` |

Do not test ten steps initially. A `0.20 s` horizon is unlikely to be trustworthy
with frozen maps until five and eight steps pass timing and model-error gates.

### Required Preparation

Compute activation before constructing per-knot Pinocchio maps. Inactive ticks
must remain pass-through and nearly independent of horizon length.

Build frozen maps around the shifted warm-start state trajectory for every
candidate, including `H1` and `H3`. Horizon length must not be confounded with a
different linearization method. Reject states outside the configured trust
region.

Add `CounterDDPOCP.nominal_rollout(x0, lower, upper)`. It must:

1. Shift the previous accepted controls by one interval.
2. Repeat the final control for the new terminal interval.
3. Clip controls to current bounds.
4. Roll out arm dynamics from measured `x0` without requiring balance maps.
5. Return nominal states and controls before `_build_knots()`.

On reset or the first solve, use zero controls. `_build_knots()` linearizes each
endpoint around the returned nominal state. `solve()` consumes that exact seed;
it must not shift a second time internally.

Shadow benchmark every horizon first. Before active `H5` or `H8`, restore:

- Nonlinear metric validation at every knot.
- Momentum-rate-specific frozen-versus-nonlinear validation.
- Full-horizon physical position, velocity, excursion, and braking checks.
- Full-horizon collision validation.

If full validation misses the timing gate, that horizon is rejected for active
use. Do not disable validation to make a longer horizon fit.

### Horizon Selection

Select a candidate satisfying:

- Controller p99 at most `15 ms`.
- Pre-publication maximum below `20 ms`.
- No increase in invalid or rejected trajectory rate relative to three steps.
- No stable-to-worse guard regression.
- More repeated paired wins than losses against three steps.
- A multi-step candidate strictly outperforms `H1` before benefit is attributed
  to look-ahead.

If five and eight steps do not outperform three steps, retain three. Do not
select a horizon only because it is longer.

## Experiment 2: Momentum-Rate Diagnostic

Fix the selected horizon before changing the objective.

For action interval `k`, compute frozen-map arm momentum at both endpoints:

\[
H_k=A_{c,k}\dot q_{c,k}+A_{m,k}\dot q_{m,k},
\]

\[
H_{k+1}=A_{c,k+1}\dot q_{c,k+1}
+A_{m,k+1}\dot q_{m,k+1}.
\]

Then:

\[
\dot H_k\approx\frac{H_{k+1}-H_k}{\Delta t}.
\]

The initial feedback target is:

\[
\dot H_k^*=K_\theta(\theta-\theta^0)+K_\omega\omega_{xy}.
\]

This target is a hypothesis. Its sign and scale are not inferred from
conservation because the proxy is not isolated base torque. They must be
identified from measured command, arm momentum, contact, and base-response data.

### Required OCP Data Changes

`FrozenBalanceKnot` currently stores only one endpoint map. Before diagnostic or
weighted momentum rate, each action must receive:

- Sample-`k` and sample-`k + 1` counter momentum maps.
- Sample-`k` and sample-`k + 1` moving-arm momentum contributions.
- Sample-zero maps for the first action.
- Exact analytical derivatives of the frozen endpoint difference.

Dividing map error by `0.02 s` amplifies noise. Add a direct nonlinear
momentum-rate validator and a rate-specific tolerance; the existing momentum
mismatch tolerance is insufficient.

First log these values with zero objective weight:

- Predicted arm momentum rate.
- Measured arm momentum rate on the next sample.
- Base angular acceleration.
- Command-to-momentum delay and gain.
- Axis sign consistency.

Use low-authority pulse identification and existing active runs. Do not enable a
weighted residual until sign, delay, and scale are repeatable.

### Actuator Calibration Gate

The current OCP assumes commanded acceleration is realized immediately with unit
gain. The measured gain and delay cannot be logged and then ignored.

If measured delay is below half a control interval and gain error is within
`20%`, retain the current command-space state transition and hard bounds. Apply
the identified static gain only to the empirical command-to-measured
momentum-rate proxy. The OCP state remains a desired command trajectory, not a
claim of measured actuator acceleration.

Otherwise, add a first-order counter-actuator state before `M1`. This is an
arm-actuator calibration model and remains iteration 3B. A model predicting base
or contact transitions belongs in iteration 4.

The augmented state is:

\[
x_k=[q_{c,k},\dot q_{c,k},a_{real,k}]\in\mathbb{R}^{12}.
\]

Use:

\[
a_{real,k+1}=a_{real,k}
+\frac{\Delta t}{\tau_a}(G_a u_k-a_{real,k}),
\]

\[
\dot q_{c,k+1}=\dot q_{c,k}+\Delta t a_{real,k+1},
\]

\[
q_{c,k+1}=q_{c,k}+\Delta t\dot q_{c,k}
+\frac{1}{2}\Delta t^2a_{real,k+1}.
\]

Initialize `a_real` from a timestamped filtered difference of measured counter
velocity. If that estimate is stale or invalid, remain shadow-only; do not fall
back to an unlogged zero estimate for active output.

The publisher still realizes the command-space setpoint from `u_k`. First-step
hard bounds must check both the commanded next position/velocity and the
predicted measured next position/velocity from `a_real,k+1`. Warm starts include
the acceleration state, and reset clears its history.

## Experiment 3: Momentum-Rate Objective

Add one normalized residual:

\[
r_{\dot H,k}=
\frac{\dot H_k-\dot H_k^*}{s_{\dot H}}.
\]

The running cost becomes:

\[
\ell_k=\ell_{iteration\ 3,k}
+\frac{w_{\dot H}}{2}\|r_{\dot H,k}\|^2.
\]

Use one scale derived from observed momentum-rate distributions. Test only:

- `M0`: diagnostic only, `w_dot_H = 0`.
- `M1`: one low nonzero weight.
- `M2`: one medium weight only if `M1` has no guard regression.

Keep the existing momentum weight fixed through `M1`. If momentum and
momentum-rate costs visibly conflict, reduce the momentum weight in a separate
ablation after `M1`; do not change both together.

The active terminal model must not force counter velocity or momentum to zero.
That would recreate braking payback.

## Experiment 4: Minimal Recovery

Use five modes:

| Mode | Condition | Behavior |
|---|---|---|
| `PASS_THROUGH` | No manipulation episode. | Publish upstream arm command exactly. |
| `MONITOR` | Manipulation started but response has not crossed entry threshold. | Publish pass-through and monitor response. |
| `ACTIVE` | Measured response crosses entry threshold. | Run selected OCP objective. |
| `RECOVERY` | Moving arm stopped but response remains unsafe. | Continue OCP with momentum-rate feedback. |
| `HANDOFF` | Base response is quiet or recovery fault bound expires. | Acceleration-limit convergence to the upstream counter command. |

Add an explicit manipulation episode ID and completion event to the runtime call
boundary. `MONITOR` remains armed for at least `1.2 s` after moving-arm
completion, covering the observed maximum `1.0 s` delayed first-lift latency
plus margin. After that minimum interval, exit only when measured gyro and tilt
remain quiet for the complete dwell. It can transition directly to `RECOVERY`
if response first crosses threshold after the arm stops.

Transition to `HANDOFF` after gyro and tilt remain below exit thresholds for a
short quiet dwell. Use one symmetric threshold set, not axis-specific FAME/ALMI
values.

Include a `2.0 s` maximum post-motion monitor/recovery duration only as a fault
bound against indefinite activation. It enters `HANDOFF`; it never jumps
directly to `PASS_THROUGH`.

### Bounded Handoff

The active terminal objective intentionally does not force counter velocity or
momentum to zero. Recovery therefore cannot publish the upstream counter sample
immediately.

In `HANDOFF`:

- Use reduced velocity and acceleration limits.
- Track the current upstream counter position and velocity with slew-bounded
  commands.
- Preserve collision, excursion, and publisher checks.
- Require position and velocity to remain inside configured tolerances for a
  completion dwell.
- Return to `ACTIVE` if measured base divergence reappears before completion.
- Enter fault hold when a safe handoff trajectory cannot be produced.

Only transition to `PASS_THROUGH` after handoff completion. Contact transition
and the `2.0 s` recovery bound also enter bounded braking or `HANDOFF`, not direct
pass-through.

### Contact Safety

ALMI stepping is protective. The controller must not fight an initiated step.

Iteration 3B active recovery is simulation-only until an explicit timestamped
contact observation is added to the runtime and controller interface.

If a reliable foot-contact signal is available:

- Allow `ACTIVE` and `RECOVERY` in confident double support.
- Stop adding new counter acceleration when contact transition begins.
- Apply acceleration- and slew-bounded braking rather than an immediate
  zero-velocity hold or forced posture return.

Define contact age, confidence, invalid-sensor behavior, and transition
thresholds before active use. If reliable contact is unavailable to the real
controller, log contact only in simulation. Do not promote a simulator-only
contact dependency as a general controller feature.

## Development Panel

Freeze a family-level split before inspecting iteration-3B candidate outcomes.
Adjacent amplitudes and mirrored variants from one physical family remain in the
same partition.

### Primary ALMI Stumbles

Development:

- `left_arm_overhead`.
- `left_fast_fall_search_09_scale_78`.
- `left_fast_fall_search_11_scale_78`.

Held-out:

- `left_manual_grasp_pitch_plus`.
- `left_manual_grasp_pitch_minus`.

### FAME Improvement Sentinels

Development:

- `left_fast_fall_search_06_scale_74`.
- `left_fast_fall_search_09_scale_78`.
- `right_fast_fall_search_11_scale_78`.

Held-out:

- `right_inner_upward_overhang_pitch_plus`.
- `left_lateral_high_reach`.
- `right_upward_arc_rank6`.

### No-Regression Guards

Development:

- ALMI `right_fast_fall_search_06_scale_74`.
- ALMI `right_fast_fall_search_11_scale_78`.

Held-out:

- ALMI `right_lateral_overhead_reach`.
- FAME `right_diagonal_rank6`.
- Mirrored stable forward and cross-body targets on both policies.
- Standing with no manipulation.

## Trial Design

Use paired frame-task, current iteration-3, and candidate runs from identical
saved simulator state when available.

When saved-state branching is unavailable, use fixed simulator seeds, interleave
controller order by repetition, preserve identical release-relative timing, and
reject a pair when either member has an infrastructure gap or incomplete
execution. Store the seed and order in the manifest.

Use five repetitions for every primary stumble, FAME improvement sentinel, and
historically stochastic boundary case. Use at least three repetitions for stable
guards. Define case outcome by majority severity and report the complete
severity distribution.

The experiment order is:

1. Benchmark horizon timing without active simulation.
2. Run horizon candidates on the compact panel with the current objective.
3. Freeze the selected horizon.
4. Log momentum-rate diagnostics with zero weight.
5. Freeze sign, delay, and scale.
6. Test `M1` on development and guard panels.
7. Test `M2` only if `M1` passes guards but lacks sufficient authority.
8. Compare the selected objective with and without recovery on development
   cases; recovery is a separate ablation.
9. Freeze the complete candidate.
10. Unseal and run the held-out panel once.
11. Run the complete 44-target FAME and ALMI focused panels as regression
    coverage, not held-out selection evidence.

Do not use full panels to select parameters and then report the same panels as
held-out evidence.

## Acceptance Gates

### Primary ALMI Gate

- At least one held-out stumble case converts to majority `drift` or `stable`.
- Total majority stumble count is lower than both `B0` frame task and `B0`
  iteration 3.
- No stumble converts to fall.
- A converted case has at least `10%` lower median peak foot displacement and
  lift, unless the corresponding value is already below its classifier
  threshold.

### Retained Improvement Gate

- No held-out or development FAME sentinel has worse majority severity than its
  `B0` iteration-3 result.
- Every historical fall rescue has majority survival in at least four of five
  repetitions.
- FAME focused-panel majority survival is no worse than `B0` iteration 3.

### No-Regression Gate

- No majority severity regression of any kind, including drift-to-stumble and
  stumble-to-fall.
- No new collision, estop, runtime, or tracking failure.
- Moving-arm position, velocity, and torque remain pass-through to tolerance.

### Real-Time Gate

- Pre-publication controller p99 at most `15 ms`.
- Pre-publication maximum below `20 ms`.
- No active result is published after the timing guard.
- Solver-plus-validation rejection rate increases by at most one percentage
  point relative to `B0` iteration 3.
- Post-write publisher stalls are reported separately and invalidate timing
  evidence when they alter execution completeness; they are not described as
  solver deadline misses.

## Diagnostics

Add per-tick fields:

- Horizon steps and physical horizon duration.
- Predicted and measured arm momentum rate.
- Momentum-rate target and residual by axis.
- Estimated command-to-momentum delay and gain.
- Base angular acceleration.
- Activation mode and transition reason.
- Recovery duration and quiet-dwell progress.
- Contact confidence or simulation-only contact state.
- Time from release to foot-lift and displacement thresholds.
- Minimum braking and excursion margins.

Videos remain mandatory for every changed outcome, every stumble, and every
fall.

## Implementation Boundaries

Keep reusable momentum-rate math and optional arm-actuator calibration dynamics
in `counter_ddp_ocp.py`. Keep mode transitions, contact safety, publication, and
diagnostics in `counter_ddp_controller.py`. Add timestamped contact observation
plumbing in the benchmark runtime and controller call boundary for simulation
experiments.

Do not add a second solver, backend-specific branch, learned model, or generalized
constraint framework in iteration 3B.

## Iteration 4 Boundary

Iteration 3B ends if momentum-rate and recovery cannot produce repeated ALMI
stumble conversion without regression.

Arm-actuator gain, delay, and first-order realization dynamics remain iteration
3B because they calibrate the existing counter-arm control channel.

Name the next work iteration 4 if it adds any of:

- Base roll, pitch, or angular velocity to the optimized state.
- An identified arm-command-to-base-response transition model.
- Contact mode or support state inside the OCP.
- Whole-body or lower-body controls.

Those changes alter the prediction model and controller architecture. Horizon
and objective shaping within the existing counter-arm OCP remain iteration 3B.
