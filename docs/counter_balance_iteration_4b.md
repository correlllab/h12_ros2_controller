# Counter-Balance Iteration 4B

## Adaptive-Authority Counter-Balance Control

## Status

Iteration 4B continues from two frozen controllers:

- Iteration 3 B0: conservative acceleration-level Crocoddyl baseline.
- Iteration 3C: high-authority velocity-level Crocoddyl controller that closely
  reproduces Iteration 2 `momentum_wide`.

Iteration 4B does not revive the failed Iteration 4A full base-response model.
Iteration 4A remains documented in `counter_balance_iteration_4a.md` and
`counter_balance_analysis_iteration_4a.md`.

## Goal

Create one policy-blind controller that:

- Retains roughly two to three repeated FAME fall rescues from Iteration 3C.
- Reduces ordinary FAME and ALMI regressions toward B0 or better.
- Introduces no new majority fall or other severe regression.
- Preserves exact manipulation-arm behavior.

ALMI stumble rescue is desirable but optional. Iteration 4B must not trade stable
ALMI behavior for an unproven stumble intervention.

## Non-Goals

- No learned full base-response dynamics.
- No policy/backend classifier.
- No target-specific or arm-side-specific thresholds.
- No new counter-arm objective.
- No longer Crocoddyl horizon initially.
- No redesign of manipulation-arm position, velocity, torque, tracking, or
  publication behavior.

## Frozen High-Authority Core

Iteration 4B uses the Iteration 3C one-step velocity-level Crocoddyl problem:

\[
q_{c,k+1}=q_{c,k}+\Delta t\dot q_{c,k}.
\]

The cost retains the Iteration 2/3C terms:

- CoM-velocity cancellation.
- Angular-momentum cancellation and gyro feedback.
- Counter-posture return.
- Velocity damping.

The controller changes only the authority applied to the existing balance
terms. At adaptive authority one, the command is exactly Iteration 3C, including
its hold/fade lifecycle.

Let `beta_3C(t)` be the frozen Iteration 3C lifecycle scale. Use:

\[
\beta_{4B}(t)=\beta_{3C}(t)\alpha(t).
\]

Then solve through the existing `balance_scale` path:

\[
v_c(\alpha)
=
\operatorname{CrocoddylSolve}
\left(
J_{3C}(v_c;\alpha)
\right),
\qquad
0\le\alpha\le1.
\]

Adaptive authority cannot exceed, extend, or reactivate the frozen 3C lifecycle.
Once `beta_3C` reaches zero, Iteration 4B also reaches zero. Scheduler state
resets on episode, reference, or ownership reset.

Do not post-blend an unsafe command after optimization. Authority enters the
same objective/target scaling already used by Iteration 2 and 3C.

## Real-Compatible Signals

Authority uses only:

- Manipulation position and velocity preview.
- Pinocchio-derived moving-arm planar momentum and momentum rate.
- Settled-reference IMU roll/pitch and planar angular velocity.
- Positive tilt-rate divergence.
- Measured counter state and existing safety margins.

These are real-compatible: preview comes from the upstream command trajectory,
and measured quantities come from IMU, joint position/velocity/torque, and
Pinocchio. No MuJoCo contact force, exact base state, policy name, target ID, or
outcome label enters the controller.

Manipulation preview is the real robot's upstream commanded trajectory, not a
privileged future measurement. The runtime supplies an aligned `0.20 s` preview
of commanded arm position/velocity with monotonic timestamps. Sample zero equals
the currently published manipulation command.

Evaluate the moving-arm centroidal map at every preview configuration while
holding measured lower/counter configuration at the current sample. Compute
momentum rate by finite difference between adjacent preview momentum samples.

Reject nonfinite, misaligned, expired, or stale preview. Maximum preview age is
two control intervals. Invalid preview sets feedforward authority to zero and
records a diagnostic; response authority may still operate inside the frozen 3C
lifecycle.

## Authority Scheduler

Use two interpretable risk channels.

### Manipulation Preview Risk

\[
\rho_m
=
\max_{k\in preview}
\left\|
\begin{bmatrix}
H_{m,xy,k}/s_H \\
\dot H_{m,xy,k}/s_{\dot H}
\end{bmatrix}
\right\|
\]

This is feedforward. It can raise authority before the base begins to fall.

### Measured Response Risk

\[
\rho_b
=
\left\|
\begin{bmatrix}
e_{\theta}/s_{\theta} \\
\omega_{xy}/s_{\omega}
\end{bmatrix}
\right\|
+w_d
\left\|
\max(e_{\theta}\odot\omega_{xy},0)
\right\|.
\]

This is feedback and recovery evidence.

### Smooth Authority

Use monotonic smooth ramps identified from development distributions:

\[
\alpha_{ff}=S_m(\rho_m),
\qquad
\alpha_{fb}=S_b(\rho_b),
\]

\[
g_c=
\begin{cases}
1, & \text{preview disturbance is rising toward its peak},\\
S_c(\rho_b), & \text{after the predicted disturbance peak}.
\end{cases}
\]

\[
\alpha^*=\max(g_c\alpha_{ff},\alpha_{fb}).
\]

This preserves preventive feedforward before the disturbance peak. After the
peak, low measured response can attenuate false-positive preview authority, while
high measured response retains recovery authority.

Rate-limit authority:

\[
\alpha_{k+1}
=
\operatorname{clip}
\left(
\alpha^*,
\alpha_k-\Delta\alpha_{down},
\alpha_k+\Delta\alpha_{up}
\right).
\]

Use faster authority increase than decrease so high-risk FAME motion receives
early authority without abrupt recovery payback.

No policy-specific ramp, scale, threshold, or latch is allowed.

## Expected Behavior

### FAME Fall-Risk Motion

Manipulation preview predicts a large rising disturbance. `alpha_ff` approaches
one and the controller reproduces Iteration 3C early counter momentum.

Expected result: retain Iteration 3C fall-to-drift rescues.

### Ordinary FAME Motion

Preview may initially request authority, but low post-peak measured response
attenuates it. This reduces unnecessary sustained counter motion and
stable-to-drift risk.

Expected result: approach B0 stable/drift behavior.

### Stable ALMI Motion

The lower policy handles the disturbance with low measured response. Any early
preview pulse is attenuated after the disturbance peak, avoiding sustained 3C
stable-to-drift behavior.

Expected result: frame/B0-like stability.

### ALMI Stumble-Risk Motion

If measured response grows while still recoverable, feedback authority may
increase. No special stumble rule is added.

Expected result: no new fall; stumble improvement is a bonus.

## Implementation Structure

Add two focused modules:

```text
counter_balance/
    authority_scheduler.py
    counter_adaptive_velocity_controller.py
```

Recommended class:

```python
class CounterAdaptiveVelocityController(CounterDDPVelocityController):
    ...
```

Responsibilities:

- `authority_scheduler.py`: pure signal normalization, smooth ramps, slew, reset,
  and diagnostics.
- `counter_adaptive_velocity_controller.py`: preview/response extraction and
  application of authority to the frozen 3C control step.

Do not copy or fork the 3C objective/solver.

## Experiment Sequence

### Experiment 4B-A0: Shadow Risk Distributions

Hypothesis: preview risk identifies motions requiring early 3C authority, while
post-peak measured response distinguishes difficult recovery from ordinary FAME
and stable ALMI.

Mechanism:

- Run frame, B0, and 3C commands unchanged.
- Log `rho_m`, `rho_b`, disturbance trend/peak phase, confirmation factor,
  proposed authority, and event timing.
- Use FAME rescue, ordinary FAME, stable ALMI, and ALMI stumble families.

Decision:

- Keep only features with stable sign/scale across repetitions and mirrors.
- Reject features that merely identify policy rather than physical response.
- Freeze normalization and development/held-out family splits before active use.

### Experiment 4B-A1: Preview-Only Authority

Hypothesis: manipulation preview alone can preserve early FAME authority while
reducing ordinary-case counter motion.

Mechanism:

- Set `alpha = alpha_ff` during manipulation.
- Multiply it by the frozen 3C lifecycle scale.
- Change no objective, bound, excursion, or solver parameter.

Primary panel:

- FAME `right_fast_fall_search_09_scale_78`.
- FAME `right_fast_fall_search_11_scale_78`.
- FAME `right_fast_fall_search_06_scale_74` as a non-rescued high-risk sentinel.
- FAME ordinary stable/drift guards where 3C differs from B0.
- ALMI right-boundary stable/drift guards.
- ALMI ordinary lateral/overhang guards.

Use five paired repetitions for fall and stochastic boundary cases and at least
three for ordinary guards.

Decision:

- Retain preview scheduling only if at least two FAME rescues remain and no new
  severe regression appears.
- Require at least one repeated ordinary FAME/ALMI majority improvement relative
  to 3C, or a preregistered reduction in ordinary excursion/backtracking with
  unchanged severity.

### Experiment 4B-A2: Response Recovery

Run only if A1 retains FAME rescues but ordinary/recovery behavior remains worse
than B0.

Hypothesis: measured response can restore authority after manipulation without
raising early authority on ordinary ALMI.

Mechanism:

- Add post-peak response confirmation and `alpha_fb` through the same equation.
- Change no preview ramp or 3C objective simultaneously.

Decision:

- Retain only if it improves repeated recovery outcomes without stable-case
  regression.

### Experiment 4B-A3: Frozen Compact Candidate

Freeze scheduler parameters and compare:

1. Frame task.
2. B0.
3. Iteration 3C.
4. Iteration 4B.

Use the three FAME rescue targets, ordinary FAME guards, stable ALMI guards, and
ALMI stumble diagnostics.

Inspect every changed outcome, ALMI stumble, and fall video.

Freeze Iteration 4B only if the compact panel retains at least two FAME rescues,
has no new severe regression, and improves at least one ordinary majority case
relative to 3C. The target is to approach or match B0 ordinary-case totals.

### Full Evaluation

Only after compact gates pass:

1. Run the complete three hard groups with three matched repetitions per cell.
2. Report boundary, exploration, hard, and pooled totals.
3. Run one Iteration 4B-only 100-target regression per policy.

Do not rerun all historical controllers on the 100-target panels.

## Success Criteria

Primary:

- Retain at least two repeated FAME fall-to-survival conversions.
- Target three rescues; require at least the two currently repeated 3C rescues.
- No new majority fall or other severe regression.
- At least one ordinary FAME/ALMI majority outcome improves relative to 3C, with
  no ordinary outcome worse than 3C. The target is B0 or better.
- Preserve moving-arm tracking/pass-through.
- Pre-publication total-controller p99 below `15 ms`.

Secondary:

- Reduce counter excursion, clipping, or collision backtracking on ordinary
  cases.
- Improve ALMI stumble severity without increasing fall risk.

ALMI stumble rescue is not required for Iteration 4B promotion.

## Stop Conditions

Stop Iteration 4B when:

- FAME rescue requires full authority on ordinary ALMI/stable cases.
- Preview risk cannot separate rescue motions from ordinary guards.
- Adaptive authority produces target-family-specific thresholds.
- Two validated risk channels are insufficient and additional heuristics begin
  accumulating.
- Real-time or safety contracts regress.

Do not add a policy classifier or revive the failed full base-response model.

## Design Summary

Iteration 4B keeps the proven 3C Crocoddyl velocity controller and changes one
thing: how much of its authority is requested.

The scheduler uses manipulation preview for early preventive action and measured
IMU response for recovery. It is continuous, policy-blind, real-compatible, and
tested one mechanism at a time.
