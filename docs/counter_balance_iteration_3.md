# Counter-Balance Iteration 3

## Status

This document is the implementation plan for a finite-horizon predictive
counter-balance controller. It does not promote a new controller and does not
replace the verified reactive implementation.

The recommended first implementation is a separate Crocoddyl candidate that:

- Preserves moving-arm frame control exactly.
- Optimizes only the four active joints of the counter arm.
- Uses counter-arm position and velocity as state.
- Uses counter-arm acceleration as control.
- Reuses the verified CoM-velocity and angular-momentum residuals at every knot.
- Solves a short, warm-started, receding-horizon problem at 50 Hz.
- Uses measured response and predicted moving-arm risk for symmetric activation.
- Holds the counter arm on every model, solver, or validation failure and logs
  every timing overrun.

The first implementation remains a fixed-base, arm-overlay controller. It is
not whole-body model predictive control and must not be described as such.

## Executive Decision

Do not wrap the current four-variable least-squares solve in a one-step
Crocoddyl problem. That architecture was already attempted and removed. It
would add solver overhead without adding temporal state, acceleration shaping,
braking awareness, or better activation.

Implement a new `CounterDDPController` beside the current
`CounterBalanceController`. Use `crocoddyl.SolverBoxFDDP` over an initial
`0.20 s` horizon with ten `0.02 s` intervals and at most two solver iterations.
The horizon length and iteration count are starting hypotheses, not promoted
constants. They must pass the real-time gate before active benchmark use.

The initial OCP should use per-knot frozen-map approximations of the verified
Pinocchio terms. Recompute the approximation once per control tick around the
shifted previous trajectory. This gives exact analytical derivatives for the
approximate model passed to Crocoddyl and avoids numerical differentiation in
the real-time loop. It is not a complete first-order linearization of the
nonlinear momentum and CoM-velocity products.

## Evidence Read For This Iteration

The design is based on the current controller, its historical variants, the
direct-ZMP Crocoddyl work, and the latest matched FAME and ALMI artifacts.

Controller documents:

- [counter_balance_plan.md](counter_balance_plan.md): current controller and
  evaluation contracts.
- [counter_balance_analysis.md](counter_balance_analysis.md): original matched
  all-candidate FAME and ALMI analysis.
- [counter_balance_iteration_1.md](counter_balance_iteration_1.md): ALMI
  observation mapping, sustained response, activation, and policy-interaction
  findings.
- [counter_balance_iteration_2.md](counter_balance_iteration_2.md): MAGPIE,
  confirmed-fall classification, moving-arm pass-through, and momentum-wide
  development.
- [zmp_balance_plan_new.md](zmp_balance_plan_new.md): existing Crocoddyl
  controller, analytical derivative timing, reaction identification, and
  braking-payback evidence.
- [legacy/zmp_balance_update_analysis.md](legacy/zmp_balance_update_analysis.md):
  distinction between arm momentum and momentum rate, finite-stroke limits,
  braking, and lower-policy interaction.

Decision-grade benchmark artifacts:

- FAME full panel:
  `runs/full_sweep/20260823_172208_fame_magpie_fast_full_reachability_momentum_wide`.
- ALMI full panel:
  `runs/full_sweep/20260825_164531_almi_magpie_fast_full_reachability_momentum_wide`.
- FAME confirmed boundary panel:
  `runs/challenge_sweep/20260821_141252_fame_magpie_fast_boundary_targets_confirmed`.
- FAME confirmed hard panel:
  `runs/challenge_sweep/20260821_145537_fame_magpie_fast_hard_targets_confirmed_final`.
- FAME confirmed exploration panel:
  `runs/challenge_sweep/20260821_151104_fame_magpie_fast_exploration_confirmed_final`.
- ALMI matched hard groups:
  `runs/challenge_sweep/20260825_155330_almi_magpie_fast_hard_groups_momentum_wide`.

### Latest Outcome Summary

The unique-scenario outcome counts are:

| Policy and panel | Frame task                             | Momentum wide                          | Interpretation                                                       |
| ---------------- | -------------------------------------- | -------------------------------------- | -------------------------------------------------------------------- |
| FAME full, 100   | 96 stable, 3 drift, 1 fall             | 94 stable, 5 drift, 1 fall             | One directional improvement and three ordinary-overhang regressions. |
| ALMI full, 100   | 98 stable, 2 drift                     | 96 stable, 4 drift                     | No reactive win and two regressions.                                 |
| FAME hard, 44    | 24 stable, 12 drift, 8 fall            | 27 stable, 13 drift, 4 fall            | Survival improves from 36 to 40 cases.                               |
| ALMI hard, 44    | 25 stable, 12 drift, 4 stumble, 3 fall | 22 stable, 13 drift, 6 stumble, 3 fall | Survival is unchanged and five paired classifications regress.       |

The FAME hard panel is the strongest positive evidence. Momentum wide rescues
four falls and improves median base-drift metrics. The full FAME panel and both
ALMI panels show that continuous reactive motion is not a generally safe
default.

The result is lower-policy dependent, but iteration 3 must not branch on the
names `FAME` or `ALMI`. A useful controller must infer whether to act from the
moving-arm forecast and measured robot response.

### Timing Evidence

The benchmark command stream runs at approximately 50 Hz:

- FAME median interval: approximately `20.078 ms`.
- ALMI median interval: approximately `20.068 ms`.
- Normal p99 interval: approximately `21.2` to `22.2 ms`.

Occasional second-scale gaps occur in both controllers at matching locations
and are infrastructure stalls, not demonstrated counter-controller cost. Future
promotion trials must invalidate timing evidence when a gap above `40 ms`
overlaps the release-relative `ACTIVE` or `RECOVERY_WAIT` window. A gap outside
that window does not automatically invalidate the physical outcome. Local
controller timing and external process stalls must be reported separately.

## Current Controller Baseline

The current implementation solves one bounded four-variable least-squares
problem per tick. Let `v` be the four active counter-joint velocities. The
verified residual targets are:

\[
b_c = \beta\left(-J_m\dot q_m-k_c e_c\right),
\]

\[
b_H = \beta\left(-A_m\dot q_m+k_\omega\omega_{xy}\right).
\]

The bounded solve minimizes normalized CoM, angular-momentum, posture, and
damping residuals. It then commands:

\[
q_{c,cmd}=q_{c,measured}+\Delta t\,v.
\]

The implementation has important strengths that iteration 3 must retain:

- The moving arm remains owned by `FrameController` or the explicit trajectory
  source.
- Only counter shoulder pitch, shoulder roll, shoulder yaw, and elbow move.
- Counter wrists stay at their captured posture.
- Position, velocity, excursion, collision, publisher, and estop checks remain
  authoritative.
- Counter-side failures do not block moving-arm publication.
- The solve is small, synchronous, and deterministic.

The finite-horizon design addresses these limitations:

- The solve has no temporal state or look-ahead.
- Velocity is the decision variable, so acceleration and braking are omitted.
- Arm momentum is shaped, but momentum-rate and braking payback are not planned.
- CoM and momentum maps are evaluated only at the current posture.
- Activation is a scalar tilt norm and can latch indefinitely.
- CoM, gyro, and command signals are not source-timestamped or filtered.
- The fixed-base model omits base translation, base orientation, and lower-body
  velocity from the prediction.
- State, limit, collision, and publisher diagnostics do not describe the full
  requested-to-applied command path.

### Interface Discrepancies To Resolve First

The normal inherited frame path integrates the IK velocity and then publishes
zero desired velocity. The counter overlay therefore receives zero moving-arm
velocity through `control_step()` and `control_step_reduced()`. The benchmark's
explicit `control_configuration_step()` path does provide the analytical moving
velocity, so the latest local benchmark evidence is not invalidated by this
defect.

The counter override also ignores the torque array supplied by inherited
steady-state integral control and recomputes gravity-only arm torque. This drops
the moving-arm integral torque bias.

Iteration 3 uses this explicit contract before physical comparison:

1. Route the pre-integration moving-arm IK velocity into predictive forecast
   data, but retain the inherited path's existing published `dq = 0` behavior.
2. Preserve the supplied moving-arm torque command exactly.
3. Compute gravity or hold torque only for the counter arm.
4. Add an explicit settled-reference capture call before manipulation release.
5. Record requested, solver-bounded, collision-backtracked, publisher-applied,
   and measured counter commands separately.

The benchmark's explicit trajectory path continues to publish its supplied
nonzero moving-arm velocity exactly. The inherited frame path uses its nonzero
IK velocity only for prediction and continues to publish zero desired velocity.
Changing inherited frame publication globally would change servo behavior and
would require refreezing every frame and reactive baseline; that change is not
part of iteration 3.

## Scope

### Goals

- Add useful look-ahead over moving-arm acceleration, motion, stopping, and
  counter-arm braking.
- Reuse the already evaluated CoM and angular-momentum metrics without adding an
  unverified balance estimator to the first active candidate.
- Preserve real-time receding-horizon behavior at 50 Hz.
- Reduce FAME hard-case falls without the ordinary-overhang regressions of
  momentum wide.
- Avoid the ALMI phase conflicts and delayed steps created by unnecessary or
  badly timed counter motion.
- Produce sufficient diagnostics to distinguish model error, activation error,
  solver error, timing failure, clipping, and lower-policy interaction.

### Non-Goals

- Do not optimize the moving arm, torso, legs, contacts, or lower-body policy.
- Do not claim whole-body MPC or contact-aware MPC.
- Do not introduce a backend-specific FAME or ALMI switch.
- Do not use the current contact-ZMP residual as a primary cost in iteration 3A.
- Do not force counter-arm momentum to zero at the horizon terminal knot.
- Do not return the counter arm on a fixed timer while measured response is
  still unsafe.
- Do not enable hardware output before source timestamps and contact-confidence
  gates exist.
- Do not use `ActionModelNumDiff` in the real-time loop.

## Controller Contract

The predictive controller retains the existing ownership and failure contract:

- The moving arm's position, velocity, and torque commands pass through without
  modification.
- The counter arm consists of four active proximal joints and three held wrist
  joints.
- The counter reference is captured from a settled pre-release state.
- Support-relative CoM reference is captured at the same event.
- Ownership remains fixed for one manipulation request.
- Every failure publishes the moving-arm sample and a zero-velocity counter
  hold after the synchronous computation returns.
- A previous plan may seed a new solve but may never be applied as a stale
  fallback command.
- The existing reactive controller remains runnable and unchanged as the
  iteration-2 comparison candidate.

## Proposed Package Structure

Keep the implementation controller-specific rather than adding another large
class to `core/robot_dynamics.py`:

```text
h12_ros2_controller/core/controller/counter_balance/
    __init__.py
    counter_balance_controller.py
    objective.py
    counter_ddp_controller.py
    counter_ddp_ocp.py
```

Responsibilities:

| File                       | Responsibility                                                                                                              |
| -------------------------- | --------------------------------------------------------------------------------------------------------------------------- |
| `counter_balance_controller.py` | Existing reactive baseline; no objective or behavior changes.                                                               |
| `objective.py`             | Existing single-step normalized least-squares functions.                                                                    |
| `counter_ddp_controller.py` | Ownership, reference capture, horizon input, activation state, command publication, validation, fallback, and diagnostics.  |
| `counter_ddp_ocp.py`        | Per-knot frozen-map data, Crocoddyl action model, Box-FDDP problem, warm start, solve result, and model-validation helpers. |

Add focused tests:

```text
test/test_counter_ddp_ocp.py
test/test_counter_ddp_controller.py
```

Do not remove the existing controller or rename its benchmark variants during
iteration 3.

### Benchmark Integration Points

The parent benchmark requires explicit support for two new runtime variants:
`counter_ddp_shadow` and `counter_ddp`.

Expected parent-repository changes are:

| File                                                 | Change                                                                                                                         |
| ---------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------ |
| `h12_zmp_benchmark/utility/config.py`                | Validate the new variants and predictive configuration mapping.                                                                |
| `h12_zmp_benchmark/runtime/arm_target_runtime.py`    | Construct the controller, generate timestamped `T + 1` trajectories, capture references, publish samples, and log diagnostics. |
| `h12_zmp_benchmark/utility/process_spec.py`          | Pass the predictive configuration and runtime variant to the arm process.                                                      |
| `h12_zmp_benchmark/bridge/arm_target_bridge.py`      | Add predictive runtime choices.                                                                                                |
| `h12_zmp_benchmark/utility/benchmark_run_summary.py` | Aggregate predictive timing, mode, model-error, and command metrics.                                                           |
| `h12_zmp_benchmark/sweep/arm_reachability_sweep.py`  | Permit predictive candidates without changing target or classifier semantics.                                                  |

Add parent tests for trajectory endpoint alignment, monotonic timestamps,
configuration resolution, command-plan serialization, summary aggregation, and
the shadow-versus-active publication distinction.

The `uv` environment already resolves Crocoddyl `3.2.1`. The ROS `setup.py` and
`package.xml` paths do not currently install or describe all numerical Python
dependencies. Iteration 3 simulation uses the `uv` environment. Packaging work
is required before ROS or hardware deployment and is not hidden inside the
controller implementation.

## Moving-Arm Horizon Interface

Finite-horizon control requires more than the current sample. Add a direct
interface with knot-aligned command samples:

```python
control_horizon_step(
    moving_q_horizon,
    moving_dq_horizon,
    moving_tau=None,
    sample_times=None,
    generated_at=None,
    authority_scale=1.0,
    forecast_source='explicit',
)
```

For `T` running intervals, the position and velocity arrays have shape
`(T + 1, 14)`. Sample zero is the exact moving-arm command published on the
current tick. Action `k` evaluates its interval-end residual with moving-arm
sample `k + 1`. Counter-side values in these arrays are ignored.

`sample_times` contains `T + 1` monotonic clock times. `generated_at` records the
monotonic generation time. The controller separately validates state-receipt
age, forecast age, first-sample command age, knot spacing, and final validity
time. Wall-clock timestamps must not be mixed with monotonic control times.

`moving_tau` is the sample-zero 14-joint arm torque command. When supplied, it
must be finite with shape `(14,)`, and the seven moving-arm values pass through
before publisher clipping. When it is `None`, compute gravity compensation once
from the current measured full-body state, matching the explicit benchmark frame
path. Log `supplied` or `gravity` as the torque source and report pre-clipping
pass-through error separately from publisher-applied torque error.

The interfaces are supplied as follows:

- Benchmark trajectories: generate exact future samples from the same analytic
  interpolation used for the current command.
- Named configuration and frame IK: pass the current pre-integration IK velocity
  and use a bounded constant-velocity forecast until a trajectory generator can
  provide a better horizon.
- Missing or invalid forecast: publish the moving sample and hold the counter
  arm; do not silently optimize against zeros.

The horizon source must be logged. Explicit and constant-velocity forecasts
must not be pooled as equivalent evidence.

## Finite-Horizon Formulation

### State And Control

Use the four active counter joints:

\[
x_k =
\begin{bmatrix}
q_{c,k} \\
\dot q_{c,k}
\end{bmatrix}
\in\mathbb{R}^8,
\qquad
u_k=\ddot q_{c,k}\in\mathbb{R}^4.
\]

Use constant acceleration over one interval:

\[
q_{c,k+1}
=q_{c,k}+\Delta t\dot q_{c,k}
+\frac{1}{2}\Delta t^2u_k,
\]

\[
\dot q_{c,k+1}
=\dot q_{c,k}+\Delta t u_k.
\]

The first candidate uses `StateVector(8)`. Do not add base state until a
source-timestamped base velocity estimate and a validated arm-to-base prediction
model exist.

### Per-Knot Full-Body Template

At each knot, construct a fixed-base motor configuration from:

- Current measured legs and torso, held over the short horizon.
- Exogenous moving-arm position for that knot.
- Predicted active counter-arm position.
- Captured counter-wrist position.

Evaluate the same full-body CoM Jacobian and angular centroidal map used by the
reactive controller. The model remains fixed-base, but the moving arm and
counter arm vary along the horizon.

### Frozen-Map Terms

For real-time derivatives, approximate around a nominal counter trajectory
`q_bar[k]`. For each knot, cache:

- `com_bar[k]`.
- `J_counter[k]` and `J_moving[k]`.
- `A_counter[k]` and `A_moving[k]` for `H_x` and `H_y`.
- Effective position, velocity, and excursion bounds.
- Exogenous moving-arm position and velocity.

Approximate planar CoM as:

\[
c_{xy}(q_c)
\approx \bar c_{xy,k}
+J_{c,k}(q_c-\bar q_{c,k}).
\]

Hold `J` and `A` fixed within one solve. This intentionally omits the
configuration derivatives of `J(q) dq` and `A(q) dq`. Recompute the maps on the
next control tick around the shifted solution. The resulting derivatives are
exact for the frozen-map model, not for the nonlinear Pinocchio residual.

After solving, recompute nonlinear Pinocchio metrics along the candidate
trajectory. Enforce a per-knot counter-position trust region around `q_bar`.
Log frozen-map-versus-nonlinear error over the full horizon and reject a command
when either the trust region or maximum normalized residual discrepancy exceeds
its configured threshold. First-step validation alone is insufficient because
future model error can change the selected first control.

### Verified Running Residuals

Evaluate costs at the interval-end state so the current acceleration can affect
the first optimized residual.

The normalized CoM-velocity residual is:

\[
r_{c,k}=
\frac{
J_{c,k+1}\dot q_{c,k+1}
+J_{m,k+1}\dot q_{m,k+1}
+k_c(c_{xy,k+1}-c^*_{xy})
}{s_c}.
\]

The normalized angular-momentum residual is:

\[
r_{H,k}=
\frac{
A_{c,k+1}\dot q_{c,k+1}
+A_{m,k+1}\dot q_{m,k+1}
-k_\omega\omega_{xy}
}{s_H}.
\]

These are the current reactive residuals written as zero-target residuals. Their
physical scales remain the current `com_velocity_scale` and `momentum_scale`.

Add regularization residuals:

\[
r_{q,k}=\frac{q_{c,k+1}-q_c^0}{s_q},
\qquad
r_{a,k}=\frac{u_k}{s_a}.
\]

The iteration-3A running cost is:

\[
\ell_k=
\frac{w_c}{2}\|G_{c,k}r_{c,k}\|^2
+\frac{w_H}{2}\|G_{H,k}r_{H,k}\|^2
+\frac{w_q}{2}\|r_{q,k}\|^2
+\frac{w_a}{2}\|r_{a,k}\|^2
+\ell_{limits,k}
+\ell_{brake,k}.
\]

`G_c` and `G_H` are axis-specific square-root activation matrices. Apply
activation once to
the balance residuals. Do not repeat the current behavior where `balance_scale`
changes both the targets and their relative weights.

Map response axes explicitly:

- CoM X uses the pitch gate.
- CoM Y uses the roll gate.
- Angular momentum X uses the roll gate.
- Angular momentum Y uses the pitch gate.

### Braking Viability

The horizon must preserve the ability to stop before an excursion or joint
limit. Define positive and negative remaining distances separately:

\[
d_i^+(q_i)=q_{i,max}-q_i,
\qquad
d_i^-(q_i)=q_i-q_{i,min}.
\]

Use smooth positive and negative velocity parts, `v_i^+` and `v_i^-`, and
penalize violations of:

\[
(v_i^+)^2\leq2a_{brake,i}d_i^+(q_i),
\qquad
(v_i^-)^2\leq2a_{brake,i}d_i^-(q_i).
\]

Use softplus-smoothed velocity parts and a smooth hinge with a configured width
as `ell_brake`. This avoids a derivative discontinuity when velocity changes
sign. Include per-joint braking acceleration and a safety factor explicitly.
This is a viability cost, not a claim of a hard Crocoddyl state constraint. The
first published step remains hard-bounded separately.

The effective braking acceleration is conservative:

\[
a_{brake,effective,i}
=\frac{a_{brake,identified\ lower,i}}{s_{brake}},
\qquad s_{brake}\geq1.
\]

Use the minimum observed safe deceleration magnitude from the preregistered
identification repeats as `a_brake,identified lower`. The position bounds in
`d_i^+` and `d_i^-` are the intersection of URDF, publisher, configured, and
captured-excursion bounds. The initial configured braking acceleration is only a
low-authority hypothesis and must be replaced by the identified conservative
value before broad active output.

### Terminal Cost

The active and recovery terminal cost contains:

- Braking-viability margin.
- Remaining excursion margin.
- Weak posture regularization.

It must not contain an unconditional target `H_counter = 0` or
`dq_counter = 0`. Those targets recreate the documented braking-payback failure
when measured base response is still diverging.

Only `RETURN` mode adds a strong terminal posture and zero-velocity target. That
mode is entered after a measured quiet dwell, not at a fixed time after moving
arm completion.

The terminal model has `nu = 0` and contains state-only terms. Acceleration is
not part of the eight-state terminal model.

### Deferred Momentum-Rate Term

Acceleration control permits the later diagnostic:

\[
\dot H
\approx A(q)\ddot q+\dot A(q,\dot q)\dot q.
\]

Do not add a weighted momentum-rate residual in iteration 3A. First validate
that the finite-horizon implementation predicts the existing CoM and momentum
metrics. Iteration 3B may add one momentum-rate term as a controlled ablation
after command-to-measured momentum delay and gain are identified.

## Activation And Recovery

The new controller uses no side-specific or lower-policy-specific rule.

Compute two axis-specific risk sources:

- Predicted feedforward risk: the maximum no-counter CoM and momentum residual
  over the moving-arm horizon.
- Measured response risk: settled-reference roll and pitch displacement,
  angular velocity, and whether each axis is moving away from or toward the
  reference.

Use normalized no-counter residuals to define feedforward risk:

\[
\rho_{ff,pitch}
=\max_k\sqrt{r_{c,x,k}^2+r_{H,y,k}^2},
\]

\[
\rho_{ff,roll}
=\max_k\sqrt{r_{c,y,k}^2+r_{H,x,k}^2}.
\]

For response axis `i`, define divergence as:

\[
D_i=(\theta_i-\theta_i^0)\omega_i>0.
\]

When `D_i` is true, response risk is the maximum normalized tilt and rate:

\[
\rho_{resp,i}=\max\left(
\frac{|\theta_i-\theta_i^0|}{\theta_{enter,i}},
\frac{|\omega_i|}{\omega_{enter,i}}
\right).
\]

Define a returning axis only when the orientation error and rate are both above
their exit thresholds and their product is negative. For any scalar risk `rho`
with `rho_enter > rho_exit`, use the hysteretic ramp:

\[
g(\rho)=\operatorname{clip}\left(
\frac{\rho-\rho_{exit}}{\rho_{enter}-\rho_{exit}},0,1
\right).
\]

Build `g_ff` from predicted risk. Build measured `g_resp` as the maximum of the
tilt and rate ramps only while the axis is diverging. When the axis is quiet,
use `max(g_ff, g_resp)`. When it is returning, use
`max(return_attenuation * g_ff, g_resp)`. Clamp the result to `[0, 1]` and
multiply it by caller authority once. Construct square-root cost gates:

\[
G_c=\operatorname{diag}(\sqrt{g_{pitch}},\sqrt{g_{roll}}),
\qquad
G_H=\operatorname{diag}(\sqrt{g_{roll}},\sqrt{g_{pitch}}).
\]

All risk values are dimensionless because they use the same physical scales as
the OCP residuals.

Use hysteretic controller modes:

| Mode            | Entry                                                                         | Behavior                                                                            |
| --------------- | ----------------------------------------------------------------------------- | ----------------------------------------------------------------------------------- |
| `IDLE`          | No predicted risk and measured response is quiet.                             | Hold the counter reference and skip Crocoddyl.                                      |
| `ACTIVE`        | Predicted risk exceeds entry threshold or measured response diverges.         | Solve with CoM and momentum residuals active.                                       |
| `RECOVERY_WAIT` | Moving-arm risk has decayed but measured response is not quiet.               | Continue receding-horizon solve without forcing posture return.                     |
| `RETURN`        | Response remains below quiet thresholds for a configured dwell.               | Solve a low-authority posture return with reduced velocity and acceleration bounds. |
| `FAULT_HOLD`    | Invalid state, support, model, solve, timing guard, or trajectory validation. | Publish moving-arm pass-through and counter hold.                                   |

Complete transition rules are:

- `IDLE -> ACTIVE`: either axis exceeds its entry threshold.
- `ACTIVE -> RECOVERY_WAIT`: feedforward risk is below exit threshold and no
  axis has divergent measured response.
- `RECOVERY_WAIT -> ACTIVE`: feedforward risk rises or measured divergence
  reappears.
- `RECOVERY_WAIT -> RETURN`: both axes remain below tilt and rate exit
  thresholds for the complete quiet dwell.
- `RETURN -> ACTIVE`: new feedforward risk or measured divergence appears.
- `RETURN -> IDLE`: posture and velocity remain inside return tolerances for the
  configured completion dwell.
- `FAULT_HOLD -> IDLE`, `ACTIVE`, or `RECOVERY_WAIT`: the next fresh, valid tick
  is classified by the same entry rules; no fault state clears by timer alone.

Roll and pitch gates can differ while the global mode remains active. Use
hysteresis and a quiet dwell to avoid switching on sensor noise. Do not use an
indefinite activation latch.

The benchmark's time-based reactive fade must not force predictive recovery.
For the predictive variant, caller authority remains available through the
evaluation window and the internal measured-response state machine decides when
to brake and return.

## Constraints And Safety

### Box-FDDP Control Bounds

Use `SolverBoxFDDP` and set each action model's `u_lb` and `u_ub`. Global bounds
include the configured acceleration limit. For the first action, also intersect:

- Next-step velocity room.
- Next-step joint-position room.
- Next-step captured-excursion room.
- Acceleration change from the previously applied command.

For a Euclidean joint, first-step position room gives:

\[
\frac{2(q_{min}-q-\Delta t\dot q)}{\Delta t^2}
\leq u_0\leq
\frac{2(q_{max}-q-\Delta t\dot q)}{\Delta t^2}.
\]

Velocity room gives:

\[
\frac{-\dot q_{max}-\dot q}{\Delta t}
\leq u_0\leq
\frac{\dot q_{max}-\dot q}{\Delta t}.
\]

The initial acceleration cap is `25 rad/s^2`, matching the existing Pink IK
acceleration limit. It must be validated against measured counter-arm response
before promotion.

### Command Realization

`u_0` is a nominal command acceleration, not a measured motor acceleration. Map
the accepted first action to the low-level position and velocity interface:

\[
q_{c,cmd}=q_{c,measured}
+\Delta t\dot q_{c,measured}
+\frac{1}{2}\Delta t^2u_0,
\]

\[
\dot q_{c,cmd}=\dot q_{c,measured}+\Delta t u_0.
\]

Publish one atomic 14-joint arm command:

- Moving proximal and wrist joints: exact sample-zero upstream position,
  velocity, and torque semantics.
- Counter active joints: integrated position and velocity above, plus gravity
  compensation torque.
- Counter wrists: captured position, zero velocity, and gravity compensation
  torque.

Apply first-step bounds before integration, collision backtracking after
integration, publisher clipping during the atomic write, and measured-response
logging on the following tick. Validate command-to-measured acceleration and
delay in a low-authority active identification stage before treating `u` as
physical acceleration or the braking model as physically calibrated.

### State And Collision Limits

Box-FDDP directly enforces control bounds, not all nonlinear state and collision
constraints. Be explicit about the two-layer safety design:

1. Add smooth high-weight position, velocity, excursion, and braking costs to
   the OCP.
2. Validate every solved knot against hard position, velocity, excursion, and
   collision checks before accepting the plan.
3. Re-run the current first-step collision backtracking before publication.
4. Let publisher clipping and the independent estop retain final authority.

If full-horizon collision validation does not fit the real-time budget, do not
disable it silently. Reduce the horizon, improve collision-distance evaluation,
or reject the candidate at the timing gate.

### Failure Behavior

Reject the predictive command and hold the counter arm when any condition is
true:

- Input state or moving horizon is nonfinite, stale, mistimed, or expired.
- Support geometry is invalid.
- The Crocoddyl solve throws an exception.
- The returned trajectories have invalid shape or nonfinite values.
- First control violates computed hard bounds.
- Any predicted state violates hard trajectory validation.
- Collision checking fails or reports collision.
- Full-horizon nonlinear metric mismatch or trust-region displacement exceeds
  threshold.
- Synchronous computation exceeds its timing guard; the current publication is
  already delayed, so the overrun is logged and the counter command is held.
- The low-command handler is estopped.

The moving-arm sample is still published unless the global estop prohibits all
publication. A synchronous solver has no wall-clock cancellation API, so this
contract preserves command values but cannot guarantee that a pathological
solve does not delay the atomic publication. Active use therefore requires a
measured maximum-time gate, not only percentile timing.

## Crocoddyl Execution Design

Use Crocoddyl `3.2.1`, which is already locked in the project environment.

Recommended solver setup:

- Solver: `crocoddyl.SolverBoxFDDP`.
- State: `crocoddyl.StateVector(8)`.
- Horizon: ten running models plus one terminal model initially.
- Iterations: two initially.
- Callbacks: none in the real-time loop.
- Derivatives: analytical derivatives of the frozen-map per-knot model.
- Numerical differentiation: tests only.

Keep one persistent problem and solver per arm ownership. Update mutable
per-knot frozen-map data and `problem.x0` every tick rather than rebuilding
Pinocchio models.

Warm start by:

1. Shift the previous accepted control sequence by one knot.
2. Repeat or safely decay the final acceleration for the last seed.
3. Clip seed controls to current box bounds.
4. Roll out the current problem from measured `x0` to produce a feasible state
   seed.
5. Call `solve(xs, us, maxiter, True, initial_regularization)`.

On the first active tick, use a zero-acceleration seed plus an optional
single-step reactive velocity target converted to bounded acceleration. The
reactive seed is only an initialization aid and must not be used as fallback
output.

Do not require Crocoddyl's convergence Boolean to be true after two iterations.
Accept a best-effort result only when it is finite, bounded, trajectory-valid,
inside the frozen-map trust region, and improves total cost over the feasible
seed by a configured minimum. Log convergence, stopping criterion, iteration
count, regularization, cost change, and timing separately.

## Real-Time Budget

The control period is approximately `20 ms`. Use the following initial gates:

- Solver p50: at most `3 ms`.
- Solver p95: at most `6 ms`.
- Solver p99: at most `10 ms`.
- Complete predictive controller p99: at most `15 ms`.
- Complete predictive controller maximum: below `20 ms` across the preregistered
  stress test and all promotion trials.
- No unexplained controller-caused interval above `20 ms`.

These are implementation gates, not expected measured results. Benchmark
horizon lengths `5`, `8`, and `10` with one, two, and three iterations. Select
the longest configuration that passes the budget; do not select horizon length
from physical outcome alone.

Keep the solve synchronous for iteration 3A shadow mode. Crocoddyl exposes an
iteration limit but no wall-clock timeout or cancellation. Active synchronous
output is allowed only after the measured maximum controller time stays below
one period in the stress test. If that gate fails, stop iteration 3A active work
and design a latest-only worker with explicit one-tick delay compensation,
bounded plan age, and deterministic hold behavior. Do not describe a post-solve
elapsed-time check as a hard deadline.

## Initial Configuration Surface

Use a new configuration section so current reactive configurations retain their
meaning:

```yaml
counter_ddp:
    enabled: false
    shadow: true
    horizon_steps: 10
    max_iterations: 2
    timing_guard: 0.015
    initial_regularization: 1.0e-6
    weights:
        com: 1.0
        momentum: 2.0
        posture: 0.02
        acceleration: 0.01
        braking: 10.0
        limit: 100.0
        terminal_posture: 0.0
        terminal_velocity: 0.0
    gains:
        com: 2.0
        gyro: 0.2
    scales:
        com_velocity: 0.1
        momentum: 1.0
        posture: 1.0
        acceleration: 25.0
    max_velocity: [2.6, 3.2, 2.6, 1.5]
    max_acceleration: [25.0, 25.0, 25.0, 25.0]
    max_acceleration_change: [5.0, 5.0, 5.0, 5.0]
    max_excursion: [0.35, 0.28, 0.20, 0.28]
    braking_acceleration: [12.5, 12.5, 12.5, 12.5]
    braking_safety_factor: 1.25
    braking_smoothing: 0.01
    max_frozen_map_displacement: 0.10
    activation:
        predicted_enter: [null, null]
        predicted_exit: [null, null]
        tilt_enter: [null, null]
        tilt_exit: [null, null]
        rate_enter: [null, null]
        rate_exit: [null, null]
        return_attenuation: [null, null]
        quiet_dwell: 0.5
        return_position_tolerance: 0.01
        return_velocity_tolerance: 0.02
        return_completion_dwell: 0.25
    freshness:
        max_state_age: null
        max_forecast_age: null
        max_command_age: null
    validation:
        full_horizon_collision: true
        full_horizon_metric_tolerance: null
        minimum_cost_improvement: 0.0
```

The initial balance weights come from momentum wide because it is the strongest
current fast-profile hard-case candidate. The excursion envelope starts from
aggressive tight because wider motion caused ordinary FAME and ALMI regressions.
This combination is a prior for shadow evaluation, not a promoted candidate.

The acceleration-change and braking values above are conservative hypotheses,
not calibrated actuator limits. `null` thresholds must be identified from
development shadow logs before active output. Do not hide guessed activation or
freshness thresholds in code defaults.

## Diagnostics And Logging

Add one compact diagnostic record per control tick with:

- Controller mode and mode transition reason.
- Reference-capture state and age.
- Forecast source, age, horizon length, and validity.
- Roll and pitch activation separately.
- Predicted feedforward risk and measured response risk separately.
- Solver type, iterations, convergence, stopping criterion, and regularization.
- Seed cost, optimized cost, and cost improvement.
- Linearization time, solve time, validation time, and total controller time.
- Deadline and control-period miss flags.
- Warm-start use and previous-plan age.
- First requested and accepted acceleration, velocity, and position.
- Minimum position, velocity, excursion, braking, and collision margins over the
  horizon.
- Frozen-map and nonlinear full-horizon CoM and momentum residual mismatch.
- Moving-arm position, velocity, and torque command errors.
- Counter command after solver, hard bounds, collision backtracking, publisher
  clipping, and measured response.
- Failure status and fallback reason.

Store horizon arrays only in an optional lower-rate debug stream. The normal
50 Hz log should contain summaries to avoid changing benchmark timing through
logging volume.

Extend `summary.json` with:

- Active solve count and mode durations.
- Solve and total-controller p50, p95, p99, and maximum times.
- Deadline misses and rejected plans.
- Warm-start acceptance rate.
- Mean and worst full-horizon model mismatch.
- Maximum commanded acceleration and acceleration change.
- Minimum predicted braking margin.
- Collision and publisher clipping counts.

## Implementation Sequence

### Stage 0: Freeze Baselines And Resolve Interfaces

1. Add tests that reproduce the inherited moving-velocity loss and moving-torque
   bias loss.
2. Route pre-integration IK velocity to forecast data without changing inherited
   published moving velocity.
3. Preserve supplied moving-arm torque in the predictive inherited path without
   changing `frame_task` publication.
4. Add explicit settled-reference capture before release in the benchmark
   runtime.
5. Add truthful requested-to-applied command diagnostics.
6. Re-run one frame and one current reactive smoke case to prove baseline
   outcomes and moving-arm pass-through are unchanged.

Exit gate: explicit trajectory position, velocity, and torque pass-through tests
pass; inherited published velocity remains unchanged; inherited forecast
velocity is nonzero; current reactive tests remain unchanged.

### Stage 1: Implement And Verify The OCP Offline

1. Implement the discrete eight-state dynamics and analytical derivatives.
2. Implement per-knot frozen-map CoM and momentum residuals.
3. Verify residual parity with `objective.py` at the same state and velocity.
4. Verify analytical derivatives against `ActionModelNumDiff` over randomized
   valid states.
5. Add Box-FDDP acceleration bounds and first-step hard bound construction.
6. Add shifted warm starts and feasible rollout.
7. Add nonlinear trajectory validation and braking-margin diagnostics.
8. Benchmark horizon and iteration combinations without DDS or MuJoCo.

Exit gate: analytical-versus-numerical derivative absolute error is at most
`1e-6` and relative error is at most `1e-5` away from smoothing boundaries;
first-step bounds are never violated; and at least one five-or-more-knot
configuration passes the solver timing gate.

### Stage 2: Integrate Shadow Mode

1. Register `counter_ddp_shadow` in the benchmark harness.
2. Generate exact moving horizons from the benchmark trajectory function.
3. Run the solver at 50 Hz while publishing frame-task counter holds.
4. Compare the frozen-map candidate trajectory with nonlinear Pinocchio
   evaluation of that same counterfactual trajectory.
5. Compare the measured no-counter next sample only with the no-counter
   prediction that was actually published.
6. Identify activation thresholds from development hard and nominal guard
   distributions.

Shadow targets must include:

- No-motion standing on FAME and ALMI.
- The 44-case hard panel on both policies.
- The three FAME ordinary-overhang regressions from iteration 2.
- ALMI `left_upward_arc_05` and `right_overhang_upward_03`.
- Mirrored benign targets for symmetry checks.

Partition targets before inspecting predictive outcomes. Group adjacent
amplitudes and correlated physical families into one partition to prevent
family leakage. Use a stratified 60% development and 40% held-out split by arm,
policy, and historical outcome severity. Freeze the split and its catalog hash.
Tune shadow risk normalization and screening thresholds only on development
data, then freeze those values before inspecting held-out shadow risk metrics.
Physical activation thresholds may later change only from Stage 3 development
trials. Held-out physical outcomes remain sealed until that second and permanent
freeze.

Exit gate: no timing regression; development hard-versus-benign risk AUROC is at
least `0.75`; held-out AUROC is at least `0.65`; frozen-map full-horizon
normalized residual mismatch has p99 at most `0.10` and maximum at most `0.25`
inside the `0.10 rad` trust region.

### Stage 2A: Synchronous Timing Qualification

Before any active command, run a preregistered timing stress matrix with normal
logging and horizon validation enabled:

- Cold and warm starts.
- Left and right counter-arm ownership.
- Every candidate horizon and iteration count.
- Nominal, near-limit, and collision-dense postures.
- Valid solves and each validation-failure path.
- Idle and representative benchmark CPU load.
- At least 1,000 solves per retained configuration and 100 per failure path.

Exit gate: total predictive computation has p99 at most `15 ms` and observed
maximum below `20 ms`. No active identification command is permitted if this
gate fails. A failed gate requires the bounded-age worker design described in
the real-time section before proceeding.

### Stage 2B: Low-Authority Active Identification

1. Use only the development partition and a reduced acceleration and excursion
   envelope.
2. Apply isolated first actions with no posture return during the identification
   window.
3. Measure command-to-measured counter position, velocity, nominal acceleration,
   momentum, CoM, and delay.
4. Calibrate or reduce nominal acceleration and braking assumptions.
5. Repeat no-motion tests on both policies after calibration.

Use at least five repeats for each active joint and command direction. Exit gate:
command-to-velocity sign is correct in every valid repeat; response onset is at
most `60 ms`; the configured effective braking acceleration is no greater than
the minimum observed safe deceleration magnitude divided by `1.25`; and no
safety or standing regression occurs. Shadow mode alone cannot validate a
counterfactual active counter command against measurement.

### Stage 3: Activate A Conservative Candidate

1. Enable output with the tight excursion envelope.
2. Start with the shortest horizon that passed Stage 1.
3. Keep the iteration-3A objective fixed during the first physical comparison.
4. Tune only activation thresholds on the development guard set.
5. Require quiet-dwell recovery before posture return.
6. Inspect every changed outcome video and time-aligned diagnostic trace.

Freeze physical activation thresholds before Stage 4 development ablations. Do
not run any held-out physical trial yet.

Exit gate: no standing activation, no moving-arm command change, no new fall,
no safety failure, and no guard-set classification regression.

### Stage 4: Isolate Horizon Value

Compare these candidates with identical metrics and limits:

| Candidate                                       | Purpose                                                |
| ----------------------------------------------- | ------------------------------------------------------ |
| Current momentum wide                           | Verified iteration-2 reactive baseline.                |
| Predictive, one interval                        | Controls for Crocoddyl and acceleration-level changes. |
| Predictive, selected horizon                    | Measures the value of look-ahead.                      |
| Predictive, selected horizon without warm start | Measures warm-start timing and solution contribution.  |

Do not add a momentum-rate cost during this comparison. The multi-step candidate
must outperform its one-interval ablation before attributing benefit to the
horizon.

Use development repetitions to select between the one-interval and multi-step
candidates. Exit gate: the selected horizon has more paired-majority wins than
losses against the one-interval ablation, adds no fall, and passes the same
timing and functional gates.

After this gate, freeze the complete candidate, including horizon, iteration
count, weights, activation, limits, and recovery rules. Evaluate it once on the
held-out physical partition. The one-interval ablation may be logged there for
scientific comparison, but it cannot replace a failed selected candidate. A
held-out failure ends iteration 3A promotion rather than starting another tuning
or model-selection pass.

### Stage 5: Optional Iteration 3B

Proceed only if iteration 3A predicts and executes reliably but physical gains
remain limited.

Candidate additions, one at a time:

1. Add a measured command-to-arm-momentum delay and gain model.
2. Add momentum-rate as a logged prediction with zero cost weight.
3. Add one weighted momentum-rate residual.
4. Augment state with previous acceleration if a horizon-wide jerk residual is
   needed.
5. Defer any identified arm-command-to-base-response transition model to
   iteration 4; iteration 3B may identify only arm-actuator gain and delay.

Each addition requires a fresh one-term ablation. Do not combine identified
base response, momentum rate, contact state, and a longer horizon in one change.

## Test Plan

### OCP Unit Tests

- State transition matches the constant-acceleration equations.
- `calc()` and `calcDiff()` agree with numerical differentiation.
- CoM and momentum residuals match the current objective at a common state.
- Left and right mirrored problems produce mirrored controls.
- Box controls obey acceleration bounds.
- First-step bounds imply valid next position, velocity, and excursion.
- Braking violation activates only outside the viability envelope.
- Terminal active mode does not force zero counter velocity.
- Return mode does add bounded posture and velocity convergence.
- Warm-start shift and rollout produce a feasible state sequence.
- Best-effort solver output is accepted only after all validation gates.

### Controller Unit Tests

- Explicit moving horizon preserves sample-zero moving commands exactly.
- Inherited IK path preserves existing published velocity semantics while its
  forecast receives the nonzero IK velocity.
- Moving-arm integral torque bias is preserved.
- Counter wrists remain at captured reference.
- Invalid support, stale horizon, model failure, solver failure, trust-region
  failure, collision, and estop use the required fallback.
- Ownership reset clears references, warm start, activation, and previous
  acceleration.
- Reference capture occurs before release when requested explicitly.
- No backend name changes controller output.
- Mode transitions obey entry, return, quiet-dwell, and fault rules.
- Publisher clipping is reflected in diagnostics.

### Integration Tests

- Run 1,000 or more warm receding-horizon solves and report timing tails.
- Run standing shadow mode for the complete benchmark duration.
- Run active no-motion tests on both lower policies.
- Run one left-moving and one right-moving hard case with complete traces.
- Verify moving-arm command error remains below `1.2e-7 rad/s`, matching the
  iteration-2 explicit-trajectory pass-through evidence.
- Verify no accepted horizon contains a limit or collision violation.

## Benchmark Plan

Use three controllers:

- `frame_task`.
- `reactive_counter_balance_momentum_wide`.
- `counter_ddp`.

Use the same MAGPIE model, `1.5 s` fast motion, corrected ALMI observation map,
confirmed-fall lifecycle, ten-second hold, support geometry, safety limits, and
target catalogs as the latest matched panels.

### Phase A: Compact Guard Set

Run at least three paired repetitions per controller and policy for:

- All 44 hard conditions.
- The three ordinary FAME overhang regressions.
- The five ALMI hard-panel reactive regressions.
- FAME `right_upward_arc_05`.
- ALMI `left_upward_arc_05`.
- ALMI `right_overhang_upward_03`.
- At least four mirrored benign targets.
- Standing with no manipulation.

Use only the preregistered development partition for threshold tuning. Lock the
configuration and its hash before running or unsealing the held-out partition.
No parameter may change after a held-out physical outcome is inspected.

Branching all candidates from an identical saved simulator and controller state
is preferred. If that is not implemented, interleave trial order and preserve
release-relative timing.

### Phase B: Full Regression Panels

After Phase A passes, run all three controllers for three matched repetitions on
every cell of the full 100-target panel on both FAME and ALMI. Define each
case-level outcome by majority severity before aggregating survival and paired
transitions. Historical single-run results remain context only and are not mixed
with repeated promotion evidence.

### Ranking Metrics

Rank in this order:

1. Confirmed falls and survival.
2. Stable, drift, stumble, and fall paired classification changes.
3. New regressions and safety failures.
4. Median and tail base orientation drift.
5. Maximum and RMS base translation and foot displacement.
6. Moving-arm tracking and pass-through error.
7. Counter excursion, velocity, acceleration, braking margin, and clipping.
8. Solve-time and control-period tails.
9. CoM, ZMP, and momentum diagnostics.

Do not rank by mean CoM margin alone. The FAME mean is dominated by rescued-fall
outliers, and ALMI's margin is saturated at one repeated value in most trials.

## Promotion Gates

A predictive candidate may be promoted only if all gates pass.

### Functional Gates

- Moving-arm position, velocity, and torque pass through to numerical tolerance.
- Counter wrists remain held.
- No stale plan is published.
- Every solver or validation failure changes only the counter command values;
  synchronous computation delay is evaluated by the timing gates.
- No accepted plan violates hard first-step limits or collision checks.
- Diagnostics identify every clip, backtrack, rejection, and deadline miss.

### Real-Time Gates

- Total predictive-controller p99 is at most `15 ms` on the benchmark host.
- Maximum total predictive-controller time is below `20 ms` across the
  preregistered stress test and all promotion trials.
- No controller-caused 50 Hz deadline miss occurs in promotion trials.
- A trial is rerun for timing evidence when an external gap above `40 ms`
  overlaps `ACTIVE` or `RECOVERY_WAIT`; physical classification remains separate
  unless the gap changes execution completeness.

### Physical Gates

- On the newly repeated FAME hard panel, predictive majority survival is no
  worse than newly repeated momentum wide and is better than newly repeated
  frame task.
- The four historical FAME hard rescues are explicit sentinels, but promotion is
  decided from their newly repeated paired-majority outcomes.
- The FAME full-panel ordinary-overhang regressions are eliminated; at minimum,
  no new fall or stable-to-worse majority regression is allowed.
- ALMI hard adds no majority classification regression relative to frame task.
- ALMI full adds no new majority drift, stumble, or fall relative to frame task.
- Standing and benign manipulation produce no unnecessary active response.
- Tracking classification does not worsen.
- No estop, collision failure, runtime failure, or unexplained infrastructure
  result is introduced.

These gates are intentionally stricter than iteration 2. The purpose of the
horizon is to improve timing and braking, not only recover the same FAME cases
with a more complex solver.

## Principal Risks And Mitigations

| Risk                                                                | Mitigation                                                                                                                                                                    |
| ------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| The moving forecast is wrong.                                       | Separate explicit and constant-velocity evidence; reject stale horizons; log forecast error.                                                                                  |
| A fixed-base model predicts the wrong physical reaction.            | Keep the first objective on verified internal metrics; validate frozen-map terms nonlinearly and use low-authority active identification; do not claim whole-body prediction. |
| Box-FDDP does not enforce nonlinear state or collision constraints. | Use OCP barriers, full-horizon hard validation, first-step backtracking, publisher bounds, and estop.                                                                         |
| Numerical differentiation misses the 50 Hz budget.                  | Use frozen-map per-knot models with exact derivatives of the approximation; reserve NumDiff for tests.                                                                        |
| Braking pays back useful momentum.                                  | Use acceleration control, braking viability, measured recovery modes, and no active terminal zero-momentum target.                                                            |
| ALMI phase-conflicts with the counter arm.                          | Use measured directional return attenuation and symmetric activation; enforce ALMI no-regression gates.                                                                       |
| Warm starts apply stale behavior.                                   | Use previous plans only as seeds, roll out from current state, and hold rather than applying stale controls.                                                                  |
| Solver convergence is false at two iterations.                      | Evaluate finite best-effort solutions by cost improvement, hard validation, trust region, model mismatch, and measured timing instead of the Boolean alone.                   |
| Collision validation dominates runtime.                             | Measure it separately; reduce horizon or optimize distance checks rather than disabling the gate.                                                                             |
| Added logging changes timing.                                       | Keep normal records compact and put horizon arrays in an optional lower-rate stream.                                                                                          |

## Decisions Fixed By This Plan

- The existing reactive controller remains the baseline.
- Iteration 3 is a separate predictive controller and benchmark variant.
- The moving arm is exogenous and never optimized.
- The counter state initially contains only four positions and four velocities.
- Counter acceleration is the control.
- The first OCP uses the verified CoM and angular-momentum residuals.
- The initial solver is Box-FDDP with analytical frozen-map derivatives.
- The first horizon hypothesis is ten steps at 50 Hz.
- Activation is symmetric and based on predicted risk plus measured response.
- Recovery is event based rather than timer based.
- Active terminal cost does not force zero arm momentum or velocity.
- Hardware output remains disabled until state age and contact confidence are
  available.

## Parameters Requiring Evidence Before Active Output

The following are deliberate experiment outputs rather than coding questions:

- Horizon length and maximum solver iterations.
- Predicted-risk entry and exit thresholds.
- Roll and pitch displacement and rate thresholds.
- Quiet-dwell duration.
- Acceleration regularization and acceleration-change limit.
- Braking viability weight and assumed braking acceleration.
- Full-horizon frozen-map-versus-nonlinear metric tolerance.
- Minimum accepted cost improvement.
- Whether ten-knot full-horizon collision validation fits one control period.

Record selected values, the artifact that selected them, and rejected values in
this document as iteration 3 proceeds.

## Implementation Readiness

Implementation can begin with Stage 0 and Stage 1 without further architecture
decisions. Low-authority output must wait for shadow-mode threshold, nonlinear
model-error evidence, and the synchronous maximum-time stress gate. Broad active
output must also wait for low-authority command-response and braking
identification. This separation keeps the first code increment testable,
preserves the verified baselines, and makes the value of finite-horizon control
measurable rather than assumed.

## Fall-Recovery Improvement Cycle

### Observation

Iteration 2 rescued three FAME right-boundary falls. At `0.5 s` after release,
its counter arm had accumulated approximately two to five times more opposing
pitch momentum than the selected iteration-3 controller. Iteration 3's later
high velocity is corrective; the fall trajectory is already established.

The selected iteration-3 baseline uses `0.25` moving-phase authority to avoid
unnecessary ALMI interference. Raising all moving-phase authority to `1.0`
restored FAME rescues in screening but introduced ALMI guard regressions.

The existing moving-arm centroidal map provides a general discriminator:

| Case family | Peak planar moving-arm momentum norm |
|---|---:|
| FAME right-boundary falls | `2.33–2.56` |
| Corresponding ALMI right-side guards | `1.56–1.89` |

ALMI `left_fast_fall_search_09_scale_78` also exceeds the high-risk range and is
a mandatory guard rather than an excluded exception.

### Hypothesis

Use predicted moving-arm momentum magnitude to schedule only the moving-phase
feedforward authority:

\[
\rho_m=\max_k\|A_{m,xy}(q_0)\dot q_{m,k}\|.
\]

\[
g_m=\operatorname{clip}\left(
\frac{\rho_m-1.5}{1.8-1.5},0,1
\right).
\]

\[
\alpha_{moving}=\max\left(
\alpha_{gyro},
0.25+(1.0-0.25)g_m
\right).
\]

The map is evaluated once at the current measured configuration before horizon
construction. Initial offline realized-momentum estimates used a different
scale; shadow diagnostics calibrated the online current-map range to `1.5–1.8`
before active comparison. The schedule adds no solver state, no target-specific
branch, and no backend-specific condition. Stationary recovery remains the
existing scalar gyro feedback.

### Controlled Variables

Keep fixed:

- Three intervals and one FDDP iteration.
- All objective weights and physical scales.
- Position, velocity, acceleration, slew, excursion, and collision limits.
- Gyro thresholds and post-motion recovery behavior.
- Moving-arm command and horizon.

Change only the moving-phase authority schedule.

### Development Screen

FAME fall targets:

- `right_fast_fall_search_06_scale_74`.
- `right_fast_fall_search_09_scale_78`.
- `right_fast_fall_search_11_scale_78`.
- `right_lateral_overhead_reach`.
- `right_inner_upward_overhang_pitch_minus`.

FAME regression guards:

- `left_fast_fall_search_09_scale_78`.
- `left_inner_upward_overhang_pitch_plus`.
- `left_lateral_overhead_reach`.

ALMI guards:

- `right_fast_fall_search_06_scale_74`.
- `right_fast_fall_search_11_scale_78`.
- `right_lateral_overhead_reach`.
- `left_fast_fall_search_09_scale_78`.
- `left_manual_grasp_pitch_plus`.
- `left_manual_grasp_pitch_minus`.

Compare paired frame task, selected iteration-3 baseline, and momentum-risk
candidate from identical initial state where available.

### Promotion Gate

- At least two repeated FAME fall-to-survival conversions.
- No majority ALMI severity regression.
- No FAME stable-to-worse or drift-to-worse regression.
- Moving-arm pass-through remains unchanged.
- Controller p99 remains below `15 ms` and no late active command is accepted.

If the candidate fails, do not add target-specific thresholds. Continue with
iteration 3B momentum-rate identification instead.
