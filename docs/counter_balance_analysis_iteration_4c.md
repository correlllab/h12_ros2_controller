# Counter-Balance Iteration 4C Analysis

## Status

Iteration 4C is complete. The retained implementation is frozen as an
unpromoted diagnostic controller. Frozen Iteration 3C
`counter_ddp_velocity_wide` and B0 remain the operational baselines.

Iteration 4C established that measured response can be added independently to
the frozen 3C manipulation feedforward without losing the two repeated FAME
rescues or violating the real-time budget. It did not establish a response law
that clearly improves ordinary FAME/ALMI behavior or converts an ALMI stumble to
standing.

The next iteration must not be another gain or scheduler sweep. It should build
a few-step model-predictive feedback correction around verified response models,
while keeping the successful 3C feedforward as a fixed nominal action.

## 1. Frozen Inputs to Iteration 4C

The source-of-truth evidence entering 4C was:

- Iteration 3C provides the only promoted one-step velocity controller and two
  repeated FAME fall-to-stand rescues, targets `09` and `11`.
- Iteration 4A validated the real-compatible observation path but rejected its
  common passive response model. Roll-rate and lateral support-relative
  CoM-velocity prediction remained inadequate after estimator correction,
  repeated data, contextual features, and structured alternatives.
- Iteration 4B showed that preview-only authority loses rescue `09`, measured
  response is necessary to retain both rescues, and one scalar applied to the
  complete 3C controller does not improve the ordinary-case tradeoff.
- Iteration 4B H3-0 showed that repeating the 3C running objective over a short
  horizon without verified response dynamics or a terminal objective is both
  behaviorally worse and too slow.

These results excluded the 4A full response model, the 4B scalar scheduler, and
H3-0 from 4C.

## 2. Infrastructure and Reproducibility Findings

Recent freezes were traced to global memory exhaustion rather than a controller
timing failure. The previous boot had exhausted swap while multiple paired
simulation processes and editor processes were resident. System watchdog
deadlines were then missed.

Iteration 4C restored reproducibility by using serial, headless trials, checking
for orphaned processes between batches, and profiling the controller from the
first active run. The final host state had approximately `27 GiB` available RAM,
zero swap use, an idle GPU, and no stale experiment processes.

The clean 3C reproduction also exposed an important experimental limitation:

- Frozen FAME target `06` was historically a repeated fall, but contemporaneous
  3C survived as drift in two runs.
- Frozen FAME target `04` was historically drift in all three runs, but
  contemporaneous 3C was stable/drift/stable.

Therefore, threshold crossings from non-contemporaneous sweeps cannot be
attributed to a controller change. Future promotion experiments must interleave
or randomize B0, 3C, and candidate trials within the same stable machine session.

## 3. Implemented 4C Controller

The retained controller is
`CounterDecoupledVelocityController`, runtime variant
`counter_decoupled_velocity`. It inherits the complete frozen 3C solve and
changes only the planar momentum target:

\[
r_{H,4C}
=
b\left(
-A_{m,H}\dot q_m
+k_\omega\omega
+k_\theta\operatorname{clip}(e_\theta,-e_{max},e_{max})
\right),
\]

where:

- `b` is the unchanged 3C manipulation lifecycle scale.
- `-A_m,H dq_m` is the unchanged preventive manipulation feedforward.
- `k_omega omega` is the unchanged 3C gyro response term.
- `k_theta = 0.5` and `e_max = 0.1 rad` define the added tilt-restoring term.

The controller remains one-step, velocity-level, policy-blind, target-blind,
real-compatible, and limited to the four proximal joints of the counter arm.
Invalid tilt data disables only the added term and recovers 3C behavior.

The implementation logs the feedforward, gyro-feedback, tilt-feedback, and
positive tilt-rate divergence components independently. This decomposition is a
useful retained contribution because future temporal models can be verified
against physical response rather than inferred from a combined authority scalar.

## 4. Experiments Attempted

### 4.1 C0: Zero-Gain Parity

The new runtime path was executed with zero tilt gain.

Success:

- Reaction targets and published commands matched 3C.
- FAME target `11` reproduced the 3C drift outcome.
- Total-controller p99 was `2.90 ms`.
- No solver, collision, runtime, or publication failure occurred.

Conclusion: the subclass and harness integration do not change behavior when the
new response term is disabled.

### 4.2 C1: Bounded Tilt-Restoring Feedback

The first active candidate used `k_theta = 0.5` and
`e_max = 0.1 rad`.

Success:

- FAME rescue `09` remained drift in all three complete repetitions.
- FAME rescue `11` remained drift in all three complete repetitions.
- Manipulation tracking remained precise in every retained-candidate trial.
- No retained-candidate solver failure or new severe outcome occurred.
- Total-controller p99 was `3.287 ms`, well below the `15 ms` gate.
- The ordinary FAME `04` cell was stable in all three candidate repetitions.

Limitations:

- Contemporaneous 3C was already stable in two of three target-`04` runs.
- Median target-`04` peak drift improved by only about `1.2%`; RMS drift changed
  by less than `1%`.
- ALMI target `11` remained drift and was slightly worse continuously.
- ALMI manual-grasp-minus remained stumble in all three repetitions.
- Manual-grasp foot displacement did not improve.
- FAME target `06` could not be counted as a new rescue because the current 3C
  baseline also survived as drift.

Conclusion: low-gain tilt feedback is safe and directionally useful in some
FAME traces, but its effect is too small and inconsistent for promotion.

### 4.3 Higher Tilt Gains

Tilt gains `1.0` and `2.0` were tested separately.

Failures:

- Neither gain produced a larger repeatable ordinary-case benefit.
- Both worsened the ALMI `11` response relative to gain `0.5`.
- Each produced one KKT-invalid ALMI stumble trial.
- Neither converted the repeated ALMI stumble to standing.

Conclusion: the tilt-to-momentum scale is not a missing scalar gain. Increasing
it drives the local one-step objective toward poor or invalid solutions before it
creates a useful recovery behavior.

### 4.4 Direction-Gated Tilt Feedback

A stronger tilt term was enabled only while tilt and angular velocity had the
same sign.

Failure:

- It did not improve the FAME rescue result.
- It remained worse than low-gain C1 on the ALMI ordinary guard.

Conclusion: a binary divergence gate does not supply the missing temporal model.

### 4.5 Divergence-Rate Feedback

An additional bounded gyro-rate term was enabled only on diverging axes.

Partial success:

- It produced the strongest continuous reduction on FAME target `11` among the
  tested response terms.

Failures:

- Ordinary FAME target `04` regressed to drift.
- ALMI stumble foot displacement increased.
- ALMI target `11` remained worse than 3C.

Conclusion: acting earlier on angular rate can reduce one FAME peak, but a local
rate rule cannot determine the correct action across lower-body response modes.

## 5. Final 4C Results

| Policy | Target | Retained 4C | Result |
| --- | --- | --- | --- |
| FAME | `09` | Drift, 3/3 | Existing rescue retained |
| FAME | `11` | Drift, 3/3 | Existing rescue retained |
| FAME | `06` | Drift, 1/1 | No defensible new rescue |
| FAME | `04` | Stable, 3/3 | Small, baseline-confounded change |
| ALMI | Right `11` | Drift, 3/3 | No ordinary improvement |
| ALMI | Manual grasp minus | Stumble, 3/3 | No stumble recovery |

Across 17 complete retained-candidate trials:

- Solver p50/p95/p99/max was `0.223/0.458/2.071/6.083 ms`.
- Total p50/p95/p99/max was `1.215/2.302/3.287/7.615 ms`.
- Solver failures were zero.
- Maximum tilt-feedback target norm was `0.032172`.

The real-time and rescue-retention gates passed. The ordinary-improvement and
stumble-recovery gates did not pass. Iteration 4C is therefore frozen but not
promoted.

## 6. What 4C Verified

Iteration 4C verified the following reusable facts:

- Early 3C manipulation feedforward can remain fixed while measured response
  independently modifies the reactive target.
- IMU roll/pitch and angular velocity are timely, real-compatible signals that
  can alter physical response without policy or target identity.
- The one-step BoxFDDP/BoxQP path has sufficient timing margin for a small
  estimator and a compact few-step solver if repeated kinematics and logging are
  avoided.
- Zero-feedback parity and invalid-signal fallback can be made exact.
- Decomposed target diagnostics are practical and should be retained.
- A common response law can preserve both FAME rescues across repeated trials.

These are architecture and signal successes, not evidence that the retained
tilt gain is an adequate recovery controller.

## 7. Why 4C Did Not Improve the Tradeoff

The retained controller has no model of how a counter-arm command changes future
base motion. It converts current tilt directly to desired arm momentum using a
heuristic dimensional scale. The optimizer sees only the current linearized
kinematics and cannot predict whether an action will:

- Brake outward angular motion.
- Oppose an already returning body.
- Cause a later overshoot after the manipulation peak.
- Consume counter-arm excursion needed for a later recovery action.
- Interfere with a lower-body step that has already become the safer response.
- Improve support conditions after a foot has started moving.

The added feedback was active only during the inherited 3C lifecycle. A stronger
gain increased authority but did not create braking, reversal, or terminal
recovery logic. Direction gates encoded one instantaneous phase relation but did
not predict the next phase.

The ALMI stumble result is particularly important. Once support is changing, a
fixed-support angular objective can reduce torso motion while increasing foot
travel. Stumble-to-stand recovery requires the controller to recognize measured
support transition, preserve or assist a viable step when necessary, and plan a
post-step settling action. Tilt and gyro alone cannot represent that problem.

## 8. Required Next Controller Architecture

The next controller should be a three-to-five-step receding-horizon controller
at the existing `50 Hz` control rate. The horizon is therefore `60-100 ms`.
Longer-term recovery should enter through a verified terminal recoverability
model, not by increasing the online horizon until timing fails.

Use a fixed nominal-plus-correction structure:

\[
u_k = u_{3C,k}^{ff} + \delta u_k.
\]

The 3C manipulation feedforward sequence is generated from the known arm
trajectory and is not scaled by the response controller. The MPC optimizes only
the feedback correction sequence `delta u`. The first nominal action must retain
the direction and preventive authority that produced rescues `09` and `11`,
except when a hard safety constraint requires less authority.

A suitable reduced state is:

\[
x_k = [
e_\theta,\ \omega,\
c_{rel},\ \dot c_{rel},\
h,\ \dot h,\
q_c,\ \dot q_c,\
s_{support}
].
\]

The exogenous manipulation disturbance is:

\[
d_k = [
h_m,\ \dot h_m,\
\dot q_m,\ \ddot q_m
].
\]

The short-horizon model should have the form:

\[
x_{k+1}
=
f_{mode}(x_k)
+B_{mode}(x_k)\delta u_k
+E_{mode}(x_k)d_k
+r_k,
\]

where `mode` is inferred only from measured support kinematics and response
state. It must never use the lower-body policy name. `r_k` is a bounded residual
or uncertainty set, not an unconstrained target-specific correction.

The MPC cost should penalize predicted tilt/rate divergence, support-relative
CoM or capture-point excursion, loss of support margin, counter-arm velocity and
excursion, action change, and terminal non-recoverability. The objective should
be support-mode aware:

- In valid fixed support, prioritize prevention and braking.
- At measured step onset, avoid fighting the step and limit destabilizing upper
  momentum.
- During support recovery, prioritize torso damping, arm recentering, and entry
  into a verified terminal standing set.

Only the first command is published. Late, infeasible, stale, or uncertain
solutions must fall back atomically to frozen 3C, a verified braking action, or a
hold according to the measured support state.

## 9. Models That Must Be Derived and Verified

### 9.1 O0: Synchronized Response-State Estimator

Purpose: provide one atomic state at each MPC tick.

Required outputs:

- IMU tilt and angular velocity in one documented canonical frame.
- Support-relative CoM position and velocity.
- Base height and vertical velocity.
- Left/right foot pose, twist, yaw divergence, and support-validity state.
- Counter-arm and moving-arm position/velocity with sample timestamps.
- State covariance, age, duplicate-tick status, and validity.

Derivation and build:

- Retain the 4A immutable LowState snapshot and release-time reference capture.
- Redesign the failed roll-rate and lateral CoM-velocity representations.
- Compare filtered differentiation, model-based velocity reconstruction, and
  support-frame transport terms one at a time.
- Preserve exact left/right canonical transforms and test inverse transforms.

Verification gate:

- Finite-difference position/rate consistency must pass on every controlled
  axis.
- Timestamp age and latency must remain within the preregistered control window.
- Noise, bias, and covariance must be calibrated on held-out runs.
- No roll-rate or lateral CoM-velocity axis may be accepted by pooled metrics
  while failing its per-axis gate.
- Simulator truth may be used only for offline validation, never online.

If lateral CoM velocity cannot pass, remove it from the online state and use a
conservative support/capture-point bound. Do not conceal the failure in a larger
state vector.

### 9.2 D0: Manipulation-Disturbance Preview Model

Purpose: convert the known arm trajectory into a short sequence of physical
disturbances.

Required outputs:

- Planar moving-arm centroidal momentum.
- Momentum rate or impulse over each control interval.
- Timing-aligned moving-joint velocity and acceleration.
- Prediction uncertainty caused by tracking lag.

Derivation and build:

- Use Pinocchio centroidal quantities evaluated on measured state and the known
  trajectory preview.
- Model the measured manipulation tracking delay instead of assuming commanded
  and realized motion are identical.
- Keep the model independent of target identity and policy identity.

Verification gate:

- One-to-five-step predictions must beat zero-order hold on every planar axis.
- Impulse sign, peak time, and integrated magnitude must be correct on held-out
  arm families and both arm ownership directions.
- Family-grouped splits must prevent mirrored or neighboring targets from
  leaking between training and validation.

### 9.3 U0: Counter-Arm Command-Realization Model

Purpose: identify what the optimized velocity command actually produces.

This model was never built in 4A because M2 remained blocked. It is mandatory
for MPC. It must map requested counter velocity to realized counter position,
velocity, acceleration, and centroidal momentum, including servo delay,
clipping, backtracking, and state-dependent effectiveness.

Derivation and build:

- Collect bounded, low-amplitude active excitation after O0 passes.
- Excite one direction or frequency band at a time around valid standing states.
- Include both counter-arm ownership directions and representative arm poses.
- Estimate discrete delay before fitting gain or dynamics.
- Identify local effectiveness matrices and their uncertainty, not one global
  scalar.

Verification gate:

- Command-response sign must be correct on every retained axis.
- Delay must be predicted within one controller tick.
- One-to-five-step realized velocity and momentum predictions must pass
  preregistered absolute and normalized error gates.
- The effectiveness matrix must remain sufficiently conditioned in the allowed
  workspace, or the controller must disable weak directions.
- Safety backtracking and clipping must be reproduced exactly.

### 9.4 R0: Short-Horizon Base-Response Model

Purpose: predict how measured base/support state responds to manipulation and
counter-arm momentum over `60-100 ms`.

Derivation and build:

- Use exact kinematic integration for tilt and position states where possible.
- Fit only rate/acceleration or residual dynamics that cannot be derived.
- Include D0 disturbance and U0 realized counter action as separate inputs.
- Include measured lower-body joint/support response features, but not policy
  identity.
- Use uncertainty bounds or an ensemble when a common point model is inadequate.

Verification gate:

- R0 must improve over both persistence and the corrected 4A M0 baseline.
- Roll rate, pitch rate, sagittal velocity, and lateral velocity must each pass
  separate one-step and five-step gates.
- Counter-action ablations must demonstrate that the fitted `B` term improves
  held-out prediction and has the correct physical sign.
- FAME and ALMI held-out results must both pass without a policy label.
- Predicted peak direction, peak time, braking time, and zero crossing must be
  accurate enough to select between continue, brake, and reverse actions.

If one common smooth model still fails, use a measured-support-mode hybrid model
or robust uncertainty set. Do not introduce policy-specific models.

### 9.5 S0: Support-Mode and Step-Onset Model

Purpose: distinguish fixed support, incipient step, moving support, landing, and
recovered support using real-compatible measurements.

Derivation and build:

- Start from the validated 4A foot pose/twist and hysteretic support observer.
- Add joint-torque or estimated contact-wrench evidence only if its
  real/simulation consistency is verified.
- Predict only the short transition needed by the horizon; do not attempt to
  identify the lower-body policy.

Verification gate:

- Fixed-support false exits and moving-support false-valid states must meet
  preregistered safety bounds.
- Step onset and landing must be detected within one control tick on held-out
  stumble families.
- Mode uncertainty must be exposed to the MPC and trigger conservative costs or
  fallback, not be replaced by a hard optimistic label.

This model is necessary for stumble-to-stand recovery. A controller that assumes
fixed support cannot safely decide whether to prevent, permit, or settle a step.

### 9.6 T0: Terminal Recoverability Model

Purpose: let a short online horizon represent recovery beyond `100 ms`.

Preferred form:

- A conservative control-invariant standing set over tilt, angular velocity,
  support-relative CoM/capture point, height, counter-arm excursion, and support
  mode.
- A mode-dependent terminal cost measuring distance to that set.

If an analytic set is too conservative, fit a policy-blind terminal viability
model from long closed-loop rollouts. It may use measured physical state and
support mode, but not target or policy identity.

Verification gate:

- False-safe predictions must be bounded explicitly and evaluated separately
  from false-conservative predictions.
- Calibration must hold on excluded target families and both lower-body systems.
- Terminal value must rank known fall, drift, stumble, and stable trajectories in
  the correct order before it enters control.
- Removing T0 must worsen multi-step recovery prediction in a preregistered
  ablation.

### 9.7 K0: Multi-Step Constraint and Braking Model

Purpose: guarantee that every candidate sequence remains executable and has a
safe exit.

Required constraints:

- Joint position, velocity, acceleration, and acceleration-change limits.
- Counter-arm excursion and return reserve.
- Collision distance with a verified trust region.
- Manipulation-arm command ownership and exact pass-through.
- Support-mode-dependent momentum and support-margin limits.
- A terminal braking or reconnect trajectory.

The existing 4A braking and reconnect prototypes are not control-verified. They
must be exercised physically under bounded active commands before they can serve
as MPC fallback or terminal constraints.

Verification gate:

- Every predicted feasible sequence must remain feasible when replayed through
  the command-realization model and collision checker.
- Braking must stop within predicted excursion and timing bounds.
- Reconnect must not introduce a command discontinuity or manipulation error.
- Any constraint-model disagreement must reject the MPC command, not relax the
  safety limit.

## 10. Model-Building Order

The required order is:

1. Freeze O0 estimator definitions, frames, timestamps, covariance, and gates.
2. Validate D0 on passive manipulation data.
3. Validate support-mode observer S0 passively.
4. Collect bounded active counter-arm excitation and fit U0.
5. Fit R0 using separate D0 and U0 inputs with family-grouped held-out runs.
6. Derive K0 multi-step constraints and physically validate braking/reconnect.
7. Derive or fit T0 terminal recoverability on long rollouts.
8. Run a shadow MPC that predicts response but publishes frozen 3C.
9. Compare predicted and measured first-action, peak, braking, and terminal
   quantities on held-out families.
10. Enable one low-risk active feedback correction with a two-step horizon.
11. Increase to three-to-five steps only after behavior and timing gates pass.

No online MPC should be implemented before U0, R0, S0, and K0 pass. Without
these models, another horizon controller would repeat H3-0 with more terms rather
than add predictive control.

## 11. Solver and Timing Requirements

H3-0 exceeded the timing gate with a minimal Crocoddyl temporal formulation.
The next implementation should therefore use a fixed-size, preallocated reduced
problem:

- Condense the three-to-five-step linear or linearized dynamics into a small
  bound-constrained QP where possible.
- Reuse Pinocchio terms and Jacobians within a control tick.
- Avoid recomputing trajectory preview or diagnostics for every horizon knot.
- Warm-start from the shifted previous solution.
- Use one real-time iteration only when nonlinear constraints require it.
- Reject late results atomically and publish the verified fallback.

The complete observation, model update, solve, safety check, and publication path
must have p99 below `15 ms`. Timing must be measured on deduplicated controller
executions from the first shadow experiment, not inferred from solver-only time.

## 12. Experimental Promotion Ladder

### Gate A: Offline and Passive Validation

- O0, D0, and S0 pass all axis, timing, and held-out-family gates.
- Current B0 and 3C are reproduced in the same stable machine session.
- No target or policy identity enters a feature, model, cost, or gate.

### Gate B: Active Identification

- U0 passes delay, sign, gain, uncertainty, and safety gates.
- R0 passes one-to-five-step open-loop prediction gates with active counter
  commands.
- Braking and reconnect are physically validated.

### Gate C: Shadow MPC

- Predicted first actions preserve the 3C feedforward component.
- Prediction residuals remain inside calibrated uncertainty.
- Support-mode and terminal decisions agree with held-out physical outcomes.
- Total-controller p99 is below `15 ms` with no accepted late command.

### Gate D: Focused Active Control

Use randomized, interleaved B0/3C/MPC trials on:

- FAME rescues `09` and `11`.
- FAME sentinel `06`.
- Ordinary FAME target `04` and a nonduplicate ordinary geometry.
- ALMI ordinary target `11`.
- ALMI manual-grasp-minus and at least one second stumble family.

Require five complete repetitions for each critical rescue, regression, or
changed cell before expansion.

### Gate E: Promotion

Promote only if the few-step controller:

- Retains every repeated 3C fall-to-stand rescue.
- Improves ordinary FAME/ALMI majority behavior to B0 or better.
- Introduces no new majority fall or stumble.
- Converts at least one repeated stumble family to standing, or demonstrates a
  preregistered material reduction in step displacement and terminal recovery
  time without worsening severity.
- Preserves manipulation tracking and ownership.
- Passes real-time, fallback, collision, and support-transition gates.

If the stumble-to-stand gate fails, retain the controller only as a fall-recovery
candidate and do not describe reduced torso motion alone as stumble recovery.

## 13. Recommended Next Iteration

The recommended next iteration is **Verified-Response Few-Step MPC**.

Its first deliverable is not an active controller. It is a source-bound model
package containing O0, D0, U0, R0, S0, T0, and K0 definitions, datasets, grouped
splits, uncertainty estimates, axis-wise validation reports, and timing profiles.

The first active controller should use a two-step horizon and optimize only
feedback corrections around fixed 3C feedforward. It should then progress to
three-to-five steps after the response and timing evidence passes. This directly
addresses what 4C lacked: command effectiveness, braking phase, support
transition, and terminal recoverability, while preserving the mechanism that
already produces the verified fall-to-stand rescues.
