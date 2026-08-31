# Counter-Balance Iteration 4

## Status

Iteration 4 introduces a new policy-blind base-response controller.

Iteration 3 B0 remains frozen and independently runnable. Iteration 4 must not
silently modify B0 code, configuration, or evidence.

Iteration 4 remains a Crocoddyl optimal-control controller. It continues the
Iteration 3 formulation by replacing the missing base transition with an
identified local response model inside a new Crocoddyl action model. It retains:

- `crocoddyl.SolverBoxFDDP`.
- Receding-horizon warm-started optimization.
- Bounded counter-arm acceleration control.
- Analytical `calc()` and `calcDiff()` derivatives.
- First-action and full-trajectory safety validation.

The fitted model is dynamics data for the optimizer, not a learned policy and not
a replacement for optimal control.

Iteration 4A is deliberately narrow:

- Identify a common local closed-loop base-response model.
- Validate it in double support.
- Add a new H3 base-aware counter-arm controller in shadow mode.
- Enable low-authority control only after causal and safety gates pass.
- Use only observations that are available on the real robot from IMU, joint
  position/velocity/torque, or quantities derived from them with Pinocchio.

Longer OCP horizons and predicted contact-mode dynamics are follow-on work. They
are not part of Iteration 4A.

---

## 1. Goal

Use one consistent controller to:

- Convert repeated FAME falls to drift or stable.
- Convert repeated ALMI stumbles to drift or stable.
- Preserve B0 drift/stable improvements.
- Add no new majority fall, stumble, or other severity regression.

FAME is a stand-only policy. Its valid physical outcomes are stable, drift, and
fall. ALMI may also produce stumble.

The manipulation arm remains immutable. Only the four proximal counter-arm
joints may be controlled.

---

## 2. Validated Motivation

The source evidence is:

- `counter_balance_analysis_iteration_3_final.md`.
- `counter_balance_iteration_3b_design_final.md`.

### 2.1 Frozen B0

B0 uses H3, one FDDP iteration, `0.25` moving feedforward authority, and scalar
gyro recovery.

Corrected FAME panel:

| Controller | Stable | Drift | Stumble | Fall | Survived |
|---|---:|---:|---:|---:|---:|
| Frame task | 17 | 19 | 0 | 8 | 36 |
| B0 | 22 | 14 | 0 | 8 | 36 |

ALMI panel:

| Controller | Stable | Drift | Stumble | Fall | Survived |
|---|---:|---:|---:|---:|---:|
| Frame task | 25 | 12 | 4 | 3 | 41 |
| B0 | 25 | 11 | 5 | 3 | 41 |

### 2.2 Iteration 3B Stop Evidence

Iteration 3B established:

- Moving-arm disturbance preview is accurate and available `80–100 ms` earlier
  than H3.
- Frozen counter momentum-rate prediction has repeatable sign and one-tick delay,
  but substantial residual error.
- Short-window momentum change performed worse than one-step rate prediction.
- Candidate reaction seeds saturated on `88–92%` of active samples.
- A first-order actuator model was not valid across all four active joints.
- Measured tilt divergence separates difficult from stable response, but B0
  already detects that response with gyro recovery.

The missing quantity is a model of how manipulation disturbance and counter-arm
action affect the measured base/support response of the closed-loop robot.

---

## 3. Policy-Blind Contract

Backend name is never a model feature, controller input, gate, cost, parameter,
or model selector.

Allowed features are real-compatible physical quantities available from the IMU,
joint position/velocity/torque, or Pinocchio-derived estimates:

- Base orientation and angular velocity.
- Support-relative CoM position and velocity.
- Base height and vertical velocity.
- Manipulation disturbance preview.
- Counter-arm state and command.
- Kinematic foot/support pose and velocity used only for support validity.
- Validated model-derived balance features such as CAM or ZMP, if later justified.

Do not use simulator-only privileged observations such as contact force, ground-truth
contact state, or external force sensing as controller/model features or activation
gates.

Measured physical state may naturally reveal different closed-loop behavior.
Policy blindness means excluding backend metadata and policy-specific models,
not trying to prevent physical observations from containing information.

Require matched-state conditional-invariance tests. If transitions still split
by policy after conditioning on measured state/history, add one physically
defined observed feature or reject the common model. Do not branch by policy.

---

## 4. Canonical Coordinates

Use one robot-centric frame and one joint ordering for both arm sides.

Define and test a mirror transform that canonicalizes:

- Lateral base and CoM quantities.
- Roll-relevant momentum/reaction quantities.
- Left/right counter-joint order and signs.
- Manipulation disturbance features.

Mirrored trajectories must produce mirrored model predictions and commands. Arm
side may select the transform, but never different behavior or parameters.

---

## 5. Observations and Runtime Interface

### 5.1 Base Observation

Add a timestamped observation:

```python
BaseObservation(
    monotonic_time,
    orientation_error_xy,
    angular_velocity_xy,
    com_position_xy,
    com_velocity_xy,
    base_height,
    base_vertical_velocity,
)
```

All values include explicit frames, units, age, and validity.

### 5.2 Proprioceptive Support Validity

Before any active trial, add a timestamped support-validity observation derived only
from IMU/joint state and Pinocchio kinematics:

```python
SupportValidityObservation(
    monotonic_time,
    left_foot_pose,
    right_foot_pose,
    left_foot_twist,
    right_foot_twist,
    double_support_valid,
    confidence,
)
```

Iteration 4A does not use foot contact force, simulator contact booleans, or other
privileged contact information. `double_support_valid` means that the proprioceptive
state remains consistent with stationary double support; it is not a direct contact
measurement.

Freeze and validate thresholds for:

- Left/right foot height and orientation relative to the settled support reference.
- Left/right foot linear and angular velocity.
- Relative foot transform change.
- Hysteresis and debounce sample count.
- Maximum observation age, initially no more than two control intervals.
- Invalid/stale behavior.

The controller may enter `ACTIVE` only while this kinematic double-support validity
gate is satisfied. If the feet begin moving consistently with a step or loss of the
stationary support assumption, stop increasing counter reaction and enter bounded
handoff.

### 5.3 Measured Counter and Support Observation

Add timestamped measured counter state:

```python
CounterStateObservation(
    monotonic_time,
    q_counter,
    dq_counter,
    valid,
)
```

Candidate and baseline response rollouts start from this same measured state.
Upstream commands are never substituted for measured counter state.

Add a support observation containing left/right foot pose and twist from Pinocchio
plus the derived support-frame pose and twist. Iteration 4A accepts only states that
pass the proprioceptive double-support validity gate.

Freeze the support-frame definition:

- Origin: midpoint of the two validated foot contact-frame origins.
- X axis: normalized average of projected foot forward axes.
- Z axis: normalized support-plane normal.
- Y axis: `Z cross X`, oriented toward the left side.
- Orientation error: planar components of the support-to-base rotation log map,
  relative to the settled reference.
- Support-relative CoM velocity:

\[
\dot c_s
=
R_s^T
\left(
v_{com}-v_s-\omega_s\times(c_{com}-o_s)
\right).
\]

Reject degenerate axes, invalid twists, nonfinite transforms, and stale samples.
The mirror transform is defined in this same support frame.

### 5.4 Controller Call

The controller boundary must include:

```python
control_horizon_step(
    upstream_arm_q_horizon,
    upstream_arm_dq_horizon,
    upstream_arm_tau_horizon,
    counter_state_observation,
    base_observation,
    support_validity_observation,
    support_observation,
    sample_times,
    generated_at,
    episode_id,
)
```

The runtime variant is `counter_base_mpc`. Reference capture, preview generation,
diagnostics, summary aggregation, and paired sweep support are explicit tasks.

`upstream_arm_q_horizon`, `upstream_arm_dq_horizon`, and
`upstream_arm_tau_horizon` contain aligned samples for all 14 arm joints. Validate
moving and counter entries, timestamps, shape, finiteness, and the additional
terminal sample required to reconstruct the final interval.

In `PASS_THROUGH`, publish all 14 upstream position, velocity, and torque values
from sample zero exactly before common publisher safety clipping. In active mode,
moving-arm
position, velocity, and torque remain exact; only counter position/velocity are
replaced. Counter candidate torque uses the same gravity-compensation synthesis
used during identification. This torque convention is part of the model artifact
and cannot differ between identification and active control.

The runtime supplies at least `0.5 s` of upstream counter position, velocity, and
torque reference during active/recovery/handoff episodes. If that horizon is
missing or invalid, Iteration 4 cannot enter active mode.

### 5.5 Simulation Observation Source

Iteration 4A is simulation-only, but the controller-facing observation contract must
match future hardware availability. The simulator may use ground truth internally to
produce sensor-equivalent IMU and joint measurements, but privileged simulator state
must not enter the controller, identified model, acceptance gate, or support-validity
gate.

The controller-facing inputs are limited to:

- IMU base orientation and angular velocity.
- Joint position, velocity, and torque.
- Pinocchio-derived CoM, CoM velocity, base/support geometry, foot pose/twist, CAM,
  ZMP, or other explicitly model-derived quantities.

Do not provide per-foot normal load, simulator contact booleans, exact external
wrenches, or ground-truth contact mode to Iteration 4A. Simulation may log these only
for offline diagnostics that do not affect controller/model development decisions.

Transport uses the same monotonic time domain or an explicitly synchronized mapping
with measured latency. Stale or missing observation selects pass-through or bounded
handoff.

Hardware activation requires the same controller-facing schema with validated
latency and estimator behavior; it must not require adding a new sensing modality.

---

## 6. Outcome-Relevant Response State

The local response state is:

\[
z_k=
\begin{bmatrix}
e_{\theta,k} \\
\omega_{xy,k} \\
c_{s,k} \\
\dot c_{s,k} \\
h_k \\
\dot h_k
\end{bmatrix}
\in\mathbb{R}^{10},
\]

where `c_s` is support-relative planar CoM.

This state includes precursors to:

- FAME tilt/height fall confirmation.
- ALMI load transfer and step initiation.

Foot displacement is not optimized. Step outcomes are evaluated over the full
physical trial, including the observed `0.7–1.0 s` delayed step window.

Proprioceptive support validity is an activation/safety gate, not an optimized
state. No contact-force or load-distribution feature is used in Iteration 4A.

---

## 7. Exact Counter Command and Pass-Through Definition

The upstream controller provides complete counter position and velocity
forecasts:

\[
q^b_{c,k},\quad \dot q^b_{c,k}.
\]

Derive the baseline command acceleration consistently from that forecast.

The Iteration 4 candidate generates an alternative bounded counter acceleration
sequence from the same measured initial state.

Pass-through means publishing the upstream counter position/velocity/torque
sample exactly. It never means `u = 0`.

For every candidate solve, create two response rollouts:

1. Baseline rollout using the upstream counter command sequence.
2. Candidate rollout using the optimized counter command sequence.

Both use the same measured state, manipulation forecast, delay state, and model.

### 7.1 Forecast Indexing and Reconstruction

Require uniform monotonic samples at the controller period. Sample `k` is the
command applied over interval `[t_k, t_{k+1})`.

For baseline response features, derive:

\[
a^b_{c,k}
=
\frac{\dot q^b_{c,k+1}-\dot q^b_{c,k}}{\Delta t}.
\]

Check supplied position consistency:

\[
q^b_{c,k+1}
\approx
q^b_{c,k}+\Delta t\dot q^b_{c,k}
+\frac{1}{2}\Delta t^2a^b_{c,k}.
\]

Freeze absolute and relative consistency tolerances before identification. The
baseline response rollout uses supplied `q^b`, `dq^b`, and reconstructed `a^b`
exactly; it does not replace positions with an integrated trajectory.

On any timestamp, position/velocity consistency, shape, or finiteness failure,
prohibit candidate comparison and record an invalid-upstream-forecast
diagnostic.

Failure behavior is mode-dependent:

- In `PASS_THROUGH` or `MONITOR`, publish exact sample-zero pass-through when that
  sample itself is finite and valid. If sample zero is invalid, trigger automatic
  global estop and publish no normal arm command.
- In `ACTIVE`, `RECOVERY`, or `HANDOFF`, enter the validated braking/handoff path
  from the last accepted command. Do not reconnect until a valid aligned
  position/velocity/torque horizon is restored.

Sample-zero mismatch relative to measured counter state is logged and included
in model features. If mismatch exits identification support, prohibit candidate
comparison. In `PASS_THROUGH` or `MONITOR`, retain exact upstream pass-through.
In `ACTIVE`, `RECOVERY`, or `HANDOFF`, enter validated braking/handoff and do not
reconnect until mismatch returns inside identification support with a valid
aligned horizon.

Require one additional upstream sample for the final interval. Do not invent a
terminal acceleration by repeating or zeroing velocity.

---

## 8. Identified Closed-Loop Model

Start with a pooled affine model:

\[
z_{k+1}
=
A z_k
+B_m d_{m,k-d_m}
+B_c r_{c,k-d_c}
+b.
\]

`d_m` is a manipulation disturbance feature. Initially, `r_c` contains only
candidate-computable counter command/state features, such as bounded acceleration
and current counter configuration. Measured realized acceleration or momentum
rate may be lagged observation or diagnostics, but not a future candidate feature.

A predicted reaction feature is permitted only after a new command-to-reaction
model passes held-out per-joint validation. Iteration 3B actuator identification
is insufficient for that promotion.

### 8.1 Delay State

The identified delays are part of the model artifact. If either delay is one
tick, augment the prediction state with that previous input. Do not omit delay
from the OCP equation and do not add unmeasured delay chains.

### 8.2 Feature Sequence

Fit in this order:

| Model | Inputs |
|---|---|
| `M0` | Response state/history only. |
| `M1` | M0 plus manipulation disturbance. |
| `M2` | M1 plus counter command/reaction. |
| `M3` | M2 plus one observed physical interaction feature, only if justified. |

Possible `M3` interaction models are:

\[
B_c(s)r_c
=
\left(B_{c,0}+\sum_i s_i B_{c,i}\right)r_c,
\]

where `s` may be a real-compatible physical quantity such as arm configuration or
a model-derived balance feature. Promote M3 only if M2 fails a predefined validation
gate and the added feature is available from the same hardware observation contract.
A simple additive context bias is insufficient when control effectiveness changes.

---

## 9. Causal Identification Experiments

### 9.1 Passive Manipulation Identification

Run frame-task trajectories with no counter correction. Fit `M0`, then `M1`.

Hypothesis: manipulation disturbance improves held-out multi-step response
prediction beyond response history alone.

### 9.2 Counter Excitation

Establish causal counter influence using:

- Randomized positive, negative, and zero-pulse trials.
- Randomized pulse timing and phase relative to manipulation.
- Balanced joint/direction counts.
- Saved-state branching where available.
- Complete-run train/validation/test splits.
- Low, then moderate, preregistered fractions of existing limits.

Do not select excitation from measured divergence. That would confound counter
input with outcome risk.

Abort on safety, loss of proprioceptive double-support validity, stale
observation, collision, or timing failure.

Hypothesis: adding randomized counter input in `M2` improves held-out
action-conditioned response prediction beyond `M1`.

### 9.3 Side and Policy Validation

Require:

- Per-joint and per-direction sign checks.
- Mirrored-side equivariance checks.
- Matched-state residual analysis across policies.
- Leave-one-policy-out diagnostics, not deployment models.

Stop if the common model requires backend identity.

---

## 10. Data Partitions

All previously inspected targets are development or regression sentinels. They
cannot be called held-out.

Create six immutable partitions with hashes:

1. Identification training runs.
2. Model-selection runs.
3. Controller-development runs.
4. Action-conditioned uncertainty-calibration runs.
5. Untouched action-conditioned uncertainty-test runs.
6. Untouched behavioral-test runs.

The uncertainty-calibration partition sets residual bounds, trust region, model
set, and robust improvement margin. Evaluate coverage and action-conditioned
gates once on the untouched uncertainty-test partition. No tuning follows that
test; failure returns to model development with a replacement uncertainty-test
partition. Neither uncertainty partition is reused for behavioral claims.

Assign every source trajectory/saved-state family atomically to one partition.
All saved-state branches, mirrors, adjacent amplitudes, shared trajectory
prefixes, and associated seeds remain together. Freeze and hash these group
assignments before fitting.

The untouched behavioral set uses preregistered new target amplitudes and new
random seeds. Adjacent amplitudes, mirrors, and the same physical family remain
in one partition. Execute all planned behavioral repetitions as one batch only
after complete controller freeze. Any later model, cost, margin, authority, or
trust-region change invalidates and replaces that behavioral set.

Claim a rescue only against a contemporaneous paired B0 majority fall from the
same saved states/seeds.

Freeze a corrected target-level evidence ledger containing policy, candidate ID,
controller, classification, classification reason, source artifact, classifier
code revision, and content hash. Promotion references this ledger rather than
aggregate counts alone.

---

## 11. Model Validation Gates

Report one-step and multi-step metrics separately for no-action and randomized
counter-action data.

Initial gates:

- Correct action-conditioned sign per response axis.
- Counter input improves held-out prediction over `M1`.
- `100 ms` rollout normalized RMSE at most `0.5` on every state axis.
- Absolute rollout errors below `20%` of the remaining margin to configured
  tilt, height, and support safety boundaries.
- Calibrated uncertainty interval coverage on the untouched uncertainty-test
  partition.
- Stable numerical rollout without suppressing genuine measured divergence.
- Candidate states/actions remain inside identification support/trust region.

Compare the pooled model with policy-specific oracle fits only as a diagnostic.
Require matched-state pooled residuals to have no persistent policy mode.

If no model passes, do not implement active MPC.

---

## 12. Iteration 4A OCP Contract

Iteration 4A uses H3 and proprioceptively validated stationary double support only.

### 12.1 State

Use:

\[
x_k=
\begin{bmatrix}
q_{c,k} \\
\dot q_{c,k} \\
a_{c,k-1} \\
z_k \\
\eta_{m,k} \\
\eta_{c,k}
\end{bmatrix},
\]

where delay queues `eta` are included only when identified delay is nonzero.

### 12.2 Control

Control remains counter acceleration:

\[
u_k=\ddot q_{c,k}.
\]

Dynamics update:

\[
q_{c,k+1}=q_{c,k}+\Delta t\dot q_{c,k}
+\frac{1}{2}\Delta t^2u_k,
\]

\[
\dot q_{c,k+1}=\dot q_{c,k}+\Delta t u_k,
\]

\[
a_{c,k}=u_k,
\]

plus the complete delayed base-response transition.

### 12.3 Slew and Braking

`SolverBoxFDDP` enforces independent acceleration bounds.

Horizon-wide acceleration change is represented as a cost using `a_{c,k-1}`.
It is a hard post-solve validator, not incorrectly described as a Box-FDDP
control bound.

Retain:

- Exact first-action slew bound.
- Position, velocity, acceleration, and excursion validation at every knot.
- Braking-viability residual and hard validation.
- Full collision validation.

Document and test complete `calc`, `calcDiff`, terminal derivatives, and state
dimensions. Validate against `ActionModelNumDiff`.

### 12.4 Braking and Handoff Viability

For each joint, use effective excursion bounds and an identified conservative
command-delay/braking model.

Do not use an instantaneous scalar stopping-distance formula. Evaluate a
discrete worst-case braking rollout from:

\[
[q_i,\dot q_i,a_{i,previous}].
\]

During identified command delay, propagate the worst admissible outward
acceleration. After the delay, change acceleration toward conservative braking at
the maximum allowed slew, respect acceleration bounds, and propagate position
and velocity until stopped or the handoff horizon expires.

Braking is viable only when every rollout state respects position/excursion
bounds, velocity bounds, collision margins, and the final velocity tolerance.
This correctly handles zero current velocity with outward acceleration and
slew-limited acceleration reversal.

Handoff uses two phases:

1. `BRAKE`: reduce counter velocity with bounded acceleration/slew while
   preserving stopping-distance, excursion, and collision margins.
2. `RECONNECT`: once stationary double-support validity is restored, solve a deterministic
   per-joint bounded-acceleration plan over at most `0.5 s` to the moving
   upstream counter position/velocity reference.

The reconnect planner includes acceleration and slew constraints in its state,
then validates the complete joint/collision rollout. It tracks the latest
receding `0.5 s` upstream reference. Position, velocity, acceleration, and torque
must remain inside frozen completion tolerances for a dwell before exact
pass-through.

Counter torque transitions from candidate gravity synthesis to the aligned
upstream torque horizon under an identified torque-rate limit. Validate the first exact upstream sample
for position/velocity/acceleration slew, torque slew, excursion, and collision.

If no feasible braking command exists, trigger automatic global estop. Do not
publish an immediate zero-velocity command and call it bounded handoff. While
stationary double-support validity is lost, do not attempt posture return or
upstream reconnection; perform only a validated conservative braking command.

Candidate acceptance, support-validity-transition braking, and fault fallback use
the same validated braking rollout implementation.

---

## 13. Objective

Initial base-aware cost:

\[
\ell_k
=
\frac{1}{2}e_{\theta,k}^TQ_\theta e_{\theta,k}
+
\frac{1}{2}\omega_k^TQ_\omega\omega_k
+
\frac{1}{2}c_{s,k}^TQ_c c_{s,k}
+
\frac{1}{2}\dot c_{s,k}^TQ_{\dot c}\dot c_{s,k}
+
\frac{1}{2}q_h(h_k,\dot h_k)
+
\ell_{arm,k}
+
\ell_{viability,k}.
\]

`ell_arm` contains posture, velocity, acceleration, and acceleration-change
regularization.

Do not simultaneously restore the old momentum/CoM objective at high weight.
Keep those terms diagnostic initially. Add one regularizer only through an
ablation.

Use a terminal base-state and braking-capacity objective. Do not force counter
velocity to zero while response remains unsafe.

### 13.1 Physical Acceptance Functional

Optimization cost and intervention acceptance are separate.

Define `J_physical` using only:

- Base tilt and angular-velocity risk.
- Support-relative CoM position/velocity risk.
- Height and vertical-velocity risk.
- Braking-capacity margin.

Arm posture, acceleration, slew, and nominal-command regularization guide the
optimizer but do not count as predicted physical improvement.

Candidate acceptance requires worst-case `J_physical` improvement above the
frozen margin. It also requires no worst-case worsening beyond frozen tolerance
for tilt, height, support-CoM, or braking components individually, and no
component safety-margin crossing under any accepted model.

Proprioceptive double-support validity is an activation/transition gate in
Iteration 4A. It is not included in the counterfactual candidate-versus-baseline
cost. Iteration 4A does not require an action-conditioned contact/load predictor.

---

## 14. Robust Pass-Through Comparison

For every tick:

1. Roll out the exact upstream counter command as the baseline.
2. Solve the common-model candidate.
3. Validate candidate commands and model trust region.
4. Roll the candidate through a calibrated split/bootstrap model set.
5. Compare candidate and pass-through predicted physical cost.

Apply only when:

\[
\min_{m\in\mathcal{M}}
\left(J_{pass,m}-J_{candidate,m}\right)
>\epsilon_J.
\]

Uncertainty must be calibrated on the action-conditioned uncertainty-calibration
partition and tested once on the untouched uncertainty-test partition. Bootstrap
spread alone is not an out-of-distribution guarantee.

Reject candidate action when:

- State/action exits identification support.
- Uncertainty coverage is invalid.
- Improvement decision changes under residual error bounds.
- Existing safety validation fails.

---

## 15. Support Validity, Recovery, and Failure

Implement support-validity transition and bounded handoff before active control.

Modes:

| Mode | Behavior |
|---|---|
| `PASS_THROUGH` | Exact upstream command. |
| `MONITOR` | Model observes; no counter correction. |
| `ACTIVE` | Robust predicted benefit while proprioceptive double support is valid. |
| `RECOVERY` | Manipulation ended; response still unsafe. |
| `HANDOFF` | Bounded convergence to upstream command. |
| `FAULT_HOLD` | Global or counter-side safety fault. |

Failure behavior is state-dependent:

- In `PASS_THROUGH` or `MONITOR`, model/solver failure preserves exact upstream
  command.
- In `ACTIVE` or `RECOVERY`, failure enters bounded braking/handoff from the last
  accepted command.
- When stationary double-support validity is lost, stop increasing reaction and
  enter bounded handoff. Do not issue immediate zero velocity.
- Global estop retains global authority.

Specify acceleration, slew, collision, excursion, and completion tolerances for
handoff before active experiments.

---

## 16. Shadow and Active Evidence

Shadow mode can validate only:

- Pass-through response prediction.
- Model coverage/trust region.
- Timing and rejection.
- Candidate command feasibility.

It cannot validate candidate counterfactual response because the candidate is
not applied.

Candidate causal accuracy requires separately randomized low-authority applied
trials after support-validity/handoff safety approval.

---

## 17. Implementation Sequence

### Stage 0: Freeze Contracts

1. Freeze B0 code/config hashes.
2. Freeze corrected target-level classification ledger and code revision.
3. Add base/counter/support observation schemas and timestamp tests using only the
   real-compatible observation contract.
4. Collect passive foot-pose/twist distributions and freeze the proprioceptive
   stationary double-support validity thresholds.
5. Validate support-validity hysteresis, debounce, latency, and stale behavior on
   held-out standing and step-initiation trajectories.
6. Implement a conservative provisional braking validator from retained limits
   and worst-case delay/effectiveness bounds. Add automatic estop fallback.
7. Implement loss-of-support-validity transition and provisional bounded handoff.
8. Only after steps 3–7 pass, run guarded no-manipulation identification for
   command delay, realized braking effectiveness, acceleration, slew, and torque
   transition limits.
9. Update the shared braking rollout with identified conservative parameters and
   revalidate it.
10. Validate low-authority physical braking/handoff across double-support to
    step-initiation trajectories.

Counter excitation is blocked until every Stage 0 support-validity/handoff gate
passes.

### Stage 1: Identification

1. Collect passive manipulation data for `M0/M1`.
2. Run randomized counter/zero-pulse excitation.
3. Fit `M2` and identify delays.
4. Run mirrored-side and matched-state policy tests.
5. Stop if the common model fails.

### Stage 2: Shadow Iteration 4A

1. Implement pure model loading/prediction.
2. Implement H3 action model and derivatives.
3. Run pass-through shadow prediction.
4. Verify timing, trust region, and model coverage.

### Stage 3: Randomized Low-Authority Validation

1. Apply preregistered candidate/zero actions from saved states.
2. Validate action-conditioned counterfactual prediction.
3. Freeze model uncertainty and improvement margin using only the dedicated
   uncertainty-calibration partition.
4. Evaluate calibrated coverage once on the untouched uncertainty-test
   partition. Do not retune after inspection.

### Stage 4: Compact Active Controller

1. Run historical cases as development/regression sentinels.
2. Execute every preregistered repetition of new untouched behavioral targets as
   one batch after complete freeze.
3. Inspect every changed outcome, ALMI stumble, and fall video.

### Stage 5: Full Regression

1. Run paired frame, B0, and Iteration 4A focused panels.
2. Use at least five repetitions for falls/stumbles/stochastic cases.
3. Run 100-target panels only after focused gates pass.

H5 and contact-mode prediction are separately preregistered follow-on work only
after Iteration 4A exposes a specific limitation.

---

## 18. Promotion Gates

Require all conditions:

- Lower total FAME majority fall count than paired B0.
- At least two repeated FAME fall-to-survival conversions.
- Target of three repeated rescues, matching Iteration 2.
- Lower total ALMI majority stumble count than paired B0.
- At least one repeated ALMI stumble-to-drift/stable conversion.
- No new majority fall on any case.
- No worse majority severity transition on any development, guard, or untouched
  case.
- No adverse foot lift, displacement, height, or contact trend hidden by an
  unchanged class.
- Preserve B0 manipulation tracking and drift/stable improvements.
- Manipulation position, velocity, and torque pass through to tolerance.
- Pre-publication p99 at most `15 ms` and maximum below `20 ms`.
- No active command accepted after timing violation.

Videos are mandatory for every changed outcome, every ALMI stumble, and every
fall.

---

## 19. Model Artifact

Store:

- Schema and code version.
- Frames, units, feature names, and state order.
- Control period and delays.
- Normalization.
- Dynamics and interaction matrices.
- Uncertainty/residual model.
- Identification support/trust region.
- Training/model-selection/behavioral split hashes.
- Source run manifests.
- Per-axis no-action/action one-step and rollout metrics.

The controller refuses an incompatible artifact.

---

## 20. Stop Conditions

Stop active Iteration 4A when:

- Randomized counter action has no repeatable causal base effect.
- No policy-blind observed-state model passes held-out gates.
- Candidate improvement is not robust under calibrated residuals.
- FAME rescue requires ALMI or stable-case regression.
- Stumble suppression converts protective steps to falls.
- Contact, handoff, safety, or real-time contracts cannot be met.

Do not respond with policy-specific models or target-specific thresholds.

If the common base model is valid but counter authority is insufficient, the
next architecture requires lower-body/whole-body coordination and is outside
this controller.

---

## 21. Expected Behavior

### FAME Difficult Motion

Pass-through predicts base divergence and reduced height margin. Counter action
predicts robust improvement across the uncertainty set. The controller applies
early bounded reaction and preserves braking capacity.

### ALMI Stable Motion

Pass-through predicts stable closed-loop response. Counter action has little or
uncertain benefit. The controller publishes exact pass-through.

### ALMI Stumble Motion

Before step initiation, pass-through predicts growing base/CoM divergence.
Counter action predicts robust stabilization. The controller reacts while
proprioceptive double support remains valid. If foot kinematics indicate step
initiation, it brakes and hands off rather than fighting it.

---

## 22. Design Summary

Iteration 4A tests one consistent decision:

> Apply counter-arm action only when a common, causally identified physical model
> predicts robust short-horizon improvement over the exact upstream command.

The design uses real-compatible proprioceptive/model-derived state, not policy
identity or simulator-only contact information. It requires causal randomized
identification, exact pass-through comparison, explicit delay state, complete
Crocoddyl dynamics/derivatives, bounded handoff, and strict case-level no-regression
gates before active promotion.
