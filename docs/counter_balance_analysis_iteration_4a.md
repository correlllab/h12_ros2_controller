# Counter-Balance Iteration 4 Analysis

## Status

Iteration 4 has completed Stage-0 real-compatible observation validation and the
first passive M0/M1 identification cycle.

No active counter excitation, M2 model, or Crocoddyl Iteration-4 controller has
been enabled. The passive common-model gate did not pass, so later stages remain
blocked by the source-of-truth design.

Iteration 3 B0 remains unchanged and independently runnable.

## 1. Implemented Infrastructure

### 1.1 Atomic LowState Snapshot

The state subscriber now preserves:

- DDS tick and unwrapped tick.
- Monotonic arrival time and compatibility wall time.
- Sequence number, received state, and tick validity.
- One locked copy of IMU and 27 motor `q/dq/ddq/tau_est` values.

Duplicate fresh ticks are accepted. Backward/out-of-order ticks are invalid.

### 1.2 Real-Compatible Observations

Implemented immutable observations for:

- Measured counter position and velocity.
- Base orientation error and angular velocity.
- Support-relative CoM position and velocity.
- Support-relative base height and vertical velocity.
- Left/right foot pose and twist from Pinocchio.
- Canonical support frame and support twist.
- Proprioceptive stationary-double-support validity.
- Moving-arm centroidal momentum and momentum rate.

All controller-facing data comes from IMU, measured joint state, or Pinocchio.
MuJoCo contact force, contact flags, exact base state, and external wrench are not
used.

### 1.3 Observer Variant

`counter_base_observer` executes ordinary frame-task commands and records the
Iteration-4 observation bundle. It does not control the counter arm.

### 1.4 Provisional Braking and Reconnect

Pure safety primitives were implemented and unit-tested:

- Discrete delay/slew/acceleration-limited worst-case braking rollout.
- Conservative braking effectiveness.
- Quintic reconnect trajectory with position, velocity, acceleration, slew,
  excursion, and torque-rate checks.

These primitives are not connected to an active controller. Physical handoff
validation remains incomplete, so counter excitation is blocked independently
of the model result.

## 2. Stage-0 Observation Experiments

### 2.1 Initial Reference Failure

The first observer captured support reference from the first received sample.
That sample preceded the settled release state and created large false residuals,
including approximately `0.32 m` relative-foot translation in stable FAME.

Decision: reject first-message reference capture. Capture explicitly at
manipulation release, with retry on the next fresh sample.

### 2.2 Timestamp Failure

The runtime initially evaluated sample age against `loop_started`. A LowState
callback arriving later in the same loop produced a negative age and false stale
classification.

Decision: evaluate age at the moment the immutable snapshot is fetched.

After correction, held-out ALMI stable observation age was:

- Median: approximately `4.3 ms`.
- P99: approximately `8.5 ms`.
- Maximum: approximately `11.5 ms`.

### 2.3 Support-Validity Calibration

Three repeated development cases per class were used. Entry thresholds were
frozen from pooled stable/pre-event p99 distributions:

| Residual | Entry threshold |
|---|---:|
| Foot height | `0.03 m` |
| Foot orientation | `0.18 rad` |
| Foot linear twist | `0.14 m/s` |
| Foot angular twist | `0.65 rad/s` |
| Relative foot translation | `0.09 m` |
| Relative foot rotation | `0.06 rad` |

Exit thresholds use 1.5 times entry thresholds, with five-sample entry and
two-sample exit debounce.

### 2.4 Held-Out Support Results

Held-out ALMI stable:

- Entered stationary-support validity at `0.08 s`.
- Valid for approximately `95%` of manipulation.
- Remained valid through recovery.

Held-out ALMI stumble:

- Exited validity at `1.287 s`.
- Offline MuJoCo audit measured first foot lift at `1.575 s`.
- Proprioceptive lead time: approximately `288 ms`.

Held-out FAME fall:

- Exited validity at `2.695 s`.
- Offline fall-pending audit began at `3.82 s`.
- Proprioceptive lead time: approximately `1.1 s`.

Privileged MuJoCo signals were used only for offline timing audit.

Decision: retain the real-compatible support-validity hypothesis. Active control
remains blocked until braking/handoff is physically validated.

## 3. Passive M0/M1 Identification

### 3.1 Dataset

Passive frame-task data was collected on FAME and ALMI fall/stumble/stable
families with repeated complete runs.

The state contains:

- Roll/pitch error and angular velocity.
- Support-relative CoM position and velocity.
- Support-relative base height and vertical velocity.

The M1 disturbance contains moving-arm planar momentum and momentum rate.

Left/right samples use a fixed canonical mirror transform. Model features contain
no policy label.

Training and held-out families remain complete-run separated.

### 3.2 Exploratory Estimator Result, Superseded

The initial CoM/height velocities used finite differences and were noisy. A
moving-support formula was explored, but independent review showed that roll rate
and lateral CoM velocity were internally inconsistent. The metrics in this
section are retained as experiment history and superseded by Section 6.

### 3.3 Balanced Repeated Result

Artifact:

`runs/key_findings_reports/iteration4_models/m01_passive_v2_context_report.json`.

M1 `100 ms` held-out metrics include:

| Axis | R2 | NRMSE | Absolute RMSE |
|---|---:|---:|---:|
| Roll error | `0.997` | `0.055` | `0.0020 rad` |
| Pitch error | `0.997` | `0.050` | `0.0016 rad` |
| Roll rate | `0.593` | `0.638` | `0.0108 rad/s` |
| Pitch rate | `0.964` | `0.190` | `0.0092 rad/s` |
| CoM X | `0.997` | `0.058` | `0.0011 m` |
| CoM Y | `0.994` | `0.077` | `0.0016 m` |
| CoM dX | `0.768` | `0.482` | `0.0114 m/s` |
| CoM dY | `0.020` | `0.990` | `0.0213 m/s` |
| Base height | `0.999` | `0.034` | `0.0003 m` |
| Base dH | `0.921` | `0.281` | `0.0023 m/s` |

M1 improves pitch-rate and sagittal CoM-velocity prediction over M0, but fails
the frozen per-axis NRMSE gate on roll rate and lateral CoM velocity.

Per-policy mean rollout R2 was approximately:

- ALMI: `0.679`.
- FAME: `0.810`.

### 3.4 Physical Context Alternatives

One real-compatible feature was tested at a time.

Support-twist context (`M1S`):

- Improved vertical velocity and pooled mean performance.
- Worsened roll-rate and lateral CoM-velocity gates.

IMU-acceleration context (`M1A`):

- Improved roll-rate R2 to `0.653`.
- Roll-rate NRMSE remained `0.589`.
- Lateral CoM-velocity NRMSE remained `0.997`.

Neither context passes the common-model gate.

## 4. Exploratory Evidence Decision, Superseded

The exploratory model did not pass every state-axis gate. Independent review
required observation/timing corrections and a new dataset before final decision.

Therefore:

- Do not run randomized counter excitation.
- Do not fit M2.
- Do not implement the Crocoddyl Iteration-4 action model.
- Do not loosen model gates after seeing held-out results.
- Do not add policy-specific models.

The next allowed experiment is a preregistered redesign of the lateral
support-relative velocity state/estimator or response representation, followed
by new family-grouped model-selection data. Until that redesign is documented,
Iteration 4 remains paused at the M1 gate.

This is an evidence stop, not a completed controller.

## 5. Independent Review Corrections

An independent review found that the initial passive-model evidence could not be
treated as final because:

- Torso gyro and pelvis orientation did not describe the same angular state.
- Model transitions admitted variable `9–31 ms` intervals while reporting a
  five-sample rollout as `100 ms`.
- Lateral CoM velocity was internally inconsistent with its position state.
- Support-twist context was not canonicalized.
- Braking validation omitted between-knot extrema and collision callbacks.

Corrections were implemented before the final decision:

- Response rates were derived from the same support-relative coordinates.
- Model transitions require `20 +/- 3 ms`; rollouts require `100 +/- 3 ms`.
- Support twist and arm side use canonical mirror transforms.
- IMU quaternion validity is strict.
- Tick recovery ignores rejected out-of-order packets.
- Braking validates the initial state, between-knot turning points, and collision
  callbacks.
- Reconnect validates all collision samples.
- Observation age/frame/twist metadata and foot rotations are logged.
- Family split assignments are hashed.

Observer computation overhead on the corrected pilot was approximately:

- Median: `0.72 ms`.
- P95: `1.80 ms`.
- P99: `2.28 ms`.
- Maximum: `3.32 ms`.

The observer remains passive, but a paired frame timing trial is still required
before calling it runtime-neutral.

## 6. Final Passive Model Evidence

### 6.1 Corrected State Pilot

Finite-difference consistency showed that the original roll-rate and lateral
CoM-velocity estimates were not self-consistent. A corrected state estimator and
new v3/v4 passive datasets were collected.

### 6.2 Model Variants

The following policy-blind models were evaluated on the final self-consistent v4
dataset:

- `M0`: response state/history only.
- `M1`: M0 plus moving-arm momentum/rate.
- `M1S`: M1 plus canonical support twist.
- `M1A`: M1 plus support-frame IMU acceleration.
- `S0/S1`: structured rate dynamics with exact trapezoidal position integration.

No counter excitation was applied.

Final artifact:

`runs/key_findings_reports/iteration4_models/m01_passive_v4_balanced_report.json`.

The balanced repeated M1 model failed the manipulation-feature gate. Its final
`100 ms` held-out metrics were:

| Axis | R2 | NRMSE |
|---|---:|---:|
| Roll error | `0.997` | `0.054` |
| Pitch error | `0.995` | `0.067` |
| Roll rate | `0.237` | `0.874` |
| Pitch rate | `0.688` | `0.558` |
| CoM dX | `0.726` | `0.523` |
| CoM dY | `-0.458` | `1.207` |
| Base dH | `0.818` | `0.427` |

M1 was worse than M0 on roll rate, pitch rate, both CoM-velocity axes, height
rate, and pooled ALMI performance. Support twist improved pitch/height-rate
prediction but not roll/lateral axes. IMU acceleration did not recover the failed
axes. Structured S0/S1 did not improve the common-model result.

### 6.3 Final Decision

The common passive response model does not pass every state-axis gate. The
failure persists across repeated data, canonicalization, alternative context,
and structured dynamics.

Therefore:

- Randomized counter excitation remains blocked.
- M2 is not fitted.
- No Iteration-4 Crocoddyl controller is implemented.
- No policy-specific model or relaxed post-hoc gate is introduced.

The next permissible redesign is a new preregistered response representation or
real-compatible estimator that specifically resolves roll-rate and lateral
support-relative velocity, followed by new family-grouped model-selection data.
Until then, Iteration 4 is paused at the M1 gate and B0 remains the active
comparison controller.
