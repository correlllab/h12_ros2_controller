# Frozen H2 and Robust H2 Controller Implementation

## 1. Purpose and Status

This document retains the frozen H2 mathematics and the **archived historical
Croc-first robust H2** implementation reference. As of 2026-09-09, the existing
robust runtime name instead selects a shared SciPy-primary nominal refactor
**source-frozen for numerical/implementation validation with a physical
limitation**. This is not unconditional physical nonregression or speed readiness.
See [Iteration 6B preparation](counter_balance_iteration_6b_preparation.md) for
the current source/API contract, completed gates, and evidence limits.

Unless explicitly marked current, the Croc-first robust descriptions, source
map, diagnostics, timing results, and verification requirements below refer to
archived controller revision `9256ab6`, not the current robust implementation.
The source/config archives and identity manifest are under benchmark-root
`runs/key_findings_reports/iteration6b_preparation/nominal_refactor/provenance/`.

Runtime identities are:

| Benchmark controller ID | Runtime variant | Python class | Status |
| --- | --- | --- | --- |
| `counter_residual_h2_frozen` | `counter_residual_h2` | `CounterResidualH2Controller` | Frozen Iteration-5 controller. |
| `counter_residual_h2_robust` | `counter_residual_h2_robust` | `CounterResidualH2RobustController` | Historical Croc-first behavior archived; current SciPy-primary source frozen with physical limitation. |

The benchmark controller ID is an overlay name defined by the benchmark sweep
configuration. The runtime variant selects a controller class. Therefore,
`counter_residual_h2_frozen` is not a separate class: it is the active,
non-shadow configuration of `CounterResidualH2Controller`.

Frozen H2 combines the frozen Iteration-3C nominal counter-arm planner with a
small, delay-aware residual model-predictive controller. Historical robust H2
changes only how a rejected nominal solve is handled. It does not change the H2
model, residual objective, authority, confidence gates, safety path, or
publication path.

Current robust H2 and new `counter_ddp_velocity_robust` share
`ScipyNominalMixin` and `scipy_nominal_planner.py`: BVLS primary plus at most one
TRF retry, each with `lsq_solver='exact'`, `tol=1e-10`, `max_iter=100`, exact
fixed-coordinate elimination, and independent original-bound/objective/KKT
checks (`5e-4`). Neither constructs a nominal OCP. Only the H2 variant loads H2
models and constructs the residual Crocoddyl OCP. Legacy 3C behavior remains
unchanged. The final manifest records `source_frozen=true`, 150 physical trials
(132 main + 8 health + 10 outer-timing), 39,359 numerical problems, and 12,964
first-repeat exact finalizer contexts. Current source archive
`current_source.tar.gz` has SHA-256
`9712a1dc1f6a3737e242301ae56c4b982099dfa00584212cb8cf6f281a460c8a` under the
benchmark-root provenance directory above. Old source/config archives remain.

**ALMI holds remain about 91.4% for both new paths versus 61.7% for old wide.**
Source comparison and identical-input replay prove the collision/hold trap is
inherited, but online entry probability remains unresolved. No objective,
collision, or hold tuning was made. The scoped freeze does not authorize ALMI
speed work; separate trajectory-bank preparation has not started.

The controller runs at `50 Hz`, with `dt = 0.02 s`. It is policy-blind,
checkpoint-blind, target-blind, and limited to real-compatible IMU, joint, and
Pinocchio-derived measurements.

## 2. Frozen and Historical Source Map

| Responsibility | Source file |
| --- | --- |
| Shared state preparation, frozen-3C planning, finalization, and publication | `h12_ros2_controller/core/controller/counter_balance/counter_balance_controller.py` |
| One-step Crocoddyl frozen-3C nominal solve | `h12_ros2_controller/core/controller/counter_balance/counter_ddp_velocity_controller.py` |
| Frozen-3C pure planner records and target construction | `h12_ros2_controller/core/controller/counter_balance/frozen_3c_planner.py` |
| Frozen H2 orchestration and model-validity gates | `h12_ros2_controller/core/controller/counter_balance/counter_residual_h2_controller.py` |
| H2 dynamics, costs, derivatives, and Box-FDDP solve | `h12_ros2_controller/core/controller/counter_balance/counter_residual_h2_ocp.py` |
| U5, R5, and N5 model definitions | `h12_ros2_controller/core/controller/counter_balance/residual_response_model.py` |
| Frozen fitted model parameters | `h12_ros2_controller/core/controller/counter_balance/verified_response_parameters.py` |
| Historical robust nominal fallback at `9256ab6`, not current file behavior | `h12_ros2_controller/core/controller/counter_balance/counter_residual_h2_robust_controller.py` |
| Shared nominal least-squares objective and historical SciPy fallback | `h12_ros2_controller/core/controller/counter_balance/objective.py` |
| One-step nominal Crocoddyl OCP and acceptance test | `h12_ros2_controller/core/controller/counter_balance/counter_velocity_ocp.py` |

The source-bound model report hashes are:

- Response report: `63c33d45f507cf454fad0be2e387022b2bc630458a051a00322db0ffd6fc495a`.
- Nominal report: `e8a513e24e2585452e0c2cb4bc3acff9795ffb9f9b47db4bce75295eee301ba6`.
- Frozen planner/model/H2/harness/config bundle:
  `258ccac38b325a2158f44da6aa506526e6046468b98bcb58381e0b48b5540e9a`.

## 3. Shared Command Architecture

Both variants publish through the same command pipeline:

\[
u_{requested}
=
u^{3C}_{nominal}+\delta u^{H2}.
\]

Only the requested velocity is composed at this point. Position integration,
collision backtracking, applied-velocity accounting, gravity compensation,
command-state mutation, and publication happen once in the shared finalizer.

One control tick executes the following sequence:

1. Verify arm ownership and validate `balance_scale`.
2. Reject active control immediately when the low-command handler is estopped.
3. Validate the 14-joint moving-arm position and velocity samples.
4. Read measured motor position and update the Pinocchio robot model.
5. Re-read measured motor position after the model update.
6. Compute support, CoM, CoM Jacobian, centroidal momentum map, and torso
   rotation from one prepared model state.
7. Capture the counter-arm, wrist, CoM-offset, and torso references if they have
   not already been captured.
8. Hold the counter arm when support or reference capture is invalid.
9. Compute lifecycle activation and the effective balance scale.
10. Split the CoM Jacobian and momentum map into moving-arm and counter-arm
    blocks.
11. Intersect robot, publisher, controller, and frozen excursion velocity
    bounds.
12. Build and solve the pure frozen-3C nominal velocity problem.
13. Run the H2 residual selector using the accepted nominal plan.
14. Add the selected residual unless the controller is in shadow mode.
15. Reject a nonfinite or incorrectly shaped combined request.
16. Collision-backtrack the complete arm candidate.
17. Compute gravity compensation and atomically publish one arm command.
18. Record the actually applied residual for the next tick's one-tick-delay
    state.

There is no second publication path for H2 or robust H2.

## 4. Frozen-3C Nominal Planner

### 4.1 Inputs and Targets

The pure planner receives copies of all required values through
`Frozen3CNominalInput`. It has no collision, publication, reference-capture, or
final command-state side effects.

For moving-arm velocity \(\dot q_m\), moving-arm CoM Jacobian \(J_m\),
moving-arm planar momentum map \(A_m\), CoM error \(e_c\), planar gyro
measurement \(\omega\), and effective lifecycle scale \(b\), the frozen targets
are:

\[
r_c=b(-J_m\dot q_m-k_c e_c),
\]

\[
r_h=b(-A_m\dot q_m+k_g\omega),
\]

\[
r_q=-k_q(q_c-q_{c,ref}).
\]

The retained sweep configuration uses:

| Quantity | Value |
| --- | ---: |
| CoM gain `k_c` | `2.0` |
| Gyro gain `k_g` | `0.2` |
| Posture gain `k_q` | `1.0` |
| CoM weight | `1.0` |
| Momentum weight | `2.0` |
| Posture weight | `0.02` |
| Damping weight | `0.0001` |
| Maximum four-joint velocity | `[2.6, 3.2, 2.6, 1.5] rad/s` |
| Maximum reference excursion | `[0.75, 0.58, 0.40, 0.58] rad` |

The normalized bounded least-squares problem stacks CoM, momentum, posture, and
damping blocks. For each positive term weight \(w_i\), the implementation adds
\(\sqrt{w_i}M_i\) to the objective matrix and \(\sqrt{w_i}y_i\) to the target.
The CoM, momentum, and posture blocks use their configured physical scales.

### 4.2 Nominal Bounds

The nominal lower and upper velocity bounds are the intersection of:

- Pinocchio joint position limits converted to one-tick velocity limits.
- Pinocchio velocity limits.
- Controller `dq_lim`.
- Optional publisher position and velocity clipping limits.
- Frozen per-joint `max_velocity`.
- Frozen excursion around the captured counter-arm reference.

An empty interval raises `CounterVelocityBoundsError` and selects a counter hold.

### 4.3 Frozen and Historical Primary Crocoddyl Solve

`CounterVelocityOCP` represents the four-joint least-squares problem as one
running knot and a zero-cost terminal knot. Its implementation:

- Uses `crocoddyl.SolverBoxFDDP` with up to `100` iterations.
- Tries initial regularization values `1e-9`, `1e-6`, and `1e-3` until the
  Box-FDDP call completes.
- Runs Crocoddyl `BoxQP` polishing on the same quadratic and bounds.
- Clips the polished result to the original, unexpanded bounds.
- Computes the projected-gradient KKT violation.
- Accepts only finite, in-bounds output with finite cost and KKT violation no
  greater than `5e-4`.

Box-FDDP convergence is diagnostic. The final acceptance decision is based on
the polished command validity and KKT test.

### 4.4 Frozen H2 Versus Historical Robust H2

Frozen H2 requires the primary nominal result to be accepted. A rejected result
causes the shared controller to publish a counter hold without invoking H2.

Historical robust H2 first runs this exact primary path. If the primary returns
an accepted result, historical robust H2 returns that same
`Frozen3CVelocitySolve` object and remains command-identical to frozen H2.

If the primary returns a result with `accepted = false`, robust H2 rebuilds the
same normalized matrix and target and calls SciPy `lsq_linear` with the same
lower and upper bounds. A successful fallback is wrapped as an accepted nominal
plan and then passed through the unchanged H2 residual and finalization paths.

The fallback is not entered when the primary raises an exception instead of
returning a rejected result. Primary exceptions, fallback exceptions, empty
bounds, unsuccessful SciPy status, or later nonfinite-command validation select
the existing shared hold behavior.

## 5. H2 Runtime Observations

The residual controller obtains the following real-compatible values each tick:

- IMU quaternion, converted to roll and pitch.
- IMU planar angular rate from the prepared counter-command context.
- Measured 27-joint velocity, indexed to the four active counter joints.
- Measured four-joint counter position.
- Counter-arm planar centroidal momentum map.
- Moving-arm momentum generated by the commanded moving-arm velocity.
- Nominal counter momentum generated by the frozen-3C velocity request.
- Moving-arm side and ownership.

The model is expressed in a canonical arm frame. The conversion is:

| Moving arm | Side feature | Roll/pitch canonical sign |
| --- | ---: | --- |
| Left | `+1.0` | `[-1.0, +1.0]` |
| Right | `-1.0` | `[+1.0, +1.0]` |

The signs are applied to tilt, angular rate, moving momentum, nominal momentum,
and rows of the counter momentum map. This lets one fitted model serve both arm
ownership cases without using policy or target identity.

The first H2 tick cannot form a two-sample nominal trend. It duplicates the
current tilt and rate into the two-step nominal arrays, marks both confidence
axes false, and abstains.

## 6. Frozen U5, R5, and N5 Models

### 6.1 U5 Residual Realization

U5 predicts the realized residual counter velocity after the measured one-tick
delay. For joint \(i\), its local gain is:

\[
g_i=
\beta_{i,0}
+\beta_{i,1}\frac{q_i-\mu_{q,i}}{s_{q,i}}
+\beta_{i,2}\frac{\dot q_i-\mu_{\dot q,i}}{s_{\dot q,i}}
+\beta_{i,3}\frac{side-\mu_{side,i}}{s_{side,i}}.
\]

The realized residual used by the H2 dynamics is
\(\dot q^{residual}_{realized}=g\odot\delta u_{pending}\).

The frozen coefficient rows are:

```text
joint 0: [0.9490105768, -0.0658536868,  0.0869843002,  0.0124430073]
joint 1: [0.8909987680, -0.0165840186, -0.0229854765,  0.0022244144]
joint 2: [0.2861639831,  0.0835486982, -0.0429719959, -0.0492074923]
joint 3: [1.2978231505,  0.1122764133,  0.0184565563,  0.0275903975]
```

The corresponding `[q, dq, side]` centers are:

```text
joint 0: [-0.1000114936, -0.4950190172, -0.25]
joint 1: [ 0.0242737426, -0.0006133704, -0.25]
joint 2: [-0.0103821434, -0.1043122198, -0.25]
joint 3: [-0.0338343340,  0.1104686077, -0.25]
```

The corresponding scales are:

```text
joint 0: [0.2727851331, 0.8428735131, 0.9682458366]
joint 1: [0.2339522031, 0.5372700835, 0.9682458366]
joint 2: [0.1246125076, 0.7220221880, 0.9682458366]
joint 3: [0.1212081238, 0.6272422092, 0.9682458366]
```

Joint index `2` is a verified weak direction. Its gain and residual bounds are
forced to exactly zero. Only residual joints `0`, `1`, and `3` are active.

### 6.2 R5 Incremental Angular Response

R5 evaluates one full `2 x 2` planar response matrix from canonical tilt, rate,
and side:

\[
c=[\theta_{roll},\theta_{pitch},\omega_{roll},\omega_{pitch},side],
\]

\[
G_{o,i}=\sum_f \gamma_{o,i,f}
\left[1,\frac{c-\mu_c}{s_c}\right]_f.
\]

The H2 dynamics then computes:

\[
\Delta\omega=
diag(confidence)G A_c\dot q^{residual}_{realized}.
\]

The context center is:

```text
[-0.0026888288, -0.0180479750, -0.0038559450, 0.0039032566, -0.25]
```

The context scale is:

```text
[0.0185574700, 0.0180406835, 0.0248585909, 0.0264255293, 0.9682458366]
```

The frozen coefficient tensor is indexed as
`[output axis, momentum input axis, feature]`:

```text
[
  [
    [-0.0795483493, -0.0076843958,  0.0085363948,
     -0.0081429244,  0.0091322067, -0.0045436524],
    [ 0.0599910551, -0.0504897723,  0.0082771796,
     -0.0013440024, -0.0031848232, -0.0129863464]
  ],
  [
    [-0.1151754105, -0.0247930702, -0.0143849748,
      0.0086097111,  0.0156945944, -0.0405895526],
    [-0.4192567749,  0.0445474214, -0.0094182728,
     -0.0060772695,  0.0231229711, -0.0126756206]
  ]
]
```

The runtime holds U5 and R5 gains fixed for one solve and reevaluates them from
the next measured state on the next control tick.

### 6.3 N5 Nominal Phase Prediction

N5 predicts the absolute frozen-3C tilt and rate trajectory against which H2
evaluates residual effects. It uses current and previous canonical tilt/rate,
the change in moving-arm momentum, and the change in nominal counter momentum.

For each of the two prediction steps:

\[
\hat\omega_{j}=
\hat\omega_{j-1}
+k_{trend}(\hat\omega_{j-1}-\hat\omega_{j-2})
+K_m\Delta h_m
+K_n\Delta h_n.
\]

The first trend is the measured
`current_rate - previous_rate`. The current momentum changes are duplicated for
both horizon steps. Tilt is integrated trapezoidally from the current measured
tilt and rate.

The frozen trend gain is:

```text
[0.0880932504, 0.2947651595]
```

The moving-momentum gain is:

```text
[[ 0.1514740573, -0.0015434027],
 [-0.0597397224, -0.0338645194]]
```

The nominal-counter-momentum gain is:

```text
[[0.1152594431,  0.0666321951],
 [0.0595378434, -0.0956495333]]
```

N5 is valid only when all calibrated limits pass:

| Check | Roll | Pitch |
| --- | ---: | ---: |
| Maximum absolute tilt | `0.0504572459 rad` | `0.0492336663 rad` |
| Maximum absolute rate | `0.0897511059 rad/s` | `0.1942660672 rad/s` |
| Maximum one-tick tilt change | `0.0139536985 rad` | `0.0040712854 rad` |
| Maximum one-tick rate change | `0.0733068066 rad/s` | `0.1120722657 rad/s` |
| Maximum tilt integration error | `0.0135959781 rad` | `0.0020632344 rad` |
| Sign deadband | `0.001` | `0.001` |

The accepted sample period range is `[0.016, 0.025] s`.

### 6.4 Confidence and Domain Gates

N5 phase confidence is evaluated independently per planar axis:

\[
|\omega_k|>
e_{N5}+2|\omega_k-\omega_{k-1}|.
\]

The frozen N5 error bound is
`[0.0450215641, 0.0160736980] rad/s`. An axis outside this no-crossing condition
is removed from R5 response and terminal angular costs. The controller may run
when at least one axis is confident; both axes are not required.

The complete residual model is valid only when:

- N5 passes every calibrated validity check.
- At least one roll/pitch confidence gate passes.
- Every U5 `[q, dq, side]` normalized feature for joints `0`, `1`, and `3` is
  within `context_limit = 3.0`.
- Every R5 normalized context feature is within `context_limit = 3.0`.
- U5 gains for joints `0`, `1`, and `3` are strictly positive.

If any condition fails, all H2 lower and upper bounds are set to zero. The
controller still obtains deterministic zero-residual solver diagnostics, but it
publishes the frozen-3C nominal command and labels the decision `abstain`.

## 7. Residual H2 OCP

### 7.1 State, Control, and Delay

The fixed state has 12 elements:

\[
x=
[\Delta\theta_{2},\Delta\omega_{2},\Delta q_{c,4},u_{pending,4}].
\]

These are residual increments around the N5 nominal trajectory, not the full
robot state. At the start of a solve, every state element is zero except
`u_pending`, which contains the residual actually applied on the preceding
tick.

The control is a four-joint requested residual velocity. There are two running
knots and one terminal knot:

\[
\delta U=[\delta u_0,\delta u_1].
\]

The stage-0 dynamics only write \(\delta u_0\) into the pending-action block.
The stage-1 dynamics realize the previous pending request:

\[
v_r=g_{U5}\odot u_{pending},
\]

\[
\Delta h=A_c v_r,
\]

\[
\Delta\omega=diag(confidence)G_{R5}\Delta h,
\]

\[
\Delta\theta\leftarrow
\Delta\theta+0.5\,dt\,\Delta\omega,
\]

\[
\Delta q_c\leftarrow\Delta q_c+dt\,v_r.
\]

Stage 1 then stores \(\delta u_1\) as pending. Because the horizon ends at that
point, \(\delta u_1\) affects regularization but does not produce a modeled
physical response. With the one-tick realization delay, H2 therefore has one
effective terminal residual action.

### 7.2 Residual Bounds

For each joint, H2 intersects the frozen trust bound with the unused nominal
velocity interval:

\[
l_{H2}=\max(-v_{trust},l_{nominal}-u^{3C}_{nominal}),
\]

\[
u_{H2}=\min(v_{trust},u_{nominal}-u^{3C}_{nominal}).
\]

The active frozen and robust variants use:

```text
trust_velocity: [0.01, 0.01, 0.0, 0.01] rad/s
```

Joint `2` is set to `[0, 0]` after the intersection. Internally, exactly fixed
Box-FDDP bounds are expanded by `1e-10` for solver compatibility, while returned
controls are validated against the original bounds and joint `2` must remain
within `1e-8` of zero.

### 7.3 Running Cost

At each running knot:

\[
\ell_k=
\frac{1}{2}
\left(
w_u\frac{\|u_k\|^2}{0.1^2}
+w_{\Delta u}\frac{\|u_k-u_{pending}\|^2}{0.1^2}
\right).
\]

The retained active weights are:

| Weight | Value |
| --- | ---: |
| Residual action `w_u` | `0.01` |
| Residual change `w_du` | `0.005` |

The lower regularization is the only retained tuning change from the initial
shadow configuration. The model, horizon, trust region, and safety path were not
changed.

### 7.4 Terminal Cost

The terminal model forms absolute predicted values:

\[
\theta_N=\hat\theta^{N5}_N+\Delta\theta_N,
\qquad
\omega_N=\hat\omega^{N5}_N+\Delta\omega_N,
\]

and positive divergence:

\[
d_N=\max(\theta_N\odot\omega_N,0).
\]

Its exact cost is:

\[
\ell_N=\frac{1}{2}\left(
2.0\frac{\|C\theta_N\|^2}{0.05^2}
+1.0\frac{\|C\omega_N\|^2}{0.2^2}
+2.0\frac{\|C d_N\|^2}{0.01^2}
+0.1\frac{\|\Delta q_{c,N}\|^2}{0.1^2}
\right),
\]

where \(C\) is the diagonal boolean confidence mask. Counter-position reserve
is not confidence-masked.

`calcDiff()` provides analytical dynamics and cost derivatives. Unit tests
compare both running models and the terminal model against finite differences.

### 7.5 Solver and Acceptance

`ResidualH2OCP` owns one persistent two-knot problem and Box-FDDP solver. Each
tick updates model context and bounds in place.

The solve behavior is:

- Maximum iterations: `1`.
- Stopping threshold: `1e-9`.
- Initial regularization passed to `solve()`: `1e-6`.
- Cold seed: two zero controls.
- Warm seed: the previous solution's second control copied into both knots and
  clipped to current bounds.
- Initial state: zero increments plus the previously applied pending residual.

An H2 result is accepted when state and control arrays have shapes `(3, 12)` and
`(2, 4)`, all values are finite, all controls satisfy bounds within `1e-8`, and
joint `2` remains zero within `1e-8`. Crocoddyl convergence is not required:
accepted converged results use status `solved`, and accepted non-converged
results use status `best_effort`.

Solver exceptions produce `solver_failure`. Invalid arrays produce
`invalid_solution`. Either result returns an exact zero residual.

## 8. Residual Selection and Applied-State Feedback

The H2 solve returns the first control \(\delta u_0\). Frozen H2 selects:

```text
nominal                    when shadow = true
nominal + accepted residual when shadow = false
nominal                    when the model or H2 solve is invalid
```

Any exception in model preparation or H2 execution is caught as
`model_failure`, resets the H2 warm start, and selects the exact nominal command.

After shared collision backtracking, the next pending residual is reconstructed
from the command that was actually applied:

\[
u_{pending,next}
=u_{applied}-s_{backtrack}u^{3C}_{nominal}.
\]

Joint `2` is reset to zero. Shadow mode and every counter-hold path clear the
pending residual. This prevents the delay model from treating an unapplied or
collision-reduced request as realized authority.

The diagnostic decision label is based on the selected residual's projection
onto the nominal command:

- `abstain`: model invalid or residual norm no greater than `1e-8`.
- `continue`: residual dot nominal is nonnegative.
- `reverse`: the combined command changes the sign of any nonzero nominal
  joint.
- `brake`: residual opposes the nominal without changing a joint's sign.

These labels describe joint-space command relation. They are diagnostics, not a
separate state machine or proof of whole-body momentum direction.

## 9. Historical Robust H2 Difference

This section describes archived `9256ab6` only. Current robust H2 uses the
SciPy-primary mixin described in Section 1 and the 6B preparation reference;
the accepted-old-Croc result identity below is not its current API contract.

At archived revision `9256ab6`, `CounterResidualH2RobustController` inherits
the complete frozen H2 class and overrides the isolated nominal solve for
fallback, with per-tick reset and diagnostic hooks. This is not the current
SciPy-primary mixin implementation.

```text
run primary one-step Crocoddyl nominal solve
if primary.accepted:
    return the primary result unchanged
else:
    solve the same bounded least-squares problem with scipy.optimize.lsq_linear
    wrap the fallback velocity as an accepted Frozen3CVelocitySolve
continue through unchanged H2 and shared finalizer
```

The fallback result retains the primary Crocoddyl diagnostics and the primary
objective matrix and target. Robust-specific diagnostics add:

- `nominal_fallback_used`: true only when SciPy returned the nominal velocity.
- `nominal_primary_accepted`: acceptance state of the primary diagnostics.

The fallback flag and primary-result reference reset at the start of every
control tick. No fallback state changes future accepted-path behavior.

Robust H2 intentionally does not add:

- A larger residual trust region.
- New H2 costs, states, or model coefficients.
- Policy, checkpoint, target, or outcome gates.
- Simulator-only measurements.
- A second finalizer or publisher.
- Additional FAME fall-rescue authority.

## 10. Frozen and Historical Failure Behavior

| Condition | Published behavior |
| --- | --- |
| Estop already active | Return zero without starting a control solve. |
| Robot-model update failure | Hold measured counter position and command zero counter velocity. |
| Model-term or support failure | Use the shared counter hold. |
| Empty nominal velocity bounds | Use the shared counter hold with `counter_bounds_infeasible`. |
| Frozen-H2 nominal rejected | Use the shared counter hold with `solver_failure`. |
| Robust-H2 nominal rejected, SciPy succeeds | Continue through unchanged H2 and finalizer. |
| Primary nominal exception or SciPy fallback failure | Use the shared counter hold with `solver_failure`. |
| H2 context, N5, or confidence invalid | Use zero H2 residual and publish nominal. |
| H2 exception or rejected H2 output | Use zero H2 residual and publish nominal. |
| Combined request nonfinite or wrong shape | Use the shared counter hold with `nonfinite_solution`. |
| Collision at full request | Try inherited backtracking scales. |
| Every backtracked candidate invalid | Use the shared counter hold. |
| Gravity computation failure | Use the inherited no-gravity safe publication path. |

The finalizer validates the complete 27-joint candidate against effective joint
limits and collision checks. It preserves the moving-arm command, fixes the
counter wrist to its captured reference, computes gravity compensation, and
publishes one 14-joint position, velocity, and torque command.

## 11. Configuration Contract

The submodule class defaults are shadow-development values. The frozen benchmark
controller must use the active overlay below; selecting runtime variant
`counter_residual_h2` alone does not imply the frozen active configuration.

```yaml
iteration5_h2:
    shadow: false
    max_iterations: 1
    trust_velocity: [0.01, 0.01, 0.0, 0.01]
    context_limit: 3.0
    weights:
        action: 0.01
        change: 0.005
        tilt: 2.0
        rate: 1.0
        divergence: 2.0
        reserve: 0.1
```

Frozen H2 uses runtime variant `counter_residual_h2`. Robust H2 uses runtime
variant `counter_residual_h2_robust` with the identical `iteration5_h2` block.

Configuration validation requires:

- `shadow` to be boolean.
- `max_iterations` to be a nonnegative integer.
- `trust_velocity` to contain four finite nonnegative values with index `2`
  exactly zero.
- `context_limit` to be finite and positive.
- Every weight to be finite and nonnegative.

## 12. Diagnostics

The controller exposes shared nominal and finalization diagnostics plus the H2
fields below:

| Group | Fields |
| --- | --- |
| Solve identity | `h2_shadow`, `h2_sequence`, `h2_status`, `h2_accepted`, `h2_error` |
| Validity | `h2_confidence`, `h2_n5_valid`, `h2_context_valid`, `h2_model_valid` |
| Measured state | `h2_current_tilt`, `h2_current_rate` |
| Nominal prediction | `h2_nominal_tilt`, `h2_nominal_rate` |
| Residual prediction | `h2_incremental_tilt`, `h2_incremental_rate`, `h2_predicted_tilt`, `h2_predicted_rate` |
| Local models | `h2_u5_gain`, `h2_r5_gain` |
| Action | `h2_residual`, `h2_pending_residual`, `h2_decision` |
| Timing and optimizer | `h2_solve_time`, `h2_total_time`, `h2_iterations`, `h2_stopping_criterion`, `h2_seed_cost`, `h2_optimized_cost`, `h2_warm_started` |
| Historical robust nominal path | `nominal_fallback_used`, `nominal_primary_accepted` |

The nominal diagnostics separately expose Crocoddyl convergence, cost, solve
time, iterations, stopping criterion, KKT violation, regularization, BoxQP
polishing, and complete velocity-controller time. In the current SciPy-primary
runtime, `nominal_backend`, `nominal_retry_used`, and `nominal_status` identify
the new path. `nominal_fallback_used` means a TRF retry was attempted, and
`nominal_primary_accepted` refers to the first SciPy attempt, not Crocoddyl.

## 13. Historical Sweeps and Current Timing

The arm reachability and hard-target sweeps replay identical saved 14-joint
trajectories across controller variants. Controller overlays are merged into the
trial configuration without changing the saved target. Frozen H2 and robust H2
must therefore use the same target geometry, motion profile, classifier, lower
policy, safety settings, nominal gains, H2 trust, and H2 weights in a matched
comparison.

The sweep classifies physical outcomes in increasing severity as stable, drift,
stumble, and fall. Execution or controller incompleteness is separate from a
physical fall. An H2 improvement claim requires a completed matched pair and a
physical severity transition, followed by video and margin review. A rejected
nominal solve that leaves a physically stable trajectory is still an operational
controller failure and cannot be counted as a completed stable result.

The historical checkpoint sweep established the distinction between the variants:

| Cell | Frozen H2 | Robust H2 |
| --- | --- | --- |
| V2 right upward-overhang minus | Physically stable `3/3`, controller-incomplete `3/3`. | Stable and controller-complete `3/3`. |
| V3 right upward-overhang minus | Physically stable `3/3`, controller-incomplete `3/3`. | Stable and controller-complete `3/3`. |
| V3 left overhead | Stable with nominal failures in `2/3`. | Stable and controller-complete `3/3`. |

Across the nine robust checkpoint reliability runs, the complete-controller
timing was `2.73/6.27/7.87/19.05 ms` at p50/p95/p99/max. The fallback activated
on eight controller ticks, and every robust run completed operationally. This
met the reported historical timing thresholds; it does not certify the current
refactor. Current validation uses complete-controller p99 `< 15 ms` for each
run, with maxima and actual-published late samples retained. Main/health timing
used an inner kernel including validation/retries/finalization but excluding
outer resets. Main H2 maxima were `32.044 ms` (FAME) and `28.912 ms` (ALMI),
each including one actual-published `>20 ms` sample. The ten separate outermost
timing runs passed: new 3C worst per-run p99/max `5.546/7.098 ms`, new H2
`8.305/9.594 ms`, with no `>15`/`>20 ms` samples. These are not end-to-end DDS
latencies. A pooled p99 is not a per-run gate or a deadline guarantee.

Historical robust H2 was promoted only as a reliability improvement. It preserves
accepted frozen-H2 commands and the established FAME `09/11` rescue behavior. It does not
establish an additional rescue for FAME `06`, does not recover the left manual
falls, and does not justify increasing residual authority.

## 14. Historical Verification Requirements

The accepted-primary identity and fallback-only checks below describe archived
Croc-first robust H2. For the current SciPy-primary acceptance, fixed-coordinate,
shared-finalizer, and per-run timing gates, use the 6B preparation reference.
The listed test commands are retained as historical instructions, not evidence
that tests were executed for this documentation update. Final 6B
`provenance/validation_tests.json` records 217 root, 38 evidence, and 90 focused
controller passes, plus six evidence subtests and dependency warnings. Prior
176 targeted passes are separate earlier evidence; baseline lint/smoke limits
are not a universally passing submodule suite. See the 6B final report for scope.

The focused submodule tests are:

| Test file | Required behavior |
| --- | --- |
| `test/test_counter_residual_h2_ocp.py` | Running and terminal derivative checks, fixed weak joint, solver reuse, warm start, and exact zero bounds. |
| `test/test_counter_residual_h2_controller.py` | Shadow publication parity, confidence abstention, model-failure nominal fallback, active composition, applied pending state, and configuration validation. |
| `test/test_counter_residual_h2_robust_controller.py` | Accepted-path command identity and SciPy activation only after a rejected primary nominal result. |

Run the focused verification from the submodule root:

```bash
PYTHONPATH=. uv run pytest \
    test/test_counter_residual_h2_ocp.py \
    test/test_counter_residual_h2_controller.py \
    test/test_counter_residual_h2_robust_controller.py
```

Any implementation change must preserve the following invariants:

- Frozen H2 remains independently runnable.
- Robust H2 is command-identical whenever the primary nominal solve is accepted.
- Zero or rejected H2 residual preserves the frozen-3C nominal command.
- Residual joint `2` remains exactly masked.
- Applied residual state reflects collision backtracking, not the requested
  residual.
- H2 and nominal failures use the documented nominal or hold fallback without a
  second publication.
- Runtime inputs remain policy-blind, target-blind, and real-compatible.
- Complete-controller p99 remains below `15 ms`.

## 15. Related Evidence

Historical design rationale, model-identification gates, and execution evidence
remain in:

- `counter_balance_iteration_5.md`.
- `counter_balance_analysis_iteration_5.md`.
- `counter_balance_iteration_5e.md`.
- `counter_balance_analysis_iteration_5e.md`.
- `counter_balance_almi_checkpoint_sweep.md`.
- `counter_balance_analysis_almi_checkpoint_sweep.md`.
- `counter_balance_almi_speed_sweep.md`.

Those documents explain why the architecture and constants were selected. This
document retains the resulting frozen H2 implementation and the historical
robust-H2 difference. The linked 6B preparation document is the current
SciPy-primary nominal-refactor freeze reference. Its numerical/implementation
gates are complete; the ALMI hold-entry probability and broader physical/speed
readiness remain unresolved.
