# Counter-Balance Iteration 4C

## Decoupled Feedforward-Feedback Control

## Status and Evidence

Iteration 4C starts from frozen Iteration 3C
`counter_ddp_velocity_wide`, not from an Iteration 4B candidate. The frozen
controller is a one-step velocity-level Crocoddyl solve with early manipulation
feedforward, IMU gyro feedback, CoM/posture terms, wide bounds, collision
backtracking, and unchanged manipulation-arm publication.

The source evidence fixes these decisions:

- Iteration 3C retains two repeated FAME fall rescues, targets `09` and `11`.
- Iteration 4A stopped before active control because no common passive response
  model passed the roll-rate and lateral CoM-velocity gates.
- Iteration 4B preview-only scaling lost rescue `09`.
- Iteration 4B measured feedback was necessary to retain both rescues, but the
  combined scalar scheduler produced no repeated ordinary improvement.
- Iteration 4B H3-0 regressed ALMI behavior and missed the timing gate.

Therefore, Iteration 4C does not tune the 4B scheduler, revive H3-0, or revive
the 4A full response model.

## Infrastructure and Reproducibility Gate

The previous experiment session exhausted RAM and swap while multiple paired
Python simulation processes and editor processes were resident. System services
then missed watchdog deadlines. This is infrastructure evidence, not controller
evidence.

Development runs must therefore:

- Run serially with one trial process tree at a time.
- Remain headless and avoid development-time video rendering.
- Check for stale MuJoCo, ROS, policy, bridge, logging, and sweep processes
  before and after each compact batch.
- Check available memory, swap use, load, GPU use, and disk space before a batch.
- Stop rather than start another trial if an earlier process tree remains.
- Record complete controller timing from the first active experiment.

The clean pre-4C baseline reproduced FAME `09` and `11` as drift and ALMI `11`
as drift. FAME sentinel `06` unexpectedly survived as drift in two runs, although
both entered the fall-pending region late. It is not an additional rescue and
must use a contemporaneous 3C comparison before any later rescue claim.

Clean 3C timing was:

- Solver p99: `0.98-3.21 ms` across the compact trials.
- Total-controller p99: `2.61-4.29 ms`.
- Observed total maximum: `5.21 ms`.
- Solver failures: zero.

## Controller Contract

Iteration 4C remains:

- One-step and velocity-level.
- Policy-blind and target-blind.
- Compatible with IMU, measured joint state/torque, and Pinocchio-derived
  quantities.
- Limited to the four proximal counter-arm joints.
- Identical to 3C for manipulation-arm position, velocity, torque, lifecycle,
  bounds, collision handling, safety, and publication.

No policy identity, target identity, historical label, simulator contact state,
external wrench, or exact simulator base state may enter control.

## Decoupled Target

Let `b(t)` be the frozen 3C lifecycle scale. Frozen 3C uses planar targets:

\[
r_{c,3C}
=
b(-J_{m,c}\dot q_m-k_c e_c),
\]

\[
r_{H,3C}
=
b(-A_{m,H}\dot q_m+k_\omega\omega).
\]

Iteration 4C preserves those terms and adds an independent measured-response
correction only to the momentum target:

\[
r_{H,4C}
=
b\left(
-A_{m,H}\dot q_m
+k_\omega\omega
+K_\theta\operatorname{clip}(e_\theta,-e_{max},e_{max})
\right).
\]

The first candidate uses diagonal `K_theta = k_theta I`. Positive measured tilt
requests counter-arm momentum with the sign that produces a restoring base
reaction. This can change reactive command magnitude and direction without
attenuating preventive feedforward.

The initial deterministic scale is:

\[
k_\theta
=
k_\omega\frac{s_\omega}{s_\theta}
=
0.2\frac{0.25}{0.10}
=
0.5.
\]

Use `e_max = 0.10 rad`, the frozen drift scale. The correction is naturally
near zero before physical response develops. It uses no preview phase or outcome
gate. Invalid tilt data selects zero correction and recovers 3C.

The existing CoM target is unchanged. Support-relative CoM velocity from 4A is
not used because its lateral representation failed validation. Pinocchio-derived
CoM or ZMP may be investigated only after the first tilt experiment identifies
a repeatable limitation, and only as one separate mechanism.

## Implementation

Add one controller:

```text
CounterDecoupledVelocityController
counter_decoupled_velocity
```

It inherits `CounterDDPVelocityController` and overrides only reaction-target
construction and diagnostics. It does not depend on the 4B preview scheduler or
the 4A observer pipeline.

Required tests:

- Zero correction reproduces 3C targets and commands.
- Tilt correction has the restorative componentwise sign.
- Correction clips at the configured physical limit.
- Invalid or unavailable tilt produces zero correction.
- Left/right ownership uses the same law and parameters.
- Manipulation-arm publication remains unchanged.
- Configuration validation rejects nonfinite or negative limits/gains.

## Experimental Sequence

### C0: Parity and Timing

Run command-level tests and a compact physical 3C/4C comparison with
`k_theta = 0`. Require identical outcome, command sign, active bounds,
backtracking, and manipulation pass-through. Require practical command parity at
the frozen 3C tolerance and total-controller p99 below `15 ms`.

### C1: Tilt-Restoring Feedback

Enable only `k_theta = 0.5`. Compare against contemporaneous 3C on:

- FAME rescues `09` and `11`.
- FAME sentinel `06`.
- Ordinary FAME cases where 3C differs from B0.
- ALMI boundary/ordinary guards.
- At least one repeated ALMI stumble case.

Inspect outcome severity, tilt/rate traces, counter velocity/displacement,
clipping, collision backtracking, foot motion, manipulation tracking, and timing.
Use at least three complete repetitions for retained rescue, severe, stochastic,
or changed cells before a freeze decision.

Retain C1 only if both 3C rescues remain, no severe regression appears, and at
least one ordinary majority outcome improves or a preregistered paired continuous
metric improves materially with unchanged severity.

### C2: One Evidence-Selected Follow-Up

Only one follow-up mechanism may be tested:

- Change feedback magnitude if the restorative sign is correct but consistently
  too weak or too strong.
- Use one diagonal roll/pitch weighting if the axes differ consistently.
- Add bounded angular-rate divergence feedback if correction timing is late.
- Add one justified support-relative CoM or Pinocchio-derived ZMP contribution
  if angular feedback is directionally insufficient and the quantity passes a
  real-compatible consistency check.

Do not combine these mechanisms in one ablation. Stop if no repeatable physical
limitation selects one.

## Metrics and Freeze Gate

For every comparison report:

- Outcome severity and survival.
- Peak/RMS tilt and angular velocity.
- Positive divergence integral
  `integral(max(e_theta * omega, 0))`.
- Frozen feedforward, gyro feedback, and new tilt-feedback target vectors.
- Requested/applied counter velocity, displacement, clipping, and backtracking.
- Foot displacement/lift and support-validity timing as diagnostics only.
- Manipulation position/velocity command error.
- Solver and total-controller p50, p95, p99, and maximum timing.
- Infrastructure, runtime, solver, collision, and estop failures separately.

Freeze one 4C controller only if it:

- Retains at least the two repeated 3C FAME rescues.
- Improves ordinary FAME/ALMI behavior beyond 3C, preferably to B0 or better.
- Adds no new majority fall, stumble, or other severe regression.
- Preserves manipulation-arm behavior.
- Has total-controller p99 below `15 ms` and no accepted late command.

An additional FAME rescue or ALMI stumble improvement is actively tested but is
not obtained by weakening these gates. If no candidate passes, retain frozen 3C
and B0 rather than promoting 4C.

## Execution Record

### Retained Candidate

The executed candidate is:

```yaml
decoupled_feedback:
  tilt_gain: 0.5
  max_tilt_error: 0.1
```

Only this mechanism remains in source. The runtime name is
`counter_decoupled_velocity`; it is available only through the dedicated serial
Iteration 4C compact configurations. It was not added to the ordinary hard/full
sweep matrices. The combined SHA-256 of the retained controller, runtime/harness
integration, and two compact configurations is
`6c4a968cfbc3c38d1d89e54466417f760cac62f5ef44aa51b8b769f905e0a239`.

Across 17 complete retained-candidate trials, controller timing was:

- Solver p50/p95/p99/max: `0.223/0.458/2.071/6.083 ms`.
- Total p50/p95/p99/max: `1.215/2.302/3.287/7.615 ms`.
- Solver failures: zero.
- Maximum tilt-feedback target norm: `0.032172`.

All retained-candidate trials had precise manipulation tracking. Host memory
remained at approximately `27 GiB` available with zero swap use, the GPU was
idle, and no trial processes remained after the final batch.

### Outcome Evidence

| Policy | Target | 3C evidence | 4C result | Decision |
| --- | --- | --- | --- | --- |
| FAME | `09` | Frozen repeated drift rescue | Drift, 3/3 | Rescue retained |
| FAME | `11` | Frozen repeated drift rescue | Drift, 3/3 | Rescue retained |
| FAME | `06` | Frozen repeated fall; current 3C drift, 2/2 | Drift, 1/1 | No new rescue claim |
| FAME | `04` | Frozen drift, 3/3; current 3C stable/drift/stable | Stable, 3/3 | Small, confounded shift |
| ALMI | Right `11` | Frozen and current 3C drift | Drift, 3/3 | No ordinary improvement |
| ALMI | Manual grasp minus | Frozen and current 3C stumble | Stumble, 3/3 | No stumble improvement |

For FAME `04`, the current median peak drift changed from `0.09937` under 3C
to `0.09815` under 4C, about `1.2%`; median RMS drift changed by less than `1%`.
This near-threshold effect is not material enough to override the contemporaneous
baseline shift. FAME `11` showed a directionally favorable but small peak/RMS
change in the first screen. ALMI `11` was slightly worse, and ALMI manual-grasp
foot displacement did not improve.

### Rejected Ablations

The following were tested one at a time and removed from source/configuration:

- Zero-gain C0: command and physical parity with 3C; used only as an integration
  check.
- Tilt gain `1.0`: no larger ordinary benefit, slightly worse ALMI `11`, and one
  KKT-invalid ALMI stumble trial.
- Tilt gain `2.0`: no larger ordinary benefit, worse ALMI `11`, and one
  KKT-invalid ALMI stumble trial.
- Divergence-gated tilt gain `2.0`: no better FAME result and worse than gain
  `0.5` on the ALMI ordinary guard.
- Divergence-rate gain `0.4`: improved FAME `11` continuously but regressed FAME
  `04` to drift and increased ALMI stumble foot displacement.

No new CoM or ZMP feedback was activated. The controller already retains 3C's
Pinocchio CoM term; 4A did not validate lateral support-relative CoM velocity,
and no ablation produced evidence strong enough to justify adding another
derived signal or duplicate kinematics computation.

### Final Decision

Iteration 4C is **frozen as an unpromoted diagnostic controller**. It retains the
two required FAME rescues, preserves manipulation behavior, introduces no
retained-candidate severe regression, and meets the real-time gate. It does not
clearly improve ordinary FAME/ALMI majority behavior, produce an additional
defensible FAME rescue, or improve the repeated ALMI stumble. Therefore the
promotion gate is not met; frozen 3C and B0 remain the operational baselines.

### Software Validation

- Benchmark-owned suite: `137 passed`.
- Focused controller/runtime integration suite: `118 passed`.
- Controller-owned functional suite: `172 passed`, with one unrelated existing
  planner smoke-test failure caused by missing named configuration
  `t_pose_elbow`.
- Python compilation and `git diff --check`: passed.
- An unscoped repository-root `pytest` command is not valid for this meta-repo;
  it recursively collects duplicate nested SDK tests, unavailable ROS
  `ament_*` lint dependencies, and a hardware joystick script.
