# ZMP Balance Update 2 Plan

## Goal

Develop the upper-body balance controller into a causal reflex:

1. Detect a new external disturbance without reacting to quiet standing or its
   own arm motion.
2. Estimate the direction and magnitude of useful counter-momentum in a common
   support frame.
3. Compute a command that both arms can actually realize under joint,
   end-effector, and posture constraints.
4. Track the delivered arm momentum and stop or recover without applying a
   harmful opposite impulse.

This update is a plan only. It does not prescribe gain changes before the
observer, feasibility, and diagnostic problems are resolved.

## Current Baseline

The matched sweep is stored in
`runs/20260713_both_fine_sweep/` in the benchmark meta-repository. It uses an
eight-direction, five-iteration search over `0` to `160 N`, a `5 s` force
start, a `1 s` force duration, and `5 N` force brackets.

| Direction | Lower-only | ZMP-enabled | Result |
| --- | --- | --- | --- |
| `+X` | `[25, 30) N` | `[30, 35) N` | ZMP improves by `5 N`. |
| `+X +Y` | `[35, 40) N` | `[35, 40) N` | Equal. |
| `+Y` | `[100, 105) N` | `[95, 100) N` | ZMP regresses by `5 N`. |
| `-X +Y` | `[30, 35) N` | `[30, 35) N` | Equal. |
| `-X` | `[25, 30) N` | `[25, 30) N` | Equal. |
| `-X -Y` | `[40, 45) N` | `[40, 45) N` | Equal. |
| `-Y` | `[95, 100) N` | `[95, 100) N` | Equal. |
| `+X -Y` | `[35, 40) N` | `[35, 40) N` | Equal. |

Both variants have a mean best passing force of `48.125 N`. The controller
therefore redistributes disturbance resistance rather than improving the
aggregate envelope.

The categorical changes are:

- At `+X, 30 N`, lower-only falls at approximately `7.535 s`; ZMP survives
  with `0.218 rad` peak tilt.
- At `+Y, 100 N`, lower-only survives; ZMP falls at approximately `9.615 s`,
  well after force removal. This points to a recovery failure rather than only
  an initial rejection failure.
- At `-Y, 95 N`, ZMP survives but peak tilt increases from `0.217 rad` to
  `0.281 rad`, reducing the remaining stability margin.

## Implemented Increment 1

The following low-risk reflex changes were implemented before introducing a
new allocator or dynamics architecture:

- Quiet calibration is reset when the benchmark start flag is observed, rather
  than relying on the frozen pre-start simulator state.
- The observer learns a `60`-sample standing ZMP reference, then waits a
  `90`-sample quiet arming dwell before detector entry is enabled.
- Detector entry uses a calibrated `0.003 m` ZMP residual with two-tick
  hysteresis. The raw absolute ZMP level is retained only for legacy rollback.
- The target no longer applies the `0.3 Nms` minimum-momentum jump. Center and
  COM-velocity gains are disabled until base-state estimation is validated.
- Target bounds support explicit lower and upper component limits. The active
  profile limits `Lx` to `[0.0, 0.2] Nms` and `Ly` to `[-1.5, 1.5] Nms`.
  This is a provisional sign guard, not a replacement for support-frame and
  reaction-sign identification.
- The actuator can hold after a direct burst, but the active profile retains
  bounded burst and return behavior because hold-only failed the `+X` case.
- The controller resets active response state on experiment start.
- ZMP logs now include calibration and arming state, detector reasons and
  episode IDs, residuals, raw and bounded targets, actuator state, and
  predicted post-limit momentum. Analysis counts episode IDs rather than stale
  response-status rows.

Focused unit coverage verifies quiet calibration, residual entry, direct hold
state behavior, direction-preserving target projection, startup reset, and the
new target-bound compatibility. The focused controller suite has `16` passing
tests.

## Increment 1 Evaluation

The matched final sweep is stored in
`runs/20260713_update2_signguard_sweep/` in the benchmark meta-repository. It
uses the same eight directions, `5 s` force start, `1 s` duration, and `5 N`
resolution as the current baseline.

| Direction | Lower-only | ZMP-enabled | Delta |
| --- | --- | --- | --- |
| `+X` | `25 N` | `30 N` | `+5 N`. |
| `+X +Y` | `35 N` | `35 N` | `0 N`. |
| `+Y` | `100 N` | `100 N` | `0 N`. |
| `-X +Y` | `30 N` | `30 N` | `0 N`. |
| `-X` | `25 N` | `25 N` | `0 N`. |
| `-X -Y` | `40 N` | `40 N` | `0 N`. |
| `-Y` | `95 N` | `95 N` | `0 N`. |
| `+X -Y` | `35 N` | `35 N` | `0 N`. |

The lower-only mean is `48.125 N`; ZMP-enabled mean is `48.750 N`. The
increment restores the `+X` gain and removes the prior `+Y` regression, but
does not yet meet the planned `5 N` aggregate mean improvement gate.

Detector quality improved materially:

- All `40` ZMP trials had zero pre-force episodes and zero pre-force arm
  commands.
- Force-window entry occurred in `38` of `40` trials.
- Entry latency ranged from `66 ms` to `901 ms`, with a `234 ms` median.

The zero false-trigger result is accepted. The latency range and asymmetrical
`Lx` sign guard are not accepted as final architecture; they motivate the
deferred work below.

## Closure Corrections And Validation

The closure increment corrected the benchmark methodology without introducing
a new allocator or reflex state machine:

- The target now uses the same calibrated ZMP residual used by detector entry.
- ZMP runtime resets live calibration when simulation starts, suppresses reflex
  output during calibration and arming, and only enables output after live
  arming completes.
- The benchmark force starts at `8 s`, leaving a quiet armed interval after the
  `60`-sample calibration and `120`-sample arming dwell.
- `upper_fixed` is the explicit fixed-upper-body baseline: it uses the
  benchmark-owned `upper_hold_runtime` and does not start a ZMP controller.
  Historical `zmp_passive` labels in older artifacts referred to a suppressed
  ZMP process without an active upper hold and should not be used as a baseline.
- Each run snapshots benchmark, safety, lower-body, and ZMP YAML files, hashes
  them, and records benchmark/controller revision and dirty-state provenance.
- The sweep now records a seeded interleaving plan and runs the passive and
  enabled variants in shuffled order for every direction/iteration pair.
- Telemetry labels full-body command momentum as predicted pre-limit and
  predicted post-limit momentum, separately from momentum estimated from the
  measured joint velocity. Burst IDs and burst-entry flags are logged.

The zero-force scenario `config/zmp_no_force.yaml` was validated with both
controls-equivalent variants: no detector episodes, no upper reflex command,
and no fall.

The closure sweep is stored in
`runs/20260714_update2_closure_sweep/`. It used ROS domain `2`, delayed
`8 s`/`1 s` forces, 5 N force brackets, frozen per-run configuration snapshots,
and seed `20260714`.

| Direction | Upper fixed | ZMP enabled | Delta |
| --- | --- | --- | --- |
| `+X` | `25 N` | `25 N` | `0 N`. |
| `+X +Y` | `35 N` | `35 N` | `0 N`. |
| `+Y` | `115 N` | `115 N` | `0 N`. |
| `-X +Y` | `30 N` | `30 N` | `0 N`. |
| `-X` | `25 N` | `25 N` | `0 N`. |
| `-X -Y` | `40 N` | `40 N` | `0 N`. |
| `-Y` | `105 N` | `105 N` | `0 N`. |
| `+X -Y` | `35 N` | `30 N` | `-5 N`. |

The passive mean is `51.250 N`; the enabled mean is `50.625 N`. All 40 enabled
trials had zero pre-force episodes and zero pre-force reflex command, but only
34 entered during the force window. Entry latency ranged from `345 ms` to
`919 ms`, with a `544 ms` median. The closure result does not satisfy the
performance or latency acceptance gates and does not justify further parameter
tuning within Update 2.

## Observed Failure Mechanisms

### Non-Causal Pre-Disturbance Response

Across all 40 ZMP sweep trials:

- The first detector activation occurs `0.085` to `0.125 s` after benchmark
  start, almost five seconds before the force.
- The detector is active during approximately `50%` to `59%` of the quiet
  interval.
- Maximum quiet ZMP error is approximately `0.045` to `0.047 m`, while the
  configured threshold is `0.012 m`.
- Median arm displacement at force onset is approximately `0.16 rad` in ZMP
  runs versus `0.03 rad` in lower-only runs.

The controller is therefore changing the initial arm posture and reflex phase
before the disturbance. The `+X` gain cannot yet be attributed entirely to a
reaction triggered by the applied force.

### Delayed True-Disturbance Detection

Post-force detector latency ranges from approximately `0.15` to `0.62 s`.
Moderate `+Y` and `-Y` disturbances are the slowest, reaching approximately
`0.55` to `0.62 s`. This aligns with the weakest directional results.

The observer derives ZMP and COM velocity from a free-flyer model whose base
linear and angular velocity entries are zero. Translational torso pushes are
therefore observed indirectly through joint motion and tilt. Derivatives also
use a fixed control period instead of the state timestamp.

### Target And Command Saturation

- The first post-force target has its `Y` momentum component at `+1.5` in all
  40 ZMP trials, regardless of disturbance direction.
- At least one target component is saturated during approximately `87%` of
  active force-window samples.
- Strong raw command norms reach `8` to `13`, while applied norms remain near
  `2.6` to `2.7`.
- The median applied-to-raw ratio during strong commands is approximately
  `0.31`.

The direct solve uses a `4 rad/s` per-joint limit, but the downstream controller
uses `1.25 rad/s` plus end-effector limits. The actuator's achieved-momentum
diagnostic is computed before this downstream scaling, so it overstates the
momentum delivered to the robot.

### Direction-Blind Allocation

The whole-body target is split equally between the arms. Allocation does not
consider:

- Directional momentum authority of each arm.
- Current full-body posture.
- Joint-position and braking margin.
- End-effector speed constraints.
- Residual momentum after one arm saturates.

The reduced arm models also freeze non-arm joints at model creation, so their
centroidal maps do not remain consistent with the current whole-body posture.

### Harmful Burst And Recovery Cycle

The current direct response alternates a `0.20 s` momentum burst with an
immediate posture return and `0.20 s` cooldown. Return begins without checking
whether the disturbance remains active, whether base angular velocity is
arrested, or whether the return momentum worsens the support error.

This can cancel useful counter-momentum during the one-second force and can
explain delayed failures such as `+Y, 100 N`. The controller also continues
issuing bursts after the benchmark fall threshold is crossed.

### Insufficient Diagnostics And Baseline Control

`response_status` remains `direct` after a burst and the analyzer counts
non-idle rows, not distinct response episodes. Current logs do not include
detector reasons, observer freshness, actuator phase, limit causes, feasible
momentum, post-limit momentum, measured momentum, or joint margins.

The lower-only upper hold and ZMP controller also use different upper-body
servo paths, gains, initialization behavior, and gravity compensation. A
controls-equivalent passive mode is needed to isolate reflex performance.

## Copied Update 1 TODOs

- [ ] Replace the standing-bias-sensitive ZMP trigger with a fast residual or
      derivative-based disturbance signal: preserve the early response needed
      for the 30 N, 1 s push without activating before the scripted impulse.
- [ ] Solve the direct momentum allocation with the arm and end-effector
      velocity limits inside the optimization: the present post-solve limiter
      makes the reported momentum target unreachable for most failing trials.
- [ ] Feed the post-limit achievable momentum back into target generation and
      allocate residual momentum across arms according to their directional
      capability instead of using fixed equal weights.
- [ ] Add burst, cooldown, target-before/after-saturation, and
      post-limit-achieved-momentum diagnostics to distinguish detector bias,
      target saturation, and actuator-limit failures.
- [ ] Repeat the `5 N` boundary trials for both variants and quantify variance
      before accepting a force-limit improvement.

## Design Principles

- Preserve the current `+X, 30 N` survival result at every stage.
- Change one control layer at a time and retain a runtime switch for rollback.
- Use measured or post-limit quantities for feedback and evaluation.
- Keep all vectors in an explicitly named support frame.
- Treat desired angular-momentum rate, arm angular momentum, and joint velocity
  as separate quantities.
- Do not increase safety velocity limits to compensate for an infeasible solve.
- Keep the upper-body position-control interface for this update; torque control
  is outside scope.

## Proposed Reflex Pipeline

```text
state freshness and quiet calibration
    -> external-disturbance residual and episode detector
    -> support-frame desired momentum-rate trajectory
    -> feasible two-arm momentum and velocity optimization
    -> acceleration-shaped position command
    -> measured momentum feedback
    -> arrest, recovery wait, and bounded posture return
```

The desired support correction should first produce a desired momentum rate:

```text
dot_L_des = mg * cross_map * support_error_feedback
```

A bounded momentum trajectory should then be generated from `dot_L_des`. The
allocator should solve both arms jointly:

```text
minimize
    ||A_left dq_left + A_right dq_right - L_feasible||_W^2
    + rho ||dq||^2
    + sigma ||dq - dq_previous||^2
```

The optimization must include the limits that are currently applied after the
solve.

## Stage 0: Establish Truthful Measurement

### Hypothesis

The existing logs hide false activation, stale diagnostics, and lost momentum.
Correct instrumentation will identify whether each failed trial is detector,
target, feasibility, tracking, or recovery limited.

### Work

- [ ] Add state sequence, source timestamp, sample age, measured observer `dt`,
      control-loop duration, and deadline-miss fields.
- [ ] Log all observer signals, validity flags, detector predicates, reasons,
      severity, and enter/exit counters.
- [ ] Add response episode ID, burst ID, exact actuator state, phase index,
      cooldown remaining, and return index.
- [ ] Log raw target, direction-preserving bounded target, feasible target,
      per-arm target, and residual momentum.
- [ ] Log unconstrained velocity, optimized velocity, final applied velocity,
      measured velocity, and the active constraint for each arm.
- [ ] Compute predicted pre-limit momentum, predicted post-limit momentum, and
      measured momentum from the next state sample.
- [ ] Clear response status and target fields on state exit.
- [ ] Count detector episodes and burst-entry transitions rather than non-idle
      log rows.
- [ ] Add pre-force, force-window, arrest-window, and recovery-window metrics to
      benchmark analysis.
- [ ] Add a reflex-disabled mode that uses the same `ZmpController`, upper-body
      gains, initialization, gravity compensation, and DDS path as reflex mode.

### Acceptance Gate

- [ ] Logged post-limit momentum equals `A(q) dq_applied` within numerical
      tolerance.
- [ ] Each physical burst produces exactly one burst-entry event.
- [ ] Idle and recovery rows never retain stale direct-response status.
- [ ] Reflex-disabled and passive upper-body trajectories agree within
      `0.02 rad` per joint during no-force standing.
- [ ] Existing `+X, 30 N` behavior is reproducible before control changes.

## Stage 1: Correct Observation And Detection

### Hypothesis

A calibrated transient residual can remove pre-force activation while reducing
true-disturbance latency below three control ticks.

### Work

- [ ] Use source timestamps and measured `dt` for derivatives.
- [ ] Reject stale or repeated state samples and expose observer validity.
- [ ] Add an explicit quiet calibration/arming state that learns standing ZMP,
      gyro, COM-motion, and force-proxy bias and variance.
- [ ] Keep the settled reference frozen for the complete disturbance episode.
- [ ] Add base angular velocity and a validated base linear velocity estimate
      to the free-flyer state used by dynamics.
- [ ] Validate joint acceleration or remove acceleration-dependent ZMP terms
      until acceleration is observable.
- [ ] Add contact or vertical-force validity to ZMP use.
- [ ] Use a fast entry residual based on high-pass ZMP, angular acceleration,
      and validated base/COM acceleration.
- [ ] Use absolute support margin and slower level signals for severity, hold,
      and exit decisions rather than initial entry.
- [ ] Prevent commanded arm motion or posture return from creating a new
      external-disturbance episode without a new residual transient.
- [ ] Evaluate the new detector in shadow mode before connecting it to arms.

### Unit Tests

- [ ] A constant `30 mm` ZMP bias for 30 seconds creates no episode.
- [ ] Slow reference drift creates no episode.
- [ ] Repeated or stale samples do not create derivative spikes.
- [ ] A synthetic disturbance step activates within two control ticks.
- [ ] Arm-only commanded movement does not create a new disturbance episode.
- [ ] Detector entry and exit hysteresis are deterministic at threshold edges.

### Simulation Gate

- [ ] Zero pre-force bursts in at least 20 no-force or delayed-force trials.
- [ ] Median force-onset-to-command latency is no more than `67 ms`.
- [ ] Maximum latency is no more than `100 ms` in all eight directions.
- [ ] Preserve at least 9/10 survival at `+X, 30 N`.

## Stage 2: Verify Frames, Signs, And Target Generation

### Hypothesis

The universal first `+Y` target saturation and lateral-axis delay indicate
frame, bias, or target-shaping errors. Explicit frame transforms and a smooth
momentum-rate trajectory will produce direction-specific counter-actions.

### Work

- [ ] Define one support-frame convention for ZMP, COM motion, IMU signals,
      target momentum, arm momentum maps, and diagnostics.
- [ ] Transform IMU body-frame signals and arm momentum maps into that frame.
- [ ] Express the ZMP-to-angular-momentum-rate cross mapping as an explicit
      matrix with sign tests.
- [ ] Validate reaction-mass sign experimentally: measured base response should
      oppose the disturbance after each small arm momentum pulse.
- [ ] Separate desired `dot_L` from the arm `L` trajectory instead of multiplying
      once by `response_time` and holding that momentum.
- [ ] Replace the hard `0.3` minimum momentum with a continuous dead zone and
      severity-dependent ramp.
- [ ] Replace independent component clipping with a direction-preserving norm
      or feasible ellipsoid projection.
- [ ] Add target slew, momentum-rate, acceleration, and jerk limits.

### Acceptance Gate

- [ ] Basis tests for `+X`, `-X`, `+Y`, and `-Y` produce a predicted support
      correction opposite the original error.
- [ ] Basis tests remain correct with nonzero base roll, pitch, and yaw.
- [ ] Initial target direction differs appropriately across all eight applied
      force directions.
- [ ] Median target-direction error is below `30 degrees` in simulation.
- [ ] Target saturation occupies less than `25%` of the force window without
      losing `+X, 30 N` survival.

## Stage 3: Compute Feasible Two-Arm Counter-Momentum

### Hypothesis

A joint constrained allocator will produce more useful momentum than equal
splitting followed by multiple clipping stages, without raising safety limits.

### Work

- [ ] Use current full-body centroidal-map arm columns rather than cached
      reduced-arm maps with frozen non-arm joints.
- [ ] Solve both arm velocities jointly each tick.
- [ ] Include the actual `1.25 rad/s` joint-velocity limits in the solve.
- [ ] Include end-effector linear and angular speed limits in the solve.
- [ ] Include predicted joint-position margin and braking distance.
- [ ] Include joint acceleration and jerk bounds relative to the prior command.
- [ ] Correct controller joint limits to match the authoritative URDF.
- [ ] Penalize orthogonal momentum, poor conditioning, wrist-heavy motion, and
      loss of joint margin.
- [ ] Return feasible momentum, residual momentum, per-arm contribution,
      directional capability, and active constraints.
- [ ] Reallocate residual demand to the arm with remaining directional capacity.
- [ ] Run the constrained solver in shadow mode against current DLS output.

### Acceptance Gate

- [ ] Solver time is below `5 ms` at the 99th percentile.
- [ ] No optimized command violates joint, end-effector, position, acceleration,
      or jerk constraints.
- [ ] No post-solve limiter changes optimized velocity by more than `1%`.
- [ ] Combined post-limit momentum tracks feasible target within `5%`.
- [ ] Median useful momentum residual is at least `25%` lower than current DLS.
- [ ] Momentum alignment with the requested direction exceeds `0.9`.
- [ ] If one arm saturates, useful residual is reassigned when feasible.

## Stage 4: Close The Loop On Delivered Momentum

### Hypothesis

Tracking measured arm momentum and shaping onset and braking will make the
counter-action repeatable despite position-servo lag.

### Work

- [ ] Estimate actual arm momentum from measured joint position and velocity.
- [ ] Close the reflex loop on feasible-target minus measured-momentum error.
- [ ] Feed delivered and residual momentum back into target generation.
- [ ] Identify command-to-measured-momentum delay and directional cross-coupling
      using small safe pulses.
- [ ] Keep the current position-command interface and existing safety limits.
- [ ] Use acceleration-shaped ramp-up and deliberate momentum braking.
- [ ] Add explicit anti-windup for target and command saturation.

### Acceptance Gate

- [ ] Measured useful momentum reaches at least `80%` of feasible target during
      the rejection phase.
- [ ] Predicted and measured momentum differ by less than `20%` after identified
      servo delay.
- [ ] No adjacent command violates configured acceleration or jerk bounds.
- [ ] Braking produces less than `10%` of the useful impulse in the harmful
      opposite direction.

## Stage 5: Replace Burst/Cooldown With A Reflex State Machine

### Hypothesis

Disturbance-aware arrest and delayed recovery will remove the present `+Y` and
`-Y` regressions caused by return motion.

### Proposed States

```text
CALIBRATING -> ARMED -> REJECT_RAMP -> REJECT
            -> ARREST -> RECOVERY_WAIT -> RETURN -> ARMED
```

### Work

- [ ] Enter `REJECT_RAMP` only on a new external-disturbance episode.
- [ ] Stay in rejection while residual direction and support risk indicate that
      counter-momentum remains useful.
- [ ] Transition to `ARREST` using measured base and arm momentum, not a fixed
      six-tick timeout.
- [ ] Wait for residual, base angular velocity, and support risk to remain below
      exit thresholds before returning posture.
- [ ] Return with a bounded time-scaled trajectory and pause immediately on a
      new disturbance.
- [ ] Retrigger from current measured arm velocity with continuous commands.
- [ ] Stop repeated responses when directional capability or posture margin is
      exhausted.
- [ ] Add a fall or invalid-state interlock that suppresses further bursts.

### Acceptance Gate

- [ ] Every state transition and persistent-disturbance path has a unit test.
- [ ] No return begins during the active one-second force unless rejection has
      already arrested the base and residual is clear.
- [ ] Return-induced peak base angular velocity is below `10%` of the original
      disturbance peak.
- [ ] Return motion creates no false detector episode.
- [ ] Arms return within `0.03 rad` per joint and `2 s` when balance permits.
- [ ] `+Y, 100 N` and `-Y, 95 N` no longer regress relative to passive control.

## Stage 6: Statistical Validation

### Boundary Matrix

Run at least ten interleaved repetitions per variant at:

| Direction | Forces |
| --- | --- |
| `+X` | `25`, `30`, and `35 N`. |
| `+Y` | `95`, `100`, and `105 N`. |
| `-Y` | `90`, `95`, and `100 N`. |
| Diagonals | The passing boundary and next `5 N` failure for each direction. |

Also include no-force trials, short pulses, the current one-second force, and
delayed forces. Randomize or interleave variant order.

### Reported Metrics

- [ ] Survival probability and confidence interval.
- [ ] Incremental peak tilt relative to the pre-force baseline.
- [ ] Peak roll and pitch rate.
- [ ] Detection latency, arrest time, and recovery time.
- [ ] Minimum support margin and maximum planar displacement.
- [ ] Delivered useful arm angular impulse and opposite braking impulse.
- [ ] Target saturation and active-constraint fractions.
- [ ] False-trigger rate and pre-force arm displacement.
- [ ] Final posture error and return-induced base motion.

### Final Acceptance Gate

- [ ] `+X, 30 N` survives in at least 9/10 trials.
- [ ] No direction loses a previously passing `5 N` boundary with statistically
      meaningful frequency.
- [ ] Mean best passing force improves by at least `5 N` over the
      controls-equivalent passive mode.
- [ ] At least one weak-axis bracket improves by `5 N`, or impulse-aligned peak
      angular velocity improves by at least `10%` without a survival regression.
- [ ] Quiet standing has zero reflex episodes in 20/20 trials.
- [ ] All solver timing, safety-limit, recovery, and validity gates pass.

## Deferred Architecture TODOs

The following changes are required to move beyond the current modest, tuned
improvement. They were not implemented in this increment.

- [ ] Reconstruct validated free-base linear and angular velocity, acceleration,
      contact confidence, and source sample timing for the observer.
- [ ] Define and test one support-frame transform for ZMP, IMU, COM motion,
      target momentum, and arm centroidal maps.
- [ ] Identify measured reaction-mass sign and cross-axis coupling with safe
      single-axis arm pulses, then replace the provisional asymmetric `Lx` guard.
- [ ] Replace fixed equal allocation and post-solve clipping with a current
      full-body, constrained two-arm optimization.
- [ ] Include joint velocity, position/braking margin, acceleration, jerk,
      end-effector speed, and directional capability constraints in that solve.
- [ ] Close feedback on measured post-limit arm momentum rather than predicted
      reduced-model momentum.
- [ ] Replace fixed burst/cooldown behavior with reject, arrest,
      recovery-wait, and bounded-return phases.
- [ ] Make the passive baseline controls-equivalent by using the same upper-body
      servo path with reflex output disabled.
- [ ] Run interleaved repeated boundary trials before accepting an aggregate
      performance claim.

## Planned Code Touchpoints

Expected controller files include:

- `core/controller/zmp/balance_observer.py`: timestamps, calibration, residuals,
  validity, and base-motion observation.
- `core/controller/zmp/perturbation_detector.py`: episode-based transient entry,
  level-based hold and exit, and arm-motion rejection.
- `core/controller/zmp/momentum_target_estimator.py`: support-frame momentum-rate
  trajectory and smooth direction-preserving bounds.
- `core/controller/zmp/momentum_allocator.py`: constrained two-arm feasibility
  and capability allocation.
- `core/controller/zmp/balance_actuator.py`: measured feedback, shaped actuation,
  state machine, arrest, and recovery.
- `core/controller/zmp_controller.py`: pipeline integration and truthful
  post-limit diagnostics.
- `core/robot_model.py` and dynamics helpers: validated base state and current
  full-body centroidal-map access.
- `config/balance_safety_split.yaml`: staged feature flags and limits, not blind
  gain increases.
- `test/`: observer, detector, sign, allocation, constraint, momentum tracking,
  and state-transition coverage.

Benchmark-side work should be limited to diagnostics, controls-equivalent
variant selection, randomized repeated trials, and analysis. The controller
implementation remains in `h12_ros2_controller`.

## Recommended Implementation Order

1. Stage 0: instrumentation and controls-equivalent passive baseline.
2. Stage 1: calibrated residual detector in shadow mode.
3. Stage 2: frame, sign, and target-trajectory validation.
4. Stage 3: constrained two-arm allocator in shadow mode.
5. Stage 4: measured momentum feedback and shaped commands.
6. Stage 5: arrest and recovery state machine.
7. Stage 6: repeated boundary trials followed by a full `5 N` sweep.

Gain tuning before Stages 0 through 3 is explicitly deferred. The current
controller is dominated by observer bias and hidden command infeasibility, so
gain-only tuning would optimize around incorrect measurements and saturation.
