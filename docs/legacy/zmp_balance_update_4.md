# ZMP Balance Update 4

## Goal

Build an effective upper-body flywheel controller that changes the robot's
support response through controlled arm angular-momentum **rate** and delays the
opposite braking impulse until the lower body can absorb it safely.

This replaces the failed assumption that tracking an arm momentum value is
itself a sustained balance action.

## Lessons Applied

The design follows `zmp_balance_update_analysis.md` and the failed experiment
record in `zmp_balance_update_4_failure.md`.

### Control The Rate, Not Only The Momentum

The ZMP relation contains angular-momentum rate:

```text
z_x = c_x - z_c/g * c_ddot_x - H_dot_y/(m g)
z_y = c_y - z_c/g * c_ddot_y + H_dot_x/(m g)
```

The current solver tracks:

```text
A_arm(q) dq_arm = H_target
```

Once arm velocity and momentum become constant, `H_dot_arm` approaches zero
and the useful reaction moment disappears. Update 4 instead treats:

```text
H_arm[k + 1] = H_arm[k] + dt * H_dot_command[k]
```

as the actuator state and dynamics.

### Braking Is Part Of The Balance Action

Returning `H_arm` to zero produces an opposite reaction impulse. Braking must
not be triggered by a fixed timer or by posture-return convenience. It begins
only when predicted capture state, base motion, and contact authority indicate
that the opposite impulse is safe.

### ZMP Is Not An External-Force Sensor

ZMP includes effects from the disturbance, lower-body policy, contact motion,
arm commands, and recovery. It remains a controlled output and safety signal,
but disturbance direction and episode timing must use source-timed IMU, COM,
contact, and command-aware residuals.

## Current Production Baseline

The accepted runtime remains the standing-qualified Update 3
`legacy_burst` profile. The failed timer-based state machine is disabled and
retained only as experimental code.

The canonical baseline is:

`runs/20260715_update3_standing_delay_sweep/`

It compares `upper_fixed` and `zmp_enabled` from a live standing-qualified
start with a continuous post-ready delay.

## Architecture

```text
source-timed state snapshot
    -> disturbance observer
    -> capture and support-state estimator
    -> flywheel momentum-rate planner
    -> joint acceleration/velocity realization
    -> measured H and H_dot feedback
    -> safe braking scheduler
```

### State

Use the planar state:

```text
x = [capture_point, COM_velocity, base_angular_velocity, H_arm]
```

with support-frame conventions:

- Origin: ankle midpoint.
- `+X`: forward.
- `+Y`: left.
- `+Z`: upward.
- Freeze support-frame yaw for one disturbance episode.

The capture point starts with:

```text
omega_0 = sqrt(g / z_c)
capture_point = c_xy + c_dot_xy / omega_0
```

### Input

The control input is:

```text
u = H_dot_arm
```

At joint level:

```text
H_dot_arm = A_arm(q) q_ddot_arm + A_dot_arm(q, q_dot) q_dot_arm
```

If the position servo makes this model inaccurate, use the experimentally
identified local command-to-`H_dot` map instead of increasing analytical gains.

### Control Objective

Use a short-horizon constrained regulator or MPC:

```text
minimize sum(
    ||capture_point - support_center||_Q_capture^2
    + ||base_angular_velocity||_Q_omega^2
    + ||H_dot_arm||_R_rate^2
    + ||delta H_dot_arm||_R_jerk^2
) + terminal_capture_cost
```

Constraints include:

- Arm momentum bounds.
- Momentum-rate bounds.
- Joint position, velocity, acceleration, and jerk.
- End-effector speed.
- Support polygon or contact-wrench margin.
- Position-servo command limits.

The terminal objective does **not** force `H_arm` to zero. It penalizes braking
risk and returns momentum only after capture safety is predicted.

## Response Phases

```text
ARMED -> SPIN_UP -> FLYWHEEL_HOLD -> SAFE_BRAKE
      -> RECOVERY_WAIT -> POSTURE_RETURN -> ARMED
```

### Spin Up

- Trigger from a source-timed disturbance estimate or simulation oracle.
- Command bounded `H_dot_arm` that moves capture state toward the support
  interior.
- Integrate measured, not only predicted, arm momentum.

### Flywheel Hold

- Keep useful `H_arm` available while external load or unsafe capture motion
  remains.
- `H_dot_arm` may be near zero in this phase; its purpose is to avoid paying
  back the impulse prematurely.
- Do not confuse momentum hold with continued rejection authority.

### Safe Brake

Choose braking rate and time from a predicted post-brake state:

```text
x_after_brake = predict(x_now, H_arm -> 0, lower_body_response)
```

Brake only if:

- Predicted capture point remains within a configurable support margin.
- Disturbance estimate is inactive.
- Base angular velocity is not moving outward.
- Contact and lower-body authority are valid.

If no safe brake exists, hold momentum and report capability exhaustion rather
than automatically returning.

### Posture Return

Posture return starts after arm momentum is near zero and balance remains quiet.
It uses separate low velocity, acceleration, and jerk limits and pauses on any
new balance risk.

## Staged Implementation

## Stage 0: Identical-State Benchmark Branching

Before changing runtime control, make comparisons deterministic.

- [ ] Save complete MuJoCo state at the qualified force-start marker.
- [ ] Save lower-policy, upper-controller, detector, and estimator state.
- [ ] Branch `upper_fixed` and candidate ZMP runs from the same snapshot.
- [ ] Apply force on the same simulation step, not only the same wall-clock
      marker.
- [ ] Record random seeds, configuration hashes, controller revisions, and
      exact force-step indices.

Acceptance:

- [ ] Pre-force state arrays are bit-identical between variants.
- [ ] Force onset differs by zero simulation steps.
- [ ] Ten repeated fixed-arm branches produce stable survival statistics.

## Stage 1: Oracle Separation Experiment

Use simulation truth to separate observer and actuator limitations.

- [ ] Trigger exactly at force onset.
- [ ] Supply true planar force direction and duration.
- [ ] Disable automatic arm braking.
- [ ] Test spin-up durations and deliberate braking times.
- [ ] Run `0.15 s`, `0.5 s`, and `1.0 s` force durations.
- [ ] Include equal-impulse comparisons.

Primary trials:

| Direction | Guard | Candidate |
| --- | --- | --- |
| `+Y` | `115 N` | `120 N` |
| `-Y` | `105 N` | `110 N` |
| `-X -Y` | `50 N` | `55 N` |
| `+X` | `30 N` | `35 N` |

Acceptance:

- [ ] Oracle control improves at least two candidate survival rates by 50
      percentage points over fixed arms in ten branches.
- [ ] No guard survival regression.
- [ ] If oracle control fails, stop runtime-observer work and increase or
      redesign physical arm authority first.

## Stage 2: Closed-Loop Reaction Identification

Identify the actual plant mapping instead of relying on `reaction_sign`.

- [ ] Apply small safe `+/-Lx` and `+/-Ly` momentum-rate pulses.
- [ ] Sweep representative arm postures and standing states.
- [ ] Measure base angular acceleration, COM acceleration, ZMP/CMP motion,
      contact moment, measured arm momentum rate, and delay.
- [ ] Fit a local mapping:

```text
[omega_dot_base, zmp_delta, c_ddot, contact_moment_delta]
    = B_identified(state) * arm_command
```

- [ ] Validate signs and cross-axis coupling on held-out pulse runs.

Acceptance:

- [ ] Predicted response sign is correct for every basis pulse.
- [ ] Normalized response prediction error is below `20%` in the operating
      region.
- [ ] Command-to-measured-`H_dot` delay and gain are repeatable within `10%`.

## Stage 3: Impulse Ledger

Add a trial-level physical accounting system:

```text
integral external_torque dt
delta H_arm
integral contact_moment dt
delta H_whole_body
```

Separate:

- Useful arm spin-up impulse.
- Flywheel hold interval.
- Harmful braking impulse.
- Posture-return impulse.
- Lower-body/contact contribution.

Acceptance:

- [ ] Ledger residual is below `10%` of applied external angular impulse.
- [ ] Useful and harmful arm impulses are reported separately.
- [ ] Every controller phase has explicit physical impulse accounting.

## Stage 4: Momentum-Rate Realization

Implement a new actuator path without replacing the accepted Update 3 path.

- [ ] Estimate current full-body arm momentum from measured state.
- [ ] Estimate `A_dot q_dot` with source-timed finite differences.
- [ ] Solve bounded joint acceleration for `H_dot_target`.
- [ ] Integrate acceleration into velocity and position commands.
- [ ] Include acceleration, jerk, position, velocity, and end-effector limits
      inside the solve.
- [ ] Feed predicted post-limit `H_dot` back into the planner.

Acceptance:

- [ ] Target-to-measured `H_dot` cosine exceeds `0.9`.
- [ ] Measured magnitude reaches at least `70%` of feasible target after known
      servo delay.
- [ ] No downstream limiter changes the optimized command by more than `1%` in
      99% of pre-fall samples.
- [ ] No joint or end-effector limit violation.

## Stage 5: Flywheel-Aware Planner

- [ ] Implement short-horizon capture-state prediction with `H_arm` as state
      and `H_dot_arm` as input.
- [ ] Add safe-braking feasibility to every planning step.
- [ ] Preserve useful momentum until braking is predicted safe.
- [ ] Report no-safe-brake and capability-exhausted conditions explicitly.
- [ ] Keep posture return outside the balance MPC.

Acceptance:

- [ ] No braking begins while predicted post-brake capture point is outside the
      support margin.
- [ ] Braking impulse does not reverse disturbance-aligned base velocity.
- [ ] Candidate improvements from the oracle stage are retained with measured
      state feedback.

## Stage 6: Runtime Disturbance Observer

Only after the oracle controller is effective:

- [ ] Propagate source sequence, timestamp, receive time, and state age.
- [ ] Reject stale or repeated samples.
- [ ] Transform IMU, COM, ZMP, and momentum quantities into the frozen support
      frame.
- [ ] Build a command-aware residual that subtracts predicted lower-body and
      arm effects.
- [ ] Estimate disturbance direction and confidence.
- [ ] Keep ZMP as a controlled output and safety signal, not the sole trigger.

Acceptance:

- [ ] Zero false triggers in 20 no-force branches.
- [ ] Median trigger latency below `100 ms`; maximum below `150 ms`.
- [ ] Direction error below `20 degrees` at controller activation.
- [ ] Runtime controller retains at least `90%` of oracle survival gain.

## Stage 7: Lower-Body Coordination

At minimum, publish expected arm reaction wrench and planned braking impulse to
the lower-body controller. Prefer a shared centroidal objective when that
interface is available.

- [ ] Lower controller receives expected arm reaction wrench and phase.
- [ ] Log whether lower-body action reinforces or cancels arm action.
- [ ] Compare independent and coordinated modes from identical state branches.

Acceptance:

- [ ] Contact impulse opposing the disturbance increases rather than being
      redistributed between controllers.
- [ ] Coordinated mode does not lose any oracle guard case.

## Validation Protocol

Use standing-qualified, identical-state, step-synchronized branches.

For each guard and candidate force:

- Run at least ten branches per variant.
- Report survival probability and confidence interval.
- Report capture-point peak, base angular-velocity peak, support margin, and
  fall time.
- Report useful spin-up impulse, braking impulse, and contact impulse.
- Report trigger latency, direction confidence, and measured `H_dot` tracking.

Only after repeated directional gates pass, run the eight-direction `5 N`
sweep with `upper_fixed` and `zmp_enabled`.

## Final Acceptance

- [ ] At least two directions improve by one `5 N` bracket with repeated
      survival evidence, not one binary-search outcome.
- [ ] No direction loses a previously passing boundary.
- [ ] Mean best passing force improves by at least `5 N`.
- [ ] Passing-boundary capture-point and angular-velocity peaks improve by at
      least `10%` in four directions.
- [ ] No pre-force command, no post-fall command, and no unsafe braking event.
- [ ] Runtime observer retains at least `90%` of oracle-controller benefit.

## Stop Conditions

Stop and redesign physical authority if oracle, identified-model control cannot
improve two directions. Stop and redesign lower-body coordination if arm
spin-up is useful in the ledger but contact impulse cancels it. Do not return to
component gain, burst-duration, or sign tuning without new identified plant
evidence.
