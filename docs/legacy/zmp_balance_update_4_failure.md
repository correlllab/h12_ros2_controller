# ZMP Balance Update 4

## Goal

Turn the upper-body response into one measured reflex episode instead of a
fixed burst/return oscillator. The controller should reject the disturbance,
arrest arm motion, wait until the base settles, and only then return posture.

## Update 3 Evidence

The standing-qualified baseline is stored in
`runs/20260715_update3_standing_delay_sweep/`.

- Update 3 improves `+Y` from `110 N` fixed to `115 N` ZMP.
- All next-boundary falls occur after the one-second force ends.
- Detection occurs `0.386` to `0.620 s` after force onset.
- Posture return starts `0.569` to `0.801 s` after onset, while force is active.
- Return momentum is almost opposite the active target.
- Passing trials use `3` to `18` bursts and can continue commanding for more
  than `5 s` after force onset.
- The full-body bounded solve aligns predicted momentum with target; allocation
  is no longer the primary defect.

The strongest opportunities are `+Y, 120 N`, `-Y, 110 N`, and
`-X -Y, 55 N`. Mandatory guards are `+X, 30 N`, `-X, 25 N`, `+Y, 115 N`,
`-Y, 105 N`, and `-X -Y, 50 N`.

## Staged Plan

### Stage 1: Reflex Episode State Machine

Implement:

```text
IDLE -> REJECT -> ARREST -> RECOVERY_WAIT -> RETURN -> IDLE
```

- `REJECT`: continuously update and realize the bounded momentum target for at
  most `0.60 s`.
- `ARREST`: ramp the last arm velocity command to zero over `0.12 s`.
- `RECOVERY_WAIT`: hold the displaced arm posture with zero velocity.
- `RETURN`: start only after the detector is inactive and ZMP residual,
  residual velocity, and base angular velocity remain quiet for `0.30 s`.
- A new detector episode interrupts recovery or return and starts a new reject
  phase without recapturing the original return posture.

Verification goals:

- No posture return during the active one-second force.
- One reject episode and one return per disturbance.
- All guard cases pass.
- At least one candidate case improves before Stage 2.

### Stage 2: Safe Transient Entry

After Stage 1 is stable, add a two-sample transient entry using ZMP residual
velocity. Target first-command latency is `0.21` to `0.30 s`, down from
`0.39` to `0.62 s`.

Verification goals:

- Zero episodes in no-force trials.
- Maximum first-command latency below `0.33 s` at candidate forces.
- Every Stage 1 guard remains passing.

### Stage 3: Measured Arrest Feedback

Use measured arm momentum and base angular velocity for arrest decisions and
target damping. Do not increase target caps until measured delivery and phase
behavior are validated.

Verification goals:

- At least `10%` lower disturbance-aligned angular-velocity peak.
- Return-induced angular-velocity peak below `10%` of the disturbance peak.
- No post-fall arm command.

### Stage 4: Repeated Directional Validation

Run small repeated trials before a full sweep:

| Purpose | Direction and force |
| --- | --- |
| Preserve gain | `+Y, 115 N` |
| Seek gain | `+Y, 120 N` |
| Preserve guard | `-Y, 105 N` |
| Seek gain | `-Y, 110 N` |
| Preserve guard | `-X -Y, 50 N` |
| Seek gain | `-X -Y, 55 N` |
| Forward guard | `+X, 30 N` |
| Backward guard | `-X, 25 N` |

Only run the full eight-direction sweep after no-force and all guard trials
pass. Final acceptance requires at least two `5 N` directional gains without a
directional regression.

## Experiment Log

Implementation and trial results are appended below. Failed state transitions,
thresholds, and target changes are retained for later work.

### Implemented State Machine

- Added selectable `state_machine` response mode with `REJECT`, `ARREST`,
  `RECOVERY_WAIT`, and bounded existing `RETURN` phases.
- `REJECT` continuously updates the full-body bounded momentum target.
- `ARREST` ramps the last arm velocity command to zero.
- `RECOVERY_WAIT` holds displaced posture and requires detector exit plus quiet
  ZMP residual, residual velocity, and base angular velocity before return.
- New detector episodes interrupt recovery or return.
- Added angular velocity and angular acceleration telemetry.

No-force validation produced zero episodes and zero commands. With the original
detector, all mandatory guard cases passed, while `+Y 120 N`, `-Y 110 N`, and
`-X-Y 55 N` remained falls.

### Experiments

- ZMP-velocity entry at `0.0015 m/s` reduced first-command latency to
  `0.227–0.278 s` with no no-force trigger, but lost the diagonal guard.
- Extending reject from `0.60` to `0.80` and `1.0 s` did not cross a candidate
  boundary.
- A `0.5 Nms/(rad/s)` angular-velocity target did not cross a candidate and
  lost the diagonal guard. Its configured gain remains zero.
- Increasing `Lx` to `0.3 Nms`, increasing `Ly` to `1.2 Nms`, and raising joint
  authority to `1.5 rad/s` improved some fall delays but did not cross a
  candidate boundary.
- Reversing the negative-Y basis gain occasionally made `-Y 110 N` survive but
  consistently lost `+Y 115 N`. Residual-sign and roll-rate-latched versions
  were rejected and removed.

### Full Sweep

The state-machine sweep is stored in
`runs/20260715_update4_full_sweep/`.

| Direction | Upper fixed | State-machine ZMP | Delta |
| --- | --- | --- | --- |
| `+X` | `30 N` | `30 N` | `0 N`. |
| `+X +Y` | `50 N` | `50 N` | `0 N`. |
| `+Y` | `115 N` | `110 N` | `-5 N`. |
| `-X +Y` | `30 N` | `30 N` | `0 N`. |
| `-X` | `25 N` | `25 N` | `0 N`. |
| `-X -Y` | `50 N` | `50 N` | `0 N`. |
| `-Y` | `105 N` | `105 N` | `0 N`. |
| `+X -Y` | `40 N` | `40 N` | `0 N`. |

Upper-fixed mean is `55.625 N`; state-machine ZMP mean is `55.0 N`. Stage 1
fails the promotion gate. The production configuration is rolled back to
`legacy_burst`; the state machine remains available behind
`zmp.response.mode` for future source-timed work.

### Conclusion

The requested multi-direction `5 N` improvement was not achieved. The
standing-qualified Update 3 profile remains the best accepted configuration,
with a `+5 N` `+Y` gain and no sweep regression.

The experiment matrix rules out more fixed gain, cap, duration, and sign
tuning. Further progress requires a larger architecture increment:

- Source-timestamped and stale-sample-aware observation.
- Validated support-frame transforms for ZMP, IMU, and reaction momentum.
- Command-to-measured momentum system identification.
- Measured arrest transitions rather than fixed reject duration.
- Acceleration-constrained arm motion and interruptible return.
- Repeated boundary optimization using survival probability rather than one
  binary-search result.

All failed experiments are retained here to prevent repeating unsafe searches.
