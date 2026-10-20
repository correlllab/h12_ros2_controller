# ZMP Balance Update 1

## Goal

Replace the slow, open-loop two-arm DDP response used by the benchmark safety
split configuration with a bounded, closed-loop arm momentum response.

## Control Change

`balance_safety_split.yaml` selects `zmp.execution.mode: direct`. At each 30 Hz
control tick, each arm evaluates its reduced centroidal angular-momentum map at
the measured arm configuration:

```text
L_arm = A_L(q_arm) dq_arm
dq_arm = A_L^T (A_L A_L^T + lambda^2 I)^-1 L_target
```

The damped least-squares velocity is clipped by `direct_max_velocity`, then by
the existing arm joint and end-effector velocity limiter before it is integrated
into the upper-body position command.

## Response Envelope

- Direct correction is limited to a `0.20 s` burst.
- A `0.20 s` cooldown uses the existing bounded posture-return controller.
- A persistent disturbance can start another burst after the cooldown.
- The response saves the arm posture at activation and returns to that posture
  after the detector exits.
- Trigger entry requires three consecutive perturbed control ticks.
- The safety-split arm and end-effector limits are set to `1.25 rad/s`,
  `1.25 m/s`, and `2.5 rad/s`, respectively.

The bounded burst prevents a direct correction from continuously driving an arm
toward a joint limit while retaining per-tick target updates during the burst.
Other configurations retain the existing DDP mode by default.

## Evaluation

The matched 30 N, 1 s torso-force case survived with a maximum base tilt of
`0.2166 rad`, compared with `0.2365 rad` for the previous DDP-based controller.

The matched lower-only and ZMP-enabled variants were evaluated with an
eight-direction, five-iteration binary-search sweep. The force pulse matched
the manual benchmark at `5 s` start and `1 s` duration. The searched range was
`0` to `160 N`, and the resulting directional brackets have `5 N` resolution.
Artifacts are stored in `runs/20260713_both_fine_sweep/` in the benchmark
meta-repository.

| Direction | Lower-only | ZMP-enabled |
| --- | --- | --- |
| `+X` | `[25, 30) N` | `[30, 35) N` |
| `+X +Y` | `[35, 40) N` | `[35, 40) N` |
| `+Y` | `[100, 105) N` | `[95, 100) N` |
| `-X +Y` | `[30, 35) N` | `[30, 35) N` |
| `-X` | `[25, 30) N` | `[25, 30) N` |
| `-X -Y` | `[40, 45) N` | `[40, 45) N` |
| `-Y` | `[95, 100) N` | `[95, 100) N` |
| `+X -Y` | `[35, 40) N` | `[35, 40) N` |

Both variants have a mean best passing force of `48.125 N` in this sweep.
The directional `+X` bracket resolves the observed manual difference: the
lower-only variant falls at `30 N`, while the ZMP-enabled variant survives it.
The aggregate mean does not improve because the other directions are mostly
equal at this resolution and the `+Y` result is slightly lower for ZMP.

At the `+X` `30 N` trial, lower-only reached a maximum base tilt of `2.386 rad`
and fell; ZMP-enabled reached `0.218 rad` and survived.

## Update 2 TODO

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
