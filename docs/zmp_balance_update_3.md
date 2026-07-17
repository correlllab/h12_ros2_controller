# ZMP Balance Update 3

## Goal

Correct the direct reflex momentum mapping before changing detector gains or
recovery timing. Update 2 requests arm momentum with reduced isolated-arm
centroidal maps, but full-body diagnostics show that the resulting whole-robot
arm momentum is usually opposite the target.

## Fresh Baseline

The historical three-variant baseline in
`runs/20260714_update3_baseline_all/` predates the explicit `upper_fixed`
variant. It is retained for diagnosis only and is not used to compare Update 3
performance. All current and future Update 3 comparisons use `upper_fixed`
against `zmp_enabled` with ROS domain `2`, an `8 s` delayed force, a `1 s`
force duration, and `5 N` force brackets.

Across the Update 2 closure sweep, target and full-body predicted arm momentum
have approximately `-0.93` median cosine alignment at first response. The
velocity limiter is not the source of this reversal. The reduced arm maps used
to solve velocity are inconsistent with the current whole-body map used to
evaluate delivered momentum.

## Update 3 Scope

Implement a selectable `full_body_bounded` direct solver:

```text
minimize ||A_arm(q) dq_arm - L_target||^2 + lambda^2 ||dq_arm||^2
subject to joint velocity and one-step position bounds
```

`A_arm(q)` is the current full-body angular centroidal momentum map restricted
to the active arm joints. Both arms are solved jointly; the target is not split
before optimization.

The existing reduced direct solver remains available for rollback. Update 3
does not change observer thresholds, target gains, burst duration, cooldown, or
posture-return behavior until full-body momentum alignment is validated.

## Implementation Plan

- [ ] Add a direct solver selector with legacy `reduced_dls` and new
      `full_body_bounded` modes.
- [ ] Evaluate the current whole-body arm centroidal map once per control tick.
- [ ] Solve both active arms jointly with damped bounded least squares.
- [ ] Bound velocity by controller, direct-solver, URDF, and one-step position
      limits.
- [ ] Report per-arm contribution, combined feasible momentum, residual,
      solver status, and solve duration.
- [ ] Keep the existing upper-controller limiter as a final safety backstop.
- [ ] Require target-to-feasible momentum alignment above `0.9` in basis tests.
- [ ] Verify zero-force behavior before applying disturbances.
- [ ] Start with `-X -Y` at `45`, `50`, and `55 N`, because it is the only
      categorical Update 2 regression.
- [ ] Verify `+X` at `25`, `30`, and `35 N` as the forward-axis guard.
- [ ] Run a full three-variant sweep only after targeted trials are stable.

## Acceptance Gates

- [ ] Full-body predicted momentum aligns with the bounded target above `0.9`.
- [ ] Final joint limiter changes the bounded solution by less than `1%` when
      end-effector limits are inactive.
- [ ] No detector episode or arm command occurs in no-force trials.
- [ ] Update 3 does not lose a passive-passing targeted boundary.
- [ ] The `-X -Y` `50 N` regression is removed.
- [ ] At least one direction improves by `5 N`, or a boundary passing case
      reduces disturbance-aligned peak tilt by at least `10%` without another
      directional regression.

## Iteration Record

### Implemented

- Added `full_body_bounded` as a rollback-controlled direct solver.
- The solver uses the current full-body arm centroidal-map columns, solves both
  arms jointly, and includes controller, URDF, and one-step position bounds.
- Added explicit `reaction_sign: -1.0`, identified from the single-direction
  response experiments.
- Replaced the provisional one-sided `Lx` guard with symmetric
  `[-0.2, 0.2] Nms` bounds.
- Preserved the existing detector, target gains, burst timing, and return
  behavior to isolate allocator effects.
- Added a deterministic full-body mapping, bound, and alignment test.

### Targeted Results

- No-force: zero detector episodes and zero arm commands.
- `+X, 30 N`: survived with `0.170 rad` peak tilt.
- `-X -Y, 50 N`: Update 2 fell; Update 3 survived with `0.178 rad` peak tilt.
- `-X -Y, 55 N`: still fell.
- `-Y, 105 N`: survived with `0.155 rad` peak tilt.
- `-Y, 110 N`: still fell.
- `+Y, 120 N`: still fell.

### Failed Experiments

- `reaction_sign: +1`: software momentum alignment was correct, but
  `-X -Y, 50 N` fell. The arm reaction direction was physically wrong.
- Response time `0.24 s` and `Lx` cap `0.3 Nms`: did not raise the `55 N`
  boundary and worsened the fall trajectory.
- One-burst hold: reduced command duty cycle but worsened the `55 N` fall.
- ZMP residual-velocity entry at `0.0015 m/s`: reduced entry latency from about
  `0.51 s` to `0.25 s`, but made the previously passing `50 N` case fall.
- Direct duration `0.40 s`: did not raise the `55 N` boundary.
- Symmetric `Lx` cap `0.3 Nms`: worsened `-X +Y, 30 N` peak tilt.

### Full Sweep

The canonical fixed-arm comparison is stored in
`runs/20260715_update3_full_sweep/`. It interleaves `upper_fixed` and
`zmp_enabled` at every direction and binary-search iteration.

| Direction | Upper fixed | Update 3 ZMP | Delta |
| --- | --- | --- | --- |
| `+X` | `30 N` | `30 N` | `0 N`. |
| `+X +Y` | `50 N` | `50 N` | `0 N`. |
| `+Y` | `110 N` | `110 N` | `0 N`. |
| `-X +Y` | `30 N` | `30 N` | `0 N`. |
| `-X` | `25 N` | `25 N` | `0 N`. |
| `-X -Y` | `45 N` | `50 N` | `+5 N`. |
| `-Y` | `105 N` | `105 N` | `0 N`. |
| `+X -Y` | `40 N` | `40 N` | `0 N`. |

Upper-fixed mean is `54.375 N`; Update 3 ZMP mean is `55.0 N`. The corrected
full-body allocator produces a distinguishable `+5 N` improvement in the
`-X -Y` direction without losing another force boundary. This is the only
valid Update 3 performance comparison; legacy `lower_only` and `zmp_passive`
artifacts are diagnostic history, not current baselines.

At the largest passing force in each direction, Update 3 ZMP has higher peak
tilt in five directions and lower peak tilt in one direction, so the boundary
gain is not yet supported by a broad trajectory-quality improvement. Repeated
fixed-boundary trials remain necessary before accepting a statistical claim.

### Momentum Cap Robustness Iteration

Failure telemetry showed that successful and failed `-X -Y, 50 N` runs had
similar first two bursts, then diverged after force removal. Failed runs kept
requesting approximately `1.0 Nms` of `Ly`, while successful runs decayed.
Reducing the symmetric `Ly` bounds from `1.5` to `0.8 Nms` preserved the initial
response and reduced post-force command growth.

Targeted results with the `0.8 Nms` bound:

- Two initial `-X -Y, 50 N` repeats survived with `0.176` to `0.178 rad` peak
  tilt, while `55 N` remained a fall.
- `+X, 30 N`, `+X +Y, 50 N`, `-X, 25 N`, `-Y, 105 N`, and `+X -Y, 40 N`
  all retained their passing boundaries.
- `-X, 25 N` peak tilt was approximately `0.156 rad`, compared with
  `0.176 rad` for the fixed-arm trial used in the first sweep.

The full capped-momentum sweep is stored in
`runs/20260715_update3_ly08_full_sweep/`:

| Direction | Upper fixed | Capped Update 3 | Delta |
| --- | --- | --- | --- |
| `+X` | `30 N` | `30 N` | `0 N`. |
| `+X +Y` | `50 N` | `50 N` | `0 N`. |
| `+Y` | `115 N` | `110 N` | `-5 N`. |
| `-X +Y` | `30 N` | `30 N` | `0 N`. |
| `-X` | `25 N` | `25 N` | `0 N`. |
| `-X -Y` | `45 N` | `45 N` | `0 N`. |
| `-Y` | `105 N` | `110 N` | `+5 N`. |
| `+X -Y` | `40 N` | `40 N` | `0 N`. |

Both means are `55.0 N`. Repeated boundary trials demonstrate substantial
near-threshold variability:

- Capped ZMP survived `-X -Y, 50 N` in 3 of 4 runs; fixed arms survived 1 of
  2 runs.
- Capped ZMP survived `-Y, 110 N` in 1 of 3 runs; fixed arms survived 0 of
  2 runs.

These results suggest useful arm authority in both negative-Y directions, but
do not establish a statistically reliable boundary increase. The `0.8 Nms`
cap is retained because it lowers post-force command magnitude and improves
targeted trajectory quality. Further threshold tuning is stopped.

Rejected follow-up experiments:

- Increasing the `Lx` bound from `0.2` to `0.25 Nms` did not make
  `-Y, 110 N` survive.
- Limiting each detector episode to two bursts caused the existing
  `-X -Y, 50 N` gain to fail and was removed completely.

The result supports moving to Update 4's measured arrest and recovery state
machine. Fixed burst/return cycling cannot robustly separate corrective action
from post-force destabilization.

### Live Standing-Qualified Sweep

The benchmark now starts live simulation without force, requires a configured
standing dwell, then requires an additional `0.5 s` continuous-standing delay
before writing the impulse marker. The force interval is measured from that
marker rather than a fixed simulator timestamp.

The latest canonical sweep is stored in
`runs/20260715_update3_standing_delay_sweep/`. It used an `8 s` minimum live
settle interval, `1 s` qualification dwell, and `0.5 s` post-ready delay. The
actual ready-to-force delay across all 80 trials ranged from `0.640` to
`0.665 s`, with a `0.655 s` median.

| Direction | Upper fixed | Update 3 ZMP | Delta |
| --- | --- | --- | --- |
| `+X` | `30 N` | `30 N` | `0 N`. |
| `+X +Y` | `50 N` | `50 N` | `0 N`. |
| `+Y` | `110 N` | `115 N` | `+5 N`. |
| `-X +Y` | `30 N` | `30 N` | `0 N`. |
| `-X` | `25 N` | `25 N` | `0 N`. |
| `-X -Y` | `50 N` | `50 N` | `0 N`. |
| `-Y` | `105 N` | `105 N` | `0 N`. |
| `+X -Y` | `40 N` | `40 N` | `0 N`. |

Upper-fixed mean is `55.0 N`; Update 3 ZMP mean is `55.625 N`. The standing
gate removes the fixed-start ambiguity and produces a distinguishable positive
boundary in `+Y` without losing a directional boundary. This remains a
single-sweep result; repeated `+Y` `110` and `115 N` trials are still required
for statistical confidence.

## Update 4 Boundary

Defer the following if full-body bounded allocation alone is insufficient:

- Source-timestamp and stale-sample-aware observer reconstruction.
- Validated free-base velocity, contact confidence, and force-derived ZMP.
- Explicit support-frame transforms and reaction-sign identification pulses.
- Acceleration, jerk, braking-distance, end-effector norm, and collision QP
  constraints.
- Measured momentum feedback and servo-delay compensation.
- Reject, arrest, recovery-wait, and bounded-return state-machine replacement.
- Persistent-disturbance multi-pulse planning or horizon MPC.

Update 4 should start with source-timestamped transient detection and a
single-episode `REJECT -> ARREST -> RECOVERY_WAIT -> RETURN` state machine.
Earlier detection cannot be enabled safely with the current burst/return cycle,
as demonstrated by the residual-velocity experiment.
