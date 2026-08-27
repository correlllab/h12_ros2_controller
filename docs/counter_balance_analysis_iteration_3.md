# Counter-Balance Iteration 3 Analysis

## Status

Iteration 3 is implemented as an experimental finite-horizon counter-arm
controller. The current candidate is suitable for focused FAME and ALMI
simulation comparison. It is not promoted for general use or hardware.

This document records implementation findings and measured changes. The design
and promotion plan remain in
[counter_balance_iteration_3.md](counter_balance_iteration_3.md).

## Implementation

The implementation adds two focused modules:

- `counter_ddp_ocp.py`: reusable Crocoddyl action models, frozen balance-knot
  data, Box-FDDP solve, shifted feasible warm start, and solve diagnostics.
- `counter_ddp_controller.py`: `CounterDDPController`, moving/counter arm
  ownership, horizon construction inputs, Pinocchio maps, hard validation,
  collision backtracking, publication, and controller diagnostics.

The existing `CounterBalanceController` and its one-step least-squares objective
are unchanged.

### OCP

The state and control are:

\[
x_k=[q_{c,k},\dot q_{c,k}]\in\mathbb{R}^8,
\qquad
u_k=\ddot q_{c,k}\in\mathbb{R}^4.
\]

The action model uses exact constant-acceleration discrete dynamics. It evaluates
the verified normalized CoM-velocity and angular-momentum residuals at each
interval endpoint. Pinocchio CoM Jacobians and angular centroidal maps are frozen
for one solve and rebuilt on the next 50 Hz control tick.

The running cost contains:

- CoM-velocity cancellation.
- Angular-momentum cancellation and measured gyro feedback.
- Counter posture regularization.
- Nominal acceleration regularization.
- Counter velocity regularization.
- A high-weight soft position/excursion limit cost.

`SolverBoxFDDP` enforces acceleration bounds. Position, velocity, excursion,
trust-region, and collision checks remain controller-side acceptance gates.

### Runtime Candidate

The initial active benchmark candidate uses:

| Parameter | Value |
|---|---:|
| Control period | `0.02 s` |
| Horizon | 3 intervals, `0.06 s` |
| FDDP iterations | 1 |
| CoM weight | 1.0 |
| Momentum weight | 2.0 |
| Posture weight | 0.02 |
| Acceleration weight | 0.01 |
| Velocity weight | 0.02 |
| Soft limit weight | 10.0 |
| Maximum acceleration | `25 rad/s^2` |
| Maximum acceleration change per tick | `5 rad/s^2` |
| Excursion envelope | `[0.35, 0.28, 0.20, 0.28] rad` |

The moving arm is an immutable exogenous trajectory. The benchmark supplies
`T + 1` exact analytic position and velocity samples. Sample zero is published
without controller modification. Publisher clipping remains authoritative and
is reported separately as moving-command error.

## Verification

Focused tests cover:

- Constant-acceleration state transitions.
- Analytical cost and dynamics derivatives against central differences.
- Crocoddyl terminal control dimension.
- Box-constrained finite trajectories.
- Shifted warm starts.
- Moving-arm position, velocity, and supplied-torque preservation.
- Counter-wrist hold behavior.
- Shadow-mode counter hold.
- Horizon and timestamp validation.
- Position, velocity, excursion, and acceleration-slew bounds.
- Controller and benchmark configuration loading.
- Runtime process and horizon plumbing.
- Sweep aliases and summary aggregation.
- The complete existing reactive controller test set.

The focused controller and benchmark suite passed `99` tests after integration.

Final validation after response gating and review fixes:

- Complete benchmark suite: `110` passed.
- Controller suite excluding unavailable ROS ament linters and one unrelated
  named-configuration smoke failure: `113` passed.
- Remaining reduced-planner tests: `10` passed with the unrelated smoke test
  deselected.
- Changed Python modules: byte compilation passed.
- Root and controller diffs: `git diff --check` passed.

The unrun ROS lint tests require `ament_copyright`, `ament_flake8`, and
`ament_pep257`, which are not installed in the `uv` environment. The unrelated
planner smoke test references missing named configuration `t_pose_elbow`; this
iteration did not modify named configurations or the planner.

## Development Runs

### Five-Knot Initial Smoke

Artifact:

`runs/key_findings/20260826_231220_counter_ddp_fame_smoke`

The first active implementation used five knots, two FDDP iterations, and no
in-solve state-limit cost.

Findings:

- The case completed stable.
- Moving position and velocity errors remained below `6e-8`.
- The counter arm exhausted the tight excursion envelope and reached about
  `0.566 rad` on one proximal joint.
- `problem_build_failure` occurred on 538 of 682 active samples after the state
  left a region with feasible one-step position and slew bounds.
- Total-controller p99 was `17.7 ms`; maximum was `26.3 ms`.
- Solve p99 was `8.3 ms`; maximum was `14.1 ms`.

Conclusion: Box acceleration bounds alone do not preserve finite-stroke state
room. The OCP required explicit velocity and soft excursion costs. Smoothness
bounds also could not override safety recovery.

### State-Limit Fix

Artifact:

`runs/key_findings/20260826_231449_counter_ddp_fame_smoke_v2`

Changes:

- Added normalized velocity regularization.
- Added a soft position/excursion limit residual active in the outer 10% of the
  effective range.
- Allowed safety bounds to override acceleration-slew bounds when their
  intersection is infeasible.
- Reduced the horizon from five to three intervals.

Findings:

- The case completed stable.
- Problem-build failures were eliminated.
- Maximum observed proximal counter position was about `0.256 rad`.
- Most ticks returned valid FDDP best-effort solutions.
- Total-controller p99 remained `20.4 ms` because every candidate was evaluated
  with duplicate full-horizon collision and nonlinear Pinocchio checks.

Conclusion: the state cost corrected the finite-stroke behavior, but duplicate
nonlinear validation did not fit the 50 Hz timing budget.

### Real-Time Baseline

Artifact:

`runs/key_findings/20260826_231828_counter_ddp_fame_smoke_v4`

Changes:

- Used one warm-started FDDP iteration.
- Retained all predicted-state position, velocity, excursion, and trust-region
  checks.
- Used the existing first-command collision backtracking as the active collision
  gate.
- Disabled duplicate nonlinear horizon validation in the active configuration.
  It remains configurable for shadow and analysis runs.

Findings:

- The case completed stable.
- Total-controller p50, p95, p99, and maximum were approximately `4.65`,
  `10.49`, `13.01`, and `15.64 ms`.
- Solve p50, p95, p99, and maximum were approximately `1.74`, `4.90`, `7.94`,
  and `8.69 ms`.
- Two samples exceeded the conservative `15 ms` diagnostic guard; none exceeded
  the `20 ms` control period.
- Maximum observed proximal counter position was about `0.286 rad`.
- Moving-arm command errors remained below `6e-8`.

Conclusion: three knots and one warm-started iteration form the first viable
real-time candidate. Crocoddyl frequently reports best effort rather than
convergence in one iteration, so acceptance correctly depends on cost, finite
trajectory, bounds, trust region, and collision validation rather than the
convergence Boolean.

### ALMI Smoke Comparison

Artifacts:

- DDP:
  `runs/hard_sweep/20260826_231934_counter_ddp_almi_smoke`.
- Frame task:
  `runs/hard_sweep/20260826_232026_counter_ddp_almi_smoke_frame`.

Target: `right_upward_overhang_pitch_plus`.

Findings:

- Both frame task and DDP classified as drift and survived.
- DDP final moving-arm error was `0.00772 rad`.
- Frame final moving-arm error was `0.01123 rad`.
- DDP final CoM margin was `0.08502 m`.
- Frame final CoM margin was `0.08751 m`.
- DDP total-controller p99 was `10.20 ms`; maximum was `11.78 ms`.
- No DDP timing guard overrun occurred.
- Maximum reported moving-position command error was approximately `0.001 rad`
  because the common publisher clipped a target at its configured position
  margin. The pre-publisher moving target was not changed by DDP.

Conclusion: the first ALMI comparison showed no classification regression and
better tracking, but slightly worse CoM margin. One trial is not promotion
evidence.

### Safety-Contract Review And Final Smoke

Artifact:

`runs/key_findings/20260826_233128_counter_ddp_fame_smoke_v5`

An independent code review found and corrected four command and evidence
contracts before the focused sweep:

- A solve exceeding the timing guard now publishes a counter hold instead of an
  active result.
- Collision backtracking records effective command acceleration from measured
  velocity and published desired velocity rather than scaling nominal
  acceleration incorrectly.
- Invalid or stale forecast metadata now preserves the valid sample-zero moving
  command while holding only the counter arm.
- A benchmark runtime error now makes the run operationally incomplete and
  ineligible for stable physical classification.

Timestamp validation now also requires sample zero to equal the monotonic
generation time and the final sample to equal the expected horizon endpoint.
The inherited frame path retains its published zero velocity while forwarding
the pre-integration IK velocity to future forecast knots.

Final smoke findings:

- The case completed stable with no runtime error.
- No timing hold was required.
- Total-controller p50, p95, p99, and maximum were approximately `4.51`, `9.01`,
  `11.99`, and `14.18 ms`.
- Statuses were 678 accepted best-effort solves and six rejected
  future-state-out-of-bounds trajectories.
- Moving-arm tracking remained unchanged to the benchmark tolerance.

Conclusion: the reviewed candidate is the first clean configuration used for
the FAME and ALMI focused comparisons. The earlier interrupted broad FAME run is
development-only evidence and must not be aggregated with final results.

## Focused Sweep Scope

The new sweep configurations combine only:

- `arm_fast_boundary_targets.yaml`.
- `arm_exploration_targets.yaml`.
- `arm_hard_targets.yaml`.

Directional targets are excluded from repeated iteration-3 tuning. Overhang
families remain represented in the hard group.

The configurations compare `counter_ddp` directly against `frame_task` on both
FAME and ALMI:

- `config/sweep_configs/fame_magpie_fast_hard_groups_counter_ddp.yaml`.
- `config/sweep_configs/almi_magpie_fast_hard_groups_counter_ddp.yaml`.

## Ungated Focused Panels

The first complete panels used full feedforward authority and established the
need for response gating.

Artifacts:

- FAME:
  `runs/hard_sweep/20260826_233226_counter_ddp_fame_hard_groups_final`.
- ALMI:
  `runs/hard_sweep/20260827_041156_counter_ddp_almi_hard_groups_final`.

FAME outcomes over 44 targets:

| Controller | Stable | Drift | Stumble | Fall | Survived |
|---|---:|---:|---:|---:|---:|
| Frame task | 21 | 13 | 0 | 10 | 34 |
| Ungated DDP | 22 | 15 | 0 | 7 | 37 |

Ungated DDP produced five classification wins and one loss:

- `left_fast_fall_search_06_scale_74`: fall to drift.
- `right_fast_fall_search_11_scale_78`: fall to drift.
- `right_inner_upward_overhang_pitch_plus`: fall to drift.
- `left_lateral_high_reach`: drift to stable.
- `right_upward_arc_rank6`: drift to stable.
- `right_diagonal_rank6`: stable to drift regression.

ALMI outcomes over 44 targets:

| Controller | Stable | Drift | Stumble | Fall | Survived |
|---|---:|---:|---:|---:|---:|
| Frame task | 25 | 11 | 5 | 3 | 41 |
| Ungated DDP | 22 | 14 | 5 | 3 | 41 |

Ungated DDP produced no ALMI win and three stable-to-drift regressions:

- `right_fast_fall_search_06_scale_74`.
- `right_fast_fall_search_11_scale_78`.
- `right_lateral_overhead_reach`.

Conclusion: finite-horizon control retained and slightly improved the FAME hard
authority of the reactive controller, but continuous intervention still failed
the cross-policy no-regression requirement.

### Response-Gate Identification

The shared right-boundary targets prove that moving-arm feedforward risk cannot
distinguish policy interaction: the same trajectory is a FAME rescue and an
ALMI regression.

Early frame-only base-response analysis showed:

- FAME rescue cases reached approximately `0.045` to `0.133 rad/s` in early
  Euler roll or pitch rate.
- The three ALMI regressions remained below approximately `0.039 rad/s` in the
  same derived metric.

Controller IMU rates were larger, so scalar thresholds at `0.04`, `0.10`, and
`0.12 rad/s` were tested on compact sentinels. Scalar gating could either act on
an ALMI pitch transient or enter too late for the FAME boundary rescue.

The final development gate is symmetric but axis specific:

| Axis | Entry rate | Full rate |
|---|---:|---:|
| Roll | `0.08 rad/s` | `0.10 rad/s` |
| Pitch | `0.20 rad/s` | `0.25 rad/s` |

An axis activates only while settled-reference tilt and angular velocity have
the same sign and tilt magnitude exceeds `0.001 rad`. Before the first
activation, the controller skips Crocoddyl and publishes the captured settled
counter reference. After an active episode, zero response authority retains the
OCP with balance costs disabled so acceleration-limited posture recovery can
complete.

Development sentinels with this gate:

- FAME `left_fast_fall_search_06_scale_74`: fall baseline, DDP drift.
- FAME `right_fast_fall_search_11_scale_78`: fall baseline, DDP drift.
- FAME `right_inner_upward_overhang_pitch_plus`: fall baseline, DDP drift.
- ALMI `right_fast_fall_search_06_scale_74`: stable baseline, DDP stable.
- ALMI `right_fast_fall_search_11_scale_78`: stable baseline, DDP stable.
- ALMI `right_lateral_overhead_reach`: stable baseline, DDP stable.

These are development results, not held-out promotion evidence. Final 44-target
response-gated panels are required before claiming cross-policy no regression.

## Frozen Response-Gated Evaluation

The final candidate was frozen with:

- Response-only activation.
- Roll entry/full rates of `0.08/0.10 rad/s`.
- Pitch entry/full rates of `0.20/0.25 rad/s`.
- Minimum tilt of `0.001 rad`.
- New episode entry only while the exogenous moving-arm horizon contains motion.
- Warm-start reset when axis authority changes by more than `0.25`.
- Exact inactive reference publication before the first response episode.
- Acceleration-limited OCP posture recovery after an active episode.

The moving-horizon entry condition removed a FAME stable-to-drift regression
whose only trigger occurred after moving-arm completion. Warm-start transition
reset prevents one-iteration FDDP from replaying stale active controls after an
axis gate changes.

### FAME

Artifacts:

- Frozen DDP:
  `runs/key_findings/20260827_095515_counter_ddp_entry_fame_final`.
- Matched frame baseline source:
  `runs/hard_sweep/20260826_233226_counter_ddp_fame_hard_groups_final`.

Outcomes over 44 targets:

| Controller | Stable | Drift | Stumble | Fall | Survived |
|---|---:|---:|---:|---:|---:|
| Frame task | 21 | 13 | 0 | 10 | 34 |
| Frozen DDP | 23 | 13 | 0 | 8 | 36 |

There were four DDP wins and no loss:

- `left_fast_fall_search_06_scale_74`: fall to drift.
- `left_fast_fall_search_09_scale_78`: drift to stable.
- `left_lateral_high_reach`: drift to stable.
- `right_inner_upward_overhang_pitch_plus`: fall to drift.

Total-controller p50, p95, p99, and maximum were approximately `4.31`, `10.28`,
`13.39`, and `49.21 ms`. Pre-publication timing overruns held the counter arm.
Rare post-write stalls remained visible after the atomic command write and are
reported separately from accepted solver timing.

### ALMI

Artifacts:

- Frozen DDP:
  `runs/key_findings/20260827_101112_counter_ddp_entry_almi_final`.
- Matched frame baseline source:
  `runs/hard_sweep/20260827_041156_counter_ddp_almi_hard_groups_final`.

Outcomes over 44 targets:

| Controller | Stable | Drift | Stumble | Fall | Survived |
|---|---:|---:|---:|---:|---:|
| Frame task | 25 | 11 | 5 | 3 | 41 |
| Frozen DDP | 25 | 11 | 5 | 3 | 41 |

There were no paired classification or survival changes. The three ungated
stable-to-drift regressions were eliminated.

Total-controller p50, p95, p99, and maximum were approximately `3.50`, `8.14`,
`13.03`, and `39.46 ms`. Nine ticks reported a post-write timing overrun. No
runtime error occurred.

### Interpretation

The single frozen panels meet the iteration-3 development objective:

- FAME has net survival and classification improvement.
- ALMI has no regression.
- Moving-arm ownership remains intact.
- The p99 synchronous controller time remains below `15 ms` on both policies.

This is single-run panel evidence. It is not enough for promotion across
stochastic boundary cases. The next evaluation should repeat changed and
threshold-near cases from identical initial state before any further parameter
change.

## Simplification After Evaluation

The response-gated candidate used in the frozen panels accumulated several
experiment-specific conditions: different roll and pitch thresholds, a minimum
gate floor, a post-motion entry rule, and an activation-change warm-start reset.
Those conditions were useful for locating the FAME/ALMI difference, but they are
not retained as the controller design.

The current implementation uses one scalar, policy-independent rule:

\[
\alpha=
\operatorname{clip}\left(
\frac{\lVert\omega_{xy}\rVert-\omega_{enter}}
{\omega_{full}-\omega_{enter}},0,1
\right),
\]

where `alpha` is zero unless the supplied moving-arm horizon contains speed
above `moving_velocity_threshold`. The same `alpha` scales both planar CoM and
angular-momentum residuals. With no authority, the counter arm holds rather
than solving or applying an additional return heuristic.

The sweep configurations use `0.10 rad/s` entry, `0.15 rad/s` full authority,
and `0.001 rad/s` moving-arm speed threshold. These are ordinary controller
parameters, not FAME/ALMI branches. They should be tuned from repeated panels,
not individual outcome labels.

Retained constraints are general safety and model contracts rather than
scenario-specific hacks:

- Four-joint counter-arm ownership only.
- Position, velocity, acceleration, acceleration-change, and captured-excursion
  bounds.
- Soft near-limit cost plus hard command validation.
- Collision validation and first-command backtracking.
- Exact moving-arm pass-through.
- Fresh, timestamped moving-arm horizon requirement.
- Hold on model, solver, bound, collision, or pre-publication timing failure.

The historical frozen-panel results above were generated before this
simplification and must not be attributed to the simplified gate. The simplified
controller needs a fresh focused comparison before promotion.

Clean post-simplification smokes:

- FAME `left_fast_fall_search_06_scale_74`:
  `runs/key_findings/20260827_134410_counter_ddp_simple_fame_passthrough_smoke`.
  The frame baseline falls; simplified DDP drifts and survives with 11 active
  ticks and `5.87 ms` p99 controller time.
- ALMI `right_fast_fall_search_11_scale_78`:
  `runs/key_findings/20260827_134317_counter_ddp_simple_almi_passthrough_smoke`.
  The frame baseline is stable; simplified DDP is stable with zero active ticks
  and `5.32 ms` p99 controller time.

These two smokes validate the simplified activation contract but do not replace
the required fresh 44-target cross-policy evaluation.

## Video Review Rule

Video selection now renders both sides of every changed paired outcome and every
latest stumble, including a stumble with no paired classification change. Replay
is available without rerunning simulation:

```bash
uv run python -m h12_zmp_benchmark.sweep.arm_reachability_sweep \
  --config CONFIG \
  --output-dir EXISTING_RUN_DIR \
  --resume \
  --render-videos
```

Replay mode forces video generation and caps the render size at `640x480`, the
MAGPIE offscreen framebuffer limit. It updates the existing manifest and writes
videos below `EXISTING_RUN_DIR/videos/`.

The FAME and ALMI Counter DDP hard-group configurations enable `640x480` replay
by default. Future runs therefore render every selected changed outcome and
every latest stumble automatically.

## Current Limitations

- The horizon is only `0.06 s`; this is a few-step reactive controller, not a
  long-horizon manipulation planner.
- The fixed-base model does not predict lower-body policy or contact response.
- Moving-arm acceleration affects the forecast through future velocities, but
  arm-to-base dynamics are not identified.
- Library defaults retain full feedforward authority for compatibility, while
  the evaluated sweep configs use symmetric response-only divergence gates.
- Recovery uses the same OCP with balance costs disabled; it does not yet have a
  separately identified braking model or quiet-dwell state machine.
- The active candidate validates future numerical state bounds but collision
  checks only the command being published. Full-horizon nonlinear validation is
  available through `validation_steps` for shadow analysis but missed the
  current timing target when enabled broadly.
- Nominal acceleration is a command-space quantity. Its measured actuator gain
  and delay have not been identified.
- Braking viability and a dedicated quiet-dwell recovery state machine remain
  future work.
- One FDDP iteration usually returns best effort rather than solver convergence.
- Hardware remains out of scope.
- Rare publisher-side stalls can cross the timing guard after the atomic command
  write. Solver/model/torque work is guarded before publication, but a hard
  cancellable wall-clock deadline would require an asynchronous architecture.

## Next Decisions

Use the focused FAME and ALMI sweep outcomes to decide whether to:

1. Retain full feedforward authority and tune only weights and excursion.
2. Add symmetric predicted-risk activation to suppress benign ALMI and FAME
   motion.
3. Add measured directional recovery before increasing horizon length.
4. Re-enable one-step nonlinear metric validation if timing remains below one
   period across the full focused panels.

Do not add momentum-rate cost or an identified base-response model until the
current three-knot candidate has cross-policy outcome evidence.
