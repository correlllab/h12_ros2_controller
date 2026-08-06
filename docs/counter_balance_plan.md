# Counter-Balance Manipulation Plan

## Goal

Extend the benchmark with Cartesian arm-workspace trials, establish whether
simple CoM and ZMP margins explain manipulation-induced balance loss, then add a
minimal `CounterBalanceController` that reserves one arm for manipulation and
uses the opposite shoulder and elbow through Crocoddyl to preserve balance.

The implementation must synchronize this finalized plan and measured findings
to `submodules/h12_ros2_controller/docs/counter_balance_plan.md`. Add numerical
findings only with exact artifact paths and distinguish exploratory runs from
repeated validation.

## Locked Decisions

- `free_arm` is the arm released to manipulation frame tasks.
- `free_arm` is fixed to `left` or `right` at controller initialization.
- The opposite arm is reserved for counter-balancing.
- Free-arm frame tasks may use all seven joints on that arm.
- Counter-balancing may use shoulder pitch, shoulder roll, shoulder yaw, and
  elbow only; counter-arm wrist joints remain held.
- The lower-body policy and torso command are unchanged.
- The first controller runs in double-support handless simulation.
- The benchmark grid uses absolute pelvis-frame XYZ targets and mirrors the
  lateral sign for left and right arms.
- The first grid tests position only; wrist orientation cost is zero.
- The first CoM target preserves the pre-manipulation CoM offset relative to the
  live support region rather than driving to its geometric center.
- The support region is a simple oriented rectangle built from fixed sole bounds
  and the live foot frames.
- A reached target counts as balanced only after returning to a continuous
  settled-standing envelope; survival is reported separately.
- The first implementation is exposed through the Python controller API and
  benchmark runtime only. ROS action-server and safety-layer delegation are
  deferred.
- Planned path execution, stepping, payloads, and hardware are out of scope.

## Existing Constraints

- The root worktree already contains uncommitted `arm_probe` benchmark work.
  Preserve it and do not revert unrelated changes.
- `FrameController` solves on a 14-joint two-arm Pink model and currently has no
  arm-ownership rule.
- `UpperController._apply_velocity_command()` publishes zero desired velocity;
  model-based commands must use the atomic `q`, `dq`, and gravity-torque pattern
  from `DirectZmpController`.
- The current inferred ZMP omits floating-base velocity and contact wrench. It
  is suitable for diagnostics, not a candidate-state Crocoddyl objective.
- The existing support rectangle is not physically usable: its lateral
  half-extent is `0.06 m`, while nominal ankle centers are near
  `y = +/-0.163 m`.
- Existing `MomentumDDP` and `BalanceDDP` provide selected-arm, bound, solver,
  and diagnostic patterns, but neither models static CoM redistribution from a
  live manipulation arm.

## Metrics

### Foot-Frame Support Rectangle

Treat each foot as a fixed sole rectangle in its ankle-roll frame. Use resolved
configuration defaults verified against the checked-in handless collision mesh:

```yaml
support_geometry:
  front: 0.174
  rear: 0.086
  half_width: 0.043
  max_yaw_divergence: 0.349  # 20 degrees
```

For each sample:

1. Project both foot-forward axes into the horizontal plane.
2. Align their signs and normalize their average as the support-forward axis.
3. Use its planar perpendicular as the lateral axis.
4. Transform both feet's four sole corners.
5. Project all eight corners onto the common axes.
6. Use the minimum and maximum projections as one oriented support rectangle.
7. Return center, axes, half-extents, validity, and invalid reason.

Mark the region invalid for missing frames, degenerate axes, nonfinite geometry,
or excessive foot-yaw divergence. Implement this as a pure NumPy helper shared
by controller diagnostics and benchmark analysis. Do not infer mesh bounds at
runtime.

For point `p`, define signed margin:

\[
m(p) = \min(h_f - |(p-c) \cdot a_f|,
            h_l - |(p-c) \cdot a_l|).
\]

Positive margin is inside the rectangle.

### CoM

Record both Pinocchio model CoM and MuJoCo ground-truth CoM. For each, store:

- Position relative to support center.
- Signed support margin.
- Pre-motion reference.
- Transition minimum.
- Final-hold mean and minimum.

Report model-versus-simulator bias and RMS error. The controller uses model CoM;
MuJoCo CoM remains independent validation.

### Contact ZMP

For each MuJoCo robot-floor contact, transform `mj_contactForce()` from contact
to world coordinates, orient the wrench as force on the robot, and sum:

\[
F = \sum_i F_i, \qquad
M = \sum_i (r_i \times F_i + \tau_i).
\]

At support-plane height `z_s`:

\[
zmp_x = (z_s F_x - M_y) / F_z, \qquad
zmp_y = (M_x + z_s F_y) / F_z.
\]

Return invalid when vertical force is below a configured threshold. Store ZMP,
vertical force, validity, valid-sample fraction, and support margin. Keep the
controller-inferred ZMP as a separately named series.

### Manipulation And Stability

Store and summarize:

- Target and achieved wrist position.
- Linear frame error, first convergence time, and continuous hold duration.
- Target displacement from the measured starting wrist pose.
- Base roll, pitch, height, linear velocity, and angular velocity.
- Fall and survival status.
- Settled-standing status and continuous settled duration.
- Free- and counter-arm positions, velocities, and joint-limit margins.
- Solver count, failures, mean and maximum solve time.
- Predicted zero-command and optimized CoM errors.
- Pre-limit and applied counter commands.
- Clipping, collision rejection, invalid-support, and estop statuses.

Use distinct outcomes:

- `survived`: no fall during the complete evaluation.
- `reached`: linear error stays below threshold for the required target hold.
- `settled`: final state stays within configured standing height, tilt, base
  linear-velocity, and base angular-velocity limits for the required hold.
- `passed`: complete process and simulation logs, `survived`, `reached`, and
  `settled`.

CoM and ZMP margins remain explanatory metrics until experiments establish that
they are repeatable and predictive.

## Control Formulation

### Arm Ownership

Implement `CounterBalanceController(FrameController)`.

Use Pink 4.2's `constraints` argument with a `LinearHolonomicTask` whose
selection matrix covers all seven reserved-arm reduced joints, target is zero,
and reference is refreshed to the current reduced configuration. This imposes
zero reserved-arm velocity as a hard IK equality constraint, so Pink's
self-collision barrier solves with the reserved arm actually fixed. Masking is
retained only as a defensive assertion.

Accept any frame task whose reduced kinematic support mask is nonempty and a
subset of the selected free-arm mask. Reject unknown, torso, opposite-arm, and
mixed-support frames before changing active tasks.

### Reference

At target release, capture:

- Counter shoulder/elbow reference `q_c_ref`.
- Counter wrist hold position.
- Initial model CoM offset from the live support center:
  `e_com_ref = com_xy - support_center`.

At each tick, recompute the live support rectangle and set:

\[
p_{com,target} = support_center + e_{com,ref}.
\]

If support geometry is invalid, continue the free-arm task, hold the counter
arm, and log `invalid_support`.

### Crocoddyl State And Control

Use only four counter-arm joints:

\[
x_k = q_{c,k} \in \mathbb{R}^4, \qquad
u_k = \dot q_{c,k} \in \mathbb{R}^4.
\]

Use one velocity-controlled transition:

\[
q_{c,k+1} = integrate(q_{c,k}, \Delta t u_k).
\]

Before solving:

1. Solve constrained free-arm Pink IK.
2. Apply free-arm limits.
3. Integrate once to predict the free-arm position for this tick.
4. Build a full motor-position template from live lower body and torso,
   predicted free arm, current counter wrist, and candidate counter
   shoulder/elbow.
5. Build the full free-flyer template once from measured joint state and IMU,
   then replace motor coordinates per candidate. Do not call helpers that mutate
   shared robot-model data from Crocoddyl action evaluation.

Evaluate full-model CoM with dedicated Pinocchio data owned by each action-data
object. Restrict the CoM Jacobian to the four counter motor columns after the
free-flyer columns.

Use a one-step running and terminal problem:

\[
\ell = \frac{w_u}{2}\|u\|^2
      + \frac{w_q}{2}\|q_{c,next}-q_{c,ref}\|^2
      + \frac{w_l}{2}\|q_{soft-limit}\|^2,
\]

\[
\ell_f = \frac{w_{com}}{2}
         \|com_{xy}(q_{full,next})-p_{com,target}\|^2
         + \frac{w_{q,f}}{2}\|q_{c,next}-q_{c,ref}\|^2.
\]

Provide analytical derivatives and compare them with finite differences in
unit tests. Normalize CoM error by `0.01 m`, counter posture error by `1 rad`,
and velocity by each joint's effective publisher velocity limit before applying
dimensionless weights. Set Crocoddyl bounds from the stricter of the model,
controller, and publisher limits so downstream clipping is exceptional rather
than normal. Keep soft-limit cost active inside the hard boundary to preserve
counter-arm stroke. Do not add ZMP, momentum, tilt, force estimation,
acceleration, jerk, or return phases before this formulation is measured.

Start the normalized development configuration at:

```yaml
counter_balance:
  maxiter: 5
  w_com: 1.0
  w_control: 0.01
  w_posture: 0.01
  w_terminal_posture: 0.01
  w_soft_limit: 1.0
  soft_limit_margin: 0.10
  improvement_tolerance: 1.0e-6
```

These are development defaults, not accepted gains. Keep them only if the
declared development subset supports them.

Accept a finite solved or best-effort command only when predicted CoM error is
not worse than the zero-counter-velocity candidate within tolerance. Otherwise
hold the counter arm and log `no_improvement`.

### Command Composition

Each control tick:

1. Update model and Pink state once.
2. Solve free-arm IK with the reserved-arm hard lock.
3. Apply free-arm limits and predict its next position.
4. Solve the four-joint counter-arm Crocoddyl problem.
5. Reject empty, nonfinite, non-improving, or out-of-bound output.
6. Merge free-arm and counter shoulder/elbow velocities; set counter wrist
   velocity to zero.
7. Apply final per-arm limits.
8. Check the combined next motor configuration against joint and self-collision
   limits.
9. Backtrack only the counter command through fixed scales to zero when it makes
   the combined candidate invalid.
10. If the constrained free-arm-only candidate is invalid, hold both arms and
    log the rejection.
11. Integrate once, compute gravity torque once, and atomically publish both arm
    slices as `q`, `dq`, and `tau`.

On solver failure, free-arm motion continues and the counter arm holds. With no
active frame task, do not solve counter-balance. After estop, publish nothing but
keep the benchmark process alive for complete logs.

## Ordered Implementation Plan

### 1. Measurement Foundation

- Add the pure support-rectangle and signed-margin helper.
- Add pure contact-wrench-to-ZMP reduction helpers with synthetic tests.
- Extend MuJoCo recording with foot transforms, support rectangle, contact ZMP,
  validity, vertical force, CoM margin, and ZMP margin.
- Extend analysis with pre-motion, movement, and final-hold windows.
- Preserve old artifact compatibility by returning unavailable metrics when new
  arrays are absent.

### 2. Shared Workspace Benchmark

Add variants:

- `frame_task`: unchanged `FrameController` baseline.
- `counter_balance`: new controller with identical target timing.

Add a validated section:

```yaml
arm_workspace:
  target_type: frame
  free_arm: left
  frame: left_wrist_yaw_link
  target_position: [0.35, 0.20, 0.10]
  position_cost: 20.0
  orientation_cost: 0.0
  lm_damping: 1.0
  move_duration: 2.0
  target_hold_duration: 2.0
  linear_threshold: 0.01
  settled_hold_duration: 1.0
  support_geometry:
    front: 0.174
    rear: 0.086
    half_width: 0.043
    max_yaw_divergence: 0.349
```

Also allow `target_type: named_config` for `frame_task` metric baselines only.
Reject named configurations for `counter_balance` in version one.

Use one shared runtime and bridge. It must instantiate either controller with
`init=False`, capture references at `impulse.flag`, interpolate only target
position, solve continuously through the final hold, write
`arm_workspace.jsonl`, remain alive after estop, and shut down resources in
`finally`.

Extend process specs, readiness, dry-run plans, provenance, and summary parsing.
Fix controller snapshot networking so the selected domain and interface, not the
hard-coded domain-2 overlay, are written to generated profiles.

### 3. Baseline Metric Experiments

First run no-force named configurations with normal `FrameController`:

- `home`.
- `arms_front_45`.
- `t_pose`.
- `arms_asym`.
- `arms_front_yaw` only if offline collision validation passes.

Use one exploratory trial, then three repeats for representative low-, medium-,
and high-margin cases. Verify signs, support geometry, ZMP validity, model CoM
error, and transition versus steady-hold noise.

Then run known archived position targets for both arms before expanding the grid.
Do not begin the full grid until mirrored reachable targets converge and the
metric pipeline is valid.

Document whether CoM and ZMP margins are monotonic with extension, repeatable,
and separated between settled and unsettled outcomes. Retain negative findings.

### 4. Cartesian Grid Sweep

Add a separate `arm_workspace_sweep.py`; do not overload `arm_reaction_sweep.py`.
The sweep configuration contains lists of absolute pelvis-frame X, lateral
magnitude, and Z values. Generate the full Cartesian product and mirror lateral
sign by arm.

Retain this initial coarse grid unless offline IK rejects a coordinate before
simulation:

```yaml
grid:
  x: [0.25, 0.35, 0.45]
  lateral: [0.20, 0.30, 0.40]
  z: [0.08, 0.16, 0.24]
```

After discovery, create a second full Cartesian grid from midpoint coordinates
adjacent to pass/fail transitions and variant-disagreement cells. Record the
parent coarse cells and refinement rule in the manifest. Do not silently merge
coarse and refined cells into a uniform-volume claim.

Each grid cell is one fresh benchmark run for each selected arm, variant, and
trial. Write a checkpoint after every run and support resume by the complete
cell/arm/variant/trial key. Do not early-stop within the grid.

Outputs:

- `arm_workspace_manifest.json`.
- `arm_workspace_results.csv`.
- Per-height pass/fail and outcome maps.
- CoM/ZMP margin maps.
- Paired variant-difference maps.
- Per-height maximum-forward and maximum-outward frontiers.
- A successful-cell count and grid-cell-volume proxy, explicitly labeled as a
  discretized comparison rather than physical workspace volume.

Use one discovery trial per coarse cell. Run one trial per refined cell. Repeat
refined pass/fail boundaries, cells where variants disagree, and adjacent
common-pass/common-fail controls three times. Majority decides the repeated
cell; a tie fails.

#### Catalog Motion-Group Sweep

After the offline reachability catalog is accepted, add a separate
catalog-driven configuration sweep. It must not be merged with the Cartesian
grid or reported as a continuous workspace volume.

- The saved catalog at `data/arm_reachability_candidates.yaml` labels every
  direction and candidate as either `directional` or `overhang`.
- The `directional` group contains the fixed five-point forward, upward,
  forward-outward, diagonal, and cross-body lines for each arm.
- The `overhang` group contains the fixed five-point, strict `0, 80, 0` degree
  wrist-pitch lines. These include outer and inner forward/upward lines plus
  the center-crossing lateral line.
- Every left/right pair must use the same rank and reflected Cartesian target:
  identical X/Z, opposite Y, and reflected orientation. Keep only pairwise
  offline-valid ranks.
- Run each catalog group independently for every arm, variant, trial, and
  movement profile. The profiles are quasi-static and dynamic, each followed
  by the same hold and standing-settle qualification.
- Write independent checkpoints, manifests, CSV results, summaries, plots, and
  frontier tables for `directional` and `overhang`. Include `motion_group`,
  catalog direction ID, candidate rank, saved 14-joint configuration, and
  profile in every trial record.
- Write a direction-rank aggregate CSV and dashboard section for every group
  and profile. Plot CoM margin, contact-ZMP margin, configuration error, and
  outcome against ranks one through five for each direction.
- Do not combine group pass counts, margins, frontiers, or grid-cell proxies.
  Compare variants only within the same motion group and profile.
- Replay and benchmark the saved joint configuration directly after offline
  filtering. Do not re-solve the Cartesian target during visualization or the
  configuration benchmark.
- Use `frame_task` as the initial baseline. The configured profiles are
  quasi-static (`6 s` move, `4 s` target hold) and bounded-fast (`1.5 s` smooth
  direct trajectory, `3 s` target hold), with zero external impulse unless a
  later force study changes that condition. Runtime limits are `dq_lim: 6`,
  `v_lim: 3`, and `w_lim: 8`, with full publisher velocity clipping. Do not use
  scaled one-step IK feedback for fast configuration replay.
- Classify each complete trial as green when it passes, orange when it survives
  with excessive configuration error or failed reach/settle, and red when it
  falls. Keep infrastructure failures separate from these physical outcomes.
- Render a MuJoCo replay MP4 only for red trials when
  `video.enable: true`; retain the `.npz` trace and an explicit unavailable
  reason when a replay cannot be produced.

### 5. Counter-Balance Solver And Controller

- Implement a simple reactive counter-arm controller before further Crocoddyl
  tuning. Treat the saved moving-arm trajectory as fixed and control the
  opposite arm's shoulder pitch, roll, yaw, and elbow.
- Form a bounded counter-arm velocity command from moving-arm feedforward and
  balance feedback. Feedforward minimizes predicted centroidal momentum from
  the moving arm. Feedback uses simulator-independent controller signals:
  support-relative CoM error, torso angular velocity, and their filtered rates.
- Solve a damped bounded least-squares problem each tick. Penalize residual
  centroidal momentum, support-center error, counter-arm speed, and deviation
  from a captured counter-arm posture. Enforce effective publisher velocity and
  position limits before publication.
- Fade counter motion to posture hold after the moving trajectory ends. Reject
  nonfinite, collision-invalid, or support-invalid commands and publish a
  zero-velocity counter-arm hold instead.
- Keep the existing one-step Crocoddyl controller as an experimental comparison,
  not the initial reactive implementation. Its static CoM objective has not yet
  demonstrated closed-loop disturbance rejection.
- Add a dedicated four-joint `CounterBalanceDDP` module rather than extending
  the existing disturbance DDP classes.
- Add `CounterBalanceController` with fixed arm roles, constrained Pink IK,
  reference capture, solver acceptance, collision backtracking, atomic command
  publication, and diagnostics.
- Preserve an optional `counter_balance` config mapping for weights, limits, and
  solver iterations. Arm role and controlled joint set remain constructor/code
  decisions in version one.
- Add an offline model check over representative grid poses to verify that the
  optimized counter direction reduces predicted CoM error before closed-loop
  runs.
- Declare a small development subset before tuning: center, far-forward,
  far-outward, and far-forward/outward cells for the left arm, plus their
  mirrored right-arm checks.
- Tune only dimensionless CoM, posture, control, and soft-limit weights on that
  subset. Record every accepted and rejected setting with config hashes.
- Freeze weights before the discovery grid. Do not tune on repeated boundary
  validation results; a new tuning iteration requires a new labeled experiment
  series.

### 6. Comparative Validation

Run in this order:

1. Compile changed modules and run unit tests.
2. Generate dry-run command plans for both variants.
3. Run no-displacement/no-force cases for both variants.
4. Run one known reachable left target and its mirrored right target with
   `frame_task`.
5. Repeat both with `counter_balance`.
6. Inspect ownership, tracking, solver timing, clipping, support validity, and
   collision status.
7. Run the discovery grid for both variants.
8. Repeat boundary and disagreement cells.
9. Only after a static effect is established, rerun representative interior and
   boundary cells with selected cardinal external forces.

Do not run a full force-by-workspace product in the first iteration.

## Source Boundaries

Controller submodule:

- Add `core/support_region.py`.
- Add `core/counter_balance_ddp.py`.
- Add `core/controller/counter_balance_controller.py`.
- Minimally extend `ik_solver.py` only to pass hard constraints through the
  reduced one-step IK path.
- Keep `frame_controller.py` unchanged. Configure workspace-only frame-task
  costs and target validation in the new runtime and counter controller.
- Preserve optional counter-balance config in `utility/controller_config.py`.
- Add focused controller, DDP, support-region, and derivative tests.
- Synchronize this plan and measured findings to the requested submodule doc.

Benchmark repository:

- Extend `runtime/mujoco_runtime.py` and `analysis.py`.
- Add one shared arm-workspace runtime and bridge.
- Extend variant/config/process/snapshot wiring.
- Add one workspace sweep and plot module.
- Add reproducible single-run and grid configs.
- Add metric, config, grid, aggregation, process-spec, and analysis tests.

Do not modify `DirectZmpController`, `BalanceDDP`, or legacy controller behavior
unless a separate shared bug is demonstrated.

## Failure Modes

- Invalid support geometry: hold counter arm, continue free task, mark run data.
- Wrong-arm or mixed frame task: reject before replacing active tasks.
- Pink infeasibility: hold both arms and report task failure.
- Crocoddyl failure/nonfinite/no improvement: continue free task, hold counter.
- Counter-induced collision: backtrack counter command to zero.
- Invalid free-only candidate: hold both arms.
- Publisher clipping: log requested and applied commands and recompute predicted
  applied CoM change for diagnostics.
- Estop: stop output, remain alive, and fail the trial without truncating logs.
- Missing controller log or invalid ZMP: do not treat as zero; mark metric or run
  incomplete as appropriate.

## Verification

Unit tests must cover:

- Foot geometry, mirrored/aligned frames, yaw divergence, and signed margins.
- Known synthetic contact-wrench ZMP and low-force invalidity.
- Workspace config validation and left/right grid mirroring.
- Separate `survived`, `reached`, `settled`, and `passed` aggregation.
- Resume keys and repeated-cell majority.
- Runtime domain/interface propagation.
- `free_arm` validation and same-arm/mixed-arm task admission.
- Pink hard lock producing zero reserved-arm IK velocity.
- Four-joint counter output and held counter wrists.
- Full candidate reconstruction with predicted free arm and live lower body.
- CoM derivatives against finite differences.
- Solver failure and no-improvement behavior.
- Atomic merged command fields and joint IDs.
- Collision backtracking and free-only rejection.
- Old `sim.npz` analysis compatibility.

Acceptance for a claimed workspace extension:

- Identical target cells and timing for both variants.
- Complete process and metric artifacts.
- No no-target or interior-target regression.
- Solver maximum observed time below one control period.
- At least three repeated trials for every claimed differing boundary cell.
- Majority pass at at least one cell that baseline fails, with adjacent control
  cells reported.
- Comparable free-arm target error between variants.
- Clear separation of IK failure, survival failure, settled-balance failure, and
  infrastructure failure.

Improved margin without repeated grid-cell extension is a valid result but must
not be described as workspace extension.

## Deferred Work

- ROS action-server integration.
- Generic safety-layer controller delegation.
- Runtime arm-role switching.
- Planned-path counter-balance overlay.
- Counter-arm collision constraints inside Crocoddyl.
- Dynamic support polygons, single support, and stepping.
- Wrist payload/contact estimation.
- Whole-body or torso optimization.
- Orientation workspace sweeps.
- Hardware deployment.

## Findings Record

### Metric Baseline

Two single-trial, no-force exploratory integration runs completed on 2026-08-04.
These are smoke tests, not repeated validation:

- `frame_task` artifact:
  `/tmp/kilo/counter_balance_smoke_20260803/20260804_000347_frame-smoke_arm_workspace_frame_task/summary.json`.
- `counter_balance` artifact:
  `/tmp/kilo/counter_balance_smoke_20260803/20260804_000539_counter-smoke_arm_workspace_counter_balance/summary.json`.
- Both runs completed all required processes, survived, and met the settled
  standing criterion without an estop.
- Neither run reached the selected pelvis-frame target. Final linear error was
  `0.11091 m` for `frame_task` and `0.10748 m` for `counter_balance`.
- Final-hold minimum simulator CoM margin was `0.13931 m` for `frame_task` and
  `0.13551 m` for `counter_balance`. Final-hold minimum contact-ZMP margin was
  `0.09534 m` and `0.09491 m`, respectively.
- The counter-balance run's pelvis-frame model-versus-simulator CoM comparison
  used 196 aligned samples. XY bias was `[0.00382, 0.00283] m`, and combined XY
  RMS error was `0.00339 m`.
- These two independent smoke trials do not establish metric monotonicity,
  repeatability, or a counter-balance improvement.

### Normal Frame-Task Grid

Pending implementation and experiment.

### Catalog Metric Response Diagnostic

The preserved rank-5 artifact is
`runs/20260805_135802_arm_reachability_balance_sweep/`. It is useful for metric
validation but is deprecated as a controller baseline because its fast profile
used scaled one-step IK feedback and produced violent arm-joint ringing.

- Stable quasi-static directional trials kept minimum CoM margins between
  `0.09804 m` and `0.13697 m`; contact-ZMP margins remained positive between
  `0.01777 m` and `0.12307 m`.
- Quasi-static overhang targets produced smaller support reserves despite tiny
  final configuration errors. The most disturbing stable examples reached
  `0.06638 m` CoM margin and `0.00925 m` contact-ZMP margin.
- In falling fast-overhang trials, CoM margins crossed below zero to
  `-0.12687 m`, while contact-ZMP margins reached `-0.66380 m`. Base tilt,
  support invalidity, and survival status changed consistently with those
  margin crossings.
- Contact-ZMP margin is the earliest and most sensitive disturbance metric in
  these examples. CoM margin is smoother and better for sustained support loss.
  Configuration error must remain separate because it measures arm tracking,
  not balance.
- The metrics therefore reflect physical balance disturbance and loss of
  support, but the red outcomes cannot be attributed cleanly to intended arm
  motion because the deprecated command path was unstable. Do not use its red
  count as evidence of a counter-balance opportunity or controller efficacy.
- New baseline and counter-balance comparisons must use the bounded `1.5 s`
  direct trajectory and report rank-wise metric trends, not only final rank-5
  classifications.

### Counter-Balance Model Validation

The exploratory counter-balance run executed 399 solver calls with four
best-effort rejection/failure ticks. Mean solve time was `2.15 ms`, maximum
solve time was `5.38 ms`, and the configured control period was `20 ms`. No
publisher clipping, collision rejection, invalid support, or estop was recorded.

During the movement window, mean predicted optimized CoM error was `0.00120 m`
versus `0.00131 m` for the zero-counter-velocity candidate. During final hold,
the corresponding means were `0.000656 m` and `0.000711 m`. This confirms the
online solver and acceptance path operated, but does not validate a closed-loop
workspace benefit because the target was not reached and the trial was not
repeated.

### Comparative Grid

Pending implementation and experiment.

### External-Force Spot Checks

Pending implementation and experiment.

### Final Limitations

The metric response has been validated qualitatively, but the preserved red
trials used a deprecated unstable command path. No repeated bounded-trajectory
baseline, counter-balance comparison, tuned-weight series, or external-force
spot check has been completed. No workspace extension is claimed.
