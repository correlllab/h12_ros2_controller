# Direct-Outcome Upper-Body Balance Control Plan

## Purpose

This document is the implementation and validation plan for a new upper-body
balance controller. The controller will optimize executable upper-body control
inputs against predicted balance outcomes. It will not track a predefined arm
angular-momentum target.

The first implementation will control both arms by default, support either arm
being reserved, and keep the accepted Update 3 controller available as an
independent runtime backend. The architecture will support manipulation task
constraints without requiring the balance controller to own the manipulation
planner.

The new public implementation will be:

```text
h12_ros2_controller/core/controller/zmp_controller.py
```

Its modules will live under:

```text
h12_ros2_controller/core/controller/zmp/
```

The accepted implementation will remain frozen at:

```text
h12_ros2_controller/core/controller/zmp_controller_legacy.py
h12_ros2_controller/core/controller/zmp_legacy/
config/legacy/balance_safety_split.yaml
```

## Current-Code Findings

### Reusable Controller Infrastructure

`UpperController` already owns the useful low-level infrastructure:

- `RobotModel` and measured DDS state access.
- Fixed-base and free-flyer Pinocchio models.
- Reduced arm model and sphere self-collision model.
- `IKSolver` and Pink task objects.
- `LowCmdHandler`, command publication, gains, clipping, and state-limit
  monitoring.
- Frame Jacobians, wrist transforms, gravity compensation, and the final DDS
  command path.
- `init=False`, which the benchmark requires to avoid the startup trajectory.

The new `ZmpController` will inherit `UpperController`, but it will not use
`_apply_velocity_command()` as its primary realization path. That method
integrates a velocity into a position target and sends zero desired velocity,
which hides the executable position-servo input from the optimizer. The new
controller will issue an explicitly optimized `q_cmd`, `dq_cmd`, and
gravity/feed-forward `tau_cmd` through `LowCmdHandler.set_joint_commands()`.

`MomentumController` and `MomentumDDP` provide useful examples of selected-arm
indexing, current-state model refresh, plan storage, and full-body command
expansion. Their hold/swing/return plan is not reusable as the new control law:
it has arm position as state, arm velocity as input, tracks angular momentum,
uses soft position limits, and performs an unconditional posture return.

### Reusable Robot-Model Infrastructure

The following APIs should be reused:

- `RobotModel.state`, `state_reduced`, and `update_kinematics()`.
- `RobotDynamics.get_com()` and `get_com_velocity()` after state-estimation
  limitations are addressed.
- `get_angular_centroidal_momentum_matrix()` and current full-body arm columns.
- `get_angular_centroidal_momentum()` for measured arm momentum diagnostics.
- `get_frame_jacobian()` and `compute_frame_twist()`.
- `get_gravity_compensation()` and `get_motion_compensation()`.
- Pinocchio collision geometry already loaded by `UpperController`.
- Processed `q`, `dq`, and `tau` clip and estop limits from
  `load_controller_config()`.

The Update 3 correction must be retained: all arm effectiveness calculations
must use current whole-body model columns, not isolated arm models frozen at
construction.

### State And Model Gaps

The current balance estimates are not sufficient to claim physical prediction
accuracy:

- `RobotModel.full_q()` reconstructs orientation but leaves base translation at
  zero.
- `RobotModel.full_v()` always inserts zero floating-base velocity.
- The MuJoCo bridge does not populate joint acceleration in `LowState`.
- The current ZMP calculation assumes both feet are in contact and uses the
  ankle midpoint as a support plane.
- No contact state, foot wrench, support polygon, or state covariance reaches
  the upper controller.
- The DDS subscriber exposes host receive time but no source sequence, source
  time, stale-sample check, or synchronization contract.
- The lower-body policy and upper-body controller have no explicit reaction
  wrench or braking-impulse interface.

The implementation must expose validity and confidence for every derived
balance quantity. Invalid ZMP, contact, or capture estimates must not silently
become zeros or normal measurements.

### Existing Physical And Safety Constraints

The repository has position, velocity, torque, wrist-speed, collision, and
publisher clipping infrastructure, but the accepted controller applies some of
it after solving. Acceleration and jerk are not enforced by the safety layer.
The effective simulator torque is:

```text
tau_effective = tau_cmd
              + kp * (q_cmd - q)
              + kd * (dq_cmd - dq)
```

The new optimizer must include this actuator-interface relation and the exact
processed limits. Publisher and safety-layer clipping will remain independent
backstops, not normal command shaping.

The Pinocchio URDF and MuJoCo/controller shoulder-roll ranges currently differ.
The initial safe bounds must use the intersection of model, controller, and
safety limits, and diagnostics must report the source of each active bound.

### Manipulation And Reference Infrastructure

`FrameController`, `ArmController`, and `IKSolver.frame_tasks` provide reusable
frame-target representations and Jacobians. They do not provide task priority,
allowed deviation, derivatives, timestamps, or balance availability. The
legacy `single_arm_task_balance` mode only chooses an assist arm; it does not
preserve a manipulation constraint.

The new controller therefore needs a small reference and task contract rather
than another fixed `assist_arm` assumption.

### Legacy Failure To Avoid

The accepted legacy path computes a desired momentum rate, multiplies it by a
response time, solves for arm velocity that produces an angular-momentum value,
then returns the arms after a fixed burst. This fails structurally because:

- Balance is affected by angular-momentum rate, not constant momentum.
- Stopping arm momentum is the opposite impulse even if posture return is
  delayed.
- Fixed-time braking can occur while the disturbance is active or capture
  motion is unsafe.
- The solve omits servo dynamics, acceleration, jerk, braking distance,
  collision, task, and contact constraints.
- A post-solve limiter changes the predicted action.
- Predicted centroidal momentum is treated as delivered support response.
- The benchmark effect is comparable to timing variation and is not
  statistically established.

The new controller will not contain a momentum-target estimator, fixed burst,
fixed cooldown, or unconditional return-to-start sequence.

## Design Principles

1. Optimize balance outcomes directly. Arm momentum is a predicted and measured
   state and a finite resource, not the tracking reference.
2. Optimize the command that the actuator interface will receive. Every model
   quantity must trace to `q_cmd`, `dq_cmd`, and `tau_cmd`.
3. Keep physical constraints hard. Balance-margin slack is minimized before
   task and posture costs; posture restoration never wins over safety.
4. Certify a braking and recovery contingency before accepting the first input
   of a new plan.
5. Separate assistance, momentum braking, quiet settling, and posture
   restoration.
6. Keep the nominal or task reference persistent across a disturbance. Never
   redefine the disturbed posture as the nominal reference.
7. Close the loop on measured joint response, IMU response, estimated balance
   outcomes, and final delivered command behavior.
8. Make arm availability and task constraints inputs to allocation. Do not
   encode both-arm use into the optimizer dimensions.
9. Keep the accepted backend independently runnable for rollback and A/B
   comparison.
10. Promote behavior only through repeated, controls-equivalent, synchronized
    benchmarks.

## Proposed Modules

### `zmp/types.py`

Defines immutable or copy-safe dataclasses and enums shared across modules:

- `BalancePhase`.
- `MeasuredState`.
- `BalanceState`.
- `SupportState`.
- `DisturbanceEstimate`.
- `NominalReference`.
- `FrameConstraint`.
- `ArmAvailability`.
- `OptimizationRequest`.
- `ControlTrajectory`.
- `BrakeCertificate`.
- `BalanceDiagnostics`.

No module should exchange unstructured dictionaries inside the control loop.
Serialization belongs in `BalanceDiagnostics.to_dict()`.

### `zmp/state_estimator.py`

Builds a source-timed measured state and balance state:

- Validates finite state, sample age, monotonic sequence/time, and repeated
  samples.
- Estimates filtered joint acceleration from measured velocity and actual
  sample time.
- Transforms IMU orientation, angular velocity, and acceleration into a frozen
  support frame for each disturbance episode.
- Computes torso roll/pitch and angular velocity.
- Computes relative COM position and velocity with explicit validity flags.
- Computes arm angular momentum and measured momentum rate.
- Computes ZMP/CMP, support margin, and capture/DCM state when their required
  inputs are valid.
- Maintains quiet-standing biases only in the nominal phase and freezes them
  before the first command-aware disturbance update.
- Reports contact confidence rather than fabricating a valid contact result.

The benchmark implementation should subscribe to the simulator base-state
source when available. Hardware without that source will use a configured
estimator path and will be prevented from enabling outcome terms whose inputs
are invalid.

### `zmp/reference_manager.py`

Owns the persistent nominal upper-body reference and manipulation constraints.

Required behavior:

- Initialize idle nominal posture from `zmp.reference.idle_posture`, with the
  balance configurations setting it to the configured zero posture.
- Store joint reference, optional joint velocity, frame references, source,
  timestamp, and generation number.
- Accept a current task reference and freeze a snapshot at balance-assist entry
  when requested by policy.
- Preserve the same nominal generation through assist, hold, brake, settle,
  and restore phases.
- Expose per-arm availability: `available`, `limited`, `reserved`, or
  `disabled`.
- Expose hard task axes, allowed task slack, and optional redundancy weights.
- Reject malformed, stale, or dimensionally inconsistent references.

Planned controller API:

```python
set_nominal_joint_reference(q, dq=None, source='idle')
set_task_reference(frame_constraints, freeze_on_assist=True)
clear_task_reference()
set_arm_availability(arm, availability)
get_reference_snapshot()
```

The future manipulation controller can keep a reserved arm's frame constraints
active while the balance optimizer uses the other arm and any declared
redundancy. No command stream will be overwritten outside the declared
decision variables.

### `zmp/control_allocation.py`

Converts arm availability, task support masks, and configuration into the
current decision-variable set:

- Supports both arms, left-only, right-only, and configured torso inclusion.
- Removes disabled and fully reserved joints.
- Keeps task-arm joints available only through null-space or bounded task slack
  declared by the task reference.
- Reports available outcome authority, joint-range reserve, collision reserve,
  and the reason a joint or arm was excluded.
- Does not split a momentum target or assign fixed left/right weights.

The optimizer sees one coupled upper-body decision vector. This preserves
cross-arm coupling and naturally allocates control to whichever joints can
improve the outcome while satisfying task and safety constraints.

### `zmp/outcome_model.py`

Predicts the balance consequences of candidate executable commands over a
short horizon.

The state includes, when valid:

```text
q, dq, previous acceleration
q_cmd, dq_cmd
COM position and velocity
capture/DCM state
torso tilt and angular velocity
arm angular momentum
ZMP/CMP and support margin
disturbance estimate and confidence
```

The command is an executable upper-body command trajectory or an equivalent
acceleration parameterization that is deterministically converted to
`q_cmd`, `dq_cmd`, and `tau_cmd` before constraint evaluation.

The first model will combine:

- Joint command integration and the configured position-servo relation.
- Current full-body centroidal-map columns.
- Momentum-rate prediction using
  `A(q) * qdd + Adot(q, dq) * dq`, with `Adot * dq` estimated from
  source-timed map differences.
- Capture/DCM propagation using measured COM height and a support-frame model.
- CMP/ZMP change from predicted upper-body momentum rate.
- A configurable local response correction from executable command to measured
  torso angular acceleration, COM acceleration, and support response.
- A constant or decaying short-horizon disturbance estimate with explicit
  uncertainty.

There is no desired momentum trajectory. Predicted arm momentum is integrated
only to enforce finite range, momentum, braking, and terminal safety.

The response correction starts as a conservative configured model. Online
adaptation will be bounded, confidence-gated, and disabled until basis-pulse
identification validates sign and delay. Prediction error is always logged.

### `zmp/constraints.py`

Builds the exact feasible set for the current decision variables and horizon:

- Joint position bounds using the intersection of Pinocchio, processed
  controller, and safety bounds.
- Joint velocity bounds from processed clip limits and tighter balance limits.
- Joint acceleration bounds.
- Joint jerk bounds relative to the previously delivered acceleration.
- Position-command slew and desired-velocity bounds.
- Feed-forward torque and predicted effective PD torque bounds.
- End-effector linear/angular velocity and acceleration bounds.
- Configurable end-effector workspace and floor/torso keep-out bounds.
- Self-collision minimum distance and velocity-damper constraints.
- Reserved-arm and manipulation frame constraints.
- Support/CMP/capture safety margins when the estimator marks them valid.
- A terminal recoverable set and joint-range reserve for braking and later
  posture restoration.

Physical and actuator constraints are hard. Already-violated balance margins
use nonnegative slack so the optimizer can select the least unsafe action.
Balance slack is optimized lexicographically before task deviation, effort, or
posture costs.

The final publisher and safety layer remain active. A difference between raw
optimized and final applied commands is a diagnostic failure, not expected
operation.

### `zmp/optimizer.py`

Solves a warm-started receding-horizon problem at each control tick.

Inputs:

- Current measured and estimated state with validity/confidence.
- Frozen support frame and disturbance estimate.
- Persistent nominal/task reference snapshot.
- Current decision-variable allocation.
- Previous delivered command and measured response.
- Previous feasible plan and braking certificate.
- Current controller phase.

Outputs:

- First executable `q_cmd`, `dq_cmd`, and `tau_cmd`.
- Predicted state and balance-outcome trajectory.
- Active constraints and safety slacks.
- Predicted arm momentum and momentum rate as diagnostics/resource states.
- A complete feasible backup braking/settling trajectory.
- Solver status, iterations, residuals, and elapsed time.

Primary outcomes, in priority order, are:

1. Avoid predicted fall, contact loss, or invalid actuation.
2. Maximize minimum support/CMP and capture margin.
3. Reduce outward capture/DCM velocity and displacement.
4. Reduce torso tilt, outward angular velocity, and predicted peak tilt.
5. Improve terminal recovery quality and preserve lower-body/contact reserve.
6. Preserve hard manipulation constraints and minimize allowed task slack.
7. Preserve braking range and collision margin.
8. Minimize command acceleration, jerk, and unnecessary end-effector motion.
9. Track the persistent posture reference only with the phase-specific weight.

The posture weight is small during assist, hold, brake, and settle. It increases
during restore, but never changes the constraint or balance priority ordering.

The low-level solver and exact control parameterization will be selected from
the existing `qpsolvers`, SciPy, Pink, and Pinocchio stack after a timing spike.
The preferred first formulation is a sequentially linearized, warm-started QP
over joint acceleration or command increments because it can represent jerk,
servo torque, task Jacobians, and a short braking tail at 30 Hz. A nonlinear
solver will only be used if it meets the same deterministic deadline and
fallback contract.

### `zmp/phase_manager.py`

Owns transitions based on measured state and certified predictions, not fixed
burst timers.

Phases:

| Phase | Purpose | Posture behavior |
| --- | --- | --- |
| `DISARMED` | Observe while output is disabled or state is invalid. | Hold the persistent reference safely. |
| `CALIBRATING` | Establish quiet biases and validate state sources. | Track idle/task reference. |
| `NOMINAL` | Normal standing or manipulation. | Track persistent reference. |
| `ASSIST` | Produce outcome-improving upper-body action. | Reduce posture weight; keep task constraints. |
| `HOLD` | Preserve useful arm state when braking is not yet safe. | Do not restore posture. |
| `BRAKE` | Remove arm momentum through a certified safe trajectory. | Do not restore posture. |
| `SETTLE` | Verify measured balance and arm momentum after braking. | Hold the post-brake posture. |
| `RESTORE` | Return toward the persistent nominal/task reference. | Increase posture weight gradually. |
| `EXHAUSTED` | No fully safe assist/brake plan exists. | Execute stored least-risk contingency and report loss of authority. |
| `FAULT` | State, command, or solver contract is invalid. | Suppress new action and use the last valid safe command or estop policy. |

Important transition rules:

- `NOMINAL -> ASSIST`: measured risk and confidence pass hysteresis, or a
  benchmark oracle explicitly requests assistance.
- `ASSIST -> HOLD`: additional action gives no outcome benefit or consumes the
  certified braking reserve, while immediate braking remains unsafe.
- `ASSIST/HOLD -> BRAKE`: disturbance confidence is low enough and a predicted
  brake keeps capture, support, tilt, collision, task, and actuator margins
  safe.
- `BRAKE -> SETTLE`: measured arm momentum and rate are near zero and the
  delivered response agrees with the brake prediction.
- `SETTLE -> RESTORE`: balance metrics remain inside quiet thresholds for a
  minimum measured-time dwell.
- `RESTORE -> ASSIST`: any renewed risk immediately lowers posture priority and
  replans assistance.
- `RESTORE -> NOMINAL`: reference error is small, balance is quiet, and no
  downstream limit altered the command.

Timers may provide debounce and minimum dwell. They may not force braking or
restoration.

### `zmp/diagnostics.py`

Defines a versioned diagnostics schema independent of candidate or legacy
internal classes. The benchmark logger must consume this schema rather than
accessing `controller.actuator` fields.

Per-tick diagnostics include:

- Source sequence/time, receive time, sample age, measured `dt`, and validity.
- Backend, profile, schema version, output mode, and reference generation.
- Phase, phase transition reason, episode ID, and decision-variable joints.
- Arm availability and active manipulation constraints.
- Measured COM/capture, support margin, ZMP/CMP, tilt, angular velocity, arm
  momentum, and arm momentum rate.
- Disturbance estimate, direction, confidence, and observer latency.
- Predicted outcome trajectory and realized one-step outcome.
- Predicted versus measured response error and identified delay/gain.
- Raw optimized command, sent command, measured command tracking, and
  safety-layer applied command when available.
- Joint range, braking distance, collision distance, end-effector limits,
  effective torque margin, and active constraint identities.
- Brake-certificate validity, minimum predicted brake margin, and capability
  exhaustion reason.
- Solver status, objective levels, iteration count, solve time, deadline miss,
  warm-start source, and fallback use.
- Useful assist impulse, braking impulse, restoration impulse, and lower-body
  or contact contribution when measurable.

## Braking And Recovery Safety Contract

Every non-nominal action must satisfy two conditions before its first command
is sent:

1. The planned command trajectory satisfies all physical, task, and actuator
   constraints.
2. From every committed near-term state, a stored contingency can brake the
   upper-body momentum and enter a bounded settling state without violating the
   configured conservative balance and recovery margins.

The optimizer will append or separately solve a braking tail. A
`BrakeCertificate` records:

- State and reference generation for which it is valid.
- Validity horizon and model-confidence requirements.
- Brake controls and predicted outcome trajectory.
- Minimum position, velocity, acceleration, jerk, torque, collision,
  end-effector, task, support, capture, and tilt margins.
- Terminal arm momentum/rate bounds.
- Terminal settling-state bounds.

If the main solve misses its deadline, the controller shifts and executes the
previously certified trajectory only while its state-error tube remains valid.
Otherwise it enters `EXHAUSTED` or `FAULT` and follows the configured least-risk
policy. It must never improvise an unconditional posture return.

Physical braking feasibility and immediate balance-safe braking are distinct:
the controller may enter `HOLD` when it must preserve useful arm momentum, but
it must still reserve enough joint range and actuator authority for the stored
contingency. If that reserve cannot be preserved, the candidate action is
rejected before execution.

## Runtime Controller Flow

Each `control_step_reduced()` will perform:

1. Refresh measured state and Pinocchio kinematics.
2. Build and validate the source-timed `MeasuredState`.
3. Update measured balance outcomes and delivered-response diagnostics.
4. Update the disturbance estimate without adapting quiet references during an
   active or entering episode.
5. Obtain the persistent reference snapshot and current arm/task allocation.
6. Advance the phase manager from measured state and the previous certificate.
7. Build and solve the direct-outcome optimization request.
8. Validate the first command and braking certificate independently.
9. Send explicit `q_cmd`, `dq_cmd`, and `tau_cmd` through `LowCmdHandler` when
   output is enabled.
10. Record prediction, command, measured response, active constraints, and
    fallback status in one temporally consistent diagnostics object.

Observation and shadow optimization continue when reflex output is disabled.
Disabling output resets active execution but does not discard the persistent
nominal/task reference.

## Runtime Backend And Rollback Integration

The benchmark and example entry point need an explicit backend switch:

```text
candidate
legacy_update3
```

Planned configuration:

```yaml
zmp:
  backend: candidate
  enabled: true
  output_mode: active  # passive, shadow, or active
```

The runtime will select the class after loading configuration. It will not
import one `ZmpController` unconditionally at module import time.

Runtime variants will be explicit:

- `upper_fixed`: existing system-health baseline.
- `zmp_passive`: candidate stack with the same gains, gravity compensation,
  DDS path, and output path, but balance assistance disabled.
- `zmp_legacy`: accepted Update 3 backend and frozen legacy profile.
- `zmp_candidate`: new direct-outcome backend.

Every run will record logical variant, backend module/class, profile, output
mode, diagnostics schema version, config hash, revision, and dirty patch hash.
The accepted controller will not import candidate modules or share mutable
candidate state.

The common runtime controller protocol is:

```python
control_step_reduced()
reset_balance_reference()
set_reflex_output_enabled(enabled)
get_balance_diagnostics()
shutdown()
```

Candidate-only reference methods are additive. The benchmark logger will use
`get_balance_diagnostics()` for both backends through a legacy adapter.

## Configuration Plan

All active `balance_*.yaml` files will contain a non-null `zmp` mapping. The
loader will reject a non-mapping `zmp` value. Shared structure:

```yaml
zmp:
  backend: candidate
  enabled: false
  output_mode: active

  control:
    available_arms: [left, right]
    include_torso: false
    horizon_steps: 8
    brake_steps: 8
    deadline_ratio: 0.8

  reference:
    idle_posture: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    freeze_task_on_assist: true
    restore_ramp_time: 0.5

  observer:
    max_state_age: 0.05
    calibration_time: 2.0
    support_frame: ankle_midpoint
    require_contact_confidence: true

  outcome:
    model: centroidal_servo
    response_delay_steps: 1
    disturbance_decay: 0.9
    capture_weight: 1.0
    support_weight: 1.0
    tilt_weight: 1.0
    angular_velocity_weight: 1.0
    recovery_weight: 1.0

  constraints:
    position_margin: 0.05
    acceleration: 8.0
    jerk: 80.0
    effective_torque_ratio: 0.6
    collision_margin: 0.03
    end_effector_linear_speed: 1.25
    end_effector_angular_speed: 2.5
    brake_position_reserve: 0.08

  phases:
    enter_cycles: 2
    exit_cycles: 5
    settle_time: 0.3
    arm_momentum_tolerance: 0.02
    arm_momentum_rate_tolerance: 0.2

  solver:
    name: auto
    max_iterations: 20
    regularization: 0.001
    trust_region: 0.2

  diagnostics:
    schema_version: 2
    log_horizon: true
```

Exact numeric defaults will be established by static limit checks, safe pulse
identification, and shadow benchmark timing. They must not be copied from the
legacy momentum caps without evidence.

`balance_debug.yaml`, `balance_safety_full.yaml`, `balance_safety_split.yaml`,
and `balance_sport.yaml` will share the same schema. Differences will remain
limited to topics, publisher mode, gains, physical limits, and intentionally
documented environment-specific safety settings.

## Implementation Order

### Stage 0: Freeze Baseline And Repair Harness Contracts

- Add candidate/legacy backend selection and explicit benchmark variants.
- Add a legacy diagnostics adapter.
- Validate that `zmp` is a mapping and replace all null active sections.
- Record backend identity in every run.
- Require every process to remain alive through the evaluation window.
- Require complete simulation coverage before a trial can pass.
- Freeze the accepted Update 3 config and reproduce its existing guard cases.

Exit gate:

- Candidate, legacy, passive, and fixed variants select distinct intended
  processes and identities.
- Legacy behavior remains runnable without importing candidate internals.
- No incomplete run can be classified as passing.

### Stage 1: Types, References, Allocation, And State Validity

- Add dataclasses, diagnostics schema, reference manager, and control
  allocation.
- Add source-time/sample-age handling and filtered measured acceleration.
- Add arm momentum/rate, tilt/rate, support, and capture validity reporting.
- Implement idle zero reference and task-reference snapshot APIs.
- Add both-arm, left-only, right-only, and reserved-arm unit tests.

Exit gate:

- The persistent reference survives all phase transitions unchanged.
- Invalid or stale state prevents active optimization.
- Reserving one arm removes it from balance decisions while retaining its task
  constraints and command ownership.

### Stage 2: Outcome Model And Constraint Model In Shadow Mode

- Implement executable command rollout and effective PD torque prediction.
- Add current full-body momentum-rate prediction.
- Add capture/CMP, tilt, angular-velocity, and recovery predictions.
- Add all joint, command, end-effector, collision, task, and braking-reserve
  constraints.
- Run the optimizer in shadow mode while legacy or passive control drives the
  robot.

Exit gate:

- All shadow trajectories satisfy hard constraints before downstream clipping.
- Prediction fields are time-aligned with the next measured sample.
- Solver p99 time is below `0.8 * controller.dt` with zero uncontrolled
  deadline fall-through.

### Stage 3: Oracle-Triggered Active Realization

- Use simulation force onset and direction only as an experimental oracle.
- Enable candidate commands with conservative limits.
- Validate measured momentum rate, support response, tilt response, and command
  delay with safe basis pulses.
- Fit or tune the bounded local response model from held-out trials.

Exit gate:

- Predicted response sign is correct for positive and negative roll/pitch
  basis actions.
- Targeted predicted-to-measured outcome error is below `20%` in the validated
  operating region.
- Measured command delay and gain are repeatable within `10%`.
- No physical constraint violation or unexpected safety clipping occurs.

### Stage 4: Braking Certificate And Phase Execution

- Implement assist, hold, brake, settle, restore, exhausted, and fault paths.
- Store and shift the last valid contingency on solver deadline miss.
- Gate braking on predicted post-brake outcomes and measured disturbance state.
- Gate restoration on measured zero arm momentum/rate and quiet balance.
- Interrupt restoration immediately on renewed risk.

Exit gate:

- Every active command has a valid logged `BrakeCertificate`.
- No braking begins when the predicted post-brake capture/support margin is
  unsafe.
- No posture restoration begins before measured braking and settling gates.
- Restoration never reverses a still-useful balance response.

### Stage 5: Runtime Observer And Measured Feedback

- Replace the oracle with source-timed command-aware disturbance estimation.
- Use delivered response to update prediction residuals and confidence.
- Reject stale/repeated state and suppress adaptation during invalid contact.
- Compare oracle and runtime-observer trajectories from identical states.

Exit gate:

- Zero pre-force active commands in 20 no-force branches.
- Median activation latency is below `100 ms` and maximum below `150 ms`.
- Disturbance direction error is below `20 degrees` at activation.
- Runtime observation retains at least `90%` of oracle survival benefit.

### Stage 6: Manipulation-Aware Allocation

- Connect current `FrameTask` targets through `ReferenceManager` adapters.
- Add hard per-axis task constraints and bounded task slack.
- Test left-task/right-assist, right-task/left-assist, and partial redundancy.
- Keep task tracking active during assist and restore the assist arm to the same
  task or nominal generation.

Exit gate:

- Reserved-arm command fields are unchanged by the balance allocator.
- Hard task axes remain within configured bounds.
- Balance uses only declared redundancy and does not reset task references.
- One-arm mode satisfies the same physical and braking-certificate contract.

### Stage 7: Lower-Body Coordination

- Publish predicted arm reaction wrench, phase, confidence, and planned braking
  impulse.
- Add a lower-controller consumer or shared centroidal objective when its
  interface is available.
- Measure whether contact impulse reinforces or cancels arm action.

Exit gate:

- Coordinated control increases disturbance-opposing contact impulse.
- No accepted guard case regresses.

## Test Plan

### Unit Tests

- Configuration rejects null, non-mapping, unknown, or dimensionally invalid
  `zmp` fields.
- State estimator handles stale, repeated, nonfinite, and irregularly timed
  samples.
- Support-frame transforms and axis signs pass analytical fixtures.
- Reference generation persists across every phase transition.
- Task snapshot, update, clear, and arm-reservation behavior is deterministic.
- Allocation covers both-arm, each single-arm, partial task slack, torso, and no
  available DoF cases.
- Outcome rollout matches finite differences for command, momentum rate,
  end-effector motion, and effective torque.
- Every constraint activates at its expected boundary.
- Collision and task constraints reject an otherwise attractive balance action.
- The optimizer chooses outcome improvement rather than a configured momentum
  value.
- A candidate without a feasible brake tail is rejected.
- Phase transitions cover renewed disturbance during brake, settle, and
  restore.
- Solver failure, infeasibility, stale certificate, and deadline miss select the
  documented fallback.
- Diagnostics from candidate and legacy adapter satisfy one schema contract.

### Property And Trajectory Tests

- Random valid upper-body states never produce a command outside exact clip
  limits.
- Predicted joint trajectories remain inside position, velocity, acceleration,
  jerk, and effective-torque bounds.
- Each committed assist step preserves a feasible brake certificate.
- Restoration cost changes cannot alter hard safety feasibility.
- Removing one arm cannot produce nonzero commands on its indices.
- Replaying the same state/reference/config yields the same first command and
  phase decision within solver tolerance.

### Integration Tests

- Controller construction with `init=False`, control loop, reset, output gate,
  diagnostics, and shutdown.
- Passive and active candidate use identical gains, initialization, gravity
  compensation, DDS topics, and safety path.
- Benchmark process specs select the correct backend and profile.
- All processes are monitored after readiness.
- Logger accepts both diagnostics backends without internal-field inspection.
- Final safety command equals optimized command within tolerance in normal
  operation.

## Benchmark Methodology

### Harness Integrity

Before efficacy tests:

- Save complete MuJoCo state and controller state at the qualified branch
  point.
- Apply disturbances on an exact simulation step.
- Branch passive, legacy, and candidate from identical state.
- Verify bit-identical pre-force simulation arrays.
- Verify zero-step force-onset difference.
- Record complete source revisions, dirty patch hashes, configs, solver version,
  and random seeds.

### Controls-Equivalent Baselines

`upper_fixed` remains useful for system health, but promotion comparisons use
`zmp_passive` so initialization, gains, gravity compensation, command transport,
and safety clipping are identical to the candidate.

Compare:

- Candidate active versus candidate passive.
- Candidate active versus accepted legacy Update 3.
- Accepted legacy versus its controls-equivalent passive mode.
- `upper_fixed` as a secondary historical reference.

### Experiment Sequence

1. No-force standing and delayed-force causality.
2. Safe positive/negative roll and pitch basis pulses.
3. Oracle-triggered `0.15 s` disturbances.
4. Oracle-triggered `0.5 s` disturbances.
5. Existing `1.0 s` sustained-load stress test.
6. Equal-impulse, different-duration tests.
7. Deliberate braking-time and braking-rate sweeps.
8. Runtime observer repetition of oracle cases.
9. One-arm reserved manipulation fixtures.
10. Repeated targeted guards, followed by the full eight-direction sweep.

### Required Metrics

- Survival probability and confidence interval.
- Complete-window and process-health status.
- Peak and minimum capture/support/CMP margin.
- Peak torso tilt and disturbance-aligned angular velocity.
- COM velocity/displacement and recovery time.
- Measured arm momentum and momentum rate.
- Useful assist, braking, settling, and restoration impulse.
- External and contact impulse ledger residual.
- Joint, end-effector, collision, task, torque, and braking reserves.
- Raw, sent, safety-applied, and measured command response.
- Prediction error, trigger latency, solver latency, deadline misses, fallback
  count, and capability exhaustion.
- Final posture and task error relative to the persistent reference.

Metrics must be reported by phase and force window, not only as global maxima.

## Acceptance Gates

### Safety And Causality

- Zero pre-force active commands in 20 of 20 no-force trials.
- Zero post-fall commands.
- Zero physical, task, collision, actuator, or watchdog violations.
- Safety-layer clipping changes less than `1%` of command magnitude in at least
  `99%` of valid pre-fall samples and never masks a systematic violation.
- Every active command has a valid braking certificate.
- No unsafe brake or premature restore event is observed.

### Realization And Prediction

- Predicted response sign is correct for all positive/negative roll and pitch
  basis pulses.
- Measured momentum-rate and key outcome response reaches at least `70%` of the
  predicted feasible response after the identified delay.
- Normalized held-out outcome prediction error is below `20%` in the promoted
  operating region.
- Solver p99 latency is below `80%` of the control period.
- Deadline fallback never executes an uncertified command.

### Recovery And Reference Behavior

- Braking starts only with a valid predicted safe post-brake state.
- Measured arm momentum and rate settle before posture restoration.
- Renewed risk pauses or reverses restoration in one control tick.
- Final upper-body joint error is below `0.03 rad` per non-task joint after the
  recovery window.
- Manipulation task error stays within its declared hard bounds.

### Balance Efficacy

- Oracle candidate improves survival by at least 50 percentage points in at
  least two targeted candidate cases over ten identical-state branches.
- Runtime observation retains at least `90%` of the oracle survival gain.
- At least two directions improve by one `5 N` bracket with repeated evidence.
- No direction loses a previously passing controls-equivalent guard.
- Mean best-passing force improves by at least `5 N` over candidate passive.
- Capture/support margin or disturbance-aligned angular-velocity peak improves
  by at least `10%` in at least four directions.

If the oracle, identified-response controller cannot improve two directions,
stop observer and gain work and redesign available physical authority. If arm
spin-up helps but the contact impulse ledger cancels it, stop and implement
lower-body coordination before further tuning.

## Major Risks And Mitigations

| Risk | Consequence | Mitigation |
| --- | --- | --- |
| Incomplete base/contact state. | Capture, ZMP, and CMP prediction may be wrong. | Add source-timed base state in simulation, validity/confidence flags, contact gating, and do not enable invalid objectives on hardware. |
| Position-servo model mismatch. | Predicted command authority and torque may be wrong. | Optimize explicit interface commands, identify gain/delay, compare measured response every tick, and keep bounded adaptation. |
| Finite arm stroke and sustained force. | No safe brake may exist after range exhaustion. | Enforce braking reserve on every action, report `EXHAUSTED`, test short and sustained disturbances separately, and coordinate lower body. |
| Lower controller cancels arm action. | Internal motion redistributes effort without expanding stability. | Add impulse ledger, reaction-wrench telemetry, and a staged coordination interface. |
| Solver deadline or infeasibility. | A stale or unsafe command may be emitted. | Warm start, deterministic deadline, stored certified fallback, state-error validity tube, and fault policy. |
| Collision linearization error. | A horizon can be numerically feasible but physically unsafe. | Conservative distance margins, trust regions, nonlinear rollout validation, and independent first-command check. |
| Task interface lacks priorities and slack. | Balance may disturb manipulation or become infeasible. | Introduce explicit per-axis frame constraints and availability; never infer task freedom from arm name alone. |
| Model and simulator joint-limit mismatch. | Planner, publisher, and simulator disagree. | Use the intersection of all bounds and log bound provenance before enabling output. |
| Online adaptation learns disturbance instead of actuation. | Response model sign or gain can drift unsafely. | Adapt only in validated windows with excitation/confidence gates and hard parameter bounds; retain static fallback. |
| Benchmark timing and statistical noise. | A one-bracket gain may be false. | Identical-state branching, step-synchronized force, repeated trials, complete-window checks, and confidence intervals. |
| Legacy and candidate identity drift. | Rollback and comparison become untrustworthy. | Explicit backend/profile metadata, frozen legacy config, separate imports, and contract tests. |

## Initial Deliverable Boundary

The first code iteration should deliver the complete module contracts,
persistent reference behavior, configurable arm availability, measured-state
diagnostics, direct executable-command optimization, physical constraints,
phase separation, braking certification, runtime backend switch, and benchmark
logging. It should begin in shadow and oracle modes before active runtime
observer promotion.

The implementation must not claim validated capture/ZMP control on the real
robot until floating-base/contact state and closed-loop response identification
pass their gates. This is an explicit safety boundary, not a reason to retain
the legacy momentum-target formulation.
