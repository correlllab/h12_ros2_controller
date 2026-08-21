# Counter-Balance Controller

## Purpose

`ReactiveCounterBalanceController` sends the manipulation trajectory to the
moving arm and uses the counter arm to reduce balance disturbance. It is evaluated in
double-support handless simulation with an unchanged lower-body policy and torso
command.

This document records the controller contract, evaluation contract, durable
lessons, and next experiment. Interactive dashboards, replay videos, raw logs,
and benchmark-specific result tables belong to the benchmark repository's run
outputs, not this controller repository.

## Controller Contract

- `moving_arm` is selected at initialization and receives the manipulation frame
  task.
- Only counter-arm shoulder pitch, shoulder roll,
  shoulder yaw, and elbow are allowed to move reactively.
- Counter-arm wrists remain held to their captured posture.
- The controller uses live robot state, CoM, support geometry, gyro feedback,
  and centroidal momentum diagnostics to choose a bounded counter-arm response.
- Counter-arm motion is bounded by velocity, joint limits, collision checks,
  and a captured-posture excursion limit.
- After moving-arm motion completes, reactive output fades to posture hold.
- The moving arm uses normal PD tracking during motion. The benchmark runtime
  then enters the existing bounded steady-state integral hold to reduce static
  target error without changing controller gains.

## Safety Contract

- The counter arm may not command outside its configured position, velocity, or
  torque limits.
- A maximum excursion is measured relative to the captured counter-arm posture.
- Collision rejection and estop retain precedence over reactive commands.
- Invalid support geometry disables reactive motion and holds the counter arm.
- Falls remain a primary outcome regardless of other metrics.

## Evaluation Contract

### Balance Outcome

Balance is measured by base orientation drift rather than final arm tracking
error.

1. Detect the arm-motion release event.
2. Identify the final contiguous `standing_ready` interval before release.
3. Use the final one-second settled subset of that interval as the roll/pitch
   reference.
4. Measure the maximum roll/pitch displacement from that reference through the
   arm motion and hold window.

The default drift threshold is `0.1 rad` and is configured by
`base_drift_threshold`.

- Green: complete run, no fall, valid settled reference, and drift within the
  threshold.
- Orange: complete run, no fall, and drift above the threshold.
- Red: fall.
- Infrastructure: incomplete run or unavailable drift measurement.

### Tracking Diagnostic

Tracking remains useful but is not the balance objective.

- Blue: final configuration error is within the configured tracking threshold.
- Magenta: tracking did not converge to that threshold.
- Grey: infrastructure failure.

Balance and tracking reports use the same candidate, trajectory, rank, and
comparison layouts. They answer different questions and must not be conflated.

### Supporting Diagnostics

The benchmark records CoM and contact-ZMP support margins, arm state, solver
status, command clipping, and collision/estop status. These explain outcomes but
do not currently decide green versus orange.

## Current Controller Variants

The benchmark compares these configurations:

- `frame_task`: moving-arm baseline with no reactive counter-arm response.
- `reactive_counter_balance`: conservative reactive reference.
- `reactive_counter_balance_gain`: bounded gain candidate with lower posture
  weight, stronger CoM response, lower gyro/posture gain, expanded velocity, and
  explicit excursion limits.
- `reactive_counter_balance_displacement`: a distinct bounded displacement
  candidate.
- `reactive_counter_balance_aggressive_tight`, `_nominal`, and `_wide`: one
  aggressive response evaluated under three counter-arm excursion limits.

`reactive_counter_balance_gain` is retained as a candidate, not promoted as a
universal default. Promotion requires a completed matched evaluation with no
material cross-backend regression.

## Historical Design Lessons

Earlier exploratory sweeps informed the design but are not current evidence.

- Unbounded aggressive counter-arm motion can create falls and operational
  failures. Excursion limits are therefore mandatory.
- CoM and ZMP feedback can improve individual directions while regressing other
  directions. Margin changes alone do not establish controller dominance.
- Far cross-body targets exposed a distinction between arm tracking residual and
  base stability. Tracking is consequently reported separately from the balance
  outcome.
- A bounded steady-state integral hold is preferable to gain escalation for
  reducing residual static tracking error.
- Direction-dependent response suggests that a learned or identified arm
  reaction map may be more useful than further scalar gain tuning.

## Evidence Policy

Current quantitative claims require a completed, matched FAME and ALMI sweep
using the same catalog, target runtime, base-drift classifier, and replay
pipeline. Partial sweeps, retired artifacts, and exploratory experiments may
inform hypotheses but do not establish comparative performance.

The completed matched all-candidate findings and next tuning gate are summarized
in [counter_balance_analysis.md](counter_balance_analysis.md).

## Next Tuning Cycle

Use the completed all-candidate sweep to develop measured-response gating:

1. Start from the tight aggressive excursion envelope.
2. Diagnose ALMI left-upward-arc regressions against matched frame trials.
3. Gate counter-arm response from observed base motion rather than backend name.
4. Preserve FAME recovery targets and enforce no-regression ALMI gates.
5. Run the compact tuning set before another full sweep.

## Deferred Work

- Contact-aware or learned direction-dependent counter-arm feedforward.
- Dynamic support polygons, single support, stepping, and payload handling.
- Runtime arm-role switching and ROS action-server integration.
- Whole-body or torso optimization.
- Hardware validation.
