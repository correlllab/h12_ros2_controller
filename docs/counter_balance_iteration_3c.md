# Counter-Balance Iteration 3C

## Purpose

Iteration 3C creates a reusable high-authority Crocoddyl controller by closely
reproducing the useful behavior of Iteration 2 `momentum_wide` before attempting
any improvement.

It has two goals:

1. Reproduce the command direction, early authority, limits, and fall-rescue
   behavior of Iteration 2 `momentum_wide`.
2. Retain the parity controller as a complete comparison variant on matched FAME and ALMI hard groups.

Iteration 3 B0 remains frozen and independently runnable.

## Scope

Iteration 3C is deliberately narrow:

- One-step Crocoddyl optimal control.
- Counter-joint velocity as the decision variable.
- A conceptually equivalent Iteration 2 objective, gains, normalization, bounds,
  excursion, references, activation, and publication behavior.
- Existing moving-arm pass-through, collision, safety, and estop contracts.

Iteration 3C does not add:

- A longer horizon.
- Acceleration-level control.
- A base-response model.
- Policy-specific activation.
- ALMI or ordinary-case regression tuning.
- New objective terms.

## Crocoddyl Formulation

Use the four active counter joints:

\[
x_k=q_{c,k}\in\mathbb{R}^4,
\qquad
u_k=\dot q_{c,k}\in\mathbb{R}^4.
\]

The one-step dynamics are:

\[
q_{c,k+1}=q_{c,k}+\Delta t u_k.
\]

The running cost preserves the Iteration 2 residual meaning and relative
priorities:

- CoM-velocity cancellation.
- Angular-momentum cancellation and gyro feedback.
- Counter-posture return.
- Velocity damping.

Use the same effective velocity and position/excursion bounds as `reactive_counter_balance_momentum_wide`.

Use a separate controller and runtime variant, for example:

```python
class CounterDDPVelocityController(...):
    ...
```

```text
counter_ddp_velocity_wide
```

Do not call the Iteration 2 least-squares solver as the active command path. It
may be used as a behavioral oracle in tests and diagnostics.

The Crocoddyl formulation does not need identical internal cost values or solver
iterations. It must produce closely matching counter-arm command direction,
magnitude, bound activity, and intervention timing.

The manipulation arm follows the existing Iteration 2 contract. Iteration 3C
does not change its position, velocity, torque/gravity-compensation, tracking, or
publication semantics. Future iterations should preserve the same manipulation
arm behavior rather than repeatedly redesigning that path.

## Parity Stage

### Command Parity

Evaluate recorded valid snapshots spanning:

- Left and right moving-arm ownership.
- Standing and moving manipulation.
- Low and high gyro response.
- Tight and active velocity bounds.
- Excursion-near states.
- Collision-free and collision-backtracked commands.

For every snapshot, compare:

- Residual direction and relative objective contributions.
- Unbounded command direction.
- Bounded counter velocity.
- Integrated counter position command.
- Activation/pass-through decision and any activation scaling.
- Moving-arm position, velocity, and torque pass-through.
- Collision/backtracking and fallback status.

Parity gates:

- Matching counter-command sign and similar magnitude on representative states.
- The same joints become bound-active in high-authority cases.
- Similar early counter displacement and momentum buildup on FAME rescue cases.
- Matching activation, fallback, and collision/backtracking decisions.
- Moving-arm command remains unchanged to existing pass-through tolerance.

Systematic command-direction, authority, bound, or timing disagreement must be
traced before behavioral testing.

Do not tune Iteration 3C to improve ALMI or ordinary targets during parity work.
Adjust implementation details only when they prevent reproduction of Iteration 2
authority.

### Behavioral Parity

Run five paired Iteration 2 and Iteration 3C repetitions on the historical FAME
boundary rescues:

- `right_fast_fall_search_06_scale_74`.
- `right_fast_fall_search_09_scale_78`.
- `right_fast_fall_search_11_scale_78`.

Include five repetitions of these fixed ALMI guards:

- `right_fast_fall_search_06_scale_74`.
- `right_fast_fall_search_09_scale_78`.
- `right_fast_fall_search_11_scale_78`.

Iteration 3C passes behavioral parity when it reproduces Iteration 2 majority
outcome severity and similar counter-arm command traces on the rescue panel.

Compare:

- Counter velocity.
- Counter displacement.
- Counter-arm momentum.
- Intervention timing relative to the manipulation-arm disturbance.

Do not tune parameters after inspecting these outcomes. Fix only implementation or parity defects.

## Runnable High-Authority Controller

After parity passes, retain Iteration 3C as a normal benchmark controller:

- It initializes and runs without the Iteration 2 solver.
- It supports direct and inherited frame-control paths.
- It emits normal controller diagnostics and timing summaries.
- It supports shadow, single-trial, hard-group, and full-panel sweeps.
- It preserves B0 and Iteration 2 as separate variants.

Iteration 3C is a high-authority baseline, not a promoted cross-policy controller.

Known FAME/ALMI regressions are characterization results, not reasons to modify the parity controller.

## Evaluation

Freeze the Iteration 3C code and parity configuration before evaluation.

### Hard-Group Comparison

Run the three established hard groups on:

1. `frame_task`.
2. Iteration 2 `reactive_counter_balance_momentum_wide`.
3. Iteration 3 B0 `counter_ddp`.
4. Iteration 3C `counter_ddp_velocity_wide`.

Use the same target catalogs, profiles, model, lifecycle, safety settings, and classifier revision.

Use three matched operationally complete repetitions per target/controller cell.

Where possible, use contemporaneous paired runs or identical saved states.

Define each target outcome by the median severity across its three complete
repetitions. Report attempted, complete, infrastructure, and runtime-failure
counts separately. Incomplete infrastructure trials may be rerun once; do not
hide repeated controller/runtime failure through unlimited retries.

Report:

| Policy | Controller | Stable | Drift | Stumble | Fall | Survived |
|---|---|---:|---:|---:|---:|---:|
| FAME | Frame task | | | 0 | | |
| FAME | Iteration 2 momentum wide | | | 0 | | |
| FAME | Iteration 3 B0 | | | 0 | | |
| FAME | Iteration 3C | | | 0 | | |
| ALMI | Frame task | | | | | |
| ALMI | Iteration 2 momentum wide | | | | | |
| ALMI | Iteration 3 B0 | | | | | |
| ALMI | Iteration 3C | | | | | |

Report boundary, exploration, and hard groups individually, followed by the
pooled hard-group total.

Also report:

- Complete paired severity transitions against frame task.
- Iteration 3C transitions relative to Iteration 2 and B0.
- Per-case survival changes.
- Moving-arm tracking/pass-through error.
- Counter velocity and excursion.
- Velocity clipping and collision backtracking.
- Solver and total-controller p50, p95, p99, and maximum timing.
- Runtime, infrastructure, collision, and estop failures.

The hard-group comparison is the primary behavioral evaluation of Iteration 3C.

### Normal-Target Regression Check

After the hard-group evaluation, run Iteration 3C once over the standard 100-target FAME panel and once over the standard 100-target ALMI panel.

This is a regression check, not a repeated statistical comparison.

Its purpose is to detect:

- Unexpected ordinary-case regressions.
- New severe outcomes.
- Manipulation tracking problems.
- Excessive counter-arm motion.
- Collision/backtracking problems.
- Runtime or infrastructure failures.

Do not rerun Frame task, Iteration 2, and B0 solely to create a new four-controller
100-target comparison. Existing frozen results may be used as context.

Do not require three repetitions for every normal target.

Repeat an individual target only when the single sweep exposes a changed, ambiguous, or severe outcome that requires confirmation or diagnosis.

Summarize the 100-target Iteration 3C regression sweep as:

| Policy | Stable | Drift | Stumble | Fall | Survived |
|---|---:|---:|---:|---:|---:|
| FAME | | | 0 | | |
| ALMI | | | | | |

Freeze the classifier revision and policy capability settings for every run:
FAME uses `stepping_capable=false`; ALMI uses `stepping_capable=true`.

## Real-Compatible Observation Contract

Maintain the real-compatible observation infrastructure established during Iteration 4.

Controller-facing observations may use only:

- IMU.
- Joint position.
- Joint velocity.
- Joint torque.
- Quantities derived from these using Pinocchio.

Do not introduce simulator-only contact force, ground-truth contact state, external wrench, or exact simulator base state.

Iteration 3C may log the Iteration 4 proprioceptive observer for diagnostics, but
it must not alter parity commands or add unvalidated Iteration 4 safety modes.

## Video Evidence

Generate videos for:

- Both sides of every changed paired hard-group outcome.
- Every ALMI stumble in the hard-group comparison.
- Every hard-group fall.
- Any severe or unexpected regression discovered in the 100-target regression sweep.

Use the common `changed_outcomes.html` review page with Improvements, Regressions, Stumble Replays, and Fall Replays sections.

## Characterization Decision

Iteration 3C evaluation is descriptive rather than a tuning loop.

The final report must state:

- Whether command and behavioral parity were achieved.
- Whether Iteration 2 behavioral authority was reproduced.
- Whether the historical FAME fall rescues were recovered.
- The FAME benefit/regression profile across the three hard groups.
- The ALMI benefit/regression profile across the three hard groups.
- Whether the normal 100-target regression sweep exposes unacceptable new behavior.
- Whether Crocoddyl overhead remains inside the real-time budget.
- Whether Iteration 3C is a useful high-authority option for later policy-blind selection.

Do not modify Iteration 3C to repair ALMI or ordinary-case regressions after the evaluation.

Any policy-blind selection or base-response logic belongs in a separate controller/iteration and may choose between exact pass-through, B0, and the frozen Iteration 3C authority baseline.
