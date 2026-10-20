# Counter-Balance Iteration 5

## Verified-Response Few-Step Crocoddyl MPC

## Status and Hypothesis

Iteration 5 is implemented and frozen as an unpromoted experimental controller.
B0 and frozen Iteration 3C `counter_ddp_velocity_wide` remain independently
runnable operational baselines. Iteration 4C remains diagnostic evidence.

The hypothesis is:

> Frozen 3C provides a proven nominal counter-balancing action. Crocoddyl can use
> verified causal response gradients to optimize bounded residual corrections
> that know when to continue, brake, or reverse.

The architecture is:

\[
\boxed{
\text{pure 3C nominal planner}
+
\text{delay-aware verified residual-response model}
+
\text{H2 Crocoddyl residual MPC}
+
\text{single shared safety/publication path}
}
\]

Start at velocity-level H2. Progress to H3 and then H5 only when H2 exposes a
specific verified horizon limitation and behavior and timing justify expansion.

## 1. Frozen Evidence

Iteration 5 must preserve these conclusions:

- Frozen 3C's immediate manipulation feedforward produces the repeated FAME
  rescues `right_fast_fall_search_09_scale_78` and
  `right_fast_fall_search_11_scale_78`.
- Iteration 4A's common full-state model failed roll-rate and lateral
  support-relative CoM-velocity gates. Do not rebuild it as a prerequisite.
- Iteration 4B preview-only scaling lost rescue `09`. Measured response is
  necessary, but scalar combined authority did not improve the ordinary tradeoff.
- Iteration 4B H3-0 repeated 3C costs without verified action-conditioned
  response or a useful terminal objective. It was behaviorally worse and too
  slow.
- Iteration 4C preserved both rescues and timing margin with decoupled feedback,
  but local tilt/rate rules could not select braking or reversal consistently.

Iteration 5 identifies only the incremental physical effect of a residual action
around 3C. It does not fit the complete humanoid response or tune another
authority scheduler.

## 2. Runtime Contract

Iteration 5 remains:

- Velocity-level and receding-horizon.
- Policy-blind and target-blind.
- Limited to the existing four proximal counter-arm joints.
- Compatible with IMU, measured joint position/velocity/torque, and justified
  Pinocchio-derived quantities.
- Identical to 3C for manipulation-arm ownership and publication.
- Subject to frozen joint, excursion, collision, estop, and atomic publication
  safeguards.

It must not use policy identity, target identity, historical outcome labels,
simulator contact truth, external-force truth, or exact simulator base state.

Preserve the validated Iteration 4A/4C observation safeguards:

- One coherent immutable state snapshot per control execution.
- Valid tick ordering and sequence metadata.
- Bounded monotonic sample age.
- Finite synchronized IMU and joint measurements.
- The existing real-compatible Pinocchio path.

Do not simplify these checks away or build new freshness infrastructure unless an
observed failure requires it.

## 3. Pure Frozen-3C Nominal Planner

The command is:

\[
u^{candidate}_k=u^{3C}_{nominal,k}+\delta u_k.
\]

The algebraic command above is the requested four-joint counter velocity before
final collision backtracking and publication. At zero residual, the complete
published position, velocity, and torque behavior must match frozen 3C.

The existing `control_configuration_step()` cannot be called to obtain the
nominal because it backtracks, mutates command state, and publishes. Refactor its
mathematical command generation into a non-publishing helper, for example:

```text
plan_frozen_3c_velocity(explicit_state, model_terms, references, moving_command)
    -> Frozen3CNominalPlan
```

`Frozen3CNominalPlan` contains at least:

- Requested four-joint counter velocity.
- Counter velocity and excursion bounds.
- CoM, momentum, and posture targets/residual terms.
- Lifecycle and activation scales.
- Solver status, KKT, and timing diagnostics.

The helper receives explicit prepared inputs and has no publication, collision
backtracking, reference-capture, command-state, or final safety side effects.
Preallocated solver workspaces may update internal numeric buffers, but no robot
or controller command state may be committed.

Use one shared execution path:

1. Acquire and validate the atomic state snapshot.
2. Update references/lifecycle exactly as frozen 3C does.
3. Compute Pinocchio terms once.
4. Call the pure 3C nominal planner.
5. Select `delta u = 0` for 3C or solve residual H2 for Iteration 5.
6. Form `u_candidate = u_3C_nominal + delta u`.
7. Run existing backtracking, safety validation, command-state update, and atomic
   publication exactly once.

Iteration 3C is behavior- and configuration-frozen, but its source may be
refactored to use this helper. Preserve the complete pre-refactor 3C source and
implementation hash as the parity oracle throughout the extraction. First run
the new helper and shared finalization path against that oracle on retained
recorded states without changing the normal 3C runtime route. Only after all
recorded-state and published-command parity gates pass may normal 3C be routed
through the shared helper. Its configurations and observable behavior remain
unchanged.

Verify before any MPC work:

- Nominal requested velocity and objective terms match the frozen implementation.
- Backtrack scale, candidate position, applied velocity, and status match.
- Final manipulation/counter position, velocity, and torque commands match within
  the established parity tolerance.
- No nominal planning call publishes or mutates final command state.

## 4. Offline Residual Identification

Saved-state matched branching is an offline simulation identification and
validation tool, never a runtime dependency.

For a candidate saved state, compare:

1. Frozen 3C.
2. `3C + delta u` for one or more bounded correction ticks.
3. Optional `3C - delta u` for central differences.

Measure causal response differences:

\[
\Delta\omega_{k+j}
=
\omega^{3C+\delta u}_{k+j}
-
\omega^{3C}_{k+j}.
\]

A branch set is causal-matched only when repeated zero-correction branches agree
within a preregistered short-horizon tolerance. Restore lower-policy/controller
history, hidden state, delays, or command queues only as needed to pass that
empirical equality gate. Discard branch sets that do not pass. Do not turn exact
MuJoCo replay into an unrelated infrastructure project.

Randomized corrections, bounded pulses, and approximately matched states form a
separate statistical identification/validation dataset. Do not treat their
differences as direct counterfactual subtraction.

Use basis corrections, symmetric signs where practical, and at least two small
amplitudes across rising disturbance, near peak, early return, and pre-step
response. Only real-compatible features may enter fitted models. Simulator truth
may audit equality or explain failures, never enter runtime control.

## 5. Compact Runtime State

The initial physical response state is:

\[
x_k=[e_{\theta,k},\ \omega_k,\ q_{c,k},\ \dot q_{c,k}],
\]

with measured support validity as context. Known manipulation momentum/rate and
the frozen 3C command provide short preview.

Use structured integration:

\[
e_{\theta,k+1}
=
e_{\theta,k}
+\frac{\Delta t}{2}(\omega_k+\omega_{k+1}),
\]

\[
q_{c,k+1}
=
q_{c,k}
+\frac{\Delta t}{2}(\dot q_{c,k}+\dot q_{c,k+1}).
\]

Do not require support-relative lateral CoM velocity, a full contact state, or a
learned viability state for H2.

## 6. Minimum Verified Models

### 6.1 U5: Residual Command Realization

U5 maps requested `delta u` to realized counter velocity and momentum response.
Identify:

- Command delay and pending action carryover.
- Velocity gain/lag and position-dependent effectiveness.
- Counter-arm momentum response from measured state and Pinocchio.
- Clipping, backtracking, excursion, and weak directions.

U5 passes when zero correction is identical to 3C, response sign is correct,
delay is predicted within one tick, useful gain excludes zero inside the trusted
region, and one/two-step realization beats command-equals-realization and
zero-response baselines.

Matched simulation data may identify U5 densely. Bounded active pulses using the
same real-compatible observations must confirm sign, delay, and useful gain
before active MPC relies on it.

### 6.2 R5: Delay-Aware Incremental Response

R5 predicts correction-induced angular-rate response, not complete base motion.
U5 first maps requested residual velocity to realized counter velocity and
counter-arm momentum, including delay and carryover. R5 then maps that realized
residual action/momentum to incremental angular rate with a short FIR model:

\[
\Delta\omega_{k+1}=G_{0,k}\Delta h^c_k,
\]

\[
\Delta\omega_{k+2}
=
G_{1,k}\Delta h^c_k
+G_{0,k}\Delta h^c_{k+1}.
\]

Evaluate the local `G0,k` and `G1,k` from the current measured state/support
context and hold them fixed for one H2 solve. Reidentify or interpolate them at
the next control tick. This avoids requiring unverified derivatives of a
state-dependent gain model. Add such derivatives later only if an ablation
demonstrates that fixed local gains limit action ranking.

U5, not R5, owns requested-command delay, velocity realization, and the mapping
to `Delta h_c`. `G0,k` and `G1,k` contain only the uncertain short angular-rate
response to realized residual momentum. `G0,k` may be zero when realized action
cannot affect the first predicted sample. Known carryover from previously
published residuals must enter as fixed context.

Obtain incremental counter position by integrating U5's realized counter
velocity. Obtain incremental tilt by integrating R5's incremental angular rate
with the structured equations in Section 5. Do not fit `G0/G1` terms for tilt or
counter position.

For Crocoddyl's Markov interface, represent FIR memory with the smallest exact
augmented state containing pending residual action/effect. The queue shift is
structured; only the uncertain response gains are identified. Use an alternative
augmented formulation only if it is simpler and produces identical gradients.

Do not add an autonomous `A` matrix unless state-perturbation experiments
explicitly identify it. Multi-step residual pulses should identify U5 and the
R5 `G0/G1` terms separately.

### 6.3 N5: Minimal 3C Nominal-Phase Predictor

Matched subtraction identifies residual effects, but the terminal cost still
depends on absolute future tilt/rate. N5 supplies one common short 3C nominal
trajectory for all candidate corrections.

Use current tilt/rate, recent measured angular evolution, known manipulation
preview, and the frozen 3C nominal sequence. Prefer structured extrapolation and
fit only terms required to preserve local phase.

Over H2, N5 must predict with calibrated uncertainty:

- Tilt and angular-rate sign.
- Divergence versus recovery.
- Peak ordering.
- Zero-crossing timing within one tick when present.

Global humanoid-state accuracy is not required. Failure of unrelated absolute
state metrics does not reject R5 when residual gradients and action ranking pass,
but an invalid N5 phase blocks MPC publication for that tick.

Predicted candidates are:

\[
x^{candidate}_{k+j}
=
\bar{x}^{3C}_{k+j}
+
\Delta x_{k+j}.
\]

### 6.4 Reproducible Action-Ranking Gate

Define distinguishability before evaluation:

1. Estimate branch/pulse noise from repeated zero and repeated correction trials.
2. Define a weighted physical response norm over tilt/rate change.
3. Set a minimum physical effect threshold before held-out evaluation.
4. Require the confidence interval for a candidate-pair difference to exceed
   both the noise bound and minimum effect threshold.
5. Evaluate ranking accuracy only on distinguishable comparisons.

Use complete family-grouped held-out splits. Report coverage: the fraction of
all comparisons that are distinguishable. R5 passes only if roll/pitch gradient
signs are correct and continue/brake/reverse ranking is at least `90%` on that
fixed distinguishable set. Do not change the threshold after viewing rankings.

## 7. H2 Crocoddyl Residual MPC

Use a fixed-size discrete Crocoddyl `ActionModelAbstract` with two running knots
and one terminal knot:

\[
\delta U=[\delta u_0,\delta u_1].
\]

`calc()` combines N5 nominal state with delay-aware U5 realization, R5 FIR
angular-rate response, and structured tilt/position integration. `calcDiff()`
chains the U5 realization Jacobians with fixed local `G0,k/G1,k` response
gradients and exact queue/integration derivatives. Check analytic derivatives
against finite differences throughout the trusted region. Wrong-sign or
unsupported gradients invalidate the MPC result.

Initial running costs contain only:

- Predicted tilt, angular-rate, and positive-divergence reduction.
- Residual magnitude and change.
- Counter velocity, acceleration, excursion, and return reserve.
- U5/R5 trust-region and uncertainty penalties.
- Conservative support-context penalties.

Use an interpretable terminal objective:

\[
\ell_N
=
w_\theta\|e_{\theta,N}\|^2
+w_\omega\|\omega_N\|^2
+w_d\|\max(e_{\theta,N}\odot\omega_N,0)\|^2
+w_r\|q_{c,N}-q_{c,ref}\|^2.
\]

Do not add learned terminal viability initially.

Constrain the combined command by joint/velocity limits, per-step acceleration,
frozen excursion, first-action collision/safety validity, manipulation ownership,
and the U5/R5 state/action trust region.

## 8. Support and ALMI Scope

Use existing proprioceptive support-validity and step-onset signals as context.
Do not block H2 on a full contact or foot-placement model.

The initial ALMI goal is pre-step stumble prevention: keep a trajectory that
would otherwise initiate a protective step inside standing support.

- Valid standing support: allow residual optimization.
- Incipient support motion: tighten residual and excursion bounds.
- Moving/uncertain support or invalid MPC: set `delta u = 0` and return to the
  frozen 3C/current shared safety path.
- Recovered support: remain with the baseline initially.

Iteration 5 introduces no new braking/reconnect or post-step controller until
that behavior is separately validated. Reduced torso motion after step onset is
diagnostic evidence, not a stumble-to-standing claim.

## 9. Implementation Workflow

1. Extract and verify the pure 3C nominal helper.
2. Revalidate the existing real-compatible observation path.
3. Identify U5 delay and residual command realization.
4. Identify delay-aware `G0/G1` gradients with matched branches and/or bounded
   residual pulses.
5. Validate the minimal N5 nominal-phase predictor.
6. Implement fixed-size H2 Crocoddyl `calc()/calcDiff()` and derivative tests.
7. Run H2 in shadow while publishing frozen 3C.
8. Activate a small residual trust region on low-risk ordinary cases.
9. Expand to interleaved FAME rescue and ALMI stumble-prevention trials only
   after prediction, timing, safety, and ordinary-case gates pass.
10. Consider H3/H5 or one additional state only when H2 exposes a specific
    verified limitation.

Change one model, cost, bound, signal, or horizon mechanism at a time.

## 10. H3/H5 and State Expansion

Increase the horizon only when H2 repeatedly chooses the wrong phase because a
verified braking point or zero crossing lies beyond two ticks:

1. H2 with the minimum model.
2. H3 with identical model, costs, and constraints.
3. H5 only if H3 still misses an event that H5 predicts correctly.

Do not change horizon and model content together. Return to the shorter horizon
if behavior does not improve repeatedly or timing margin is lost.

Add CoM, capture point, ZMP, support margin, richer support transition, or a
learned terminal model only when active H2 evidence identifies that specific
missing state. Every extension must improve held-out prediction and active
behavior in a separate ablation.

## 11. Crocoddyl and Timing

Crocoddyl is the default few-step solver. Avoid H3-0's repeated computation:

- Preallocate fixed state, action, residual, cost, and solver data.
- Update numeric terms in place.
- Reuse one atomic snapshot, Pinocchio update, and manipulation preview per tick.
- Warm-start from the shifted previous residual sequence.
- Start with one BoxFDDP real-time iteration.
- Apply BoxQP polishing only when required by the KKT gate.
- Log one compact record per solve, not per internal knot or logger tick.
- Reject late results atomically.

Shadow total-controller p99 should be below `10 ms`. Active total-controller p99
must be below `15 ms`, preferably with substantial margin.

## 12. Experimental Ladder

Use serial, headless experiments. Before each batch, clear stale processes and
check CPU, GPU, memory, swap, and disk health. Randomize or interleave Frame, B0,
3C, and Iteration 5 in the same stable machine session.

Required FAME cells:

- `right_fast_fall_search_09_scale_78`: existing repeated 3C rescue.
- `right_fast_fall_search_11_scale_78`: existing repeated 3C rescue.
- `right_fast_fall_search_06_scale_74`: major surpass-previous-iterations
  challenge. Iteration 2 historically rescued it, but later 3C did not reproduce
  that recovery.
- One ordinary 3C regression and one nonduplicate ordinary geometry.

Any Iteration 5 rescue claim for `06` requires contemporaneous/interleaved Frame,
3C, and Iteration 5 trials in the same stable machine session.

Required ALMI cells:

- Right target `11` as the ordinary guard.
- Manual-grasp-minus and one second repeated stumble family for prevention.

Use three complete repetitions for screening and five for every rescue,
regression, stochastic, or changed cell before promotion. Report infrastructure
failures separately.

Required traces include 3C nominal and residual actions, realized response,
N5/R5 predicted versus measured tilt/rate, FIR gradients and uncertainty, action
ranking, support onset, foot motion, excursion, backtracking, tracking error, KKT,
and complete timing.

## 13. Success Hierarchy

Required:

- Retain every contemporaneously verified 3C FAME rescue.
- Introduce no new severe regression.
- Preserve manipulation tracking, safety, and real-time operation.

Strong success:

- Recover `right_fast_fall_search_06_scale_74` or another repeated FAME fall
  that contemporaneous 3C does not rescue.

Major cross-policy success:

- Convert at least one repeated ALMI stumble family to standing through pre-step
  prevention.

Ultimate target:

- Combine all existing FAME fall rescues, B0-or-better ordinary behavior, and
  repeatable stumble prevention in one policy-blind, real-compatible controller.

Stop and retain B0/3C if U5/R5 cannot predict residual sign reliably, N5 cannot
identify H2 phase, H2 loses an existing rescue, ordinary behavior does not
improve, timing exceeds the gate, or improvement requires policy, target, or
simulator-truth gates.

## 14. Initial Deliverable

The first implementation contains:

- A pure frozen-3C nominal helper plus a single shared finalization path.
- Recorded-state parity tests proving unchanged standalone 3C behavior.
- Source-bound U5, R5, and N5 datasets and validation reports.
- A delay-aware H2 Crocoddyl model with verified `calc()/calcDiff()`.
- Bounded residual trust regions and existing safety checks.
- Exact residual-zero fallback on moving/uncertain support or MPC invalidity.
- One H2 shadow panel followed by one low-risk active panel.

This minimum implementation tests the Iteration 5 hypothesis directly. No large
estimator, full-state model, simulator runtime, or learned terminal prerequisite
is permitted without specific H2 evidence.

## 15. Execution Record

### 15.1 Verified Model Package

The final model uses one-tick U5 realization, a full contextual `2x2` R5
gradient, and confidence-gated N5 phase prediction. Runtime features are limited
to IMU tilt/rate, counter joint state, ownership, and Pinocchio momentum terms.

- U5 retains joints 0/1/3 and masks joint 2.
- U5 retained-joint sign accuracy is `1.00/1.00/0.969`.
- R5 targeted H2 sign accuracy is `1.00/1.00`.
- R5 continue/brake ranking is `0.906` over `128/128` pairs.
- N5 confidence-gated phase sign is `1.00/0.992` over `34/128` samples.

### 15.2 H2 Shadow

The fixed-size 12-state H2 model contains incremental tilt/rate, counter position,
and pending residual action. Knot 0 stores the residual; knot 1 applies U5/R5
after the measured one-tick delay. `calc()/calcDiff()` pass finite differences.

Across final FAME `09/11/04`, ALMI `11`, and ALMI manual shadow runs:

- Model-valid decisions: `103`.
- Model-valid H2 sign: `1.00/0.968` roll/pitch.
- Model-valid H2 RMSE: `0.0195/0.0146 rad/s`.
- Decisions: 29 continue, 74 brake, zero reverse.
- H2 p99/max: `3.52/5.29 ms`.
- Full-controller p99/max: `8.95/20.10 ms`.
- Published behavior remained exact frozen 3C.

Per-axis abstention replaced the rejected requirement that both axes be
simultaneously confident. A right-arm MuJoCo address bug exposed by ALMI manual
was corrected using named joint addresses and the model was reverified before
active control.

### 15.3 Active Development

The first `0.01 rad/s` active trust region with default regularization produced
sub-distinguishable corrections and no ordinary benefit. It was rejected.

The only retained cost change reduced generic action/change regularization from
`1.0/0.5` to `0.01/0.005`, leaving model, horizon, trust, and safety unchanged.
The frozen controller alias is `counter_residual_h2_frozen`. The combined frozen
planner/model/H2/harness/config SHA-256 is
`258ccac38b325a2158f44da6aa506526e6046468b98bcb58381e0b48b5540e9a`.

Retained active timing across 14 development runs:

- H2 p50/p95/p99/max: `1.12/2.85/3.48/5.06 ms`.
- Full-controller p50/p95/p99/max: `3.16/7.07/8.80/25.89 ms`.
- H2 model failures: zero.
- One underlying frozen-3C solver failure occurred on the ALMI stumble cell.

### 15.4 Outcome Evidence

| Cell | 3C | Frozen H2 | Decision |
| --- | --- | --- | --- |
| FAME `04` ordinary | Stable 2/3 | Stable 3/3 | Small majority improvement |
| ALMI right `11` ordinary | Drift 3/3 | Drift 3/3 | No B0-level improvement |
| FAME rescue `09` | Drift 3/3 | Drift 3/3 | Rescue retained; neutral continuously |
| FAME rescue `11` | Drift 3/3 | Drift 3/3 | Rescue retained; peak/RMS improved |
| FAME challenge `06` | Fall 1/1 | Fall 1/1 | No additional rescue |
| ALMI manual minus | Stumble; one solver failure | Stumble; one solver failure | No prevention |

For rescue `11`, median peak drift improved from `0.1661` to `0.1558` and RMS
from `0.1101` to `0.1052`. Rescue `09` remained effectively unchanged. H2 did
not improve ALMI ordinary severity to B0 and did not produce stumble prevention.

### 15.5 Final Decision

Iteration 5 is **frozen but not promoted**. It passes rescue-retention, tracking,
model, gradient, and p99 timing gates and provides the best defensible H2 result.
It does not pass the B0-or-better ALMI, additional-rescue, or stumble-prevention
goals.

H3/H5 were not attempted. With one-tick realization delay, H2 has one effective
terminal residual action. H3 requires separately verified 60 ms nominal phase
and response evidence; extending the horizon without it would repeat H3-0.
