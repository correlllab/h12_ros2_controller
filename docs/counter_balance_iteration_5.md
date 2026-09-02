# Counter-Balance Iteration 5

## Verified-Response Few-Step Crocoddyl MPC

## Status and Hypothesis

Iteration 5 is a proposed experimental controller. B0 and frozen Iteration 3C
`counter_ddp_velocity_wide` remain the baselines. Iteration 4C remains an
unpromoted diagnostic controller.

The hypothesis is:

> Frozen 3C provides a proven nominal counter-balancing action. Crocoddyl can use
> verified causal response gradients to optimize small residual corrections that
> know when to continue, brake, or reverse.

Start with H2 at the existing velocity-control level. Progress to H3 and then H5
only when a measured event lies beyond the shorter horizon and prediction,
behavior, and timing evidence justify expansion.

## 1. Frozen Evidence

Iteration 5 must preserve these conclusions:

- Frozen 3C's immediate manipulation feedforward produces the repeated FAME
  rescues `right_fast_fall_search_09_scale_78` and
  `right_fast_fall_search_11_scale_78`.
- Iteration 4A's common full-state passive model failed roll-rate and lateral
  support-relative CoM-velocity gates. Do not rebuild it as an Iteration 5
  prerequisite.
- Iteration 4B preview-only scaling lost rescue `09`. Measured response is
  necessary, but one scalar applied to the complete controller did not improve
  the ordinary tradeoff.
- Iteration 4B H3-0 repeated 3C costs without verified action-conditioned
  response or a useful terminal objective. It was behaviorally worse and too
  slow.
- Iteration 4C proved that measured response can modify counter action without
  losing 3C's feedforward, rescue behavior, or timing margin.
- Iteration 4C's tilt/rate gains could not reliably select braking or reversal.
  Strong gains worsened ALMI and produced KKT-invalid trials.

Iteration 5 therefore identifies the incremental physical effect of a residual
action. It does not tune another authority scheduler or fit the complete nominal
humanoid response.

## 2. Controller Contract

The runtime controller is:

- Velocity-level and receding-horizon.
- Policy-blind and target-blind.
- Limited to the existing four proximal counter-arm joints.
- Compatible with IMU, measured joint position/velocity/torque, and justified
  Pinocchio quantities.
- Identical to 3C for manipulation-arm ownership and publication.
- Subject to frozen joint, excursion, collision, estop, and atomic publication
  safeguards.

It must not use policy identity, target identity, historical outcome labels,
simulator contact truth, external-force truth, or exact simulator base state.

## 3. Complete 3C Nominal Plus Residual

The command is:

\[
u_k = u^{3C}_k + \delta u_k.
\]

`u_3C,k` is the complete frozen 3C command, including its proven manipulation
feedforward, measured feedback, bounds, and safety handling. It is not only the
feedforward term.

The implementation contract is:

- Compute the current 3C command through the frozen 3C path.
- Optimize only the correction sequence `delta U`.
- Apply safety constraints to the combined command.
- Publish only the first combined action.
- Replan from a fresh atomic observation every control tick.
- At `delta u = 0`, reproduce the complete 3C command bit-for-bit before final
  transport serialization and within the frozen numeric tolerance after it.
- If MPC observation, model, solve, timing, or safety is invalid, publish the
  frozen 3C result while support remains valid.

The residual trust region must preserve 3C's early authority. While the known
manipulation disturbance is rising, `delta u_0` may add damping or an orthogonal
correction but may not reduce the nominal feedforward-aligned projection unless
the verified model predicts a braking benefit larger than its uncertainty.

Continue, brake, and reverse are descriptions of the correction's projection
onto the nominal action. They are not policy-, target-, or outcome-specific
modes.

## 4. Matched Branching for Causal Identification

Saved-state matched branching is primarily an offline simulation identification
and validation tool. It is not part of runtime control.

At selected physical states, save enough state to reproduce the future exactly:

- MuJoCo positions, velocities, actuators, time, and deterministic seeds.
- Lower-body policy internal state and command history.
- Controller references, filters, delays, and lifecycle state.
- Pending communication samples or equivalent deterministic replay state.

From the identical saved state, execute matched branches:

1. Frozen 3C nominal.
2. `3C + delta u` for one or more bounded correction ticks.
3. Optional symmetric `3C - delta u` for central differences.

Measure causal differences such as:

\[
\Delta\omega_{k+j}
=
\omega^{3C+\delta u}_{k+j}
-
\omega^{3C}_{k+j},
\]

\[
\Delta x_{k+1}
\approx
B(x_k)\delta u_k.
\]

Matched branches cancel the shared nominal lower-body response. Iteration 5 does
not need to relearn that response at global accuracy to identify whether a
correction improves the next few ticks.

Branch states must cover rising disturbance, near peak, early return, and
pre-step response. Use basis corrections, symmetric signs, and at least two
small amplitudes to establish sign, local linearity, and the trusted action
radius.

Only real-compatible features may enter the fitted model. Simulator truth and
contact state may be used offline to audit branch equality or explain failures,
never as runtime features.

## 5. Minimum Runtime State and Model

The initial response state is:

\[
x_k = [e_{\theta,k},\ \omega_k,\ q_{c,k},\ \dot q_{c,k}],
\]

with measured support validity as context. Known manipulation momentum and rate
provide short disturbance preview.

Use structured transitions:

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

Predict a common compact 3C nominal trajectory `x_bar_3C` over H2 using current
measurements, recent angular evolution, known manipulation preview, and frozen
3C commands. Candidate trajectories are deviations from that common nominal:

\[
\Delta x_0 = 0,
\]

\[
\Delta x_{j+1}
=
A_j\Delta x_j
+B_j\delta u_j,
\]

\[
x_{j}=\bar{x}^{3C}_{j}+\Delta x_j.
\]

The common nominal predictor need only preserve the local phase well enough for
H2 action comparison. Fit `A` and `B` from matched differences and active
validation; do not fit an unconstrained full-humanoid transition.

## 6. Minimum Verified Components

### 6.1 O5: Observation

Reuse the validated atomic Iteration 4A/4C path:

- Timestamped IMU orientation and angular velocity.
- Measured moving/counter joint state and torque.
- Release-time references.
- One Pinocchio update per control tick.
- Proprioceptive foot pose/twist and support validity.

Recheck timestamp age and finite-difference consistency before identification.
H2 does not require support-relative lateral CoM velocity.

### 6.2 U5: Correction Realization

U5 maps requested `delta u` to realized counter velocity and momentum difference.
Identify command delay, lag, position-dependent effectiveness, clipping,
backtracking, and weak directions.

U5 passes when:

- Zero-correction branches are identical.
- Response sign is correct on every retained axis.
- Delay is predicted within one control tick.
- Expected response gain excludes zero within the trusted region.
- One/two-step realization beats command-equals-realization and zero-response
  baselines.

Matched simulation branches provide dense identification. Bounded active pulses
using the same real-compatible observations must confirm sign, delay, and useful
gain before active MPC relies on U5.

### 6.3 R5: Incremental Angular Response

R5 predicts the correction-induced difference, not the complete base response:

\[
\Delta\omega_{k+1}
=
B_\omega(x_k,s_k)\delta u_k+r_k.
\]

Use exact tilt integration and fit only uncertain angular-rate sensitivity and a
bounded residual. Factor the model through realized counter momentum when U5 and
Pinocchio provide a better-conditioned physical representation.

R5 passes on held-out branch states when:

- Roll and pitch pass separate sign and error gates.
- Symmetric corrections produce consistent central-difference gradients.
- Gradient sign remains correct across retained action amplitudes.
- Relative continue/brake/reverse ranking agrees with measured branch outcomes
  on at least `90%` of materially distinguishable cases.
- Braking direction and zero crossing are predicted within one tick when they
  occur inside H2.
- Model uncertainty blocks extrapolation outside the verified state/action
  region.

Prioritize physical sign, relative ranking, and useful gradients over global
state-prediction accuracy.

### 6.4 S5: Support and Step Onset

Use existing proprioceptive support-validity and step-onset signals as context.
Do not block H2 on a full contact or foot-placement model.

- Valid standing support: allow normal residual optimization.
- Incipient support motion: tighten correction and excursion limits.
- Moving or uncertain support: stop claiming stumble prevention and hand off
  conservatively to the verified baseline/safety path.
- Recovered support: initially remain with the baseline; post-step recovery is
  outside the first Iteration 5 scope.

Measure support-signal latency on held-out stumbles. It may change H2 costs or
constraints only if its conservative latency is useful before support motion.

## 7. H2 Crocoddyl Formulation

Use a fixed-size discrete Crocoddyl `ActionModelAbstract` with two running knots
and one terminal knot. The decision is:

\[
\delta U=[\delta u_0,\delta u_1].
\]

`calc()` evaluates the common 3C nominal plus the incremental U5/R5 transition.
`calcDiff()` exposes the verified local `A` and `B` gradients. Check every
analytic derivative against finite differences throughout the trusted region.
Wrong-sign or unsupported gradients invalidate the MPC action.

Initial running costs are limited to:

- Predicted tilt, angular-rate, and positive-divergence reduction.
- Small residual magnitude and action change.
- Counter velocity, acceleration, excursion, and return reserve.
- Deviation outside the identified state/action trust region.
- Conservative support-context penalties.

Use an interpretable terminal braking objective:

\[
\ell_N
=
w_\theta\|e_{\theta,N}\|^2
+w_\omega\|\omega_N\|^2
+w_d\|\max(e_{\theta,N}\odot\omega_N,0)\|^2
+w_r\|q_{c,N}-q_{c,ref}\|^2.
\]

Do not add a learned terminal viability model initially.

Hard constraints include combined-command joint/velocity limits, per-step
acceleration, frozen excursion, first-action collision/safety validity,
manipulation ownership, and the U5/R5 trust region.

## 8. ALMI Scope

The initial ALMI target is stumble prevention, not general post-step recovery.
Iteration 5 should keep a trajectory that would otherwise initiate a protective
step inside standing support by choosing an earlier continue, brake, or reverse
correction.

Once support is already moving, use conservative handoff. Reduced torso motion
or foot travel after step onset is diagnostic evidence, not a stumble-to-standing
claim for the initial controller.

## 9. Progressive Workflow

### Stage 0: Stable Baselines

- Use serial, headless trials.
- Terminate stale simulation, ROS, policy, bridge, logging, and sweep processes.
- Check CPU, GPU, memory, swap, and disk health.
- Interleave contemporaneous Frame, B0, and 3C trials.
- Profile complete timing from the first shadow execution.

### Stage 1: Matched Identification

- Verify zero-correction branch identity.
- Identify U5 delay and realization.
- Fit R5 one/two-step incremental gradients.
- Hold out complete state families and both arm ownership directions.
- Reject the model if action sign or ranking is unreliable.

Do not wait for CoM, ZMP, a full contact model, or learned terminal viability.

### Stage 2: H2 Shadow

Run H2 every tick while publishing frozen 3C.

Require:

- Exact zero-correction command parity.
- Runtime inputs limited to real-compatible O5 signals.
- Predicted nominal and incremental responses inside calibrated uncertainty.
- All candidate actions inside the trusted region.
- Shadow total-controller p99 below `10 ms`.
- No stale, infeasible, or late result accepted.

### Stage 3: H2 Active

Start on low-risk ordinary cases with a small correction radius. Change one
model, cost, bound, or horizon mechanism at a time. Inspect matched-branch and
active predicted-versus-measured traces after every changed outcome.

Expand to rescue and stumble-prevention cases only after ordinary behavior is no
worse than 3C, gradients remain calibrated, and active total-controller p99 is
below `15 ms`.

### Stage 4: H3 and H5

Increase the horizon only when H2 repeatedly selects the wrong phase because a
measured braking point or zero crossing lies beyond two ticks.

Progress in order:

1. H2 with the minimum model.
2. H3 with identical model, costs, and constraints.
3. H5 only if H3 still misses a verified event that H5 captures.

Do not change horizon and model content together. Return to the shorter horizon
if behavior does not improve repeatedly or timing margin is lost.

### Stage 5: Evidence-Selected State Extensions

Add one quantity only when active H2 evidence identifies a specific missing
state:

- CoM or capture point when angular ranking is correct but standing-support
  outcomes remain wrong.
- Pinocchio ZMP or support margin when it predicts pre-step loss better than the
  compact angular state and passes real-compatible consistency checks.
- A richer support-transition model when prevention fails specifically because
  step onset is detected too late.
- Learned terminal viability only when H3/H5 remains myopic despite accurate
  local incremental dynamics and the simple terminal cost.

Every extension must improve held-out prediction and active behavior in a
separate ablation.

## 10. Crocoddyl and Timing

Crocoddyl is the default few-step solver. Use the existing BoxFDDP/BoxQP path
where compatible, while avoiding H3-0's repeated computation:

- Preallocate fixed H2 state, action, residual, cost, and solver data.
- Update numeric terms in place.
- Reuse one Pinocchio update and one manipulation preview per tick.
- Warm-start from the shifted previous residual sequence.
- Start with one BoxFDDP real-time iteration.
- Apply BoxQP polishing only when required by the KKT gate.
- Log one compact record per actual solve, not per internal knot or logger tick.
- Reject late results atomically.

Total-controller p99 must remain below `15 ms`, preferably below `10 ms` for H2
and H3.

## 11. Compact Experimental Panel

Randomize or interleave Frame, B0, 3C, and Iteration 5 in the same stable machine
session.

Required FAME cells:

- `right_fast_fall_search_09_scale_78`: existing repeated 3C rescue.
- `right_fast_fall_search_11_scale_78`: existing repeated 3C rescue.
- `right_fast_fall_search_06_scale_74`: major surpass-previous-iterations
  challenge. Iteration 2 historically rescued it, but 3C did not reproduce that
  recovery.
- One ordinary 3C regression and one nonduplicate ordinary geometry.

Any Iteration 5 rescue claim for `06` requires contemporaneous, interleaved
Frame, 3C, and Iteration 5 trials from the same stable machine session.

Required ALMI cells:

- Right target `11` as the ordinary guard.
- Manual-grasp-minus and one second repeated stumble family as prevention tests.

Use three complete repetitions for screening and five for every rescue,
regression, stochastic, or changed cell before promotion. Report infrastructure
failures separately.

Required traces include nominal 3C, residual actions, realized counter response,
predicted/measured incremental tilt and rate, gradient uncertainty, action
projection, support onset, foot motion, excursion, clipping/backtracking,
tracking error, KKT, and complete timing.

## 12. Success Hierarchy

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

Stop and retain B0/3C if U5/R5 cannot predict correction sign reliably, H2 loses
an existing rescue, ordinary behavior does not improve, timing exceeds the gate,
or improvement requires policy, target, or simulator-truth gates.

## 13. Initial Deliverable

The first implementation should contain:

- Source-bound matched-branch datasets and U5/R5 validation reports.
- Complete frozen 3C command generation with exact zero-residual parity.
- A fixed-size H2 Crocoddyl action model exposing verified gradients.
- Bounded residual trust regions and existing safety checks.
- Existing support-validity context with conservative moving-support handoff.
- Parity, gradient, constraint, fallback, and timing tests.
- One H2 shadow panel followed by one low-risk active panel.

This minimum implementation directly tests the Iteration 5 hypothesis. Additional
state, horizon, support prediction, or learned terminal value is justified only
by a specific measured failure.
