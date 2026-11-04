# Counter-Balance Iteration 5B

## Verified-Response H3 Crocoddyl MPC

## Status and Hypothesis

Iteration 5B is executed and stopped after its ordinary active gate. B0, frozen
3C, and `counter_residual_h2_frozen` remain independently runnable baselines.
H3 remains available in shadow only.

H3 changes only the horizon. The hypothesis is:

> With one-tick U5 realization delay, H2 has one effective terminal residual.
> A separately verified 60 ms H3 model can make the second residual effective
> and improve continue/brake/reverse timing without changing authority or state.

The architecture remains:

\[
u_k=u^{3C}_{nominal,k}+\delta u_k.
\]

## 1. Frozen H2 Contract

Iteration 5B preserves without tuning:

- Complete frozen-3C nominal planning and zero-residual parity.
- The 12-element incremental tilt/rate, counter-position, and pending-action
  state.
- Velocity-level residual control on joints 0/1/3 with joint 2 fixed at zero.
- Contextual U5/R5 models using joint state, IMU tilt/rate, ownership, and
  Pinocchio momentum only.
- N5 per-axis confidence and no-zero-crossing abstention.
- The `0.01 rad/s` residual trust region.
- Action/change weights `0.01/0.005` and all other H2 costs.
- Existing bounds, collision backtracking, safety, fallback, command-state
  update, and atomic publication.

No policy/target identity, simulator truth, full-state model, CoM/ZMP term,
larger authority, learned terminal value, or new support controller is allowed.

## 2. Required 60 ms Validation

H3 implementation is blocked until U5/R5/N5 are verified at three control ticks.

### U5-3

Retain the verified one-tick delay and weak-joint mask. Validate that a residual
issued at knot 0 has measured realization at knots 1/2/3 and that a knot-1
residual has useful realization at knot 3. Require:

- Retained-joint sign accuracy at least `0.90`.
- At least 30 distinguishable samples per retained joint.
- Positive improvement over zero realization.
- Stable carryover sign through `60 ms`.

### R5-3

Extend exact mechanical branches to samples at `20/40/60 ms`. Fit only the
additional impulse/carryover term required by H3. Keep the same contextual
features and ownership canonicalization.

Require on family/amplitude-held-out FAME and ALMI data:

- Targeted roll/pitch sign accuracy at least `0.90`.
- Positive zero-response improvement on both axes.
- Continue/brake/reverse ranking accuracy at least `0.90`.
- Nontrivial distinguishability coverage.
- Stable gradient sign across ownership, policy, amplitude, and phase.

### N5-3

Extend the same nominal predictor and physical no-zero-crossing gate to `60 ms`.
Do not add state. Require:

- Confident roll/pitch phase-sign accuracy at least `0.90`.
- At least 30 confident held-out samples per axis.
- Correct peak/zero-crossing ordering within one tick when covered.
- Explicit abstention where 60 ms phase cannot be trusted.

Failure of one 60 ms gate stops H3. H2 remains frozen.

## 3. H3 Implementation Difference

Use three running knots and one terminal knot with the same Crocoddyl action
models and state:

1. Knot 0 stores `delta u_0`; no response is assumed before U5 delay.
2. Knot 1 applies realized `delta u_0` and stores `delta u_1`.
3. Knot 2 applies the verified carryover/realization of prior residuals, applies
   realized `delta u_1`, and stores `delta u_2`.
4. The terminal knot evaluates the unchanged cost at `60 ms`.

`delta u_2` cannot affect the H3 terminal under the one-tick delay and should
remain zero through regularization. It is retained only to preserve standard
Crocoddyl shooting structure.

Preallocate the three action models, terminal model, shooting problem, solver,
states, and warm start. Expose the verified 60 ms derivatives through
`calcDiff()` and check them against finite differences.

Shadow and active paths must continue to use the single shared finalizer. Invalid
H3, uncertain phase, moving support, or late solve selects zero residual and
returns to frozen 3C.

## 4. Experimental Ladder

### Stage A: 60 ms Models

- Extend existing branch/probe logs to H3 without new controller state.
- Validate U5-3, R5-3, and N5-3 with fixed thresholds.
- Stop before Crocoddyl implementation if any gate fails.

### Stage B: H3 Unit and Shadow

- Verify zero-residual H2/3C parity.
- Finite-difference-check every running and terminal derivative.
- Run H3 shadow on FAME `09/11/04`, ALMI right `11`, and ALMI manual.
- Compare H3 nominal/response predictions and phase decisions with measured
  asynchronous 3C trajectories.
- Require no model/solver failure and no publication difference from 3C.

### Stage C: Low-Risk Active Screen

Only after shadow passes, compare B0, 3C, frozen H2, and H3 on:

- FAME `04` ordinary guard.
- ALMI right `11` ordinary guard.

Use three repetitions. Change no cost, trust, state, or model while evaluating
the horizon effect.

### Stage D: Capability Gates

Proceed in order:

1. Preserve FAME rescues `09` and `11` for three screening repetitions.
2. Test FAME challenge `06` against Frame, 3C, frozen H2, and H3.
3. Test ALMI manual-grasp-minus and one second stumble family for pre-step
   prevention.
4. Use five repetitions only for a changed rescue/stumble cell before promotion.

All runs remain serial, headless, contemporaneous, and interleaved.

## 5. Timing Gates

- Profile from the first 60 ms model and H3 shadow execution.
- Require H3 solve p99 below `6 ms`.
- Require complete-controller p99 below `15 ms`, preferably below `12 ms`.
- Report maxima and reject accepted commands that miss the `20 ms` control
  period.
- Reuse one observation, Pinocchio update, nominal plan, and preview per tick.
- Log one compact record per H3 solve, not per knot.

## 6. Success and Stop Criteria

Iteration 5B succeeds only if H3:

- Retains both repeated FAME rescues.
- Introduces no new majority severe regression.
- Improves at least one H2 limitation materially: ALMI ordinary severity, FAME
  `06`, repeated stumble prevention, or a preregistered ordinary metric.
- Preserves tracking, safety, confidence, gradient, and timing gates.

Strong evidence is an additional repeated FAME rescue or repeated ALMI
stumble-to-standing prevention.

Stop and retain frozen H2 if:

- A 60 ms U5/R5/N5 gate fails.
- H3 merely reproduces H2 decisions or outcomes.
- Timing loses required margin.
- Improvement requires larger authority, new state, target/policy gates, or
  unverified terminal behavior.

H5 is outside Iteration 5B.

## 7. Execution Record

- The final 60 ms U5/R5/N5 gate passed.
- H3 `calc()/calcDiff()` and exact 3C shadow publication passed.
- Shadow H3 p99 was `4.13 ms`; full-controller p99 was `9.35 ms`.
- H3's second action was effective; inert third action was corrected to zero.
- FAME `04` H3 was stable/drift/stable, below frozen H2's 3/3 stable evidence.
- ALMI right `11` H3 remained drift 3/3 and was continuously worse than H2.
- Rescue/challenge/stumble active panels were blocked by the ordinary gate.

Iteration 5B is **not promoted**. Increasing the verified horizon from H2 to H3
does not provide meaningful capability beyond frozen H2.
