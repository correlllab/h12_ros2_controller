# Counter-Balance Iteration 5C

## Risk-/Support-Aware Residual MPC

## Status and Hypothesis

Iteration 5C starts from frozen H2. Finalized Iteration 5/5B and the ALMI speed
sweep are source evidence. B0, 3C, H2, and H3 shadow remain runnable baselines.

The speed sweep found no standing/stumble boundary from `1.5` to `12 s` and no
H3 advantage. Manual stumble remained angularly severe at every speed, while
slow overhead stumbled with low torso tilt and unchanged support-validity state.

The hypothesis is:

> Existing angular response is useful but does not represent incipient
> protective-step risk across families. A minimal verified proprioceptive
> foot/support risk signal may let H2 choose residuals that preserve standing.

## 1. Frozen Contract

Preserve unchanged:

- Complete frozen-3C nominal plus bounded H2 residual architecture.
- Joints 0/1/3, joint-2 mask, `0.01 rad/s` trust, costs, U5/R5/N5, and H2.
- Per-axis confidence abstention and zero-residual fallback.
- Real-compatible atomic observation and Pinocchio path.
- Shared bounds, collision, safety, state update, and publication.
- Policy-/target-blind runtime behavior and timing gates.

H3 is a boundary reference only. H5 is excluded.

## 2. Signal Gate

First log, without changing commands:

- Canonical left/right foot pose and twist relative to captured standing support.
- Foot separation/yaw residual and support-validity hysteresis.
- Pinocchio CoM/support and ZMP/support margin only as diagnostic candidates.
- Existing tilt, rate, divergence, momentum, counter state, and residual.

For manual-minus and overhead across slow and nominal motion, determine whether a
candidate signal:

- Changes before the first `0.03 m` foot-displacement or `0.005 m` lift event.
- Separates nonmoving support from eventual stumble across both families.
- Adds information beyond tilt/rate using family-held-out thresholds.
- Has consistent left/right canonicalization and real-compatible timing.

Select one smallest signal. Do not bundle support, CoM, and ZMP terms.

## 3. Residual Sensitivity Gate

Before inserting a signal into Crocoddyl, use existing bounded momentum-aligned
probe/branch infrastructure to measure whether H2 residual action changes its
one/two-tick trajectory.

Require:

- Correct directional sign on held-out FAME/ALMI families.
- At least 30 distinguishable samples.
- Positive improvement over zero response.
- Action-ranking accuracy at least `0.90`.
- A trust region compatible with frozen H2 authority.

If a signal predicts stumbling but has no measurable residual-action gradient,
use it only for abstention/handoff diagnostics. Do not add an uncontrollable cost.

## 4. Candidate Mechanisms

Test one mechanism at a time in this order:

1. **Risk interpretation:** use existing angular/support signals to penalize
   predicted pre-step risk rather than only terminal tilt/rate magnitude.
2. **Foot-twist risk:** add the smallest validated foot pose/twist residual and
   its verified action gradient to H2.
3. **Support margin:** only if foot twist is predictive but not actionable.
4. **CoM or ZMP margin:** only if it adds held-out information beyond foot/support
   signals and has verified residual sensitivity.

No target-specific threshold, policy identity, broad gain sweep, learned
terminal value, or simultaneous signal bundle is allowed.

## 5. Experimental Ladder

### A. Passive Signal Validation

- Use nominal and slow manual-minus and overhead trajectories.
- Run frozen H2 with diagnostics only.
- Predeclare pre-step onset and family-held-out signal thresholds.

### B. Shadow Cost

- Add one validated risk residual to H2 shadow.
- Require correct risk/action ranking, unchanged 3C publication, and p99 below
  `15 ms`.

### C. Near-Risk Active Screen

No speed boundary exists, so use `12 s` only as an identification regime, not a
success claim. Activate the unchanged trust region first on ordinary FAME/ALMI
guards, then on slow stumble families if ordinary behavior passes.

### D. Nominal Validation

Progress to nominal-speed ALMI only after a slow-family pre-step metric improves.
Use interleaved B0/3C/H2/5C comparisons and three screening repetitions.

### E. FAME Gates

- Retain rescues `09` and `11`.
- Test challenge `06` only after ordinary and stumble-safety gates.
- Use five repetitions for any rescue or stumble promotion claim.

## 6. Success and Stop Criteria

Required:

- Retain every verified FAME rescue with no new severe regression.
- Preserve tracking, safety, confidence, and p99 timing.

Useful success:

- Reduce pre-step foot motion or shift the stumble boundary beyond frozen H2.

Major success:

- Repeatedly convert one ALMI stumble family to standing.

Strong FAME success:

- Recover target `06` or another contemporaneous 3C/H2 fall.

Stop and retain frozen H2 if:

- No real-compatible signal predicts pre-step risk across both families.
- Predictive support risk has no residual-action sensitivity.
- The first validated mechanism does not improve slow pre-step behavior.
- Improvement requires target/policy gates, larger authority, H3/H5, or an
  unverified state bundle.

## 7. Execution Record

- The ALMI speed sweep found no standing boundary from `1.5` to `12 s`.
- Foot linear twist was the earliest consistent real-compatible step signal.
- Current support validity did not reject slow multi-meter step trajectories.
- Foot-twist residual-action sensitivity had zero distinguishable H2 samples in
  both manual-minus and overhead families.
- No Crocoddyl risk residual or active 5C controller was implemented.

Iteration 5C stops at the sensitivity gate. Frozen H2 remains the controller.
