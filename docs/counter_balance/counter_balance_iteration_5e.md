# Counter-Balance Iteration 5E

## Multi-Policy Fall Rescue

## Evidence and Hypothesis

The current v2/v3 checkpoint benchmark supersedes Iteration 5D guards. Both
checkpoints are Frame-stable 44/44; frozen H2 adds no severity improvement and
causes repeatable nominal-solver failures on
`right_upward_overhang_pitch_minus`. V3 also has an inconsistent
`left_arm_overhead` failure.

Current FAME evidence retains H2 rescues `09/11`, rescues `06` only 1/3, and
leaves both left manual-grasp extremes as falls 3/3. Those unrecovered falls do
not saturate residual authority, so larger residual bounds are rejected.

The 5E hypothesis is:

> A deterministic fallback for rejected nominal Crocoddyl velocity solves can
> remove cross-checkpoint H2 regressions without changing accepted H2 behavior,
> rescues, residual authority, prediction, or safety.

## Controller Change

Keep frozen H2 independently runnable. Add one robust variant:

- Run the unchanged one-step Crocoddyl nominal planner first.
- If accepted, preserve the exact frozen H2 path.
- If rejected, run the existing bounded SciPy velocity solve on the same
  objective, targets, and bounds.
- Feed the fallback nominal into unchanged H2 residual MPC and the single shared
  finalizer.
- Reject nonfinite/infeasible fallback output and use the existing hold.

No policy/checkpoint/target identity, new state, larger trust, new cost, or
simulator input is allowed.

## Experimental Ladder

1. Unit-test accepted-path parity, rejected-path fallback, bounds, diagnostics,
   publication count, and timing.
2. Repeat `right_upward_overhang_pitch_minus` on v2/v3 and
   `left_arm_overhead` on v3 against frozen H2/Frame.
3. Confirm no change on neutral v2/v3 cells.
4. Recheck FAME `09/11`, challenge `06`, and left manual falls.

## Gates

Promote only if robust H2:

- Removes repeatable v2/v3 operational regressions.
- Is command-identical to frozen H2 whenever nominal Crocoddyl is accepted.
- Retains FAME `09/11` and does not worsen current `06`/manual outcomes.
- Keeps full-controller p99 below `15 ms`.

Stop if SciPy fallback is also infeasible, violates timing, changes normal H2
behavior, or fails to remove the repeated checkpoint regressions.

## Execution Result

Robust H2 removes every repeated v2/v3 operational regression while preserving
accepted frozen-H2 behavior. It is stable 3/3 on the shared right-overhang failure
for both checkpoints and stable 3/3 on v3 left overhead. Fallback activates only
on rejected nominal solves. FAME accepted paths remain unchanged.

Iteration 5E freezes `counter_residual_h2_robust` as the new reliability-improved
controller. It does not claim additional FAME fall authority.
