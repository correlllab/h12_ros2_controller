# ALMI Checkpoint Robustness Sweep

## Purpose

Compare MJLab ALMI stand and manipulation-fine-tuned checkpoints under identical
static-pose and dynamic manipulation benchmarks. Frozen H2 is characterized, not
tuned.

Checkpoints:

- `mjlab_almi_stand`: `model_almi_stand.pt`, SHA-256 `080971...`.
- `mjlab_almi_manip`: `model_almi_manip.pt`, SHA-256 `dd11b1...`.

Both use the same 65-D recurrent actor ABI, gains, default positions, action
scale, zero command, policy rate, safety lifecycle, and policy-aligned initial
lower pose/base height.

## Static Scan

Use manual-grasp-minus and overhead families at alpha
`0.25/0.50/0.75/1.00`, with no arm motion or impulse. Compare against the clean
`mjlab_almi_stand` policy-root matrix and repeat cells around any changed static
boundary.

Record stable/drift/stumble/fall/initialization-instability, foot displacement,
base tilt/rate, CoM/ZMP/support margins, readiness, and videos.

## Dynamic Matrix

Run all 44 targets from:

- Hard: 16.
- Exploration: 20.
- Fast boundary: 8.

For each checkpoint run:

- `frame_task`.
- Frozen H2 `counter_residual_h2_frozen`.

Keep target geometry, nominal fast trajectory, classifier, safety, H2 model,
trust, costs, and timing unchanged. Runs are serial and headless.

## Repetition Rule

Repeat three times only when:

- Checkpoints change outcome severity.
- H2 changes outcome severity relative to Frame.
- A cell is near a stable/drift/stumble/fall boundary.
- Infrastructure or solver failure obscures a physical outcome.

Use five repeats only for a promotion-quality transition claim.

## Metrics

- Outcome class and checkpoint/H2 transition.
- Foot displacement/lift and onset.
- Peak/RMS torso tilt and angular rate.
- CoM/ZMP/support margins.
- H2 residual magnitude, confidence, continue/brake/reverse decisions, and
  counter-arm excursion.
- Manipulation tracking and complete-controller timing.

## Decision Outputs

Determine:

1. Static/dynamic envelope expansion from ALMI stand to ALMI manip.
2. Repeatable H2 improvements and regressions on each checkpoint.
3. Strongest cross-policy counter-balance opportunities.
4. Cases with weak apparent counter-arm authority.
5. The minimal source-evidence panel for Iteration 5D.

No policy or controller parameter is tuned in this sweep.

## Execution Result

- Fine-tuning expands static overhead stability through alpha `1.0` and
  manual-minus stability from below `0.5` to alpha `0.5`.
- Complete dynamic matrices contain 44 targets × Frame/H2 per checkpoint.
- Manip fine-tuning reduces Frame falls from 33 to 5 and H2 falls from 33 to 4.
- H2 has repeatable improvements and regressions documented in the analysis.

See `counter_balance_analysis_almi_checkpoint_sweep.md` for final source evidence.
