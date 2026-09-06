# ALMI Manip v2/v3 Controller Benchmark

## Purpose

Characterize independently fine-tuned MJLab manipulation checkpoints without
tuning frozen H2. The objective is to select a primary ALMI benchmark based on
static posture envelope, repeatable dynamic behavior, and useful remaining
counter-balance challenge, not on whichever checkpoint yields the most H2 wins.

## Checkpoints

- `mjlab_almi_manip_2`: `model_almi_manip_2.pt`.
- `mjlab_almi_manip_3`: `model_almi_manip_3.pt`.

All three manipulation snapshots use the same 65-D one-history LSTM ABI,
`[64,32]` head, action scale, training gains, default pose, and zero-command
deployment. Only checkpoint weights/training metadata differ:

| Version | Checkpoint SHA-256 | Stored iteration |
| --- | --- | ---: |
| v1 | `dd11b1...` | 2999 |
| v2 | `6432f8...` | 1749 |
| v3 | `3d1c0a...` | 499 |

The first v2/v3 deployment attempt incorrectly reused ALMI-stand gains and a
five-history normalized actor configuration. Corrected v2/v3 deployment matches
the shared manipulation-training contract: one-history actor, normalization off,
`[64,32]` head, lower `kp 200/80`, and lower `kd 6/3`.

## Static Scan

For manual-minus and overhead, run alpha `0.25/0.50/0.75/1.00` static upper
poses with policy-aligned lower initialization. Both v2 and v3 are stable through
alpha `1.00`, unlike the earlier v1 static boundary. No changed static boundary
requires repetition.

## Dynamic Matrix

For every target in Hard (16), Exploration (20), and Boundary (8), run:

- Frame/no counter balance.
- Frozen H2.

Record severity, foot motion, base tilt/rate, support/CoM/ZMP margins,
counter-arm residual/excursion, tracking, solver status, and timing.

Repeat nontrivial or inconsistent cells three times. Classify each important
target as H2 improvement on both/one checkpoint, H2 regression on both/one, or
neutral on both.

## FAME Reference

Run Frame/H2 contemporaneously on rescues `09/11`, challenge `06`, and selected
current FAME Hard/Boundary Frame falls. FAME is a fixed reference, not a
checkpoint-selection metric.

## Decision

Recommend a primary ALMI benchmark based on stable static behavior, repeatable
dynamic response, sensible residual response, and useful remaining challenge.
Treat the other checkpoint as an independent generalization check.

## Results

### Static

Both corrected versioned deployments are stable for manual-minus and overhead at
alpha `0.25/0.50/0.75/1.00`. V3 has slightly lower static drift, but both have
large margin below stumble thresholds.

### Complete Dynamic Matrix

| Checkpoint/controller | Stable | Controller-incomplete | Other severity |
| --- | ---: | ---: | ---: |
| v2 Frame | 44 | 0 | 0 |
| v2 H2 | 44 | 1 | 0 |
| v3 Frame | 44 | 0 | 0 |
| v3 H2 | 44 | 2 | 0 |

There are no H2 severity improvements or regressions on either checkpoint:
Frame and frozen H2 are physically stable 44/44. Frozen H2 has one v2 and two v3
controller-incomplete rows caused by rejected nominal solves.

### Repeated Nontrivial Cells

- `right_upward_overhang_pitch_minus`: Frame and frozen H2 physically stable 3/3
  on both checkpoints; frozen H2 controller-incomplete 3/3.
- `left_arm_overhead`: v2 Frame/H2 physically stable; v3 Frame/frozen H2
  physically stable 3/3, with frozen H2 controller-incomplete 2/3.
- Robust H2 removes these controller-health failures without changing physical
  classification or accepted H2 behavior.

Classification:

- H2 improvement on both: none.
- H2 improvement on one: none.
- H2 regression on both: none by physical severity.
- H2 regression on one: none by physical severity.
- Neutral on both: all 44 targets by physical severity.
- Controller reliability issue on both: `right_upward_overhang_pitch_minus`.
- Additional controller reliability issue on v3: `left_arm_overhead`.

### FAME Reference

- Rescues `09/11`: Frame fall, H2/robust-H2 drift in the contemporaneous screen.
- Challenge `06`: Frame fall 3/3; H2 drift/fall/fall.
- Left manual-grasp plus/minus: Frame and H2 fall 3/3.
- Right manual-grasp plus/minus: Frame and H2 stable in the screen.

Unrecovered FAME manual falls do not saturate the `0.01 rad/s` residual bound;
they have sparse model-valid windows and low residual magnitude. Increased
authority is not justified by current evidence.

## Recommendation

Use `mjlab_almi_manip_2` as the primary ALMI benchmark checkpoint. It matches
v3's complete Frame/H2 physical stability and static envelope while producing
fewer frozen-H2 controller failures. Use v3 as an independent
generalization/reliability check.

Neither checkpoint offers ALMI rescue development cells at current difficulty;
both should be regression guards. The shared nominal-solver failure is addressed
by robust H2; controller development should focus on FAME observability/model
coverage rather than residual authority.

## Merged Full Artifacts

The repaired complete result artifacts are:

- `runs/checkpoint_sweep/almi_manip_v23_full/mjlab_almi_manip_2/`.
- `runs/checkpoint_sweep/almi_manip_v23_full/mjlab_almi_manip_3/`.

Each artifact contains the complete 44-target Frame/frozen-H2 matrix and actual
robust-H2 repair rows, a source-run manifest, regenerated balance/tracking plots,
and dashboards. Robust coverage is deliberately labeled targeted; no unexecuted
robust rows were synthesized.
