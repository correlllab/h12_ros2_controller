# Counter-Balance Iteration 5E Analysis

## Status

Iteration 5E freezes `counter_residual_h2_robust`. It preserves accepted frozen
H2 behavior and adds an existing bounded-SciPy nominal fallback only after a
rejected one-step Crocoddyl nominal solve.

The earlier `infrastructure` classifications are benchmark operational failures,
not external OS, DDS, MuJoCo, or process crashes. A rejected nominal solve made
`operational_complete` false before the physical outcome could be accepted by
the standard classifier. Robust reruns are required to obtain an actual
stable/drift/stumble/fall result for those same cells.

## 1. Current Identification Evidence

The corrected v2/v3 benchmark supersedes Iteration 5D guards:

- Both checkpoints are Frame-stable 44/44 dynamically.
- Frozen H2 provides no severity improvement.
- V2 H2 remains physically stable on right-upward-overhang-minus 3/3 but has a
  nominal solver failure in every run.
- V3 H2 has the same stable/controller-incomplete pattern 3/3 and left overhead
  stable with solver failures 2/3.

Current FAME evidence:

- `09/11`: Frame fall and H2 drift in the current reference.
- `06`: Frame fall 3/3; H2 drift/fall/fall.
- Left manual-grasp plus/minus: Frame and H2 fall 3/3.

Unrecovered FAME falls do not saturate the `0.01 rad/s` residual limit. They have
few model-valid ticks and residual norms below the bound. Increased authority is
not justified.

## 2. Mechanism

For every nominal solve:

1. Run unchanged one-step Crocoddyl.
2. If accepted, return the exact frozen-H2 result.
3. If rejected, run the existing bounded SciPy solver on the same objective and
   bounds.
4. Pass the fallback nominal through unchanged H2 residual MPC and the shared
   safety/finalizer.
5. If fallback is invalid, retain the existing hold path.

No model, state, authority, cost, policy/target gate, or simulator input changes.

## 3. Cross-Policy Reliability Results

| Checkpoint/cell | Frozen H2 | Robust H2 |
| --- | --- | --- |
| V2 right-upward-overhang-minus | Stable 3/3; controller-incomplete 3/3 | Stable and controller-complete 3/3 |
| V3 right-upward-overhang-minus | Stable 3/3; controller-incomplete 3/3 | Stable and controller-complete 3/3 |
| V3 left overhead | Stable 3/3; controller-incomplete 2/3 | Stable and controller-complete 3/3 |

Fallback activation across robust repetitions:

- V2 right-overhang: two fallback ticks over three runs.
- V3 right-overhang: three fallback ticks over three runs.
- V3 left-overhead: three fallback ticks over three runs.

Runs with accepted Crocoddyl output remain on the frozen path and are
command-identical by unit test.

## 4. FAME Guard

Robust H2 produced:

- `06`: drift in the screen, with no fallback activation.
- `09`: drift, no fallback activation.
- `11`: drift, no fallback activation.
- Left manual plus/minus: fall, no fallback activation.

These outcomes match the current frozen-H2 distribution. Robust fallback does
not create a false additional-rescue claim and does not alter accepted rescue
commands.

## 5. Timing

Across nine robust v2/v3 reliability runs:

- Full-controller p50/p95/p99/max:
  `2.73/6.27/7.87/19.05 ms`.
- Fallback activations: eight controller ticks.
- All robust runs completed operationally.

Timing passes the `15 ms` p99 and `20 ms` command-period gates.

## 6. Final Decision

Iteration 5E is **promoted as a reliability improvement over frozen H2**.

It removes the repeatable current v2/v3 H2 controller-health failures, preserves
physical stability and FAME rescue behavior, and changes no accepted nominal
command. It does not expand
FAME fall authority: `06` is not repeatably rescued and left manual falls remain.

The remaining FAME limit is observability/model-valid coverage, not demonstrated
residual saturation. Further authority tuning is not supported. Future work
should start from robust H2 and identify why model confidence disappears before
the unrecovered manual falls.

## 7. Checkpoint Compatibility Note

`model_almi_manip.pt`, v2, and v3 share the same recurrent actor architecture:
65 inputs, one history sample, LSTM hidden size 64, `[64,32]` policy head,
normalization disabled, and 12 actions. They differ in checkpoint weights and
stored iteration metadata.

The first v2/v3 deployment configs were incompatible because they reused stand
deployment gains/normalization/history. Corrected v2/v3 configs match the
manipulation-training contract. The v2/v3 full repair artifacts are under
`runs/checkpoint_sweep/almi_manip_v23_full/`.
