# ALMI Checkpoint Robustness Sweep Analysis

## Status

The checkpoint characterization is complete. MJLab ALMI manipulation fine-tuning
substantially expands static and dynamic stability relative to MJLab ALMI stand.
Frozen H2 provides repeatable improvements on both checkpoints, but also exposes
repeatable stand-checkpoint regressions. No controller was tuned.

## 1. Checkpoints and Protocol

- Stand checkpoint SHA-256:
  `080971023516db57e7e0dfcdcc63e5c0433feec5f6ddac22d1df4d672418196d`.
- Manip checkpoint SHA-256:
  `dd11b10177e54dd380af742560d43080c0d8a390eabf3fcfb91f16f3686317ef`.

Both use identical ALMI actor ABI, lower/upper defaults and gains, action scale,
zero command, 50 Hz policy rate, policy-aligned initialization, safety, and
classification. Static scans use alpha `0.25/0.50/0.75/1.00`. Dynamic scans use
all 16 Hard, 20 Exploration, and 8 Boundary targets with Frame and frozen H2.

## 2. Static Stability Expansion

Repeated alpha `0.50/0.75` results:

| Family | ALMI stand | ALMI manip |
| --- | --- | --- |
| Manual-minus alpha `0.50` | Initialization-unstable 2/2 | Stable 2/2 |
| Manual-minus alpha `0.75` | Initialization-unstable 2/2 | Fall 2/2 |
| Overhead alpha `0.50` | Initialization-unstable 2/2 | Stable 2/2 |
| Overhead alpha `0.75` | Initialization-unstable 2/2 | Stable 2/2 |

Fine-tuning removes the stand checkpoint's alpha-0.5 collapse in both families
and keeps overhead stable through the full target in the discovery scan. It does
not make manual-minus alpha `0.75+` viable.

Static roots:

- `runs/static_arm_initialization/mjlab_almi_stand/`.
- `runs/static_arm_initialization/mjlab_almi_manip/`.

## 3. Complete Dynamic Envelope

### MJLab ALMI Stand

| Controller | Stable | Drift | Stumble | Fall | Infrastructure |
| --- | ---: | ---: | ---: | ---: | ---: |
| Frame | 7 | 4 | 0 | 33 | 0 |
| Frozen H2 | 8 | 3 | 0 | 33 | 0 |

### MJLab ALMI Manip

| Controller | Stable | Drift | Stumble | Fall | Infrastructure |
| --- | ---: | ---: | ---: | ---: | ---: |
| Frame | 37 | 2 | 0 | 5 | 0 |
| Frozen H2 | 38 | 0 | 0 | 4 | 2 |

Fine-tuning therefore adds 30 Frame-stable and 30 H2-stable cells while removing
28/29 falls respectively. Most of the dynamic envelope expansion comes from the
checkpoint, not counter balance.

Dynamic roots:

- `runs/checkpoint_sweep/20260905_024720_almi_checkpoint_stand/`.
- `runs/checkpoint_sweep/20260905_032931_almi_checkpoint_manip/`.

## 4. Repeatable H2 Improvements

### Checkpoint-Independent Boundary Improvement

- Stand right boundary `11`: Frame drift 3/3 → H2 stable 3/3.
- Manip left boundary `11`: Frame drift 3/3 → H2 stable 3/3.

This opposite-side pair is the strongest cross-checkpoint predictive-control
evidence and should be a primary Iteration 5D development cell.

### Additional Manip Improvement

- Left lateral-high-reach: Frame drift/stable/drift → H2 stable 3/3.
- Right arm overhead: Frame fall 3/3; H2 produced one stable physical run but two
  operationally incomplete runs. This is a high-value but solver-obscured rescue
  opportunity, not a completed claim.

### Stand Ordinary Improvement

- Right lateral-high-reach: H2 stable 3/3, while Frame was
  drift/stable/stable. Both have stable majority, so this is continuous evidence
  rather than a severity transition.

## 5. Repeatable H2 Regressions

- Stand right-forward-rank6: Frame drift 3/3 → H2 fall 3/3.
- Stand right-diagonal-rank6: Frame fall/drift/drift versus H2 drift/fall/fall;
  H2 changes the majority from drift to fall.
- Stand right-arm-forward-yaw: Frame stable 3/3 versus H2
  drift/stable/stable. Majority remains stable, but one regression is retained as
  a guard.

These cases show that the existing H2 angular objective can apply counter-arm
action with the wrong physical consequence on the stand checkpoint. They are
mandatory negative guards for 5D.

## 6. Counter-Arm Authority and Reliability

Manip checkpoint right-arm-overhead is the clearest apparent-authority case:
Frame falls 3/3, while H2 can remain physically stable but repeatedly incurs
one-step solver/operational failures. Improving solver/fallback reliability may
convert existing physical authority into a repeatable rescue without increasing
trust.

Conversely, the stand right-forward and right-diagonal regressions demonstrate
that authority is not universally beneficial. Risk/phase interpretation must
improve before any authority increase.

Across complete H2 matrices:

- Stand full-controller p50/p95/p99/max:
  `3.09/7.10/8.93/26.50 ms`.
- Manip full-controller p50/p95/p99/max:
  `2.78/6.38/7.87/26.01 ms`.
- H2 solve p99: `3.39 ms` stand and `2.86 ms` manip.
- Maximum residual norm: approximately `0.017 rad/s`.

P99 timing passes. Rare maxima and one-step solver failures remain relevant.

Manipulation tracking is precise on completed stable/drift runs. Most imprecise
rows are trajectories terminated by falls before target completion.

## 7. Best Iteration 5D Evidence Panel

Positive development cells:

- Stand right boundary `11`: repeated drift → stable.
- Manip left boundary `11`: repeated drift → stable.
- Manip left lateral-high-reach: repeated majority drift → stable.
- Manip right-arm-overhead: possible fall rescue obscured by solver reliability.

Negative guards:

- Stand right-forward-rank6: repeated H2 fall regression.
- Stand right-diagonal-rank6: repeated majority regression.
- Stand right-arm-forward-yaw: occasional stable → drift regression.

Mandatory frozen FAME gates remain rescues `09/11` and challenge `06`.

## 8. Conclusions

1. `mjlab_almi_manip` substantially expands static overhead and complete dynamic
   stability, but manual-minus high-alpha instability remains.
2. Frozen H2 has repeatable cross-checkpoint boundary improvements.
3. The strongest 5D opportunity is a policy-blind mechanism that retains the
   left/right boundary-11 improvements while removing stand right-forward and
   right-diagonal regressions.
4. Right-arm-overhead on the manip checkpoint shows physical rescue authority
   but insufficient solver reliability.
5. Increasing residual authority is not justified; phase/risk interpretation and
   fallback reliability are the evidence-selected 5D mechanisms.

The sweep supports continuing Iteration 5D with the existing H2 foundation and a
compact cross-policy guard/development panel. It does not support target-specific
or checkpoint-specific controller logic.
