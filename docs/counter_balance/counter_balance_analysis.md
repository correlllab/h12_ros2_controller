# Counter-Balance Controller Analysis

## Scope

This document summarizes the completed matched all-candidate evaluation and
provides a tuning handoff for improving ALMI without losing the FAME benefit.
The evaluation covered:

- Two lower-body policies: FAME and ALMI.
- Seven controller variants.
- One hundred targets in two speed profiles.
- Two hundred trials per controller and 1,400 trials per backend.
- Green/orange/red classification from maximum base roll/pitch drift relative
  to the settled pre-motion reference.
- A separate blue/magenta tracking classification.

The balance threshold was `0.1 rad`. Tracking precision was evaluated with the
configured `0.02 rad` final joint-error threshold.

## Candidate Definitions

| Candidate | Response weights: CoM / momentum / posture | Feedback gains: CoM / gyro / posture | Maximum velocity | Maximum excursion |
|---|---|---|---|---|
| Frame task | No reactive response | No reactive response | n/a | n/a |
| Conservative | `1.0 / 1.0 / 0.05` | `2.0 / 0.1 / 2.0` | `[1.5, 1.8, 1.5, 1.0]` | No explicit overlay limit |
| Gain | `1.8 / 0.5 / 0.02` | `3.0 / 0.05 / 0.75` | `[2.0, 2.5, 2.0, 1.2]` | `[0.65, 0.50, 0.35, 0.50]` |
| Displacement | `2.0 / 0.25 / 0.01` | `2.75 / 0.0 / 0.7` | `[2.2, 2.8, 2.2, 1.3]` | `[0.65, 0.50, 0.35, 0.50]` |
| Aggressive tight | `2.2 / 0.15 / 0.005` | `3.25 / 0.0 / 0.6` | `[2.6, 3.2, 2.6, 1.5]` | `[0.35, 0.28, 0.20, 0.28]` |
| Aggressive nominal | Same aggressive response | Same aggressive response | Same aggressive response | `[0.55, 0.42, 0.30, 0.42]` |
| Aggressive wide | Same aggressive response | Same aggressive response | Same aggressive response | `[0.75, 0.58, 0.40, 0.58]` |

The tight, nominal, and wide variants isolate an **excursion limiter**, not a
motor torque or effort limiter. A future effort-limit experiment must be named
and configured separately.

## Aggregate Results

### FAME

| Candidate | Green | Orange | Red | Mean drift | 95th-percentile drift |
|---|---:|---:|---:|---:|---:|
| Frame task | 194 | 5 | 1 | `0.0386` | `0.0913` |
| Conservative | 193 | 6 | 1 | `0.0374` | `0.0934` |
| Gain | 193 | 7 | 0 | `0.0360` | `0.0944` |
| Displacement | 193 | 7 | 0 | `0.0362` | `0.0942` |
| **Aggressive tight** | **196** | **4** | **0** | `0.0365` | `0.0941` |
| Aggressive nominal | 193 | 6 | 1 | `0.0376` | `0.0930` |
| Aggressive wide | 192 | 7 | 1 | `0.0371` | `0.0934` |

Aggressive tight is the strongest FAME candidate because it produces two paired
improvements and no paired regressions. Mean drift alone would favor gain or
displacement, but those variants regress three previously green target/profile
cells. Outcome preservation has priority over a small mean improvement.

### ALMI

| Candidate | Green | Orange | Red | Mean drift | 95th-percentile drift |
|---|---:|---:|---:|---:|---:|
| **Frame task** | **180** | **20** | **0** | `0.0428` | `0.1409` |
| Conservative | 178 | 22 | 0 | `0.0546` | `0.2019` |
| Gain | 176 | 24 | 0 | `0.0589` | `0.1975` |
| Displacement | 177 | 23 | 0 | `0.0584` | `0.2025` |
| Aggressive tight | 179 | 21 | 0 | `0.0503` | `0.1757` |
| Aggressive nominal | 177 | 23 | 0 | `0.0558` | `0.1935` |
| Aggressive wide | 175 | 25 | 0 | `0.0622` | `0.2084` |

Frame task is the best ALMI result. Every reactive candidate adds regressions
and produces no paired improvement. Aggressive tight is the least harmful
reactive candidate, but it still changes one green cell to orange.

## Exact Paired Changes

The values below are maximum base-orientation drift in radians.

### FAME Improvements And Regressions

| Candidate | Change | Target and profile | Frame drift | Candidate drift |
|---|---|---|---:|---:|
| Conservative | Orange to green | `right_upward_arc_05`, fast | `0.10073` | `0.08955` |
| Conservative | Green to orange | `right_overhang_inner_forward_01`, quasi-static | `0.09575` | `0.10187` |
| Conservative | Green to orange | `right_overhang_inner_forward_02`, quasi-static | `0.08818` | `0.10045` |
| Gain | Orange to green | `right_upward_arc_05`, fast | `0.10073` | `0.07913` |
| Gain | Red to green | `right_overhang_inner_upward_05`, fast | `0.30356` | `0.09565` |
| Gain | Green to orange | `right_overhang_inner_forward_01`, quasi-static | `0.09575` | `0.10143` |
| Gain | Green to orange | `right_overhang_inner_forward_02`, quasi-static | `0.08818` | `0.10016` |
| Gain | Green to orange | `right_overhang_inner_forward_02`, fast | `0.09399` | `0.10331` |
| Displacement | Orange to green | `right_upward_arc_05`, fast | `0.10073` | `0.08612` |
| Displacement | Red to green | `right_overhang_inner_upward_05`, fast | `0.30356` | `0.09369` |
| Displacement | Green to orange | `right_overhang_inner_forward_01`, quasi-static | `0.09575` | `0.10156` |
| Displacement | Green to orange | `right_overhang_inner_forward_02`, quasi-static | `0.08818` | `0.10129` |
| Displacement | Green to orange | `right_overhang_inner_forward_02`, fast | `0.09399` | `0.10248` |
| **Aggressive tight** | **Orange to green** | **`right_upward_arc_05`, fast** | `0.10073` | `0.09337` |
| **Aggressive tight** | **Red to green** | **`right_overhang_inner_upward_05`, fast** | `0.30356` | `0.09748` |
| Aggressive nominal | Orange to green | `right_upward_arc_05`, fast | `0.10073` | `0.08900` |
| Aggressive nominal | Green to orange | `right_overhang_inner_forward_01`, quasi-static | `0.09575` | `0.10037` |
| Aggressive nominal | Green to orange | `right_overhang_inner_forward_02`, fast | `0.09399` | `0.10061` |
| Aggressive wide | Orange to green | `right_upward_arc_05`, fast | `0.10073` | `0.08479` |
| Aggressive wide | Red to green | `right_overhang_inner_upward_05`, fast | `0.30356` | `0.09340` |
| Aggressive wide | Green to orange | `left_overhang_upward_05`, fast | `0.08758` | `0.10760` |
| Aggressive wide | Green to orange | `right_overhang_inner_forward_01`, quasi-static | `0.09575` | `0.10413` |
| Aggressive wide | Green to orange | `right_overhang_inner_forward_02`, quasi-static | `0.08818` | `0.10172` |
| Aggressive wide | Green to orange | `right_overhang_inner_forward_02`, fast | `0.09399` | `0.10161` |
| Aggressive wide | Orange to red | `right_overhang_upward_05`, fast | `0.13943` | `0.30120` |

The progression from tight to wide shows that excursion authority, not only
response gain, controls safety. Wide motion can improve the same two FAME
boundaries while destabilizing five other cells.

### ALMI Regressions

No reactive candidate improves a frame-task classification under ALMI.

| Candidate | Target and profile | Frame drift | Candidate drift |
|---|---|---:|---:|
| Conservative | `left_upward_arc_02`, quasi-static | `0.09568` | `0.10832` |
| Conservative | `left_upward_arc_05`, quasi-static | `0.08848` | `0.11352` |
| Gain | `left_upward_arc_02`, quasi-static | `0.09568` | `0.12564` |
| Gain | `left_upward_arc_04`, quasi-static | `0.06244` | `0.10165` |
| Gain | `left_upward_arc_05`, quasi-static | `0.08848` | `0.12033` |
| Gain | `left_upward_arc_05`, fast | `0.08800` | `0.11542` |
| Displacement | `left_upward_arc_04`, quasi-static | `0.06244` | `0.10265` |
| Displacement | `left_upward_arc_05`, quasi-static | `0.08848` | `0.12096` |
| Displacement | `left_upward_arc_05`, fast | `0.08800` | `0.11795` |
| **Aggressive tight** | **`left_upward_arc_05`, quasi-static** | `0.08848` | `0.10610` |
| Aggressive nominal | `left_upward_arc_02`, fast | `0.08016` | `0.10718` |
| Aggressive nominal | `left_upward_arc_05`, quasi-static | `0.08848` | `0.11842` |
| Aggressive nominal | `left_upward_arc_05`, fast | `0.08800` | `0.11266` |
| Aggressive wide | `left_upward_arc_03`, quasi-static | `0.09792` | `0.10016` |
| Aggressive wide | `left_upward_arc_04`, quasi-static | `0.06244` | `0.10746` |
| Aggressive wide | `left_upward_arc_04`, fast | `0.06370` | `0.10992` |
| Aggressive wide | `left_upward_arc_05`, quasi-static | `0.08848` | `0.12797` |
| Aggressive wide | `left_upward_arc_05`, fast | `0.08800` | `0.12625` |

The ALMI regression is localized to the moving-left-arm upward-arc family. The
counter arm is therefore the right arm. This asymmetry is the primary tuning
target.

## Backend-Level ALMI Boundaries

Two ALMI families are orange for every rank, profile, and controller:

- `left_overhang_forward_01` through `_05`: 70 orange cells.
- `left_overhang_inner_forward_01` through `_05`: 70 orange cells.

These cells include frame task and all reactive variants. They expose a
lower-body-policy or arm-load asymmetry, but do not discriminate the current
counter-balance candidates. Do not tune global reactive gains to these cells;
use them as backend-health sentinels.

## Tracking Precision

Tracking results are identical across lower-body policies:

| Controller type | Precise | Non-converged |
|---|---:|---:|
| Frame task | 194 | 6 |
| Every reactive variant | 196 | 4 |

All variants miss the tracking threshold on `left_cross_body_05` and
`right_cross_body_05` in both profiles. Frame task additionally has large final
errors on `left_overhang_inner_forward_01` fast and
`right_overhang_inner_forward_01` fast. Reactive control closes those two arm
errors, but ALMI base drift remains orange on the left target. This confirms
that improved arm tracking does not imply improved balance.

## Interpretation

1. Tight excursion is the only current FAME candidate with classification gains
   and no regression.
2. Increasing excursion does not improve generality. It amplifies target- and
   side-specific regressions.
3. ALMI already reacts strongly to moving-left-arm trajectories. The current
   counter-arm response likely reinforces or phase-conflicts with the lower-body
   correction.
4. Scalar gain tuning is unlikely to produce one universal candidate. The next
   controller should gate or scale reaction from observed base response.
5. The systematic ALMI left-overhang boundary is not solved by any current arm
   candidate and should be treated separately from candidate ranking.

## Recommended ALMI Tuning Sequence

Start from aggressive tight and keep its excursion envelope fixed.

### 1. Diagnose The ALMI Phase Conflict

For matched frame and aggressive-tight trials on `left_upward_arc_05`, compare:

- Base roll/pitch drift and angular velocity.
- Counter-arm position and velocity by joint.
- CoM offset and support margin.
- Time of peak drift relative to moving-arm acceleration and counter-arm motion.
- Whether the right counter arm initially reduces drift and later overshoots, or
  moves in the wrong direction from onset.

Use both quasi-static and fast profiles. Quasi-static is mandatory because the
only aggressive-tight ALMI regression occurs there.

### 2. Add Feedback-Based Reaction Gating

Prefer measured-response gating over backend-specific constants:

- Attenuate reaction when the observed roll/pitch velocity is already returning
  toward the settled reference.
- Reject or backtrack a counter-arm update whose predicted CoM change and
  measured base response indicate increasing drift.
- Separate sagittal and lateral response scales instead of applying one scalar
  authority to all directions.
- Preserve the tight excursion limit as a hard safety bound.

### 3. Require Cross-Backend Promotion Gates

A candidate may advance only if it:

- Keeps both FAME improvements:
  `right_upward_arc_05` fast and `right_overhang_inner_upward_05` fast.
- Adds no FAME regression at `right_overhang_inner_forward_01` or `_02`.
- Keeps all ALMI left-upward-arc frame-green cells green.
- Introduces no new red or infrastructure result.
- Does not worsen tracking classification.

## Fast Iteration Target Set

### Stage A: Nineteen-Cell Tuning Gate

Use these exact target/profile cells before any broader sweep:

- `left_upward_arc_02` through `_05`, both profiles: 8 cells.
- `right_upward_arc_05`, fast: 1 cell.
- `right_overhang_inner_upward_05`, fast: 1 cell.
- `right_overhang_inner_forward_01`, quasi-static: 1 cell.
- `right_overhang_inner_forward_02`, both profiles: 2 cells.
- `left_overhang_upward_05`, fast: 1 cell.
- `right_overhang_upward_05`, fast: 1 cell.
- `left_overhang_forward_01`, both profiles: 2 backend sentinels.
- `left_overhang_inner_forward_01`, both profiles: 2 backend sentinels.

For all seven controllers and both backends, this is 266 trials instead of
2,800. For one candidate against frame on both backends, it is 76 trials.

### Stage B: Reduced Promotion Sweep

If Stage A passes, use:

- Directional rank 5 for every family and arm.
- `left_upward_arc` ranks 2 through 4 in addition to rank 5.
- Overhang ranks 1, 2, 4, and 5 for every family and arm.
- Both profiles.

This preserves every observed candidate-discriminating rank while omitting:

- Directional rank 1, which was green in all 280 backend/candidate cells.
- Non-upward directional interior ranks that produced no classification change.
- Overhang rank 3, which produced no FAME candidate difference and duplicated
  ALMI's systematic left-overhang outcome.

Do not remove quasi-static globally. It contains most ALMI regressions and key
FAME inner-forward regressions.

## Decision

- Retain aggressive tight as the FAME-leading candidate and next tuning base.
- Keep frame task as the ALMI reference winner.
- Do not promote any reactive candidate as a cross-backend default.
- Tune measured-response gating on left upward-arc cases before another full
  all-candidate sweep.
