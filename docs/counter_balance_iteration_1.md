# Counter-Balance Iteration 1

## Objective

Develop one counter-balance candidate that prevents lower-body stepping during
manipulation while preserving the strongest FAME fall-recovery and drift
improvements. The desired ALMI outcome is stationary double support, not merely
survival after a recovery step.

Promotion requires current ten-second post-motion windows and the stumble-aware
classifier. Historical short-window green, orange, and red outcomes are useful
discovery evidence but are not promotion evidence.

## Evaluation Contract

The outcome severity is stable, drift, stumble, fall, and execution failure.
A stumble is detected when either:

- One ankle moves horizontally more than `0.075 m` from its pre-release
  reference.
- The same ankle moves more than `0.03 m` and rises more than `0.005 m`.

Both quasi-static and fast profiles retain the manipulation target for exactly
ten seconds after commanded arm motion. Falls take precedence over stumbles,
and stumbles take precedence over base-orientation drift.

## Baseline Evidence

The focused current-window run is:

```text
runs/20260818_213501_left_upward_arc_02_almi_10s_review
```

It evaluated `left_upward_arc_02` with seven controllers in both profiles.
All 14 trials survived and tracked precisely, but all 14 stumbled. No safety
estop or runtime error occurred.

| Candidate | Quasi-static displacement / lift | Fast displacement / lift |
|---|---:|---:|
| Frame task | `0.6002 / 0.0233 m` | `0.5264 / 0.0215 m` |
| Conservative | `0.0712 / 0.0097 m` | `0.0648 / 0.0084 m` |
| Gain | `0.0645 / 0.0094 m` | `0.7639 / 0.0262 m` |
| Displacement | `0.0664 / 0.0090 m` | `0.6113 / 0.0262 m` |
| Aggressive tight | `0.0727 / 0.0097 m` | `0.0605 / 0.0090 m` |
| Aggressive nominal | `0.0542 / 0.0091 m` | `0.2124 / 0.0179 m` |
| Aggressive wide | `0.0614 / 0.0102 m` | `0.1305 / 0.0156 m` |

Aggressive tight is the best existing cross-profile compromise on this target.
It substantially reduces frame-task foot translation but still causes a lifted
step in both profiles.

The replacement full ALMI sweep is stored at:

```text
runs/20260819_003453_arm_reachability_all_candidates_almi
```

It completed all 1,400 current-window trials without a fall, estop, runtime
error, or execution failure:

| Candidate | Stable | Drift | Stumble |
|---|---:|---:|---:|
| Frame task | 177 | 19 | 4 |
| Conservative | 176 | 12 | 12 |
| Gain | 175 | 18 | 7 |
| Displacement | 174 | 18 | 8 |
| Aggressive tight | 176 | 21 | 3 |
| Aggressive nominal | 176 | 18 | 6 |
| Aggressive wide | 173 | 19 | 8 |

Aggressive tight has the fewest stumbles but increases drift relative to frame.
No existing candidate dominates the baseline.

## Arm-To-ALMI Signal Path

The safety split gives motors 0 through 11 exclusively to the lower-body policy
and motors 12 through 26 exclusively to the upper controller. ALMI and the arm
controller therefore do not issue competing actuator commands. Their coupling
is physical and observational:

1. Arm motion changes whole-body inertia, CoM, momentum, and measured upper-joint
   state.
2. ALMI observes IMU state and 21 mapped joint positions and velocities.
3. ALMI changes its 12 leg targets in response.
4. The learned policy can select stepping as a recovery strategy.
5. Counter-arm motion changes both physical disturbance and ALMI's upper-body
   observation.

The current reactive controller predicts arm-induced planar CoM and angular
momentum changes with the live model, then solves a bounded four-joint
counter-arm velocity problem. The lower policy remains a separate learned
feedback loop, so a counter-arm response that is mechanically favorable can
still reinforce or phase-conflict with ALMI's learned correction.

## Observation-Mapping Defect

ALMI was trained with 21 joints ordered as 12 legs, torso, four left-arm joints,
and four right-arm joints. The 27-motor robot inserts three left-wrist motors
before the right shoulder. The current contiguous mapping is:

```yaml
observation_joint2motor_idx: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
                              12, 13, 14, 15, 16, 17, 18, 19, 20]
```

Consequently, ALMI receives left-wrist state where it expects most right-arm
state. For left-arm manipulation, this means the right counter arm is mostly
missing or mislabeled in the policy observation. The expected mapping is:

```yaml
observation_joint2motor_idx: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
                              12, 13, 14, 15, 16, 20, 21, 22, 23]
```

This defect must be tested before controller gain tuning. A mapping correction
changes the lower-policy input contract and therefore requires matched baseline
and candidate runs rather than retrospective reclassification.

## Preliminary Retained-Signal Probe

The focused run's `sim.npz` artifacts contain applied actuator control, full
joint state, IMU orientation, support geometry, arm momentum, and ankle poses.
They do not contain ALMI's raw observation or action, so applied leg-control
departure is used only as a policy-response proxy.

Across the 14 `left_upward_arc_02` trials:

- Applied leg control departs from its pre-release mean within approximately
  `0.2` to `0.6 s` after release.
- The simultaneous `0.03 m` translation and `0.005 m` lift event occurs around
  `7.9` to `8.5 s` in quasi-static trials.
- The same event occurs around `5.0` to `6.7 s` in fast trials.
- Frame task steps with only about `0.028 rad` of incidental right-arm joint
  displacement, proving counter-arm motion is not the original stepping trigger.
- Aggressive tight reaches about `0.503 rad` four-joint counter-arm displacement
  norm quasi-static and `0.417 rad` fast. Its peak base response is nearly
  coincident with the lifted-foot event.
- Existing reactive candidates reduce or delay some steps, but none prevents
  ALMI from entering its delayed recovery behavior.

The runtime supplies full reactive authority only through moving-arm motion,
then linearly fades it over `2.5 s`, returns the counter arm toward its nominal
target, and switches to ordinary steady-state arm hold. Quasi-static motion ends
at `6.0 s`; its lifted-foot events near `8.0 s` occur during this return. Fast
motion ends at `1.5 s`, the fade ends at `4.0 s`, and foot events begin after
`5.0 s`. Counter-arm withdrawal is therefore a likely delayed disturbance in
addition to the original arm-induced ALMI response.

The policy reacts well before foot lift, which is consistent with an immediate
upper-observation or arm-induced dynamics response followed by a delayed learned
recovery step. The old six-second fast evaluation could miss late events such as
aggressive wide at approximately `6.7 s`.

## Probe Plan

### Probe 1: Observation Mapping

Run frame task on `left_upward_arc_02` in both profiles with the current and
corrected mappings. Include the right-arm mirror to test side symmetry. Compare:

- Time of first foot lift and placement change.
- Foot displacement and lift by side.
- Base roll/pitch and angular velocity.
- ALMI action change from the settled pre-release mean.
- Leg target change by joint.

The corrected-mapping probe is:

```text
runs/20260819_073156_almi_observation_mapping_probe
```

| Target/profile | Controller | Current mapping | Corrected mapping |
|---|---|---:|---:|
| Left arc 02, quasi-static | Frame | Stumble, `0.7448 m` | Stable, `0.0095 m` |
| Left arc 02, fast | Frame | Stumble, `0.7788 m` | Stable, `0.0199 m` |
| Left arc 02, quasi-static | Aggressive tight | Stumble, `0.0558 m` | Stumble, `0.0559 m` |
| Left arc 02, fast | Aggressive tight | Stumble, `0.5070 m` | Stable, `0.0235 m` |
| Right arc 02, quasi-static | Frame | Stable | Stable, `0.0319 m` |
| Right arc 02, fast | Frame | Stable | Stable, `0.0235 m` |

Correct mapping removes the severe frame-task left-arm asymmetry and makes
aggressive tight stable in fast motion. The remaining aggressive-tight
quasi-static step is controller-specific rather than a general ALMI failure.

### Probe 2: Policy Diagnostics

Extend ALMI logs with the mapped joint positions and velocities, complete
65-element observation, raw action, clipped action, and final leg target. Keep
release-relative timestamps so policy response can be aligned with arm
acceleration, counter-arm motion, foot lift, and base response.

### Probe 3: Upper-Observation Ablation

Use simulation-only ablations to isolate the stepping trigger:

- Hold all upper-body observation channels at their settled nominal values.
- Supply actual moving-arm channels and nominal counter-arm channels.
- Supply nominal moving-arm channels and actual counter-arm channels.
- Hold the settled ALMI action briefly through arm motion as a causal probe.

These are diagnostic experiments, not candidate controller designs.

### Probe 4: Counter-Arm Phase

After correcting the observation contract, compare frame task and aggressive
tight. Determine whether the counter arm initially reduces base response and
later overshoots, or moves in the wrong direction from onset. Inspect response
separately in sagittal and lateral axes.

## Candidate Design Hypotheses

Candidate work begins only after the probes establish the causal mechanism.
The initial hypotheses are:

1. Correct observation mapping may allow ALMI to account for counter-arm motion
   without stepping.
2. A measured-response gate may attenuate counter-arm authority when base motion
   is already returning toward the settled reference.
3. Separate sagittal and lateral authority may avoid reinforcing ALMI's lateral
   recovery response.
4. A no-step manipulation mode may require filtering upper-body observation or
   rate-limiting lower-policy action changes, rather than more counter-arm gain.
5. Aggressive tight's excursion envelope should remain the initial hard bound
   because wider excursions caused FAME regressions.
6. A sustained-tight candidate should retain measured counter-balance authority
   through the ten-second manipulation hold instead of withdrawing it after
   `2.5 s`.

## Tilt-Gated Sustained Candidate

The corrected-ALMI promotion gate is:

```text
runs/20260819_083103_almi_gated_sustained_tight_probe
```

Ungated sustained tight stumbles on `left_upward_arc_03` and `_04` in both
profiles. Frame task is stable there, proving that unnecessary counter-arm
motion creates the steps. The gated design:

- Captures settled IMU roll and pitch before motion.
- Remains inactive below `0.07 rad` tilt displacement.
- Ramps linearly to full authority at `0.10 rad`.
- Latches the largest activation for the remaining manipulation hold.
- Retains full right-arm-moving authority for the FAME recovery family.

The gate keeps all arc 2 through 4 cells stable with `0.007` to `0.018 m` foot
displacement. It also retains both arc 5 drift-to-stable improvements at about
`0.095 rad` maximum drift.

An initial symmetric tilt gate failed to recover the FAME
`right_overhang_inner_upward_05` fall because activation occurred too late.
The implemented iteration-1 candidate therefore bypasses the tilt gate for
right-arm-moving manipulation and keeps it for left-arm-moving manipulation.

This is an empirical side-specific workaround, not a general physical contract.
It follows the observed target distribution: the harmful ALMI counter-arm
responses were moving-left-arm cases, while the early FAME recovery case was a
moving-right-arm target. It can fail to generalize to hard left-arm disturbances
or benign right-arm motions. Iteration 2 should replace the bypass with a
side-independent activation defined by the maximum of predicted manipulation
risk and measured base response.

## Sustained-Tight Probe

The initial sustained-response probe uses corrected ALMI mapping and the
aggressive-tight controller with an effectively constant balance scale through
the post-motion window:

```text
runs/20260819_074130_almi_sustained_tight_probe
```

| Profile | Frame displacement | Aggressive tight | Sustained tight |
|---|---:|---:|---:|
| Quasi-static | `0.0078 m` | `0.0288 m` | `0.0140 m` |
| Fast | `0.0097 m` | `0.0237 m` | `0.0117 m` |

All six trials are stable. Sustained tight approximately halves aggressive
tight's foot displacement in both profiles. Its maximum drift is `0.0603 rad`
quasi-static and `0.0620 rad` fast, safely below the `0.1 rad` threshold.

The mapping probe and sustained probe disagree on one aggressive-tight
quasi-static outcome, so the boundary is stochastic. Repeated matched trials are
required before promotion.

Three additional matched repeats are stored at:

```text
runs/20260819_074802_almi_sustained_tight_repeat_1
runs/20260819_074909_almi_sustained_tight_repeat_2
runs/20260819_075016_almi_sustained_tight_repeat_3
```

Sustained tight remains stable in all six repeated profile cells, with
`0.0093` to `0.0160 m` displacement and less than `0.0007 m` lift. Ordinary
aggressive tight stumbles in three accumulated repeated cells. Including the
initial and implemented-candidate probes, sustained tight is stable in all eight
profile trials.

The implemented candidate uses an explicit `reactive_hold_duration: 10.0`
rather than the probe's long-fade approximation. Its validation run is:

```text
runs/20260819_075508_almi_sustained_tight_probe
```

It reproduces stable feet at `0.0126 m` quasi-static and `0.0148 m` fast.

## FAME Preservation Gates

A candidate must retain the aggressive-tight improvements under the current
ten-second window:

- `right_upward_arc_05`, fast: frame drift to candidate stable.
- `right_overhang_inner_upward_05`, fast: frame fall to candidate stable or
  drift without fall.

It must not regress these known boundaries:

- `right_overhang_inner_forward_01`, quasi-static.
- `right_overhang_inner_forward_02`, both profiles.
- `left_overhang_upward_05`, fast.
- `right_overhang_upward_05`, fast.

The primary current-window FAME probe is:

```text
runs/20260819_080120_fame_sustained_tight_hard_probe
```

| Target/profile | Frame | Aggressive tight | Sustained tight |
|---|---:|---:|---:|
| `right_upward_arc_05`, fast | Stable, `0.0996 rad` | Stable, `0.0956 rad` | Stable, `0.0959 rad` |
| `right_overhang_inner_upward_05`, fast | Fall, `0.3038 rad` | Stable, `0.0933 rad` | Stable, `0.0942 rad` |

Sustained tight preserves the aggressive-tight boundary improvement and hard
fall recovery. The wider no-regression gate is:

```text
runs/20260819_080746_fame_sustained_tight_regression_gate
```

All frame-stable cells remain stable. Sustained tight also changes
`right_overhang_inner_forward_01` fast from aggressive-tight drift
(`0.1015 rad`) to stable (`0.0998 rad`). The two `right_overhang_upward_05`
profiles remain drift for frame and both candidates; sustained tight introduces
no classification regression in the gate.

The side-aware gated hard retest is:

```text
runs/20260819_084502_fame_sustained_tight_hard_probe
```

It keeps `right_upward_arc_05` stable at `0.0929 rad` and recovers the frame fall
on `right_overhang_inner_upward_05` to stable at `0.0943 rad`.

The final side-aware regression gate is:

```text
runs/20260819_085036_fame_sustained_tight_regression_gate
```

All but one frame-stable cell remain stable. The remaining
`right_overhang_inner_forward_01` fast cell is a stochastic threshold boundary:
frame and reactive outcomes vary between approximately `0.097` and `0.102 rad`
across repeats. A right-authority scale probe at:

```text
runs/20260819_085900_fame_gated_sustained_right_scale_probe
```

shows no monotonic scale relationship. Scales `0.6`, `0.8`, and `1.0` all recover
the hard fall; `1.0` produces the best sampled inner-forward drift. The final
candidate therefore retains full right-arm-moving authority.

## Final Full Candidate Matrices

### Corrected ALMI Matrix

The final corrected-ALMI matrix is:

```text
runs/20260819_090636_arm_reachability_all_candidates_almi_iteration_1
```

All 1,600 cells completed with no infrastructure failure, fall, or safety estop.
The ALMI observation mapping correction is included in every trial.

| Candidate | Stable | Drift | Stumble | Tracking note |
|---|---:|---:|---:|---|
| Frame task | 194 | 6 | 0 | 200 precise |
| Conservative | 189 | 8 | 3 | 200 precise |
| Gain | 181 | 14 | 5 | 200 precise |
| Displacement | 179 | 14 | 7 | 200 precise |
| Aggressive tight | 190 | 6 | 4 | 200 precise |
| Aggressive nominal | 183 | 11 | 6 | 200 precise |
| Aggressive wide | 177 | 16 | 7 | 200 precise |
| Gated sustained tight | 194 | 6 | 0 | 196 precise, 4 imprecise |

The gated sustained candidate is the only reactive candidate without an ALMI
stumble. It improves both `left_upward_arc_05` profiles from drift to stable,
but regresses both `right_overhang_upward_03` profiles from stable to drift. Its
four tracking misses are rank-5 cross-body targets.

### FAME Matrix With Unified 3x Safety Limits

The final FAME matrix is:

```text
runs/20260819_171758_arm_reachability_all_candidates_fame_iteration_1_3x
```

The physical matrix completed, but an execution outage produced 127 child exits
with status 1 across 18 target/profile blocks. The failures are nearly uniform
across all controller candidates and include no summary artifact, so they are
infrastructure failures rather than physical outcomes. They must be rerun before
treating the raw 1,600-row report as final.

The 182 target/profile cells that completed for every candidate provide a valid
matched comparison:

| Candidate | Stable | Drift | Fall | Improvements | Regressions | Tracking |
|---|---:|---:|---:|---:|---:|---|
| Frame task | 178 | 3 | 1 | Baseline | Baseline | 182 precise |
| Conservative | 177 | 5 | 0 | 1 | 2 | 182 precise |
| Gain | 174 | 8 | 0 | 1 | 5 | 182 precise |
| Displacement | 175 | 7 | 0 | 1 | 4 | 182 precise |
| Aggressive tight | 179 | 3 | 0 | 1 | 0 | 182 precise |
| Aggressive nominal | 177 | 5 | 0 | 1 | 2 | 182 precise |
| Aggressive wide | 174 | 8 | 0 | 1 | 5 | 182 precise |
| Gated sustained tight | 179 | 3 | 0 | 1 | 0 | 178 precise, 4 imprecise |

Aggressive tight is the narrow FAME-only winner because it ties the best physical
outcome with no regression and has no tracking miss. Gated sustained tight ties
its physical outcome and recovers the frame fall on
`right_overhang_inner_upward_05` from a fall at `0.3035 rad` to stable at `0.0947 rad`,
but it retains four cross-body tracking misses.

## Candidate Conclusions And Failed Designs

- Frame task: strongest passive reference after the ALMI observation correction.
  It has no ALMI stumble, but no active fall recovery.
- Conservative reactive: recovers the FAME hard fall but has three ALMI stumbles
  and two FAME regressions.
- Gain: recovers the FAME hard fall, but has five ALMI stumbles and five FAME
  regressions. It is superseded.
- Displacement: gives the lowest sampled FAME hard-fall drift, but causes seven
  ALMI stumbles and four FAME regressions. It is superseded.
- Aggressive tight: strongest fully symmetric existing candidate. It has four
  ALMI stumbles, but is the best FAME-only candidate with no matched regression
  and all precise tracking.
- Aggressive nominal: wider excursion than tight without better results: six
  ALMI stumbles and two FAME regressions.
- Aggressive wide: largest envelope and weakest trade-off: seven ALMI stumbles
  and five FAME regressions.
- Gated sustained tight: best overall iteration-1 candidate. It eliminates ALMI
  reactive stumbles and preserves FAME hard fall recovery, but uses the ad hoc
  right-arm bypass and has four tracking misses.

Failed attempts were informative:

- The original ALMI observation mapping omitted or mislabeled right-arm state.
  Correcting it removed severe frame-task stepping for left-arm manipulation.
- Sustained tight without activation solved the rank-2 target but created steps
  on ALMI upward arcs 3 and 4 where frame task was stable.
- A symmetric tilt-only gate removed those ALMI steps but activated too late to
  recover the FAME inner-upward hard fall.
- Right-arm activation scales of `0.6`, `0.8`, and `1.0` did not yield a
  monotonic FAME boundary improvement; full right-arm authority was retained.
- The first full FAME iteration-1 checkpoint was discarded because concurrent
  execution caused resource failures and mixed historical safety snapshots.

## Experiment Log

| Iteration | Change | ALMI `left_upward_arc_02` | FAME preservation | Decision |
|---|---|---|---|---|
| Baseline | Seven existing candidates | All 14 stumbled | Aggressive tight had historical gains | Probe lower-policy interaction |
| Mapping probe | Correct right-arm observation indices | Frame stable in both profiles; tight fast stable | Not applicable | Adopt mapping fix, isolate tight quasi-static |
| Sustained tight 0 | Retain tight response through hold | Stable in both profiles; `0.0140 / 0.0117 m` displacement | Pending | Repeat for robustness |
| Sustained tight 1 | Three matched repetitions | Stable in all six cells; `0.0093–0.0160 m` | Hard recovery preserved | Run no-regression gate |
| Sustained tight 2 | Explicit ten-second reactive hold | Stable at `0.0126 / 0.0148 m` | No gate regression; one tight drift becomes stable | Run ALMI promotion gate |
| Gated sustained 0 | Symmetric tilt activation | Arcs 2–5 stable | Hard FAME fall not recovered | Preserve early right-arm response |
| Gated sustained 1 | Tilt gate for left motion; full right response | All tested ALMI arcs stable | Hard recovery preserved; boundary noise remains | Promote to full sweep |

## Historical Iteration-1 Candidate

The following candidate records the best iteration-1 empirical result. It is
retired from runnable configurations because its side-specific activation bypass
does not represent a general physical controller. Future controller comparisons
must use symmetric logic and parameters for left- and right-moving-arm tasks.

`reactive_counter_balance_gated_sustained_tight` uses:

```yaml
reactive_counter_balance:
    weights:
        com: 2.2
        momentum: 0.15
        posture: 0.005
    gains:
        com: 3.25
        gyro: 0.0
        posture: 0.6
    max_velocity: [2.6, 3.2, 2.6, 1.5]
    max_excursion: [0.35, 0.28, 0.20, 0.28]
    activation:
        tilt_threshold: 0.07
        tilt_full_scale: 0.10
        latch: true
        always_active_moving_arms: [right]
        always_active_scale: 1.0
arm_target:
    reactive_hold_duration: 10.0
    reactive_fade_duration: 2.5
```

The ALMI observation map is corrected to motors 0 through 16 followed by motors
20 through 23. This was the best iteration-1 empirical result, but not a final
general architecture. The asymmetric bypass is retained here only to explain the
historical evidence and is no longer supported by executable controller code.
