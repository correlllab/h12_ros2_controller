# Iteration 6C: Final Challenge-Speed Analysis

Status: **COMPLETE**. This restored report uses the preserved final numerical
exports, not the accidentally restored Stage-1 draft. No new experiment was run
to restore this document.

## Question And Result

Does frozen robust 3C expand the manipulation-speed stability envelope relative
to Frame? The result is policy-specific:

- **FAME:** repeatable nonfall protection and observed speed-boundary improvements
    on important targets. The benefit does not grow monotonically with speed.
- **Reproduction:** 26 of the 39 changed discovery cells reproduce their signed
    transition in all three fresh paired repetitions, and 35 of 39 in at least
    two. Thirteen cells are not 3/3 consistent, so no global all-changes-reproduce
    claim is made. See the three-repetition reproduction audit below.
- **ALMI-Manip-v2:** no observed nonfall separation through gamma=2.5, and confirmed
    fastest-duration stable-to-drift regressions. A general increasing advantage
    with speed is not supported.
- **Paper scope:** the evidence supports a bounded, policy-specific FAME
    commanded-speed robustness result, not a universal improvement across policies
    or a certified hardware/physical-speed envelope.

## Frozen Setup And Accounting

The comparison is `frame_task` versus `counter_ddp_velocity_robust`, with unchanged
FAME and corrected `mjlab_almi_manip_2` policies. All 40 unique challenge geometries
use alpha=1.0. No H2 evaluation, payload, target redesign, controller tuning or
selected-policy change was made.

| Evidence | Runs | Interpretation |
| --- | ---: | --- |
| Historical nominal T=1.5 | 160 | Compatible context, not fresh repetitions |
| Discovery T=1.25, 1.0, 0.8, 0.6 | 640 | Full 40-target panels for both policies/controllers |
| Focused confirmation | 516 | 86 comparisons, three fresh paired repetitions |
| Additional headline repetitions | 16 | Repetitions four and five for four cases |
| Unique fresh simulations | 1172 | Headline summaries reuse 24 confirmation observations |

The [design](counter_balance_iteration_6c.md) retains the execution contract.
The actual endpoint reference is a cubic from measured release position, with
zero commanded endpoint velocities, and exact peaks `1.5*abs(delta_q)/T` and
`6*abs(delta_q)/T^2`. Admission checked model/publication position and velocity
bounds and sampled release-path collision validity. Acceleration was reported,
not certified using an invented or straight-line-bank ceiling. The unchanged
actuator-force caps apply in simulation; measured tracking remains a diagnostic.

Physical classification is primary: stable, drift, stumble where applicable, fall.
FAME and ALMI retain their original policy-specific stumble semantics. Endpoint
precision, command delivery, solver/runtime health, and motion achievement remain
separate. A surviving robot with unverified or incomplete manipulation is not
automatically qualified achieved-speed evidence.

## Full Discovery Results

Each row covers all 40 targets. S/D/F means stable/drift/fall; no stumbles occurred.
Gamma is `1.5/T`. Improvements and regressions are paired class transitions. The
nominal rows are historical. No repeated speeds are treated as independent
significance samples.

| Policy | T (s) | Gamma | Frame S/D/F | 3C S/D/F | Improvements | Regressions | Unchanged |
| --- | --- | --- | --- | --- | --- | --- | --- |
| FAME | 1.5 historical | 1 | 24/4/12 | 26/7/7 | 6 | 0 | 34 |
| FAME | 1.25 | 1.2 | 22/5/13 | 23/9/8 | 6 | 0 | 34 |
| FAME | 1 | 1.5 | 22/5/13 | 23/8/9 | 5 | 0 | 35 |
| FAME | 0.8 | 1.875 | 21/6/13 | 22/9/9 | 5 | 0 | 35 |
| FAME | 0.6 | 2.5 | 19/5/16 | 23/6/11 | 8 | 0 | 32 |
| ALMI-Manip-v2 | 1.5 historical | 1 | 37/3/0 | 38/2/0 | 1 | 0 | 39 |
| ALMI-Manip-v2 | 1.25 | 1.2 | 37/3/0 | 36/4/0 | 1 | 2 | 37 |
| ALMI-Manip-v2 | 1 | 1.5 | 37/3/0 | 39/1/0 | 2 | 0 | 38 |
| ALMI-Manip-v2 | 0.8 | 1.875 | 37/3/0 | 39/1/0 | 2 | 0 | 38 |
| ALMI-Manip-v2 | 0.6 | 2.5 | 38/2/0 | 37/3/0 | 0 | 1 | 39 |

FAME's fall-count advantage is 5, 5, 4, 4, 5 targets across these durations.
ALMI's stable-category effect changes sign and its nonfall result never separates.
The hypothesis that faster motion necessarily makes counter-balance increasingly
effective is therefore not supported as a general conclusion.

## All Target Envelopes

These are raw physical **observed maxima**, not continuous safe thresholds. `NF`
means nonfall; `R` is right-censoring at the fastest tested duration; `N` marks
nonmonotonic success as duration decreases; `null` means no observed success, not
zero speed. Historical nominal contributes to these descriptive maxima.

Qualified Common lists the fresh durations with common qualified coverage:
`a=1.25`, `b=1.0`, `c=0.8`, `d=0.6`. Qualification additionally considers motion,
precision, controller completion and observed publication/delivery. Physical falls
remain failures rather than being erased for imprecision. Missing qualification
does not change the recorded physical label. The full CSV retains durations,
exclusions, coverage and both raw and qualified values.

### FAME

| Target | Frame NF | 3C NF | Frame Stable | 3C Stable | Qualified Common |
| --- | --- | --- | --- | --- | --- |
| left_arm_forward | 2.5R | 2.5R | 2.5R | 2.5R | abd |
| left_arm_forward_yaw | 2.5R | 2.5R | 2.5R | 2.5R | ab |
| left_arm_overhead | 2.5R | 2.5R | null | 1 | b |
| left_cross_body_rank6 | 2.5R | 2.5R | 1.875 | 2.5R | c |
| left_diagonal_rank6 | 2.5R | 2.5R | 2.5R | 2.5R | abcd |
| left_extended_down_forward_01 | 2.5R | 2.5R | 2.5R | 2.5R | abd |
| left_extended_down_outward_02 | 2.5R | 2.5R | 2.5R | 2.5R | ab |
| left_extended_down_rear_03 | null | null | null | null | abcd |
| left_extended_down_rear_04 | null | null | null | null | abcd |
| left_extended_horizontal_forward_01 | 2.5R | 2.5R | 2.5R | 2.5R | cd |
| left_extended_horizontal_outward_02 | 2.5R | 2.5R | 2.5R | 2.5R | d |
| left_extended_horizontal_rear_03 | null | null | null | null | abcd |
| left_extended_horizontal_rear_04 | null | 1.2 | null | null | abcd |
| left_extended_up_forward_01 | 2.5R | 2.5R | 2.5R | 2.5R | abcd |
| left_extended_up_outward_02 | 2.5R | 2.5R | 2.5R | 2.5R | abcd |
| left_extended_up_rear_03 | null | 2.5R | null | 1 | abcd |
| left_fast_fall_search_04_scale_76 | 1.875 | 2.5R | 1 | 1 | bc |
| left_fast_fall_search_09_scale_78 | 1.875 | 2.5R | 1.2 | 2.5RN | bc |
| left_fast_fall_search_11_scale_78 | 2.5R | 2.5R | 1.875N | 2.5R | bc |
| left_forward_outward_high_rank6 | 2.5R | 2.5R | 2.5R | 2.5R | cd |
| right_arm_forward | 2.5R | 2.5R | 2.5R | 2.5R | cd |
| right_arm_forward_yaw | 2.5R | 2.5R | 2.5R | 2.5R | bc |
| right_arm_overhead | 2.5R | 2.5R | null | null | bc |
| right_cross_body_rank6 | 2.5R | 2.5R | 2.5R | 2.5R | d |
| right_diagonal_rank6 | 2.5R | 2.5R | 1.5 | 2.5R | c |
| right_extended_down_forward_01 | 2.5R | 2.5R | 2.5R | 2.5R | cd |
| right_extended_down_outward_02 | 2.5R | 2.5R | 2.5R | 2.5R | ad |
| right_extended_down_rear_03 | null | null | null | null | abcd |
| right_extended_down_rear_04 | null | null | null | null | abcd |
| right_extended_horizontal_forward_01 | 2.5R | 2.5R | 2.5R | 2.5R | abcd |
| right_extended_horizontal_outward_02 | 2.5R | 2.5R | 2.5R | 2.5R | ad |
| right_extended_horizontal_rear_03 | null | null | null | null | abcd |
| right_extended_horizontal_rear_04 | null | null | null | null | abcd |
| right_extended_up_forward_01 | 2.5R | 2.5R | 2.5R | 2.5R | abcd |
| right_extended_up_outward_02 | 2.5R | 2.5R | 2.5R | 2.5R | abd |
| right_extended_up_rear_03 | null | 1 | null | null | abcd |
| right_fast_fall_search_04_scale_76 | 1 | 2.5R | null | null | ad |
| right_fast_fall_search_09_scale_78 | null | 1.875 | null | null | acd |
| right_fast_fall_search_11_scale_78 | null | 1.875 | null | null | cd |
| right_forward_outward_high_rank6 | 1.875 | 2.5R | null | null | cd |

### ALMI-Manip-v2

| Target | Frame NF | 3C NF | Frame Stable | 3C Stable | Qualified Common |
| --- | --- | --- | --- | --- | --- |
| left_arm_forward | 2.5R | 2.5R | 2.5R | 2.5R | a |
| left_arm_forward_yaw | 2.5R | 2.5R | 2.5R | 2.5R | abc |
| left_arm_overhead | 2.5R | 2.5R | 2.5R | 2.5R | acd |
| left_cross_body_rank6 | 2.5R | 2.5R | 2.5R | 2.5R | bcd |
| left_diagonal_rank6 | 2.5R | 2.5R | 2.5R | 2.5R | abcd |
| left_extended_down_forward_01 | 2.5R | 2.5R | 2.5R | 2.5R | bd |
| left_extended_down_outward_02 | 2.5R | 2.5R | 2.5R | 2.5R | abc |
| left_extended_down_rear_03 | 2.5R | 2.5R | 2.5R | 2.5R | c |
| left_extended_down_rear_04 | 2.5R | 2.5R | 2.5R | 2.5R | bcd |
| left_extended_horizontal_forward_01 | 2.5R | 2.5R | 2.5R | 2.5R | ab |
| left_extended_horizontal_outward_02 | 2.5R | 2.5R | 2.5R | 2.5R | ac |
| left_extended_horizontal_rear_03 | 2.5R | 2.5R | 2.5R | 2.5R | abd |
| left_extended_horizontal_rear_04 | 2.5R | 2.5R | 2.5R | 2.5R | bcd |
| left_extended_up_forward_01 | 2.5R | 2.5R | 2.5R | 2.5R | cd |
| left_extended_up_outward_02 | 2.5R | 2.5R | 2.5R | 2.5R | cd |
| left_extended_up_rear_03 | 2.5R | 2.5R | 2.5R | 2.5R | d |
| left_fast_fall_search_04_scale_76 | 2.5R | 2.5R | 2.5R | 2.5R | bcd |
| left_fast_fall_search_09_scale_78 | 2.5R | 2.5R | 2.5R | 2.5R | bcd |
| left_fast_fall_search_11_scale_78 | 2.5R | 2.5R | 2.5R | 2.5R | c |
| left_forward_outward_high_rank6 | 2.5R | 2.5R | 2.5R | 2.5RN | bd |
| right_arm_forward | 2.5R | 2.5R | 2.5R | 2.5R | abd |
| right_arm_forward_yaw | 2.5R | 2.5R | 2.5R | 2.5R | acd |
| right_arm_overhead | 2.5R | 2.5R | 2.5R | 2.5R | bcd |
| right_cross_body_rank6 | 2.5R | 2.5R | 2.5R | 2.5R | d |
| right_diagonal_rank6 | 2.5R | 2.5R | 2.5R | 2.5R | ac |
| right_extended_down_forward_01 | 2.5R | 2.5R | 2.5R | 2.5R | abcd |
| right_extended_down_outward_02 | 2.5R | 2.5R | 2.5R | 2.5R | acd |
| right_extended_down_rear_03 | 2.5R | 2.5R | null | 1.875N | b |
| right_extended_down_rear_04 | 2.5R | 2.5R | null | null | b |
| right_extended_horizontal_forward_01 | 2.5R | 2.5R | 2.5R | 2.5R | abcd |
| right_extended_horizontal_outward_02 | 2.5R | 2.5R | 2.5R | 2.5R | acd |
| right_extended_horizontal_rear_03 | 2.5R | 2.5R | 2.5RN | 1.875 | c |
| right_extended_horizontal_rear_04 | 2.5R | 2.5R | 2.5R | 2.5RN | abd |
| right_extended_up_forward_01 | 2.5R | 2.5R | 2.5R | 2.5R | ab |
| right_extended_up_outward_02 | 2.5R | 2.5R | 2.5R | 2.5R | abc |
| right_extended_up_rear_03 | 2.5R | 2.5R | 2.5R | 2.5R | ab |
| right_fast_fall_search_04_scale_76 | 2.5R | 2.5R | 2.5R | 2.5R | acd |
| right_fast_fall_search_09_scale_78 | 2.5R | 2.5R | 2.5R | 2.5R | bc |
| right_fast_fall_search_11_scale_78 | 2.5R | 2.5R | 2.5R | 2.5R | abc |
| right_forward_outward_high_rank6 | 2.5R | 2.5R | 2.5R | 2.5R | acd |

## Fresh Confirmation

The focused set retained every observed signed disagreement, adjacent class
boundary, existing near-threshold candidate and genuine solver-health ambiguity.
It contained 86 policy/target/duration comparisons: 63 FAME and 23 ALMI. The
existing threshold selection band included precision as well as physical metrics;
it did not change classification thresholds. Both controllers ran three fresh,
rotated interleaved repetitions. No p-value stopping rule was used.

| Policy | Selected Paired Repetitions | Improving | Regressing | Unchanged |
| --- | ---: | ---: | ---: | ---: |
| FAME | 189 | 81 | 1 | 107 |
| ALMI-Manip-v2 | 69 | 13 | 11 | 45 |

These selected repeated observations are not a new full-panel average or
independent speed samples. Four finite FAME nonfall differences repeat in all
three fresh boundary comparisons:

| Target | Frame Fastest Observed Nonfall T | 3C Fastest Observed Nonfall T | Gamma Frame -> 3C |
| --- | --- | --- | --- |
| `right_fast_fall_search_04_scale_76` | 1.25 s | 0.60 s | 1.2 -> 2.5R |
| `left_fast_fall_search_04_scale_76` | 0.80 s | 0.60 s | 1.875 -> 2.5R |
| `left_fast_fall_search_09_scale_78` | 0.80 s | 0.60 s | 1.875 -> 2.5R |
| `right_forward_outward_high_rank6` | 0.80 s | 0.60 s | 1.875 -> 2.5R |

The first target's Frame maximum is 1.2 in fresh confirmation, not the discovery
estimate of 1.0. Nonmonotonic stable outcomes remain explicit. The confirmation
subset does not retest every duration of every target; the boundary CSV states
the actual common tested durations for each repetition.

## Three-Repetition Reproduction Audit

Every baseline-versus-3C outcome change reported in the Full Discovery Results
table was matched against its three fresh paired confirmation repetitions. This
audit covers all changed cells, not only the headline cases.

The changed set is 39 policy/target/duration cells: 30 FAME and 9 ALMI-Manip-v2.
That is exactly the improvement and regression counts of the discovery table,
historical nominal rows included. All 39 fall inside the 86-comparison
confirmation set, and each has exactly three paired repetitions; none is missing
or partially covered. The audit reads the 516 per-repetition confirmation records
and their `summary.json` physical outcomes directly, one comparison at a time. It
does not infer agreement from the improving/regressing totals above. Re-deriving
the fall flag independently from each `summary.json` agrees with the recorded
per-run classification in 516/516 runs.

A repetition **reproduces** a cell when its signed class transition has the same
sign as the discovery transition: an improvement stays an improvement, a
regression stays a regression. Exact class-pair agreement is stricter and is
reported separately. Classes are abbreviated S/D/F for stable/drift/fall.

| Audit Result | Cells |
| --- | ---: |
| Changed discovery cells audited | 39 |
| Cells with three located paired repetitions | 39 |
| Reproduced in **3/3** repetitions | 26 |
| Exact class pair in 3/3 repetitions | 25 |
| Reproduced in at least 2/3 repetitions | 35 |
| Not 3/3 consistent | 13 |

**The global 3/3 claim is therefore not supported and is not made.** Thirteen of
the 39 changed cells do not reproduce their discovery transition in all three
paired repetitions.

| Policy | Target | T (s) | Discovery | Rep 1 | Rep 2 | Rep 3 | Reproduced |
| --- | --- | ---: | --- | --- | --- | --- | --- |
| FAME | `left_arm_overhead` | 1.5 | D->S | D->S | D->D | D->S | 2/3 |
| FAME | `left_cross_body_rank6` | 0.6 | D->S | D->S | D->S | D->S | **3/3** |
| FAME | `left_extended_horizontal_rear_04` | 1.5 | F->D | F->D | F->D | F->D | **3/3** |
| FAME | `left_extended_horizontal_rear_04` | 1.25 | F->D | F->D | F->D | F->F | 2/3 |
| FAME | `left_extended_up_rear_03` | 1.5 | F->S | F->S | F->S | F->S | **3/3** |
| FAME | `left_extended_up_rear_03` | 1.25 | F->D | F->D | F->S | F->D | **3/3** |
| FAME | `left_extended_up_rear_03` | 1 | F->D | F->D | F->D | F->D | **3/3** |
| FAME | `left_extended_up_rear_03` | 0.8 | F->D | F->D | F->D | F->D | **3/3** |
| FAME | `left_extended_up_rear_03` | 0.6 | F->D | F->D | F->D | F->D | **3/3** |
| FAME | `left_fast_fall_search_04_scale_76` | 0.6 | F->D | F->D | F->D | F->D | **3/3** |
| FAME | `left_fast_fall_search_09_scale_78` | 1 | D->S | D->S | D->S | D->S | **3/3** |
| FAME | `left_fast_fall_search_09_scale_78` | 0.6 | F->S | F->S | F->S | F->S | **3/3** |
| FAME | `left_fast_fall_search_11_scale_78` | 1.25 | D->S | S->S | S->D | S->S | 0/3 |
| FAME | `left_fast_fall_search_11_scale_78` | 0.6 | D->S | D->S | D->S | D->S | **3/3** |
| FAME | `right_diagonal_rank6` | 0.8 | D->S | D->S | S->S | D->S | 2/3 |
| FAME | `right_diagonal_rank6` | 0.6 | D->S | D->S | D->S | D->S | **3/3** |
| FAME | `right_extended_up_rear_03` | 1.5 | F->D | F->D | F->D | F->D | **3/3** |
| FAME | `right_fast_fall_search_04_scale_76` | 1.25 | F->D | D->D | D->D | D->D | 0/3 |
| FAME | `right_fast_fall_search_04_scale_76` | 1 | F->D | F->D | F->D | F->D | **3/3** |
| FAME | `right_fast_fall_search_04_scale_76` | 0.8 | F->D | F->D | F->D | F->D | **3/3** |
| FAME | `right_fast_fall_search_04_scale_76` | 0.6 | F->D | F->D | F->D | F->D | **3/3** |
| FAME | `right_fast_fall_search_09_scale_78` | 1.5 | F->D | D->D | F->D | F->D | 2/3 |
| FAME | `right_fast_fall_search_09_scale_78` | 1.25 | F->D | F->D | F->D | F->D | **3/3** |
| FAME | `right_fast_fall_search_09_scale_78` | 1 | F->D | F->D | F->D | F->D | **3/3** |
| FAME | `right_fast_fall_search_09_scale_78` | 0.8 | F->D | F->D | F->F | F->D | 2/3 |
| FAME | `right_fast_fall_search_11_scale_78` | 1.5 | F->D | F->D | F->D | F->D | **3/3** |
| FAME | `right_fast_fall_search_11_scale_78` | 1.25 | F->D | F->D | F->D | F->D | **3/3** |
| FAME | `right_fast_fall_search_11_scale_78` | 1 | F->D | F->D | F->D | F->D | **3/3** |
| FAME | `right_fast_fall_search_11_scale_78` | 0.8 | F->D | F->D | F->D | F->D | **3/3** |
| FAME | `right_forward_outward_high_rank6` | 0.6 | F->D | F->D | F->D | F->D | **3/3** |
| ALMI | `left_forward_outward_high_rank6` | 1.25 | S->D | S->S | S->S | S->S | 0/3 |
| ALMI | `right_extended_down_rear_03` | 1 | D->S | D->D | D->S | D->S | 2/3 |
| ALMI | `right_extended_down_rear_03` | 0.8 | D->S | D->S | D->S | D->S | **3/3** |
| ALMI | `right_extended_horizontal_rear_03` | 1.5 | D->S | D->S | D->D | D->D | 1/3 |
| ALMI | `right_extended_horizontal_rear_03` | 1.25 | D->S | D->S | D->S | D->S | **3/3** |
| ALMI | `right_extended_horizontal_rear_03` | 1 | D->S | D->S | D->S | D->D | 2/3 |
| ALMI | `right_extended_horizontal_rear_03` | 0.8 | D->S | D->S | S->S | D->S | 2/3 |
| ALMI | `right_extended_horizontal_rear_03` | 0.6 | S->D | S->D | S->D | S->D | **3/3** |
| ALMI | `right_extended_horizontal_rear_04` | 1.25 | S->D | S->S | S->D | S->D | 2/3 |

### Exceptions

These thirteen cells are listed rather than averaged away. `Frame differs` and
`3C differs` count the repetitions whose class on that side differs from the
discovery record, which identifies whether the baseline or the counter-balance
side moved.

| Policy | Target | T (s) | Discovery | Agreement | Frame differs | 3C differs |
| --- | --- | ---: | --- | ---: | ---: | ---: |
| FAME | `left_fast_fall_search_11_scale_78` | 1.25 | D->S | 0/3 | 3 | 1 |
| FAME | `right_fast_fall_search_04_scale_76` | 1.25 | F->D | 0/3 | 3 | 0 |
| ALMI | `left_forward_outward_high_rank6` | 1.25 | S->D | 0/3 | 0 | 3 |
| ALMI | `right_extended_horizontal_rear_03` | 1.5 | D->S | 1/3 | 0 | 2 |
| FAME | `left_arm_overhead` | 1.5 | D->S | 2/3 | 0 | 1 |
| FAME | `left_extended_horizontal_rear_04` | 1.25 | F->D | 2/3 | 0 | 1 |
| FAME | `right_diagonal_rank6` | 0.8 | D->S | 2/3 | 1 | 0 |
| FAME | `right_fast_fall_search_09_scale_78` | 0.8 | F->D | 2/3 | 0 | 1 |
| FAME | `right_fast_fall_search_09_scale_78` | 1.5 | F->D | 2/3 | 1 | 0 |
| ALMI | `right_extended_down_rear_03` | 1.0 | D->S | 2/3 | 0 | 1 |
| ALMI | `right_extended_horizontal_rear_03` | 0.8 | D->S | 2/3 | 1 | 0 |
| ALMI | `right_extended_horizontal_rear_03` | 1.0 | D->S | 2/3 | 0 | 1 |
| ALMI | `right_extended_horizontal_rear_04` | 1.25 | S->D | 2/3 | 0 | 1 |

Three cells reproduce in no repetition, all at T=1.25. In two of them the Frame
baseline itself reclassified, so the discovery transition had no room to appear.
FAME `right_fast_fall_search_04_scale_76` has a non-falling Frame run in all three
repetitions, the same boundary correction already recorded above as a Frame
maximum of gamma 1.2 rather than the discovery estimate of 1.0; its 3C side drifts
in all three, matching discovery. FAME `left_fast_fall_search_11_scale_78` has a
stable Frame run in all three, and its 3C side is stable twice and drifts once, so
that cell also contains one repetition in which 3C is the worse of the pair. The
third, ALMI `left_forward_outward_high_rank6`, is the opposite case: the discovery
`stable->drift` regression does not appear at all, with both controllers stable in
3/3, consistent with the nonmonotonic `2.5RN` marker in the ALMI envelope table.

Nine of the remaining ten exceptions differ in exactly one of their three
repetitions, each on one side only and by one observed class step; no stumbles
occurred, so fall and drift are adjacent here. The fastest tested duration is the
most reproducible: every changed cell at T=0.6 reproduces in 3/3, while the
exceptions concentrate at T=0.8 and slower (3 at 0.8, 2 at 1.0, 5 at 1.25,
3 at 1.5). This does not rescue a general monotonic speed claim. It means the
near-nominal boundary cells are the noisy ones, and both policies contribute
exceptions (7 FAME, 6 ALMI).

The audit consumes only existing artifacts. The changed-cell list is
[`discovery_signed_transitions.csv`](../../../runs/key_findings_reports/iteration6c/reports/final/discovery_signed_transitions.csv),
cross-checked against the 400 rows of
[`discovery_pairs.csv`](../../../runs/key_findings_reports/iteration6c/reports/final/discovery_pairs.csv).
The repetition outcomes come from the per-run rows and `summary.json` files of the
canonical confirmation root
`runs/challenge_sweep/iter6c_repetitions/confirmation`, which
[`provenance.json`](../../../runs/key_findings_reports/iteration6c/reports/final/provenance.json)
records as the confirmation stage root. No simulation was run, no classification
threshold was changed, and no physical label or runtime flag was rewritten for
this audit.

## Five-Repetition Cases

The signed headline rule selected two improvements and two regressions before
fresh repetitions four and five. Existing confirmation repetitions are reused
in this five-pair summary, not counted as new simulations. These four cases are
additional depth on selected cells; the three-repetition audit above does not
rest on them, and their 5/5 agreement is not evidence about the other 35 changed
cells. Two of them, FAME `left_extended_up_rear_03` at T=1.5 and FAME
`left_fast_fall_search_09_scale_78` at T=0.6, are also 3/3 in that audit. The
ALMI `right_extended_down_rear_03` T=0.6 case is a repetition-stage finding:
discovery recorded `drift->drift` at that duration, so the cell is not one of the
39 changed discovery cells, and its `stable->drift` regression rests on the
repetition runs alone.

| Policy | Target | T (s) | Frame -> 3C | Agreement |
| --- | --- | --- | --- | --- |
| FAME | `left_extended_up_rear_03` | 1.5 | Fall -> stable | 5/5 |
| FAME | `left_fast_fall_search_09_scale_78` | 0.6 | Fall -> stable | 5/5 |
| ALMI-Manip-v2 | `right_extended_down_rear_03` | 0.6 | Stable -> drift | 5/5 |
| ALMI-Manip-v2 | `right_extended_horizontal_rear_03` | 0.6 | Stable -> drift | 5/5 |

The ALMI counterexamples are retained. No parameters were changed to repair them.

## Base-Motion Diagnostics

Means below average per-run summary values, not pooled timestamps. Tilt is the
mean of run peaks, not time-mean tilt. RMS drift is orientation drift from the
run's baseline, not RMS absolute tilt. Translation uses the cached pelvis
displacement diagnostic. All-outcome FAME means include falls and consequently
are influenced by different fall counts; they are not an independent causal
estimate of a base-stabilization mechanism.

| Policy | T | Peak Tilt Degrees F / 3C | RMS Drift Degrees F / 3C | Pelvis Translation cm F / 3C |
| --- | --- | --- | --- | --- |
| FAME | 1.5 historical | 62.35 / 39.51 | 27.08 / 17.92 | N/A |
| FAME | 1.25 | 66.56 / 43.34 | 29.99 / 20.34 | 17.25 / 12.34 |
| FAME | 1 | 66.24 / 48.02 | 30.29 / 22.52 | 17.33 / 12.94 |
| FAME | 0.8 | 66.35 / 47.55 | 30.78 / 22.56 | 17.44 / 12.87 |
| FAME | 0.6 | 79.52 / 56.54 | 37.25 / 26.88 | 20.15 / 15.22 |
| ALMI | 1.5 historical | 4.04 / 4.00 | 1.63 / 1.58 | N/A |
| ALMI | 1.25 | 4.01 / 4.38 | 1.60 / 1.63 | 3.72 / 3.57 |
| ALMI | 1 | 3.99 / 3.93 | 1.59 / 1.53 | 3.72 / 3.48 |
| ALMI | 0.8 | 4.00 / 3.95 | 1.60 / 1.51 | 3.76 / 3.46 |
| ALMI | 0.6 | 4.01 / 4.01 | 1.60 / 1.56 | 3.82 / 3.61 |

These metrics can describe local advantages, but should not be selected to
manufacture a general ALMI benefit. At T=1.25 ALMI tilt and RMS drift are worse
with 3C; some smaller translation/CoM diagnostics are favorable at faster T.
The physical regression and mixed policy-level result remain authoritative.

### Measured Mean And RMS Tilt

Actual mean and RMS **absolute gravity tilt** were subsequently reduced offline
from all 800 nominal/discovery recordings. Unlike the cached full-record peaks
above, the comparison below uses the same release-relative interval for each
pair, ending at the earliest of the planned move plus ten-second hold, either
recording's end, or either confirmed fall. Samples at/after that fall are excluded.
This prevents a fallen robot's post-fall posture from dominating the comparison.
No trajectory was rerun, aligned by motion onset, or reclassified.

These are equally weighted means of per-run statistics over 40 matched pairs
per row, in degrees. The common window is outcome-dependent and shorter for
falling pairs; it is descriptive, not an independent causal effect estimate.

| Policy | T (s) | Mean Tilt F / 3C | RMS Tilt F / 3C | Mean Peak F / 3C | 3C Mean Wins |
| --- | --- | --- | --- | --- | --- |
| FAME | 1.5 historical | 5.802 / 5.342 | 6.797 / 6.085 | 14.576 / 10.805 | 27/40 |
| FAME | 1.25 | 5.710 / 5.256 | 6.789 / 6.061 | 15.059 / 11.204 | 24/40 |
| FAME | 1.0 | 5.897 / 5.432 | 6.979 / 6.220 | 15.371 / 11.252 | 29/40 |
| FAME | 0.8 | 5.965 / 5.442 | 7.068 / 6.225 | 15.480 / 11.177 | 30/40 |
| FAME | 0.6 | 6.386 / 5.448 | 7.687 / 6.266 | 17.411 / 11.547 | 30/40 |
| ALMI | 1.5 historical | 1.5647 / 1.5815 | 1.6803 / 1.6900 | 2.8138 / 2.7970 | 18/40 |
| ALMI | 1.25 | 1.5722 / 1.5713 | 1.6835 / 1.6662 | 2.8090 / 2.7376 | 22/40 |
| ALMI | 1.0 | 1.5756 / 1.5645 | 1.6811 / 1.6477 | 2.8076 / 2.6269 | 22/40 |
| ALMI | 0.8 | 1.5870 / 1.5661 | 1.6912 / 1.6495 | 2.8088 / 2.6527 | 22/40 |
| ALMI | 0.6 | 1.5938 / 1.5781 | 1.6969 / 1.6774 | 2.8474 / 2.6939 | 24/40 |

FAME's T=0.6 mean reduction is about 14.69%. ALMI's is about 0.985%, a small
motion-quality benefit, not evidence that its categorical stability regressions
vanish. Historical ALMI mean/RMS tilt are slightly worse with 3C. No significance
test or pooled-speed independence assumption is used for these new reductions.

The [per-run](../../../runs/key_findings_reports/iteration6c/reports/base_motion_per_run.csv),
[paired](../../../runs/key_findings_reports/iteration6c/reports/base_motion_pairs.csv),
and [duration summary](../../../runs/key_findings_reports/iteration6c/reports/base_motion_by_duration.csv)
exports retain both the full move/hold and common pre-fall windows, sample counts,
censoring and unchanged physical outcomes. The methods metadata records targeted
array reads and file-stat checks, not a new full-byte audit of every recording.
Reproduce with `python -m h12_zmp_benchmark.experiment.iteration6c_base_motion`.

## Execution And Interpretation Limits

- All 160 fastest discovery runs touched a moving-joint actuator-force cap.
    This establishes saturation contact, not saturation duty cycle.
- Measured peak speed increases less than commanded gamma. Eventual joint
    excursion and endpoint precision do not prove arrival within shortened T.
- Only 438/640 fresh discovery records satisfy the stricter observability and
    publication qualification. Unknown sampled delivery is not promoted to success.
- One discovery and one confirmation run retain a solver-failure flag and false
    controller-completion flag despite raw physical stability. No physical label
    or runtime flag is rewritten to favor a result.
- Thirteen of the 39 changed discovery cells do not reproduce their signed
    transition in all three paired repetitions. Discovery class labels at a single
    boundary cell are therefore not by themselves repeatable evidence, and the
    audit's per-cell table should be consulted before a cell is cited.
- Nominal runs are compatible historical context, not fresh confirmation.
- Reported envelopes are maxima over finite tested samples, with censoring and
    nonmonotonic outcomes. There is no continuous or hardware-safety certificate.

## Reports And Organization

Use each sweep's **`plots/index.html`** for the established browsable summary,
tables and plot links. Raw ownership is separate from cumulative campaign
provenance: each per-root report describes only that root's physical recordings.

| Root Under `runs/challenge_sweep/` | Own Fresh Runs | HTML Entry |
| --- | ---: | --- |
| `20260912_iter6c_initial_discovery` | 4 | [Open](../../../runs/challenge_sweep/20260912_iter6c_initial_discovery/plots/index.html) |
| `20260912_iter6c_discovery_analysis_v2` | 256 | [Open](../../../runs/challenge_sweep/20260912_iter6c_discovery_analysis_v2/plots/index.html) |
| `20260913_iter6c_speed_discovery_frame_vs_3c` | 380 | [Open](../../../runs/challenge_sweep/20260913_iter6c_speed_discovery_frame_vs_3c/plots/index.html) |
| `iter6c_repetitions/confirmation` | 516 | [Open](../../../runs/challenge_sweep/iter6c_repetitions/confirmation/plots/index.html) |
| `20260913_iter6c_headline_repetitions` | 16 | [Open](../../../runs/challenge_sweep/20260913_iter6c_headline_repetitions/plots/index.html) |

The [central index](../../../runs/key_findings_reports/iteration6c/README.md)
links the combined final reports and existing paired videos. Standard roots
contain configs, runs, summaries, plots, results and paired-comparison CSVs.
There is no compatibility-symlink tree. The relocation changed organization,
not experiment outcomes. No new video rendering was needed for this report repair.

The [combined discovery HTML report](../../../runs/key_findings_reports/iteration6c/discovery_overview/plots/index.html)
contains all 640 discovery runs exactly once and links the additional measured
base-motion tables. Historical nominal rows in that supplemental tilt comparison
are labeled separately and do not inflate the primary discovery report.

- [Complete final exports](../../../runs/key_findings_reports/iteration6c/reports/final/).
- [Run inventory](../../../runs/key_findings_reports/iteration6c/reports/final/run_inventory.csv).
- [Discovery pairs](../../../runs/key_findings_reports/iteration6c/reports/final/discovery_pairs.csv).
- [All target envelopes](../../../runs/key_findings_reports/iteration6c/reports/final/discovery_envelopes.csv).
- [Confirmed boundaries](../../../runs/key_findings_reports/iteration6c/reports/final/confirmation_boundaries.csv).
- [Changed discovery cells](../../../runs/key_findings_reports/iteration6c/reports/final/discovery_signed_transitions.csv), the 39 cells audited against three repetitions.
- [Headline cases](../../../runs/key_findings_reports/iteration6c/reports/final/headline_cells.csv).
- [Preserved paired videos and plots](../../../runs/key_findings_reports/iteration6c/videos/README.md).

The videos show FAME fast fall-to-stable, FAME drift-to-stable, and the same ALMI
target's mixed benefit at T=0.8 versus repeatable regression at T=0.6. They use
the first fresh repetition, fixed-camera release alignment and visible
end-of-recording labels, not favorable-error selection or new physics.

## Final Conclusions

All 39 baseline-versus-3C outcome changes in the discovery sweep were audited
against three fresh paired repetitions each. Twenty-six reproduce the same signed
transition in 3/3 trials and 35 in at least 2/3, but thirteen are not 3/3
consistent, so the claim that all observed baseline--counter-balance outcome
changes were reproduced consistently across three paired repetitions is **not**
supported and is not made. Every changed cell at the fastest tested duration
T=0.6 does reproduce in 3/3; the exceptions sit at T=0.8 and slower and include
both policies.

FAME has repeatable policy-specific protection and observed nonfall boundary
shifts. ALMI has no observed nonfall separation and clear fastest-duration
strict-stability counterexamples. Challenge-speed robustness can be a main
simulation result only with those scope limits, saturation/tracking qualifications
and negative findings explicit. It is not a universal statement that increasing
speed makes counter-balance more effective. Iteration 6C is complete; no further
simulation or controller modification is scheduled here.
