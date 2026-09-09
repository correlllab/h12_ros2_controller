# Counter-Balance Iteration 6A Analysis

## Status

Iteration 6A retains `counter_residual_h2_robust`; no new controller is promoted.
The finalized refresh contains 22 completed sweeps with no pending work.
Its exact broad and repeated evidence is summarized below, separately from
the original 116 physical trials: 111 historical trials and five
robust-H2 classification checks. In that original recheck, target 06 completes **5/5 full-duration
drifts with precise tracking** under the corrected 0.50-rad soft threshold.
The previous 4/5 result was a detector artifact, not demonstrated physical
recovery inconsistency. Established 09/11 evidence remains supported; manual
and distinct overhang limitations remain. No additional independent rescue
geometry or H2-residual advantage over frozen 3C has been established.
See [the 6A design](counter_balance_iteration_6a.md) for the frozen contract,
preregistered comparison panel, and promotion gates.

## Finalized Refresh Results

The [refresh index][refresh] is the current broad evidence source, backed by
the [final campaign JSON][refresh-report], [target-level CSV][refresh-csv],
and [key-video index][refresh-videos]. It replaces deprecated broad-root
numbers rather than layering historical broad tables. No controller,
policy, geometry, gain, residual authority, or lifecycle tuning was performed.

Broad refresh runs live in the dedicated `runs/full_sweep/` and
`runs/challenge_sweep/` directories. Under `runs/key_findings_reports/iteration6a/`,
`benchmark_results.md` indexes them; `reports/benchmark/` and `videos/` hold
refresh reports and videos. Refresh repetitions and health checks live directly
in `focused_repetitions/` and `health/`, separate from the unchanged original
`characterization/`, `classification_recheck/`, `confirmation/`, and `expansion/`.

Each policy/controller has 100 full-catalog plus 44 challenge observations.
The named denominator is 144 IDs but only 140 geometries per policy. Both-side
lateral-high/Boundary-04 and lateral-overhead/Boundary-06 aliases are not new
independent families. Counts below are stable/drift/stumble/fall, from the
final report; all initial latest attempts have zero infrastructure/unknown
classifications.

| Policy | Controller | Full 100 | Challenge 44 | Total 144 |
| --- | --- | --- | --- | --- |
| FAME | Frame | 96/3/0/1 | 24/12/0/8 | 120/15/0/9 |
| FAME | Frozen 3C | 95/4/0/1 | 28/12/0/4 | 123/16/0/5 |
| FAME | Robust H2 | 94/5/0/1 | 28/12/0/4 | 122/17/0/5 |
| ALMI v2 | Frame | 100/0/0/0 | 44/0/0/0 | 144/0/0/0 |
| ALMI v2 | Frozen 3C | 100/0/0/0 | 44/0/0/0 | 144/0/0/0 |
| ALMI v2 | Robust H2 | 100/0/0/0 | 44/0/0/0 | 144/0/0/0 |

All target-level transitions and metric differences are linked in the refresh
index, which also tabulates every initial FAME category change. Initially,
Frame-to-3C has 10 improvements/3 regressions/131 unchanged, Frame-to-H2 has
10/4/130, and 3C-to-H2 has 1/2/141. ALMI has no category changes. These are
named-target observations, not residual-only rescue counts.

### Focused Confirmation and Attribution

FAME has 54 three-repetition screen cells, with 11 extended to five by the
completed `confirm_r4/r5`: 184 trials per controller. ALMI has three screen
cells, two extended to five: 13 per controller. Do not count all screened
cells as five-run confirmations or add the initial screen to their denominators.
Eight health sweeps remain separate. There are no classified stumbles.

| Focused policy | Controller | Stable | Drift | Fall | Precise tracking |
| --- | --- | ---: | ---: | ---: | ---: |
| FAME | Frame | 126 | 26 | 32 | 156/184 |
| FAME | Frozen 3C | 138 | 30 | 16 | 178/184 |
| FAME | Robust H2 | 141 | 27 | 16 | 177/184 |
| ALMI v2 | Frame | 13 | 0 | 0 | 13/13 |
| ALMI v2 | Frozen 3C | 13 | 0 | 0 | 13/13 |
| ALMI v2 | Robust H2 | 13 | 0 | 0 | 13/13 |

Right 06/09/11 each gives Frame fall **5/5**, 3C drift **5/5**, and robust-H2
drift **5/5**, with precise tracking in both counter controllers. These fresh,
matched corrected-detector runs close the old Frame/3C/robust-H2 comparison
gap, not a frozen-H2 comparison or causal residual ablation. No H2 residual
credit is warranted where 3C gives the same rescue.

The full eleven-cell confirmation table is in the refresh index. Its other
conclusions are not uniformly favorable:

- **Shared regressions:** right inner-forward 01/02 each gives Frame stable
    5/5 versus both counter controllers drift 5/5.
- **Unreliable additional rescue:** right inner-upward 05 gives Frame fall 5/5,
    3C stable 1/fall 4, and H2 drift 1/fall 4. This is not a repeatable rescue.
- **Shared drift improvements:** left inner-overhang pitch minus and right
    upward-arc 05/rank 6 each give Frame drift 5/5 and both counters stable 5/5.
- **Category-boundary differences:** left overhead gives Frame drift 5,
    3C stable 1/drift 4, H2 stable 4/drift 1; left 06 gives Frame drift 5,
    3C stable 4/drift 1, H2 stable 5. These do not isolate residual causality.
- **Unrecovered physical failures:** left manual plus/minus and right
    inner-overhang rank 6/pitch minus still fall 3/3 for every controller.
    They remain three-run screens, not five-run claims.

The initial H2-only drift regressions on left Boundary 04 and left inner-upward
05 do not repeat: all controllers are stable 3/3 in each focused cell. This
does not erase the two repeatedly shared right inner-forward regressions.

### Tracking, Reliability, and Completion

Initial precise tracking is Frame 135/144 versus 3C/H2 142/144 on FAME;
ALMI is 144/144 for all three. Precision does not mean no tracking cost:
right inner-forward 01 final error spans `0.000286943-0.000288850 rad` for
Frame, `0.005686214-0.005728236 rad` for 3C, and
`0.005684605-0.005704811 rad` for H2 over five runs. Its maximum angular
drift spans `0.097524-0.098952 rad`, `0.111527-0.112942 rad`, and
`0.106917-0.113239 rad` respectively. Report `max_base_drift` and
`rms_base_drift` in **rad**, relative to the pre-release steady roll/pitch
reference, not meters. Height and foot displacement/lift remain in meters.

Both counter controllers remain imprecise 3/3 on each manual failure, but
precise 3/3 on each right overhang failure. Stable-but-imprecise outliers also
remain: Frame left inner-upward 05 is imprecise 1/3 (maximum error
`0.059900513 rad`), and H2 right overhang-upward 03 is imprecise 1/3
(`0.039683621 rad`). Do not hide this H2 tracking anomaly behind stable labels.

FAME has zero initial solver failures; focused 3C has two on left overhead,
while H2 has zero failures and no fallback. ALMI initially has three 3C
failures versus zero H2 failures and three fallback activations. Focused ALMI
has 16 3C failures versus zero H2 failures and 13 fallbacks. Specifically,
right overhang-upward 05 has 9 3C failures and 6 H2 fallbacks over five runs;
right upward-overhang pitch minus has 7 and 7. Both are stable/precise 5/5
for all controllers. Left inner-overhang rank 6 stays a stable/precise
three-run guard. Robust H2's ALMI nominal-solver reliability benefit is retained,
without creating a physical ALMI boundary.

`controller_complete` means no counted controller solver failures, not
successful process execution. `operational_complete` means no logged runtime
errors; `run_complete` requires the simulation window, release log, and
operational completion. Stable, precise, operationally complete runs can have
`controller_complete=false`. Even `passed` does not independently certify
solver reliability. Accepted `best_effort`, abstention, and successful bounded
fallback are distinct from a solver failure. Campaign timing ranges summarize
per-trial statistics, not pooled quantiles or a universal `15 ms` timing pass.

The two readiness failures in FAME r1/r2 were repaired in attempt 2, with
attempt 1 retained: 3C right inner-forward 01 is drift/precise and H2 right
upward-arc 01 is stable/precise, both with zero solver failures. Neither
original attempt reached release. The final latest-attempt tables contain no
infrastructure outcomes. Genuine 3C failures were not rerun as infrastructure;
the separate repetitions retain that reliability evidence. See the refresh
index for the exact retry record and cleanup provenance.

### Immediate Decision

Retain robust H2 without a new controller iteration. Optional v3 was not run;
there is no fresh v3 generalization claim. ALMI v2 stays physically stable and
is a reliability/regression guard, not a discovered physical limit. Specific
FAME phase/model coverage, collision-feasible momentum timing, shared nominal
regressions, and tracking anomalies take priority over broad ALMI speed search.
The original rejected-confidence and mechanical diagnostics below remain valid
within their stated scope; this refresh does not authorize tuning.

## Separate Original 116-Trial Evidence

The sections below preserve the original focused diagnostic campaign and its
detector correction. Historical labels, timing, tests, and unmatched-controller
limitations describe that cohort only, not the finalized refresh above.

## Starting Evidence

Finalized 5E and corrected v2/v3 evidence establish:

- FAME 09/11: established Frame fall to H2 drift rescues.
- FAME 06: Frame fall 3/3; frozen H2 drift/fall/fall in the latest repeated
    reference. A robust-H2 drift screen is not a repeatable-rescue claim.
- Left manual-grasp plus/minus: Frame and frozen H2 fall 3/3; robust H2 falls
    in its screen.
- Right manual-grasp plus/minus: Frame/H2 stable in the reference screen.
- Corrected ALMI-Manip v2/v3: Frame-stable 44/44; robust H2 repairs nominal
    solver failures and supplies reliability guards, not additional rescue cells.

These are historical source labels, not current physical-fall validation.
The audit below invalidates using moderate-tilt 06 labels as controller failure
evidence. Existing physical FAME falls do not demonstrate residual saturation.

## Collection Audit

The first sandboxed launch failed before MuJoCo readiness because the nested
runner could not write the normal uv cache. Its attempts contain no usable
physical experiment and are excluded from outcome statistics. The panel was
relaunched with approved access to the normal benchmark environment. No
controller, model, target, or safety parameter changed to address this failure.

## Measurement Contract

Physical fall direction uses signed simulator pelvis Euler angles through fall
entry and confirmation, before post-fall Euler wrapping becomes misleading.
Positive pitch leans forward; negative pitch leans backward. Positive roll leans
right. Controller signals use the frozen ownership canonicalization: right-arm
manipulation leaves roll/pitch unchanged; left-arm manipulation mirrors roll.

The offline reducer reconstructs the frozen body-model centroidal map from
logged joint states. `A_m dq_m` and `A_c dq_c` describe measured joint-motion
contributions; `A_c u_nominal`, `A_c u_applied`, and `A_c delta_u` describe
requested nominal, applied, and residual contributions. These are distinct from
the simulator's complete arm-subtree momentum, which also includes transport by
the moving base. Both are retained in trace artifacts.

MuJoCo subtree momentum is expressed globally about the subtree CoM. The
simulator logger translates arm momentum to the whole-body CoM. Mechanical
branch comparisons rotate complete vectors into one fixed initial pelvis frame
before applying ownership canonicalization, so momentum and angular response
are not compared across different coordinate frames. See the
[MuJoCo API reference][mujoco-api]
and the project logger/branch implementation for these conventions.

Controller solve records are deduplicated by H2 sequence and aligned through
the retained simulation tick. H1/H2 prediction checks require consecutive solve
sequences. The analysis records reconstruction mismatches because joint-state
diagnostics are sampled after controller execution; an isolated near-threshold
U5 reconstruction is not more authoritative than the logged validity decision.

Diagnostic torso onset is `0.10 rad` on either planar axis for `0.10 s`.
Frame/H2 divergence is a `0.02 rad` axis difference sustained for `0.10 s`.
These do not replace the existing outcome classifier. Fresh-run differences are
not causal subtraction of the residual contribution.

## Corrected Fall Criterion and Full-Window Recheck

The old detector confirmed any tilt above 0.35 rad for 0.75 seconds, even when
base height stayed normal. Tilt is the norm of roll and pitch. Confirmation is
latched; two additional recording seconds cannot revoke it. Historical robust
06 confirmation repetition 2 exceeded 0.35 rad for 0.775 seconds, returned below
it 25 ms after confirmation, and ended supported. Its peak tilt was 0.376 rad,
minimum height 0.935 m, and final tilt/height 0.291 rad/0.950 m. The run stopped
at 5.830 seconds after release, so its unrecorded full hold remains unknown.

The evaluation soft threshold is now 0.50 rad. Hard tilt 0.60 rad, hard height
0.75 m, persistence 0.75 seconds, and two-second observation remain unchanged.
This is an evaluation correction, not a controller/safety-path change.

Five fresh robust-H2 repetitions use the same target, controller, and motion:

| Repetition | Outcome / tracking | Peak tilt (degrees) | Final tilt (degrees) | Minimum height (m) | Old detector replay |
| --- | --- | ---: | ---: | ---: | --- |
| 1 | Drift / precise | 21.10 | 15.39 | 0.937 | No confirmation |
| 2 | Drift / precise | 21.76 | 15.40 | 0.935 | Persistent entry |
| 3 | Drift / precise | 20.96 | 15.42 | 0.937 | No confirmation |
| 4 | Drift / precise | 21.46 | 15.47 | 0.936 | No confirmation |
| 5 | Drift / precise | 21.70 | 15.42 | 0.935 | Persistent entry |

All five complete 13.505 seconds after release with zero solver failures.
Repetitions 2 and 5 independently reproduce the old false-positive mechanism:
they would be stopped early by the old rule, yet finish supported near 0.965 m.
See the [fresh results and videos][6a-recheck].

A saved-data audit of all 116 trials finds **six historical 06 fall labels with
no hard-criterion crossing and no updated-detector confirmation**:

- Frozen H2: two of its three development runs were truncated on moderate tilt.
- Frozen 3C: three of its five confirmation runs were truncated on moderate tilt.
- Robust H2: one of its five historical confirmation runs was truncated.

Do not relabel these prefixes as full-duration drifts. Do withdraw claims that
their old fall counts demonstrate robust-H2 superiority or nominal/lower-body
recovery inconsistency. The audit finds hard-criterion crossings in all eight
historical Frame-06 runs, both five-run Frame-09/11 cells, all 18 left-manual
runs, all nine rank-6 runs, all six pitch-minus overhang runs, and the three
Frame lateral-overhead alias runs. Thus the remaining failure families and
historical Frame contrast survive the detector review.

The original recheck cohort verifies repeatable robust-H2 drift on 06. A contemporaneous
Frame/3C/frozen-H2/robust-H2 comparison under one criterion is still needed to
rank all four controllers or credit H2 residuals. No such comparison was run
in this original cohort; the refresh above now compares Frame/3C/robust H2.
Historical tables below retain original labels solely for provenance.

## Repeated Core and 06 Confirmation Results

The development panel comprises 54 completed trials. Counts below are outcomes
over three repetitions of each target/controller cell under the old detector.
The 06 fall cells are superseded as physical-failure evidence by the audit above.

| Target | Frame | Frozen H2 | Robust H2 |
| --- | --- | --- | --- |
| 09 | Fall 3 | Drift 3 | Drift 3 |
| 11 | Fall 3 | Drift 3 | Drift 3 |
| 06 | Fall 3 | Fall 2, drift 1 | Drift 3 |
| Right inner overhang rank 6 | Fall 3 | Fall 3 | Fall 3 |
| Left manual plus | Fall 3 | Fall 3 | Fall 3 |
| Left manual minus | Fall 3 | Fall 3 | Fall 3 |

The original five 06 confirmations recorded Frame fall **5/5**, frozen 3C
drift **2/5** / fall **3/5**, and robust H2 drift **4/5** / fall **1/5**.
These are historical labels: the 3C and robust fall labels are unsupported by
hard criteria in their truncated windows. The development frozen-H2/robust
label difference likewise does not establish a recovery difference. Fallback
never activated in that panel.

In historical confirmation repetitions 1 and 2, the maximum move residual is only
`4.7e-11 rad/s` in both runs, yet the recorded labels are drift and fall respectively.
All five historical robust runs enter the old pending region at
`3.04-3.095 s`; only repetition 2 receives a fall label (`3.825 s`). Their last valid model ticks are
`1.25-1.27 s`, and late invalid-candidate holds occur around `3.055-3.135 s`.
Neither that hold nor an occasional slow tick explains a physical failure in
these recordings.
The corrected full-window runs identify detector-boundary sensitivity, not
a demonstrated nominal/lower-body recovery failure. These runs still do not
isolate an H2 rescue mechanism. An accepted zero residual
with H2 `best_effort` is also distinct from a rejected solve or fallback.

The first two confirmation repetitions also repeat 09/11. Each now has Frame
fall **5/5** and robust H2 drift **5/5**, with precise manipulation tracking in
all robust runs. Frozen 3C also drifts on both confirmation repetitions. These
are preserved established rescues, not newly discovered H2-only improvements.

## Hard/Boundary Expansion and Target Identity

The expansion panel adds 30 completed trials, with three repetitions per cell.

| Target | Frame | Robust H2 |
| --- | --- | --- |
| Right inner overhang pitch minus | Fall 3 | Fall 3 |
| Right lateral overhead reach | Fall 3 | Drift 3 |
| Right manual plus | Stable 3 | Stable 3 |
| Right manual minus | Stable 3 | Stable 3 |
| Left Boundary 04 | Stable 2, drift 1 | Stable 3 |

**Right lateral overhead is the exact joint-vector alias of Boundary 06.**
Both catalogs specify the same right-arm configuration, endpoint, and `0.74`
amplitude. With the same fast profile, these are extra 06 repetitions, not a new
rescue family. Historically the alias gave ten drift labels and one suspect truncated fall
label across 11 runs of that geometry. The new five-run recheck is reported
separately; do not pool mixed criteria into a rescue success rate. A redundant
independent-rescue confirmation was cancelled before launch.
The catalog audit also records the left lateral-high/Boundary-04 alias used by
the ordinary guard. Catalog definitions and names remain unchanged.

All nine robust ordinary-guard runs are stable with precise tracking. The Frame
guards are also precise, including the one Boundary-04 drift. Robust tracking is
precise on all expansion runs, even the unrecovered overhang. This limited
ordinary panel supports preservation; it is not a fresh full ALMI or catalog
regression campaign. No active controller change was made that would require
promotion through those larger frozen suites.

## Physical Findings

The historical detector-entry motion is pitch-dominant in the core cases.
For 06, entry describes a moderate lean, not a demonstrated physical fall. Overhang tilts backward-left
(median entry roll/pitch `-0.091/-0.339 rad`). The 06 trajectory develops the
same direction (`-0.130/-0.325 rad` at its historical pending entry), then
returns toward supported drift in the complete fresh rechecks. Left manual plus/minus fall forward-right, with entry
roll/pitch approximately `+0.190/+0.295 rad`. The ownership mirror changes the
manual roll sign in controller traces, not the physical direction.

To oppose a developing negative pitch rate, increasing negative counter-arm
pitch momentum should produce a positive torso reaction. A developing positive
pitch rate instead calls for increasing positive counter pitch momentum. This
is an incremental mechanical relation; gravity, contact, transport, and the
lower-body policy also determine the measured angular rate.

Right inner overhang develops negative pitch. The initial backward hand motion
is useful: its negative counter pitch momentum opposes a backward angular
response. In repetition 1, counter wrist x in the body model decreases from
`0.229 m` to `0.087 m` by about `0.8 s`, then returns to `0.189 m` near
`1.2 s`. During that forward return, counter pitch momentum rises positive
while measured pitch rate becomes negative. The nominal command is cancelling
the manipulation arm's negative momentum during this phase. A wrong-sign H2
residual is not responsible for the visible motion: robust
H2 has only one valid residual tick in this run, at about `0.60 s`, with residual
norm `0.00064 rad/s`.

That early brake has an explicit optimizer explanation. In repetition 1 at
`0.60 s`, only pitch is confident. N5 predicts terminal pitch
`-0.03330 rad` and rate `+0.02036 rad/s`, a recovery phase. Along the H2
incremental-rate direction, the tilt term contributes gradient `-0.2664` and
the rate term `+0.5089`; positive-divergence cost is inactive. The net gradient
therefore favors a negative rate correction. The optimizer requests a tiny
positive counter pitch-momentum correction, for which R5/U5 predict
`delta omega_pitch = -6.39e-5 rad/s`. This is a locally consistent cost decision,
not a reversed action gradient. It does not establish that braking this recovery
is beneficial over the eventual fall horizon, nor explain the much larger
nominal retraction/return cycle or later safety hold.

At about `1.17 s`, the shared finalizer changes from backtracking to
`counter_candidate_invalid` and holds the counter arm. Offline collision checks
on measured states identify `left_shoulder_yaw_link_sphere2_0` against
`torso_link_sphere1_0`. This is a collision-model overlap, not proof of physical
simulator contact. The shoulder/torso safety constraint is preserved.

The repeated phase comparison below uses median per-run window means. Momentum
is the joint-motion contribution in `kg m^2/s`; rate is in `rad/s`. Requested
and realized signals have the expected actuator lag and safety differences.

| Target / window (s) | Pitch rate | Moving momentum | Counter requested | Counter realized |
| --- | ---: | ---: | ---: | ---: |
| Overhang / 0.8-1.2 | -0.021 | -0.608 | +0.344 | +0.182 |
| Overhang / 1.4-1.8 | -0.145 | -0.135 | +0.027 | +0.003 |
| 06 / 1.4-1.8 | -0.228 | -0.201 | -0.095 | -0.088 |
| 09 / 1.4-1.8 | -0.185 | -0.141 | -0.062 | -0.061 |
| 11 / 1.4-1.8 | -0.212 | -0.205 | -0.059 | -0.062 |
| Manual plus / 1.4-1.8 | +0.192 | +0.078 | +0.006 | -0.014 |
| Manual minus / 1.4-1.8 | +0.186 | +0.077 | +0.006 | -0.014 |

Overhang's applied momentum in the first row is only `+0.261`, and in the
second row it is zero because the finalizer holds. Its shoulder-pitch joint
first moves from about `+0.014` to `+0.431 rad`, then returns to `+0.111 rad`
near `1.2 s`; the elbow moves from `+0.019` through `-0.258` to `-0.439 rad`.
The saved traces include all four commanded/realized velocities and excursions.
The visible return is mainly nominal action, far larger than the H2 residual.

In 09/11, the earlier positive counter momentum occurs while pitch rate is
positive, during recovery. Later, negative counter momentum accompanies the
developing backward rate, without overhang's early collision hold. Thus the
same Cartesian or momentum direction can be helpful in one phase and harmful
in another. The 06 pattern resembles these guards. Its smaller margin to the old detector
threshold does not establish smaller physical recovery margin or an opposite
action gradient; its apparent inconsistency was classification-sensitive.

Manual plus/minus instead develop positive pitch. Their residual-valid windows
end before the tilt crosses into the forward-diverging phase. Later counter
motion has a braking pitch contribution, but the robot still falls. The
overhang's early collision hold is therefore not a universal explanation for
the manual failures.

Manual pitch crosses forward around `1.4 s`. At that transition, realized
counter pitch momentum remains slightly negative despite a near-zero/positive
request. By about `2.0 s`, positive counter momentum does oppose the developing
forward response, but rate and tilt continue growing. This supports insufficient
anticipation and late realized braking, with lost model coverage, rather than
a universal sign inversion. The complete pre-fall half-second still has positive
counter pitch momentum (about `+0.04` to `+0.05`) against positive pitch rates
around `+0.34` to `+0.35 rad/s`.

The frozen benchmark lifecycle also fades nominal balance scale between the
end of the `1.5 s` move and approximately `2.0 s`. Afterward, nominal I-stage
posture return remains while the balance target is zero. This is an existing lifecycle
constraint, not an H2 optimizer brake/reverse decision. It must be distinguished
from model abstention and physical counter-arm reversal.

## Predictive-Confidence Experiment

The single proposed confidence change retains the N5 model and its error bounds,
but uses the predicted two-knot intervals rather than a raw-rate-change margin.
It requires both intervals to exclude zero and agree with the current rate sign.
No command was published under this alternative.

Repetition 1 was development evidence; repetitions 2/3 were held out. Newly
admitted samples have the following two-knot phase-sign results:

| Family | New roll samples / correct | New pitch samples / correct |
| --- | ---: | ---: |
| Right inner overhang | 7 / 3 | 13 / 12 |
| 06 | 16 / 14 | 17 / 16 |
| 09 | 7 / 4 | 14 / 13 |
| 11 | 15 / 12 | 16 / 16 |
| Manual plus | 18 / 12 | 11 / 7 |
| Manual minus | 18 / 13 | 21 / 17 |

The alternative adds coverage but fails the `0.90` sign gate, especially in the
manual families. Development error quantiles also exceed the unchanged
calibration bounds. Selecting only favorable families would introduce forbidden
target-dependent behavior. The change is **rejected before active control**.

The complete core panel has zero mismatches between reconstructed and logged
N5/context-valid decisions on all 18 robust-H2 runs. Thus the observed abstention
is explained by the frozen terms, not a missing or misread validity flag.

Some model-valid manual ticks report H2 `best_effort` with zero residual. A
separate iteration-count diagnostic reconstructs those contexts and compares one
versus two iterations under the same residual box and elbow velocity constraint.
Other nominal constraints are relaxed, so this is not a full runtime replay.
Across 25 contexts, all 12 zero-action contexts remain zero with either budget.
Nonzero solutions can differ by up to `0.0025 rad/s`; the comparison does not
establish that every solution is converged. It does show that a second iteration
does not unblock the observed zero-action cases or add model-valid coverage.
No active iteration-budget change is justified by this diagnostic.

## Repeated Model Coverage and Phase Evidence

The following robust-H2 timings are relative to manipulation release. Valid
counts cover the `1.5 s` move; onset uses the diagnostic tilt threshold above.

| Target | Valid ticks in repetitions 1/2/3 | Last valid time (s) | Tilt onset (s) |
| --- | --- | --- | --- |
| Overhang rank 6 | 1 / 5 / 2 | 0.600-0.700 | 1.715-1.720 |
| 06 | 13 / 12 / 12 | 1.265-1.285 | 1.565-1.580 |
| 09 | 15 / 14 / 14 | 1.220-1.235 | 1.620-1.625 |
| 11 | 12 / 13 / 12 | 1.275-1.295 | 1.665-1.670 |
| Manual plus | 3 / 7 / 2 | 1.195-1.210 | 2.035-2.050 |
| Manual minus | 2 / 6 / 5 | 1.185-1.205 | 2.075-2.080 |

Overhang has no sustained `0.02 rad` Frame/H2 separation before the first fall
entry in any development repetition. Its safety hold begins at `1.155-1.185 s`,
well after its last useful model-valid tick and well before fall entry at
`3.435-3.480 s`. Manual Frame/H2 separation occurs at `1.010-1.180 s`, before
large tilt, but subsequently both trajectories fall. For 06 the first such
separation ranges from `1.520` to `2.305 s`; for 09 it is `1.440-1.495 s`, and
for 11 it is `1.610-2.330 s`. These fresh-run separations cannot isolate residual
causality or prove that the earliest command created the eventual outcome.

The frozen U5 joint-3 position domain is approximately `[-0.3975, 0.3298] rad`.
Overhang exits it as the elbow returns, while R5 pitch-rate context is limited
to approximately `[-0.0754, 0.0832] rad/s`. N5 additionally limits current and
previous tilt/rate; its pitch tilt limit is about `0.0492 rad`. These checks
explain later abstention independently of residual authority. Earlier low-rate
windows frequently have no confident N5 axis. The manual families subsequently
lose R5 rate and N5 rate/tilt validity as their forward response develops.

Successful 09/11 also lose model validity before peak tilt. Coverage loss alone
therefore does not classify a fall or supply a safe enable/disable rule. The
distinguishing physical requirement is useful momentum during the developing
response, within the available collision-feasible excursion and verified phase.

Absolute two-knot pitch predictions on model-valid, pitch-confident core ticks
give the following pooled results. These are phase diagnostics, not isolated
causal validation of R5 action response.

| Target | Samples | Rate-sign accuracy | Rate RMSE (rad/s) |
| --- | ---: | ---: | ---: |
| Overhang rank 6 | 8 | 1.000 | 0.0165 |
| 06 | 35 | 0.943 | 0.0156 |
| 09 | 43 | 0.953 | 0.0152 |
| 11 | 37 | 1.000 | 0.0158 |
| Manual plus | 10 | 1.000 | 0.0377 |
| Manual minus | 11 | 0.909 | 0.0325 |

Manual prediction magnitude is less accurate even in the sparse accepted
window. High sign agreement on these few samples does not validate expansion
into the later forward-diverging region.

Move-phase H2 decision counts pooled across the three core repetitions are:

| Target | Abstain | Continue | Brake | Reverse | Peak residual norm (rad/s) |
| --- | ---: | ---: | ---: | ---: | ---: |
| Overhang rank 6 | 220 | 0 | 8 | 0 | 0.00270 |
| 06 | 222 | 3 | 4 | 0 | 0.00946 |
| 09 | 187 | 4 | 38 | 0 | 0.01319 |
| 11 | 217 | 7 | 6 | 0 | 0.00967 |
| Manual plus | 228 | 0 | 1 | 0 | 0.00090 |
| Manual minus | 223 | 4 | 2 | 0 | 0.00991 |

`Brake` means residual projection against the nominal joint command; it does
not identify the physical momentum sign. `Abstain` also includes a negligible
accepted residual in a model-valid context. The vector norm can exceed `0.01`
while every joint respects the box. There are no H2 `reverse` decisions here;
the large visible counter-arm reversal is a nominal trajectory phenomenon.
The first non-negligible overhang residual appears at `0.535-0.600 s`, after
manipulation momentum begins and well before large tilt, but coverage then ends.
Manual plus has no non-negligible residual in two repetitions and only a tiny
one around `1.155 s` in the third. Earlier residual timing alone does not separate
the successful guards from failures.

## Controlled Mechanical Audit

The audit restores 126 recorded states across the six core families and three
repetitions. Two axes and two amplitudes produce **504 symmetric branch pairs**.
Repeated zero branches match exactly. The one-tick-delayed first sample has
exactly zero paired angular-rate and counter-momentum difference.

At `40 ms`, no pair exceeds the established `0.002 rad/s` material-response
threshold. At maximum joint request `0.01 rad/s`, the largest targeted angular
response is `0.000906 rad/s`. Raw pitch signs agree with the reaction direction
on all 126 maximum-amplitude pitch samples. Maximum-amplitude roll signs agree
on all right-arm samples, but only 12/21 in each left-manual family. All remain
below the material threshold: this is useful mechanical sign evidence, not a
new verified response model or permission to expand context.

The branches hold lower-body and manipulation controls fixed rather than
replaying online policy history. They cannot prove a long-horizon rescue or
that increasing authority would help. None of the unrecovered core targets or 06 saturates its per-joint trust bound. Two successful 09 guard ticks
reach `-0.01 rad/s` on joint 1; saturation is not a failure separator. The
evidence supports investigating phase and feasible coverage before authority.

## Tracking, Solver, Safety, and Timing Audit

All 111 original physical trials complete their controller process.
The detailed timing/model statistics in this section cover that original
campaign; the five new rechecks add completed precise drifts and zero reported
solver failures, but have not been incorporated into this timing audit. There are no rejected
H2 solves or nominal fallback activations in the 60 frozen/robust-H2 runs.
Accepted `best_effort` and zero residuals are retained in the audit rather than
misclassified as solver failure. All 29,571 deduplicated H2 solve records have
finite residuals, respect the `0.01 rad/s` per-joint box, and keep joint 2 masked.
Reconstructed N5/context validity matches logged decisions throughout.

All 09/11 robust trials and the ordinary guards have precise tracking. Left
manual plus/minus remain imprecise in all six robust development trials, with
final configuration error approximately `0.089-0.093 rad`; frozen H2 has the
same failure pattern. This is an unresolved manipulation/fall limitation, not
a newly introduced tracking regression.

The additional inner-overhang pitch-minus geometry also develops backward-left
pitch-dominant falls. Its early invalid-candidate hold begins around `1.0 s`;
measured-state checks identify an elbow/torso sphere overlap in sampled states.
This differs from rank 6's shoulder/torso pair but reinforces the need for a
collision-feasible momentum path. Neither overlap authorizes bypassing safety
or proves physical simulator contact. Full excursions, backtracking, requested
versus applied velocity, and model-domain departures are retained per run.

| Controller | Runs / solves | Total p50 / p95 / p99 (ms) | Maximum (ms) |
| --- | ---: | --- | ---: |
| Frozen H2 | 18 / 7,100 | 2.85 / 6.49 / 8.26 | 19.85 |
| Robust H2 | 42 / 22,471 | 2.73 / 6.19 / 8.10 | 31.04 |

Pooling does **not** establish a universal timing pass. Robust rank-6 repetition
3 has full-run p99 `24.18 ms`, above the `15 ms` gate, with three late safety-hold
ticks after fall entry. Across robust H2, 12 logged publications exceed `20 ms`:
nine have `solved` status and three `counter_candidate_invalid`. The motion-only
pooled p99 is `10.85 ms`, but its worst individual run is `18.38 ms`. Report
these observations as timing failures/outliers, not silently rejected late
results. No new candidate passes a timing promotion gate in 6A.

The manual failures have maxima below `20 ms`; overhang loses useful H2 coverage
long before its late timing cluster. The historically mislabeled robust 06 run has a slow tick
near `1.025 s`, but successful 06 runs also have similar slow ticks. Timing is
therefore a retained reliability concern, not a demonstrated common cause of
the unrecovered falls. Host health and resource use were recorded; the machine
was not an isolated real-time host.

## Original Decision and Research Handoff

Retain `counter_residual_h2_robust` and all existing frozen terminology. There
is no accepted 6A controller change and no new independent repeatable rescue.
The controller stop condition remains supported by the real manual/overhang
failures and rejected confidence experiment. Remove 06 classification instability
as a rationale for controller tuning: correcting evaluation already yields
five complete robust-H2 drifts without changing H2.

- **Prediction/phase coverage:** overhang and manual useful intervention windows
    extend beyond calibrated U5/R5/N5 coverage. The proposed interval-confidence
    replacement adds wrong-phase samples, so broader context needs new held-out
    calibration and material action-response evidence before publication.
- **Nominal momentum timing and feasibility:** rank 6 first retracts usefully,
    then returns with locally adverse pitch momentum and reaches a safety hold.
    Manual braking becomes physically opposing after forward divergence develops.
    Verify a feasible phase-dependent response/reserve description before
    changing nominal return, balance fade, horizon, or support objectives.
- **Evaluation and attribution:** 06 is a repeatable supported-drift case under
    the corrected criterion. Audit moderate-tilt fall labels before treating
    them as control failures. Retain 06 as a regression/attribution case; compare
    Frame, 3C, frozen H2, and robust H2 with full windows under one criterion
    before ranking all four controllers or assigning H2 credit. The finalized
    refresh now shows the same 06/09/11 rescue labels for 3C and robust H2.
    Do not tune H2 to fix the six unsupported historical labels.
- **Regression and timing:** preserve 09/11, ordinary tracking, collision checks,
    and independently runnable frozen controllers. Any future active candidate
    must pass three development repetitions, five final rescue repetitions,
    the applicable ALMI v2/v3 regression gates, and individual timing checks.

Do not increase the residual box, flip an action sign globally, add a target ID
rule, loosen collision checks, or extend the balance lifecycle from these data.
The remaining questions require verified physical/model information, not more
ad-hoc tuning of the mature H2 architecture.

## Reproducibility and Validation

The local [artifact index][6a-evidence] lists all sweep
directories, exact rerun commands, diagnostics, and exclusions. Representative
[rank-6 traces][rank6-traces]
and [manual-plus traces][manual-traces]
show commands, realized momentum, torso response, model decisions, and timing.
The [rank-6 intervention detail][rank6-detail] enlarges the early phase;
prediction curves include only model-valid, pitch-confident samples.
Full numerical timelines and repeated statistics are under
the owning trials' `counter_response.npz` files and the evidence group's
`reports/`; illustrative first runs do not replace repeats. One-time scripts
and duplicate reductions have been retired. Reusable reduction and plotting
commands are documented in the benchmark's counter-response diagnostic guide.

The source-preservation audit verifies all 28 initial controller/model and
target hashes unchanged, plus residual/mask constraints and all 504 matched
branch controls. The repository benchmark tests and six relevant frozen H2,
robust H2, OCP, response-model, 3C-planner, and velocity-controller test modules
pass: **203 tests**, with two dependency deprecation warnings. No frozen
controller, policy, target, collision/safety-path parameter, or ALMI checkpoint
was changed. The subsequent soft fall-evaluation threshold changed to 0.50 rad;
24 focused detector/runtime/summary tests passed for that update. Historical
source hashes and test counts describe the original campaign snapshot.

[mujoco-api]: https://github.com/google-deepmind/mujoco/blob/main/doc/APIreference/functions.rst
[6a-evidence]: ../../../runs/key_findings_reports/iteration6a/README.md
[rank6-traces]: ../../../runs/key_findings_reports/iteration6a/reports/figures/right_inner_upward_overhang_rank6.png
[manual-traces]: ../../../runs/key_findings_reports/iteration6a/reports/figures/left_manual_grasp_pitch_plus.png
[rank6-detail]: ../../../runs/key_findings_reports/iteration6a/reports/figures/right_inner_upward_overhang_rank6_intervention.png

[6a-recheck]: ../../../runs/key_findings_reports/iteration6a/classification_recheck/README.md
[refresh]: ../../../runs/key_findings_reports/iteration6a/benchmark_results.md
[refresh-report]: ../../../runs/key_findings_reports/iteration6a/reports/benchmark/campaign/campaign_summary.json
[refresh-csv]: ../../../runs/key_findings_reports/iteration6a/reports/benchmark/campaign/campaign_summary.csv
[refresh-videos]: ../../../runs/key_findings_reports/iteration6a/reports/benchmark/campaign/key_video_index.html
