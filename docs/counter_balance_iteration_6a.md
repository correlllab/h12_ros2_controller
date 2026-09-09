# Counter-Balance Iteration 6A

## FAME Rescue Expansion

## Status and Source of Truth

Iteration 6A starts from `counter_residual_h2_robust`, frozen by Iteration 5E.
The H2 architecture is mature. This iteration diagnoses remaining FAME falls
before selecting one minimal, verified change. Execution is complete; the
confidence candidate failed its offline gates and no new controller is promoted.
The subsequent detector correction and five full-duration 06 drifts revise the
classification conclusion, not the controller. The finalized benchmark refresh
adds 22 completed sweeps without tuning or a new iteration. See
[the revised analysis](counter_balance_analysis_iteration_6a.md).

Read together with the finalized design and analysis for
[Iteration 5](counter_balance_iteration_5.md),
[5B](counter_balance_iteration_5b.md), [5C](counter_balance_iteration_5c.md),
[5D](counter_balance_iteration_5d.md), and [5E](counter_balance_iteration_5e.md),
the [checkpoint analysis](counter_balance_analysis_almi_checkpoint_sweep.md),
and the corrected [v2/v3 benchmark](counter_balance_almi_manip_v23_benchmark.md).
The matching 6A analysis distinguishes historical evidence, fresh experiments,
mechanical interpretation, and unverified hypotheses.

The source review also includes the finalized analyses for
[5](counter_balance_analysis_iteration_5.md),
[5B](counter_balance_analysis_iteration_5b.md),
[5C](counter_balance_analysis_iteration_5c.md),
[5D](counter_balance_analysis_iteration_5d.md), and
[5E](counter_balance_analysis_iteration_5e.md), the
[checkpoint protocol](counter_balance_almi_checkpoint_sweep.md), and
[ALMI-Manip training notes](../../h12_mjlab/docs/almi_manip.md).

## Finalized Benchmark Refresh

The [refresh index][refresh] and [final campaign report][refresh-report] replace
deprecated broad-sweep evidence. Each of FAME and corrected ALMI-Manip v2 has
100 full plus 44 challenge trials per controller, comparing Frame, frozen 3C,
and robust H2. The 144 named IDs represent 140 geometries per policy; aliases
do not create independent rescue families. Optional v3 was not run and has no
fresh result in this campaign.

All 22 sweeps are complete, including FAME `confirm_r4/r5` and the two ALMI
reliability cells through five repetitions. FAME has 54 three-run screen cells,
11 extended to five; ALMI has three screen cells, two extended to five.
Initial, focused, health, and original 116-trial diagnostics remain separate.
The index gives exact aggregates, every target-level transition, video links,
tracking anomalies, retained attempts, reproduction, and cleanup provenance.

Under the corrected full-window criterion, right 06/09/11 each gives Frame
fall 5/5 versus both 3C and robust H2 drift 5/5 with precise counter-controller
tracking. This closes the matched-comparison gap for those three controllers,
not a frozen-H2 comparison or an isolated residual attribution. Both counter
controllers also repeat right inner-forward 01/02 stable-to-drift regressions
5/5; right inner-upward 05 remains fall 4/5 for each. Manual and distinct
overhang failures remain specific diagnosis priorities.

ALMI is stable/precise 144/144 per controller initially and 13/13 in the
focused panel. Genuine 3C nominal failures remain in those stable runs;
robust H2 retains zero solver failures with bounded fallback. This supports
solver reliability, not a new physical ALMI boundary. The two readiness
failures in FAME r1/r2 were repaired with attempts preserved; true solver
failures were not retried as infrastructure. `controller_complete` is distinct
from operational and run completion, as detailed in the analysis.

Retain robust H2 and the frozen contract. The immediate conclusion is specific
FAME phase/momentum-feasibility and shared-regression diagnosis, not broad
ALMI speed search or ad-hoc controller tuning.

## Frozen Contract

- Preserve pure frozen-3C nominal planning plus bounded H2 residual MPC.
- Preserve the 12-state, two-knot, one-tick-delay U5/R5/N5 formulation.
- Retain residual joints 0/1/3, masked joint 2, and the `0.01 rad/s` per-joint
    trust bound with action/change weights `0.01/0.005`.
- Preserve per-axis confidence, model-domain checks, zero-residual abstention,
    and the single collision/safety/atomic-publication path.
- Preserve robust H2's bounded SciPy nominal fallback only on rejected
    Crocoddyl nominal solves. Frozen H2 and robust H2 remain independently
    runnable.
- Runtime remains policy-blind, target-blind, and real-compatible. Simulator
    state is permitted only for offline diagnosis and matched mechanical tests.
- Require complete-controller p99 below `15 ms`; report maxima, rejected late
    results, solver failures, fallback use, tracking, and safety separately.

H3 passed model and timing gates but failed ordinary behavior in 5B. Foot twist
predicted support motion but lacked distinguishable H2 action sensitivity in 5C.
Benefit/phase/confidence gates could not separate old ALMI improvements and
regressions in 5D. The accepted 5E change repaired solver reliability without
adding FAME authority. None justifies a larger residual, unverified support cost,
longer horizon, or target-specific rule in 6A.

## Original Focused Targets and Repetitions

Use the existing Magpie catalogs without changing geometry or classification:

- Hard: `data/arm_hard_targets.yaml`, 16 targets.
- Exploration: `data/arm_exploration_targets.yaml`, 20 targets.
- Boundary: `data/arm_fast_boundary_targets.yaml`, 8 targets.

Keep the `1.5 s` fast move and `10 s` hold, existing gains, initialization,
and safety. The original protocol retained the fall detector; after review,
its soft tilt threshold changes from 0.35 to 0.50 rad for the five-run
classification recheck. Keep hard criteria and observation durations unchanged.
Historical outcome labels retain their original criterion. The suffix `_fast_a01` identifies a trial; the
diagnostic catalog ID is `right_inner_upward_overhang_rank6`.

The first panel interleaves Frame, frozen H2, and robust H2 on:

1. `right_fast_fall_search_09_scale_78`: mandatory established rescue.
2. `right_fast_fall_search_06_scale_74`: priority classification/attribution case after detector correction.
3. `right_fast_fall_search_11_scale_78`: mandatory established rescue.
4. `right_inner_upward_overhang_rank6`: priority physical-sign diagnostic.
5. `left_manual_grasp_pitch_plus`: established unrecovered Hard fall.
6. `left_manual_grasp_pitch_minus`: established unrecovered Hard fall.

Use three repetitions, rotating controller order between repetitions. Execute
serially and headlessly with one worker. Check machine health before batches.
Retain failed attempts separately; execution failure is not physical fall
evidence. Right manual-grasp plus/minus and distinct ordinary geometries are
regression/expansion guards, not assumed Frame falls.

The follow-up Frame/robust-H2 panel repeats the historically fall-prone Hard
targets `right_inner_upward_overhang_pitch_minus` and
`right_lateral_overhead_reach`, plus right manual-grasp plus/minus and
`left_fast_fall_search_04_scale_76` ordinary guards. Use three repetitions with
alternating controller order. The catalog identity audit establishes that right
lateral overhead is the exact joint-target alias of Boundary 06. Count it as
additional 06 evidence, not an independent geometry or rescue family. The other
cases preserve another overhang geometry and both-side ordinary evidence.

Any final additional-rescue claim needs five contemporaneous repetitions of the
changed cell and both 09/11 guards. Include Frame and frozen 3C on final 06
comparisons, as required by Iteration 5. A discovery result alone is insufficient.

The original development labels showed three robust-H2 06 drifts and mixed
frozen-H2 labels. Later detector auditing invalidates interpreting the frozen-H2
fall labels as physical failures; retain the original follow-up protocol below
as historical provenance.
Since no accepted-path controller change explains that difference, run five
fresh Frame/3C/robust-H2 06 triplets with rotating controller order. Report these
five runs and the original three separately and together. Add 09/11 to the first
two confirmation repetitions so their Frame/robust guards each reach five.
Do not select a favorable subset or attribute a fresh-run difference to fallback
when fallback did not activate.

Do not launch an independent rescue confirmation for the lateral-overhead
alias. Its favorable expansion results cannot count as a new conversion or replace
the dedicated full-window 06 classification check. Compare actual joint vectors across catalogs
before interpreting distinct target names as distinct experiments.

Corrected ALMI-Manip v2 is the primary ALMI regression benchmark; v3 is an
independent generalization/reliability check. Both are Frame-stable on all 44
current targets. Their static/fine-tuning gains do not count as FAME rescues.

## Physical and Signal Diagnosis

Align to manipulation release and deduplicate controller solve records. Inspect
the full timeline through fall confirmation or completed hold, with separate
early motion, pre-divergence, first divergence, and pre-fall windows.

For each priority case, establish:

- Signed roll/pitch tilt, angular rate, eventual fall axis and direction.
- Manipulation and counter-arm centroidal momentum contributions in the same
    coordinate frame, distinguishing measured motion from commanded motion.
- Nominal, residual, combined, applied, and realized counter-joint velocity;
    excursion, clipping, collision backtracking, and publication status.
- The counter momentum and incremental angular response that would oppose the
    developing fall; the response actually requested and realized.
- U5 realization gains, R5 contextual gradient and domain validity, N5 phase
    prediction, per-axis confidence, and each failed model-validity term.
- Predicted versus measured H1/H2 angular response, residual magnitude/sign,
    and timing relative to manipulation momentum and Frame/H2 divergence.
- Continue/brake/reverse labels and their underlying joint-space projection.
    These labels alone do not establish physical momentum direction.

A retracting hand is not proof of a wrong-sign action. Use the centroidal map
and angular response; separate mechanical reaction from gravity/contact/policy
response and from whole-body transport of an arm during a fall. Fresh runs are
not exact counterfactual branches, so do not attribute their entire difference
to a small residual.

## Experiment and Promotion Rule

Follow experiment, physical interpretation, signal diagnosis, minimal change,
and retest. Compare failures with 09/11 before selecting a mechanism. Prefer
verified predictive direction, anticipation, timing, or model coverage. Expand
a model domain only with held-out sign/ranking and phase-confidence evidence;
removing a validity gate is not model validation.

Change one meaningful mechanism at a time. Rerun the same three-repetition
development panel and ordinary guards. Reject any change that loses 09/11,
degrades tracking or ordinary behavior materially, violates safety, or fails
timing. Advance to five-repetition claims only after those gates pass.

Stop with robust H2 retained if repeated traces and controlled tests establish
a limitation requiring new verified response/support information for 6B/6C.
Do not manufacture a candidate when the necessary action gradient or phase
evidence is missing.

## Preregistered Offline Mechanism Tests

After the three-repetition panel, test one predictive-confidence alternative
without publication. Keep N5 predictions, calibration error bounds, U5/R5 domain,
trust, and costs unchanged. Require both predicted knots to exclude zero by the
existing per-axis error bound and retain the current rate sign. This replaces
only the conservative raw-rate-change extrapolation in the confidence decision.
Report newly covered samples separately from the original confident samples.
Use repetition 1 for diagnosis and repetitions 2/3 as held-out evidence. Require
at least 30 newly covered samples per claimed axis, at least `0.90` two-knot sign
accuracy, and error calibration compatible with the unchanged bound. Reject the
change before active control if it does not create useful pre-divergence
coverage or misses its prediction gates.

Separately audit mechanical sensitivity at `0.3/0.6/0.9/1.2/1.5/2.0/2.5 s`,
excluding samples after fall entry. Restore one recorded state and identical
lower-body/manipulation controls; apply symmetric momentum-aligned residual
requests at `0.005/0.01 rad/s` maximum joint magnitude after the frozen one-tick
delay. Preserve the weak-joint mask. Repeat zero branches and compare realized
counter velocity, momentum, and angular response at `20/40 ms`. Retain the
established `0.002 rad/s` material-effect threshold. This is a mechanical
diagnostic, not an online-policy counterfactual or authorization to expand trust.

## Artifact Locations

Original diagnostic evidence stays under `runs/key_findings_reports/iteration6a/`, grouped
into `characterization/`, `confirmation/`, `expansion/`, and
`classification_recheck/`. Each timestamped
sweep retains its manifest, `configs/sweep.yaml`, classifications, full JSONL
controller traces, simulation replay arrays, and logs. The standard sweep CLI
is the entry point; the one-time 6A runner has been retired.

Broad refresh sweeps live in `runs/full_sweep/` and `runs/challenge_sweep/`,
not under `key_findings_reports/`. Within `iteration6a/`, the refresh index is
`benchmark_results.md`, reports are in `reports/benchmark/`, and videos are in
`videos/`. Refresh repetitions live directly in `focused_repetitions/`, named
`<timestamp>_iter6a_<policy>_<screen_rN/confirm_rN>`; health checks live directly
in `health/`, named `<timestamp>_iter6a_<policy>_check`.

Original cross-run results and figures live in `reports/`; source hashes and relocation
provenance live in `reports/provenance/`. Videos use each sweep's
`videos/<controller>/` directory. The current collection is
`classification_recheck/videos/index.html`; the obsolete four-drift/one-fall
collection was removed while preserving canonical recordings in their runs.
See the [artifact index][6a-evidence] for paths and diagnostic commands.
`reports/classification_audit.json` audits all 116 saved trial windows;
`historical_repeated_summary.json` retains the old labels and signal reductions
without presenting them as current 06 reliability conclusions.

## Original Focused Execution Decision

The 111-trial historical campaign preserves 09/11 at five robust-H2 drifts
against five Frame runs reaching hard fall criteria each. The additional five
06 rechecks all complete 13.505 seconds after release as drift with precise
tracking and zero solver failures. Six historical 06 fall labels, spanning
frozen H2, frozen 3C, and robust H2, are unsupported by hard criteria in their
truncated saved windows. Do not use those labels to rank controller recovery.

The conclusion changes from inconsistent 06 rescue to repeatable robust-H2
06 drift under corrected evaluation. This does not isolate an H2-residual
advantage or constitute a fresh matched-controller rescue comparison. Additional
lateral-overhead drifts remain the same geometry. Manual and distinct overhang
failures still reach hard criteria and remain the controller-research priority.

Retain the frozen independently runnable robust H2. Carry verified phase/model
coverage and collision-feasible momentum timing into 6B/6C, as specified in the
analysis. Do not extend confidence, residual authority, horizon, or lifecycle
based on the rejected diagnostics.

[6a-evidence]: ../../../runs/key_findings_reports/iteration6a/README.md
[refresh]: ../../../runs/key_findings_reports/iteration6a/benchmark_results.md
[refresh-report]: ../../../runs/key_findings_reports/iteration6a/reports/benchmark/campaign/campaign_summary.json
