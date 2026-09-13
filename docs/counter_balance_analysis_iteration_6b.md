# Counter-Balance Analysis: Iteration 6B

Status: **CLOSED / FINALIZED** on 2026-09-13.

## 1. Final Experimental Question And Setup

This final report asks whether frozen robust 3C (`counter_ddp_velocity_robust`)
improves saved straight-line manipulation tracking relative to Frame (`frame_task`)
on the current rebuilt banks. It covers ALMI-Manip-v2 and FAME, the ten-trajectory
`directional_traj` and `overhang_traj` banks, and duration factors
`1.0/0.8/0.6/0.5/0.4/0.3`. Smaller factors are faster. All controllers, policies,
banks, runtime thresholds, classifications, and recorded evidence are frozen.

The canonical grid contains 24 roots, 480 first attempts, and seven retries. The
machine-readable 487-record source is
[`canonical_inventory.csv`](../../../docs/data/iteration6b_spatial/canonical_inventory.csv).
Every record has policy, bank, trajectory, controller, factor, attempt, path,
current bank/geometry hash, recorded classification, metric recoverability,
derive from this inventory before pairing. The report generator is:

```bash
uv run python -m h12_zmp_benchmark.experiment.iteration6b_spatial_report
```

It is offline-only: it reconstructs recorded physics metrics and writes derived
exports under `docs/data/iteration6b_spatial/`. It does not simulate, replay,
repair, reclassify, tune, or modify evidence. The audit confirms all 487 records
pass current-bank hash gating, all 479 recoverable Ideal / Timed RMS values match
their recorded values exactly, and all tracked inputs were unchanged.

## 2. Final Trajectory Banks

The current schema-v2 banks each contain ten fixed, validated Magpie trajectories:

- `directional_traj`: extended-arm directional reaches.
- `overhang_traj`: elevated/overhang reaches.

Their manifests and all geometry payload hashes are current and pinned in each
saved configuration. No archived trajectory enters a final aggregate. The original
stable-standing full SE(3) capture registers each trial's fixed desired world-frame
segment. No fitted line, timestamp shift, DTW, spatial realignment, progress
normalization, smoothing, or live-reference recapture is used.

FAME `left_overhang_forward` remains a known initialization limitation. Eight
attempts failed standing readiness without a stable-standing capture. They remain
unrecoverable for reference-relative spatial metrics and are not replaced or
repaired here. Repair is deferred and not needed for the present conclusion.

## 3. Final Metric Definitions

The hierarchy is intentional:

1. **Primary spatial accuracy:** Uniform Spatial RMS, then Segment Spatial RMS.
2. **Secondary timing-sensitive tracking:** Ideal / Timed RMS.
3. **Task guards:** completion, endpoint error, projected progress, and physical
   outcome.
4. **Mechanism diagnostics:** base-induced wrist displacement, pelvis tilt, and
   pelvis translation/drift.

For measured world point `x_i` and fixed desired finite segment `[a,b]`, nearest
segment distance uses a clipped projection. Segment Spatial RMS is the RMS of that
weighted. Uniform Spatial RMS removes that dwell weighting: it preserves the
chronological measured polyline, removes only exact consecutive duplicates needed
for interpolation, resamples it at 1001 uniform arc-length positions, and takes
the same finite-segment distance RMS. Reversals, overshoot and excursions remain.
A zero-length measured path has undefined, not zero, Uniform RMS.

Ideal / Timed RMS retains the existing scheduled-clock comparison and includes
phase/servo lag as well as geometric error. It can legitimately demonstrate a
timing-sensitive task benefit, but it cannot establish a generally straighter path.
Endpoint, progress, completion and recorded outcome remain visible because a small
spatial error on a partial path is not successful manipulation. Base-fixed execution
RMS is retained only as a mechanism diagnostic, not a spatial-task substitute.
The detailed contract and test rationale are in the
[straight-line benchmark specification](../../../docs/straight_line_trajectory_benchmark.md).

## 4. Final Spatial Results

All values below are mm. `U` is Uniform Spatial RMS; `S` is Segment Spatial RMS;
`T` is Ideal / Timed RMS. Values are `Frame / 3C`; gains are `Frame - 3C`, so a
positive gain favors 3C. Wins count strictly positive unrounded per-pair gain.
Every row averages matching per-run values only; raw timestamps are never pooled.
First attempts are primary. The separate latest-attempt sensitivity is in
[`latest_table.csv`](../../../docs/data/iteration6b_spatial/latest_table.csv) and
does not create independent repetitions.

`Completion` is `true / recovered-known` for Frame and 3C over the nominal ten
trajectories. `Init` and `Infra` are selected-attempt counts `Frame / 3C`. A valid
RMS pair can be infrastructure-marked; its original runtime classification remains
unchanged. Full per-run endpoint error, progress, path length, Segment Max, outcome
and coverage are in [`per_run.csv`](../../../docs/data/iteration6b_spatial/per_run.csv)
[`per_trajectory_pairs.csv`](../../../docs/data/iteration6b_spatial/per_trajectory_pairs.csv).

| Policy | Bank | Factor | Pairs | U F / 3C | U Gain (Wins) | S F / 3C | S Gain (Wins) | T F / 3C | T Gain | Completion F / 3C | Init F / 3C | Infra F / 3C |
| --- | --- | ---: | ---: | --- | --- | --- | --- | --- | ---: | --- | --- | --- |
| ALMI | directional | 1.0 | 10/10 | 8.201 / 8.781 | -0.580 (5/10) | 9.196 / 10.106 | -0.910 (5/10) | 18.202 / 18.805 | -0.603 | 6/10 / 3/10 | 0 / 0 | 0 / 1 |
| ALMI | directional | 0.8 | 10/10 | 7.766 / 8.214 | -0.449 (3/10) | 8.112 / 8.885 | -0.774 (4/10) | 22.108 / 22.152 | -0.044 | 6/10 / 3/10 | 0 / 0 | 0 / 1 |
| ALMI | directional | 0.6 | 10/10 | 7.947 / 8.052 | -0.105 (4/10) | 7.499 / 7.834 | -0.335 (4/10) | 30.397 / 29.354 | +1.043 | 7/10 / 2/10 | 0 / 0 | 0 / 0 |
| ALMI | directional | 0.5 | 10/10 | 10.399 / 10.420 | -0.021 (3/10) | 9.493 / 9.597 | -0.104 (3/10) | 37.619 / 36.633 | +0.986 | 4/10 / 4/10 | 0 / 0 | 0 / 0 |
| ALMI | directional | 0.4 | 10/10 | 14.124 / 14.350 | -0.226 (5/10) | 12.632 / 12.839 | -0.208 (4/10) | 50.251 / 48.099 | +2.152 | 0/10 / 1/10 | 0 / 0 | 0 / 0 |
| ALMI | directional | 0.3 | 10/10 | 18.808 / 19.502 | -0.694 (3/10) | 15.896 / 16.544 | -0.649 (3/10) | 69.139 / 67.635 | +1.504 | 0/10 / 0/10 | 0 / 0 | 0 / 0 |
| ALMI | overhang | 1.0 | 10/10 | 8.813 / 10.316 | -1.503 (2/10) | 8.813 / 10.839 | -2.026 (2/10) | 16.319 / 18.066 | -1.747 | 7/10 / 7/10 | 0 / 0 | 0 / 2 |
| ALMI | overhang | 0.8 | 10/10 | 8.573 / 9.064 | -0.491 (6/10) | 8.481 / 9.336 | -0.855 (3/10) | 19.232 / 19.517 | -0.285 | 8/10 / 7/10 | 0 / 0 | 0 / 0 |
| ALMI | overhang | 0.6 | 10/10 | 8.026 / 8.430 | -0.404 (6/10) | 7.733 / 8.281 | -0.548 (5/10) | 24.313 / 23.709 | +0.604 | 6/10 / 6/10 | 0 / 0 | 0 / 1 |
| ALMI | overhang | 0.5 | 10/10 | 8.346 / 9.332 | -0.985 (2/10) | 7.855 / 8.790 | -0.935 (2/10) | 28.788 / 27.591 | +1.197 | 5/10 / 3/10 | 0 / 0 | 0 / 1 |
| ALMI | overhang | 0.4 | 10/10 | 9.591 / 10.256 | -0.665 (3/10) | 8.827 / 9.366 | -0.538 (3/10) | 36.029 / 33.982 | +2.047 | 3/10 / 3/10 | 0 / 0 | 0 / 0 |
| ALMI | overhang | 0.3 | 10/10 | 11.228 / 12.812 | -1.584 (4/10) | 10.266 / 11.502 | -1.237 (3/10) | 47.731 / 45.250 | +2.481 | 1/10 / 1/10 | 0 / 0 | 0 / 1 |
| FAME | directional | 1.0 | 10/10 | 10.632 / 11.725 | -1.092 (3/10) | 11.898 / 13.132 | -1.234 (3/10) | 20.201 / 21.084 | -0.883 | 2/10 / 2/10 | 0 / 0 | 0 / 0 |
| FAME | directional | 0.8 | 10/10 | 9.462 / 10.656 | -1.194 (3/10) | 10.540 / 11.756 | -1.216 (2/10) | 21.778 / 22.735 | -0.957 | 2/10 / 2/10 | 0 / 0 | 0 / 0 |
| FAME | directional | 0.6 | 10/10 | 9.084 / 10.011 | -0.926 (3/10) | 9.535 / 10.411 | -0.876 (3/10) | 28.773 / 29.405 | -0.631 | 5/10 / 3/10 | 0 / 0 | 0 / 0 |
| FAME | directional | 0.5 | 10/10 | 9.148 / 10.081 | -0.933 (3/10) | 9.394 / 10.282 | -0.888 (2/10) | 35.067 / 35.377 | -0.309 | 4/10 / 2/10 | 0 / 0 | 0 / 1 |
| FAME | directional | 0.4 | 10/10 | 12.465 / 13.906 | -1.441 (2/10) | 11.616 / 12.836 | -1.219 (2/10) | 47.354 / 47.359 | -0.005 | 2/10 / 1/10 | 0 / 0 | 0 / 0 |
| FAME | directional | 0.3 | 10/10 | 16.556 / 17.485 | -0.929 (4/10) | 14.067 / 14.909 | -0.843 (4/10) | 65.968 / 65.795 | +0.172 | 0/10 / 0/10 | 0 / 0 | 0 / 0 |
| FAME | overhang | 1.0 | 9/10 | 9.069 / 11.264 | -2.195 (2/9) | 10.767 / 12.859 | -2.092 (3/9) | 15.668 / 17.735 | -2.067 | 2/10 / 3/9 | 0 / 1 | 0 / 0 |
| FAME | overhang | 0.8 | 9/10 | 7.940 / 8.678 | -0.738 (3/9) | 8.956 / 9.596 | -0.640 (4/9) | 17.350 / 16.885 | +0.465 | 2/9 / 5/10 | 1 / 0 | 0 / 0 |
| FAME | overhang | 0.6 | 9/10 | 7.479 / 7.377 | +0.102 (3/9) | 7.455 / 7.737 | -0.282 (3/9) | 21.857 / 20.892 | +0.965 | 8/9 / 6/10 | 1 / 0 | 0 / 0 |
| FAME | overhang | 0.5 | 10/10 | 6.827 / 7.111 | -0.285 (4/10) | 6.552 / 6.889 | -0.337 (2/10) | 25.926 / 25.192 | +0.734 | 8/10 / 5/10 | 0 / 0 | 0 / 0 |
| FAME | overhang | 0.4 | 9/10 | 10.095 / 9.131 | +0.964 (5/9) | 9.324 / 8.387 | +0.937 (4/9) | 34.963 / 32.079 | +2.885 | 4/9 / 5/10 | 1 / 0 | 0 / 1 |
| FAME | overhang | 0.3 | 10/10 | 9.591 / 10.490 | -0.899 (3/10) | 8.899 / 9.553 | -0.654 (4/10) | 45.102 / 43.454 | +1.648 | 2/10 / 4/10 | 0 / 0 | 0 / 0 |

The primary spatial conclusion is unambiguous at the cell level: Frame has lower
mean Segment Spatial RMS in **23/24** cells and lower Uniform Spatial RMS in
**22/24** cells. The main positive spatial exception is FAME overhang at `0.4x`
(`+0.937 mm` Segment; `+0.964 mm` Uniform). FAME overhang `0.6x` is a small
Uniform-only positive mean (`+0.102 mm`). The latest-attempt sensitivity keeps the
same interpretation: FAME overhang `0.4x` stays positive spatially, but no general
spatial 3C advantage appears.

Uniform RMS at 1001 samples was recomputed at 2002 samples for all 479 recoverable
curves. The maximum absolute change is `0.027246 mm`, mean absolute change is
`0.004146 mm`, and no paired win/loss/tie changed in either primary or latest view.
The resolution audit is in
[`paired_uniform_resolution_table.csv`](../../../docs/data/iteration6b_spatial/paired_uniform_resolution_table.csv).

## 5. Ideal / Timed RMS: Secondary Result

3C has a positive mean Ideal / Timed RMS gain in **14/24** primary cells. This is
meaningful as scheduled-clock task tracking: it includes reduced phase/servo lag
where present. It is not a general geometric-path result. For example, ALMI
are `-0.208 mm` and `-0.226 mm`. Faster timed tracking, endpoint proximity, or
progress can coexist with a slightly less straight measured path.

No statistical test treats repeated factors of the same trajectory as independent
samples. The old pooled-speed significance claims are superseded and are not part
of the final inference.

## 6. Mechanism Diagnostics

Base-induced wrist displacement, peak pelvis tilt and pelvis translation/drift are
retained as diagnostics. They help describe whether a controller trades rotational
and translational base motion, but they do not replace the fixed-segment spatial
task metric and do not establish causation. The saved-line results therefore do not
support a claim that 3C generally improves geometry through one identified base
mechanism. All detailed diagnostic values remain in the recorded summaries and
per-run export; they are not pooled into a new efficacy statistic.

## 7. Existing Challenge-Group Context

The separate nominal FAME `new_challenge40` endpoint sweep remains the strongest
categorical evidence. On the same 40 endpoints, Frame has 12 falls and frozen 3C
has 7. The paired transition view has **six improvements and zero regressions**:
four `fall -> drift`, one `fall -> stable`, and one `drift -> stable`. ALMI has one
`drift -> stable` improvement and zero regressions, with both controllers keeping
all endpoints up. These challenge results are contextual comparison, not a speed

Challenge evidence remains under:

- `runs/challenge_sweep/20260909_205330_iter6b_new_challenge40_frame_vs_3c_fame/`.
- `runs/challenge_sweep/20260909_214657_iter6b_new_challenge40_frame_vs_3c_almi/`.

## 8. Limitations

- The saved-line comparison is a fixed-grid descriptive experiment, not a general
  proof of counter-balance benefit or a usable-envelope study.
- A recorded polyline cannot reveal unobserved between-sample excursions. Uniform
  resampling adds no physical evidence.
- Completion remains low at fast factors in several cells; a small spatial RMS does
  not certify a successful manipulation task.
- Nine infrastructure-marked attempts have metric-valid physics traces but remain
  runtime/infrastructure failures. Offline recoverability is not runtime health.
- Eight FAME `left_overhang_forward` initialization failures have no stable-standing
  capture, so no spatial metric is manufactured for them.
- Retries and repeated factors are not independent observations. No pooled-speed
  significance claim is made.

## 9. Final 6B Conclusions

Iteration 6B is closed. The final claim is intentionally narrow:

- Segment and Uniform Spatial RMS do **not** show a general geometric tracking
  advantage for frozen 3C over Frame on the rebuilt saved-line banks.
- Ideal / Timed RMS retains some fast-factor advantages, but these are
  timing-sensitive and must not be presented as generally straighter paths.
- The saved-line benchmark is supporting/descriptive evidence, not the primary
  proof of counter-balance benefit.
- The strongest existing result remains the separate FAME challenge outcome:
  six paired improvements, zero regressions, falls `12 -> 7`.

## 10. Iteration 6C Handoff

**Iteration 6C:** challenge-group manipulation-speed envelope, Frame versus frozen
robust 3C, on ALMI-Manip-v2 and FAME.

Before that experiment, document an applicable moving-arm endpoint executability
contract. `overhang_forward` repair, runtime-continuity repair, trajectory redesign
and controller tuning are deferred. H2 is removed from the active future-work path.
No new 6C simulation is started by this closeout.

## Superseded Development History

Earlier 6B work explored timing-gate behavior, pre-rebuild banks, overhang-only
speed sweeps, overlay/video inspection, and pooled-speed tests. It is retained only
as concise methodological history and archive provenance. The pre-rebuild 20260910/11
campaign is under `runs/archive/traj_sweep_prerebuild_iter6b/`; it contains useful
negative and regression evidence but no longer contributes to final aggregates.
The pre-rebuild analysis/visual capsules are moved under
`runs/archive/traj_sweep_prerebuild_iter6b/references/` with an explicit cleanup
manifest. Old claims of general fast-speed spatial
improvement, H2 selection uncertainty, solved timing-gate blockers, debugging
narratives, and proposals to collect runs merely to reach `p < 0.05` are superseded.
