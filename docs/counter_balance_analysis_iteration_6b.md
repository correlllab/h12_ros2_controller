# Counter-Balance Analysis: Iteration 6B

Status: **CLOSED / FINALIZED** on 2026-09-13.

The targeted overhang-pair correction is tracked in
[Iteration 6D Preparation](counter_balance_iteration_6d_preparation.md). It is complete: its 48
replacement runs are merged into the existing overhang sweep roots, and the
numeric tables below are already refreshed from the current bank, without
rerunning the unaffected trajectories.

## 1. Final Experimental Question And Setup

This final report asks whether frozen robust 3C (`counter_ddp_velocity_robust`)
improves saved straight-line manipulation tracking relative to Frame (`frame_task`)
on the current rebuilt banks. It covers ALMI-Manip-v2 and FAME, the ten-trajectory
`directional_traj` and `overhang_traj` banks, and duration factors
`1.0/0.8/0.6/0.5/0.4/0.3`. Smaller factors are faster. All controllers, policies,
banks, runtime thresholds, classifications, and recorded evidence are frozen.

The canonical grid contains 24 roots, 480 first attempts, and one retained retry.
The machine-readable 481-record source is
[`canonical_inventory.csv`](../../../docs/data/iteration6b_spatial/canonical_inventory.csv).
Every record has policy, bank, trajectory, controller, factor, attempt, path,
current bank/geometry hash, recorded classification, metric recoverability,
infrastructure/init flags and evidence status. Tables derive from this inventory
before pairing. The report generator is:

```bash
uv run python -m h12_zmp_benchmark.experiment.iteration6b_spatial_report
```

It is offline-only: it reconstructs recorded physics metrics and writes derived
exports under `docs/data/iteration6b_spatial/`. It does not simulate, replay,
repair, reclassify, tune, or modify evidence. The audit confirms all 481 records
pass current-bank hash gating, all 481 recoverable Ideal / Timed RMS values match
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

FAME `left_overhang_forward` was repaired in
[Iteration 6D Preparation](counter_balance_iteration_6d_preparation.md). Its earlier
initialization failures are superseded by an inward mirrored `overhang_forward`
pair across both policies and controllers. Repair is not part of the current
spatial conclusion, and the replacement cells are documented separately.

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
distance over chronological physics samples, so it remains sample/dwell weighted.
Uniform Spatial RMS removes that dwell weighting: it preserves the
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
and the paired audit is in
[`per_trajectory_pairs.csv`](../../../docs/data/iteration6b_spatial/per_trajectory_pairs.csv).

| Policy | Bank | Factor | Pairs | U F / 3C | U Gain (Wins) | S F / 3C | S Gain (Wins) | T F / 3C | T Gain | Completion F / 3C | Init F / 3C | Infra F / 3C |
| --- | --- | ---: | ---: | --- | --- | --- | --- | --- | ---: | --- | --- | --- |
| ALMI | directional | 1.0 | 10/10 | 8.201 / 8.781 | -0.580 (5/10) | 9.196 / 10.106 | -0.910 (5/10) | 18.202 / 18.805 | -0.603 | 6/10 / 3/10 | 0 / 0 | 0 / 1 |
| ALMI | directional | 0.8 | 10/10 | 7.766 / 8.214 | -0.449 (3/10) | 8.112 / 8.885 | -0.774 (4/10) | 22.108 / 22.152 | -0.044 | 6/10 / 3/10 | 0 / 0 | 0 / 1 |
| ALMI | directional | 0.6 | 10/10 | 7.947 / 8.052 | -0.105 (4/10) | 7.499 / 7.834 | -0.335 (4/10) | 30.397 / 29.354 | +1.043 | 7/10 / 2/10 | 0 / 0 | 0 / 0 |
| ALMI | directional | 0.5 | 10/10 | 10.399 / 10.420 | -0.021 (3/10) | 9.493 / 9.597 | -0.104 (3/10) | 37.619 / 36.633 | +0.986 | 4/10 / 4/10 | 0 / 0 | 0 / 0 |
| ALMI | directional | 0.4 | 10/10 | 14.124 / 14.350 | -0.226 (5/10) | 12.632 / 12.839 | -0.208 (4/10) | 50.251 / 48.099 | +2.152 | 0/10 / 1/10 | 0 / 0 | 0 / 0 |
| ALMI | directional | 0.3 | 10/10 | 18.808 / 19.502 | -0.694 (3/10) | 15.896 / 16.544 | -0.649 (3/10) | 69.139 / 67.635 | +1.504 | 0/10 / 0/10 | 0 / 0 | 0 / 0 |
| ALMI | overhang | 1.0 | 10/10 | 8.992 / 10.308 | -1.315 (2/10) | 9.002 / 10.833 | -1.831 (3/10) | 16.628 / 18.392 | -1.764 | 7/10 / 7/10 | 0 / 0 | 0 / 2 |
| ALMI | overhang | 0.8 | 10/10 | 8.787 / 9.169 | -0.382 (5/10) | 8.686 / 9.491 | -0.805 (3/10) | 19.738 / 19.898 | -0.160 | 8/10 / 7/10 | 0 / 0 | 0 / 0 |
| ALMI | overhang | 0.6 | 10/10 | 8.129 / 8.607 | -0.478 (5/10) | 7.862 / 8.476 | -0.614 (5/10) | 24.982 / 24.294 | +0.688 | 6/10 / 6/10 | 0 / 0 | 0 / 1 |
| ALMI | overhang | 0.5 | 10/10 | 8.412 / 9.306 | -0.893 (2/10) | 7.946 / 8.804 | -0.859 (2/10) | 29.436 / 28.170 | +1.266 | 4/10 / 3/10 | 0 / 0 | 0 / 1 |
| ALMI | overhang | 0.4 | 10/10 | 10.199 / 10.641 | -0.441 (4/10) | 9.444 / 9.786 | -0.342 (4/10) | 37.292 / 35.049 | +2.243 | 3/10 / 2/10 | 0 / 0 | 0 / 0 |
| ALMI | overhang | 0.3 | 10/10 | 12.875 / 13.918 | -1.043 (5/10) | 11.779 / 12.611 | -0.833 (5/10) | 50.095 / 47.696 | +2.399 | 1/10 / 1/10 | 0 / 0 | 0 / 2 |
| FAME | directional | 1.0 | 10/10 | 10.632 / 11.725 | -1.092 (3/10) | 11.898 / 13.132 | -1.234 (3/10) | 20.201 / 21.084 | -0.883 | 2/10 / 2/10 | 0 / 0 | 0 / 0 |
| FAME | directional | 0.8 | 10/10 | 9.462 / 10.656 | -1.194 (3/10) | 10.540 / 11.756 | -1.216 (2/10) | 21.778 / 22.735 | -0.957 | 2/10 / 2/10 | 0 / 0 | 0 / 0 |
| FAME | directional | 0.6 | 10/10 | 9.084 / 10.011 | -0.926 (3/10) | 9.535 / 10.411 | -0.876 (3/10) | 28.773 / 29.405 | -0.631 | 5/10 / 3/10 | 0 / 0 | 0 / 0 |
| FAME | directional | 0.5 | 10/10 | 9.148 / 10.081 | -0.933 (3/10) | 9.394 / 10.282 | -0.888 (2/10) | 35.067 / 35.377 | -0.309 | 4/10 / 2/10 | 0 / 0 | 0 / 1 |
| FAME | directional | 0.4 | 10/10 | 12.465 / 13.906 | -1.441 (2/10) | 11.616 / 12.836 | -1.219 (2/10) | 47.354 / 47.359 | -0.005 | 2/10 / 1/10 | 0 / 0 | 0 / 0 |
| FAME | directional | 0.3 | 10/10 | 16.556 / 17.485 | -0.929 (4/10) | 14.067 / 14.909 | -0.843 (4/10) | 65.968 / 65.795 | +0.172 | 0/10 / 0/10 | 0 / 0 | 0 / 0 |
| FAME | overhang | 1.0 | 10/10 | 8.893 / 10.671 | -1.778 (3/10) | 10.585 / 12.343 | -1.758 (4/10) | 15.861 / 16.884 | -1.023 | 1/10 / 3/10 | 0 / 0 | 0 / 1 |
| FAME | overhang | 0.8 | 10/10 | 7.957 / 9.006 | -1.050 (3/10) | 8.996 / 9.894 | -0.898 (4/10) | 17.771 / 17.918 | -0.147 | 2/10 / 4/10 | 0 / 0 | 0 / 0 |
| FAME | overhang | 0.6 | 10/10 | 7.706 / 6.955 | +0.751 (4/10) | 7.737 / 7.474 | +0.263 (4/10) | 22.977 / 20.566 | +2.411 | 7/10 / 7/10 | 0 / 0 | 0 / 1 |
| FAME | overhang | 0.5 | 10/10 | 7.027 / 7.527 | -0.499 (4/10) | 6.810 / 7.356 | -0.545 (2/10) | 26.364 / 25.850 | +0.515 | 8/10 / 5/10 | 0 / 0 | 0 / 0 |
| FAME | overhang | 0.4 | 10/10 | 9.904 / 9.453 | +0.450 (4/10) | 9.196 / 8.792 | +0.404 (3/10) | 35.139 / 33.165 | +1.974 | 4/10 / 4/10 | 0 / 0 | 0 / 1 |
| FAME | overhang | 0.3 | 10/10 | 11.577 / 12.439 | -0.862 (3/10) | 10.717 / 11.332 | -0.614 (4/10) | 47.382 / 46.013 | +1.369 | 2/10 / 4/10 | 0 / 0 | 0 / 1 |

The primary spatial conclusion is unchanged in direction: Frame has lower mean
Segment Spatial RMS in **22/24** cells and lower Uniform Spatial RMS in **22/24**
cells. The positive spatial exceptions are now two FAME overhang cells, `0.6x`
(Segment `+0.263 mm`, Uniform `+0.751 mm`) and `0.4x` (Segment `+0.404 mm`,
Uniform `+0.450 mm`). Each has only four paired 3C wins of ten, so the positive mean
is not a majority. The latest-attempt sensitivity is identical because the 48
replacement cells are fresh first attempts and the single remaining retry is
`FAME right_overhang_inner_forward` 3C, unrelated to this pair.

Uniform RMS at 1001 samples was recomputed at 2002 samples for all 481 recoverable
curves. The maximum absolute change is `0.027246 mm`, mean absolute change is
`0.004146 mm`, and no paired win/loss/tie changed in either primary or latest view.
The resolution audit is in
[`paired_uniform_resolution_table.csv`](../../../docs/data/iteration6b_spatial/paired_uniform_resolution_table.csv).

## 5. Ideal / Timed RMS: Secondary Result

3C has a positive mean Ideal / Timed RMS gain in **13/24** primary cells. This is
meaningful as scheduled-clock task tracking: it includes reduced phase/servo lag
where present. It is not a general geometric-path result. For example, ALMI
directional at `0.4x` has `+2.152 mm` timed gain while Segment and Uniform gains
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
per-run export. Section 7 now evaluates base motion itself on the same grid, as a
descriptive comparison of recorded base kinematics rather than a causal efficacy
statistic for the spatial result.

## 7. Base-Motion Evaluation

This section asks one narrow question: while executing the prescribed straight-line
trajectories, does frozen robust 3C reduce whole-body base motion relative to Frame?
It is an evaluation of base motion only. It makes no causal claim and says nothing
about end-effector path geometry, which Section 4 settles separately.

### Scope And Method

The evaluation reuses the same canonical current-bank grid as Section 4 and adds no
runs: FAME and ALMI-Manip-v2, Frame and frozen robust 3C, `directional_traj` and
`overhang_traj`, ten trajectories per bank, duration factors
`1.0/0.8/0.6/0.5/0.4/0.3`. It uses the finalized Iteration 6D `overhang_forward`
replacement pair and no superseded overhang recording; every consumed record passes
the same current-bank hash gate, 480/480. That is 480 first attempts, 240 matched
pairs, 24 policy/bank/factor cells of ten nominal pairs each. No simulation was run
and no controller, policy, trajectory, threshold or classification was modified.

Metrics are reduced from the recorded physics traces, aligned to the recorded
`arm_motion_released` event rather than a fitted motion onset. The primary window
runs from that release to the shorter recorded end of the two runs in a pair, so
both members of every pair are reduced over an identical release-relative window
covering the prescribed move and its hold. The startup and standing-qualification
transient before release is excluded: it precedes the commanded trajectory and
differs by policy startup rather than by controller.

- **Primary:** peak base tilt, the maximum absolute gravity tilt of the pelvis in
    degrees.
- **Secondary:** RMS base tilt; peak and RMS orientation drift, the geodesic
    deviation of pelvis orientation from its own pose at release; and pelvis
    translation, the maximum horizontal pelvis displacement from its release
    position in centimetres.
- **Diagnostic only:** base-induced wrist displacement, the separation between the
    measured wrist position and the frozen-pelvis forward kinematics. It is not a
    headline metric.

Gains are `Frame - 3C` for every metric, so a positive gain always means less base
motion under 3C. Wins count the matched trajectories in a cell with a strictly
positive unrounded gain. Pairing is trajectory-by-trajectory within a cell; all 240
pairs are matched and none is missing. Runtime flags stay separate from the physical
metric: no trial is discarded or relabelled, and the 13 infrastructure-flagged first
attempts, all on the frozen 3C path, are retained in the means and carried as their
own columns in the exports. The recorded physical outcomes of the 480 runs are 467
stable and 13 infrastructure, with zero initialization failures and zero falls.

### Per-Cell Results

Peak tilt in degrees, pelvis translation in centimetres, values `Frame / 3C`.

| Policy | Group | Factor | Pairs | Peak Tilt F / 3C | Gain | 3C Wins | Pelvis Translation F / 3C |
| --- | --- | ---: | ---: | --- | ---: | ---: | --- |
| FAME | directional | 1 | 10/10 | 3.127 / 3.254 | -0.127 | 5/10 | 1.878 / 2.122 |
| FAME | directional | 0.8 | 10/10 | 3.212 / 3.263 | -0.052 | 6/10 | 1.940 / 2.173 |
| FAME | directional | 0.6 | 10/10 | 3.317 / 3.324 | -0.007 | 7/10 | 2.018 / 2.231 |
| FAME | directional | 0.5 | 10/10 | 3.375 / 3.322 | +0.053 | 7/10 | 2.057 / 2.219 |
| FAME | directional | 0.4 | 10/10 | 3.422 / 3.400 | +0.021 | 7/10 | 2.095 / 2.261 |
| FAME | directional | 0.3 | 10/10 | 3.422 / 3.449 | -0.027 | 5/10 | 2.025 / 2.179 |
| FAME | overhang | 1 | 10/10 | 3.454 / 3.584 | -0.130 | 3/10 | 1.827 / 2.265 |
| FAME | overhang | 0.8 | 10/10 | 3.570 / 3.670 | -0.100 | 4/10 | 1.975 / 2.376 |
| FAME | overhang | 0.6 | 10/10 | 3.648 / 3.732 | -0.084 | 6/10 | 2.045 / 2.391 |
| FAME | overhang | 0.5 | 10/10 | 3.686 / 3.727 | -0.041 | 5/10 | 2.079 / 2.496 |
| FAME | overhang | 0.4 | 10/10 | 3.794 / 3.809 | -0.016 | 5/10 | 2.100 / 2.461 |
| FAME | overhang | 0.3 | 10/10 | 3.722 / 3.718 | +0.004 | 5/10 | 2.104 / 2.499 |
| ALMI-Manip-v2 | directional | 1 | 10/10 | 1.418 / 1.381 | +0.037 | 5/10 | 1.582 / 1.629 |
| ALMI-Manip-v2 | directional | 0.8 | 10/10 | 1.516 / 1.436 | +0.080 | 5/10 | 1.734 / 1.746 |
| ALMI-Manip-v2 | directional | 0.6 | 10/10 | 1.657 / 1.493 | +0.164 | 6/10 | 1.954 / 1.854 |
| ALMI-Manip-v2 | directional | 0.5 | 10/10 | 1.723 / 1.535 | +0.188 | 6/10 | 2.040 / 1.928 |
| ALMI-Manip-v2 | directional | 0.4 | 10/10 | 1.791 / 1.544 | +0.247 | 6/10 | 2.102 / 1.934 |
| ALMI-Manip-v2 | directional | 0.3 | 10/10 | 1.841 / 1.588 | +0.253 | 7/10 | 2.163 / 1.953 |
| ALMI-Manip-v2 | overhang | 1 | 10/10 | 1.612 / 2.030 | -0.418 | 4/10 | 1.280 / 1.671 |
| ALMI-Manip-v2 | overhang | 0.8 | 10/10 | 1.604 / 2.006 | -0.402 | 4/10 | 1.298 / 1.665 |
| ALMI-Manip-v2 | overhang | 0.6 | 10/10 | 1.593 / 1.949 | -0.356 | 4/10 | 1.323 / 1.644 |
| ALMI-Manip-v2 | overhang | 0.5 | 10/10 | 1.606 / 1.897 | -0.292 | 4/10 | 1.311 / 1.569 |
| ALMI-Manip-v2 | overhang | 0.4 | 10/10 | 1.613 / 1.853 | -0.240 | 4/10 | 1.278 / 1.520 |
| ALMI-Manip-v2 | overhang | 0.3 | 10/10 | 1.604 / 1.791 | -0.187 | 5/10 | 1.322 / 1.533 |

### Aggregate Result

Grand means are the equally weighted mean of the 24 cell means. `Cells` counts the
cells with a positive mean gain; `Pairs` counts the matched trajectories with a
positive gain out of 240.

| Metric | Unit | Cells | Pairs | Frame / 3C grand mean | Mean cell gain | Cell gain range |
| --- | --- | ---: | ---: | --- | ---: | --- |
| Peak base tilt (primary) | deg | 9/24 | 125/240 | 2.5552 / 2.6149 | -0.0597 | -0.4182 .. +0.2530 |
| RMS base tilt | deg | 0/24 | 97/240 | 1.8872 / 1.9312 | -0.0440 | -0.1127 .. -0.0028 |
| Peak orientation drift | deg | 10/24 | 142/240 | 1.8133 / 1.8741 | -0.0607 | -0.3890 .. +0.3868 |
| RMS orientation drift | deg | 12/24 | 146/240 | 1.3851 / 1.3583 | +0.0267 | -0.1455 .. +0.2205 |
| Pelvis translation | cm | 4/24 | 122/240 | 1.8138 / 2.0133 | -0.1995 | -0.4388 .. +0.2102 |
| Base-induced wrist (diagnostic) | mm | 4/24 | 111/240 | 32.039 / 35.443 | -3.4036 | -7.5663 .. +3.4060 |

**3C lowers mean peak base tilt in 9 of the 24 cells, and lowers mean pelvis
translation in 4 of the 24.** Neither is a majority, and the grand means move the
wrong way for both: peak tilt is `0.060 deg` higher under 3C and pelvis translation
is `0.199 cm` higher. RMS base tilt favours Frame in every one of the 24 cells.
Only RMS orientation drift is marginally positive, at 12/24 cells and `+0.027 deg`.

The direction is **not** consistent across policies or banks. Splitting the primary
metric by block makes the pattern explicit:

| Block | Peak tilt cells favouring 3C | Peak orientation drift | RMS orientation drift | Pelvis translation |
| --- | ---: | ---: | ---: | ---: |
| FAME directional | 2/6 | 4/6 | 6/6 | 0/6 |
| FAME overhang | 1/6 | 0/6 | 0/6 | 0/6 |
| ALMI-Manip-v2 directional | 6/6 | 6/6 | 6/6 | 4/6 |
| ALMI-Manip-v2 overhang | 0/6 | 0/6 | 0/6 | 0/6 |

ALMI directional is the one block where 3C consistently reduces base motion, and it
is also the only block where the advantage grows with commanded speed, from
`+0.037 deg` at factor 1.0 to `+0.253 deg` at 0.3. ALMI overhang is the consistent
opposite, `-0.418 deg` at 1.0 narrowing to `-0.187 deg` at 0.3. FAME is close to a
wash in both banks: every FAME cell gain lies within `+/-0.130 deg` on a mean peak
tilt of roughly `3.1` to `3.8 deg`, which is under 4 percent of the quantity being
compared.

Magnitude matters as much as the counts here. The largest single-cell peak-tilt
effect in either direction is `0.418 deg`, the mean absolute cell gain is
`0.147 deg` and the median `0.113 deg`. The whole grid stands with peak tilts
between about `1.4` and `3.8 deg` and horizontal pelvis excursions of roughly
`1.3` to `2.5 cm`.
These are small absolute motions in a grid with no falls, so the comparison is
between two controllers that both keep the base nearly still.

### Window Robustness

Restricting the same reduction to the commanded move alone, from release to the
recorded line end and excluding the hold, moves several counts across the majority
line: peak tilt becomes 17/24 cells and `+0.077 deg`, peak orientation drift 15/24
and `+0.040 deg`, RMS base tilt 14/24 and `+0.032 deg`, RMS orientation drift 14/24
and `+0.013 deg`, while pelvis translation stays negative at 9/24 and `-0.051 cm`
and the wrist diagnostic at 6/24 and `-0.811 mm`. The sign of the headline tilt
result therefore depends on whether the ten-second hold is included. This is
reported rather than resolved by choosing the more favourable window: a base-motion
benefit that appears during the commanded move and does not survive the subsequent
hold is not a general base-motion reduction, and the move-and-hold window is the
one that matches how the rest of Iteration 6B measures these trials. Both windows
are exported in full.

### Figure

![Mean peak base tilt](../../../figures/iteration6b_base_motion/peak_base_tilt_2x2.png)

Rows are policies, columns are trajectory groups, the x-axis runs from duration
factor 1.0 to 0.3 so faster motion is to the right, and the y-axis is mean peak base
tilt in degrees. Panels within a policy row share a zero-based y-axis so the two
banks are directly comparable; the two rows are deliberately not shared, because
FAME peak tilts are roughly double ALMI's and a common scale would render the ALMI
panels unreadable. The axes are zero-based rather than zoomed so that sub-degree
differences are not visually inflated.

### What This Does And Does Not Show

- It does not show that counter-balancing reduces base motion during these
    standardized manipulation trajectories. On the primary metric and window the
    result is a minority of cells and a slightly negative grand mean.
- It does show one bounded positive case, ALMI-Manip-v2 on the directional bank,
    which is consistent across all six duration factors and all four orientation and
    tilt metrics, and which strengthens as commanded duration shortens.
- No causal mechanism is claimed. These are recorded kinematic summaries of the
    base, not an identification of why either controller moves the base as it does.
- No end-effector geometry claim follows from them. The base-induced wrist
    displacement column is retained as a diagnostic only, and Section 4 remains the
    authority on measured path straightness.
- Peak absolute tilt is close to RMS absolute tilt throughout, which means these
    trials hold a nearly steady tilted posture rather than showing a transient
    excursion. The orientation-drift metrics, which are referenced to each run's own
    pose at release, are the cleaner motion-relative readings and are reported
    alongside the requested primary metric for that reason.

### Artifacts

Regenerate with:

```bash
uv run python -m h12_zmp_benchmark.experiment.iteration6b_base_motion
```

The reducer is offline: it reads recorded traces and the canonical inventory, and it
writes only the exports below.

- [Per-run metrics](../../../docs/data/iteration6b_base_motion/per_run.csv), 480 rows with both windows, flags and hash status.
- [Matched pairs](../../../docs/data/iteration6b_base_motion/pairs.csv), 240 rows with per-metric gains and win flags.
- [Cell summary](../../../docs/data/iteration6b_base_motion/cell_summary.csv), the 24 cells behind the table above.
- [Move-only pairs](../../../docs/data/iteration6b_base_motion/pairs_move_only.csv) and [move-only cells](../../../docs/data/iteration6b_base_motion/cell_summary_move_only.csv), the window robustness check.
- [Aggregate summary](../../../docs/data/iteration6b_base_motion/summary.json), the exact values quoted in this section.
- [Rendered table](../../../docs/data/iteration6b_base_motion/tables.txt).
- Figure: `figures/iteration6b_base_motion/peak_base_tilt_2x2.png` and `peak_base_tilt_2x2_no_title.png`.

## 8. Paper-Facing Interpretation

Section 7 reports that frozen robust 3C does not reduce base motion across the grid
as a whole. This section asks the sharper question the paper needs: given that
aggregate, is there a coherent and statistically defensible Frame-versus-3C story in
the trajectory groups? It uses the same finalized current-bank evidence, drops no
cell, changes no metric definition and adds no run.

**Scope decision, 2026-09-14.** The paper's *simulation* analysis is scoped to the
stability/challenge target sweep. The directional trajectory evidence is carried by
the real-robot runs instead, because the real directional batch gives more consistent
results; that evidence lives in
[the round 2 real run sweep](../../../docs/real_run_sweep_round2.md) and is not
restated here. The simulation trajectory-group analysis below is therefore **retained
supporting evidence and a scope boundary, not a headline paper claim**. It is kept in
full because it is finalized, because it bounds where counter-balancing helps, and
because the analysis conclusion should not change when the presentation scope does.
Nothing in this section was recomputed or re-scoped to suit that decision.

### Statistical Treatment

All 240 matched pairs are reconstructed trajectory by trajectory across 2 policies,
2 banks, 6 duration factors and 10 trajectories per cell. The six duration factors of
one trajectory are repeated measures of the same geometry, not independent samples,
so every interval below comes from a **cluster bootstrap that resamples whole
trajectories**, 20000 draws over the 40 policy/bank/trajectory clusters. Differences
are `Frame - 3C` throughout, so positive always means less base motion under 3C. A
stratum is called decided only when its 95 percent interval excludes zero.

One correction to how the standing threshold is read. The classifier gates
`max_base_drift` at `0.1 rad = 5.7296 deg`; it does not gate absolute gravity tilt.
The gated quantity is therefore reported here as its own metric, and absolute tilt is
not compared against that threshold.

### Headline Aggregates

| Metric | Stratum | Pairs | Frame / 3C | Difference | 95% CI | Wins | Relative | Verdict |
| --- | --- | ---: | --- | ---: | --- | ---: | ---: | --- |
| RMS orientation drift (deg) | all | 240 | 1.3851 / 1.3583 | +0.0267 | [-0.0428, +0.0919] | 146/240 | +1.93% | inconclusive |
| RMS orientation drift (deg) | directional | 120 | 1.5833 / 1.4499 | **+0.1334** | **[+0.0726, +0.1974]** | 92/120 | +8.43% | **favours 3C** |
| RMS orientation drift (deg) | FAME directional | 60 | 1.5655 / 1.4850 | **+0.0805** | **[+0.0196, +0.1400]** | 42/60 | +5.14% | **favours 3C** |
| RMS orientation drift (deg) | ALMI directional | 60 | 1.6012 / 1.4148 | **+0.1864** | **[+0.0885, +0.2872]** | 50/60 | +11.64% | **favours 3C** |
| RMS orientation drift (deg) | overhang | 120 | 1.1868 / 1.2667 | -0.0800 | [-0.1843, +0.0118] | 54/120 | -6.74% | inconclusive |
| Gated max base drift (deg) | directional | 120 | 1.6636 / 1.4002 | **+0.2633** | **[+0.0716, +0.4589]** | 83/120 | +15.83% | **favours 3C** |
| Gated max base drift (deg) | ALMI directional | 60 | 1.5815 / 1.1703 | **+0.4112** | **[+0.1158, +0.7090]** | 49/60 | +26.00% | **favours 3C** |
| Gated max base drift (deg) | overhang | 120 | 1.6443 / 1.8264 | -0.1821 | [-0.4579, +0.0713] | 62/120 | -11.07% | inconclusive |
| Peak orientation drift (deg) | directional | 120 | 2.0527 / 1.8737 | +0.1791 | [-0.0073, +0.3676] | 94/120 | +8.72% | inconclusive |
| Peak orientation drift (deg) | overhang | 120 | 1.5739 / 1.8744 | -0.3005 | [-0.5565, -0.0712] | 48/120 | -19.09% | favours Frame |
| Peak absolute tilt (deg) | all | 240 | 2.5552 / 2.6149 | -0.0597 | [-0.2384, +0.1075] | 125/240 | -2.34% | inconclusive |
| RMS absolute tilt (deg) | all | 240 | 1.8872 / 1.9312 | -0.0440 | [-0.1019, +0.0116] | 97/240 | -2.33% | inconclusive |
| Pelvis translation (cm) | all | 240 | 1.8138 / 2.0133 | -0.1995 | [-0.4136, +0.0062] | 122/240 | -11.00% | inconclusive |
| Pelvis translation (cm) | overhang | 120 | 1.6618 / 2.0075 | -0.3457 | [-0.6768, -0.0283] | 61/120 | -20.80% | favours Frame |
| Base-induced wrist (mm) | overhang | 120 | 27.3750 / 33.2017 | -5.8267 | [-10.9716, -1.3156] | 51/120 | -21.28% | favours Frame |

The grand aggregate hides a clean conditional structure. **The trajectory group, not
the policy and not the speed, is the variable that decides the sign.**

### The One Decided Positive Result

3C lowers RMS orientation drift on the directional bank by `0.1334 deg`, about
`8.4 percent`, with a 95 percent interval of `[+0.0726, +0.1974]` that excludes zero,
and it does so **in both policies independently**: FAME `+0.0805 deg` at `+5.1 percent`
and ALMI-Manip-v2 `+0.1864 deg` at `+11.6 percent`, each with an interval excluding
zero. The same directional block is also decided on the gated `max_base_drift`
quantity, `+0.2633 deg` at `+15.8 percent`, which matters because that is the
quantity the stability classifier actually uses.

This benefit is **broad rather than subset-driven**, which is the question that
usually kills a result like this:

- 16 of the 20 directional trajectory clusters have a positive mean difference.
- 12 of 20 favour 3C at **all six** duration factors.
- Leave-one-trajectory-out re-estimates of the directional mean span
    `+0.1130` to `+0.1466 deg`. No single trajectory carries the effect, and removing
    any one never changes its sign.

It is also **flat in speed**, not speed-amplified: `+8.43, +7.70, +7.66, +8.55,
+11.56, +6.57 percent` at factors `1.0` through `0.3`. That is a property of the
motion family, not of commanded speed.

On the overhang bank the same metric goes the other way and is not decided,
`-0.0800 deg [-0.1843, +0.0118]`. Overhang is decided *against* 3C on peak
orientation drift, pelvis translation and the wrist diagnostic. The honest reading is
that 3C helps sustained base rotation on extended directional reaches and does not
help, and on some measures hurts, on elevated overhang reaches.

### Rotation-Versus-Translation Trade: Not Supported

The hypothesis that 3C buys lower rotation by permitting more translation is
**refuted by these data**. Across the same paired runs the rotation gain and the
translation gain are *positively* correlated, Spearman `rho = +0.45` overall
(`+0.35` directional, `+0.51` overhang, all `p < 0.001`); rotation gain and peak-tilt
gain likewise `rho = +0.37`. When 3C reduces sustained rotation it tends to reduce
translation as well, not to spend it. Only 53 of 240 pairs show the trade pattern of
better rotation with worse translation, against 93 pairs where both improve. There is
no evidence here that 3C changes the *character* of base motion; it changes the
*amount*, and only on one trajectory family.

### Preservation: Descriptive, Not A Formal Equivalence Claim

No equivalence or non-inferiority margin was pre-specified for base motion, and none
of the available thresholds is a physically justified margin for a *difference*
between two controllers. **No statistical equivalence is therefore claimed.** What
can be stated is descriptive preservation, in physical units:

| Metric | Frame / 3C | Mean absolute difference | vs metric mean | vs across-trajectory SD | vs 0.1 rad gate | Runs over gate F / 3C |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| RMS orientation drift (deg) | 1.3851 / 1.3583 | 0.1905 | 13.8% | 0.28 | 3.32% | 0 / 0 |
| Peak orientation drift (deg) | 1.8133 / 1.8741 | 0.3911 | 21.6% | 0.41 | 6.83% | 0 / 0 |
| RMS absolute tilt (deg) | 1.8872 / 1.9312 | 0.1371 | 7.3% | 0.14 | 2.39% | 0 / 0 |
| Gated max base drift (deg) | 1.6539 / 1.6133 | 0.4587 | 27.7% | 0.47 | 8.01% | 0 / 0 |
| Pelvis translation (cm) | 1.8138 / 2.0133 | 0.5371 | 29.6% | 0.53 | n/a | n/a |

Every one of the 480 runs stayed stable, with zero falls and zero initialization
failures. On the gated quantity no run of either controller reached the `0.1 rad`
threshold; the worst single run is `5.7132 deg`, `99.7 percent` of the gate, and it
is a 3C run, so the margin is preserved but not with room to spare in every case.
Typical controller differences are `0.14` to `0.46 deg`, which is `2.4` to
`8.0 percent` of the gate and `0.14` to `0.47` of the ordinary spread across
trajectories. In other words, switching controller moves base motion by well under
half of what simply changing the trajectory does.

That is a defensible preservation statement in absolute and safety terms. It is not a
claim that every relative change is small: on the overhang bank 3C raises pelvis
translation by `20.8 percent` and the wrist diagnostic by `21.3 percent`, and those
relative changes are not negligible even though their absolute magnitudes are.

### Combined Story With The Challenge Group

The proposed combined sentence is *"counter-balancing reduces severe standing failures
on challenging motions while producing only small changes in base motion during
representative manipulation trajectories."* Against this evidence it is **defensible
with two qualifications**.

The first half is supported by the separate challenge-group result in Section 9:
falls `12 -> 7` on the same 40 FAME endpoints, six paired improvements and zero
regressions. It must carry the reproduction caveat recorded in the
[Iteration 6C analysis](counter_balance_analysis_iteration_6c.md): of the 39 changed
discovery cells, 26 reproduce in 3/3 paired repetitions and 13 do not, so the
headline improvements should be cited as the repeated cases rather than as the whole
transition list.

The second half is quantified above rather than assumed: typical differences of
`0.14` to `0.46 deg` of base rotation, `2.4` to `8.0 percent` of the standing gate,
no run of either controller over that gate and no falls anywhere in 480 runs. The
qualification is that "small" means small in absolute and safety terms, not
uniformly small in relative terms, and that on the directional bank the change is
small *and favourable* while on the overhang bank it is small *and unfavourable*.

### Recommended Figures

**Main paper.** `figures/iteration6b_base_motion/paired_rms_orientation_drift.png`:
the paired-difference distribution of RMS orientation drift, all 240 pairs as points
stratified by policy and trajectory group, with the cluster-bootstrap mean and 95
percent interval on each stratum and a zero reference line. It is preferred to four
line plots because it shows the effect, its spread, its per-pair support and the
group-conditional sign reversal in a single panel, and because it makes the
inconclusive strata visibly inconclusive rather than hiding them inside a mean.

**Appendix.** `figures/iteration6b_base_motion/rotation_translation_tradeoff.png`:
paired rotation gain against paired translation gain by bank, which shows directly
that the two move together and that the tradeoff hypothesis does not hold. The
existing 2x2 mean peak-tilt panel from Section 7 remains useful as a second appendix
figure for absolute magnitudes.

### Draft Result Discussion

> Across 240 matched trajectory pairs spanning two lower-body policies, two motion
> families and six commanded durations, counter-balancing changed whole-body base
> motion only modestly during prescribed straight-line manipulation. Aggregated over
> the full grid the difference in sustained base rotation was not resolvable
> (`+0.027 deg`, 95 percent CI `[-0.043, +0.092]`), and peak pelvis tilt, RMS tilt and
> pelvis translation were likewise undecided. The comparison was however conditional on
> the motion family. On extended directional reaches counter-balancing reduced RMS
> pelvis orientation drift by `0.133 deg`, about `8.4 percent`
> (CI `[+0.073, +0.197]`), and reduced the gated peak base-drift margin by `0.263 deg`,
> about `15.8 percent` (CI `[+0.072, +0.459]`). The effect appeared independently under
> both policies (FAME `+5.1 percent`, ALMI-Manip-v2 `+11.6 percent`, both CIs excluding
> zero), was broad across the motion set rather than concentrated in a few
> trajectories (16 of 20 trajectories favourable, 12 of 20 favourable at all six
> durations, leave-one-trajectory-out estimates spanning `+0.113` to `+0.147 deg`), and
> did not grow with commanded speed. On elevated overhang reaches the same comparison
> reversed and counter-balancing increased peak orientation drift and pelvis
> translation.
>
> We found no evidence that counter-balancing trades rotational for translational base
> motion: paired rotation and translation gains were positively correlated
> (`rho = +0.45`, `p < 0.001`), so the two improved or degraded together. Throughout,
> the controller difference remained small relative to the task itself. Mean absolute
> differences of `0.14` to `0.46 deg` correspond to `2.4` to `8.0 percent` of the
> `0.1 rad` standing-stability threshold and to under half of the variation induced by
> changing trajectory, no run of either controller crossed that threshold, and all 480
> runs remained stable with no falls. Taken with the separate challenge-endpoint
> result, where counter-balancing reduced falls from 12 to 7 of 40 with six paired
> improvements and no regressions, the evidence supports a narrow claim:
> counter-balancing reduces severe standing failures on challenging motions while
> leaving base motion during representative manipulation trajectories essentially
> preserved, with a reproducible but motion-family-specific reduction in sustained
> base rotation on directional reaches. These are descriptive paired comparisons of
> recorded base kinematics; no causal mechanism and no end-effector path-geometry
> benefit is claimed.

### Recommendation

**B, a metric-specific positive result, scoped to the directional bank, combined with
C as descriptive preservation for the rest of the grid.**

- **A is not available.** There is no general positive base-motion result. The grand
    aggregate is undecided or slightly negative on every metric, and the overhang bank
    is decided against 3C on three of them.
- **B is available and is the strongest defensible claim.** RMS orientation drift on
    directional reaches is decided, replicated independently in both policies, broad
    across trajectories, stable to leave-one-out, consistent across all six speeds,
    and corroborated by the independently gated `max_base_drift` quantity. It must be
    stated with its scope: directional reaches, not the whole bank set.
- **C is a valid supporting frame but only as descriptive preservation.** No
    pre-specified margin exists, so report the magnitudes and the untouched safety
    gate rather than claiming statistical equivalence.
- **D is what the scope decision selects, and that is a presentation choice rather
    than a change in what the evidence supports.** On the merits the simulation
    trajectory-group result did not need demoting: stated as B plus C it bounds where
    counter-balancing helps and tells a reader what it costs when it is not needed.
    Under the 2026-09-14 scope decision it becomes secondary evidence, with the
    directional claim carried by the real runs and the simulation headline carried by
    the stability target sweep. Two consequences should be tracked rather than
    forgotten. First, the overhang result is the negative control that keeps the
    challenge-target claim from reading as a selected success; if the simulation
    trajectory groups are not presented, that control is not presented either, and no
    real overhang batch has been collected to replace it. Second, B and C remain the
    accurate description of this grid, so any later decision to promote the simulation
    trajectory groups back into the paper can reuse this section unchanged.

### Artifacts

Regenerate with:

```bash
uv run python -m h12_zmp_benchmark.experiment.iteration6b_base_motion_paper
```

- [Paired statistics](../../../docs/data/iteration6b_base_motion/paired_statistics.csv), every metric in every stratum with cluster-bootstrap intervals.
- [Rotation/translation tradeoff](../../../docs/data/iteration6b_base_motion/rotation_translation_tradeoff.csv).
- [Preservation scale](../../../docs/data/iteration6b_base_motion/preservation_scale.csv).
- [Paper summary](../../../docs/data/iteration6b_base_motion/paper_summary.json), the exact values quoted above.
- Main figure: `figures/iteration6b_base_motion/paired_rms_orientation_drift.png` and its `_no_title` variant.
- Appendix figure: `figures/iteration6b_base_motion/rotation_translation_tradeoff.png` and its `_no_title` variant.

## 9. Existing Challenge-Group Context

The separate nominal FAME `new_challenge40` endpoint sweep remains the strongest
categorical evidence. On the same 40 endpoints, Frame has 12 falls and frozen 3C
has 7. The paired transition view has **six improvements and zero regressions**:
four `fall -> drift`, one `fall -> stable`, and one `drift -> stable`. ALMI has one
`drift -> stable` improvement and zero regressions, with both controllers keeping
all endpoints up. These challenge results are contextual comparison, not a speed
envelope and not recomputed by the spatial reducer.

Challenge evidence remains under:

- `runs/challenge_sweep/20260909_205330_iter6b_new_challenge40_frame_vs_3c_fame/`.
- `runs/challenge_sweep/20260909_214657_iter6b_new_challenge40_frame_vs_3c_almi/`.

## 10. Limitations

- The saved-line comparison is a fixed-grid descriptive experiment, not a general
  proof of counter-balance benefit or a usable-envelope study.
- A recorded polyline cannot reveal unobserved between-sample excursions. Uniform
  resampling adds no physical evidence.
- Completion remains low at fast factors in several cells; a small spatial RMS does
  not certify a successful manipulation task.
- Nine infrastructure-marked attempts lose their metric-valid runtime flags under
  the rebuilt grid: 13 total infrastructure attempts remain (12 selected), all
  flagged by controller sample gaps in the frozen 3C path. Offline recovery is
  not runtime health.
- The eight former FAME `left_overhang_forward` initialization failures are
  resolved; no current-bank record lacks a stable-standing capture, so no spatial
  metric is manufactured or missing in this grid.
- The base-motion evaluation in Section 7 is descriptive and window-sensitive: the
  sign of the mean peak-tilt result changes between the move-and-hold window and the
  move-only window. It supports no causal claim and no end-effector geometry claim.
- The decided directional result in Section 8 is conditional on the trajectory family
  and does not transfer to the overhang bank, where the same comparison reverses. No
  equivalence or non-inferiority margin was pre-specified for base motion, so the
  preservation statement there is descriptive rather than a statistical equivalence
  claim.
- Peak and RMS absolute pelvis tilt are close throughout this grid, so these trials
  hold a nearly steady tilted posture. Absolute tilt therefore carries a static
  postural component that is not motion induced.
- Retries and repeated factors are not independent observations. No pooled-speed
  significance claim is made.

## 11. Final 6B Conclusions

Iteration 6B is closed. The final claim is intentionally narrow:

- Segment and Uniform Spatial RMS do **not** show a general geometric tracking
  advantage for frozen 3C over Frame on the rebuilt saved-line banks.
- Ideal / Timed RMS retains some fast-factor advantages, but these are
  timing-sensitive and must not be presented as generally straighter paths.
- The saved-line benchmark is supporting/descriptive evidence, not the primary
  proof of counter-balance benefit.
- Base motion during these prescribed trajectories does **not** show a general
  reduction under frozen 3C. Mean peak base tilt is lower with 3C in 9 of 24 cells
  and mean pelvis translation in 4 of 24, with grand means moving the wrong way by
  `0.060 deg` and `0.199 cm`. The one consistent positive block is ALMI-Manip-v2 on
  the directional bank, 6/6 cells and up to `+0.253 deg` at the fastest factor.
- The defensible paper claim from the trajectory groups is metric- and
  family-specific, not general: on the directional bank 3C lowers RMS pelvis
  orientation drift by `0.1334 deg`, `8.4 percent`, 95 percent CI
  `[+0.0726, +0.1974]`, independently in both policies, broad across trajectories and
  flat in speed. Elsewhere the comparison is undecided or favours Frame, and the
  correct supporting frame is descriptive preservation. Section 8 states this in
  paper-facing form.
- The strongest existing result remains the separate FAME challenge outcome:
  six paired improvements, zero regressions, falls `12 -> 7`.

## 12. Iteration 6C Handoff

**Iteration 6C:** challenge-group manipulation-speed envelope, Frame versus frozen
robust 3C, on ALMI-Manip-v2 and FAME.

That experiment is now complete; see the
[Iteration 6C analysis](counter_balance_analysis_iteration_6c.md). The independent
overhang-pair repair is tracked in [6D preparation](counter_balance_iteration_6d_preparation.md).
Runtime-continuity repair and controller tuning are not part of this targeted
geometry correction. H2 remains outside the active comparison.

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
