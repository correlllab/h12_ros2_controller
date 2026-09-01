# Counter-Balance Iteration 3C Analysis

## Status

Iteration 3C is implemented as a standalone one-step Crocoddyl velocity-control
controller:

```text
counter_ddp_velocity_wide
```

It inherits Iteration 2 ownership, objective targets, activation, bounds,
excursion, publication, collision, and safety behavior. Only the bounded
least-squares solver is replaced by `SolverBoxFDDP`.

Iteration 3 B0 and Iteration 2 remain separate runnable variants.

## Command Parity

The one-knot Crocoddyl quadratic matches the Iteration 2 bounded solution over
randomized valid problems:

- 95th-percentile maximum joint-velocity difference below `5e-4 rad/s`.
- Worst randomized difference below `3e-3 rad/s`.
- Matching bound activity and command direction.
- Native Crocoddyl BoxQP polishing with accepted-command KKT violation below
  `5e-4`.
- Zero terminal cost and no new objective term.

The small residual difference is Box-FDDP stopping accuracy rather than a
different controller target.

## Five-Repeat Behavioral Parity

The three historical FAME boundary targets and the same ALMI guards were run five
times for Iteration 2 and Iteration 3C.

Every case had identical controller outcome distributions:

| Policy | Target | Iteration 2 | Iteration 3C |
|---|---|---|---|
| FAME | `right_fast_fall_search_06_scale_74` | 5 fall | 5 fall |
| FAME | `right_fast_fall_search_09_scale_78` | 5 drift | 5 drift |
| FAME | `right_fast_fall_search_11_scale_78` | 5 drift | 5 drift |
| ALMI | `right_fast_fall_search_06_scale_74` | 5 stable | 5 stable |
| ALMI | `right_fast_fall_search_09_scale_78` | 5 drift | 5 drift |
| ALMI | `right_fast_fall_search_11_scale_78` | 5 drift | 5 drift |

Behavioral parity passes. The current environment no longer reproduces the
historical rescue for boundary `06` under either controller; this is an
environment/outcome change shared by the reference and parity controller.

Final BoxQP parity artifacts:

- `runs/key_findings/iteration3c/parity/20260901_035544_iter3c_boxqp_parity_fame`
  and the adjacent FAME repetitions.
- `runs/key_findings/iteration3c/parity/20260901_040228_iter3c_boxqp_parity_almi`
  and the adjacent ALMI repetitions.

## Three-Hard-Group Comparison

Three matched repetitions were run for every controller/target cell. Case
outcomes use median severity. One retry was permitted for incomplete
infrastructure cells; attempted and infrastructure counts remain reported.

Artifact:

`runs/key_findings/iteration3c/reports/iter3c_boxqp_hard_group_majority_summary.json`.

All final case cells contain three complete physical outcomes. Superseded
pre-polish/incomplete attempts and their bounded-hold relabeling remain visible
in the source run manifests. The report contains attempted, complete,
infrastructure, runtime-failure, source-run, and selected-repetition records for
every controller/target cell.

Implementation binding:

`runs/key_findings/iteration3c/reports/iter3c_implementation_manifest.json`.

The combined implementation hash is recorded in the final parity, hard-group,
retry, and full-panel manifests.

### FAME Pooled

| Controller | Stable | Drift | Stumble | Fall | Survived |
|---|---:|---:|---:|---:|---:|
| Frame task | 18 | 18 | 0 | 8 | 36 |
| Iteration 2 momentum wide | 24 | 14 | 0 | 6 | 38 |
| Iteration 3 B0 | 22 | 16 | 0 | 6 | 38 |
| Iteration 3C | 23 | 15 | 0 | 6 | 38 |

Iteration 3C versus frame task:

- Five drift-to-stable improvements.
- Two fall-to-drift improvements.
- Six falls unchanged.

Iteration 3C matches Iteration 2 FAME survival and fall count. It has one fewer
stable and one additional drift case.

### FAME Per Group

| Group | Controller | Stable | Drift | Fall | Survived |
|---|---|---:|---:|---:|---:|
| Exploration | Frame | 13 | 5 | 1 | 19 |
| Exploration | Iteration 2 | 17 | 2 | 1 | 19 |
| Exploration | B0 | 14 | 5 | 1 | 19 |
| Exploration | Iteration 3C | 17 | 2 | 1 | 19 |
| Boundary | Frame | 1 | 4 | 3 | 5 |
| Boundary | Iteration 2 | 2 | 5 | 1 | 7 |
| Boundary | B0 | 3 | 4 | 1 | 7 |
| Boundary | Iteration 3C | 2 | 5 | 1 | 7 |
| Hard | Frame | 3 | 9 | 4 | 12 |
| Hard | Iteration 2 | 5 | 7 | 4 | 12 |
| Hard | B0 | 5 | 7 | 4 | 12 |
| Hard | Iteration 3C | 4 | 8 | 4 | 12 |

Exploration and boundary totals match Iteration 2 exactly. In the hard group,
Iteration 3C has one fewer stable and one more drift, with identical survival.

### ALMI Pooled

| Controller | Stable | Drift | Stumble | Fall | Survived |
|---|---:|---:|---:|---:|---:|
| Frame task | 25 | 11 | 5 | 3 | 41 |
| Iteration 2 momentum wide | 24 | 12 | 5 | 3 | 41 |
| Iteration 3 B0 | 25 | 11 | 5 | 3 | 41 |
| Iteration 3C | 24 | 12 | 5 | 3 | 41 |

Iteration 3C and Iteration 2 have identical ALMI pooled and per-group totals.
Relative to frame task, each converts one stable case to drift and changes no
stumble, fall, or survival outcome.

### ALMI Per Group

| Group | Controller | Stable | Drift | Stumble | Fall | Survived |
|---|---|---:|---:|---:|---:|---:|
| Exploration | Iteration 3C | 16 | 2 | 1 | 1 | 19 |
| Boundary | Iteration 3C | 2 | 4 | 2 | 0 | 8 |
| Hard | Iteration 3C | 6 | 6 | 2 | 2 | 14 |

These three rows match Iteration 2 exactly. Relative to B0, Iteration 3C has one
fewer stable and one additional drift case, with identical stumble, fall, and
survival totals.

## 100-Target Regression Checks

Only Iteration 3C was rerun. Historical frozen panels are context rather than a
new matched comparison.

### FAME

Artifact:

`runs/full_sweep/20260901_053845_iter3c_boxqp_fame_full`.

| Stable | Drift | Stumble | Fall | Survived |
|---:|---:|---:|---:|---:|
| 95 | 4 | 0 | 1 | 99 |

Historical Iteration 2 context was `94 stable, 5 drift, 1 fall`. Iteration 3C has
the same survival count with one additional stable case.

### ALMI

Artifact:

`runs/full_sweep/20260901_061348_iter3c_boxqp_almi_full`.

| Stable | Drift | Stumble | Fall | Survived |
|---:|---:|---:|---:|---:|
| 95 | 5 | 0 | 0 | 100 |

Historical Iteration 2 context was `96 stable, 4 drift`. Iteration 3C has one
additional drift with unchanged survival.

## Timing

Across the hard-group Iteration 3C runs:

- Solver p50: approximately `0.42 ms`.
- Solver p95: approximately `0.76 ms`.
- Solver p99: approximately `3.02 ms`.
- Total-controller p50/p95/p99: approximately `2.77/5.25/9.27 ms`.
- Observed solver/total maxima were approximately `38.2/52.5 ms`; these rare
  process/infrastructure stalls are reported separately from p99 behavior.

Runtime artifact:

`runs/key_findings/iteration3c/reports/iter3c_boxqp_runtime_summary.json`.

Accepted-command maximum KKT violation was below `5e-4`. Moving-arm velocity
pass-through error remained below `1.2e-7 rad/s`; maximum recorded position error
was approximately `0.001 rad` from common publisher clipping. No normal command
was accepted from an invalid numerical solve; empty safe velocity intervals are
reported separately as `counter_bounds_infeasible` holds.

The one-step Crocoddyl solver fits comfortably inside the normal control-period
budget.

## Decision

Iteration 3C is retained as a runnable Crocoddyl high-authority baseline.

It achieves:

- Numerical command parity within practical tolerance.
- Exact five-repeat behavioral parity on the fixed rescue/guard panel.
- Exact Iteration 2 ALMI hard-group totals.
- Exact Iteration 2 FAME exploration/boundary and pooled survival totals.
- One additional stable FAME full-panel outcome relative to historical
  Iteration 2 context.

Iteration 3C is not tuned to repair that difference or the known ALMI
stable-to-drift behavior. Any policy-blind selection between pass-through, B0,
and Iteration 3C belongs in a separate controller/iteration.
