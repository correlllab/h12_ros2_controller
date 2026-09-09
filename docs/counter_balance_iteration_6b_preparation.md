# Iteration 6B Nominal-Refactor Preparation

## Status and Scope

Final validation snapshot, 2026-09-09. The freeze manifest records
**`source_frozen=true`, scoped to numerical/implementation validation with a
physical limitation**. Main, health, timing, numerical, exact-finalizer, unchanged
objective/model/finalizer, test/diff, and inventory gates are recorded as true.
This is not unconditional physical nonregression or speed readiness.

**ALMI hold fractions remain about 91.4% for both new controllers versus 61.7%
for old wide.** Exact source comparison and identical-input replay demonstrate
an inherited collision/hold trap, but its online entry probability remains
unresolved. Stable/precise tracking and zero nominal failures do not establish
continuous counter-arm authority. No objective, collision, or hold tuning was
made to obtain this freeze.

This is a nominal-backend comparison, not a new control objective. Preserve
weights, normalization, target gains, bounds, lifecycle activation, reference
capture, collision backtracking, gravity compensation, and the single publication
path. Do not introduce straight-line motion, catalog changes, speed changes, or
model changes. Finalized 6A evidence remains historical and must not be rewritten.

## Runtime Identity

| Runtime | Nominal backend | Residual |
| --- | --- | --- |
| `counter_ddp_velocity_wide` | Existing frozen-3C Crocoddyl Box-FDDP plus BoxQP. | None. |
| `counter_ddp_velocity_robust` | New shared SciPy BVLS primary, at most one TRF retry. | None; no H2 models or OCP. |
| `counter_residual_h2_robust` | Same shared SciPy backend, not historical Croc-first fallback. | Existing H2 models and Crocoddyl residual OCP. |

The legacy 3C runtime retains its existing behavior: the default observation
hooks read live state and its OCP factory still constructs `CounterVelocityOCP`.
Both current robust classes use `ScipyNominalMixin`; its `_create_velocity_ocp()`
returns `None`, so neither constructs a nominal OCP. The mixin captures one
per-tick observation snapshot, performs a nonpublishing nominal solve, commits
diagnostics once, and retains the inherited finalizer. Only robust H2 loads the
H2 models and constructs the residual Crocoddyl OCP. This is an object/runtime
distinction, not a claim that inherited modules never import Crocoddyl.

Historical robust H2 used Crocoddyl first and SciPy TRF only after a returned
nominal rejection. Its source is archived at controller revision `9256ab6`
(`9256ab618c3c59ea6967ef1ab00546bdea811ab6`), not selected by the current robust
runtime name. Benchmark-root provenance lives in
`runs/key_findings_reports/iteration6b_preparation/nominal_refactor/provenance/`:

- `controller_pre_refactor_9256ab6.tar.gz`: archived controller source.
- `benchmark_configs_pre_refactor.tar.gz`: archived benchmark configurations.
- `pre_refactor_identity.json`: source identity, hashes, versions, and config paths.
- `freeze_manifest.json`: final gate scope and limitations.
- `current_source.tar.gz`: frozen current source/config/test archive, SHA-256
    `9712a1dc1f6a3737e242301ae56c4b982099dfa00584212cb8cf6f281a460c8a`.

Old source/config archives are retained for historical reconstruction and runtime
compatibility. `current_source_identity.json`, `git_provenance.json`, and
`evidence_identity.json` bind the final capture and retained evidence; the final
source/checkpoint capture is not retroactive source attestation for every run.

The [H2 implementation reference](frozen_h2_and_robust_h2_implementation.md)
retains historical Croc-first details and the unchanged residual mathematics.

## Exact Shared Objective

Source of truth is `objective.py:reaction_targets()` and
`objective.py:bounded_velocity_problem()`, called through
`frozen_3c_planner.py:plan_frozen_3c_velocity()`. Let \(b\) be the effective
lifecycle balance scale, \(J\) the planar CoM Jacobian, \(A\) the planar momentum
map, \(v\in\mathbb R^4\) the counter velocity, and \(o_c,o_h\) the explicit
planner RHS offsets:

\[
r_c=b(-J_m\dot q_m-k_c e_c)+o_c,\qquad
r_h=b(-A_m\dot q_m+k_g\omega)+o_h,\qquad
r_q=-k_q(q_c-q_{c,ref}).
\]

With physical normalization scales \(s_c,s_h,s_q\), the problem is exactly

\[
\min_{l\le v\le u} f(v)=\tfrac12\|Mv-y\|^2,
\qquad
M=\begin{bmatrix}
\sqrt{bw_c}J_c/s_c\\
\sqrt{bw_h}A_c/s_h\\
\sqrt{w_q}I/s_q\\
\sqrt{\lambda}I/s_q
\end{bmatrix},\quad
y=\begin{bmatrix}
\sqrt{bw_c}r_c/s_c\\
\sqrt{bw_h}r_h/s_h\\
\sqrt{w_q}r_q/s_q\\
0_4
\end{bmatrix}.
\]

Only positive-weight blocks are stacked. **Balance scale appears twice**: in
the reaction targets and in the CoM/momentum block weights. Do not simplify away
either occurrence. Damping is normalized by the **posture velocity scale**,
not an unscaled identity. The planner offsets are added after reaction scaling.

Retained nominal settings are gains `(com, gyro, posture) = (2.0, 0.2, 1.0)`,
weights `(com, momentum, posture) = (1.0, 2.0, 0.02)`, damping `0.0001`, and
scales `(0.1, 1.0, 1.0)`. Maximum velocity remains
`[2.6, 3.2, 2.6, 1.5] rad/s`; excursion remains
`[0.75, 0.58, 0.40, 0.58] rad`. Bounds still intersect robot position/velocity,
publisher clipping, controller limits, and frozen velocity/excursion limits.
Empty intervals retain the shared hold behavior. No lifecycle or bound retuning
is part of this preparation.

## Solver and API Contract

Current source paths below are relative to
`h12_ros2_controller/core/controller/counter_balance/`:

| Source/API | Responsibility |
| --- | --- |
| `scipy_nominal_planner.py:ScipyNominalConfig` | Explicit weights and three normalization scales. |
| `solve_scipy_nominal(..., config)` | Build the shared objective; return `Frozen3CVelocitySolve` with matrix/target and diagnostics. |
| `solve_bounded_least_squares(matrix, target, lower, upper)` | Solve the four-column bounded problem without publishing. |
| `counter_ddp_velocity_robust_controller.py:ScipyNominalMixin` | Shared solve, snapshot, diagnostic commit, and no nominal OCP. |
| `counter_residual_h2_robust_controller.py:CounterResidualH2RobustController` | H2 composition with strict residual validation and safe reset handling. |

The primary call is `scipy.optimize.lsq_linear` with `method='bvls'`,
`lsq_solver='exact'`, `tol=1e-10`, and `max_iter=100`. A rejected candidate or
backend exception permits exactly one `method='trf'` attempt with the **same**
exact solver, tolerance, and iteration budget. No Crocoddyl nominal fallback,
retry loop, or tolerance relaxation is used. These iteration budgets are not
wall-clock guarantees. The recorded replay environment uses SciPy `1.15.3`;
the [official API reference](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.lsq_linear.html)
confirms the method and dense exact-solver parameter semantics. The explicit
source constants, not changing library defaults, define this contract.

Exactly fixed coordinates \(F=\{i:l_i=u_i\}\) are eliminated with
\(v_F=l_F\), \(y'=y-M_Fl_F\), and free-column matrix \(M_{\bar F}\).
Reconstruct the full vector before validation. All-fixed input is validated
without calling SciPy. Narrow nonzero intervals remain free; no fixed bounds
are expanded and no returned candidate is clipped into acceptance.

Acceptance is independent of SciPy `success`, `status`, or `optimality`:

- Require finite input and a finite candidate of shape `(4,)` inside the
    original bounds, with no feasibility slack.
- Recompute the original objective and gradient \(g=M^T(Mv-y)\). Compare
    against \(v_0=\operatorname{clip}(0,l,u)\), requiring
    \(f(v)\le f(v_0)+10^{-12}+10^{-10}f(v_0)\).
- Require finite projected KKT violation at most `5e-4`: use \(|g_i|\) in
    the interior, \(\max(-g_i,0)\) at a lower bound,
    \(\max(g_i,0)\) at an upper bound, and zero for exactly fixed coordinates.
    Bound activity tolerance is `min(1e-10, (upper-lower)/4)` per coordinate.
- If neither attempt validates, return a rejected plan for the inherited hold
    path; the feasible baseline is not promoted to an accepted solution.

Diagnostics include `nominal_backend`, `nominal_retry_used`, `nominal_status`,
and independent attempt records. Existing `nominal_fallback_used` now means a
TRF retry was attempted, not historical Croc-to-SciPy fallback success;
`nominal_primary_accepted` refers to the first SciPy attempt (or all-fixed
validation). Invalid/rejected H2 output selects zero residual and nominal-only
execution through the inherited selector. H2 authority, models, and costs are
not retuned.

## Completed Panel

All paths in this section are under the benchmark-root
`runs/key_findings_reports/iteration6b_preparation/nominal_refactor/`.

The inventory is **150 physical trials = 132 main + 8 health + 10 outer-timing**.
`audit_output/audit.json` validates all 132 main trials without exclusions or
validation errors: 96 FAME, 36 ALMI, 33 matched quadruplets, and 44
controller/policy-target cases with three repetitions. Health and timing trials
are separate, not extra main-panel repetitions. Existing saved joint targets and
the fast profile (`1.5 s` move, `10 s` hold) were retained:

| Policy | Existing target IDs | Cells |
| --- | --- | ---: |
| FAME | `right_fast_fall_search_06_scale_74`, `right_fast_fall_search_09_scale_78`, `right_fast_fall_search_11_scale_78` | 3 |
| FAME | `right_manual_grasp_pitch_plus`, `right_manual_grasp_pitch_minus` | 2 |
| FAME | `left_fast_fall_search_04_scale_76` | 1 |
| FAME | `right_overhang_inner_forward_01`, `right_overhang_inner_forward_02` | 2 |
| Corrected ALMI-Manip v2 | `right_overhang_upward_05`, `right_upward_overhang_pitch_minus`, `left_inner_upward_overhang_rank6` | 3 |

Three repetitions of `frame_task`, `counter_ddp_velocity_wide`,
`counter_ddp_velocity_robust`, and current `counter_residual_h2_robust` completed
**11 x 3 x 4 = 132 trials**, with rotating controller order. These are 11
policy-target cells, not 11 targets crossed with both policies.

| Main controller | FAME stable/drift/stumble/fall | ALMI stable/drift/stumble/fall | Precise | Controller-complete | Nominal failure ticks |
| --- | --- | --- | ---: | ---: | ---: |
| Frame | 13/2/0/9 | 8/1/0/0 | 24/33 | 33/33 | Not applicable. |
| Old wide | 8/16/0/0 | 9/0/0/0 | 33/33 | 29/33 | 5 |
| New 3C | 9/15/0/0 | 9/0/0/0 | 33/33 | 33/33 | 0 |
| New H2 | 10/14/0/0 | 9/0/0/0 | 33/33 | 33/33 | 0 |

All three counter variants preserve the FAME `06/09/11` Frame-fall-to-drift
rescue **3/3 on each target**, with precise tracking. Ordinary guards remain
precise. This does not establish a new H2-specific rescue over nominal 3C.
All four old-wide controller-incomplete runs are ALMI: upward 05 in each repeat
and upward-overhang pitch minus in repeat 3, totaling five nominal failure ticks.
Both new paths have nine stable/precise ALMI trials, but the hold-rate limitation
in the status section remains material.

Health remains legitimate evidence: four controllers on FAME `right_arm_forward`
and four on ALMI `right_overhang_upward_05`, all stable/precise. Old wide has one
ALMI solver-failure tick; both new paths have zero. Health is not a substitute
for the main panel. `audit_output/controllers.csv`, `cases.csv`, and `runs.csv`
retain aggregate, target/repeat, and run-level distinctions between physical
outcome, precision, execution, controller completeness, holds, and H2 requests.

## Numerical and Finalizer Evidence

`nominal_replay_complete.json` and `.npz` cover **39,359 recorded prepared
problems**, with zero builder mismatches and zero new-invalid results. All
17 old-3C rejections are repaired. Of 39,342 accepted old-3C results, command
difference p95 is `1.2868988330527738e-8 rad/s` and maximum is
`1.144624463511601e-4 rad/s`, within the documented parity thresholds of
`5e-4` p95 and `3e-3` maximum. **Sixteen old-accepted results fail the common KKT
validation**; they are not hidden or treated as valid simply because old 3C
accepted them. The archived TRF comparison has 369 old-accepted common-invalid
results. Recorded new requests and re-solved requests match exactly.

The complete set contains 37,081 ordinary and 2,278 saturated problems; 34,411
have balance scale below one. None has exactly fixed coordinates. The earlier
health replay's fixed-coordinate copies remain explicitly **synthetic derived
coverage**, not additional recorded problems or physical trials. Its smaller
`4.15e-8` maximum is not the full-set maximum.

`exact_command_replay_fame_r1.json` and `exact_command_replay_almi_r1.json`
contain **12,964 captured pre-finalization contexts** from first repetitions
only (10,804 FAME + 2,160 ALMI). New 3C versus new H2 with residual forced to zero
has exactly zero full-command delta. Accepted-old finalizer decisions match,
but accepted-old/new FAME command differences reach `3.967285156258882e-5`:
**not all are below `1e-6`**. ALMI accepted-old full-command maximum is
`3.794493430575585e-8`; seven old rejections instead become six solved commands
and one backtracked command. The resulting full-command maximum change `2.6`
is intentional rejection repair, not rejected-old-command parity.

Source AST comparison in `provenance/unchanged_ast.json` and replay source proofs
show unchanged objective, response models, finalizer, collision checks,
backtracking, hold, and arm-command publication semantics. This proves the
collision/hold trap is inherited on identical inputs, not that independently
evolving online trajectories enter it with equal probability. Replay uses real
Pinocchio and mock DDS with one arm-command write; external safety/publisher
clipping is not simulated. It is not an online physical counterfactual.
`full_command_replay.json` is the earlier reconstructed health replay, not the
12,964-context exact capture evidence.

## Timing Gate

The gate is **per-run complete-controller p99 < 15 ms**, not a pooled percentile
or a hard `20 ms` deadline guarantee. Main and health used the old **inner
complete-kernel** boundary, including validation, retries, and finalization but
excluding outer resets. Current instrumentation covers the outermost
`control_configuration_step`; these measurements are not end-to-end DDS latency.
Do not silently relabel earlier timing as outermost timing.

Main timing from `audit_output/controllers.csv` is retained explicitly:

| Policy/controller | Worst run p99 (ms) | Maximum (ms) | Published samples >15/>20 ms |
| --- | ---: | ---: | ---: |
| FAME old wide | 7.440 | 26.232 | 12/6 |
| FAME new 3C | 5.989 | 15.115 | 1/0 |
| FAME new H2 | 9.111 | **32.044** | 2/1 |
| ALMI old wide | 6.694 | 20.650 | 2/1 |
| ALMI new 3C | 8.764 | 19.316 | 3/0 |
| ALMI new H2 | 9.955 | **28.912** | 4/1 |

The two new-H2 `>20 ms` samples are real retained late publications, not discarded
outliers. Here "published late" means an actually published command with measured
controller duration above threshold, not measured transport latency.

The separate ten-run outermost audit (`provenance/timing_audit.json`) passes with
zero nominal failures and no missing solver/timing samples. Each run has 684
runtime samples. Abbreviations below use the target IDs in the panel table:

| Policy/target | Controller | p99 (ms) | Maximum (ms) | >15/>20 ms |
| --- | --- | ---: | ---: | ---: |
| FAME 06 | New 3C | 4.419 | 7.098 | 0/0 |
| FAME 06 | New H2 | 8.133 | 9.130 | 0/0 |
| FAME 09 | New 3C | 3.888 | 4.286 | 0/0 |
| FAME 09 | New H2 | 7.670 | 8.640 | 0/0 |
| FAME 11 | New 3C | 3.884 | 4.734 | 0/0 |
| FAME 11 | New H2 | 8.042 | 9.594 | 0/0 |
| ALMI upward-overhang pitch minus | New 3C | 5.394 | 6.174 | 0/0 |
| ALMI upward-overhang pitch minus | New H2 | 8.305 | 9.500 | 0/0 |
| ALMI upward 05 | New 3C | 5.546 | 6.558 | 0/0 |
| ALMI upward 05 | New H2 | 7.755 | 8.406 | 0/0 |

Worst outermost per-run p99/max are `5.546/7.098 ms` for new 3C and
`8.305/9.594 ms` for new H2. There are no `>15 ms` or `>20 ms` controller-duration
or published-late samples in these ten runs. That observation does not erase
the main late maxima, certify all future deadlines, or authorize ALMI speed tests.

## Verification and Handoff

`provenance/validation_tests.json` records **217 root tests + 38 evidence tests
(and six subtests) + 90 focused controller tests passed**, with root/submodule
`git diff --check` successful. The separately reported prior 176 targeted passes
are earlier evidence, not the final freeze test count or an additional disjoint
suite. The final artifact includes JAXopt-maintenance and meshcat/pyzmq
deprecation warnings. It does not establish a universally clean submodule
suite, lint suite, or smoke environment; baseline lint/smoke limitations remain
outside the focused freeze gates and must not be represented as universal passes.
`provenance/dependencies.json` records Python `3.10.12`, NumPy `2.2.6`, SciPy
`1.15.3`, Crocoddyl `3.2.1`, and Pinocchio `4.1.0` with the full package inventory.

The [final artifact index](../../../runs/key_findings_reports/iteration6b_preparation/nominal_refactor/README.md)
lists all ten sweep directories and the exact offline finalization command from
`provenance/rerun_commands.json`. `provenance/cleanup.json` records no temporary
debug physics runs created, only the nominal-refactor validation cache removed,
and all 150 physical trials plus legitimate health/replay evidence retained.

The numerical/implementation baseline is frozen with the physical limitation
above. Trajectory-bank preparation is a separate next task and **has not
started**. ALMI speed work remains **not authorized**. No straight-line,
catalog, speed, model, objective, collision, or hold tuning is implied by this
handoff. This documentation-only finalization runs no physics or tests and
changes no production code, configurations, provenance, or telemetry.
