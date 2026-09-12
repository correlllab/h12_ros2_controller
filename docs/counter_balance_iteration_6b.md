# Counter-Balance Iteration 6B

## Dynamic Manipulation Evaluation

Status: the straight-line speed study is executed. All ten `directional` and ten
`overhang` entries were swept at `0.8x` on ALMI and FAME, Frame versus frozen
3C-robust, with a `0.5x` extension. This is evaluation of frozen controllers, not
controller development. The question is: **does counter-balancing expand the usable
dynamic manipulation envelope?**

The first-shortening rejection was traced to a defect in the override timing gate,
not to physical infeasibility: the old bound multiplied the geometry spline's
endpoint derivative spikes by the mid-move quintic-clock peak, two maxima that
never co-occur, over-stating acceleration by up to ~3800x. It was replaced with the
exact continuous timed-derivative gate (see the
[Speed Validation Contract](../../../docs/straight_line_trajectory_benchmark.md#speed-validation-contract)),
which changes no geometry, controller, or metadata. The `25 rad/s^2` acceleration
figure was then shown to be a conservative bank-generation constant, not a runtime
limit -- the runtime gates velocity and torque, not joint acceleration -- so speed
exploration raises it as an explicit runtime override (`45 rad/s^2` at `0.8x`,
`100 rad/s^2` at `0.5x`), keeping `6 rad/s` velocity binding and the bank immutable,
so **all 20 entries admit `0.8x`**. See
[the analysis](counter_balance_analysis_iteration_6b.md) for results and mechanism:
at `0.8x` 3C is net-neutral versus Frame, direction-dependent, with one repeatable
cross-policy improvement (`right_overhang_sideways`).

## Source Of Truth

Read together with the finalized design and analysis documents for
[5](counter_balance_iteration_5.md), [5B](counter_balance_iteration_5b.md),
[5C](counter_balance_iteration_5c.md), [5D](counter_balance_iteration_5d.md),
[5E](counter_balance_iteration_5e.md), and [6A](counter_balance_iteration_6a.md).
The corresponding `counter_balance_analysis_iteration_*.md` documents establish
accepted evidence and supersession. Additional reviewed contracts are:

- [Current robust nominal preparation](counter_balance_iteration_6b_preparation.md).
- [H2 implementation](frozen_h2_and_robust_h2_implementation.md) and
    [3C methodology](iteration_3c_controller_methodology.md).
- [Corrected ALMI v2/v3 benchmark](counter_balance_almi_manip_v23_benchmark.md),
    [fine-tuning](../../h12_mjlab/docs/almi_manip.md), and
    [ALMI environment](../../h12_mjlab/docs/almi.md).
- [Straight-line benchmark](../../../docs/straight_line_trajectory_benchmark.md),
    current bank manifests, endpoint catalogs, and sweep/runtime implementations.

Historical nominal-only authorization is not speed evidence. The current user
request starts a new evaluation, but does not waive timing validation. The
preparation note's unbuilt-bank snapshot is superseded by the existing schema-v2
banks. Retained nominal comparison manifests also postdate the straight-line
document's smoke-only status; they are not speed-search or repeated 6B evidence.

Do not reopen H3, foot-twist costs, selective gates, confidence relaxation,
residual-authority expansion, or sign changes. Corrected ALMI deployment supersedes
old incompatible-gain/history comparisons. Corrected 6A Frame-versus-counter
rescues do not establish an H2-specific advantage. Catalog aliases are not
independent geometries, and truncated detector-contaminated windows are not
completed survival trials.

## Frozen Contracts

- Primary counter method: `counter_ddp_velocity_robust`; baseline: `frame_task`.
- Secondary endpoint method: `counter_residual_h2_robust`, with its current active
    overlay, not the base shadow configuration or historical Croc-first backend.
- Both robust methods use the shared SciPy BVLS primary and at most one TRF retry,
    preserving independent acceptance, bounds, objective, and KKT checks. Retry
    diagnostics must use their current meaning, not historical fallback semantics.
- Preserve H2's frozen models, two-knot delayed residual formulation, weak-joint
    mask, trust `[0.01, 0.01, 0, 0.01]`, weights `0.01/0.005`, confidence/domain
    abstention, applied-action accounting, and shared finalization path.
- Preserve all gains, controller limits, lifecycle, collision backtracking, holds,
    gravity compensation, safety settings, and corrected classification thresholds.
- Use `mjlab_almi_manip_2` first. Keep its corrected observation/history,
    normalization, gains, checkpoint, and initialization. V3 is optional confirmation,
    not a substitute for the primary panel.
- Retain the unresolved ALMI collision/hold-entry limitation and individual-run
    complete-controller p99 below 15 ms criterion. Report timing tails and applied
    authority; solver success alone is not continuous counter-arm execution.

No controller, fitted model, bank geometry, threshold, or parameter was changed
to obtain the preflight result.

## 6B-A Straight-Line Speed Exploration

Use all 10 `directional_traj` and all 10 `overhang_traj` entries as defined by
`bank.json`. Keep exact saved metadata and payload hashes. Membership and geometry
must not depend on controller outcomes.

For each trajectory, preflight nominal duration and then shorten toward `0.8`.
The override gate is the exact continuous timed-derivative check, not a sampled
substitute and not the superseded analytical bound; bank geometry, nominal
metadata, controllers, and the `6 rad/s` velocity limit are unchanged. The
`25 rad/s^2` acceleration figure is a conservative bank-generation constant, not
a runtime limit — the runtime clips velocity and torque and gates no moving-arm
acceleration, and the target sweep runs its cubic blend without any acceleration
ceiling. Speed exploration therefore raises the acceleration envelope to
`45 rad/s^2` (`SavedLineTrajectory.set_acceleration_limit` /
`--acceleration-envelope` / `arm_target.speed_acceleration_limit`), a runtime
override that never mutates the bank and never lowers velocity. At that envelope
**all 10 directional and all 10 overhang entries admit `0.8x`** (peak exact
acceleration about `39 rad/s^2`, peak velocity at or below about `2.4 rad/s`, well
under `6 rad/s`); the overhang entries admit deeper factors down to `0.5x`.
Per-entry admitted factors are recorded by
`experiment/iteration6b_speed_audit.py --acceleration-envelope`.

The existing fresh geometric validation includes joint-position extrema, dense
and adaptive samples, FK against the saved position/orientation schedule, the
original orientation cone, velocity, acceleration, and sphere collision clearance.
These remain sampled geometric checks, not a continuous collision/FK proof.

At admitted durations, run one trial per trajectory/controller/duration, serially
and headlessly. Start at nominal and progress only through valid shortenings.
Preserve initialized saved `q_start`, release checks, captured stable-standing full
SE(3), and the fixed world reference `p0 + R0 @ p_line(t)` and
`R0 @ R_line(t)`. No measured-onset fit, recapture, live-pelvis retargeting, time
warp, or Cartesian replanning is permitted. Keep the full 10-second endpoint hold.

Retain the existing metrics independently:

- Timed world-position RMS/max and finite line-segment RMS/max deviation.
- Endpoint position/orientation error, completion, progress, and stall status.
- Prescribed orientation RMS/max and endpoint-hold tracking.
- Pelvis drift/tilt, foot displacement/lift, and physical classification.
- Manipulation desired, published, and measured joint tracking.
- Solver acceptance, retries, collision/hold/fallback status, controller coverage,
    full-controller p99/max, deadline exceedances, and actual late publications.

Do not conflate stable balance, trajectory precision, controller completeness,
process completion, and physical survival. Preserve missing metrics as missing,
especially incomplete endpoint windows. A timing-invalid request has no measured
world-trajectory error.

Rank discovery pairs by signed Frame-minus-3C timed RMS error, retaining all
neutral and negative pairs alongside improvements. Inspect max/segment/endpoint
errors, orientation, balance, and health before selecting confirmations. Record
the selected set and reasons before repeats; do not choose favorable attempts
afterward. Use three fresh repetitions for strongest improvements and meaningful
regressions, alternating Frame/3C order between repetitions. Use five total fresh
confirmation repetitions only for highlighted final claims; report discovery
separately. Optional v3 checks use a small explicitly identified important subset.

## 6B-B Endpoint Dynamic Failure Search

After the straight-line stage, compare Frame, 3C-robust, and H2-robust using the
existing Hard, Exploration, and Boundary catalogs. Their 44 named endpoints
represent 40 unique geometries after the four exact lateral-high/overhead aliases
are removed. Do not substitute `new_challenge.yaml` or alter endpoints.

Keep amplitude `alpha = 1.0`; vary duration only through `1.5`, `1.25`, `1.0`,
`0.8` seconds, with `0.6` only if needed and physically executable. Nominal
stability is not a certificate for accelerated execution. Use statically valid
high-risk unique geometries and record the selection before dynamic results.
Boundary 04/06/09/11 on both sides, manual grasp plus/minus, overhead, cross-body,
and inner-upward extremes are candidate families, not asserted corrected-ALMI
failures. Retain nominal/neutral results and all exclusions.

The endpoint runtime uses a cubic joint blend from measured release state, unlike
the saved-line quintic. Its in-move requirements are
`max(abs(dq)) = 1.5 * abs(delta_q) / T` and
`sup(abs(ddq)) = 6 * abs(delta_q) / T**2`, with acceleration discontinuities at
hold transitions. Saved endpoint validity flags do not validate that full path.
The current endpoint infrastructure has no applicable all-moving-joint
acceleration gate. Do not silently borrow the counter-controller acceleration
parameter or bank-only acceleration contract. This executability gap must be
resolved before an accelerated endpoint sweep can be represented as validated.

Use one discovery trial per cell, then three fresh repetitions around interesting
transitions, rotating all three controller orders. Extend to five only for final
claims. Compare physical categories and continuous tracking/balance margins:
Frame to 3C measures nominal benefit; 3C to H2 measures residual benefit. Report
H2 abstention, actual applied activity, and shared collision holds. Lack of a
challenging repeated panel cannot establish practical equivalence.

## Execution And Evidence

Each speed sweep is a standard sweep directory directly under `runs/traj_sweep/`,
named `<timestamp>_iter6b_speed<factor>_<bank>_frame_vs_3c_<policy>`, mirroring the
nominal `frame_vs_3c` comparison sweeps. Always sweep the full ten `directional`
and ten `overhang` entries for general coverage; do not restrict to a favourable
subset. Analysis, overlays, and the feasibility audit live in a grouping folder
`runs/traj_sweep/iter6b_speed<factor>_analysis/`. The superseded ad-hoc
`iteration6b/` tree has been removed.

Use one worker and no overlapping simulators. Between sweeps, scan process health,
storage, attempted cells, captures, complete windows, config and input hashes,
solver/hold status, and timing. Separate infrastructure, initialization, tracking,
solver, timing, and physical failures. Record exact duration, nominal duration,
factor, acceleration envelope, policy, controller, geometry hash, generated config,
capture, logs, and classifications in each cell identity.

For selected improvements, regressions, stumbles, and falls, generate the
left-front/center/right-front world-path overlays (`plot_speed_tracking_overlay`),
overlaying the registered desired world path and the measured wrist path aligned by
recorded release. Never present kinematic bank playback as a controller comparison.
Synchronized rendered video requires a video-enabled re-run and is not produced by
the headless sweeps.

The offline feasibility audit reports each entry's minimum feasible duration and
admitted factors under a given envelope, validating each factor through the full
sampled contract re-timed at the shortened duration:

```bash
uv run python -m h12_zmp_benchmark.experiment.iteration6b_speed_audit --acceleration-envelope 45
```

## Speed Campaign Execution

Each speed sweep runs `arm_reachability_sweep` on a saved bank with
`--duration-factor` and `--acceleration-envelope`. The factor scales every
entry's nominal duration; the envelope raises only the acceleration ceiling of the
exact timed gate (velocity stays the saved limit). A candidate whose scaled
duration still fails the gate is excluded and recorded, never forced -- at the
envelopes used here every entry admits its factor, so all twenty are always swept.
The overrides flow into `move_duration` and the trajectory's runtime acceleration
limit and propagate through saved-line sampling, the line/hold phase boundary, the
`line_end_tick` schedule, the total physics window, controller coverage, the frozen
world reference `p0 + R0 @ p_line(tau)` re-timed at the executed duration, and the
cell slug/resume identity. The exact timed maxima are warmed at setup so the first
line command does not stall, and the summary re-derives the phase with the
runtime's own release-time formula. Nominal execution, reference registration, the
immutable bank, and all prior tests are unchanged; the suite passes.

Run each sweep serially and headlessly, one worker, `frame_task` versus
`counter_ddp_velocity_robust`, over the full ten-entry bank, with the nominal
(`1.0x`) `frame_vs_3c` sweep as the baseline. Health-scan between sweeps. Do not
alter the controller, bank geometry, or the velocity limit.

Executed: `0.8x` (envelope `45 rad/s^2`) across all twenty entries on ALMI and
FAME, plus a `0.5x` (envelope `100 rad/s^2`) extension. Selection of a final
controller is deferred until the accelerated results and the endpoint study are
complete.
