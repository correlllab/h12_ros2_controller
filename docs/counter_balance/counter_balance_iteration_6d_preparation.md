# Iteration 6D Preparation: Robust Overhang Check

Status: **COMPLETE**. Diagnosis verified, pair installed, and 48 replacement cells
merged into the existing overhang speed-sweep roots. Temporary diagnostics and
superseded pair recordings were removed after their outcomes were recorded here.

## Scope

Repair only `left_overhang_forward` and its right mirror in `overhang_traj`.
Keep Frame, frozen robust 3C, FAME and ALMI-Manip-v2 unchanged. Do not rerun a
whole bank or alter standing/classification thresholds. After validation, replace
only these targets' rows in the existing 12 overhang speed-sweep roots, preserving
all other entries. Retain concise old/new geometry and outcome provenance here;
remove temporary diagnostic recordings after their findings are recorded.

## Existing Evidence

The original left line is `(0.100,0.300,0.200) -> (0.304,0.300,0.200)` m in the
pelvis frame; the right line reflects y. Existing FAME-left evidence contains
10 successful initializations and eight failures across 18 attempts. All eight
failed runs physically collapsed before trajectory release: height reached
0.0246-0.0336 m. Their fall flags stayed false because normal fall detection is
armed only after release. The original `initialization_failed` classifications
remain correct experiment-stage labels, but must not be mistaken for upright
readiness timeouts.

Initial physical q/qvel and held q/tau/gains were identical within a side across
speeds and controllers. Gravity compensation and bounded policy warm-up were
present. Live update timing varied, without separating successes from failures.
Successful FAME-left starts already peaked at 0.182-0.190 rad tilt and took
6.3-7.1 s to settle; the right mirror peaked below 0.141 rad and settled in
4.7-5.7 s. ALMI passed all 24 mirrored control cases with much smaller tilts.
This supports a marginal, timing-sensitive FAME left-pose transient, not a
demonstrated missing-command/gain/torque handoff bug or a trajectory-speed effect.

## Minimal Verification Plan

- Run two initial-only probes of the existing left pose: Frame and 3C on FAME.
    The simulator never releases the arm trajectory; after standing qualification,
    require five further seconds continuously satisfying the unchanged gate.
    Detect and record physical falls during this diagnostic phase.
- Test a small symmetric pose adjustment, rather than changing global startup,
    lower-body policy or counter-controller parameters. Reject a candidate if it
    fails; do not collect successful retries to conceal failures.
- Screen the accepted left/right pair on both policies and both controllers with
    initialization-only probes. Then the six requested duration factors supply
    additional independent startup observations during the required replacement
    runs: `1.0/0.8/0.6/0.5/0.4/0.3`.
- Generate and run only 48 replacement cells: two targets, two controllers, two
    policies and six factors. Preserve each root's existing acceleration envelope
    and all other configuration values. Record actual outcomes rather than assume
    geometric validation proves standing or task success.
- Update the current bank pair, current-bank inventory, full sweep tables, standard
    `plots/index.html` reports and documentation. Remove superseded pair traces
    and temporary diagnostic runs only after the replacements and retained
    unaffected evidence have been verified.

The small `arm_target.initialization_hold_duration` option uses the normal saved
start, policy warm-up and gravity hold. It suppresses both simulator and supervisor
trajectory release and writes a diagnostic result before clean shutdown. Normal
sweep behavior is unchanged when the option is absent. Seventy targeted regression
tests passed before diagnostic execution.

## Diagnostic Outcomes

All probes held the saved arm pose and never released the trajectory. The two
additional original-left checks passed, but still reached 0.178/0.188 rad startup
tilt, consistent with the earlier marginal transient. These two successes do not
erase the eight historical collapses.

The only candidate tested moves the forward family inward from y=0.30 to y=0.25 m,
with exact right mirroring. The unchanged geometry/extension solver selected:

| Pair | Start, pelvis m | End, pelvis m | Length | Nominal / hold |
| --- | --- | --- | --- | --- |
| Previous left | (0.100, 0.300, 0.200) | (0.304, 0.300, 0.200) | 0.204 m | 1.5 / 10 s |
| Updated left | (0.138, 0.250, 0.200) | (0.367, 0.250, 0.200) | 0.229 m | 1.5 / 10 s |
| Updated right | (0.138, -0.250, 0.200) | (0.367, -0.250, 0.200) | 0.229 m | 1.5 / 10 s |

Only the y coordinate of the search domain was changed; the selected start x
also changed through the original extension/limit procedure. This is a real
geometry revision, not a renamed old recording or an easier shortened segment.
The right q14 knots are the exact signed left mirror and independently pass FK,
collision and joint-limit validation. Clearance exceeds 0.038 m on both sides;
maximum position error is below 0.010 mm. All six existing duration-factor gates
pass at the original velocity bound and each sweep's recorded acceleration
envelope (25/45/70/100/160/280 rad/s^2 respectively).

| Probe Group | Frame | Robust 3C | Startup Peak Tilt Range | Five-Second Quiet Hold |
| --- | --- | --- | --- | --- |
| Original FAME left | 1/1 | 1/1 | 0.178-0.188 rad | Passed |
| Updated FAME left | 3/3 | 3/3 | 0.163-0.177 rad | Passed |
| Updated FAME right | 1/1 | 1/1 | 0.132-0.133 rad | Passed |
| Updated ALMI left | 1/1 | 1/1 | 0.0542-0.0545 rad | Passed |
| Updated ALMI right | 1/1 | 1/1 | 0.0381-0.0384 rad | Passed |

The candidate passed 12/12 initialization-only checks, with no confirmed falls,
no trajectory release, and no changed readiness thresholds. Initialization arm
error after the hold remained approximately 0.0107-0.0110 rad, within the existing
0.05 rad gate. FAME-left trials used three fixed, interleaved observations per
controller; there was no sequence of failed candidate retries hidden by later
successes. The 48 required full replacement runs additionally test six fresh
initializations per policy/controller/side. This is an empirical robustness check,
not a guarantee for every possible startup state.

## Replacement Outcomes

All 48 cells (two targets, two controllers, two policies, six factors) completed
in the existing 12 overhang sweep roots. No kinematically infeasible duration
was admitted; each cell used its root's recorded acceleration envelope.

| Outcome | Count |
| --- | ---: |
| Stable and released | 44 |
| Infrastructure (3C controller sample gap) | 4 |
| Initialization failure | 0 |
| Physical fall | 0 |

Every replacement cell produced a valid stable-standing capture and released
motion. The eight former FAME-left initialization collapses did not recur after
the inward mirrored pair was adopted. Four cells retain an infrastructure flag
from a `0.055 s` controller sample gap, each on frozen robust 3C, never on Frame:

- FAME left, factor 1.0.
- FAME left, factor 0.6.
- FAME right, factor 0.3.
- ALMI left, factor 0.3.

These gaps exceed the unchanged `0.05 s` continuity gate and remain honest
runtime flags, not physical fall/initialization failures. The final rebuilt grid
therefore contains 481 records: 480 first attempts plus the retained
FAME `right_overhang_inner_forward` 3C `a02` retry, with 0 initialization
failures and 13 infrastructure attempts.

## Effect On Iteration 6B

The rebuilt `overhang_forward` change does not reverse the 6B spatial conclusion.
Frame still has lower mean Segment Spatial RMS in 22/24 cells and Uniform Spatial
RMS in 22/24 cells. The positive exceptions remain FAME overhang `0.6x` and
`0.4x`, which no longer represent an initialization gap but rather a fully
executed comparison that favours 3C with four of ten paired wins. The other 18
trajectories and both directional banks were revalidated byte-identical and keep
their prior metrics. All the full-grid exports, the standard sweep plots and the
[analysis](counter_balance_analysis_iteration_6b.md) are regenerated from the
current bank.

## Cleanup

The temporary initialization probes (`iteration6d_preparation_tmp`) and the
superseded pre-change bank pair recordings were removed after their findings were
recorded in the table above. No unrelated run or challenge evidence was deleted.
