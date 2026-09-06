# Counter-Balance Iteration 5 Analysis

## Status

Iteration 5 initially stopped at the U5/R5/N5 model gate, continued through a
focused SNR redesign, then implemented and activated H2 after the reduced model
passed. The best H2 controller is frozen but unpromoted. No H3/H5 controller was
attempted. B0 and frozen Iteration 3C remain the operational baselines.

The retained work is:

- A source-bound pre-refactor 3C oracle.
- A pure, non-publishing 3C nominal planner.
- One shared backtracking, safety, state-update, and publication path.
- A bounded residual-probe controller and synchronized diagnostics.
- Pure NumPy U5, R5, and N5 model/fitting/validation infrastructure.
- Model reports defining the trusted domain permitted for future H2 shadow use.

## 1. Infrastructure Gate

Experiments ran serially and headlessly. No stale simulator, ROS, bridge, policy,
or sweep process remained between batches. The final collection state retained
approximately `12 GiB` available memory with negligible swap use.

The repository has no coordinated checkpoint/restore path for MuJoCo, FAME/ALMI
hidden history, controller state, safety relay state, DDS queues, and timing.
MuJoCo-only state restore would not produce a valid matched branch. Therefore:

- No collected run is described as a matched counterfactual.
- Residual pulses are labeled statistical identification data.
- Sign-inverted fresh-run pairs are central statistical pairs, not saved-state
  branches.
- Simulator-only values are excluded from runtime model inputs.

## 2. Frozen 3C Refactor and Parity

The pre-refactor oracle is bound by:

`test/counter_balance_3c_oracle_manifest.json`.

Key identities are:

- Controller commit: `0ed45b5251fbf10bc43014b1e732482bf1bfe95d`.
- Base-controller Git blob: `69384524e218beeaefc5d6b9550c300b21a35d2a`.
- Base-controller SHA-256:
  `aa14a47a9d9da8c2a71c92f82c7f374ac1ee58a63142229a8d30f4dce8450ec3`.
- Canonical 3C combined SHA-256:
  `9c37c1b33e6b604a0ab1465c4d4ab5ee8f4515a9ef9b3f0d907dcd3794524801`.

The refactor added immutable nominal plan/solve/context records and extracted
the target/posture/velocity solve into `plan_frozen_3c_velocity()`. Planning has
no collision, publication, reference-capture, or final command-state side
effects. The shared finalizer remains the sole owner of backtracking, gravity,
command-state update, publisher clipping, and atomic publication.

Parity evidence:

- Pre-refactor focused suite: `44 passed`.
- Preserved-source side-by-side stateful cases: exact parity in three cases.
- Post-refactor planner/finalizer/4C/4B compatibility suite: `59 passed`.
- Pre-refactor FAME `11`: drift with precise tracking.
- Post-refactor FAME `11`: drift with precise tracking.
- Zero-residual probe FAME `11`: drift with precise tracking.

The physical continuous metrics varied between fresh runs, as expected from the
Iteration 4C reproducibility findings, but no classification, tracking, solver,
or safety regression appeared. Frozen 3C remains independently runnable.

## 3. Residual-Probe Implementation

`CounterResidualProbeController` overrides only residual selection between the
pure nominal planner and shared finalizer. It injects bounded joint or
momentum-aligned one/two-tick pulses and logs:

- Nominal, residual, combined, and applied counter velocity.
- LowState tick, unwrapped tick, sequence, and monotonic sample age.
- Counter position/velocity and planar momentum.
- Moving and nominal-counter momentum.
- IMU tilt and angular rate.
- Pulse step, seed, sign, and active state.

Zero, positive, negative, low-amplitude, and higher-amplitude schedules were run
in randomized orders. All completed FAME `11` probe trials remained drift with
precise tracking and no severe regression.

Across `9,568` released probe samples:

- Total-controller p50/p95/p99/max was
  `2.041/4.315/5.691/22.870 ms`.
- Solver p99/max was `2.673/20.135 ms`.

The p99 gate passed with margin. Rare maxima still exceeded one `20 ms` period
and remain relevant to any future H2 timing budget.

## 4. U5 Result

U5 consistently identified one controller-tick realization delay on all four
counter joints.

The higher-amplitude central-pair pilot estimated:

- Delay: `[1, 1, 1, 1]` ticks.
- Gain: `[0.161, 0.241, 0.194, 0.299]`.
- Carryover: `[0.297, 0.399, 0.262, 0.183]`.

The earlier low-amplitude fits also produced positive gains and one-tick delay.
This is useful action-realization evidence and validates the need for pending
residual state in any future H2 model.

U5 alone is not sufficient to authorize MPC. Its result must be combined with a
valid angular-response gradient.

## 5. R5 Result

The first low-amplitude statistical fit failed:

- Roll/pitch sign accuracy: `0.59/0.38`.
- Pitch prediction was worse than zero response.
- Fresh-run 95th-percentile angular noise was approximately `0.046 rad/s`.

Sign-inverted low-amplitude pairs did not recover the model:

- Roll/pitch sign accuracy: `0.45/0.36`.
- Both held-out axes were worse than zero response.

Pulse amplitude was then increased from `0.08/0.16` to `0.3/0.5 rad/s` while
remaining inside existing velocity and safety bounds. Median H1/H2 angular
effects remained only about `0.006-0.010 rad/s`; maxima were about
`0.025-0.026 rad/s`, still below the `0.048 rad/s` fresh-run noise bound.

The stronger central-pair fit also failed:

- Roll/pitch sign accuracy: `0.47/0.62`.
- Roll/pitch zero-baseline improvement: `-0.12/-0.17`.

A final diagonal physics-structured FIR test retained the expected immediate
negative response sign but still achieved only `0.50/0.56` sign accuracy and was
worse than zero response on both axes.

R5 therefore cannot provide a trustworthy `calcDiff()` action gradient. Stronger
pulses would identify behavior outside the intended small residual trust region
and were not justified.

Primary reports:

- `runs/key_findings_reports/iteration5/models/fame11_pilot.json`.
- `runs/key_findings_reports/iteration5/models/fame11_central_pairs.json`.
- `runs/key_findings_reports/iteration5/models/fame11_high_central_pairs.json`.

## 6. N5 Result

The minimal trend-based H2 nominal-phase predictor was calibrated and validated
on separate zero-residual FAME `11` runs.

Higher-amplitude-panel validation reported:

- Valid-state fraction: `1.0`.
- Roll/pitch rate RMSE: `0.0194/0.0096 rad/s`.
- Roll/pitch sign accuracy: `0.815/0.925`.

Pitch passed the `0.90` sign gate; roll did not. Known moving/nominal momentum was
added to new diagnostics for a future predictor, but no post-hoc feature fit was
used to relax the preregistered gate.

N5 therefore cannot reliably establish both controlled angular phases for H2.

## 7. Initial H2 Decision

The finalized design requires H2 to remain blocked when U5/R5 residual sign or
N5 nominal phase is unreliable. Both blocking conditions occurred.

Consequently:

- No Crocoddyl H2 action model was implemented.
- No unverified `calcDiff()` gradient was exposed to a controller.
- No H2 shadow or active command was run.
- No FAME `09/11/06` or ALMI promotion panel was entered.
- No H3/H5 expansion was considered.

Implementing H2 despite these failures would repeat the central H3-0 error:
temporal optimization without verified physical response.

## 8. Initial Verdict

At the initial statistical-identification checkpoint, Iteration 5 was
**stopped at the response-model gate**.

Successes:

- The pure 3C planner/shared finalizer architecture is implemented and parity
  verified.
- Existing 3C rescue behavior is retained in the physical parity sentinel.
- Residual injection uses one final safety/publication path.
- U5 identifies a repeatable one-tick delay and positive realization gains.
- Probe logging and model tooling are policy-blind, target-blind at runtime, and
  real-compatible.
- Probe timing p99 leaves substantial nominal margin.

Failures:

- Statistical fresh-run pulses do not separate the small residual angular effect
  from nominal phase variability reliably enough for R5.
- R5 fails sign, baseline-improvement, and action-gradient gates.
- N5 fails the roll phase-sign gate.
- No H2 behavior, additional FAME rescue, or ALMI stumble prevention can be
  claimed.

The next permissible work is a focused identification redesign, not MPC tuning:

- Improve short-horizon experimental matching or phase alignment without adding
  simulator dependencies to runtime.
- Use the newly logged moving/nominal momentum to test one preregistered N5 phase
  representation.
- Seek a higher signal-to-noise residual-response measurement inside the intended
  active trust region.
- Reopen H2 only after held-out R5 action ranking reaches `0.90` with nontrivial
  distinguishable coverage and N5 passes both axes.

B0 and frozen 3C remain the operational controllers.

## 9. Software Validation

- Benchmark-owned suite: `139 passed`.
- Focused controller/model/observation/H2 suite: `122 passed`.
- Final H2 derivative/controller unit suite: `39 passed`.
- Python compilation and staged/unstaged `git diff --check`: passed.
- Retained planner/probe/model/harness/config combined SHA-256:
  `258ccac38b325a2158f44da6aa506526e6046468b98bcb58381e0b48b5540e9a`.

The source hash excludes documentation, tests, and ignored run artifacts. The
pre-refactor 3C oracle remains independently bound by its original manifest.

## 10. Continued Identification and SNR Result

Iteration 5 continued without changing controller architecture or implementing
H2. The identification experiment replaced joint-basis pulses with desired
roll/pitch momentum probes computed from the live counter-arm momentum map using
a realization-weighted damped pseudoinverse. The schedules used symmetric signs,
one/two-tick pulses, eight-tick spacing, randomized order, and interleaved zero
intervals.

All continued FAME `11` probe runs remained drift with precise tracking and no
operational failure.

### 10.1 Complete U5 Statistics

The expanded central-pair panel provided `60` influenced held-out samples per
joint. U5 reported:

- Delay: `[1, 1, 1, 1]` ticks with `1.0` episode consistency.
- Gain: `[0.332, 0.292, 0.209, 0.477]`.
- Bootstrap 95% gain lower bounds: `[0.238, 0.208, 0.060, 0.384]`.
- Held-out sign accuracy: `[0.783, 0.933, 0.667, 0.850]`.
- Zero-baseline improvement: `[0.763, 0.761, 0.011, 0.825]`.
- Command-equals-realization improvement: `[0.947, 0.957, 0.468, 0.939]`.

Delay and positive realization are strongly supported. Joint 2 remains weak and
three axes miss the `0.90` sign gate, so U5 is still not fully verified.

### 10.2 Momentum-Aware N5

N5 added only moving-arm and nominal-3C momentum changes and used a third
untouched zero run for validation:

- Roll/pitch sign accuracy: `0.664/0.902`.
- Divergence accuracy: `0.669/0.897`.
- Peak-order accuracy: `0.676/0.912`.
- Rate RMSE: `0.0204/0.0075 rad/s`.

Pitch approaches or passes the phase gates. Roll remains materially below every
phase gate. The additional momentum features do not solve nominal roll phase.

### 10.3 Within-Run Innovation R5

R5 used U5-predicted residual momentum and angular-rate innovation relative to
N5 from the same current state. Across `390` leave-one-schedule-out samples:

- Distinguishability coverage: `0.113`.
- Ranking coverage/accuracy: `0.172/0.537`.
- Roll/pitch sign accuracy: `0.519/0.578`.
- Roll/pitch zero-baseline improvement: `-0.012/0.018`.
- Diagonal gradient-sign consistency across schedules: `1.0/1.0`.

Momentum alignment raises measurable coverage, but candidate ranking remains
near chance and roll remains worse than zero prediction. Statistical R5 does not
pass.

Primary report:

`runs/key_findings_reports/iteration5/models/fame11_momentum_innovation_v2.json`.

### 10.4 Narrow Mechanical Branch

Because improved statistical probes still failed, a `60 ms` offline mechanical
branch replayed identical recorded lower-body/moving-arm controls from one exact
MuJoCo state and varied only counter-arm actuator controls after the identified
one-tick delay. Repeated zero branches were exactly equal.

Across three amplitudes and `96` leave-one-amplitude-out samples, the diagonal
mechanical model found:

- `G0 = diag(-0.079, -0.384)`.
- Gradient-sign consistency: `1.0/1.0`.
- Roll/pitch zero-baseline improvement: `0.660/0.722`.
- Roll/pitch sign accuracy: `0.729/0.844`.
- Roll/pitch RMSE: `0.0036/0.0042 rad/s`.

This verifies a stable immediate negative mechanical response direction and is a
useful prior. It excludes online policy response and still misses the `0.90`
sign gate, especially in roll, so it cannot authorize H2 by itself.

Applying the mechanical diagonal prior to the full asynchronous innovation data
also failed:

- Roll/pitch sign accuracy: `0.56/0.31`.
- Roll/pitch zero-baseline improvement: `-0.01/-0.45`.

Immediate mechanics therefore do not substitute for the missing measured-phase
closed-loop response.

Primary report:

`runs/key_findings_reports/iteration5/models/fame11_mechanical_response_aggregate.json`.

### 10.5 Continued Decision

At this stage H2 remained closed. The initial SNR redesign improved data
quantity, U5 confidence, mechanical response consistency, and statistical
distinguishability, but did not yet produce reliable nominal roll phase or
cross-family action ranking. Section 11 records the subsequent focused model
refinement.

## 11. Good-Enough Reduced Model Identification

Further work retained the same Iteration 5 architecture and added only physical
conditioning variables selected by held-out failures.

### 11.1 Failure Localization

Action-aligned evaluation showed that the first FAME `11` mechanical model
already explained `83%` of targeted response energy with `91.7%/97.9%`
roll/pitch sign accuracy. Cross-policy validation localized failures to:

- Missing axial-vector canonicalization for right-counter ownership.
- Cross-axis residual momentum omitted by the diagonal response model.
- Missing physical ownership context when right-counter data were held out.
- Near-zero U5 responses counted as sign errors.
- N5 predictions evaluated during imminent zero crossings where H2 should
  abstain.

These are representation and confidence failures, not evidence that a reduced
response model is impossible.

### 11.2 Final U5 Structure

The exact mechanical branch now synthesizes residual actuator effort from the
requested velocity and recorded low-level `kp/kd` gains rather than subtracting
fresh-run actuator controls. U5 uses measured per-joint position/velocity and
ownership side to predict one-tick realization.

With a preregistered `0.01 rad/s` physical distinguishability threshold:

- Retained joints 0/1/3 sign accuracy: `1.00/1.00/0.95`.
- Retained-joint zero-baseline improvement: `0.938/0.994/0.953`.
- Distinguishable samples: `160/156/160`.
- Joint 2 sign accuracy: failed and is explicitly masked.
- The remaining three-joint momentum map stays rank two and well conditioned.

### 11.3 Final R5 Structure

R5 is a compact full `2x2` momentum-to-angular-rate matrix whose gains vary
bilinearly with only:

- Canonical IMU tilt.
- Canonical IMU angular rate.
- Counter-arm ownership side.

No simulator height, policy identity, target identity, contact truth, or learned
latent state is used. Right-counter axial quantities use the physical
`[-1, +1]` roll/pitch mirror.

The final leave-one-family/amplitude-out dataset contains `224` exact mechanical
branch samples from FAME `11`, mirrored FAME `04`, and ALMI `11`:

- Targeted H2 sign accuracy: `1.00/1.00`.
- Targeted zero-baseline improvement: `0.940`.
- Targeted RMSE: `0.00525 rad/s`.
- Gradient diagonal-sign consistency: `1.00/1.00`.
- Continue/brake ranking accuracy: `0.911`.
- Ranking coverage: `112/112` distinguishable pairs.

Primary report:

`runs/key_findings_reports/iteration5/models/cross_policy_verified_response_model.json`.

### 11.4 Final N5 Confidence Gate

N5 retains the compact rate-trend plus moving/nominal momentum-change model. A
95th-percentile error bound from two fixed calibration runs defines a physical
no-zero-crossing confidence gate. The residual controller must use zero
correction on an uncertain axis.

Across six held-out zero-residual runs spanning FAME `11`, mirrored FAME `04`,
and ALMI `11`:

- Confident samples: `34/128` roll/pitch.
- Confidence coverage: `0.083/0.314`.
- H2 phase-sign accuracy: `1.00/0.992`.
- FAME `04` roll correctly abstains rather than extrapolating through a likely
  zero crossing.

Primary report:

`runs/key_findings_reports/iteration5/models/cross_policy_nominal_phase_confidence.json`.

### 11.5 Revised H2 Decision

The minimum identification gates now pass inside an explicit trusted domain:

- U5 disables one weak counter-joint direction.
- N5 abstains per axis when phase can cross zero inside H2.
- R5 uses only real-compatible tilt/rate/ownership context.
- Exact mechanical action gradients and continue/brake rankings pass across
  FAME and ALMI held-out folds.

H2 was not implemented in this continuation, as requested. The evidence is now
sufficient to reopen **H2 shadow implementation only**. Shadow validation must
confirm the complete asynchronous prediction/timing path before any residual is
published.

## 12. H2 Shadow and Active Development

### 12.1 H2 Implementation

Iteration 5 added a fixed two-running-knot Crocoddyl problem with a 12-element
state:

- Incremental planar tilt and angular rate.
- Incremental four-joint counter position.
- Pending four-joint residual action.

Knot 0 stores the proposed residual. Knot 1 applies the verified U5 realization
and R5 angular response after the one-tick delay. Joint 2 is fixed at zero.
Terminal costs use absolute N5 tilt/rate, positive divergence, and residual arm
reserve. Running costs regularize residual magnitude and change.

The solver and action/terminal data are preallocated and warm-started. Both
running models and the terminal gradient pass central finite differences.
Frozen 3C remains the nominal and the shared finalizer remains the only
backtracking/safety/publication path.

### 12.2 Shadow Repair Chain

The first shadow run produced zero model-valid ticks because it required both N5
axes to be confident simultaneously. This contradicted the verified per-axis
abstention contract. The fix masks uncertain R5 rows and terminal costs per axis
and permits a solve when either axis is confident.

Final shadow evidence across FAME `09/11/04`, ALMI `11`, and ALMI manual was:

- Model-valid decisions: `103`.
- Model-valid nominal H2 sign: `1.00/0.968`.
- Model-valid nominal H2 RMSE: `0.0195/0.0146 rad/s`.
- Continue/brake/reverse decisions: `29/74/0`.
- H2 p99/max: `3.52/5.29 ms`.
- Full-controller p99/max: `8.95/20.10 ms`.
- H2 model failures: zero.

ALMI manual initially abstained because the offline branch utility used incorrect
right-arm `qpos/qvel` offsets in the Magpie model. Named MuJoCo joint addresses
fixed the hand-DOF offset. U5/R5 were regenerated with exact PD residual effort,
revalidated across FAME and ALMI, and only then returned to shadow.

### 12.3 Active Candidates

Candidate A used the `0.01 rad/s` trust region with default action/change weights
`1.0/0.5`. Median residual was below `0.001 rad/s`; it was behaviorally neutral
and was rejected as sub-distinguishable.

Candidate B changed only action/change weights to `0.01/0.005`. Model, H2,
confidence, trust region, weak-joint mask, and safety were unchanged. This is the
retained `counter_residual_h2_frozen` controller.

Across 14 retained-candidate development runs:

- H2 p50/p95/p99/max: `1.12/2.85/3.48/5.06 ms`.
- Full-controller p50/p95/p99/max: `3.16/7.07/8.80/25.89 ms`.
- Model-valid decisions: `278`.
- Continue/brake/reverse nonzero decisions: `59/103/0`.
- Median/max residual norm: `0.00089/0.01571 rad/s`.
- H2 model failures: zero.
- Underlying frozen-3C solver failures: one, on ALMI manual.

### 12.4 Repeated Outcomes

Ordinary FAME `04`:

- B0: stable 3/3, median peak drift `0.0904`.
- 3C: stable 2/3, median peak drift `0.0978`.
- Frozen H2: stable 3/3, median peak drift `0.0984`.

H2 improves the near-threshold majority classification but does not reach B0
continuously.

Ordinary ALMI right `11`:

- B0: stable 3/3, median peak drift `0.0944`.
- 3C: drift 3/3, median peak/RMS `0.1027/0.0857`.
- Frozen H2: drift 3/3, median peak/RMS `0.1030/0.0851`.

H2 slightly improves RMS but does not improve severity or reach B0.

FAME rescues:

- Target `09`: 3C drift 3/3; H2 drift 3/3. Median peak is effectively neutral
  (`0.10895` versus `0.10923`) and RMS slightly improves.
- Target `11`: 3C drift 3/3; H2 drift 3/3. Median peak improves from `0.1661` to
  `0.1558`; RMS improves from `0.1101` to `0.1052`.

FAME challenge `06` remained fall for Frame, 3C, and H2 in the screening run.
H2 did not justify rescue repetitions.

ALMI manual-grasp-minus remained stumble. Both 3C and H2 had one underlying
one-step solver failure, making the run operationally incomplete. Foot travel and
base response did not show a material prevention signal, so no favorable retry
or promotion claim is made.

### 12.5 Final Verdict

Iteration 5 H2 is **frozen but not promoted**.

Successes:

- Complete 3C nominal behavior and both repeated rescues are retained.
- FAME `11` improves materially and FAME `04` improves majority classification.
- No new majority severe regression appears.
- H2 model, derivative, confidence, weak-joint, fallback, and p99 timing gates
  pass.

Unmet goals:

- ALMI ordinary behavior remains below B0.
- Target `06` remains a fall.
- No repeated ALMI stumble prevention occurs.
- One extreme ALMI cell still exposes the inherited 3C one-step solver failure.

The clearest limit is temporal. One-tick U5 delay leaves H2 with one effective
terminal residual; the second optimized action cannot affect H2. A meaningful
H3 requires new verified 60 ms N5/R5 evidence. Extending the horizon or adding
larger costs without that evidence would be ad-hoc and would repeat H3-0.
