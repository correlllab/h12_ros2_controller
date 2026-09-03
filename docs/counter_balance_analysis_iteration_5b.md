# Counter-Balance Iteration 5B Analysis

## Status

Iteration 5B implemented and executed verified-response H3. It passed 60 ms
model, derivative, shadow, safety, and timing gates, but failed the ordinary
active-behavior gate. H3 is retained in shadow only and is not promoted. B0,
frozen 3C, and `counter_residual_h2_frozen` remain the baselines.

## 1. H3-Specific Hypothesis

Frozen H2 has one effective terminal residual because U5 realization is delayed
one tick. H3 tested whether a second effective residual and 60 ms terminal phase
could improve continue/brake timing without changing state, cost, trust,
authority, or sensing.

## 2. 60 ms Model Validation

Exact mechanical branches were extended from `20/40 ms` to `20/40/60 ms` across
FAME `11`, mirrored FAME `04`, ALMI right `11`, and ALMI manual. The model kept
the same real-compatible context and weak-joint mask.

### Initial Gate

- U5-3 passed with retained-joint sign accuracy `1.00` and positive improvement.
- R5-3 ranked actions at `0.983`, but roll sign was `0.867` before applying the
  preregistered `0.002 rad/s` material-effect threshold.
- N5-3 sign was `1.00/0.933`, but only 15 confident roll samples were available.

### Focused Repairs

R5 was refit at the 60 ms sample with the unchanged contextual full-matrix
structure. Sign evaluation was restricted to effects above the same physical
threshold already used by action ranking.

N5 retained the same rate/momentum structure, trained on mixed FAME/ALMI zeros,
and added independent ALMI zero runs only to satisfy the predeclared sample
count. No threshold or state changed.

### Final Gate

- U5 carryover: `[0.460, 0.524, 0.0, 0.341]`.
- U5 retained-joint sign: `1.00/1.00/1.00`.
- U5 retained-joint zero-baseline improvement: `0.978/0.997/0.970`.
- R5 roll/pitch sign: `0.916/0.984`.
- R5 ranking accuracy: `0.983` over 117 distinguishable pairs.
- R5 zero-baseline improvement: `0.751`.
- N5 roll/pitch sign: `1.00/0.98`.
- N5 confident samples: `30/50`.
- N5 coverage: `0.041/0.068`.

The final report is:

`runs/key_findings/iteration5/models/iteration5b_h3_model_gate_final.json`.

## 3. H3 Implementation

H3 retains the H2 12-element state. Its pending block stores realized residual
velocity:

1. Knot 0 applies U5 gain to `delta u_0` after the delay.
2. Knot 1 applies R5 response and propagates U5 carryover plus `delta u_1`.
3. Knot 2 applies the second effective response and stores `delta u_2`.
4. The unchanged terminal cost evaluates the 60 ms state.

The three action models, terminal model, shooting problem, and BoxFDDP solver are
preallocated and warm-started. Joint 2 remains fixed. N5/R5 confidence remains
per-axis. The shared 3C finalizer remains the sole active publication path.

All three running-model derivatives pass finite differences. H3 shadow preserves
the exact frozen-3C command.

## 4. Shadow Evidence

Across FAME `09/11/04`, ALMI right `11`, and ALMI manual:

- H3 model-valid decisions: 41.
- H3 confident H1/H2/H3 sign: `1.00/1.00` on every knot.
- H3 terminal RMSE: `0.0211/0.0203 rad/s`.
- Continue/brake/reverse decisions: `14/24/0` in the first complete panel.
- H3 p99/max: `4.13/5.30 ms`.
- Full-controller p99/max: `9.35/24.76 ms`.
- H3 model failures: zero.
- Published outcomes remained frozen 3C.

Sequence diagnostics confirmed that `delta u_1` is nonzero and changes the H3
terminal state. The inert `delta u_2` initially followed pending action because
of the generic change cost; removing only its stage-2 change penalty reduced it
below `7e-9`.

H3 shadow passed.

## 5. Active Ordinary Gate

H3 used the exact frozen-H2 `0.01 rad/s` trust region and costs. No parameter was
retuned.

### FAME `04`

- B0 contemporaneous result: stable, peak `0.0893`.
- 3C contemporaneous result: drift, peak `0.1009`.
- Frozen H2 contemporaneous result: stable, peak `0.0980`.
- H3 repetitions: stable/drift/stable.
- H3 median peak/RMS: `0.09847/0.04237`.

H3 does not improve frozen H2's 3/3 stable evidence and introduces one threshold
drift in three runs.

### ALMI Right `11`

- B0 contemporaneous result: stable, peak `0.0930`.
- 3C contemporaneous result: drift, peak `0.1021`.
- Frozen H2 contemporaneous result: drift, peak `0.1027`.
- H3 repetitions: drift 3/3.
- H3 median peak/RMS: `0.10302/0.08670`.

H3 is continuously worse than frozen H2 and remains below B0.

Because the ordinary gate failed, active rescue `09/11`, challenge `06`, and
stumble panels were not run. Frozen H2's prior rescue evidence remains the
required baseline; H3 makes no rescue-preservation claim.

## 6. Active Timing and Reliability

Across six H3 active ordinary runs:

- H3 p50/p95/p99/max: `0.91/2.37/3.42/5.19 ms`.
- Full-controller p50/p95/p99/max: `3.04/6.87/8.78/19.99 ms`.
- H3 accepted solves: `4104/4104`.
- H3 model failures: zero.
- Model-valid decisions: 73.
- Continue/brake/reverse decisions: `26/30/0` nonzero decisions.

Timing and reliability pass. They are not the limiting factors.

## 7. Final Decision

Iteration 5B is **stopped and not promoted**.

Verified 60 ms modeling and H3 implementation are successful, but horizon length
is not the missing capability. H3 reduces confidence coverage, adds no reverse
decision, and is worse than frozen H2 on both ordinary guards. Continuing to
rescue/stumble cells after that failure would violate the experimental ladder.

No new state, authority, terminal model, or H5 was attempted. Any future work
must begin from frozen H2 and identify a missing physical state or objective with
new evidence; it must not continue tuning H3.

## 8. Software Validation

- Benchmark-owned suite: `139 passed`.
- Focused H2/H3/controller/model suite: `97 passed`.
- H3 derivative/parity integration suite: `73 passed`.
- Python compilation and staged/unstaged `git diff --check`: passed.
- Iteration 5B implementation/config combined SHA-256:
  `797fcf464c212e6a3501118a59a3c5583889ec01ef859f93f9961e5c86137b4a`.
