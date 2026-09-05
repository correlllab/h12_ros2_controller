# Counter-Balance Iteration 5C Analysis

## Status

Iteration 5C stopped at the residual-action sensitivity gate. It added passive
real-compatible support/foot diagnostics to H2/H3 runtime logging but did not
change H2 commands, costs, state, trust, or safety. Frozen H2 remains the best
Iteration-5 controller.

## 1. Source Evidence

The finalized speed sweep found:

- Manual-grasp-minus and overhead stumble at every tested `1.5-12 s` duration.
- Frozen H2 does not shift a standing/stumble speed boundary.
- H3 does not improve the slowest `12 s` H2 result.
- Manual torso response is nearly speed-invariant.
- Slow overhead steps several meters despite peak tilt below `0.10 rad`.
- Existing support validity remains true through many physical stumble traces.

This rejected disturbance speed and horizon length as the primary missing
mechanisms and selected pre-step foot/support risk for validation.

## 2. Passive Signal Integration

The existing Iteration-4 `BaseResponseObservationPipeline` was enabled passively
for residual-probe, H2, and H3 variants. It logs, without changing commands:

- Atomic tick/sequence/age metadata.
- Canonical foot position, rotation, and twist.
- Combined support pose/twist, confidence, and normalized residual.
- Existing support-validity hysteresis.
- CoM/support quantities and base height.
- IMU orientation/rate and moving-arm momentum.

Observer integration tests passed and H2 publication remained unchanged.

## 3. Signal Selection

A non-stumble ALMI right-`11` run calibrated the 99th-percentile foot linear-speed
reference at `0.467 m/s` and support normalized residual at `0.936`.

### Manual-Grasp-Minus

- `12 s` foot-speed onset: `2.96 s` after release.
- `12 s` foot-displacement onset: `3.54 s`.
- Angular tilt/rate threshold onset: about `7.93 s`.
- Foot twist leads displacement by `0.58 s` and angular risk by about `5 s`.
- Support validity never becomes false.

At nominal `1.5 s`, foot speed rises at `0.10 s`, displacement at `0.87 s`, and
angular thresholds only at `1.62-1.70 s`.

### Overhead

- `12 s` foot-speed onset: `0.44 s`.
- `12 s` foot-displacement onset: `3.76 s`.
- Angular tilt/rate threshold onset: `8.21-9.31 s`.
- Foot twist leads displacement by `3.32 s` and angular risk by more than `7 s`.
- Support validity never becomes false.

Foot linear twist is therefore the smallest signal adding repeatable pre-step
information beyond torso tilt/rate across both families. Support normalized
residual is less consistent, and CoM/ZMP terms were not advanced.

## 4. Residual-Action Sensitivity

Symmetric low-amplitude momentum-aligned residual probes were rerun with the
passive observer enabled. H1/H2 foot-speed differences were compared between
sign-inverted schedules; non-pulse intervals supplied the fresh-run noise bound.

### Manual-Grasp-Minus

- H2 samples: 12.
- Median/max residual foot-speed effect: `0.048/0.225 m/s`.
- 95th-percentile fresh-run variation: `0.896 m/s`.
- Distinguishable samples: zero.

### Overhead

- H2 samples: 12.
- Median/max residual foot-speed effect: `0.013/0.122 m/s`.
- 95th-percentile fresh-run variation: `3.54 m/s`.
- Distinguishable samples: zero.

The signal is predictive but not measurably controllable through frozen H2's
residual authority and horizon. Sign, response-improvement, and `0.90` ranking
gates cannot be evaluated on a distinguishable set.

## 5. Decision

No risk residual was inserted into Crocoddyl. A cost on foot twist without a
verified action gradient would encourage unsupported optimizer behavior and
violate the 5C design.

Consequently:

- No 5C active controller exists.
- No ordinary or FAME rescue gates were rerun because commands are unchanged.
- Frozen H2's FAME `09/11` evidence remains authoritative.
- Target `06` remains unrescued.
- No ALMI stumble-boundary or standing claim is made.

## 6. Final Interpretation

Iteration 5C is **stopped before control modification**.

The ALMI stumble is observable before large foot travel, but the current
counter-arm residual does not change that risk above variation. The supported
limit is control authority/coupling to support dynamics, not lack of a warning
signal.

Future work must change one of the frozen assumptions before a support-aware MPC
cost is meaningful, for example:

- Coordinate with lower-body support/step control.
- Identify a larger but safe counter-arm authority envelope.
- Add a directly controllable support-placement action.
- Use foot twist only for conservative handoff/abstention.

Those are architectural changes outside Iteration 5C. Further H2/H3 risk-cost
tuning with the current authority would be ad hoc.

## 7. Artifacts and Validation

Passive signal roots:

- `runs/key_findings/20260903_193923_20260904_iter5c_signal_manual_quasi12`.
- `runs/key_findings/20260903_194032_20260904_iter5c_signal_manual_nominal`.
- `runs/key_findings/20260903_194123_20260904_iter5c_signal_overhead_quasi12`.
- `runs/key_findings/20260903_194226_20260904_iter5c_signal_overhead_nominal`.

Residual-sensitivity roots:

- `runs/key_findings/20260903_194602_20260904_iter5c_foot_probe_manual_plus`.
- `runs/key_findings/20260903_194650_20260904_iter5c_foot_probe_manual_minus`.
- `runs/key_findings/20260903_194739_20260904_iter5c_foot_probe_overhead_plus`.
- `runs/key_findings/20260903_194822_20260904_iter5c_foot_probe_overhead_minus`.

- Benchmark-owned tests: `139 passed`.
- Python compilation and staged/unstaged diff checks: passed.
- Speed/observer/config combined SHA-256:
  `03e847e17bcf01cb66d49112578b4d1f32329f9c094c5aedfd772e4d6f9af8ac`.
