# Counter-Balance Iteration 5D Analysis

## Status

Iteration 5D stopped before controller modification. The ALMI checkpoint sweep
provides repeatable cross-policy H2 improvements and regressions, but frozen H2's
existing real-compatible prediction state cannot distinguish them. Frozen H2
remains independently runnable and no 5D active variant was created.

## 1. Source Evidence

Positive repeated cells:

- MJLab ALMI stand right boundary `11`: Frame drift 3/3 → H2 stable 3/3.
- MJLab ALMI manip left boundary `11`: Frame drift 3/3 → H2 stable 3/3.
- MJLab ALMI manip left lateral-high-reach: Frame majority drift → H2 stable 3/3.

Negative repeated cells:

- MJLab ALMI stand right-forward-rank6: Frame drift 3/3 → H2 fall 3/3.
- MJLab ALMI stand right-diagonal-rank6: Frame majority drift → H2 majority fall.
- MJLab ALMI stand right-arm-forward-yaw: Frame stable 3/3 versus H2
  drift/stable/stable.

Solver-obscured opportunity:

- MJLab ALMI manip right-arm-overhead: Frame fall 3/3; H2 one stable and two
  operationally incomplete physical runs.

## 2. Prediction Diagnosis

Positive and negative cells overlap in all current H2 selection quantities.

| Cell | Median benefit | Median residual | Continue | Brake |
| --- | ---: | ---: | ---: | ---: |
| Stand boundary `11` improvement | `0` | approximately `0` | 27 | 10 |
| Manip boundary `11` improvement | `0` | approximately `0` | 0 | 96 |
| Stand right-forward regression | `6.7e-5` | `0.00375` | 6 | 25 |
| Stand right-diagonal regression | `1.1e-4` | `0.00584` | 20 | 12 |

The improvements are not consistently associated with larger predicted benefit
or one phase label. A benefit threshold would preferentially remove boundary
improvements while retaining some regressions.

Confidence also does not separate them:

- Stand right-forward/diagonal regressions are pitch-only confident.
- Manip left boundary `11` improvement is also entirely pitch-only confident.
- Stand boundary `11` is mostly pitch-only and has no nonzero both-axis action.

Therefore, requiring both axes, pitch-only, stronger confidence, continue-only,
or brake-only cannot remove regressions without removing verified improvements.

## 3. Rejected Mechanisms

### Predicted-Benefit Margin

Rejected offline. Positive and negative distributions overlap, and repeated
boundary improvements often have zero median modeled benefit because most valid
ticks abstain while a small subset drives the physical change.

### Decision-Phase Gate

Rejected offline. Both continue and brake actions occur in improvements and
regressions. The manip boundary improvement is brake-only, while the strongest
stand regression is also brake-dominant.

### Confidence-Axis Gate

Rejected offline. Pitch-only confidence is shared by both repeatable improvement
and regression cells.

### Increased Authority

Rejected by frozen evidence. Current residuals already convert some drift/fall
cells while causing severe stand regressions. More authority would amplify an
unresolved risk interpretation.

## 4. FAME Objective

No new 5D candidate passed the cross-policy development gate, so FAME rescue
`09/11` and challenge `06` were not rerun under an unchanged controller. Frozen
H2's repeated FAME rescue evidence remains authoritative; `06` remains
unrecovered.

Focusing only on FAME with scalar H2 tuning would ignore the newly proven ALMI
regressions and violate the single policy-blind-controller objective.

## 5. Architectural Limit

The current H2 state predicts short angular response but not whether the
lower-body policy will convert that response into stabilizing support action or a
fall. The ALMI checkpoint sweep demonstrates opposite physical consequences from
similar pitch confidence, residual magnitude, predicted benefit, and phase.

The missing quantity must describe controllable support risk/response, not merely
another scalar threshold. Iteration 5C already showed that foot twist predicts
stepping but has no measurable gradient under current residual authority.

Further progress requires one genuine architectural change, such as:

- A verified lower-body/support response model conditioned on proprioceptive
  foot/contact-wrench estimates.
- Coordinated lower-body support or foot-placement action.
- A solver/fallback path that preserves physical H2 rescue authority when the
  one-step nominal solve fails.

These cannot be introduced as H2 parameter tuning.

## 6. Final Decision

Iteration 5D is **stopped with no new controller**.

The checkpoint sweep supplies valuable positive cells for future model
identification, especially opposite-side boundary `11`, and mandatory negative
guards at stand right-forward/right-diagonal. However, the current H2 prediction
contract cannot distinguish them using policy-blind existing signals.

Frozen H2 remains the best controller. A future iteration must explicitly add
and verify support-response information before active tuning.
