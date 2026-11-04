# Counter-Balance Iteration 5D

## Cross-Policy Predictive Counter-Balance

## Status and Goal

Iteration 5D starts from frozen H2 and the finalized ALMI checkpoint sweep. B0,
3C, and H2 remain independently runnable.

The goal is one policy-blind controller that retains FAME rescues `09/11`, keeps
repeatable ALMI boundary improvements, removes cross-policy H2 regressions, and
recovers additional falls where existing authority is physically useful.

## 1. Source Evidence

Repeatable H2 improvements:

- MJLab ALMI stand right boundary `11`: drift 3/3 → stable 3/3.
- MJLab ALMI manip left boundary `11`: drift 3/3 → stable 3/3.
- MJLab ALMI manip left lateral-high-reach: majority drift → stable 3/3.
- MJLab ALMI manip right-arm-overhead: Frame fall 3/3; H2 physically stable in
  one run but operationally incomplete in two.

Repeatable H2 regressions:

- MJLab ALMI stand right-forward-rank6: drift 3/3 → fall 3/3.
- MJLab ALMI stand right-diagonal-rank6: majority drift → majority fall.
- MJLab ALMI stand right-arm-forward-yaw: stable 3/3 versus H2
  drift/stable/stable.

The evidence rejects larger residual authority. It selects phase/risk
interpretation and solver/fallback reliability for diagnosis.

## 2. Frozen Contract

Preserve complete 3C nominal plus H2 residual, joints 0/1/3, joint-2 mask,
`0.01 rad/s` trust, U5/R5/N5, confidence abstention, Crocoddyl formulation,
single safety/publication path, real-compatible sensing, and policy blindness.

Do not use checkpoint, policy, target, outcome label, simulator truth, or new
state bundles at runtime.

## 3. Development Method

Compare positive and regression traces for:

- Predicted terminal cost improvement and uncertainty.
- N5 confidence/context validity.
- Continue/brake/reverse phase and residual projection.
- Residual magnitude/timing relative to peak/divergence.
- Solver/fallback status and counter-arm excursion.

Select one smallest mechanism. Candidate mechanisms, in evidence order, are:

1. Reject residuals whose predicted terminal benefit is below a predeclared
   absolute/relative margin.
2. Require consistent benefit across both H2 knots or uncertainty bounds.
3. Improve solver/fallback acceptance without changing desired residual.
4. Add one verified risk state only if existing predictions cannot separate
   positive and negative cells.

Change one mechanism at a time.

## 4. Experimental Ladder

### Development Panel

- Stand right boundary `11` and manip left boundary `11` positive cells.
- Manip left lateral-high-reach positive cell.
- Stand right-forward and right-diagonal regression guards.
- Manip right-arm-overhead solver-obscured opportunity.

Use Frame, frozen H2, and 5D with three repetitions for changed cells.

### Frozen FAME Gate

- Preserve rescues `09/11` for three screening repetitions.
- Test challenge `06` only after cross-policy guards pass.
- Add other Hard/Boundary falls only when the mechanism generalizes.

### Expansion

Use five repetitions only for a rescue or promotion claim. Keep H2; H3/H5 are
excluded without new horizon evidence.

## 5. Gates

Required:

- Retain both FAME rescues and all repeated ALMI H2 improvements.
- Remove or materially reduce repeated H2 regressions.
- Add no majority severe regression or tracking/safety failure.
- Keep complete-controller p99 below `15 ms`.

Strong success:

- Repeatably rescue manip right-arm-overhead or FAME `06`/another fall.

Stop if positive and negative cells cannot be separated by real-compatible H2
prediction evidence, or if every selective gate removes benefits together with
regressions. That result would require a new physical risk model rather than H2
tuning.

## 6. Execution Result

Existing H2 prediction evidence cannot separate repeated positive and regression
cells. Predicted benefit, residual magnitude, continue/brake label, and
confidence axis overlap. No selective H2 gate was implemented because every
candidate would remove verified improvements together with regressions.

Iteration 5D stops without a new active controller. Frozen H2 remains the
baseline; further progress requires a new support-risk/response architecture.
