# Counter-Balance Iteration 4B

## Adaptive-Authority Crocoddyl Control

## Goal

Iteration 4B develops one strongest practical controller from the frozen
Iteration 3C high-authority foundation.

The final controller should:

- Retain at least the two repeated Iteration 3C FAME fall rescues.
- Seek additional FAME rescues when evidence supports more authority or better
  timing.
- Improve ordinary FAME and ALMI behavior toward or beyond B0.
- Add no new majority fall or other severe regression.
- Preserve exact manipulation-arm behavior.
- Remain real-compatible and policy-blind.

ALMI stumble improvement is actively evaluated but is not required for initial
promotion.

Iteration 4B is one progressive controller-development workflow. Intermediate
experiments are ablations and decision points, not permanent controller variants.

## Frozen Foundation

Iteration 3C remains unchanged and independently runnable:

- One-step velocity-level Crocoddyl control.
- Iteration 2 `momentum_wide` objective and wide bounds.
- Early high counter-arm authority.
- Existing references, lifecycle, collision checks, safety, and publication.

For the scheduler-only H1 controller, full adaptive authority must reproduce
Iteration 3C. A later H3 extension intentionally adds temporal optimization and
is compared against, not equated to, H1.

Let `beta_3C(t)` be the frozen Iteration 3C lifecycle scale. Iteration 4B uses:

\[
\beta_{4B}(t)=\beta_{3C}(t)\alpha(t),
\qquad
0\le\alpha\le1.
\]

Authority enters the existing Crocoddyl cost/target scaling. Do not post-blend an
unvalidated velocity command after optimization.

## Invariants

- Manipulation position, velocity, torque, tracking, and publication remain
  identical to Iteration 3C/Iteration 2.
- Only four proximal counter joints are controlled.
- Existing position, velocity, excursion, collision, publisher, timing, and
  estop contracts remain authoritative.
- No policy name, target ID, arm-side behavior branch, historical outcome, or
  privileged simulator signal enters control.
- Inputs are limited to commanded manipulation preview, IMU, measured joint
  position/velocity/torque, and Pinocchio-derived quantities.
- Scheduler state resets on episode, reference, ownership, stale-state, and
  fault reset.

## Real-Compatible Signals

### Manipulation Preview

The upstream command generator supplies an aligned `0.20 s` preview of
manipulation-arm position and velocity with monotonic timestamps. Sample zero is
the currently published command.

At each preview sample, compute moving-arm planar momentum with Pinocchio while
holding measured lower/counter configuration at the current state:

\[
H_{m,k}=A_{m,xy}(q_k)\dot q_{m,k}.
\]

Approximate rate by adjacent preview differences:

\[
\dot H_{m,k}
=
\frac{H_{m,k+1}-H_{m,k}}{\Delta t}.
\]

Reject stale, nonfinite, misaligned, or incomplete preview. Invalid preview
contributes no feedforward authority and is logged.

### Measured Balance Response

Use settled-reference quantities:

- Roll/pitch error.
- Planar angular velocity.
- Positive tilt-rate divergence.
- Existing counter state and safety margins.

No full base-state predictor is required.

## Initial Adaptive Authority

Start with one simple authority signal.

Define normalized preview and response risks:

\[
\rho_m
=
\max_k
\left\|
\begin{bmatrix}
H_{m,k}/s_H \\
\dot H_{m,k}/s_{\dot H}
\end{bmatrix}
\right\|,
\]

\[
\rho_b
=
\left\|
\begin{bmatrix}
e_\theta/s_\theta \\
\omega_{xy}/s_\omega
\end{bmatrix}
\right\|
+w_d
\left\|
\max(e_\theta\odot\omega_{xy},0)
\right\|.
\]

Use smooth monotonic maps identified from development distributions:

\[
\alpha_{ff}=S_m(\rho_m),
\qquad
\alpha_{fb}=S_b(\rho_b).
\]

Use one cubic smoothstep definition for every ramp:

\[
S(\rho;e,f)=3z^2-2z^3,
\qquad
z=\operatorname{clip}\left(\frac{\rho-e}{f-e},0,1\right).
\]

Fit scheduler quantities on the pooled development set only:

- `s_H` and `s_Hdot`: pooled 95th-percentile nonzero norms during manipulation.
- `s_theta`: `0.10 rad`, the frozen base-drift scale.
- `s_omega`: `0.25 rad/s`, the frozen standing-response scale.
- `w_d`: `1.0` initially; change only through a separate ablation.
- Ramp entry/full values: pooled 50th/90th percentiles of each normalized risk.

Use the same pooled scales and ramps for both policies, both arms, and all target
families. Freeze them before active runs.

Before the predicted disturbance peak, preserve preventive feedforward. After
the peak, allow low measured response to attenuate unnecessary authority:

\[
g_c=
\begin{cases}
1, & k_{peak}>0,\\
S_c(\rho_b), & k_{peak}=0,
\end{cases}
\]

`k_peak` is the first preview index maximizing `||Hdot_m||`. A peak at sample
zero means the disturbance is no longer rising inside the available preview.
`S_c` uses the same response-risk entry/full values as `S_b` initially.

\[
\alpha^*=\max(g_c\alpha_{ff},\alpha_{fb}).
\]

Rate-limit authority, with faster increase than decrease:

\[
\alpha_{k+1}
=
\operatorname{clip}
\left(
\alpha^*,
\alpha_k-\Delta\alpha_{down},
\alpha_k+\Delta\alpha_{up}
\right).
\]

Initialize `alpha=0` at reference capture. Reset authority, peak phase, and all
filter history on episode, ownership, reference, preview-validity, or fault reset.

Initial slew is fixed before A1:

- Full-scale rise time: `0.10 s`, so `Delta alpha_up = dt / 0.10`.
- Full-scale decay time: `0.30 s`, so `Delta alpha_down = dt / 0.30`.

Change one slew time only through a separate preregistered ablation after A1;
do not tune both from outcome labels.

This is the first active hypothesis, not the assumed final design.

Treat it as an assumption test, not a scheduler-parameter project. Use the
deterministic scales, percentile ramps, and slew times above for the first active
experiment. Do not extensively tune percentiles, slew, divergence weight, or
normalization before deciding whether the remaining limitation is preview timing,
directional response, or feedforward/feedback coupling.

## Progressive Workflow

### Step 0: Freeze Evidence and Splits

Freeze:

- Iteration 3C code/config/hash and repeated rescue outcomes.
- B0 comparison results.
- Correct classifier revision and policy capabilities.
- Development, model-selection, and untouched evaluation families.

Current repeated FAME rescues are:

- `right_fast_fall_search_09_scale_78`.
- `right_fast_fall_search_11_scale_78`.

`right_fast_fall_search_06_scale_74` is a high-risk non-rescue sentinel.

### Step 1: Shadow Signal Validation

Hypothesis: preview identifies motions needing early authority, while post-peak
measured response distinguishes difficult recovery from ordinary FAME and stable
ALMI.

Run frame, B0, and 3C commands unchanged. Log:

- Preview momentum/rate and time to peak.
- Response risk and divergence.
- Proposed authority and slew.
- Existing 3C lifecycle scale.
- Counter motion and safety margins.

Retain only signals with repeatable sign, scale, and mirrored behavior. Freeze
normalization before active experiments. Step 1 selects valid signals and checks
their assumptions; it does not optimize scheduler parameters against outcomes.

### Step 2: Simple Adaptive Authority

Apply the initial authority scheduler to the frozen one-step 3C Crocoddyl solve.

Change no objective, bound, excursion, horizon, or solver setting.

Development panel:

- FAME rescue targets `09` and `11`.
- FAME non-rescue target `06`.
- Ordinary FAME cells where 3C differs from B0.
- Stable/drift ALMI right-boundary and lateral/overhang guards.
- ALMI stumble cases as safety/bonus diagnostics.

Use five paired repetitions for fall/stochastic cases and at least three for
ordinary guards.

Keep this mechanism only if:

- Both repeated 3C rescues remain.
- No new severe regression appears.
- At least one ordinary majority outcome improves relative to 3C, or a
  preregistered meaningful continuous metric improves with unchanged severity.
  Eligible metrics include base drift, counter excursion, clipping, collision
  backtracking, and another physically justified response metric frozen before
  the run.

### Step 3: Diagnose the Remaining Limitation

Use Step-2 evidence to select exactly one next mechanism.

Possible decisions:

- If rescue authority arrives too late or braking/recovery timing is poor, test
  short-horizon velocity-level Crocoddyl control.
- If preview timing is adequate but response direction is repeatedly wrong, test
  weak directional base-response shaping.
- If feedforward and feedback need different timing/scales, separate their
  authority channels.
- If no repeatable limitation is identified, freeze Step 2 and stop adding
  complexity.

Do not test these mechanisms simultaneously.

## Optional Short-Horizon Extension

Use only when Step 2 shows a temporal limitation.

First implement H1 with no temporal term and require exact Iteration 3C command
parity at `alpha=1`.

If Step 2 identifies a temporal limitation, extend to H3 with state and control:

\[
x_k=
\begin{bmatrix}
q_{c,k} \\
v_{c,k-1}
\end{bmatrix},
\qquad
u_k=v_{c,k}.
\]

Dynamics are:

\[
q_{c,k+1}=q_{c,k}+\Delta t v_{c,k}.
\]

\[
v_{c,k+1}^{previous}=u_k.
\]

Initialize `v_previous` from the previous accepted/published counter velocity. On
episode or controller reset, initialize it from measured counter velocity clipped
to the frozen 3C bounds.

Use the known manipulation preview at each knot and retain the 3C running cost.
Define optional temporal residuals:

\[
r_{\Delta v,k}
=
\frac{u_k-v_{c,k-1}}{s_{\Delta v}},
\]

\[
r_{brake,i,k}
=
\frac{
\operatorname{softplus}
\left(
(u_{i,k}^{dir})^2-2a_{brake,i}d_i^{dir}(q_k)
\right)
}{s_{brake,i}},
\]

where `d_dir` is remaining excursion in the direction of velocity. Hard position,
velocity, excursion, collision, and first-action checks remain unchanged at every
knot.

Freeze numerical scales from existing evidence:

- `s_delta_v = v_max`, using frozen 3C per-joint velocity limits.
- `s_q = q_excursion`, using frozen 3C wide excursion limits.
- `s_v = v_max`.
- `a_brake`: per-joint 5th-percentile realized deceleration magnitude from frozen
  3C development traces, capped by the configured actuator acceleration limit.
- `s_brake = v_max^2`.
- `softplus_kappa = 0.05 s_brake`, with
  `softplus_kappa(x) = kappa log(1 + exp(x/kappa))`.

If a valid positive `a_brake` distribution is unavailable, do not enable the
braking residual.

The terminal reserve residual is:

\[
r_{reserve}
=
\begin{bmatrix}
(q_{c,N}-q_{c,ref})/s_q \\
u_{N-1}/s_v
\end{bmatrix}.
\]

Do not enable all terms initially. `H3-0` uses only the repeated frozen 3C running
cost, velocity-level control, and known manipulation preview. It has no velocity
smoothing, braking residual, terminal reserve, weak response shaping, or other
new term.

Add `r_delta_v` only if H3-0 exhibits command oscillation; choose its weight so
its median contribution is at most `10%` of median 3C running cost on development
data. Add braking viability only if H3 approaches stopping/excursion limits. Add
terminal reserve only if repeated horizon behavior consumes arm stroke. Each term
requires its own ablation and is removed if it produces no repeatable benefit.

For any enabled residual `r`, set its initial weight deterministically:

\[
w_r
=
\frac{0.1\operatorname{median}(J_{3C})}
{\operatorname{median}(\|r\|^2)+10^{-9}}.
\]

Compute medians on the frozen development set before active evaluation. Start
braking and terminal weights at zero until their evidence gates trigger. Change
one weight only through its own preregistered ablation.

H3 intentionally adds temporal shaping and is not required to equal one-step 3C.
Test H3 against the exact H1 parity model before considering H5.

Reject the extension if it loses rescues, exceeds p99 timing, or adds ordinary
regressions.

## Optional Weak Base-Response Shaping

Use only when measured evidence shows repeatable, physically correct directional
response that improves action choice.

The signal may predict only direction/sign, for example whether a candidate
counter reaction increases or decreases planar divergence. It need not predict
the complete base state.

Use it as a low-weight shaping residual or candidate validator, never as trusted
dynamics. Require:

- Repeatable sign across policies and mirrors.
- Better held-out command/outcome decisions than preview/response authority
  alone.
- No policy-label input.
- Bounded influence so invalid/uncertain shaping recovers the scheduler-only
  controller.

Do not revive the failed Iteration 4A full response model.

## Optional Feedforward/Feedback Separation

Separate authority channels only when prior experiments show different optimal
timing or scale.

Feedforward controls preventive authority before the disturbance peak. Feedback
controls post-peak continuation/recovery inside the frozen 3C lifecycle.

Change one channel at a time and retain separate diagnostics/ablations. Do not
introduce policy-specific schedules.

## Converging on One Candidate

Every retained mechanism becomes part of the same Iteration 4B controller.
Rejected mechanisms are removed rather than kept as runtime modes.

Before final freeze, compare:

1. Frame task.
2. B0.
3. Iteration 3C.
4. Current Iteration 4B candidate.

Compact-panel freeze requires:

- At least two repeated FAME fall-to-survival conversions.
- No new majority fall or other severe regression.
- No ordinary outcome worse than 3C.
- Ordinary-case improvement demonstrated by either:
    - At least one repeated FAME/ALMI majority-severity improvement relative to
      3C; or
    - A preregistered meaningful improvement in base drift, counter excursion,
      clipping, collision backtracking, or another frozen physical response
      metric, with unchanged outcome severity.
- Manipulation pass-through unchanged.
- Pre-publication total-controller p99 below `15 ms`.

The target is three FAME rescues and ordinary totals at B0 or better. ALMI
stumble improvement is a bonus.

## Final Evaluation

Only after compact freeze:

1. Run boundary, exploration, and hard groups with three matched repetitions per
   cell.
2. Report each group and pooled totals for Frame, B0, 3C, and 4B.
3. Run one Iteration-4B-only 100-target regression per policy.
4. Generate videos for every changed outcome, ALMI stumble, and fall.

Do not rerun all historical controllers on the 100-target panels.

## Implementation Structure

Add only modules justified by retained mechanisms:

```text
counter_balance/
    authority_scheduler.py
    counter_adaptive_velocity_controller.py
```

Optional horizon or weak-shaping code extends these modules; it does not fork a
new controller variant.

Recommended class:

```python
class CounterAdaptiveVelocityController(CounterDDPVelocityController):
    ...
```

Reuse the frozen 3C objective, solver, manipulation path, safety, collision, and
publication code.

## Stop Conditions

Stop adding complexity when:

- FAME rescues require high authority on ordinary stable cases.
- Preview/response signals do not generalize across mirrors and policies.
- A mechanism fails to produce repeatable behavioral improvement.
- Additional logic becomes target- or policy-specific.
- Real-time, pass-through, or safety contracts regress.

Freeze the simplest candidate that passes the compact gates. If no adaptive
candidate surpasses 3C and B0 jointly, retain them as separate baselines rather
than forcing a more complex Iteration 4B.
