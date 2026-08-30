# Counter-Balance Iteration 3 Analysis

## Status

Iteration 3 is complete as an experimental finite-horizon counter-arm controller.

The selected Iteration 3 baseline, `B0`, is the reference controller for later comparisons.
It is not promoted for hardware or general use.

The final Iteration 3 experiments establish three useful facts:

1. finite-horizon counter-arm control can improve difficult FAME outcomes without changing the manipulation-arm command;
2. the counter arm has enough mechanical authority to reduce disturbance metrics, including ALMI foot displacement;
3. further scalar tuning of feedforward authority, risk thresholds, excursion, and acceleration slew does not reliably convert the remaining FAME falls or ALMI stumbles.

The next controller-development stage is therefore **Iteration 3B**.

Iteration 3B keeps the same counter-arm-only predictive architecture but explores a better representation and timing of the arm-induced reaction.
Adding predicted base dynamics, contact mode, or lower-body control remains an Iteration 4 change.

---

## 1. Implemented Iteration 3 Architecture

Iteration 3 introduced a separate finite-horizon controller implemented around Crocoddyl `SolverBoxFDDP`.

The optimized counter-arm state is

\[
x_k =
\begin{bmatrix}
q_{c,k} \\
\dot q_{c,k}
\end{bmatrix}
\in \mathbb{R}^{8},
\]

with control

\[
u_k = \ddot q_{c,k} \in \mathbb{R}^{4}.
\]

Only the four proximal joints of the counter arm are optimized.
Counter-arm wrist joints remain at the captured reference.

The manipulation arm is an immutable exogenous trajectory.
Its position, velocity, and supplied torque command remain owned by the upstream manipulation controller and pass through the counter-balance layer.

The selected `B0` configuration is:

| Item | Value |
|---|---:|
| Control period | `0.02 s` |
| OCP horizon | 3 intervals, `0.06 s` |
| FDDP iterations | 1 |
| State | counter position and velocity, 8 values |
| Control | counter acceleration, 4 values |
| CoM weight | `1.0` |
| Angular-momentum weight | `2.0` |
| Posture weight | `0.02` |
| Acceleration weight | `0.01` |
| Velocity weight | `0.02` |
| Near-limit weight | `10.0` |
| Maximum acceleration | `25 rad/s^2` |
| Maximum acceleration change | `5 rad/s^2` per tick |
| Excursion envelope | `[0.35, 0.28, 0.20, 0.28] rad` |
| Moving-phase feedforward authority | `0.25` |
| Gyro entry / full authority | `0.10 / 0.15 rad/s` |
| Post-motion gyro feedback | enabled |

The running objective contains normalized:

- planar CoM-velocity cancellation;
- planar centroidal angular-momentum shaping;
- measured gyro feedback;
- posture regularization;
- acceleration regularization;
- velocity regularization;
- near-limit regularization.

Pinocchio CoM Jacobians and angular centroidal maps are frozen for one solve and rebuilt on the next control tick.

Box-FDDP directly enforces acceleration bounds.
Position, velocity, excursion, trust-region, collision, publisher, and timing checks remain separate acceptance gates.

The resulting controller is a **short predictive counter-arm overlay**.
It is not whole-body MPC and does not predict the lower-body policy or contact dynamics.

---

## 2. Iteration 3 Baseline Evidence

### 2.1 Paired FAME panel

Artifacts:

- `runs/hard_sweep/20260828_014310_iter3_candidate_vs_frame_fame`

Outcomes over 44 targets:

| Controller | Stable | Drift | Stumble | Fall | Survived |
|---|---:|---:|---:|---:|---:|
| Frame task | 17 | 18 | 1 | 8 | 36 |
| Selected Iteration 3 | 22 | 14 | 0 | 8 | 36 |

The selected controller produced five classification improvements and no regression:

- `left_fast_fall_search_04_scale_76`: drift to stable;
- `left_fast_fall_search_11_scale_78`: drift to stable;
- `left_inner_upward_overhang_pitch_plus`: drift to stable;
- `left_lateral_high_reach`: drift to stable;
- `left_lateral_overhead_reach`: stumble to stable.

The selected controller did **not** reduce the eight FAME falls in this panel.

This is important.
The controller is useful, but the remaining failure mode is not solved by the current H3 objective and `0.25` feedforward schedule.

### 2.2 Paired ALMI panel

Artifacts:

- `runs/hard_sweep/20260828_021447_iter3_candidate_vs_frame_almi`

Outcomes over 44 targets:

| Controller | Stable | Drift | Stumble | Fall | Survived |
|---|---:|---:|---:|---:|---:|
| Frame task | 25 | 12 | 4 | 3 | 41 |
| Selected Iteration 3 | 25 | 11 | 5 | 3 | 41 |

The single panel changed `left_fast_fall_search_09_scale_78` from drift to stumble.

Five fresh repetitions classified both controllers as stumble in all five trials:

| Controller | Five-repeat result |
|---|---|
| Frame task | 5 stumble |
| Selected Iteration 3 | 5 stumble |
| Full-feedforward capped recovery | 4 stumble, 1 drift |

The single-panel difference is therefore not a demonstrated majority regression.

More importantly, the counter controller reduced peak foot displacement in every repeated ALMI stumble trial but did not convert majority severity.

Historical ungated ALMI stumble cases used approximately:

- `0.2–0.48 rad` counter displacement;
- `1.0–1.4 rad/s` counter velocity;
- active counter solves through much of the relevant interval.

Across the five identified ALMI stumble cases, counter control reduced foot displacement in all five and reduced base drift in four.

This is strong evidence that **lack of raw counter-arm authority is not the leading limitation**.

---

## 3. Timing Evidence

The paired B0 panels measured approximately:

- FAME total-controller p99: `15.0 ms`;
- ALMI total-controller p99: `16.7 ms`.

This is behaviorally useful but the ALMI tail exceeds the desired `15 ms` promotion target.

The first Iteration 3B foundation moved activation before per-knot Pinocchio map construction.

Post-change smokes were:

- inactive / low-risk ALMI:
  `runs/key_findings/iteration3_tuning/20260828_031218_iter3_final_inactive_timing_smoke`,
  approximately `8.8 ms` p99;
- active FAME:
  `runs/key_findings/iteration3_tuning/20260828_031331_iter3_final_active_smoke`,
  approximately `11.5 ms` p99 with one timing hold.

This supports retaining a short active OCP and avoiding unnecessary map construction when inactive.

---

## 4. Failed Horizon-Only Explanation

Iteration 3 directly screened longer OCP horizons.

Five intervals:

- lost all tested right-boundary rescues;
- reached approximately `20.6 ms` total-controller p99.

Eight intervals:

- retained only one rescue;
- reached approximately `32.4 ms` p99;
- frequently exceeded the frozen-map trust region.

Therefore, simply extending the OCP from H3 to H5/H8 did not provide useful predictive behavior.

A longer horizon is not automatically a better controller when:

- the optimized physical objective is incomplete;
- frozen-map model error grows with horizon length;
- full validation becomes expensive;
- the real-time budget is already tight.

H3 remains the default OCP horizon until a later diagnostic identifies a specific need for longer counter-arm optimization.

---

## 5. FAME Early-Reaction Gap

Iteration 2 provided an important comparison.

On three historical right-boundary FAME fall cases, Iteration 2 generated approximately two to five times more opposing counter-arm pitch momentum around `0.5 s` than the selected Iteration 3 controller.

Counter shoulder displacement at that time was approximately:

- Iteration 2: `0.14–0.18 rad`;
- selected Iteration 3: `0.04–0.08 rad`.

The selected H3 controller with `0.25` moving-phase authority remains near its captured counter reference through much of the early moving-arm momentum peak.

Its later large counter velocity is therefore often **corrective rather than preventive**.

Full moving-phase authority recovered several FAME boundary cases in compact screens, but indiscriminate full authority also produced ALMI regressions.

The unresolved question is therefore not simply:

> How much counter-arm authority should be allowed?

A better question is:

> What arm-induced reaction should be produced, at what time, and when is that reaction actually needed?

---

## 6. Final Iteration 3 Momentum-Risk Experiment

The final Iteration 3 controller experiment tested whether predicted moving-arm momentum could schedule feedforward authority without changing the OCP objective.

The schedule used the maximum predicted planar moving-arm momentum over the supplied manipulation horizon.

The first assumed activation range, `2.2–2.5`, never activated because the observed online current-map risk on relevant FAME boundary falls was approximately `1.58–1.81`.

Zero-weight diagnostics were therefore used to calibrate a lower online range around `1.5–1.8`.

Calibrated compact-screen artifacts:

- FAME:
  `runs/key_findings/iteration3_tuning/20260828_145327_iter3_momentum_risk_calibrated_fame`;
- ALMI:
  `runs/key_findings/iteration3_tuning/20260828_152300_iter3_momentum_risk_calibrated_almi`.

The calibrated compact screen recovered `right_fast_fall_search_09` and `right_fast_fall_search_11`.
All tested ALMI stable guards remained stable.
`left_fast_fall_search_09` improved from stumble to drift in that compact screen.

This was useful mechanism evidence and justified a full paired comparison.

### 6.1 Full paired momentum-risk result

Artifacts:

- FAME:
  `runs/hard_sweep/20260828_155338_iter3_momentum_risk_vs_frame_fame`;
- ALMI:
  `runs/hard_sweep/20260828_163951_iter3_momentum_risk_vs_frame_almi`.

FAME:

| Controller | Stable | Drift | Stumble | Fall | Survived |
|---|---:|---:|---:|---:|---:|
| Frame task | 20 | 15 | 1 | 8 | 36 |
| Momentum-risk DDP | 23 | 14 | 0 | 7 | 37 |

The candidate produced five improvements and no regression.
It included one fall-to-drift rescue on `right_fast_fall_search_11_scale_78`.

ALMI:

| Controller | Stable | Drift | Stumble | Fall | Survived |
|---|---:|---:|---:|---:|---:|
| Frame task | 25 | 11 | 5 | 3 | 41 |
| Momentum-risk DDP | 25 | 10 | 6 | 3 | 41 |

The single changed case was `left_fast_fall_search_04_scale_76`, drift to stumble.

Five fresh repetitions classified both frame task and momentum-risk DDP as drift in all five trials.
The full-panel single-run difference is therefore not a majority regression.

### 6.2 Promotion result

The momentum-risk candidate was **not promoted**.

Reasons:

1. it produced only one full-panel FAME fall rescue rather than the preregistered two repeated fall-to-survival conversions;
2. additional aggressive threshold, medium/wide excursion, and relaxed acceleration-slew ablations produced no additional FAME fall rescue;
3. the full ALMI panel contained substantial pre-publication timing holds;
4. one active manual-grasp case reached approximately `27.2 ms` p99.

The sweep default therefore remains B0:

- H3;
- one FDDP iteration;
- `0.25` moving-phase feedforward;
- scalar gyro feedback during and after manipulation.

The momentum-risk candidate is retained as **mechanism evidence**, not as the new baseline.

---

## 7. Iteration 3 Closure

The momentum-risk experiment is the natural endpoint for Iteration 3 controller tuning.

It tested the last low-complexity hypothesis available without changing the physical objective:

> Can predicted manipulation-arm momentum choose when to expose more of the existing counter-arm objective?

The answer is partially positive:

- early feedforward action can recover a FAME fall;
- moving-arm prediction contains useful information;
- policy-independent risk scheduling can avoid obvious stable ALMI regressions.

However, the result is not sufficiently repeatable or real-time reliable to justify further threshold search.

Iteration 3 should therefore **not** continue with additional:

- scalar feedforward-authority searches;
- momentum-risk threshold tuning;
- wider excursion as the primary hypothesis;
- acceleration-slew relaxation as the primary hypothesis;
- direct H5/H8 re-screening with the existing objective.

Additional Iteration 3 runs are justified only as **closure diagnostics**.
They may replay or repeat frozen controllers to characterize successful versus failed reactions.
They should not produce another tuned Iteration 3 controller.

---

## 8. Literature-Informed Interpretation

The following literature does not replace the measured Iteration 3 evidence.
It provides useful physical interpretations and design hypotheses for Iteration 3B.

### 8.1 Zhang et al. 2014: momentum change rate

Da-song Zhang, Rong Xiong, Jun Wu, and Jian Chu,
“Balance Maintenance in High-Speed Motion of Humanoid Robot Arm-Based on the 6D Constraints of Momentum Change Rate,”
*The Scientific World Journal*, 2014.
DOI: `10.1155/2014/535294`.

This work is the closest manipulation-specific antecedent to the current problem.
It considers a high-speed operating arm and an auxiliary arm and formulates balance maintenance using constraints on the time derivative of two-arm momentum.

The important implication for the current controller is that arm **momentum change rate**, rather than momentum alone, may better represent the transient reaction generated by arm acceleration.

The transfer is not exact.
Our arm momentum contribution is expressed relative to the whole-body centroidal model, and its derivative is not an isolated base torque.
Ground/contact wrench, gravity, reference-point motion, lower-body torque, and body motion also contribute.

Therefore:

\[
-\dot H_{\mathrm{arm}}
\]

must be treated as an **empirical arm-reaction proxy**, not as a conservation identity.

### 8.2 Lee, Jeon, and Kim 2025: CAM as shared balance information

Ho Jae Lee, Se Hwan Jeon, and Sangbae Kim,
“Learning Humanoid Arm Motion via Centroidal Momentum Regularized Multi-Agent Reinforcement Learning,”
2025, arXiv:`2507.04140`.

Their arm and leg agents share base-state and centroidal-angular-momentum information.
The relevant implication is not to copy the RL architecture.
It is that whole-body CAM can be informative for deciding whether arm intervention is useful.

For Iteration 3B, CAM is therefore a candidate **measured response / activation signal**, not a predicted OCP state.

### 8.3 Raza, Zhu, and Hayashibe 2021: arm acceleration and CMP

Fahad Raza, Wei Zhu, and Mitsuhiro Hayashibe,
“Balance Stability Augmentation for Wheel-Legged Biped Robot Through Arm Acceleration Control,”
*IEEE Access*, 2021.
DOI: `10.1109/ACCESS.2021.3071055`.

The morphology differs from H1-2, but the conceptual connection is direct:

- active arm acceleration is the balance input;
- balance quality should be evaluated through whole-body/contact response, not only arm motion;
- CMP can expose the effect of centroidal angular-momentum rate on support behavior.

For Iteration 3B, CMP is best used initially as a simulation diagnostic or response signal.
It should not become another weighted OCP term unless the measured data show that it predicts useful intervention.

### 8.4 Shen, Chemori, and Hayashibe 2021: predictive body response

Keli Shen, Ahmed Chemori, and Mitsuhiro Hayashibe,
“Reproducing Human Arm Strategy and Its Contribution to Balance Recovery Through Model Predictive Control,”
*Frontiers in Neurorobotics*, 2021.
DOI: `10.3389/fnbot.2021.679570`.

Their work predicts simplified body recovery together with arm/ankle/hip strategy.

This provides a useful boundary:

- Iteration 3B may improve arm-side reaction representation and timing;
- if that remains insufficient, the next missing information is likely explicit prediction of body response.

That change belongs to Iteration 4.

### 8.5 Zhang et al. 2025: heavy-limb coupling

Tianlin Zhang et al.,
“Whole-Body Control Framework for Humanoid Robots with Heavy Limbs: A Model-Based Approach,”
2025, arXiv:`2506.14278`.

This work reinforces that heavy-limb motion can materially alter whole-body mass/inertia behavior and motivates explicitly coupled body/contact prediction.

It supports the Iteration 4 direction rather than expanding Iteration 3B into whole-body MPC.

---

## 9. Central Interpretation

The complete Iteration 3 evidence supports the following interpretation.

### 9.1 The counter arm has useful authority

It already:

- improves several FAME classifications;
- reduces ALMI foot displacement;
- reduces base drift in most ALMI stumble examples;
- can recover at least one FAME fall when early authority is exposed.

Therefore a stronger arm is not the missing component.

### 9.2 Scalar authority tuning has saturated

Increasing or scheduling the same objective can change outcomes, but it does not produce robust cross-policy gains.

This suggests that the controller needs a better representation of **what reaction to generate**, not another global authority multiplier.

### 9.3 Reaction timing is a leading hypothesis

The existing objective shapes arm momentum \(H\).
The control variable is arm acceleration \(\ddot q_c\).

The reaction that acceleration produces is more directly related to momentum change rate:

\[
\dot H.
\]

The Iteration 2 versus Iteration 3 comparison also shows that early opposing arm momentum matters, while large later motion can be merely corrective.

### 9.4 Prediction horizon and disturbance preview should be separated

The manipulation trajectory is known exogenously farther into the future than the short H3 counter-arm optimization horizon.

Therefore Iteration 3B should distinguish:

- **disturbance preview horizon**: how far ahead the controller inspects manipulation-arm motion;
- **OCP horizon**: how far ahead it optimizes counter-arm state and acceleration.

A long disturbance preview does not require an H5/H8 counter-arm OCP.

This is a high-value design direction because it may provide earlier reaction without repeating the computation and frozen-map failures of longer OCP horizons.

### 9.5 Instantaneous momentum rate may not be the only useful objective

A finite difference over one `20 ms` interval can amplify measurement and frozen-map error.

A short-window angular-momentum change,

\[
\Delta H = H(t+T)-H(t),
\]

is equivalent to integrated momentum rate over that window and may provide a more robust short-horizon reaction target.

Iteration 3B should treat instantaneous \(\dot H\) and short-window \(\Delta H\) as **alternative reaction representations**, not automatically add both.

### 9.6 The lower-body policy already has useful disturbance tolerance

ALMI often remains stable without arm intervention and may use stepping as a protective strategy.

Therefore the objective should not necessarily force perfect cancellation of every manipulation disturbance.

An empirically identified safe reaction envelope or deadband may be preferable to zero-error regulation.

---

## 10. Design Implications for Iteration 3B

Iteration 3B should be sufficiently ambitious to test whether the remaining performance is recoverable inside the counter-arm-only architecture.

The most important design directions are:

1. **Arm-reaction identification**
   - identify command-to-measured arm acceleration and momentum-rate gain/delay;
   - identify sign and axis mapping between arm reaction and base angular response.

2. **Manipulation feedforward + measured feedback**
   - separate predicted manipulation-arm disturbance from measured base-response correction;
   - allow a calibrated feedforward gain rather than assuming full cancellation.

3. **Long disturbance preview with short H3 OCP**
   - inspect manipulation-arm disturbance farther into the future;
   - retain the short real-time counter-arm optimization horizon unless longer optimization is later justified.

4. **Reaction representation**
   - test instantaneous momentum rate \(\dot H\);
   - retain short-window momentum change \(\Delta H\) as an alternative if instantaneous rate is noisy or model-sensitive.

5. **Safe reaction envelope**
   - consider hinge/deadband regulation if continuous cancellation causes unnecessary ALMI intervention.

6. **Physics-informed warm start**
   - use the identified reaction map to seed the first active counter acceleration when there is no useful previous active solution;
   - retain shifted warm starts during continuous active control.

7. **Finite counter-arm reaction capacity**
   - account for remaining excursion, current velocity, braking capability, and current momentum effectiveness;
   - avoid spending counter-arm authority in a way that creates unavoidable braking payback.

8. **Measured intervention signal**
   - evaluate gyro, settled-reference tilt, whole-body CAM, and valid CMP/ZMP/contact diagnostics;
   - keep activation policy-independent.

9. **Recovery and handoff**
   - remain available after manipulation completion while measured response is unsafe;
   - stop adding counter acceleration when a protective contact transition/step begins;
   - use bounded braking and handoff.

10. **Optional longer OCP horizon**
    - revisit H5/H8 only if a validated H3 reaction objective works and remaining failures specifically indicate insufficient braking/reaction look-ahead.

---

## 11. Expected Iteration 3B Success

Iteration 3B does not need to eliminate every hard failure to justify the design.

A strong outcome would be:

- repeated FAME fall-to-survival conversions beyond B0;
- at least one repeated ALMI stumble-to-drift/stable conversion;
- preservation of existing FAME classification improvements;
- no stable-to-worse majority regressions;
- moving-arm pass-through unchanged;
- active controller p99 at or below `15 ms` and pre-publication maximum below `20 ms`.

Such a result would demonstrate that better arm-side disturbance representation and reaction timing materially improve balance without predicting the lower body.

If the best validated Iteration 3B combination cannot achieve repeated improvements without regression, the evidence would support moving to an explicit base-response prediction model.

---

## 12. Iteration 4 Boundary

Iteration 4 begins if the optimized prediction model adds any of:

- base roll or pitch as dynamic state;
- base angular velocity as predicted dynamic state;
- an identified arm-command-to-base-response transition model;
- support/contact mode inside the OCP state or dynamics;
- predicted contact wrench/force as an optimization variable;
- lower-body or whole-body controls.

Arm-side actuator calibration, reaction objectives, disturbance preview, warm starts, viability/capacity shaping, measured CAM/CMP activation, recovery, and OCP horizon shaping remain within Iteration 3B.

---

## 13. Final Decision

Freeze Iteration 3 B0 as the comparison baseline.

Retain the momentum-risk controller only as evidence that anticipatory manipulation-arm information can expose useful early counter action.

Do not continue scalar authority tuning.

Proceed to Iteration 3B with the central question:

> Can a short real-time counter-arm OCP produce better balance by predicting the manipulation disturbance earlier and shaping the timing and magnitude of arm reaction more directly?

If the answer is yes, Iteration 3B may provide the desired strong controller result without the complexity of whole-body MPC.

If the answer is no after controlled arm-side exploration, the next justified change is Iteration 4 base-response prediction.

---

## 14. Iteration 3B Progress

### 14.1 Experiment A0: Arm-Reaction Identification

Iteration 3B began with zero-weight diagnostics only. No B0 objective,
activation, horizon, limit, or solver parameter changed.

The frozen first-step arm model predicts measured counter momentum-rate with a
consistent one-tick delay and positive per-axis gain. A development fit produced
effectiveness `[0.105, 0.115]`; held-out correlation was `[0.674, 0.625]` and
held-out sign consistency was approximately `[75.6%, 77.5%]`.

This validates a useful arm-side realization signal but not a direct weighted
reaction cost. Prediction error remains substantial, command-to-acceleration
delay is one to two ticks, and measured counter momentum-rate has only weak
correlation with base angular acceleration.

Two source-state-timestamp confirmation runs retained the one-tick delay and
positive effectiveness: corrected FAME gain was `[0.078, 0.104]`, and corrected
ALMI gain was `[0.122, 0.128]`. The identification conclusion is unchanged, but
one global gain remains approximate.

The one-step delayed model is retained for diagnostics and possible future
reaction-informed initialization. It is not promoted into the OCP objective.

A receding `60 ms` short-window momentum-change construction was also tested and
rejected. It reduced held-out correlation to `[0.350, 0.122]`, with second-axis
`R^2` below zero. Iteration 3B will not add both instantaneous and short-window
reaction costs without stronger evidence.

### 14.2 Next Decision

The next experiment is zero-weight exogenous manipulation disturbance preview.
It keeps H3 and B0 frozen while asking whether a `0.20 s` preview identifies the
upcoming moving-arm disturbance earlier and with more reliable sign than the
current H3 view. Preview remains diagnostic until validated against realized
moving-arm momentum.

### 14.3 Experiment A1: Manipulation Disturbance Preview

A separate `0.20 s` exogenous preview was added while H3 control remained
unchanged. The frozen current-map preview predicted future moving-arm momentum
with planar correlation `0.73–0.96`, including pitch correlation above `0.95` in
all four development/guard cases.

The preview exposed disturbance approximately `80–100 ms` before H3 and showed
about three times the H3-visible momentum change. FAME and ALMI versions of the
same motion remained nearly identical, so preview is useful for feedforward
timing but not for deciding which lower-body policy needs intervention.

Successful Iteration 2 rescue traces were then inspected to identify a candidate
reaction target. Opposite moving-to-counter pitch momentum-rate correlation was
`0.81–0.88`, with realized effectiveness `0.32–0.37` and approximately `30 ms`
lag. Failed B0 cases produced only `0.05–0.08` pitch effectiveness.

The next experiment remains zero-weight: compute a bounded reaction-informed
first-action seed and measure its feasibility/saturation before using it for
FDDP initialization or adding a reaction cost.

### 14.4 Experiments A2 and A3

The preview-informed reaction seed was infeasible or saturated in `88–92%` of
active samples. Median reaction error remained around `0.8`. It was rejected
without changing FDDP initialization.

A first-order actuator model fit shoulder acceleration moderately well on
held-out data (`R^2` around `0.59–0.63`) but failed on the elbow (`R^2 0.15`). A
12-state arm augmentation was therefore rejected.

Measured-response diagnostics showed clear full post-motion divergence
separation:

- FAME difficult response: `0.0273` peak positive divergence.
- ALMI stumble response: `0.0554`.
- ALMI stable guard: `0.0036`.

However, existing gyro recovery was already active during difficult response and
mostly passive on the stable guard. The missing quantity is not another response
threshold. It is a trustworthy prediction of what counter reaction will improve
the whole-body response.

### 14.5 Iteration 3B Conclusion

Iteration 3B retains useful diagnostic infrastructure:

- one-tick delayed arm momentum-rate observation;
- calibrated arm-side effectiveness estimates;
- separate `0.20 s` manipulation preview with H3 control;
- response divergence diagnostics;
- zero-weight reaction-seed feasibility analysis.

It does not add a new weighted objective or alter B0 behavior. The arm-only
frozen model cannot yet map accurate manipulation preview into a feasible,
reliable whole-body-improving counter reaction. Continuing with arm-side weights
or thresholds would violate the evidence-driven stop rule.

The next justified controller architecture is Iteration 4: a minimal identified
base-response model that predicts planar base response to manipulation
disturbance and counter-arm action, with contact remaining an observed/safety
mode rather than a lower-body control variable initially.
