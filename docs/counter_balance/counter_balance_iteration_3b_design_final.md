# Counter-Balance Iteration 3B Design

## Status

Iteration 3B is the next design stage for the finite-horizon counter-arm controller.

Iteration 3 is frozen.
Its selected baseline `B0` remains the reference controller.

Iteration 3B keeps the same counter-arm-only architecture but expands the design space around:

- arm reaction identification;
- disturbance preview;
- momentum-rate or angular-impulse shaping;
- response-aware intervention;
- recovery;
- finite reaction capacity;
- real-time solver initialization.

The goal is to extract as much performance as possible from the counter arm **before** introducing explicit base/contact prediction.

Iteration 3B is not whole-body MPC.

---

## 1. Goal

The primary goal is to improve difficult standing-manipulation outcomes while preserving the clean ownership and no-regression behavior established in Iteration 3.

Desired behavioral gains are:

- repeated FAME fall-to-drift/stable conversions;
- repeated ALMI stumble-to-drift/stable conversions;
- preservation of existing FAME drift/stumble improvements;
- no new stable-to-worse majority regressions.

The manipulation arm remains immutable.
Only the four active proximal joints of the counter arm may be optimized.

A strong Iteration 3B result would demonstrate that better **arm-side reaction timing and disturbance representation** can improve whole-body balance without predicting the lower-body controller.

---

## 2. Frozen Baseline

The Iteration 3 baseline `B0` is:

| Item | Value |
|---|---:|
| Control period | `0.02 s` |
| OCP horizon | 3 intervals, `0.06 s` |
| FDDP iterations | 1 |
| State | counter position and velocity, 8 values |
| Control | counter acceleration, 4 values |
| Moving-phase feedforward authority | `0.25` |
| Gyro entry / full authority | `0.10 / 0.15 rad/s` |
| Post-motion gyro feedback | enabled |

The running objective contains:

- normalized planar CoM-velocity residual;
- normalized planar angular-momentum residual;
- posture regularization;
- acceleration regularization;
- velocity regularization;
- near-limit regularization.

B0 does not directly optimize arm momentum rate or predict base response.

Paired B0 artifacts:

- FAME:
  `runs/challenge_sweep/20260828_014310_iter3_candidate_vs_frame_fame`;
- ALMI:
  `runs/challenge_sweep/20260828_021447_iter3_candidate_vs_frame_almi`.

B0 FAME:

| Controller | Stable | Drift | Stumble | Fall | Survived |
|---|---:|---:|---:|---:|---:|
| Frame task | 17 | 19 | 0 | 8 | 36 |
| B0 | 22 | 14 | 0 | 8 | 36 |

B0 ALMI:

| Controller | Stable | Drift | Stumble | Fall | Survived |
|---|---:|---:|---:|---:|---:|
| Frame task | 25 | 12 | 4 | 3 | 41 |
| B0 | 25 | 11 | 5 | 3 | 41 |

The apparent ALMI drift-to-stumble change on `left_fast_fall_search_09_scale_78` did not persist under five fresh repetitions: both frame task and B0 were stumble in all five.

The momentum-risk Iteration 3 experiment is retained as mechanism evidence, not as the baseline.

Its full FAME panel produced one fall-to-drift rescue and five total improvements.
It failed the repeated-rescue and real-time promotion gates.
Additional authority-threshold, excursion, and slew tuning did not produce additional robust FAME fall recovery.

---

## 3. Fixed Architecture

Iteration 3B retains:

- immutable manipulation-arm position, velocity, and torque commands;
- four proximal counter-arm optimization joints;
- held counter-arm wrist joints;
- counter position and velocity as the core arm state;
- counter acceleration as the control;
- Crocoddyl `SolverBoxFDDP`;
- frozen Pinocchio maps within one solve;
- exact manipulation trajectory as exogenous forecast data when available;
- hard position, velocity, excursion, collision, publisher, and timing gates;
- policy-independent behavior;
- no stale-plan publication.

The moving arm remains owned by the upstream manipulation controller.

Counter-side model or solver failures may change only the counter-arm command.
They must not modify or block a valid manipulation-arm sample except under global estop behavior.

---

## 4. Iteration 3B Boundary

Iteration 3B may add:

- measured counter-actuator gain/delay;
- a first-order counter-actuator realization model if needed;
- arm momentum-rate prediction;
- short-window angular-momentum-change prediction;
- manipulation disturbance preview beyond the OCP horizon;
- new arm-side residuals;
- response-aware cost gating;
- CAM/CMP/ZMP/contact diagnostics;
- event-based recovery and handoff;
- physics-informed warm starts;
- reaction-capacity or braking-viability shaping;
- a longer counter-arm OCP horizon if later justified.

Iteration 3B must not add:

- predicted base orientation dynamics to the optimized state;
- predicted base angular-velocity dynamics;
- an identified arm-command-to-base-response transition model inside the OCP;
- contact/support mode as OCP state;
- contact force/wrench optimization;
- lower-body controls;
- whole-body optimization;
- FAME/ALMI-specific branches;
- a learned lower-policy classifier.

Those changes define Iteration 4.

---

# 5. Design Principles

## 5.1 Separate disturbance from response

The manipulation trajectory is known exogenously.
Its predicted disturbance should be treated separately from measured robot response.

The controller should distinguish:

- **feedforward**: anticipated disturbance from the manipulation arm;
- **feedback**: measured tilt/rate/CAM response indicating that the disturbance is not being handled sufficiently.

This avoids treating every manipulation motion as requiring the same counter reaction.

## 5.2 Separate disturbance-preview horizon from OCP horizon

The controller can inspect manipulation motion farther into the future than it optimizes counter-arm dynamics.

Define:

- \(T_p\): manipulation disturbance-preview horizon;
- \(T_{\mathrm{ocp}}\): counter-arm optimization horizon.

Initially:

\[
T_{\mathrm{ocp}} = 0.06\ \mathrm{s}
\]

remains fixed at H3.

\(T_p\) may be substantially longer because manipulation-arm prediction is exogenous and cheap relative to solving additional counter-arm knots.

This distinction is central to Iteration 3B.

## 5.3 Do not assume perfect reaction cancellation

The lower-body controller already tolerates or actively handles many disturbances.

Therefore the desired counter reaction should not automatically be:

\[
\dot H_c = -\dot H_m.
\]

A calibrated gain, response feedback, and possibly a safe reaction envelope should determine how much counter-arm action is useful.

## 5.4 Preserve finite-stroke viability

The counter arm is a finite reaction resource.

Useful early motion must not create unavoidable later:

- excursion saturation;
- high counter velocity;
- braking payback;
- unsafe posture return.

Reaction performance and remaining braking/excursion capacity must be considered together.

## 5.5 Change one physical hypothesis at a time

The design contains multiple possible improvements.
They are exploration axes, not one mandatory combined controller.

A new mechanism should earn inclusion through a controlled comparison before being combined with another mechanism.

---

# 6. Design Choice A: Arm-Reaction Identification

## Motivation

Iteration 3 uses commanded counter acceleration as if it is realized immediately.

It also shapes arm momentum \(H\), while the physical reaction produced by arm acceleration is more closely associated with momentum change.

Before weighting a new reaction objective, identify the actual arm-side control channel.

## Quantities

For planar roll/pitch-relevant axes, log:

- manipulation-arm angular momentum \(H_m\);
- counter-arm angular momentum \(H_c\);
- total arm angular momentum;
- predicted \(\dot H_m\);
- predicted \(\dot H_c\);
- predicted total arm \(\dot H\);
- measured arm momentum and momentum rate;
- commanded counter acceleration;
- measured counter position and velocity;
- filtered measured counter acceleration;
- settled-reference base tilt;
- base angular velocity;
- base angular acceleration;
- whole-body CAM;
- ZMP and support margin when valid;
- CMP and CMP margin when consistently defined;
- contact state/confidence in simulation;
- manipulation start/completion;
- first foot-lift time;
- foot displacement and lift;
- monotonic timestamps for command and state data.

## Questions

The identification should determine:

1. whether predicted arm momentum rate matches measured momentum rate;
2. command-to-arm-response gain;
3. command-to-arm-response delay;
4. axis sign consistency;
5. whether a useful relation exists between counter-arm reaction and base angular acceleration;
6. whether the relation is reasonably consistent across FAME, ALMI, and stable guards.

The goal is not to identify full base dynamics.

## Actuator realization

If measured delay is small compared with the `20 ms` control interval and gain error is modest, retain the current command-space state transition.

A static calibrated effectiveness factor may be used in the reaction model.

If delay or gain materially changes the reaction timing, Iteration 3B may augment the arm state with a first-order realized-acceleration state:

\[
a_{r,k+1}
=
a_{r,k}
+
\frac{\Delta t}{\tau_a}
\left(
G_a u_k-a_{r,k}
\right).
\]

Then:

\[
\dot q_{c,k+1}
=
\dot q_{c,k}
+
\Delta t\,a_{r,k+1},
\]

\[
q_{c,k+1}
=
q_{c,k}
+
\Delta t\,\dot q_{c,k}
+
\frac{1}{2}\Delta t^2 a_{r,k+1}.
\]

This remains an arm-actuator calibration model.
It must not predict base/contact response.

## Main risk

A noisy or policy-dependent arm-to-base relation may make momentum-rate feedback misleading.

## Evidence to continue

Proceed with a weighted reaction objective only if sign, scale, and delay are sufficiently repeatable to define a stable normalized residual.

---

# 7. Design Choice B: Manipulation Feedforward + Balance Feedback

## Motivation

The manipulation arm is a known disturbance source.

The counter arm should be able to react before large base motion develops, while measured feedback should correct for lower-body and contact effects that cannot be predicted by the fixed-base arm model.

## Candidate decomposition

Write planar arm momentum rate as

\[
\dot H_{\mathrm{arm}}
=
\dot H_m+\dot H_c.
\]

A candidate counter-arm reaction target is

\[
\dot H_c^*
=
-\lambda_{\mathrm{ff}}\dot H_m
+
K_\theta e_\theta
+
K_\omega\omega_{xy},
\]

where:

- \(\dot H_m\) is predicted manipulation-arm momentum rate;
- \(\lambda_{\mathrm{ff}}\) is a calibrated feedforward effectiveness factor;
- \(e_\theta = \theta-\theta^0\) is settled-reference orientation error;
- \(\omega_{xy}\) is measured planar angular velocity.

Do not assume \(\lambda_{\mathrm{ff}}=1\).

The physical meaning is:

- predicted manipulation disturbance produces anticipatory reaction;
- measured base divergence adds corrective reaction.

This is not a claim that \(-\dot H_{\mathrm{arm}}\) equals base torque.

## Diagnostic separation

Even if the final implementation uses one combined residual, log feedforward and feedback contributions separately:

\[
r_{\mathrm{ff}}
\sim
\dot H_c+\lambda_{\mathrm{ff}}\dot H_m,
\]

\[
r_{\mathrm{fb}}
\sim
\dot H_c-
\dot H_{c,\mathrm{feedback}}^*.
\]

This makes it possible to identify whether an improvement came from anticipation or measured-response correction.

## Main risk

Strong feedforward can recreate the ALMI over-intervention behavior seen under indiscriminate full authority.

## Evidence to support

The mechanism is useful if it increases repeated FAME rescue or ALMI stumble improvement without stable-guard regression.

---

# 8. Design Choice C: Long Disturbance Preview With Short OCP

## Motivation

The manipulation trajectory can be known much farther into the future than H3.

The previous H5/H8 screen coupled more disturbance look-ahead with:

- more optimization variables;
- more Pinocchio maps;
- more frozen-map error;
- more validation work;
- worse real-time behavior.

Iteration 3B should decouple these effects.

## Design

Retain:

\[
T_{\mathrm{ocp}} = 0.06\ \mathrm{s}
\]

initially.

Introduce a separate disturbance preview:

\[
T_p > T_{\mathrm{ocp}}.
\]

Candidate preview ranges may include approximately `0.1–0.4 s`, constrained by the validity of the supplied manipulation trajectory.

No counter state is optimized beyond the H3 OCP horizon.

## Preview features

Possible low-dimensional preview quantities include:

### Peak predicted manipulation momentum rate

\[
\rho_{\dot H_m}
=
\max_{\tau\in[0,T_p]}
w(\tau)
\left\|
\frac{\dot H_m(t+\tau)}{s_{\dot H}}
\right\|.
\]

### Predicted momentum change

\[
\Delta H_m(\tau)
=
H_m(t+\tau)-H_m(t).
\]

### Discounted disturbance exposure

\[
\rho_{\mathrm{int}}
=
\int_0^{T_p}
w(\tau)
\left\|
\dot H_m(t+\tau)
\right\|
d\tau.
\]

### Time to predicted disturbance peak

\[
t_{\mathrm{peak}}
=
\arg\max_{\tau\in[0,T_p]}
\|\dot H_m(t+\tau)\|.
\]

The initial implementation should prefer one or two simple features, not a large feature vector.

## Uses

Disturbance preview may influence:

- activation timing;
- feedforward reaction target;
- reaction-seed initialization;
- preparation for a predicted sign change;
- braking anticipation.

It should not directly create a new target-specific branch.

## Main risk

Long preview based on a frozen current configuration can exaggerate or mis-sign future momentum.

Preview values must therefore be validated against the actual exogenous manipulation trajectory and current/nominal arm configuration assumptions.

## Evidence to support

The design is useful if it triggers helpful reaction earlier than the current gyro gate while preserving H3 real-time performance and stable guards.

---

# 9. Design Choice D: Instantaneous Momentum Rate vs Short-Window Angular Impulse

## Motivation

A one-step momentum-rate estimate,

\[
\dot H_k
\approx
\frac{H_{k+1}-H_k}{\Delta t},
\]

divides map and measurement error by `0.02 s`.

This can amplify noise and frozen-map mismatch.

The useful physical quantity may instead be the net angular-momentum change over a short response interval.

## Candidate 1: instantaneous momentum-rate residual

\[
r_{\dot H,k}
=
\frac{
\dot H_{c,k}
-
\dot H_{c,k}^*
}
{s_{\dot H}}.
\]

This gives direct reaction timing when the estimate is sufficiently clean.

## Candidate 2: short-window angular-impulse residual

For a short interval spanning \(r\) knots:

\[
\Delta H_{c,k:r}
=
H_{c,k+r}-H_{c,k},
\]

\[
\Delta H_{m,k:r}
=
H_{m,k+r}-H_{m,k}.
\]

A candidate target is

\[
\Delta H_c^*
=
-\lambda_{\mathrm{ff}}\Delta H_m
+
\Delta H_{\mathrm{feedback}}^*.
\]

The residual is

\[
r_{\Delta H}
=
\frac{
\Delta H_c-\Delta H_c^*
}
{s_{\Delta H}}.
\]

For H3, the natural window may span part or all of the `60 ms` horizon.

## Interpretation

\(\Delta H\) is the integral of momentum rate over the window.

It allows the optimizer some freedom in exactly when the useful acceleration is generated while still controlling the net arm reaction.

## Exploration rule

Treat \(\dot H\) and \(\Delta H\) as alternative reaction representations initially.

Do not add both weighted costs simply because both are available.

## Main risk

A long impulse window can hide a poorly timed reaction that cancels later in the interval.

## Evidence to choose

Prefer \(\dot H\) when rate prediction is clean and the timing of reaction matters strongly.

Prefer \(\Delta H\) when instantaneous rate is noisy but short-window momentum change correlates consistently with successful response.

---

# 10. Design Choice E: Safe Reaction Envelope

## Motivation

ALMI demonstrates that the lower-body controller can tolerate many manipulation disturbances without counter-arm intervention.

Perfect reaction cancellation may therefore be unnecessarily aggressive.

## Candidate formulation

Instead of penalizing all reaction error quadratically, define an empirically safe interval:

\[
r_{\min}
\le
r_{\mathrm{reaction}}
\le
r_{\max}.
\]

Use a smooth hinge outside the interval:

\[
\ell_{\mathrm{env}}
=
\phi(r_{\mathrm{reaction}}-r_{\max})
+
\phi(r_{\min}-r_{\mathrm{reaction}}).
\]

Inside the safe region:

\[
\ell_{\mathrm{env}}\approx 0.
\]

The envelope may be centered around a nonzero desired reaction if diagnostics support it.

## Interpretation

The design objective becomes:

> keep arm-induced reaction within what the combined arm/lower-body system can safely tolerate,

rather than:

> force all manipulation reaction to zero.

## Data rule

The safe envelope must be identified from development/stable data before held-out evaluation.

It must not be tuned directly on held-out severity labels.

## Main risk

An overly wide envelope reacts too late.
An overly narrow envelope recreates full-time cancellation.

## Evidence to support

Use the envelope only if a basic reaction residual improves difficult cases but produces unnecessary stable/ALMI intervention.

---

# 11. Design Choice F: Physics-Informed FDDP Warm Start

## Motivation

Iteration 3 uses only one FDDP iteration.

Shifted warm starts work well during continuous active control, but the first important `MONITOR -> ACTIVE` tick may have no useful previous active control sequence.

The first reaction tick is precisely where FAME early-reaction evidence suggests timing matters.

## Candidate reaction seed

Under the frozen local model, approximate the relation between counter acceleration and counter-arm momentum rate as

\[
\dot H_c
\approx
B_{\dot H}u + d.
\]

Generate an initial acceleration seed:

\[
u_{\mathrm{seed}}
=
B_{\dot H}^{\dagger}
\left(
\dot H_c^*-d
\right).
\]

Then:

1. clip to acceleration bounds;
2. apply acceleration-slew bounds;
3. enforce first-step position/velocity/excursion viability;
4. roll out the remaining H3 seed;
5. pass the feasible seed to FDDP.

The seed is only an initialization aid.

It must never bypass the solver or validation gates.

## Main risk

A poorly identified reaction Jacobian can create a worse first guess than zero/shifted acceleration.

## Evidence to support

Compare ordinary and reaction-informed initialization with the same objective and limits.

Useful evidence includes:

- earlier correct-sign first action;
- improved solver cost after one iteration;
- reduced rejection rate;
- improved physical outcome;
- no timing regression.

---

# 12. Design Choice G: Finite Reaction Capacity

## Motivation

The counter arm is a finite-stroke actuator.

A useful early counter reaction can still be harmful if it leaves insufficient room to brake or respond to a later disturbance.

## Capacity signals

For each active counter joint, maintain:

- positive and negative remaining excursion;
- current velocity;
- conservative identified braking acceleration;
- current acceleration/slew limits.

A basic braking condition is:

\[
(v_i^+)^2
\le
2a_{\mathrm{brake},i}d_i^+,
\]

\[
(v_i^-)^2
\le
2a_{\mathrm{brake},i}d_i^-.
\]

The effective bounds use the intersection of:

- URDF limits;
- publisher limits;
- configured limits;
- captured excursion limits.

## Reaction-capacity metric

Define a normalized capacity indicator

\[
\eta_{\mathrm{cap}}\in[0,1]
\]

from braking margin, remaining excursion, and optionally the local arm-momentum effectiveness.

The first version should remain simple and interpretable.

Possible uses include:

- reducing feedforward reaction when braking margin is nearly exhausted;
- biasing toward a direction with more usable excursion;
- increasing posture/viability pressure before saturation;
- deciding when a long-preview disturbance should trigger early preparation.

Do not use capacity merely to weaken all reaction uniformly.

## Main risk

Over-conservative capacity scaling can remove the early authority needed for FAME rescue.

## Evidence to support

Capacity shaping is useful only if it preserves early rescue while reducing late excursion/braking payback or rejection.

---

# 13. Design Choice H: Response-Aware Intervention

## Motivation

The Iteration 3 scalar gyro gate is useful but often late.

The intervention signal should combine predicted manipulation risk with measured evidence that the robot is diverging.

## Candidate measured signals

Evaluate:

- settled-reference roll/pitch error;
- planar angular velocity;
- whether orientation error and angular velocity indicate divergence or return;
- whole-body planar CAM;
- CMP/ZMP/support margin when reliable.

CAM and CMP are initially measured diagnostics/signals.
They are not predicted OCP states.

## Candidate divergence logic

For axis \(i\),

\[
D_i
=
(\theta_i-\theta_i^0)\omega_i.
\]

When

\[
D_i>0,
\]

the axis is moving away from the settled reference.

A response score may combine normalized tilt, angular rate, and CAM:

\[
\rho_{\mathrm{resp},i}
=
\max
\left(
\frac{|e_{\theta,i}|}{s_\theta},
\frac{|\omega_i|}{s_\omega},
w_H\frac{|H_{\mathrm{CAM},i}-H_{\mathrm{CAM},i}^0|}{s_H}
\right)
\]

or another low-dimensional identified formulation.

The exact combination should follow diagnostic separation rather than be assumed in advance.

## Combined intervention concept

Use both:

- predicted feedforward disturbance risk;
- measured response risk.

Possible controller behavior:

- anticipate when predicted disturbance is large;
- increase corrective authority when measured response diverges;
- attenuate reaction while the base is returning;
- remain passive when both predicted disturbance and measured response are small.

## Main risk

CAM can include useful lower-body recovery motion and should not automatically be minimized.

## Evidence to support

A new intervention metric should enter earlier on successful FAME rescues while avoiding unnecessary stable/ALMI activation.

---

# 14. Design Choice I: Recovery, Step Safety, and Handoff

## Motivation

Observed first foot lift may occur:

- near the end of the `1.5 s` manipulation motion;
- or approximately `0.7–1.0 s` after manipulation has stopped.

Moving-arm speed alone is therefore insufficient to determine when counter control should end.

## Modes

A minimal event-based structure is:

| Mode | Meaning |
|---|---|
| `PASS_THROUGH` | no manipulation episode and no active recovery |
| `MONITOR` | manipulation/recovery window exists but reaction is not required |
| `ACTIVE` | predicted disturbance or measured response justifies counter optimization |
| `RECOVERY` | manipulation has stopped but measured response remains unsafe |
| `HANDOFF` | response is quiet or a protective transition requires bounded counter-arm return |

## Recovery condition

After manipulation completion, keep monitoring long enough to cover the observed delayed step onset.

Transition out of recovery from measured quietness, not from target name or a fixed normal completion timer.

A maximum recovery duration may remain only as a fault bound.

## Contact / step safety

ALMI stepping may be protective.

If reliable timestamped contact information is available:

- allow active reaction in confident double support;
- stop adding new counter acceleration when a contact transition/step begins;
- use acceleration- and slew-bounded braking;
- do not force posture return during an initiated protective step.

If reliable contact information is unavailable outside simulation, keep it diagnostic-only.

## Handoff

Do not immediately jump from an active counter trajectory to upstream pass-through.

Handoff must preserve:

- position/velocity limits;
- acceleration/slew limits;
- collision/excursion checks;
- finite braking margin.

Return to `ACTIVE` if base divergence reappears before handoff completion.

## Main risk

Poor recovery logic can either fight a protective step or retain counter-arm motion too long.

---

# 15. Design Choice J: Optional Longer Counter-Arm Horizon

## Motivation

A longer OCP may still be useful for:

- acceleration buildup;
- braking timing;
- stopping distance;
- excursion planning.

However, Iteration 3 showed that longer horizon alone is not sufficient.

## Default

Keep H3:

\[
T_{\mathrm{ocp}} = 0.06\ \mathrm{s}.
\]

Use longer disturbance preview first.

## Revisit condition

Only revisit H5/H8 if:

1. a reaction representation has demonstrated useful H3 behavior;
2. remaining failures show evidence of insufficient counter-arm planning horizon rather than bad objective/activation;
3. nominal-trajectory linearization is available;
4. complete nonlinear metric validation is active;
5. full position/velocity/excursion/braking validation is active;
6. full-horizon collision validation is active;
7. the real-time budget still passes.

Candidate horizons remain:

| Candidate | Intervals | Duration |
|---|---:|---:|
| H1 | 1 | `0.02 s` |
| H3 | 3 | `0.06 s` |
| H5 | 5 | `0.10 s` |
| H8 | 8 | `0.16 s` |

A longer horizon must outperform H3 physically.
Lower predicted cost alone is not sufficient.

---

# 16. Design Space

The major exploration axes are:

| Axis | Baseline | Candidate choices |
|---|---|---|
| Reaction representation | momentum \(H\) | \(\dot H\), short-window \(\Delta H\) |
| Manipulation anticipation | H3 moving horizon | longer disturbance preview |
| Feedforward | scalar `0.25` authority | calibrated \(\lambda_{\mathrm{ff}}\dot H_m\) |
| Feedback | gyro | tilt/rate, optional CAM |
| Reaction cost | quadratic | quadratic, later safe envelope |
| Initial active seed | shifted/zero | reaction-informed seed |
| Capacity handling | near-limit + hard gates | braking/reaction-capacity shaping |
| Recovery | gyro after motion | event-based recovery/handoff |
| OCP horizon | H3 | H5/H8 only if justified |

These are not intended to be enabled simultaneously from the start.

The core 3B hypothesis is:

> better representation and timing of arm reaction can improve the existing H3 controller.

Secondary mechanisms should be added only when the evidence exposes the corresponding limitation.

---

# 17. Core vs Supporting Mechanisms

## Core mechanisms

The strongest candidates for producing direct performance gains are:

1. reaction/actuator identification;
2. manipulation-arm reaction feedforward;
3. long disturbance preview with short H3 OCP;
4. \(\dot H\) or short-window \(\Delta H\) reaction objective;
5. measured-response recovery.

## Supporting mechanisms

Use when the core controller exposes the corresponding problem:

- safe reaction envelope: if useful reaction over-intervenes on stable/ALMI cases;
- CAM-enhanced intervention: if gyro/tilt misses useful separation;
- physics-informed warm start: if first active action or one-iteration convergence is limiting;
- reaction-capacity shaping: if early gains are followed by braking/excursion payback;
- longer OCP horizon: if H3 becomes demonstrably horizon-limited.

This distinction keeps Iteration 3B ambitious without turning it into an uncontrolled combination of mechanisms.

---

# 18. Development and Guard Cases

Keep physical families together when separating development and held-out cases.

Historical primary ALMI stumble cases include:

- `left_arm_overhead`;
- `left_fast_fall_search_09_scale_78`;
- `left_fast_fall_search_11_scale_78`;
- `left_manual_grasp_pitch_plus`;
- `left_manual_grasp_pitch_minus`.

Important FAME improvement/fall sentinels include:

- `left_fast_fall_search_06_scale_74`;
- `left_fast_fall_search_09_scale_78`;
- `right_fast_fall_search_11_scale_78`;
- `right_inner_upward_overhang_pitch_plus`;
- `left_lateral_high_reach`;
- `right_upward_arc_rank6`.

No-regression guards include:

- ALMI `right_fast_fall_search_06_scale_74`;
- ALMI `right_fast_fall_search_11_scale_78`;
- ALMI `right_lateral_overhead_reach`;
- FAME `right_diagonal_rank6`;
- mirrored stable forward/cross-body targets;
- standing with no manipulation.

The final family-level development/held-out split should be frozen before candidate outcome inspection.

---

# 19. Evaluation Design

Use paired frame-task, B0, and candidate runs from identical saved simulator state whenever available.

If exact branching is unavailable:

- use fixed simulator seeds;
- interleave controller order;
- preserve identical release-relative timing;
- reject operationally incomplete pairs;
- store seed and trial order.

Use repeated majority severity rather than single-run classification.

Recommended evidence level:

- at least five repetitions for primary stumble, fall-rescue, and historically stochastic cases;
- at least three repetitions for stable/no-regression guards.

Full 44-target FAME and ALMI panels are final regression coverage after candidate selection.
They must not be used to tune parameters and later presented as held-out evidence.

Videos remain required for:

- every changed classification;
- every stumble;
- every fall;
- representative stable guard behavior when a new activation mechanism is introduced.

---

# 20. Metrics

Rank outcomes in this order:

1. confirmed falls and survival;
2. stable / drift / stumble / fall paired transitions;
3. new regressions and safety failures;
4. foot lift and foot displacement;
5. base orientation drift;
6. base translation;
7. moving-arm tracking/pass-through error;
8. counter excursion, velocity, acceleration, and braking margin;
9. controller timing and rejection;
10. diagnostic CoM, CAM, ZMP, CMP, \(H\), \(\dot H\), and \(\Delta H\).

Do not rank candidate quality from mean CoM margin alone.

---

# 21. Acceptance Criteria

## Physical

A strong Iteration 3B candidate should demonstrate:

- at least one repeated ALMI stumble-to-drift/stable conversion;
- lower total majority stumble count than B0 without adding falls;
- repeated FAME fall-to-survival improvement beyond B0;
- preservation of historical B0 FAME improvements;
- no stable-to-worse majority regression.

For a claimed converted stumble, require meaningful reduction in foot displacement/lift unless the relevant metric is already below the classifier threshold.

## Functional

Require:

- manipulation-arm position, velocity, and torque pass-through to tolerance;
- counter wrists remain held;
- no stale-plan publication;
- no accepted collision or hard-limit violation;
- explicit diagnostics for clip, backtrack, rejection, model mismatch, and timing hold.

## Real time

Require:

- pre-publication controller p99 `<= 15 ms`;
- pre-publication maximum `< 20 ms`;
- no active command accepted after a timing-guard violation.

Post-write infrastructure/publisher stalls must be reported separately from solver/controller deadline misses.

---

# 22. Failure Interpretation

Iteration 3B should not continue adding arm-only terms indefinitely.

If the best validated combination of:

- actuator calibration;
- manipulation disturbance preview;
- momentum-rate or angular-impulse shaping;
- measured-response intervention;
- recovery/handoff;
- reaction-capacity management;
- appropriate short-horizon optimization

cannot produce repeated physical improvement without regression, the result is scientifically useful.

It would indicate that the missing information is likely not another arm-only residual, but the actual dynamic response of the base/lower-body/contact system.

That is the Iteration 4 transition.

---

# 23. Iteration 4 Boundary

Name the next work Iteration 4 if it adds any of:

- base roll/pitch to the optimized dynamic state;
- base angular velocity to the optimized dynamic state;
- an identified manipulation/counter-arm-to-base transition model;
- contact/support mode inside the OCP;
- predicted ground reaction/contact wrench as an optimization variable;
- lower-body or whole-body controls.

A minimal future Iteration 4 model may eventually use a state such as

\[
x_b =
[
\theta_{\mathrm{roll}},
\theta_{\mathrm{pitch}},
\omega_x,
\omega_y
]
\]

with identified local response to manipulation-arm disturbance and counter-arm action.

That model is intentionally excluded from Iteration 3B.

---

# 24. Relevant Literature

### Zhang et al. 2014

Da-song Zhang, Rong Xiong, Jun Wu, Jian Chu,
“Balance Maintenance in High-Speed Motion of Humanoid Robot Arm-Based on the 6D Constraints of Momentum Change Rate,”
*The Scientific World Journal*, 2014.
DOI: `10.1155/2014/535294`.

Relevant idea:
auxiliary-arm balance regulation through momentum change rate with joint velocity/acceleration constraints.

### Lee, Jeon, and Kim 2025

Ho Jae Lee, Se Hwan Jeon, Sangbae Kim,
“Learning Humanoid Arm Motion via Centroidal Momentum Regularized Multi-Agent Reinforcement Learning,”
2025, arXiv:`2507.04140`.

Relevant idea:
base state and centroidal angular momentum provide useful shared balance information for arm/leg coordination.

### Raza, Zhu, and Hayashibe 2021

Fahad Raza, Wei Zhu, Mitsuhiro Hayashibe,
“Balance Stability Augmentation for Wheel-Legged Biped Robot Through Arm Acceleration Control,”
*IEEE Access*, 2021.
DOI: `10.1109/ACCESS.2021.3071055`.

Relevant idea:
active arm acceleration can augment balance, and CMP can expose the support effect of centroidal angular-momentum rate.

### Shen, Chemori, and Hayashibe 2021

Keli Shen, Ahmed Chemori, Mitsuhiro Hayashibe,
“Reproducing Human Arm Strategy and Its Contribution to Balance Recovery Through Model Predictive Control,”
*Frontiers in Neurorobotics*, 2021.
DOI: `10.3389/fnbot.2021.679570`.

Relevant idea:
predicting body recovery dynamics together with arm strategy can improve balance; this motivates the Iteration 4 boundary if arm-only prediction saturates.

### Zhang et al. 2025

Tianlin Zhang, Linzhu Yue, Hongbo Zhang, Lingwei Zhang, Xuanqi Zeng, Zhitao Song, Yun-Hui Liu,
“Whole-Body Control Framework for Humanoid Robots with Heavy Limbs: A Model-Based Approach,”
2025, arXiv:`2506.14278`.

Relevant idea:
heavy-limb coupling motivates explicit whole-body/contact-aware prediction when a fixed-base arm-overlay model becomes insufficient.

---

# 25. Design Summary

Iteration 3B should remain centered on one question:

> Can the existing counter arm produce a better-timed and better-sized reaction when the manipulation disturbance is anticipated explicitly?

The preferred design space is:

\[
\boxed{
\text{identify arm reaction}
\rightarrow
\text{preview manipulation disturbance}
\rightarrow
\text{shape } \dot H \text{ or } \Delta H
\rightarrow
\text{use measured response}
\rightarrow
\text{preserve reaction capacity}
}
\]

while keeping the OCP short and real-time.

The most promising combination is expected to involve:

- H3 OCP;
- longer manipulation disturbance preview;
- calibrated manipulation-arm feedforward;
- \(\dot H\) or \(\Delta H\) reaction shaping;
- measured tilt/rate feedback;
- event-based recovery.

Safe envelopes, CAM-enhanced activation, reaction-informed warm start, and capacity-aware authority are supporting design choices that can be introduced when the corresponding failure mode is observed.

Only after a good reaction objective is established should H5/H8 be reconsidered.

If these arm-side mechanisms cannot produce repeated gains without regression, the correct next step is not another scalar tuning pass.
It is Iteration 4 base-response prediction.

---

# 26. Experiment Log

## Experiment 3B-A0: Zero-Weight Arm-Reaction Identification

### Hypothesis

The frozen arm model predicts the sign and timing of realized counter-arm
momentum change well enough to define a normalized reaction residual. Commanded
counter acceleration has a small, repeatable delay and gain across FAME, ALMI,
and stable guards.

### Minimum Mechanism

Add diagnostics only. Do not change B0 activation, objective, horizon, limits,
or solver behavior.

Log with monotonic timing:

- Current measured counter, moving-arm, and total planar arm momentum.
- Filtered measured counter, moving-arm, and total momentum rate.
- Predicted first-step counter, moving-arm, and total momentum rate.
- Requested and applied counter acceleration.
- Filtered measured counter acceleration.
- Measured planar gyro and filtered base angular acceleration.
- Diagnostic sample interval and validity flags.

### Development Cases

- FAME `right_fast_fall_search_11_scale_78`: historical fall-rescue sentinel.
- FAME `left_lateral_overhead_reach`: B0 improvement sentinel.
- ALMI `left_fast_fall_search_09_scale_78`: stumble/boundary response.
- ALMI `right_fast_fall_search_11_scale_78`: stable guard.

Use three repeated B0 trials per case. The diagnostic implementation has zero
cost weight and must not change paired outcome or command pass-through.

### Evidence To Continue

Proceed to a weighted reaction representation only if:

- Predicted and measured counter momentum-rate axes have repeatable sign.
- Command-to-measured acceleration and momentum-rate delay is stable within two
  control intervals.
- A single per-axis gain/scale is meaningful across repeated cases.
- Frozen prediction error is bounded enough to normalize a residual.
- Base angular-acceleration relation is reported separately and is not assumed
  to be causal from correlation alone.

If one-step momentum rate is too noisy or delay-sensitive, reject instantaneous
rate and test short-window momentum change before any weighted objective.

### Result

Artifacts are stored under `runs/archive/key_findings/iteration3b/a0_reaction_identification/`.
Three B0 repetitions were run for each development case without changing the
objective or controller behavior.

Physical outcomes were:

- FAME `right_fast_fall_search_11_scale_78`: 2 drift, 1 fall.
- FAME `left_lateral_overhead_reach`: 3 stable.
- ALMI `left_fast_fall_search_09_scale_78`: 2 stumble, 1 drift.
- ALMI `right_fast_fall_search_11_scale_78`: 3 stable.

The arm-side identification result was:

- Predicted-to-measured counter momentum-rate delay: one tick in all 12 runs.
- Commanded-to-measured acceleration delay: one to two ticks.
- Per-axis static effectiveness from development data: approximately
  `[0.105, 0.115]`.
- Held-out predicted/measured rate correlation: `[0.674, 0.625]`.
- Held-out rate-model `R^2`: `[0.422, 0.384]`.
- Held-out sign consistency: `[75.6%, 77.5%]`.
- Counter momentum-rate to base angular-acceleration correlation: weak,
  approximately `0.07–0.35`.

The initial runs used controller-entry time as the sample timestamp. After
diagnostic isolation and source-state timestamp correction, confirmation runs
retained the one-tick delay:

- `runs/archive/key_findings/iteration3b/timestamp_confirmation/20260829_072134_iter3b_a0_timestamp_confirm_fame_right11`.
- `runs/archive/key_findings/iteration3b/timestamp_confirmation/20260829_072257_iter3b_a0_timestamp_confirm_almi_right11`.

Corrected FAME correlation/gain was `[0.644, 0.519]` / `[0.078, 0.104]`.
Corrected ALMI correlation/gain was `[0.785, 0.762]` / `[0.122, 0.128]`.
The one-tick delay and positive effectiveness conclusion remain valid, while the
single global gain remains approximate.

Decision:

- Keep the one-tick delayed arm-side gain model as a diagnostic and possible
  initialization model.
- Do not use the weak base-acceleration correlation as a feedback model.
- Do not add a weighted instantaneous momentum-rate residual yet; normalized
  error remains large.

A `60 ms` short-window momentum-change estimate constructed from the same
receding predictions performed worse on held-out data: correlation
`[0.350, 0.122]`, `R^2 [0.102, -0.019]`, and sign consistency around `63%`.
Reject that construction rather than adding both reaction representations.

## Experiment 3B-A1: Zero-Weight Manipulation Disturbance Preview

### Hypothesis

A longer exogenous manipulation preview can expose the time, sign, and magnitude
of the upcoming moving-arm disturbance earlier than H3 without increasing the
counter-arm optimization horizon or changing B0 control.

### Minimum Mechanism

Keep the H3 OCP and B0 objective unchanged. Add a diagnostic-only preview of the
known manipulation trajectory over `0.20 s` initially.

Log:

- Peak predicted moving-arm momentum rate.
- Peak predicted moving-arm momentum change from the current sample.
- Discounted absolute momentum-rate exposure.
- Time to peak disturbance.
- Current H3 values of the same quantities for direct comparison.
- Frozen-map versus realized moving-arm momentum error as the preview unfolds.

### Development Cases

Reuse the A0 cases and add mirrored FAME boundary and ALMI stable guards if the
initial distributions overlap.

### Evidence To Continue

The preview mechanism is retained only if it:

- Detects useful FAME disturbance earlier than H3.
- Preserves sign when compared with realized moving-arm momentum change.
- Has bounded error across FAME and ALMI.
- Separates disturbance prediction from measured response without changing B0
  commands or timing materially.

Do not use preview for activation, objective weight, or warm start until these
diagnostics are established.

### Result

Artifacts:

- `runs/archive/key_findings/iteration3b/a1_disturbance_preview/20260829_021008_iter3b_a1_fame_right11`.
- `runs/archive/key_findings/iteration3b/a1_disturbance_preview/20260829_021130_iter3b_a1_fame_lateral_overhead`.
- `runs/archive/key_findings/iteration3b/a1_disturbance_preview/20260829_021252_iter3b_a1_almi_left09`.
- `runs/archive/key_findings/iteration3b/a1_disturbance_preview/20260829_021406_iter3b_a1_almi_right11`.

The frozen-current-map preview predicted future moving-arm momentum with planar
correlation between `0.73` and `0.96`. Pitch correlation was `0.95–0.96` in all
four cases. The preview exposed the disturbance approximately `80–100 ms`
earlier than H3 and showed roughly three times the H3-visible momentum change.

FAME and ALMI versions of the same motion had nearly identical preview
features. Preview is therefore retained for anticipatory feedforward timing, not
as a policy or outcome discriminator.

Decision:

- Keep the `0.20 s` diagnostic preview with H3 control frozen.
- Do not use preview directly for activation magnitude.
- Use previewed moving-arm momentum rate in the next reaction-feasibility
  diagnostic.

## Experiment 3B-A2: Zero-Weight Reaction-Seed Feasibility

### Hypothesis

The early counter reaction observed in successful Iteration 2 FAME rescues can
be represented by a bounded counter-acceleration seed under the B0 local arm
model.

Successful Iteration 2 right-boundary rescues showed opposite moving-to-counter
pitch momentum-rate correlation `0.81–0.88`, realized pitch effectiveness
`0.32–0.37`, and approximately `30 ms` lag. B0 failed cases produced only
`0.05–0.08` pitch effectiveness.

### Minimum Mechanism

Do not change the solver seed or objective. Compute and log a candidate first
acceleration:

\[
\dot H_c^*=-\lambda_{ff}\dot H_{m,preview},
\]

using diagnostic `lambda_ff = 0.30` initially. Correct the frozen model target
with the A0 arm-side effectiveness `[0.105, 0.115]`, solve the local least-squares
acceleration seed, and pass it through the existing acceleration, slew,
position, velocity, excursion, and collision-independent first-step bounds.

Log:

- Unbounded and clipped candidate acceleration.
- Joint saturation mask.
- Desired measured reaction.
- Predicted achieved measured reaction after effectiveness correction.
- Reaction residual before and after clipping.
- Whether the candidate is finite and first-step feasible.

### Evidence To Continue

Use the candidate seed in FDDP only if it is finite, sign-consistent, and not
persistently saturated on FAME development cases. It must not request materially
larger or more saturated reaction on stable ALMI guards for equivalent preview
motion without measured-response justification.

### Result

Artifacts:

- `runs/archive/key_findings/iteration3b/a2_reaction_seed_feasibility/20260829_021914_iter3b_a2_fame_right11`.
- `runs/archive/key_findings/iteration3b/a2_reaction_seed_feasibility/20260829_022036_iter3b_a2_almi_left09`.
- `runs/archive/key_findings/iteration3b/a2_reaction_seed_feasibility/20260829_022151_iter3b_a2_almi_right11`.

The candidate reaction seed was infeasible in `88–92%` of active samples and
saturated at least one joint in the same fraction. Median relative reaction
error remained `0.78–0.84`; high-percentile error was substantially larger.
FAME and ALMI cases were similarly saturated.

The initial feasibility diagnostic omitted the captured excursion intersection.
That check is now included. The omission made feasibility optimistic, so it does
not reverse the rejection decision.

A first-order realized-acceleration model was also identified offline. Shoulder
joints achieved held-out `R^2` around `0.59–0.63`, but elbow `R^2` was only
`0.15`. Explicit additional command lag did not improve the model.

Decision:

- Reject reaction-informed warm start at this stage.
- Reject a 12-state actuator augmentation based on a partially valid model.
- Do not lower the reaction target until it fits; that would be another scalar
  search without a validated reaction objective.

## Experiment 3B-A3: Zero-Weight Measured Response

### Hypothesis

Settled-reference tilt divergence can distinguish difficult recovery episodes
from stable lower-policy response and indicate when post-motion monitoring is
needed.

### Minimum Mechanism

Log settled tilt error, planar gyro, per-axis tilt-rate product, and positive
divergence norm. Do not change B0 activation or recovery.

### Result

Artifacts:

- `runs/archive/key_findings/iteration3b/a3_response_diagnostics/20260829_022616_iter3b_a3_fame_right11`.
- `runs/archive/key_findings/iteration3b/a3_response_diagnostics/20260829_022737_iter3b_a3_almi_left09`.
- `runs/archive/key_findings/iteration3b/a3_response_diagnostics/20260829_022851_iter3b_a3_almi_right11`.

Post-motion divergence peaks were:

- FAME fall/rescue sentinel: `0.0273`.
- ALMI stumble/boundary case: `0.0554`.
- ALMI stable guard: `0.0036`.

Existing gyro recovery already remained active for 19–26 post-motion ticks in
the difficult cases and only 5 ticks in the stable guard. Measured response is a
useful recovery monitor, but B0 already detects the important divergence.

Decision:

- Retain tilt divergence as a diagnostic and future recovery signal.
- Do not add another response threshold to B0.
- Do not claim that response magnitude defines the counter reaction direction or
  target.

## Iteration 3B Stop Decision

The controlled 3B experiments establish:

1. Manipulation preview is accurate and earlier than H3, but nearly identical
   for equivalent FAME and ALMI motions.
2. Arm-side momentum-rate prediction has repeatable delay/sign but insufficient
   accuracy for a weighted residual.
3. Short-window momentum change performs worse than one-step rate prediction.
4. The identified reaction target is mostly infeasible under existing bounds.
5. A first-order arm-actuator model is not valid across all active joints.
6. Measured response distinguishes difficult recovery from stable response, but
   existing gyro recovery already captures that distinction.

No weighted Iteration 3B mechanism is justified by the evidence. Adding one now
would combine an accurate disturbance preview with an unreliable map from arm
command to useful whole-body response.

Further improvement requires predicting how manipulation disturbance and
counter-arm action affect planar base response. Contact may remain an observed
safety/mode signal initially; predicted contact evolution is added only if the
base-response model proves insufficient. Both changes remain inside the
Iteration 4 boundary defined in this document. Iteration 3B stops with diagnostic
infrastructure retained and B0 control unchanged.
