# Why the ZMP Balance Updates Show Limited Improvement

## Bottom Line

The results are internally consistent. **Update 3 fixed a genuine implementation bug**: the reduced-arm solve produced whole-body momentum nearly opposite the requested direction. But after correcting that, the controller is still limited by three deeper issues:

1. **It regulates arm angular momentum \(H_{\text{arm}}\), while balance is affected by its rate \(\dot H_{\text{arm}}\).**
2. **Update 4 still arrests that momentum on a fixed timer, often paying back the corrective impulse while the disturbance or recovery is still active.**
3. **A one-second force is a sustained-load benchmark, whereas finite-stroke arm motion is primarily a transient flywheel actuator.**

This naturally produces occasional one-bracket gains, directional tradeoffs, and post-force failures rather than broad improvement.

---

## 1. The Most Important Conceptual Loophole: \(H\) Versus \(\dot H\)

The ZMP relation contains angular-momentum **rate**:

\[
z_x
=
c_x-\frac{z_c}{g}\ddot c_x-\frac{\dot H_y}{mg}.
\]

Rearranging,

\[
\ddot c_x
=
\frac{g}{z_c}(c_x-z_x)
-\frac{\dot H_y}{m z_c}.
\]

But the implemented allocator solves approximately

\[
A_{\text{arm}}(q)\dot q_{\text{arm}}
=
H_{\text{target}},
\]

so it regulates arm **momentum**, not momentum rate. Once the arm reaches roughly constant velocity and therefore constant momentum,

\[
\dot H_{\text{arm}}\approx0,
\]

and the arm no longer supplies a useful reaction moment.

The original plan converts \(\dot H_{\text{des}}\) into a short \(H_{\text{target}}\), but that approximation only makes sense if acceleration, duration, and braking are deliberately controlled.

This gives an important integral result:

\[
\Delta \dot c_x^{\text{arm}}
=
-\frac{H_y(t_2)-H_y(t_1)}{m z_c}.
\]

If the arms begin and end with zero momentum, the **direct net change in COM velocity from the flywheel term is zero** under the simplified model. The arms can reshape the trajectory and buy time for the feet and lower-body controller, but the opposite momentum eventually has to be absorbed somewhere.

Therefore, braking and recovery are not secondary details. They are half of the balance action.

---

## 2. Update 4 Delays Posture Return, but Not Momentum Payback

Update 4 introduces a cleaner state machine:

```text
IDLE -> REJECT -> ARREST -> RECOVERY_WAIT -> RETURN -> IDLE
```

However, the critical transitions are still largely timer based:

- `REJECT` lasts at most \(0.60\) s.
- `ARREST` ramps arm velocity to zero over \(0.12\) s.
- Only the later `RECOVERY_WAIT -> RETURN` transition is gated by measured quietness.

The key loophole is:

> **Stopping arm velocity is already the harmful opposite impulse, even if arm posture is not returning yet.**

Suppose the arm spins up in the useful direction. During `ARREST`, its momentum changes from \(H_{\text{arm}}\) back to zero. The torso receives the opposite reaction. If `ARREST` starts at \(0.60\) s during a one-second force, the controller pays back its momentum while the external force is still being applied.

Therefore, the criterion “no posture return during the force” is insufficient. The stronger requirement is:

\[
\text{Do not remove useful arm momentum until the lower body can absorb the braking impulse safely.}
\]

Extending `REJECT` to \(0.8\) or \(1.0\) s does not necessarily solve this:

- Once \(H_{\text{target}}\) is reached, holding it provides little \(\dot H\).
- Braking near force removal can interfere with natural lower-body recovery.
- A fixed force-end time is not the same as a dynamically safe braking time.

This explains why Update 4 can be conceptually cleaner yet perform worse than repeated bursts.

---

## 3. The Benchmark Is Unfavorable to Finite Arm Momentum

The accepted Update 3 configuration limits approximately:

\[
|H_x|\le0.2\ \mathrm{N\,m\,s},
\qquad
|H_y|\le0.8\ \mathrm{N\,m\,s}.
\]

For a humanoid with mass \(m\) and COM height \(z_c\), the largest temporary COM-velocity change available from an arm momentum change is approximately

\[
|\Delta\dot c|
\lesssim
\frac{\Delta H}{m z_c}.
\]

For example, using \(m=60\) kg and \(z_c=0.9\) m:

- \(0.8\ \mathrm{N\,m\,s}\) corresponds to about \(0.015\ \mathrm{m/s}\).
- \(0.2\ \mathrm{N\,m\,s}\) corresponds to about \(0.0037\ \mathrm{m/s}\).

Use the robot’s actual mass and COM height, but the order of magnitude is what matters.

If \(0.8\ \mathrm{N\,m\,s}\) is generated over \(0.12\) s,

\[
\dot H\approx6.7\ \mathrm{N\,m},
\]

which shifts ZMP by approximately

\[
\Delta z \approx \frac{\dot H}{F_z}.
\]

For \(F_z\approx600\) N, this is about \(11\) mm, but only during spin-up. Spread the same momentum change over one second and the average shift is only around \(1.3\) mm.

So the controller may provide a useful short transient near a stability boundary, but it cannot continuously oppose a one-second load without exhausting arm momentum. This is consistent with isolated \(5\) N boundary changes rather than a large aggregate gain.

A better characterization should include:

- \(0.1\)–\(0.2\) s impulses, where arm reflexes should help most.
- \(0.5\) s pulses.
- The existing \(1.0\) s sustained force.
- Equal-impulse tests as well as equal-peak-force tests.

The controller may be effective as an impulse reflex while being fundamentally unable to increase sustained-load capacity.

---

## 4. ZMP Residual Is Not an External-Disturbance Observer

The original design proposed combining:

- ZMP,
- COM motion,
- IMU motion,
- and an external-force proxy.

In practice, the accepted controller relies heavily on calibrated ZMP residual, while COM velocity and angular-velocity feedback remain disabled, incomplete, or unvalidated.

The problem is:

> **ZMP is an output of the external disturbance, lower-body controller, contact dynamics, and arm motion simultaneously.**

It does not directly identify which of those produced the change.

This creates several failure modes:

- The lower body can move ZMP before significant base motion occurs.
- A translating torso push may only appear after joint deformation if free-base velocity is incomplete.
- Arm acceleration and braking themselves move ZMP.
- During recovery, ZMP residual can reverse sign while disturbance-induced COM velocity remains dangerous.
- A target continuously following ZMP residual can chase the lower-body controller with substantial phase lag.

Update 2 observed force-window detection latencies as high as \(0.919\) s. Update 4 still had approximately \(0.386\)–\(0.620\) s latency with the conservative detector. By then, \(40\%\)–\(60\%\) of the one-second pulse has already occurred.

The faster residual-velocity trigger reduced command latency to about \(0.23\)–\(0.28\) s but lost a guard case. This does not necessarily mean early reaction is harmful. More likely:

- the faster signal is not sufficiently exogenous,
- or the actuation and braking policy cannot safely handle earlier activation.

---

## 5. Update 3 Proved Kinematic Alignment, Not Physical Effectiveness

Update 3 was important. It replaced inconsistent isolated-arm centroidal maps with current full-body map columns and removed the roughly \(-0.93\) target-to-predicted-momentum alignment.

But several gaps remain between “predicted momentum aligns with target” and “the robot receives a useful balance moment”:

- The solve tracks \(H_{\text{arm}}\), not \(\dot H_{\text{arm}}\).
- End-effector velocity constraints may still be applied after the solve.
- Acceleration, jerk, braking distance, and servo dynamics are omitted.
- Commands are integrated through a position-control interface.
- Measured acceleration can differ substantially from planned velocity changes.
- The angular centroidal objective does not constrain undesirable linear arm momentum.
- It does not model how the lower-body controller reacts to arm motion.
- A scalar `reaction_sign: -1` is not a substitute for a validated, state-dependent input-output mapping.
- Arm momentum about the COM is not identical to the change in support moment or base angular acceleration produced by the closed-loop robot.

The actual plant mapping of interest is closer to

\[
\Delta
\begin{bmatrix}
\dot\omega_{\text{base}}\\
z_{\text{ZMP}}\\
\dot c\\
M_{\text{contact}}
\end{bmatrix}
=
B(q,\dot q,\text{contact},K_{\text{lower}})
\,u_{\text{arm}},
\]

where \(u_{\text{arm}}\) includes the real position-servo response.

Centroidal-map alignment validates only one internal part of this mapping.

---

## 6. The Arm Controller Is Not Coordinated With the Lower-Body Controller

Arm motion cannot directly remove whole-body horizontal linear momentum. Internal forces preserve whole-body COM momentum. The feet and environment must provide the net horizontal impulse.

Arms can help indirectly by:

- shifting the centroidal moment pivot,
- changing the contact moment required from the feet,
- delaying base rotation,
- buying time for ankle, hip, or stepping strategies.

But the arm reflex is layered on top of the lower-body controller rather than jointly optimized with it.

Consequences include:

- The lower body may counteract the arm reaction.
- The arm controller may react to ZMP motion deliberately created by the lower-body controller.
- Arm braking may occur exactly when the lower body is trying to arrest the base.
- The lower-body controller receives no feed-forward notification of the expected arm reaction wrench.

The arms may therefore mostly **redistribute controller effort** rather than increase the reachable stability set.

At minimum, the lower-body controller should receive the expected arm reaction wrench. A stronger solution is a shared centroidal objective where foot wrench, ZMP, and arm momentum rate are optimized together.

---

## 7. Statistical Resolution Is Comparable to Timing Noise

The benchmark can easily hide or manufacture a \(5\) N result.

Between the latest Update 3 and Update 4 sweeps:

- The fixed-upper \(+Y\) boundary moved from \(110\) N to \(115\) N.
- The ZMP boundary moved from \(115\) N to \(110\) N.

That sign reversal is larger than the claimed controller effect and indicates substantial boundary sensitivity.

A useful impulse comparison:

- A \(5\) N difference over one second is \(5\ \mathrm{N\,s}\).
- One \(30\) Hz control tick at \(115\) N is approximately

\[
115\times0.033\approx3.8\ \mathrm{N\,s}.
\]

- A \(25\) ms force-onset timing variation at \(115\) N corresponds to about \(2.9\ \mathrm{N\,s}\).
- First-command latency variations of hundreds of milliseconds correspond to tens of \(\mathrm{N\,s}\).

Thus, one control tick of timing difference is already comparable with the full claimed \(5\) N boundary improvement.

Repeated Update 3 trials also show near-threshold survival variation. The defensible conclusion is therefore:

> There is evidence of small directional arm authority, but no statistically established increase in the global disturbance envelope.

---

## Most Revealing Next Experiments

### 1. Use an Oracle Trigger and Oracle Disturbance Direction

In simulation:

- Trigger exactly at force onset.
- Supply the true disturbance direction.
- Keep the same arm actuation.

This separates observer limitations from actuator limitations.

Interpretation:

- If oracle triggering still gives little improvement, authority and actuation are the primary limits.
- If it gives a large improvement, observation and timing dominate.
- If rejection improves but the robot still falls after force removal, braking and recovery dominate.

### 2. Run a “No Braking Until Commanded” Experiment

Spin the arms up once, then hold their achieved momentum through the complete force interval. Do not automatically arrest at \(0.60\) s. After force removal, test several braking times.

This directly tests whether `ARREST` destroys the benefit.

Plot:

\[
H_{\text{arm}},
\quad
\dot H_{\text{arm}},
\quad
\omega_{\text{base}},
\quad
\dot c,
\quad
\text{capture point},
\quad
\text{ZMP/CMP},
\quad
M_{\text{contact}}.
\]

### 3. Build an Impulse Ledger

For every trial, integrate:

\[
\int \tau_{\text{external}}\,dt,
\qquad
\Delta H_{\text{arm}},
\qquad
\int M_{\text{contact}}\,dt,
\qquad
\Delta H_{\text{whole body}}.
\]

Separate:

- useful arm spin-up impulse,
- harmful arm braking impulse,
- posture-return impulse,
- lower-body/contact contribution.

This will reveal whether the arm helps initially and then cancels itself.

### 4. Branch Both Variants From an Identical Simulator State

At force onset:

- Save the complete MuJoCo state.
- Save actuator and controller-integrator state.
- Branch into `upper_fixed` and `zmp_enabled`.
- Apply exactly the same step-indexed force.

This is stronger than interleaving trials or enforcing a standing dwell. It removes pre-force trajectory and asynchronous onset differences.

### 5. Identify the Actual Closed-Loop Arm Reaction Map

Apply small, safe basis pulses in arm joint acceleration or velocity increments and measure:

- base angular acceleration,
- ZMP displacement,
- contact moment,
- linear COM acceleration,
- delay,
- cross-axis coupling.

Fit a local mapping from command to measured support response. This determines whether `reaction_sign = -1` is sufficient or whether a state-dependent \(2\times n\) or \(3\times n\) mapping is required.

---

## Recommended Controller Formulation

The next controller should treat arm angular momentum as a bounded state and its rate as the control:

\[
H_{k+1}=H_k+\Delta t\,\dot H_k.
\]

A flywheel-aware MPC or constrained regulator can use a state such as

\[
x=
\begin{bmatrix}
\text{capture point}\\
\dot c\\
\omega_{\text{base}}\\
H_{\text{arm}}
\end{bmatrix},
\]

with input

\[
u=\dot H_{\text{arm}}.
\]

Constraints should include:

- arm momentum \(H_{\text{arm}}\),
- momentum rate \(\dot H_{\text{arm}}\),
- joint velocity and position,
- acceleration and jerk,
- end-effector motion,
- support polygon or contact wrench.

The terminal objective should not force \(H_{\text{arm}}\to0\) immediately. It should return arm momentum to zero only when the predicted capture state and lower-body/contact authority can absorb the braking impulse.

At the joint level, solve for momentum rate rather than momentum:

\[
\dot H
=
A(q)\ddot q+\dot A(q,\dot q)\dot q.
\]

If the position servo makes the analytical model inaccurate, use an experimentally identified command-to-\(\dot H\) model.

---

## Ranking of the Current Bottlenecks

1. **Controlling \(H\) instead of \(\dot H\), with uncontrolled braking impulse.**
2. **Fixed-time `ARREST` while the force or dynamic recovery remains active.**
3. **Detection latency and incomplete source-timestamped base/COM state.**
4. **Insufficient arm momentum authority for a sustained one-second load.**
5. **No coordination with the lower-body/contact controller.**
6. **Predicted centroidal momentum used as a proxy for actual support response.**
7. **Single-run, 5 N-resolution boundaries with tick-scale timing variability.**

Update 4 does not show that the state-machine idea is wrong. It shows that **changing burst sequencing without changing the controlled variable and arrest criterion is insufficient**.

The next decisive test is an oracle-triggered, momentum-rate-controlled experiment with braking deliberately delayed and measured.
