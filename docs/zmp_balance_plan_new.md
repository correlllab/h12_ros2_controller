# Direct ZMP Balance Controller

## Status

This document records the first small, benchmark-driven implementation of a
direct balance-to-arm controller. It is intentionally not a replacement for
the lower-body policy and does not yet improve every disturbance metric.

The accepted result is directional but now includes a survival-margin gain:

- Positive X, 35 N for 1 s: fixed arms fall in 4/4 validation and baseline
  runs; the direct controller survives 3/4 runs.
- Positive X, 37.5 N: the direct controller falls in 2/2 runs, giving a current
  direct-control bracket of 35-37.5 N.
- No-force behavior remains quiet: zero solves and zero arm command.
- An eight-direction 30 N regression preserves the previous pass/fail pattern:
  both variants pass seven directions and fail negative X.
- Negative X, 30 N: both fixed-arm and direct controllers fall; no improvement
  is established in that direction.
- Positive and negative Y, 30 N: the current ZMP residual does not reliably
  distinguish these impulses from no-force motion, so the arm controller stays
  inactive.

This is a repeatable positive-X pass-rate and threshold improvement, not yet a
general full-sweep margin improvement.

## Existing System Findings

### Benchmark workflow

One headless impulse run can be executed with:

```bash
uv run python -m h12_zmp_benchmark.experiment.impulse_sweep \
    --config config/zmp_no_force.yaml \
    --variant direct_zmp \
    --directions 1 \
    --force 30 \
    --output-dir direct_px30
```

The retained threshold workflow tests eight directions in 5 N increments and
stops each variant-direction after its first failure:

```bash
uv run python -m h12_zmp_benchmark.experiment.impulse_sweep \
    --config config/zmp_no_force.yaml \
    --variant upper_fixed \
    --variant direct_zmp \
    --directions 8 \
    --min-force 5 \
    --max-force 160 \
    --force-step 5 \
    --output-dir threshold_comparison
```

Explicit repeated `--force` arguments retain fixed-force grid mode. The
threshold sweep writes:

- `sweep_manifest.json`: configuration, run directory, completion, fall, and
  result metrics for every run.
- `impulse_sweep_results.csv`: plotting-friendly per-run results.
- `impulse_limits.csv`: highest pass and first failure for each
  variant-direction.
- `runs/<run>/summary.json`: full-run, impulse-window, policy, process, and
  direct-controller metrics.

The top-level `settle_time` and `readiness.stable_time` fields are not used by
the current runtime. Standing qualification uses the `standing` section. The
applied "impulse" is a rectangular force over the configured duration.

### Baselines and legacy controller

`upper_fixed` runs the adaptive lower-body policy while holding the 15 upper
body joints. The archived `legacy_zmp` controller is not a valid active
comparison in the current checkout because its frozen profile has
`zmp.enabled: false`.

The legacy controller pipeline is:

```text
balance observer
    -> disturbance detector
    -> fixed-response-time momentum target
    -> per-arm allocation
    -> burst/return actuator
```

The principal issue is that measured balance error is converted to a momentum
target through a preselected response duration. The actuator then executes a
burst and may return the arms even when the disturbance is still active.

### Reused interfaces

The new controller reuses:

- `UpperController`: DDS state, command publisher, safety checking, robot
  model, reduced arm model, and command limits.
- `BalanceObserver`: calibrated ZMP residual and support measurements.
- `RobotDynamics`: centroidal quantities, gravity compensation, and Crocoddyl
  construction.
- `LowCmdHandler.set_joint_commands()`: atomic arm-only position, velocity, and
  torque commands.

The existing `MomentumDDP` uses one isolated arm, arm position as state, arm
velocity as control, and hold, momentum, and return phases. It remains
available but is not used by the new controller.

## First Formulation

### Measurement

The first controller uses only the calibrated planar ZMP residual:

\[
e_k = K_z (z_k - z_{ref}).
\]

COM velocity is not included because the current floating-base velocity in
`RobotModel.full_v()` is zero. Angular acceleration, force proxy, and support
margin are also excluded so each added signal can be evaluated separately.

### Crocoddyl state and control

The Crocoddyl state is the 14-joint two-arm configuration:

\[
x_k = q_{arm,k} \in \mathbb{R}^{14}.
\]

The control is the two-arm joint velocity:

\[
u_k = \dot q_{arm,k} \in \mathbb{R}^{14}.
\]

The one-step transition is:

\[
q_{k+1} = \operatorname{integrate}(q_k, \Delta t u_k).
\]

Both arms are solved together. Torso and lower-body joints are frozen in the
reduced Pinocchio model.

### Direct balance prediction

Let arm angular momentum be:

\[
H(q,u) = A_H(q)u.
\]

The controller compares commanded momentum to measured arm momentum and
predicts the planar balance residual after one control step:

\[
\hat e_{k+1} = e_k + \frac{s}{mg\Delta t}
\begin{bmatrix}
\Delta H_y \\
-\Delta H_x
\end{bmatrix},
\qquad
\Delta H = H(q,u) - H_{measured}.
\]

`effect_sign` is `+1.0`. A forward fall produces a rearward-negative X ZMP
residual; the corrected sign requests positive arm angular momentum about Y,
which moves both arms backward and gives the torso a counter-pitch reaction.
The earlier `-1.0` sign moved both arms forward and amplified the fall.

### Loss

The one-step running cost is:

\[
\ell =
\frac{w_b}{2}\|\hat e_{k+1}\|^2
+ \frac{w_v}{2}\|u-\dot q_{measured}\|^2
+ \frac{w_q}{2}\|q-q_{ref}\|^2
+ \frac{w_l}{2}\|q_{violation}\|^2.
\]

The terminal model contains only weak posture and limit costs. The control
measurement also includes pitch-rate damping:

\[
e_{control,x} = e_{zmp,x} - k_{\omega}\omega_y.
\]

ZMP residual gates entry, while the combined measurement is used by
Crocoddyl. After pitch rate first exceeds 0.10 rad/s and then decays to
0.05 rad/s, the response terminates and holds the measured arm posture. This
state event prevents the continuous controller from driving the robot into a
backward fall.

Local analytical derivatives are supplied to Crocoddyl. Numerical
differentiation took about 66.6 ms per solve and reduced the feedback loop to
roughly 15 Hz. Analytical derivatives reduced mean solve time to about 0.94 ms
and maximum observed solve time below 3 ms.

### Command interface

The optimized velocity must be sent as a velocity command. The generic
`UpperController._apply_velocity_command()` is not sufficient because it
integrates the velocity into a position target and publishes desired velocity
as zero.

The direct controller therefore publishes, for the 14 arm joints only:

\[
q_{cmd} = \operatorname{integrate}(q, \Delta t u),
\qquad
\dot q_{cmd} = u,
\qquad
\tau_{cmd} = \tau_g(q).
\]

When feedback is inactive, desired arm velocity is explicitly set to zero and
the last arm position target is retained.

## Validated Operating Region

The retained configuration is in
`config/balance_direct_safety_split.yaml`:

```yaml
zmp:
  enabled: true
  feedback:
    zmp_gain: [1.0, 1.0]
    control_zmp_gain: [1.0, 1.0]
    angular_velocity_gain: [0.10, 0.0]
    tilt_gain: [0.0, 0.0]
    enter_threshold: 0.012
    exit_threshold: 0.005
    max_error: 0.04
    abort_threshold: 1.0
    stop_on_rate_decay: true
    rate_peak_threshold: 0.10
    stop_rate_threshold: 0.05
  ddp:
    maxiter: 2
    effect_sign: 1.0
    w_balance: 1.0
    w_velocity: 0.0001
    w_posture: 0.001
    w_terminal_posture: 0.001
    w_limit: 100.0
    max_velocity: 4.0
    damping: 0.0001
```

The hysteresis is intentionally minimal:

- Enter: residual norm reaches 0.012 m.
- Continue: solve from residual plus pitch-rate feedback.
- Terminate: measured forward pitch rate peaks above 0.10 rad/s and decays to
  0.05 rad/s.
- Hold: retain the measured arm posture after the response.

After internal estop, the controller remains alive for benchmark logging but
publishes no further command. This prevents a robot fall from truncating the
evaluation process.

## Experiment Record

### Baseline strong impulse

The first +X 37.5 N fixed-arm run fell:

- Peak whole-run tilt: 2.77996 rad.
- Minimum base height: 0.09467 m.

Early controller iterations fell at both forces. The accepted peak-decay
formulation extends positive-X survival to 35 N but not 37.5 N.

### Rejected iterations

1. Initial numerical-difference DDP:
   mean solve time was 66.6 ms and the controller could not maintain 30 Hz.
2. Residual deadband at 0.002 m:
   no-force motion caused continuous arm activity.
3. Hard deadband at 0.010 m:
   no-force activity stopped, but useful feedback was also suppressed.
4. Hysteretic 0.012/0.005 m activation:
   retained because it suppresses no-force output while allowing full feedback
   after a detected residual.
5. `effect_sign: -1.0`:
   moved both arms forward during a forward fall and increased the 35 N
   impulse-window peak from 0.203 rad fixed to 0.302 rad.
6. Continuous corrected-sign ZMP feedback:
   reduced the initial peak to about 0.117 rad but drove shoulders beyond
   1 rad backward and caused a delayed backward fall.
7. Pitch-rate feedback without event termination:
   reduced the 35 N impulse peak as low as 0.083 rad but still over-corrected
   backward.
8. Proportional tilt feedback:
   converted orientation error into sustained arm velocity and destabilized
   standing unless separately gated; the gated version still regressed.
9. Raw MuJoCo motor acceleration in ZMP:
   produced 0.20 m no-force residual spikes and false activation because base
   velocity, base acceleration, and contact wrench remain absent. It was
   reverted.
10. Slow, fast, and tilt-gated returns:
    either generated the return reaction too early or failed before restoring
    posture.
11. Constant velocity bounds from 0.3 to 0.9 rad/s:
    bracketed forward and backward failure but no constant value survived.
12. Peak-decay termination:
    accepted. It uses seven Crocoddyl solves in typical 35 N passes and stops
    with shoulder pitch near +0.15 rad.

### Final no-force run

Artifact:
`runs/20260721_201643_peak_decay_final_no_force_zmp_no_force_direct_zmp`.

- Complete: true.
- Fell: false.
- Active solve count: 0.
- Maximum arm command: 0.
- Peak whole-run tilt: 0.16231 rad.

Earlier fixed-force experiments suggested a lower +X RMS response at 30 N, but
those exploratory artifacts have been removed. The threshold sweep is the
retained evidence for survivability; it does not show a threshold improvement.

### Eight-direction threshold sweep

Artifact: `runs/direct_zmp_threshold_8dir_5n_20_60_20260721`.

The final threshold experiment used eight directions, 5 N force increments,
and independent early stopping after the first failure. Lateral directions
were extended to the historical 160 N ceiling. The merged artifact contains
156 sweep, extension, and boundary-validation trials.

| Direction | Fixed-arm bracket | Direct-ZMP bracket |
| --- | ---: | ---: |
| +X | 30-35 N | 30-35 N |
| 45 degrees | 50-55 N | 50-55 N |
| +Y | 110-115 N | 110-115 N |
| 135 degrees | 30-35 N | 30-35 N |
| -X | 25-30 N | 25-30 N |
| 225 degrees | 45-50 N | 45-50 N |
| -Y | 105-110 N | 105-110 N |
| 315 degrees | 40-45 N | 40-45 N |

The initial 225-degree sweep trial suggested a 5 N direct-controller gain:
the direct controller passed 50 N once while fixed arms failed. Three paired
50 N validation repetitions caused both variants to fall in every run. With
majority aggregation, both variants have a 45-50 N bracket. The original direct
pass was an outlier.

The first +Y extension similarly suggested different 110-115 N and 115-120 N
brackets. Three paired 115 N repetitions caused both variants to fall in every
run, producing the shared 110-115 N bracket shown above.

That sweep describes the controller before the sign and peak-decay fixes. The
new positive-X experiments supersede its +X row but not the other seven
directional thresholds.

### Positive-X peak-decay validation

Artifacts:

- `runs/x35_deterministic_direction_validation_20260721`.
- `runs/x37_5_peak_decay_limit_20260721`.
- `runs/peak_decay_8dir_30n_regression_20260721`.

At 35 N, three paired validation runs give:

| Variant | Passes | Falls | Mean impulse peak |
| --- | ---: | ---: | ---: |
| Upper fixed | 0 | 3 | 0.173 rad |
| Direct ZMP | 2 | 1 | 0.121 rad |

Including the immediately preceding discovery run, direct control passes 3/4
and fixed arms pass 0/4. At 37.5 N, direct control falls in 2/2 runs. The
current empirical X brackets are therefore approximately 34-35 N for fixed
arms and 35-37.5 N for direct control.

The 30 N eight-direction regression introduces no pass/fail regression: both
variants pass every direction except negative X.

## Controlled Arm Reaction Identification

### Purpose and tooling

The `arm_probe` benchmark variant was added to identify how prescribed arm
motions affect centroidal dynamics before constructing a larger optimal-control
problem. It continuously publishes through the upper-body safety input and
releases a profile from the same `impulse.flag` used by MuJoCo.

Two profile shapes are available:

- `bump`: smooth motion to a configured displacement followed by a smooth
  return to the original posture.
- `hold`: smooth one-way motion followed by a position hold, matching the
  accepted direct controller more closely.

The MuJoCo recorder now includes:

- Full `qpos`, `qvel`, and `qacc`.
- Applied actuator controls and actuator forces.
- Whole-body COM position and velocity.
- Whole-body centroidal angular momentum.
- Left, right, and combined arm angular momentum translated to the whole-body
  COM.

Probe command, measured joint state, release tick, and profile time are written
to `arm_probe.jsonl`. Validation produced zero-step or one-step release skew and
tracked a 0.15 rad shoulder profile to approximately 0.155 rad.

Reusable commands are:

```bash
uv run python -m h12_zmp_benchmark.experiment.arm_reaction_sweep \
    --config config/arm_probe.yaml \
    --direction none \
    --force 0 \
    --profile shoulder_pitch_pos \
    --profile shoulder_pitch_neg \
    --output-dir arm_reaction_basis

uv run python -m h12_zmp_benchmark.plot.analyze_arm_reaction \
    runs/arm_reaction_basis
```

The analyzer writes raw metrics, antisymmetric reaction pairs,
baseline-subtracted directional effects, and aligned time-series plots.

### No-disturbance joint basis

The primary basis used 0.15 rad, 0.8 s closed bump profiles. Positive and
negative profiles were differenced and divided by two to suppress standing
offset and lower-policy phase variation.

| Mode | Dominant arm momentum | Base-rate reaction | Interpretation |
| --- | ---: | ---: | --- |
| Shoulder pitch | `H_y = -0.108` | `omega_y = +0.0098` | Strong pitch authority. |
| Elbow | `H_y = +0.0318` | `omega_y = -0.0003` | About 30% arm momentum and negligible measured base authority. |
| Same-sign shoulder roll | `H_x = -0.121` | `omega_x = +0.0244` | Strongest roll authority. |
| Opposed shoulder roll | `H_x = +0.0256` | `omega_x = +0.0080` | Mostly cancels and has strong coupling/noise. |

Units are kg m²/s for angular momentum and rad/s for base rate. Signs describe
the positive profile command in the current MuJoCo joint coordinates.

The map is not axis diagonal. Shoulder pitch also produced about 0.063 kg m²/s
of X angular momentum, and same-sign shoulder roll produced about
0.051 kg m²/s about Z. A future controller should use the measured full matrix,
not a hand-written planar sign swap.

### Braking and momentum payback

The second half of the closed bump reverses arm velocity. Antisymmetric braking
momentum was:

- Shoulder pitch: `H_y = +0.0466`, opposite the `-0.108` spin-up momentum.
- Elbow: `H_y = -0.0309`, nearly the full opposite of its spin-up momentum.
- Same-sign shoulder roll: `H_x = +0.0861`, about 71% of the opposite spin-up
  momentum.

The useful reaction cannot be evaluated from spin-up alone. Elbow motion is
especially unattractive because almost all of its small momentum is paid back
during braking.

One-way 0.15 rad, 0.3 s move-and-hold profiles further showed that base motion
can reverse after the arm stops even without a commanded return. For shoulder
pitch, paired spin-up base rate was `+0.0209 rad/s`, while the held-phase value
became `-0.0102 rad/s`. Servo braking and the lower-body policy therefore create
payback even when arm position remains fixed.

### Amplitude and duration scaling

Shoulder pitch paired `H_y` at 0.075, 0.15, and 0.225 rad was approximately:

```text
-0.0548, -0.1077, -0.1308 kg m^2/s
```

The response is nearly linear through 0.15 rad and then saturates. Same-sign
shoulder-roll paired `H_x` was approximately:

```text
-0.0804, -0.1214, -0.2371 kg m^2/s
```

Actual peak shoulder velocity increased from approximately 0.27 to 0.56 and
0.82 rad/s across the amplitude sweep. The 0.4 s shoulder profile reached the
roughly 0.9 rad/s safety boundary. A 1.2 s profile produced similar sampled arm
momentum but almost no paired base-rate reaction (`0.0002 rad/s`), showing that
the lower policy absorbs slow motions. Useful control authority depends on
momentum rate and timing, not only arm momentum or displacement.

Artifacts:

- `runs/arm_reaction_basis_20260722`.
- `runs/arm_reaction_scale_0075_20260722`.
- `runs/arm_reaction_scale_0225_20260722`.
- `runs/arm_reaction_duration_04_20260722`.
- `runs/arm_reaction_duration_12_20260722`.
- `runs/arm_reaction_hold_basis_20260722`.

### Four-direction closed-bump experiments

At 20 N, baseline, predicted matching sign, opposite sign, and a weaker or
canceling joint mode were tested in all four cardinal directions.

| Direction | Baseline peak | Best tested profile | Profile peak | Opposite or weak profile |
| --- | ---: | --- | ---: | ---: |
| +X | 0.0897 | Elbow positive | 0.0888 | Shoulder pitch positive: 0.0931 |
| -X | 0.1229 | Shoulder pitch negative | 0.1207 | Shoulder pitch positive: 0.1228 |
| +Y | 0.0641 | Same-sign roll positive | 0.0565 | Same-sign roll negative: 0.0677 |
| -Y | 0.0506 | Same-sign roll negative | 0.0487 | Same-sign roll positive: 0.0550 |

The X differences at 20 N are too small to separate from run variation. The
lateral sign effect is clearer. At 60 N:

| Direction | Baseline | Roll positive | Roll negative |
| --- | ---: | ---: | ---: |
| +Y | 0.1054 | 0.0957 | 0.1062 |
| -Y | 0.0682 | 0.0717 | 0.0675 |

Positive roll reduced +Y roll angle by about 10.5%. Negative roll provided only
a small -Y improvement. These are single-trial system-identification results,
not threshold claims.

Artifacts:

- `runs/arm_reaction_directional_20260722`.
- `runs/arm_reaction_lateral_60n_20260722`.

### Four-direction one-way experiments

The one-way profile gives a substantially different result because it removes
the explicit return half of the bump.

| Direction and force | Baseline peak | Positive profile | Negative profile |
| --- | ---: | ---: | ---: |
| +X, 35 N shoulder pitch | 0.2122, fall | 0.0949, pass | 0.3730, fall |
| -X, 25 N shoulder pitch | 0.1386 | 0.1406 | 0.1383 |
| +Y, 60 N same roll | 0.1046 | 0.0904 | 0.1116 |
| -Y, 60 N same roll | 0.0684 | 0.0596 | 0.0795 |

For +X, the positive shoulder-pitch hold profile changed failure to survival,
matching the sign used by the accepted direct controller. The isolated
spin-phase base-rate sign alone would have predicted the opposite profile.
The time series show that positive shoulder pitch arrests divergence after the
force interval, reduces long-term COM excursion, and avoids explicit braking
payback. Therefore the balance benefit is a closed-loop trajectory effect, not
an instantaneous reaction-sign effect.

Both +Y and -Y preferred the same positive shoulder-roll profile in this
posture and with this lower-body policy. The system is not laterally symmetric;
mass redistribution and policy coupling are as important as arm angular
momentum sign.

Artifact: `runs/arm_reaction_hold_directional_20260722`.

### Current direct-controller trigger behavior

The retained 30 N direct-controller logs show:

| Direction | Crocoddyl solves | Maximum absolute ZMP residual | Main arm motion |
| --- | ---: | ---: | --- |
| +X | 6 | X: 0.0232 m | Shoulder pitch: about 0.15 rad. |
| +Y | 19 | X: 0.0120 m, Y: 0.0115 m | Shoulder pitch: about 0.35 rad. |
| -X | 32 | X: 0.0645 m | Shoulder pitch: about 0.86 rad. |
| -Y | 19 | X: 0.0123 m, Y: 0.0116 m | Shoulder pitch: about 0.35 rad. |

The Y pushes do not produce a clean Y trigger. Instead, X residual noise reaches
the 0.012 m threshold and starts pitch-oriented arm motion. The current
controller is therefore not a valid planar directional controller even though
the 30 N pass/fail regression is neutral.

### Implications for the next controller

1. Use measured arm-to-centroid reaction matrices:
   shoulder pitch for the pitch subspace and same-sign shoulder roll for the
   roll subspace. Elbow and opposed roll should have low allocation priority.
2. Optimize momentum rate and body state over time:
   instantaneous arm momentum sign does not predict whole-response benefit.
3. Represent spin-up, servo braking, held posture, and optional return as
   separate phases or within a short horizon.
4. Include arm posture and remaining stroke:
   held arm position changes COM and lower-policy behavior.
5. Use a directional trigger:
   classify X and Y independently from a better COP/IMU/COM state instead of
   triggering on the norm of a noisy residual.
6. Include coupled objectives:
   roll modes generate yaw and pitch momentum, and pitch modes generate roll
   momentum in the current geometry.
7. Validate a future model against measured outputs:
   predicted arm momentum, base angular acceleration, COM change, and braking
   payback should all be checked after safety clipping.

The next Crocoddyl state should include at least arm position and velocity,
body roll and pitch, body angular velocity, COM position and velocity, and arm
angular momentum. Arm acceleration is a more appropriate control than arm
velocity for modeling the measured reaction.

## Benchmark Metrics

`summary.json` now includes:

- `max_base_tilt`: maximum over the complete run.
- `max_impulse_tilt`: maximum from force onset through one second after force
  release.
- `impulse_tilt_delta`: impulse-window maximum minus the preceding one-second
  mean tilt.
- `rms_impulse_tilt`: RMS tilt over the same impulse window.
- `zmp.active_count`: ticks with an applied direct-controller solution.
- `zmp.max_arm_velocity`: largest requested arm velocity.
- `zmp.max_zmp_residual`: largest calibrated residual norm.
- `zmp.mean_solve_time` and `zmp.max_solve_time`: Crocoddyl runtime.

Impulse-window metrics are required for moderate forces because whole-run peak
tilt is often set by unrelated standing oscillation.

## Known Limitations

- Floating-base linear and angular velocity are omitted by
  `RobotModel.full_v()`, reducing COM velocity and ZMP fidelity.
- ZMP uses motor acceleration and a two-ankle support plane without contact
  confidence or measured contact wrench.
- The Y-direction residual remains below the no-force separation threshold at
  30 N.
- The reduced two-arm model freezes lower-body and torso motion.
- The local derivatives ignore the derivative of the centroidal momentum map
  with respect to arm configuration.
- Arm collision and end-effector constraints are absent.
- A one-step model improves RMS settling but not peak response.
- No manipulation-task arbitration exists; this controller owns both arms.
- The strong-force abort is an operating-region guard, not recovery behavior.

## Next Small Experiments

The next iteration should remain incremental:

1. Add measured IMU angular velocity as an independently weighted balance
   input, starting with Y-direction observability tests and no arm output.
2. Log commanded and measured arm momentum to identify command delay and gain
   before adding another Crocoddyl state or loss.
3. Add an acceleration bound or control-rate cost and evaluate peak tilt versus
   RMS tilt. Do not add both in one experiment.
4. Replace the frozen reduced momentum map with the current full-body arm
   columns while retaining the one-step problem.
5. Add contact-confidence gating before enabling any real-robot experiment.
6. Test one reserved arm only after both-arm behavior is repeatable.

Each parameter or loss change must include, at minimum:

- One no-force run.
- Three paired runs in the target direction and force.
- One cardinal sweep at the selected force.
- Impulse-window peak, delta, RMS, fall, command, and solve-time reporting.
