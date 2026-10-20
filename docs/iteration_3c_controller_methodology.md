# Iteration 3C Controller Methodology

## Scope and Status

Iteration 3C is a reactive counter-arm velocity controller. It preserves a
prescribed manipulation-arm command while choosing four velocities on the
opposite arm to trade off instantaneous center-of-mass (CoM) motion,
centroidal angular momentum, posture return, and damping. Each tick solves a
convex box-constrained least-squares problem using Crocoddyl BoxFDDP followed
by BoxQP. It is not a multi-step predictive balance controller.

This is the consolidated methodology reference for the implemented
`counter_ddp_velocity_wide` benchmark variant and a **proposed, unimplemented
3C-robust** extension. The latter would change numerical fallback only, not
the nominal control law. No controller code, registered robust-3C variant,
or robust-3C experimental results are introduced by this document.

The source baseline is controller revision
`47c0807a35644951a5950f009f66d5ea62fffa00` and benchmark revision
`f87a2509fd90d7f34f97296c4911c23648917a17`. Source code takes precedence over
historical design prose. All blocks labeled **current source** are literal
excerpts, with enclosing indentation removed where appropriate. The proposed
algorithm is separately labeled and is not an existing API.

## Source Map

Links are relative to this document. Benchmark links traverse to the enclosing
`h12_zmp_benchmark` checkout; they require that workspace layout. Line numbers
refer to the revisions above, not immutable remote permalinks.

| Responsibility | Source and useful lines |
| --- | --- |
| State, ownership, planning, finalization | [CounterBalanceController][base], 120-317, 327-404 |
| Crocoddyl adapter and diagnostics | [CounterDDPVelocityController][ddp], 18-135 |
| Non-publishing planner and immutable output arrays | [Frozen 3C planner][planner], 11-137 |
| Targets, normalized objective, SciPy solve | [Objective][objective], 9-90 |
| One-knot problem, BoxQP, acceptance | [Velocity OCP][ocp], 28-211 |
| Existing robust H2 nominal fallback | [Robust H2 controller][robust-h2], 35-81 |
| Geometric support rectangle | [Support region][support], 41-130 |
| Joint names and motor indices | [Joint definitions][joints], 100-189 |
| Frequency and inherited steady state | [Upper controller][upper], 29-38, 461-547 |
| Model limits and collision queries | [Robot model][robot], 392-421 |
| Atomic command storage, clips, estop monitor | [Low-command handler][publisher], 117-142, 214-235 |
| Processed configuration and frequencies | [Config loader][loader], 156-200, 273-281 |
| Wide gains and fast profile | [Benchmark hard-group configuration][wide-config], 31-52, 106-114 |
| Variant dispatch, execution, lifecycle | [Benchmark runtime][runtime], 63-98, 142-242, 407-524 |
| Simulation-specific gains and safety limits | [Simulation safety configuration][sim-config], 59-118 |

## Observations and Ownership

### Real-Compatible Inputs

The observation contract permits IMU, joint position, joint velocity, joint
torque, and quantities derived from those observations with Pinocchio.
Simulator contact forces, ground-truth contact flags, external simulator
wrenches, and exact simulator base pose or velocity are forbidden as control
inputs. Simulator outcomes may be used for offline evaluation, not fed into
this control law. Policy, checkpoint, and catalog target identities do not
enter the nominal objective; the commanded trajectory and arm side do.

The actual nominal path uses measured motor positions, commanded moving-arm
velocities, model-derived maps, and IMU angular rate. Measured velocities are
available but do not enter a counter-acceleration state or the nominal
feedforward target. Joint torque belongs to the safety/observation interface,
not to a nominal objective term. Missing or malformed motor positions fall
back to the stored position command; remaining nonfinite positions become
zero. Invalid velocities become zero. This sanitation is not an observation
freshness guarantee.

`_model_terms()` builds a motor-only configuration for `model_body`, uses
private Pinocchio data, and recomputes forward kinematics, CoM, its Jacobian,
and the centroidal map at the current measured configuration. These are
fixed-base body-model quantities, not exact inertial-world simulator state.
The angular momentum map is `data.Ag[3:, :]`; its first two rows and the CoM
Jacobian's first two rows are used. The torso gyroscope is rotated into this
model frame by the current `torso_link` rotation. Missing usable gyro data
gives a zero vector and `gyro_available = false`, not an automatic hold.

**Current source**, [body-model terms][base], lines 731-736:

```python
com = np.asarray(pin.centerOfMass(model, data, body_q))
com_jacobian = np.asarray(
    pin.jacobianCenterOfMass(model, data, body_q)
)
pin.computeCentroidalMap(model, data, body_q)
momentum_map = np.asarray(data.Ag[3:, :])
```

The support reference is an oriented rectangle enclosing two modeled sole
rectangles at `left_ankle_roll_link` and `right_ankle_roll_link`. Its axes come
from the projected foot-forward directions; invalid transforms, degenerate
geometry, or excessive yaw divergence invalidate it. It does not identify
which feet actually contact the ground, enforce a contact wrench cone, or
constrain ZMP. In 3C, its center supplies a CoM reference, not a hard support
margin constraint.

### Seven Moving Joints and Four Counter DOFs

The 14-arm vector is ordered as seven left-arm joints then seven right-arm
joints. Each side uses shoulder pitch, shoulder roll, shoulder yaw, elbow,
wrist roll, wrist pitch, and wrist yaw. Left motor IDs are `13..19`; right
motor IDs are `20..26`. Pinocchio configuration and velocity indices are
looked up by joint name, rather than assumed equal to motor indices.

The moving arm follows all seven supplied position and velocity components.
Only the opposite shoulder pitch, shoulder roll, shoulder yaw, and elbow
are optimized. Thus the decision is always a four-vector
\(u=\dot q_c\in\mathbb{R}^4\). The three counter wrists hold their captured
positions with zero commanded velocity. Legs and torso are not overwritten
by the 14-joint overlay.

**Current source**, [ownership][base], lines 131-138:

```python
self.counter_ids = self.counter_arm_ids[:4]
self.counter_wrist_ids = self.counter_arm_ids[4:]
self.moving_local = self._arm_local_indices(self.moving_ids)
self.counter_local = self._arm_local_indices(self.counter_arm_ids)
self.counter_active_local = self.counter_local[:4]
self.counter_wrist_local = self.counter_local[4:]
self.moving_v_indices = self.motor_v_indices[self.moving_ids]
self.counter_v_indices = self.motor_v_indices[self.counter_ids]
```

Ownership may be explicit, inferred from a frame that depends on exactly one
arm, or inferred from a named target that changes exactly one arm relative to
`home`. Changing ownership resets references and activation state.

## References and Activation

At the first valid control sample, reference capture stores measured active
counter posture \(q_{c,ref}\), counter wrist positions, IMU roll/pitch, and
the planar CoM offset from the support center:

\[
d_{ref}=c_{xy,0}-s_0,\qquad
c_{target,k}=s_k+d_{ref},\qquad
e_{c,k}=c_{xy,k}-c_{target,k}.
\]

The controller therefore preserves the initial CoM offset as the modeled
support center moves; it does not force CoM to the geometric center or use
`home` as the counter-posture reference. Capture completes only with valid
support and finite CoM. Until then, the counter arm holds.

Let \(b=b_{caller}a_{tilt}\) be the effective `balance_scale`. The caller
scale must be finite and nonnegative, but the controller does not cap it at
one. Tilt activation is one when its threshold is nonpositive. Otherwise it
linearly ramps, with clipping to `[0, 1]`, using the Euclidean norm of the
roll/pitch change from capture between `tilt_threshold` and
`tilt_full_scale`. Optional latching takes the maximum activation so far.
The wide configuration omits activation, so its default threshold is zero
and tilt activation is always one.

The benchmark caller supplies one through motion and an optional reactive
hold, then linearly fades to zero. Defaults are `reactive_hold_duration = 0`
and `reactive_fade_duration = 0.5 s`. These are lifecycle settings, not a
learned authority selector. A zero scale removes balance costs but leaves
posture return and damping active; it does not mean zero counter velocity.

## Nominal Control Law

### Instantaneous Targets

For \(\dot q_m\in\mathbb{R}^7\), define the planar CoM Jacobian blocks
\(J_m\in\mathbb{R}^{2\times7}\), \(J_c\in\mathbb{R}^{2\times4}\),
and planar angular momentum blocks \(A_m\), \(A_c\) of the same sizes.
Let \(\omega\in\mathbb{R}^2\) contain the model-frame gyro's first two
components. At each tick the targets are

\[
r_c=b(-J_m\dot q_m-k_c e_c),\qquad
r_h=b(-A_m\dot q_m+k_g\omega),\qquad
r_q=-k_q(q_c-q_{c,ref}).
\]

**Current source**, [reaction targets][objective], lines 12-18:

```python
com_rhs = balance_scale * (
    -com_moving @ moving_dq - com_gain * com_error
)
momentum_rhs = balance_scale * (
    -momentum_moving @ moving_dq + gyro_gain * gyro[:2]
)
return com_rhs, momentum_rhs
```

The positive gyro sign is intentional and must not be silently changed to
negative rate feedback. The momentum term requests counter-arm angular
momentum; it is not a directly commanded body angular acceleration or a
measurement of total momentum cancellation. Only arm columns appear in the
objective, although the maps and CoM use the live full body configuration.

The [pure planner][planner] receives copied inputs, computes targets, invokes
an injected solve callback, and returns copied read-only arrays. It does not
publish, capture references, integrate positions, or commit command state.
Its two RHS-offset fields are added after the scaled targets are built.
Both offsets are exactly zero in `CounterBalanceController` and inherited
3C; they are extension points, not hidden nominal residual terms.

### Normalization and Exact Scale Placement

Let \(s_c,s_h,s_q>0\) be the configured CoM-velocity, momentum, and
posture-velocity scales. The solver minimizes

\[
\begin{aligned}
\min_{\ell\le u\le v}\quad f(u)=\frac12\bigg[
&b w_c\left\|\frac{J_cu-r_c}{s_c}\right\|_2^2
+b w_h\left\|\frac{A_cu-r_h}{s_h}\right\|_2^2\\
&+w_q\left\|\frac{u-r_q}{s_q}\right\|_2^2
+\lambda\left\|\frac{u}{s_q}\right\|_2^2
\bigg].
\end{aligned}
\]

Here \(v\) denotes the upper velocity bound, not measured velocity. The
implementation stacks the square-root-weighted rows into \(M\) and target
\(y\), giving \(f(u)=\tfrac12\|Mu-y\|^2\). Zero-weight terms are omitted.
With all terms active, \(M\) has shape `12 x 4`. Crucially, \(b\) appears
both inside \(r_c,r_h\) and in the two balance weights. It does not multiply
the counter maps inside the residual, posture terms, damping, or the final
velocity as a separate post-solve scaling. Moving it outside the solve or
scaling both sides of the residual would change the implemented law.

**Current source**, [objective assembly][objective], lines 61-82:

```python
terms = (
    (
        balance_scale * com_weight,
        com_counter / com_velocity_scale,
        com_rhs / com_velocity_scale,
    ),
    (
        balance_scale * momentum_weight,
        momentum_counter / momentum_scale,
        momentum_rhs / momentum_scale,
    ),
    (
        posture_weight,
        np.eye(4) / posture_velocity_scale,
        posture_target / posture_velocity_scale,
    ),
    (
        damping,
        np.eye(4) / posture_velocity_scale,
        np.zeros(4),
    ),
)
```

CoM velocity is in `m/s`, angular momentum in `kg m^2/s`, joint velocity in
`rad/s`, and joint position in `rad`. The gains convert their corresponding
errors to these targets; the gyro gain is not dimensionless when physical
units are retained. Positive damping contributes
\(\lambda I/s_q^2\) to \(M^TM\), making the nominal quadratic strictly
convex. For finite valid inputs and a nonempty box, a unique minimizer exists.
The balance equations are soft objectives: inability to cancel both exactly
is not infeasibility.

## Bounds and Numerical Solve

### Effective Bounds

Let \(q^-\), \(q^+\) intersect model position limits with optional processed
publisher position clips, and let \(e^{max}\) denote maximum reference
excursion. For each active joint,

\[
\begin{aligned}
\bar u_i&=\min(u^{model}_i,\;dq_{lim},\;u^{publisher}_i,
                 \;u^{max}_i),\\
\ell_i&=\max\left(-\bar u_i,
             \frac{q^-_i-q_{c,i}}{dt},
             \frac{q_{c,ref,i}-e^{max}_i-q_{c,i}}{dt}\right),\\
v_i&=\min\left(\bar u_i,
             \frac{q^+_i-q_{c,i}}{dt},
             \frac{q_{c,ref,i}+e^{max}_i-q_{c,i}}{dt}\right).
\end{aligned}
\]

Absent optional publisher limits do not add a bound. These constraints use
measured counter position, not the previous integrated command. An empty
interval raises `CounterVelocityBoundsError` in the solve path and causes
`counter_bounds_infeasible`. Invalid limit configuration can instead raise
before the planner's exception handler and reach the caller's error path.

There is no acceleration, velocity-change, jerk, braking-distance, or torque
constraint in this 3C optimization. In particular, `counter_ddp` acceleration
settings in a shared sweep file belong to another controller and do not
constrain 3C. Publisher position/velocity/torque clipping and measured-state
estop monitoring remain separate from the optimization.

### One Running Knot, Not Predictive MPC

The OCP has a four-dimensional `StateVector`, one running action, and a
zero-cost terminal action. Its formal dynamics are
\(x_1=x_0+dt\,u\). The adapter actually calls `solve()` with
`[0.0, 0.0, 0.0, 0.0]` as the initial state, not measured counter posture.
Because the running cost depends only on \(u\), and there is no terminal
cost or state constraint, this state is bookkeeping. Measured posture has
already entered the targets and bounds.

**Current source**, [action model][ocp], lines 58-62:

```python
def calc(self, data, x, u=None):
    u = np.asarray(u, dtype=np.float64)
    data.xnext[:] = x + self.dt * u
    residual = self.matrix @ u - self.target
    data.cost = 0.5 * float(residual @ residual)
```

The analytic derivatives are \(F_x=I\), \(F_u=dt I\),
\(L_u=M^T(Mu-y)\), \(L_{uu}=M^TM\); state cost derivatives vanish.
There is no `dt` multiplier on the cost. The maps are frozen for this solve
and rebuilt next tick. No future manipulation samples, base-response model,
contact dynamics, or predicted tilt trajectory enter this problem. Calling
the numerical backend DDP does not make the control law horizon-predictive.

The implemented numerical sequence is:

1. Validate shapes and finite objective/bound inputs. Bounds classified by
    `np.isclose(lower, upper, atol=1e-12)` are expanded by `1e-10` on each
    side internally. NumPy's default relative tolerance is retained.
2. Construct a fresh `SolverBoxFDDP`, set `th_stop = 1e-9`, seed a single
    control with clipped zero, and roll out initial states. Allow up to
    `100` iterations. Try initial regularizations `1e-9`, `1e-6`, and `1e-3`
    only until a solver call returns without an exception. A returned
    non-converged solve does not trigger the next regularization.
3. Always run `crocoddyl.BoxQP(4, 100, 0.1, 1e-9, 1e-9)` on Hessian
    \(M^TM\) and linear term \(-M^Ty\), using the BoxFDDP output or clipped
    zero if all BoxFDDP attempts threw. A BoxQP exception propagates.
4. Clip the polished velocity back to the original bounds. Recompute cost,
    gradient, and the custom projected-gradient KKT violation. Accept or
    reject the returned result independently of BoxFDDP convergence.

**Current source**, [acceptance gate][ocp], lines 176-184:

```python
accepted = bool(
    velocity.shape == (NQ,)
    and np.all(np.isfinite(velocity))
    and np.isfinite(cost)
    and np.isfinite(kkt_violation)
    and kkt_violation <= KKT_TOLERANCE
    and np.all(velocity >= np.asarray(lower) - 1e-8)
    and np.all(velocity <= np.asarray(upper) + 1e-8)
)
```

`KKT_TOLERANCE = 5e-4`. With \(g=M^T(Mu-y)\), the custom violation starts
as \(|g_i|\), uses \(\max(-g_i,0)\) near a lower bound, then
\(\max(g_i,0)\) near an upper bound, and zero for coordinates classified
fixed. Proximity uses `2e-4`; fixed classification again uses `np.isclose`
with that absolute tolerance and NumPy's default relative tolerance. The
reported violation is the maximum coordinate value. This is the actual
tolerance-based implementation, not an exact symbolic KKT certificate.

A rejected result is a numerical validation failure, not proof of an empty
box. Conversely, `converged = true` alone does not establish acceptance.
Current 3C has no outer SciPy retry; rejection or solver exceptions select
the shared counter hold.

## Motion Execution and Publication

### Ordered Tick

The direct `control_configuration_step()` pipeline is:

1. Require ownership, validate the caller scale, reset tick diagnostics, and
    return zero without updating the model or publishing if estopped.
2. Require position and velocity samples of shape `(14,)`. Copy only the
    moving-arm entries into a measured-arm pose and zero-velocity template;
    nonfinite moving-arm commands raise before publication.
3. Read motor state, update the model, then reread motor state and rebuild
    the sample. Compute model terms from this prepared motor position.
4. Capture references if necessary; hold the counter arm if support or
    reference capture is invalid. Compute activation, CoM error, rotated
    gyro, the arm map blocks, zero nominal offsets, and bounds.
5. Build one `Frozen3CNominalInput`, solve the pure plan, and explicitly
    commit its solver diagnostics. Hold on returned rejection. Otherwise
    select exactly the nominal velocity; 3C adds no residual.
6. Validate the selected four-vector, backtrack the complete candidate, and
    finalize once. Integrate counter position from the measured posture,
    compute gravity compensation, and write one combined arm command.

The prescribed moving-arm velocity, not measured moving velocity, drives
feedforward cancellation. The full 27-joint collision candidate combines
current measured legs/torso, prescribed moving-arm positions, and proposed
counter-arm positions. The motion command is prepared before solving the
counter motion; the overlay does not first publish the moving arm and then
publish a second counter correction.

### Backtracking and Holds

**Current source**, [counter backtracking][base], lines 946-954:

```python
q_delta = self.dt * requested
for scale in self.backtrack_scales:
    candidate_q = np.copy(arm_q)
    command_dq = np.copy(arm_dq)
    candidate_q[self.counter_active_local] = counter_q + scale * q_delta
    candidate_q[self.counter_wrist_local] = self.counter_wrist_ref
    command_dq[self.counter_active_local] = scale * requested
    command_dq[self.counter_wrist_local] = 0.0
    full_candidate = self._full_candidate(motor_q, candidate_q)
```

The scales are `(1.0, 0.5, 0.25, 0.125, 0.0)`. Each endpoint must be finite,
within robot-model position limits, and collision-free. The first valid
candidate wins; a check exception stops the search. This is endpoint
validation, not continuous swept-path collision certification. The named
`collision_backtracked` status and `collision_rejection` flag also cover
reductions caused by position-limit rejection.

Backtracking scales only the counter motion and does not re-solve the QP.
It also does not recheck the original velocity/excursion box. In particular,
if zero is outside that box because the measured posture requires recovery,
scaling toward zero need not preserve those bounds. The endpoint model-limit
check is distinct from the processed publisher limits already used in the
QP. These details limit any claim of recursive constraint satisfaction.

If no candidate succeeds, or model update, model terms, support, or the
nominal solve fails in its guarded region, `_publish_counter_hold()` keeps
the moving-arm command, holds measured active counter positions, fixes the
wrists to their captured reference when available, and sets all counter
velocities to zero. A hold does not rerun the candidate collision checker
and is not a guarantee that the moving-arm target is safe. Estop is the
separate no-publication path. Malformed caller input and some preparation
or publication errors propagate; the benchmark's outer handler can then
publish an all-arm measured-position hold unless estopped.

### Gravity and Command Semantics

The finalizer obtains gravity compensation at measured full motor position,
selects the 14 arm torques, and calls `set_joint_commands(q, dq, tau,
joint_ids=arm_ids)`. The handler atomically updates command arrays under its
publisher lock, clips position, velocity, and torque, and the background
publisher transmits those arrays. One successful direct overlay tick makes
one such write, not necessarily one network packet. The default control,
publication, and safety-check rates are `50`, `500`, and `1000 Hz`.

Gravity failure replaces the entire counter arm with measured positions and
zero velocities, preserves the moving-arm position/velocity command, and
uses the previous finite 27-joint torque command's arm entries or zero
torque. This is fallback feedforward, not newly computed gravity. An estop
detected before or during writing suppresses publication. Other publisher
exceptions propagate. The background safety monitor checks measured upper
joint position, velocity, and torque against separate estop limits.

Inherited frame tracking routes through `_publish_position_command()` into
the same overlay. The benchmark normally uses direct saved-trajectory
samples; after motion, reactive hold, and fade, it can enter inherited
steady-state I control. On transition it calls the direct path before
initializing and stepping steady state, so that outer loop iteration can
contain two overlay calls. Moving-arm joints alone determine I-stage
clipping/convergence. Importantly, the overlay ignores the inherited `tau`
argument and recomputes gravity: although the parent accumulates an integral
torque bias, that bias is not forwarded by this override. Do not describe
the 3C overlay as passing through arbitrary input torque or guaranteeing
unclipped moving-arm commands.

## Wide Configuration and Reproducibility

The runtime maps `counter_ddp_velocity_wide` to
`CounterDDPVelocityController`. The name alone does not load wide gains.
The representative [hard-group sweep][wide-config] supplies the following
`reactive_counter_balance` values; remaining entries below come from class
or frequency defaults, not the unrelated `counter_ddp` block.

| Parameter | Effective reference value | Provenance |
| --- | --- | --- |
| `weights.com`, `momentum`, `posture` | `1.0`, `2.0`, `0.02` | Wide sweep. |
| `gains.com`, `gyro`, `posture` | `2.0`, `0.2`, `1.0` | Wide sweep. |
| `damping` | `0.0001` | Wide sweep. |
| `max_velocity` | `[2.6, 3.2, 2.6, 1.5] rad/s` | Sweep; intersected. |
| `max_excursion` | `[0.75, 0.58, 0.40, 0.58] rad` | Wide sweep. |
| `com_velocity_scale` | `0.1 m/s` | Class default. |
| `momentum_scale` | `1.0 kg m^2/s` | Class default. |
| `posture_velocity_scale` | `1.0 rad/s` | Class default. |
| Tilt threshold/full scale; latch | `0.0`, `0.0`; `false` | Defaults. |
| Sole front/rear/half width | `0.174`, `0.086`, `0.043 m` | Class defaults. |
| Foot yaw divergence | `deg2rad(20.0)` | Class default. |
| `frequency.ctrl_hz`; `dt` | `50 Hz`; `0.02 s` | Defaults. |
| Fast `dq_lim`, `v_lim`, `w_lim` | `6.0`, `3.0`, `8.0` | Profile overrides. |
| Fast-profile gain scales | `kd: 1.5`, `ki: 1.0` | Runtime profile overrides. |

The vector ordering is shoulder pitch, shoulder roll, shoulder yaw, elbow
on whichever arm is the counter arm. The default `dq_lim` without the fast
override is `1.0 rad/s`, which can substantially reduce the advertised wide
velocity limits. The class alone also defaults to momentum weight `1.0`,
posture weight `0.05`, gyro gain `0.1`, posture gain `0.5`, maximum velocity
`1.0`, and unbounded excursion. Instantiate with the resolved wide
configuration, not just the class name, to reproduce this methodology.

`dt` is derived from `frequency.ctrl_hz` and passed explicitly into the
velocity OCP. The constructor's nominal `0.02` default is therefore not the
only authority. The runtime sleeps for the remaining tick budget but the
integration and bounds use configured `dt`, not measured elapsed time.
The `adaptive_authority.scheduler.dt` and `counter_ddp.horizon_steps` in the
same sweep do not configure 3C. Likewise, `arm_target.support_geometry` is
used by benchmark support reporting; 3C reads support geometry from
`reactive_counter_balance.support_geometry`. The near-equal yaw defaults
must not be assumed to be one shared setting.

The reference fast profile uses `sim_safety_split.yaml`, with deliberately
relaxed simulation estop settings. Its raw clip policy uses position offset
`0.001`, velocity ratio `3.0`, and torque ratio `3.0`; the loader derives
motor-wise arrays. These values are not a hardware safety recommendation.
Effective limits also depend on the selected URDF and collision assets.

For reproducibility, archive the resolved controller configuration, selected
profile and all overlays, controller and benchmark revisions, model/asset
revisions, dependency lock/environment, both arm ownership cases, capture
state, exact trajectory and lifecycle durations, lower-body policy and
gains, safety relay settings, and actual timing. Keep target geometry,
initialization, classifier, and operational-completion criteria matched in
comparisons. Record simulator metrics separately from permitted inputs.
Neither solver acceptance nor a valid support rectangle proves whole-body
stability or hardware safety.

## Diagnostics and Interpretation

The shared diagnostics report status, arm ownership, effective balance and
activation scales, requested/applied counter velocity, backtrack scale,
support validity/reason, CoM/target/error, gyro availability, moving command
errors, estop, and publication. The DDP adapter adds availability,
convergence, cost, solve time, iterations, stopping criterion, KKT violation,
regularization, BoxQP polishing, and total direct-controller time. Benchmark
logs prefix these fields with `counter_ddp_velocity_`.

The predicted normalized residuals are

\[
\rho_c=(J_c u_{bt}-r_c)/s_c,\qquad
\rho_h=(A_c u_{bt}-r_h)/s_h,
\]

where \(u_{bt}\) is the backtracked request. They are instantaneous model
residuals, not predicted future body trajectories, measured ZMP errors, or
weighted cost contributions. They are computed before publication. The
`applied_counter_dq` field is subsequently reread from the publisher's
clipped velocity array, but these residuals are not recomputed after that
clipping. Here "applied" denotes stored command, not measured motor response.
The returned 14-vector is also the pre-publisher-clipping velocity command.

Moving command errors compare stored publisher commands with the overlay
input, not measured tracking error. `clipped` is reset false in this base
path and is not updated to detect all publisher clipping; it is not a
complete saturation metric. Cost and KKT describe the requested nominal
solve, not the collision-reduced or gravity-fallback command. Solver time
excludes model preparation/finalization, while total controller time still
excludes surrounding runtime logging and transport latency.

## Proposed 3C-Robust

### Existing Robust H2 Precedent

The existing `CounterResidualH2RobustController` is evidence that a nominal
solver fallback fits this architecture. It is **not** an implementation of
3C-robust. H2 adds a separate delay-aware residual controller after the 3C
nominal plan; robust H2 retains that residual path. Proposed 3C-robust would
retain only the nominal 3C law, with no H2 residual, response model, horizon,
confidence gate, or trust region.

In [robust H2][robust-h2], `_isolated_velocity_solve()` first explicitly calls
`CounterDDPVelocityController._isolated_velocity_solve()` with the same
maps, targets, bounds, and scale. It stores primary diagnostics and returns
the exact primary object if accepted. Only a returned `accepted = false`
result reaches the following code.

**Current source**, [robust H2 fallback][robust-h2], lines 49-70:

```python
self.latest_nominal_primary_result = primary.diagnostics
if primary.accepted:
    return primary
requested = CounterBalanceController._solve_bounded_velocity(
    self,
    com_counter,
    momentum_counter,
    com_rhs,
    momentum_rhs,
    posture_target,
    lower,
    upper,
    balance_scale=balance_scale,
)
self.latest_nominal_fallback_used = True
return Frozen3CVelocitySolve(
    requested_counter_dq=requested,
    accepted=True,
    diagnostics=primary.diagnostics,
    objective_matrix=primary.objective_matrix,
    objective_target=primary.objective_target,
)
```

That explicit base-class call reaches [the SciPy objective helper][objective],
rebuilding the same normalized matrix/target with the same inputs and
controller constants. The helper checks finite inputs and empty bounds,
calls `scipy.optimize.lsq_linear` with its defaults, raises on unsuccessful
status, and returns `result.x` on success.

**Current source**, [SciPy status handling][objective], lines 42-50:

```python
values = (matrix, target, lower, upper)
if not all(np.all(np.isfinite(value)) for value in values):
    raise ValueError('Least-squares input is nonfinite')
if np.any(lower > upper):
    raise CounterVelocityBoundsError('Counter velocity bounds are empty')
result = lsq_linear(matrix, target, bounds=(lower, upper))
if not result.success:
    raise RuntimeError('Bounded least-squares solve failed')
return np.asarray(result.x, dtype=np.float64)
```

Successful SciPy output is wrapped with `accepted=True` **without rerunning
the primary custom KKT, finite-cost, and bound acceptance gate**. Later
shared selected-command validation still checks shape/finiteness and the
normal finalizer still runs. These are not equivalent to the primary gate.
Primary cost/KKT/timing diagnostics are retained even when SciPy supplies
the command, so they must not be read as fallback quality metrics.
`nominal_fallback_used` becomes true only after SciPy returns successfully;
`nominal_primary_accepted` reports the primary state. Both associated tick
states reset at each call.

The robust H2 override does not catch primary exceptions. Such exceptions
do not trigger SciPy. Fallback exceptions and unsuccessful SciPy status
also reach the existing outer hold path. SciPy requires strictly ordered
bounds, so an exactly fixed interval supported by the Crocoddyl handling
can still cause this fallback to raise. Numerical failure does not thereby
establish that the original non-strict box is infeasible.

### Proposed Nominal-Only Retry Contract

The proposed extension would subclass the 3C adapter, not the H2 controller,
and alter only its isolated solve behavior. Its required invariants are:

- Preserve the exact primary result and command whenever primary acceptance
    succeeds; do not solve again, blend, rescale, or change future seeds.
- Retry only returned primary rejection, matching the existing precedent.
    Retain primary-exception hold behavior unless a separately reviewed
    design explicitly expands this contract.
- Reuse exactly the same \(M,y,\ell,v\), or reconstruct them from identical
    inputs and constants. Do not change gains, `balance_scale`, posture,
    damping, bounds, references, or lifecycle authority.
- Require SciPy success and then apply the same shape, finite-value,
    finite-cost, original-bound, and custom KKT gate as the primary.
    This additional fallback validation is **proposed**, not implemented
    in current robust H2 or 3C.
- Pass one accepted nominal request through the unchanged selector,
    integration, backtracking, gravity, and publication path. Otherwise
    retain the existing hold; never introduce a second publication.
- Keep primary and fallback status, exception, timing, cost, and KKT records
    distinguishable. Record fallback attempted separately from accepted.

**Proposed illustrative pseudocode**, not current source or callable APIs:

```text
primary = run_unchanged_3c_primary(inputs)
if primary.accepted:
    return primary

record_fallback_attempt(primary.diagnostics)
fallback = scipy_solve_same_quadratic_and_original_box(inputs)
if not fallback.success:
    return rejected_plan_for_existing_hold()
if not same_primary_finite_bound_cost_kkt_gate(fallback.velocity, inputs):
    return rejected_plan_for_existing_hold()
return accepted_nominal_plan_with_separate_solver_diagnostics(fallback)
```

Exceptions would continue to the shared controller's existing handlers.
Exactly fixed coordinates need an explicit implementation decision before
coding: for example, eliminate them and subtract their fixed contribution
from the target, solve the remaining strictly bounded problem, then restore
and validate against the original full problem. This would preserve the
nominal mathematics, unlike silently enlarging the feasible set. It is a
proposed compatibility detail, not behavior already provided by the SciPy
helper.

The term "robust" here means numerical reliability through an alternative
solver for the same deterministic optimization. It does not mean robust
optimization over uncertainty sets, min-max disturbance rejection,
chance constraints, or certified stability under model error. A retry may
recover a numerically rejected feasible problem; it cannot repair truly
empty bounds or missing physical authority. Any latency, acceptance-rate,
or balance-outcome benefit requires new evidence. Existing robust H2 results
must not be relabeled as 3C-robust results.

## Verification and Evidence Boundaries

Existing focused tests provide source-level checks of nominal behavior:

| Test source | Coverage |
| --- | --- |
| [Frozen planner tests][test-planner] | No publication/input mutation; explicit solve inputs. |
| [Velocity OCP tests][test-ocp] | Analytic derivatives, SciPy parity, active bounds. |
| [DDP velocity controller tests][test-ddp] | Random problem parity, diagnostics, infeasible excursion holds, explicit diagnostic commit. |
| [Base controller tests][test-base] | Ownership, scales, targets, model routing, limits, backtracking, publication, gravity failure, estop. |
| [Robust H2 tests][test-robust] | Accepted-path publication identity and SciPy use after returned rejection. |

A focused command from the controller submodule root is:

```bash
PYTHONPATH=. uv run pytest \
    test/test_frozen_3c_planner.py \
    test/test_counter_velocity_ocp.py \
    test/test_counter_ddp_velocity_controller.py \
    test/test_counter_balance_controller.py \
    test/test_counter_residual_h2_robust_controller.py
```

Before implementing or evaluating 3C-robust, add tests for exact accepted
primary identity, unchanged objective/bounds, rejected-primary recovery,
primary exceptions not retried, fallback exceptions/status failure, finite
and KKT rejection, fixed coordinates, both-arm publication parity, and
single finalization. Include collision/hold/estop cases and separate solver
diagnostics. Existing robust H2 tests do not establish the proposed stronger
fallback gate.

For a paper, report numerical acceptance separately from operational trial
completion and physical outcome. Compare matched trajectories and initial
conditions; report fallback attempts, accepted retries, remaining holds,
constraint activity, moving-arm tracking, and end-to-end timing tails.
No performance or robustness-optimization claim follows from this document
or from unit-test parity alone.

Historical rationale and evaluation remain in the
[Iteration 3C design][design] and [Iteration 3C analysis][analysis]. The
[H2 implementation document][h2-doc] provides broader residual-controller
context, but is not required to understand the nominal law or fallback
distinction specified here.

[base]: ../h12_ros2_controller/core/controller/counter_balance/counter_balance_controller.py
[ddp]: ../h12_ros2_controller/core/controller/counter_balance/counter_ddp_velocity_controller.py
[planner]: ../h12_ros2_controller/core/controller/counter_balance/frozen_3c_planner.py
[objective]: ../h12_ros2_controller/core/controller/counter_balance/objective.py
[ocp]: ../h12_ros2_controller/core/controller/counter_balance/counter_velocity_ocp.py
[robust-h2]: ../h12_ros2_controller/core/controller/counter_balance/counter_residual_h2_robust_controller.py
[support]: ../h12_ros2_controller/core/support_region.py
[joints]: ../h12_ros2_controller/utility/joint_definition.py
[upper]: ../h12_ros2_controller/core/controller/upper_controller.py
[robot]: ../h12_ros2_controller/core/robot_model.py
[publisher]: ../h12_ros2_controller/core/low_cmd_handler.py
[loader]: ../h12_ros2_controller/utility/controller_config.py
[wide-config]: ../../../config/sweep_configs/fame_magpie_fast_hard_groups_counter_ddp.yaml
[runtime]: ../../../h12_zmp_benchmark/runtime/arm_target_runtime.py
[sim-config]: ../config/sim_safety_split.yaml
[test-planner]: ../test/test_frozen_3c_planner.py
[test-ocp]: ../test/test_counter_velocity_ocp.py
[test-ddp]: ../test/test_counter_ddp_velocity_controller.py
[test-base]: ../test/test_counter_balance_controller.py
[test-robust]: ../test/test_counter_residual_h2_robust_controller.py
[design]: counter_balance_iteration_3c.md
[analysis]: counter_balance_analysis_iteration_3c.md
[h2-doc]: frozen_h2_and_robust_h2_implementation.md
