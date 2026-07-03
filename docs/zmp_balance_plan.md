# ZMP Balance Plan

This document sketches a more responsive arm balance-assist controller. It is
intended to replace the previous `zmp_controller` implementation, which was too
slow, too swing-heavy, and too coupled to a single-arm corrective impulse flow.
The replacement may still use the module and class name `zmp_controller`, but it
should be implemented as a new modular controller rather than a patch of the old
formulation.

The new design separates balance observation, perturbation detection, momentum
target generation, arm-role allocation, and arm actuation.

The first implementation should support both arms assisting balance. The coding
interface should also support a future mode where one arm is executing a task and
the other arm actively counter-balances.

## Goals

- Detect true out-of-balance events or large external perturbations quickly.
- Avoid arm motion while the robot is standing normally.
- Generate smaller, shorter, and more frequent counter-actions.
- Keep balance logic independent from the arm-motion planner.
- Support both-arm balance now and one-free-arm balance later.
- Keep the public name `zmp_controller` available for the new implementation.

## Use Cases

### Both Arms Assist Balance

Both arms are available for balance recovery. The controller observes balance
state continuously and allocates the desired counter angular momentum across the
left and right arms.

Expected behavior:

- If the robot is standing quietly, no arm motion is commanded.
- If a push or out-of-balance signal is detected, both arms generate a short
    counter-momentum response.
- After the response, both arms return to their saved pre-response posture.
- If the disturbance persists, the controller can issue another short response.

### One Arm Moving, One Arm Assist Balance

One arm is reserved by another task, such as reaching or manipulation. The other
arm remains available for balance assistance.

Expected behavior:

- The task arm is marked unavailable or partially available.
- The balance controller allocates counter-momentum only to the assist arm.
- The assist arm avoids disturbing the task arm command interface.
- The same detection and target-generation logic is reused.

This mode should not be implemented first, but the interfaces should be designed
so that it can be added without rewriting the controller.

## Balance Metrics

The controller should use multiple metrics because ZMP alone can be noisy and
can trigger unnecessary arm motion while the robot is standing still.

### ZMP Error

```text
e_z = zmp_xy - z_ref
```

`z_ref` is the desired ZMP location in the support region. It can be the foot
support center plus a configured offset.

This metric identifies whether the contact wrench is moving toward the edge of
the support region. It should be required for balance actuation, but should not
trigger actuation by itself.

### Center Shift

```text
delta c = c_xy - c_ref
```

`c_ref` is a learned quiet-standing center reference. It adapts slowly when the
robot is not perturbed and stays fixed when a perturbation is detected.

This metric prevents false positives from normal standing posture. The robot may
stand safely with a nonzero offset from the foot midpoint, so the absolute COM
position should not be treated as a perturbation.

### COM Velocity

```text
v_c = dot c_xy
```

This metric detects pushes before the center shifts far. It is important for a
responsive controller because waiting for large displacement can be too late.

### Angular Velocity And Acceleration

```text
omega = imu angular velocity
alpha = dot omega
```

These metrics detect rotational perturbations. Angular acceleration should be
computed from a filtered finite difference of angular velocity.

### External Force Proxy

If direct force sensing is unavailable, use a force-like proxy from centroidal
dynamics:

```text
F_contact = m (ddot c - g)
F_external_proxy = F_contact - F_expected
force_proxy = ||F_external_proxy_xy||
```

Use `force_proxy` as a scalar norm in threshold logic. Keep the first
implementation conservative because `ddq` can be noisy. If the force estimate is
unstable, use large COM acceleration, large ZMP jumps, and large angular
acceleration as proxy signals instead.

### Support Region Margin

The first implementation can use ZMP error from the support center:

```text
e_z = zmp_xy - z_ref
```

This is simple and usable, but it does not distinguish a safe ZMP near the
center from a ZMP near the support boundary. Add a small helper that can later
replace this with support-polygon margin:

```text
support_margin = distance from zmp_xy to support polygon edge
zmp_bad = support_margin < support_margin_threshold
```

Start with ankle midpoint plus foot-size bounds if a full support polygon is not
available. Keep the interface as `compute_support_margin(balance_state)` so the
implementation can improve without changing the detector.

## Perturbation Criteria

The controller should actuate only when an out-of-balance condition and a motion
condition agree.

Recommended trigger:

```text
zmp_bad = ||e_z|| >= zmp_threshold
# later: zmp_bad = support_margin < support_margin_threshold

motion_bad = any([
    ||delta c|| >= center_shift_threshold,
    ||v_c|| >= com_velocity_threshold,
    ||omega|| >= angular_velocity_threshold,
    ||alpha|| >= angular_acceleration_threshold,
    force_proxy >= external_force_threshold,
])

perturbed = zmp_bad and motion_bad
```

Add hysteresis and confirmation:

```text
enter perturbation if perturbed for N_enter ticks
exit perturbation if not perturbed for N_exit ticks
```

This avoids single-sample noise causing arm swings.

## Counter-Momentum Target

The controller should estimate a desired centroidal angular momentum impulse
that pushes the ZMP back toward the support target.

The current ZMP relation is:

```text
z_x = c_x - c_z F_x / F_z - dot L_y / F_z
z_y = c_y - c_z F_y / F_z + dot L_x / F_z
```

Use a horizontal correction vector:

```text
r = K_z e_z + K_c delta c + K_v v_c + K_a alpha_xy
```

Assuming `F_z ~= mg`, convert the desired ZMP correction into angular momentum
rate:

```text
dot L_des = [ -mg r_y,  mg r_x,  0 ]^T
```

Convert rate to a short impulse target:

```text
L_des = T_response dot L_des
L_target = clip(L_des, -L_max, L_max)
```

For responsiveness, `T_response` and `L_max` should be small. The controller
should prefer repeated short corrections over one large slow swing.

## Arm Role Model

Represent each arm with a role and availability state.

```text
ArmRole = balance | task | idle | disabled
ArmAvailability = available | reserved | limited | unavailable
```

The balance layer should not assume both arms are always free. It should ask an
allocator which arms are available for balance at the current tick.

Suggested policy:

```text
both_arm_balance:
    left role = balance
    right role = balance

single_arm_task_balance:
    task arm role = task
    assist arm role = balance
```

## Momentum Allocation

The allocator converts a whole-body momentum target into per-arm targets.

For both-arm balance:

```text
L_left = w_left L_target
L_right = w_right L_target
w_left + w_right = 1
```

For symmetric balance, start with:

```text
w_left = 0.5
w_right = 0.5
```

For one-arm assist:

```text
L_assist = L_target
L_task = 0
```

Future improvements can choose weights from arm manipulability, distance to
joint limits, collision margin, current task priority, and available velocity.

Each arm solve should report the momentum it actually achieves:

```text
L_left_achieved = peak useful momentum from left plan
L_right_achieved = peak useful momentum from right plan
L_combined_achieved = L_left_achieved + L_right_achieved
```

Log target momentum, per-arm target momentum, per-arm achieved momentum, and
combined achieved momentum. This makes it visible when two independent arm DDP
solves do not sum to the requested whole-body target.

## Actuation Strategy

Use short, bounded DDP responses.

Recommended properties:

- Short hold phase: near zero or one control tick.
- Short momentum phase: fast impulse generation.
- Short return phase: brief DDP return followed by closed-loop posture return.
- Small momentum cap: prevent large visible swings.
- Frequent retriggering: allow another response if perturbation persists.

The actuation layer should own the response state machine:

```text
idle -> solving -> executing_impulse -> returning -> idle
```

If a stronger perturbation arrives while returning, the controller should be able
to interrupt return and start a new response.

Keep interruption simple: blend from the current return velocity into the new
impulse velocity over a short fixed window, for example two to three control
ticks:

```text
u = (1 - beta) u_return + beta u_new
beta: 0 -> 1 over N_blend ticks
```

Do not abruptly replace the return command with a new impulse command.

Solver failure policy should be explicit:

- If both arms fail, suppress the response and keep posture return active.
- If one arm succeeds and one arm fails, execute the successful arm only if its
    target momentum norm is above a useful minimum.
- Log all failures with target momentum, active arms, and solver status.

## Proposed Modules

### BalanceObserver

Responsibilities:

- Read `RobotModel` state.
- Compute ZMP, support target, support margin, COM, COM velocity, angular
    velocity, angular acceleration, and force proxy.
- Maintain quiet-standing COM reference.
- Return a `BalanceState` dataclass.

Suggested fields:

```python
@dataclass
class BalanceState:
    zmp: np.ndarray
    zmp_target: np.ndarray
    zmp_error: np.ndarray
    support_margin: float
    com_xy: np.ndarray
    center_reference: np.ndarray
    center_shift: np.ndarray
    com_velocity: np.ndarray
    angular_velocity: np.ndarray
    angular_acceleration: np.ndarray
    force_proxy: float
```

### PerturbationDetector

Responsibilities:

- Apply thresholds and hysteresis.
- Distinguish quiet standing from perturbation.
- Return a `PerturbationState` dataclass.

Suggested fields:

```python
@dataclass
class PerturbationState:
    active: bool
    severity: float
    reasons: list[str]
```

### MomentumTargetEstimator

Responsibilities:

- Convert `BalanceState` and `PerturbationState` into `L_target`.
- Apply response-time scaling and momentum clipping.
- Keep target-generation independent from arm availability.

### ArmRoleManager

Responsibilities:

- Track whether each arm is available for balance.
- Support `both_arm_balance` and `single_arm_task_balance` modes.
- Provide available balance arms to the allocator.

### MomentumAllocator

Responsibilities:

- Split `L_target` into per-arm targets.
- Respect unavailable or task-reserved arms.
- Provide an extension point for manipulability and collision-aware weighting.

Suggested output:

```python
@dataclass
class ArmMomentumTarget:
    arm: str
    target_momentum: np.ndarray
```

### BalanceActuator

Responsibilities:

- Own one `MomentumBehavior` per balance-capable arm.
- Solve short DDP responses.
- Execute planned velocity commands.
- Return arms to saved postures.
- Allow response interruption when a larger perturbation arrives.
- Blend interrupted commands over a short fixed window.
- Apply the explicit solver failure policy.

## Coding Plan

### Phase 0: Remove Deprecated Implementation

- Do not build on the old `ZmpController` behavior.
- Remove the deprecated single-arm ZMP controller and example entry point.
- Keep this document as the replacement design source of truth.
- Reintroduce `h12_ros2_controller/core/controller/zmp_controller.py` only when
    it contains the new modular architecture.

### Phase 1: Build New Modular Core

- Add `BalanceState`, `PerturbationState`, and `ArmMomentumTarget` dataclasses.
- Define one clean nested `zmp` config shape and parse it directly.
- Implement metric computation in `BalanceObserver`.
- Implement threshold and hysteresis logic in `PerturbationDetector`.
- Implement `L_target` computation in `MomentumTargetEstimator`.
- Add a new `ZmpController` shell that composes these modules but does not yet
    assume one-arm or both-arm actuation.

### Phase 2: Add Arm Roles And Allocation

- Add config field `zmp.mode` with values `both_arm_balance` and
    `single_arm_task_balance`.
- Add config field `zmp.assist_arm` for the future one-arm mode.
- Implement `ArmRoleManager` with both arms marked as balance in phase 2.
- Implement `MomentumAllocator` with equal weights for both-arm mode.

### Phase 3: Support Both-Arm Balance Execution

- Instantiate one `MomentumBehavior` per active balance arm.
- Allow `MomentumBehavior` to be created for `left` and `right` arms from the
    same controller config.
- Solve one short DDP plan per available arm.
- Compose per-arm velocity commands into one full-body velocity command.
- Execute both arm responses in the same control tick.
- Return each arm to its saved start posture independently.
- Use this as the first working controller mode.

### Phase 4: Improve Responsiveness

- Add response interruption while returning.
- Add two to three tick velocity blending when interrupting return.
- Add severity-scaled momentum target clipping.
- Add cooldown or minimum replan interval to prevent solver spam.
- Add logging for trigger reasons, severity, targets, per-arm achieved momentum,
    combined achieved momentum, solver failures, and plan duration.
- Add explicit solver failure handling for both-arm and one-arm-success cases.

### Phase 5: Add One-Arm Moving Compatibility

- Add an API for task controllers to reserve an arm.
- Allow the task controller to report whether the task arm is unavailable,
    limited, or available for partial balance assistance.
- Allocate full balance target to the assist arm when the task arm is reserved.
- Keep the task arm command stream untouched by the balance actuator.

## Config Sketch

```yaml
zmp:
  enabled: true
  mode: both_arm_balance
  assist_arm: right

  thresholds:
    zmp_error: 0.02
    support_margin: 0.03
    center_shift: 0.03
    com_velocity: 0.08
    angular_velocity: 0.15
    angular_acceleration: 1.0
    force_proxy: 20.0

  target:
    response_time: 0.08
    max_momentum:
      - 1.2
      - 1.2
      - 0.0

  hysteresis:
    enter_cycles: 2
    exit_cycles: 5

  observer:
    center_reference_alpha: 0.02

  blending:
    interrupt_ticks: 3

  gains:
    zmp:
      - 1.0
      - 1.0
    center:
      - 0.5
      - 0.5
    com_velocity:
      - 0.4
      - 0.4
    angular_acceleration:
      - 0.0
      - 0.0

  allocation:
    left_weight: 0.5
    right_weight: 0.5

  ddp:
    hold_duration: 0.02
    momentum_duration: 0.10
    return_duration: 0.12
    return_timeout: 0.4
    max_velocity: 6.0

  solver_failure:
    allow_single_arm_success: true
    min_useful_momentum: 0.1
```

Use only this nested shape for the new controller. Each module should read its
own subsection directly.

## First Implementation Scope

Implement first:

- Continuous observer and perturbation detector.
- Direct parsing of the new nested `zmp` config.
- Both-arm balance mode.
- Equal momentum allocation across both arms.
- Short DDP responses with frequent retriggering.
- Support-center ZMP error first, with `support_margin` as a helper extension
    point.
- Two to three tick blending when interrupting return.
- Explicit solver failure policy.
- Logs for trigger reasons, target momentum, per-arm achieved momentum, combined
    achieved momentum, solver failures, and plan duration.

Defer until later:

- One-arm moving task integration.
- Manipulability-aware allocation.
- Collision-aware allocation.
- Hard self-collision constraints inside DDP.
- Direct external-force estimation from contact sensors.

## Key Risks

- ZMP and force-proxy estimates may be noisy because they depend on acceleration.
- Solving two DDP plans may be too slow for immediate response unless horizons
    are short.
- Arm motion can itself perturb the robot if momentum caps are too high.
- Interruptible responses need careful command blending to avoid discontinuities.
