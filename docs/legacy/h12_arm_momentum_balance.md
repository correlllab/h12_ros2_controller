# H1_2 Arm Momentum Balance

## Summary

Arm angular momentum control can be used as a residual balance layer on top of
an independent lower-body policy. The lower-body policy remains responsible for
contacts, foot placement, and primary posture stabilization, while the arm layer
uses measured balance signals to produce bounded centroidal angular momentum
targets for one or both arms.

The connection between balance error and arm motion comes from the ZMP/CMP and
centroidal angular momentum relation. Horizontal angular momentum rate changes
the effective ground reaction moment and can shift the ZMP/CMP. COM drift, COM
velocity, ZMP error, or support-margin error can therefore be mapped to a
desired angular momentum rate, then integrated into an arm momentum target.

When one arm is already moving for a task, its estimated angular momentum can be
subtracted from the desired whole-body momentum target. The remaining residual
target is assigned to the counter arm. This makes the arm controller compensate for
both external pushes and internal disturbances from manipulation, subject to
filtering, saturation, joint limits, and task-priority constraints.

## Control Plan

1. Measure robot state:

    - Joint positions, velocities, and torques.
    - Center of mass position and velocity.
    - Measured or estimated zero moment point.
    - Contact state and support polygon.

2. Build a balance error signal:

    - COM error relative to a nominal or support-centered reference.
    - COM velocity drift.
    - ZMP error relative to the desired ZMP or support polygon center.
    - Optional capture-point or divergent-component-of-motion error.

3. Convert balance error into a desired angular momentum rate:

    - Use centroidal dynamics to map a desired ZMP or CMP correction into
      desired horizontal angular momentum rate.
    - Saturate and low-pass filter the result.

4. Integrate angular momentum rate into an arm angular momentum target:

    - Keep a bounded target $h_{arm}^*$.
    - Subtract angular momentum already caused by the moving arm.
    - Allocate the remaining target to the counter arm.

5. Track the target with the available arm:

    - Use the arm block of the centroidal momentum matrix.
    - Solve a velocity, acceleration, torque, or DDP/QP tracking problem.
    - Add posture, joint-limit, collision, and smoothness costs.

6. Blend with safety logic:

    - Gate compensation by contact confidence and support margin.
    - Disable or reduce compensation near arm joint limits.
    - Do not let arm balance override hard safety or manipulation priority.

## Centroidal Dynamics

Let $c \in \mathbb{R}^3$ be the center of mass, $m$ the total mass, and
$H_G = [H_{lin}, H_{ang}]$ the centroidal momentum about the center of mass.
The angular part is $H = H_{ang}$.

The centroidal momentum matrix maps generalized velocity to momentum:

$$
H_G = A_G(q)\dot{q}
$$

The angular momentum part is:

$$
H = A_H(q)\dot{q}
$$

Centroidal dynamics are:

$$
m\ddot{c} = \sum_i f_i + mg
$$

$$
\dot{H} = \sum_i (p_i - c) \times f_i + \tau_i
$$

where $p_i$, $f_i$, and $\tau_i$ are contact point positions, contact
forces, and contact torques.

The arms cannot directly create net external force. They can change internal
angular momentum. Through coupling and lower-body reaction, that angular
momentum changes the required contact moment and shifts the effective
centroidal moment pivot.

## ZMP Relation with Angular Momentum

Assume flat ground with vertical axis $z$, center of mass
$c = [x, y, z]^T$, and net contact force:

$$
F = [F_x, F_y, F_z]^T
$$

For moderate motions:

$$
F_x \approx m\ddot{x},
\quad
F_y \approx m\ddot{y},
\quad
F_z \approx m(g + \ddot{z})
$$

Let $p_z = [p_x, p_y, 0]^T$ be the ZMP. By definition, the horizontal
contact moment about the ZMP is zero. The moment balance about the center of
mass gives:

$$
\dot{H} = (p_z - c) \times F
$$

Expanding the horizontal components:

$$
\dot{H}_x = (p_y - y)F_z + zF_y
$$

$$
\dot{H}_y = (x - p_x)F_z - zF_x
$$

Solving for the ZMP gives:

$$
p_x = x - \frac{zF_x}{F_z} - \frac{\dot{H}_y}{F_z}
$$

$$
p_y = y - \frac{zF_y}{F_z} + \frac{\dot{H}_x}{F_z}
$$

In vector form:

$$
p_{zmp,xy}
= c_{xy} - \frac{z}{F_z}F_{xy}
+ \frac{1}{F_z}
\begin{bmatrix}
    -\dot{H}_y \\
    \dot{H}_x
\end{bmatrix}
$$

This is the key transfer. Horizontal angular momentum rate shifts the ZMP.
Therefore, if the measured or predicted ZMP is drifting in a bad direction,
the arm can request a compensating $\dot{H}$.

## Balance Error to Momentum Rate

Choose a desired ZMP reference $p_z^*$. This can be the support polygon
center, the lower-body policy's nominal ZMP, or the closest safe point inside
the support polygon.

Define ZMP error:

$$
e_z = p_{zmp,xy} - p_z^*
$$

Define COM regulation error:

$$
e_c = c_{xy} - c_{xy}^*
$$

$$
e_v = \dot{c}_{xy} - \dot{c}_{xy}^*
$$

Build one balance correction vector in the ground plane:

$$
\Delta p = -K_z e_z - K_c e_c - K_v e_v
$$

where $\Delta p$ is the desired ZMP shift caused by angular momentum.

From the ZMP relation:

$$
\Delta p = \frac{1}{F_z}
\begin{bmatrix}
    -\dot{H}_y^* \\
    \dot{H}_x^*
\end{bmatrix}
$$

so a practical angular momentum rate command is:

$$
\dot{H}_x^* = F_z\Delta p_y
$$

$$
\dot{H}_y^* = -F_z\Delta p_x
$$

$$
\dot{H}_z^* = 0
$$

In matrix form:

$$
\dot{H}^* = F_z
\begin{bmatrix}
    0 & 1 \\
    -1 & 0 \\
    0 & 0
\end{bmatrix}
\Delta p
$$

This target should be clipped:

$$
\dot{H}^* \leftarrow \mathrm{clip}(\dot{H}^*, \dot{H}_{min}, \dot{H}_{max})
$$

Then integrate it to a bounded momentum target:

$$
H^*_{cmd,k+1}
= \mathrm{sat}\left(H^*_{cmd,k} + \Delta t\,\dot{H}^*_k\right)
$$

A leak term is useful so the arm naturally returns to a neutral momentum
target after the disturbance:

$$
H^*_{cmd,k+1}
= \mathrm{sat}\left((1 - \alpha\Delta t)H^*_{cmd,k}
+ \Delta t\,\dot{H}^*_k\right)
$$

## Compensating for a Moving Arm

If one arm is doing a task, estimate its angular momentum contribution:

$$
H_{task} = A_{H,task}(q)\dot{q}_{task}
$$

The counter arm should track the residual:

$$
H_{comp}^* = H_{cmd}^* - H_{task}
$$

This makes the counter arm cancel both external pushes and momentum injected by
the moving arm.

If both arms can participate, allocate the target with weights:

$$
\min_{H_L,H_R}
\frac{1}{2}\|H_L + H_R - H_{cmd}^*\|^2
+ \frac{1}{2}\rho_L\|H_L\|^2
+ \frac{1}{2}\rho_R\|H_R\|^2
$$

A larger $\rho$ gives that arm lower priority.

## Tracking the Arm Momentum Target

Partition the angular centroidal momentum map into lower body, moving-arm, and
counter-arm blocks:

$$
H = A_{H,base}\dot{q}_{base}
+ A_{H,task}\dot{q}_{task}
+ A_{H,comp}\dot{q}_{comp}
$$

With a black-box lower body, do not command $\dot{q}_{base}$. Instead,
estimate the current residual and command only the counter arm.

A velocity-level controller solves:

$$
\dot{q}_{comp}^*
= \arg\min_v
\frac{1}{2}\|A_{H,comp}(q)v - H_{comp}^*\|_{W_H}^2
+ \frac{1}{2}\|v\|_{W_v}^2
+ \frac{1}{2}\|q_{comp} + \Delta t v - q_{nom}\|_{W_q}^2
$$

subject to:

$$
q_{min} \le q_{comp} + \Delta t v \le q_{max}
$$

$$
v_{min} \le v \le v_{max}
$$

A simple closed-form seed is the damped least-squares solution:

$$
\dot{q}_{comp}^*
= A_{H,comp}^T
\left(A_{H,comp}A_{H,comp}^T + \lambda I\right)^{-1}
H_{comp}^*
$$

with a null-space posture term:

$$
\dot{q}_{comp}
= A_{H,comp}^{\#}H_{comp}^*
+ \left(I - A_{H,comp}^{\#}A_{H,comp}\right)
\dot{q}_{posture}
$$

where:

$$
\dot{q}_{posture} = -K_q(q_{comp} - q_{nom}) - K_d\dot{q}_{comp}
$$

## Acceleration or Torque-Level Version

If arm torque control is available, track angular momentum rate directly.
Differentiate angular momentum:

$$
\dot{H}_{comp}
= A_{H,comp}(q)\ddot{q}_{comp}
+ \dot{A}_{H,comp}(q, \dot{q})\dot{q}_{comp}
$$

Solve a QP for arm acceleration:

$$
\ddot{q}_{comp}^*
= \arg\min_a
\frac{1}{2}\|A_{H,comp}a + \dot{A}_{H,comp}\dot{q}_{comp}
- \dot{H}_{comp}^*\|_{W_{\dot{H}}}^2
+ \frac{1}{2}\|a\|_{W_a}^2
$$

Then convert to torque with inverse dynamics or an operational-space arm
controller:

$$
\tau_{comp} = M_{aa}(q)a^* + h_a(q, \dot{q})
$$

Use torque, velocity, and position limits as hard constraints or strong
penalties.

## Recommended Runtime Loop

```text
every control tick:
    read q, dq, tau, com, dcom, zmp, contacts
    compute support-safe zmp reference
    compute balance errors e_z, e_c, e_v
    compute desired momentum rate dH_star
    filter, clip, and integrate to H_cmd_star
    estimate moving-arm momentum H_task
    set compensating target H_comp_star = H_cmd_star - H_task
    solve arm momentum tracking QP/DDP
    send command to counter arm
```

## Practical Notes

- Filtering: low-pass ZMP and COM velocity before computing momentum targets.
- Deadband: ignore tiny ZMP errors to avoid arm jitter.
- Saturation: limit $H^*$, $\dot{H}^*$, joint velocity, joint acceleration,
  and torque.
- Support margin: scale compensation up as ZMP approaches the support
  boundary.
- Contact confidence: disable aggressive compensation during uncertain or
  changing contacts.
- Task priority: let manipulation override balance when the lower-body policy
  is already stable.
- Policy interaction: monitor whether the lower-body policy fights arm
  momentum, and reduce gains if oscillations appear.

## Minimal Starting Controller

A good first implementation is:

$$
\Delta p = -K_z(p_{zmp,xy} - p_z^*) - K_v\dot{c}_{xy}
$$

$$
\dot{H}^* = F_z
\begin{bmatrix}
    0 & 1 \\
    -1 & 0 \\
    0 & 0
\end{bmatrix}
\Delta p
$$

$$
H_{comp}^* = \mathrm{sat}(H_{cmd}^*) - H_{task}
$$

and a damped least-squares arm velocity command:

$$
\dot{q}_{comp}^*
= A_{H,comp}^T
\left(A_{H,comp}A_{H,comp}^T + \lambda I\right)^{-1}
H_{comp}^*
$$

This gives a clean bridge from balance metrics to arm angular momentum
without modifying the lower-body policy.
