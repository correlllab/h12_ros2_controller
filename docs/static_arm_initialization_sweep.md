# ALMI Static Arm Initialization Sweep

## Purpose

This side-quest tests whether repeated ALMI stumble targets are primarily caused
by dynamic manipulation or by static upper-body configurations outside the
lower-body policy's standing distribution.

## Cases and Samples

Canonical stumble families:

- `left_manual_grasp_pitch_minus`.
- `left_arm_overhead`.

For each target arm vector `q_target`, hold:

\[
q_{arm}(\alpha)=(1-\alpha)q_{home}+\alpha q_{target},
\]

at `alpha = 0, 0.25, 0.50, 0.75, 1.0`. The three interior values are the
required samples; endpoints provide home and full-target controls.

The 15 upper joints are `[torso, left arm, right arm]`. Torso remains at home.
MuJoCo `initial_pose.qpos` and the `upper_fixed` command use the same sampled
configuration from startup onward.

For a backend with policy-specific lower defaults, initialize lower joints from
the copied deployment configuration and translate free-base z so ankle-roll
support height matches the qpos0 baseline. This prevents a lower-body warmup
transition from being mistaken for static upper-pose behavior.

## Isolation Contract

- Lower-body policy: ALMI.
- Upper controller: `upper_fixed` only.
- No arm trajectory, target interpolation, residual MPC, or impulse.
- Base and lower-body joints start from the model `home` keyframe.
- Upper joints are overwritten by named MuJoCo joint addresses.
- Duration: `10 s` after standing release.
- A permissive but fixed release gate (`0.35 rad`, `1.0 m/s`, `1.0 rad/s`) is
  used for all samples so unstable static poses still enter the observed phase.
- Experiments are serial and headless.
- Every run renders a replay by default.

## Artifact Layout

```text
runs/static_arm_initialization/<timestamp>_<name>/
    configs/<sample_id>.yaml
    runs/<sample_id>/<timestamped run>/
    videos/upper_fixed/<sample_id>_a01.webm
    static_arm_initialization_manifest.json
    static_arm_initialization_results.csv
```

The manifest records source target/catalog hashes, alpha, sampled arm/upper
vectors, complete initial qpos, generated config, run directory, summary, and
video path.

## Metrics and Interpretation

Record run completion, fall/stumble/drift/stable severity, base tilt, foot
displacement/lift, support margins, standing readiness, and process failures.

- Static intermediate/full poses stumble: evidence for pose-dependent/OOD
  lower-body behavior.
- Static poses remain standing while dynamic trajectories stumble: evidence for
  disturbance timing/response rather than terminal pose OOD.
- A monotonic alpha transition: identify the approximate static-pose boundary.
- Nonmonotonic or family-specific outcomes: configuration geometry/support
  coupling matters beyond simple arm magnitude.

Any apparent static transition must be repeated before a strong OOD claim.

## Results

The corrected retained sweep is:

`runs/static_arm_initialization/20260904_012711_almi_stumble_cases_v2`.

Boundary repeats are:

- `runs/static_arm_initialization/20260904_014030_almi_boundary_r2`.
- `runs/static_arm_initialization/20260904_014500_almi_boundary_r3`.

Both families were stable through alpha `0.50` and stumbled from alpha `0.75`.
The repeated boundary is documented in `static_arm_initialization_analysis.md`.
