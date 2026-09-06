# ALMI Static Arm Initialization Analysis

## Status

The original timestamped static sweep artifacts were removed during policy-root
cleanup. The clean ALMI matrix is now `runs/static_arm_initialization/almi/` and
contains one alpha `0.25/0.50/0.75/1.00` run per target. Historical repeated
boundary values below remain design evidence but are not current clean-artifact
claims.

## 1. Implementation

The dedicated package is:

`h12_zmp_benchmark.experiment.static_arm_initialization`.

It provides:

- Source-bound target loading and interpolation.
- Named-joint MuJoCo qpos generation from the `home` keyframe.
- Matching 15-joint `upper_fixed` commands.
- Serial checkpointed benchmark execution.
- Standard summary-based classification.
- Default replay rendering and controller-grouped video links.
- Manifest/CSV regeneration from retained summaries.

All cases use ALMI, zero impulse, no manipulation trajectory, and a fixed
`10 s` observation. A permissive common release gate lets unstable static poses
enter the observed phase instead of being discarded as readiness failures.

## 2. Sampled Configurations

The torso and nonmoving arm remain at home. Values below are the seven left-arm
joints in target order.

### Manual-Grasp-Minus

| Alpha  | Left-arm configuration                                                       |
| ------:| ---------------------------------------------------------------------------- |
| `0.00` | `[0, 0, 0, 0, 0, 0, 0]`                                                      |
| `0.25` | `[0.182693, 0.833449, -0.097783, -0.025865, -0.719067, 0.115409, -0.165697]` |
| `0.50` | `[0.365386, 1.666898, -0.195567, -0.051731, -1.438134, 0.230818, -0.331393]` |
| `0.75` | `[0.548079, 2.500348, -0.293350, -0.077596, -2.157201, 0.346227, -0.497090]` |
| `1.00` | `[0.730772, 3.333797, -0.391134, -0.103462, -2.876268, 0.461636, -0.662787]` |

### Overhead

| Alpha  | Left-arm configuration      |
| ------:| --------------------------- |
| `0.00` | `[0, 0, 0, 0, 0, 0, 0]`     |
| `0.25` | `[0, 0.725, 0, 0, 0, 0, 0]` |
| `0.50` | `[0, 1.450, 0, 0, 0, 0, 0]` |
| `0.75` | `[0, 2.175, 0, 0, 0, 0, 0]` |
| `1.00` | `[0, 2.900, 0, 0, 0, 0, 0]` |

The manifest records full 14-arm, 15-upper, and 46-qpos vectors for every sample.

## 3. Discovery Results

### Manual-Grasp-Minus

| Alpha  | Outcome | Peak drift | Foot displacement | Foot lift       |
| ------:| ------- | ----------:| -----------------:| ---------------:|
| `0.00` | Stable  | `0.012`    | `0.020 m`         | `0.0004 m`      |
| `0.25` | Stable  | Small      | Below threshold   | Below threshold |
| `0.50` | Stable  | `0.060`    | `0.028 m`         | `0.0007 m`      |
| `0.75` | Stumble | `0.028`    | `2.529 m`         | `0.0175 m`      |
| `1.00` | Stumble | `0.036`    | `3.499 m`         | `0.0442 m`      |

### Overhead

| Alpha  | Outcome | Peak drift | Foot displacement | Foot lift  |
| ------:| ------- | ----------:| -----------------:| ----------:|
| `0.00` | Stable  | `0.016`    | `0.018 m`         | `0.0005 m` |
| `0.25` | Stable  | `0.011`    | `0.016 m`         | `0.0002 m` |
| `0.50` | Stable  | `0.015`    | `0.013 m`         | `0.0007 m` |
| `0.75` | Stumble | `0.076`    | `0.644 m`         | `0.0172 m` |
| `1.00` | Stumble | `0.043`    | `3.071 m`         | `0.0241 m` |

## 4. Boundary Repetitions

| Family       | Alpha `0.50`                     | Alpha `0.75`                      |
| ------------ | -------------------------------- | --------------------------------- |
| Manual minus | Stable 3/3; foot `0.028-0.031 m` | Stumble 3/3; foot `2.520-2.538 m` |
| Overhead     | Stable 3/3; foot `0.012-0.013 m` | Stumble 3/3; foot `0.397-1.039 m` |

The transition is repeated, family-independent at the sampled resolution, and
large relative to classification thresholds.

## 5. Readiness Evidence

Before the permissive common release gate was introduced, strict standing
readiness already failed at:

- Manual alpha `0.75` and `1.0`.
- Overhead alpha `1.0`.

The failure state showed elevated base linear/angular velocity despite no arm
trajectory. The permissive rerun then observed those poses as static stumbles.
This strengthens rather than replaces the post-release outcome evidence.

## 6. OOD Interpretation

The results strongly support a static upper-body pose distribution/support
problem in ALMI:

- No manipulation velocity, acceleration, or momentum impulse exists.
- Both unrelated target geometries transition at the same alpha interval.
- Stumble severity jumps discontinuously while torso drift can remain small.
- Full target poses produce multi-meter foot travel from startup.
- Alpha `0.50` remains repeatedly stable under the identical policy and runtime.

This does not prove which ALMI observation or training-distribution component is
responsible. Plausible mechanisms include unobserved upper-body configuration,
changed whole-body CoM/inertia, support-reference mismatch, or policy training
that omitted large arm poses.

The result also explains why slower dynamic manipulation did not remove the
stumble: the terminal/intermediate pose itself crosses a static policy boundary.

## 7. Implications

- Counter-arm residual MPC cannot solve a static lower-body policy OOD boundary
  reliably through small angular corrections.
- Foot twist remains useful for detecting the resulting protective step, but it
  is not the root cause.
- ALMI training/observation should include upper-body pose or corresponding
  centroidal/support features.
- A policy-side posture-conditioned standing experiment is better motivated than
  further H2/H3 cost tuning.
- Dynamic controller comparisons should distinguish trajectory disturbance from
  static-pose instability.

## 8. Artifacts

Videos:

`runs/static_arm_initialization/almi/videos/upper_fixed/`.

Every executed case has a rendered WebM. The main manifest contains exact source
hashes and full sampled vectors.

## 9. Validation

- Clean ALMI policy-root videos: 16.
- Benchmark-owned test suite: `142 passed`.
- Python compilation and staged/unstaged diff checks: passed.
- Static experiment implementation/config/test SHA-256:
  `74696978030790f975f77ab0ee2f472ed4e62d19e15e73fbea54423b1806519b`.
