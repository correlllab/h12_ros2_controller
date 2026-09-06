# ALMI Manipulation-Speed Sweep

## Status and Purpose

This experiment precedes Iteration 5C. It holds target geometry, smooth
interpolation shape, controller parameters, limits, safety, fade, and hold
behavior fixed while varying only manipulation `move_duration`.

The goals are to locate repeated ALMI stumble transitions, determine whether
frozen H2 shifts them relative to Frame/B0/3C, and test H3 once at an informative
H2 boundary without reopening general H3 tuning.

## Targets

- `left_manual_grasp_pitch_minus`: repeated hard-target stumble family.
- `left_arm_overhead`: independent exploration-target stumble family.

## Controllers

- `frame_task`.
- B0 `counter_ddp`.
- Frozen 3C `counter_ddp_velocity_wide`.
- Frozen H2 `counter_residual_h2_frozen`.
- H3 only for the final focused boundary check.

## Speed Grid

The smooth cubic trajectory is unchanged. Speed is represented by movement
duration:

| Profile | Duration | Relative nominal speed |
| --- | ---: | ---: |
| `quasi_12s` | `12.0 s` | `0.125x` |
| `quasi_6s` | `6.0 s` | `0.25x` |
| `slow_3p5s` | `3.5 s` | `0.43x` |
| `medium_2p25s` | `2.25 s` | `0.67x` |
| `nominal_1p5s` | `1.5 s` | `1.0x` |

Run one discovery trial for every target/controller/speed cell. Repeat three
times around every apparent standing/stumble transition. Use five repetitions
only for a changed boundary or promotion-quality claim.

## Recorded Evidence

For each cell report:

- Stable/drift/stumble/fall severity and tracking status.
- Peak/RMS torso tilt and angular rate.
- Positive tilt-rate divergence and peak timing.
- Support-validity loss and foot-motion onset relative to release.
- Foot displacement and lift.
- Counter-arm requested/applied velocity and excursion.
- Manipulation configuration error.
- Solver and complete-controller timing.

The fastest non-stumble duration is defined from repeated outcomes, not a single
trial. A useful controller-development boundary must retain stumble pressure for
Frame/B0 while H2 is near transition; a speed where every controller is stable
is not informative.

## H2/H3 Boundary Check

After locating H2's transition, compare frozen H2 and unchanged H3 only at that
speed and target. Use three repetitions, or five if a severity boundary changes.
Do not tune H3.

If H3 does not repeatedly move the boundary beyond H2, retain the Iteration 5B
conclusion that missing physical risk information, rather than horizon length,
is the more likely limit.

## Results

### Execution

The first launch used insufficient total duration for slower profiles and failed
configuration validation before simulation. The corrected profile duration was
set to `move_duration + 10 s` while preserving the same hold. The retained
discovery root is:

`runs/archive/key_findings/20260903_184645_20260904_almi_h2_speed_discovery_v2`.

Because all controllers still stumbled at `6 s`, the grid was extended to
`quasi_12s` (`0.125x` nominal speed). Missing interrupted cells were rerun as
individual trials.

### Outcome Boundary

Every physical run in both families exceeded stumble thresholds at every tested
duration from `1.5` through `12 s`.

| Family | Duration range | Frame | B0 | 3C | H2 |
| --- | --- | --- | --- | --- | --- |
| Manual minus | `1.5-12 s` | Stumble throughout | Stumble throughout | Stumble throughout | Stumble throughout |
| Overhead | `1.5-12 s` | Stumble throughout | Stumble throughout | Stumble/fall | Stumble/fall |

Some 3C/H2 cells were classified infrastructure because one-step solver failures
made the run operationally incomplete. Their physical trajectories had already
crossed the stumble threshold and are not standing outcomes.

There is no observed fastest tolerable manipulation speed for any controller.
No tested controller remained standing repeatedly even at `12 s`.

### H2 Trace Evidence

Manual-grasp-minus is nearly speed-invariant:

| Duration | Peak tilt | Peak rate | Divergence integral | Foot travel |
| ---: | ---: | ---: | ---: | ---: |
| `12 s` | `0.241 rad` | `0.583 rad/s` | `0.180` | `3.10 m` |
| `6 s` | `0.235 rad` | `0.613 rad/s` | `0.168` | `3.67 m` |
| `3.5 s` | `0.237 rad` | `0.571 rad/s` | `0.166` | `3.67 m` |
| `2.25 s` | `0.233 rad` | `0.578 rad/s` | `0.149` | `3.29 m` |
| `1.5 s` | `0.236 rad` | `0.609 rad/s` | `0.152` | `3.23 m` |

Overhead responds to speed in torso coordinates but not stumble severity:

| Duration | Peak tilt | Peak rate | Divergence integral | Foot travel |
| ---: | ---: | ---: | ---: | ---: |
| `12 s` | `0.096 rad` | `0.306 rad/s` | `0.051` | `2.72 m` |
| `6 s` | `0.108 rad` | `0.491 rad/s` | `0.045` | `2.79 m` |
| `3.5 s` | `0.134 rad` | `0.774 rad/s` | `0.053` | `2.64 m` |
| `2.25 s` | `0.210 rad` | `0.884 rad/s` | `0.090` | `2.32 m` |
| `1.5 s` | Fall trajectory | Fall trajectory | `5.09` | `2.52 m` |

Manipulation tracking remained precise in completed non-fall runs. H2 timing
across retained speed runs was `3.40/8.19/12.85/28.65 ms`
p50/p95/p99/max. Requested/applied counter velocities reached the frozen
`[2.6, 3.2, 2.6, 1.5] rad/s` limits, especially during long trajectories. The
speed sweep did not tune H2.

### Support and Indicator Finding

Torso tilt/rate is a strong pre-step indicator for manual-grasp-minus, but it is
not sufficient across families. Overhead at `12 s` steps several meters while
peak tilt remains below `0.10 rad`.

The existing logged `support_valid` signal never became false on manual H2 runs
and remained true on slow overhead stumbles. It therefore does not expose the
incipient protective step in these cases. Final foot displacement changes by
meters while angular response and manipulation speed change much less.

The limiting factor is not disturbance magnitude alone. Evidence points to
support/posture dynamics and missing pre-step foot-motion risk information.
Counter-arm authority and horizon length cannot correct a risk that is absent
from the objective/confidence state.

## Focused H2/H3 Check

No true H2 transition region exists. To test the slower-timescale hypothesis
conservatively, H2 and unchanged H3 were compared three times at the slowest
`12 s` manual-grasp-minus regime.

- H2 physical outcomes: stumble 3/3; median foot travel `3.10 m`.
- H3 physical outcomes: stumble 3/3; median foot travel `3.13 m`.
- Both sets contained inherited one-step solver failures that made some runs
  operationally incomplete.
- H3 did not improve severity or shift a standing/stumble boundary.

The negative nominal-speed H3 conclusion is not reversed at a slower timescale.
Missing physical risk information, not horizon length, remains the supported
limitation.

## Conclusions

1. No controller has a repeated non-stumble speed in the tested `1.5-12 s`
   range.
2. Frozen H2 does not shift the ALMI stumble boundary relative to Frame/B0/3C.
3. Angular indicators remain strong for manual grasp but fail to explain slow
   overhead stepping.
4. No near-boundary speed suitable for residual-cost development was found.
5. The primary missing mechanism is support/foot-motion risk representation;
   slower disturbance, greater horizon, and existing angular costs are
   insufficient.

Iteration 5C should retain H2, use slow trajectories only for identification,
and first test whether real-compatible foot pose/twist or support residuals add
repeatable pre-step information beyond tilt/rate. Nominal speed remains the hard
validation target.
