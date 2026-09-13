# Iteration 6C: Challenge-Group Manipulation-Speed Envelope

## Status And Question

Iteration 6C is a new experiment. Stage 1 is in progress; no faster discovery
motion is authorized until the endpoint execution contract passes. Iteration 6B
remains closed and its spatial result is not reinterpreted.

**Does frozen robust 3C expand the manipulation-speed stability envelope relative
to Frame?** Physical classification is primary, not trajectory RMS optimization.
The companion [analysis](counter_balance_analysis_iteration_6c.md) records actual
execution, exclusions and blockers separately from this preregistration.

## Frozen Comparison

- Baseline: `frame_task`.
- Method: `counter_ddp_velocity_robust`, the current SciPy-primary robust 3C.
- Policies: FAME and corrected `mjlab_almi_manip_2`, analyzed separately.
- Targets: all 40 unique current challenge geometries, with `alpha = 1.0`.
- Nominal duration: `1.5 s`; existing evidence is reusable only after compatibility
    verification of target vectors, release/motion/hold protocol, controller,
    policy/checkpoint and evaluation configuration.
- Candidate discovery durations: `1.25`, `1.00`, `0.80 s`. `0.60 s` is conditional
    on executability and unresolved boundary information after that panel.
- No controller tuning, H2 comparison, target redesign, payloads, lower-body policy
    changes, or borrowed straight-line acceleration envelope.

## Stage 1: Endpoint Execution Contract

Inspect the actual runtime cubic, measured release state and endpoint derivatives
before assuming the rest-to-rest formulas. The intended audit reports per joint
position extrema, maximum commanded velocity and acceleration, model/publisher
bounds and actuator/effort semantics for every geometry and candidate duration.
Collision evidence must state its scope: endpoint or sampled checks are not a
continuous swept-path certificate, and a nominal fixed-counter posture cannot
certify the evolving counter arm or lower body.

Duration acceptance must use independently justified endpoint runtime bounds.
Missing evidence is unknown, not an infinite physical limit or automatic pass.
Record rejected and unresolved cases with reasons. Recheck release-dependent
quantities against the actual measured release state before publishing faster
motion. Do not silently retime or clip a rejected cubic into a different motion.

### Verified Interpolation And Release

The actual simulation route is benchmark `runtime/arm_target_runtime.py`,
`_trajectory_target`, `interpolation_alpha` and `interpolation_velocity`. At release
it captures measured reduced joint position, but does not insert measured velocity
into the polynomial. For `delta = q_target - q_release`, `u = t/T`, and `0 < u < 1`:

```text
q_ref = q_release + delta * (3*u^2 - 2*u^3)
dq_ref = 6*delta*u*(1-u)/T
ddq_ref = 6*delta*(1-2*u)/T^2
max(abs(dq_ref)) = 1.5*abs(delta)/T
sup(abs(ddq_ref)) = 6*abs(delta)/T^2
```

Both commanded endpoint velocities are zero. The physical release velocity need
not be zero: the nominal standing configuration disables its joint-velocity gate
with `max_joint_velocity: 0.0`. It must be recorded separately; setting a Hermite
initial derivative to measured velocity would change the existing interpolation.
The analytic helper supports general derivatives for tests, but its endpoint
admission wrapper uses the actual zero-commanded-derivative law.

The hold joins continuously in reference position and velocity, with one-sided
acceleration jumps from/to zero. There is no finite global jerk certificate. The
endpoint clock is monotonic wall time, sampled at the control loop, not the saved
straight-line physics-tick clock. The faster publisher repeats the command buffer;
it does not evaluate a new continuous cubic. Integral hold starts at `T` for Frame
and after reactive hold/fade for 3C. Keep that existing method lifecycle unchanged.
Time-triggered hold entry is not evidence of physical endpoint attainment.

### Applicable Limits And Scope

| Moving Joint | Magpie URDF Velocity (rad/s) | URDF Effort / MJCF Actuator Force Cap (N m) |
| --- | ---: | ---: |
| Shoulder pitch | 9 | 40 |
| Shoulder roll | 9 | 40 |
| Shoulder yaw | 20 | 18 |
| Elbow | 20 | 18 |
| Wrist roll / pitch / yaw | 31.4 each | 19 each |

The effective position range is the intersection of the actual controller-model
URDF and both publication stages' position clips. In particular, actual Magpie
shoulder-roll limits are `[-0.19,3.4]` left and `[-3.4,0.19]` right, tighter than
the hardcoded publisher's `-0.38/0.38` bounds. Use source-derived arrays, not this
table as a replacement for asset identity checks.

The nominal simulation profiles multiply publisher velocity and feedforward torque
clips by three. Those are not physical motor ratings. The profile's `dq_lim = 6`
constrains counter-arm bounds but is bypassed by direct moving-arm endpoint
publication; it must not be labeled an existing moving-arm admission limit. The
smallest supported reference-speed check intersects actual model and publication
velocity bounds and requires `T >= max_i(1.5*abs(delta_i)/V_i)`.

No independent moving-arm acceleration limit is supplied by the URDF, publisher
or endpoint runtime. The current catalog explicitly reports acceleration demand
without certifying it. The real-app `25 rad/s^2` value is borrowed from the
straight-line bank and is not adopted here. Likewise, the observed catalog peak
`8.204473 rad/s^2` at `1.5 s` is a demand, not an allowable limit.

The simulator applies joint actuator-force caps to the motor effort resulting
from `tau_ff + kp*(q_cmd-q) + kd*(dq_cmd-dq)`. The bridge computes this expression
on command reception, not freshly at every physics step. Feedforward clipping
alone does not limit total requested servo effort. Gravity compensation is not
full trajectory inverse dynamics, and dividing rated torque by armature or a
guessed inertia is not a justified acceleration limit. Bounded actuator effort
does not prove saturation-free tracking; it also must not be confused with a
requirement to prove whole-body stability before measuring that stability.

The catalog's collision result covers a sampled zero-home-to-target segment, with
joint-space step at most `0.01 rad` and at least 30 intervals. Actual measured
release, evolving counter arm and lower-body posture are not that nominal segment.
Frame's direct publication has no per-sample collision check; 3C backtracks only
its counter candidate and can retain moving-arm intent on a counter hold. Preserve
that distinction. Below-floor direct-joint paths are explicitly permitted in the
current catalog; no planner floor is introduced as a new exclusion.

### Implemented Reference Check

`utility/endpoint_execution_contract.py` analytically computes position extrema,
maximum reference velocity and acceleration, and one-sided boundary accelerations.
`assess_endpoint_reference` uses the actual zero-derivative endpoint law and
provided model/publication bounds. It returns per-joint position/velocity validity,
rejection reasons, measured-release velocity mismatch, and `reference_admissible`.
Acceleration feasibility and dynamic tracking feasibility remain explicitly
uncertified when there is no independently supported bound.

The opt-in `arm_target.endpoint_reference_contract` version 1 now connects this
check to the actual measured-release path. It intersects the declared, source-
derived bounds with the loaded reduced model and actual controller publisher
bounds; it cannot relax those bounds. It samples the measured-start arm segment
with fixed observed non-arm posture, at least 30 intervals and joint-space step
at most `0.01 rad`, using the existing collision model and SRDF. The nominal
counter path used by this screen is not a certificate of the future reactive
counter motion. A rejection is recorded before motion and aborts that command,
not silently clipped or retimed into a different experiment.

Each run stores `endpoint_release_contract.json`, including captured q/dq, state
tick/sequence, exact extrema, effective limits, sampled collision outcome, scope
and gate duration. Rejected starts also write `endpoint_reference_rejected.flag`.
The controller log additionally records intended and published arm q/dq, published
feedforward torque, observed tick/sequence and move/hold phase. Physics recording
adds `qfrc_actuator`: joint-level clamping affects this field, whereas MuJoCo's
`actuator_force` reports pre-joint-clamp actuator force. Neither is total contact
or passive generalized force. This is telemetry, not a change to actuation.

**Admission means a source-bound reference and sampled geometry check, executed
under the verified unchanged actuator/safety constraints. It does not mean a
proof of acceleration-limited, saturation-free or safe hardware tracking.** An
unavailable acceleration certificate is not alone a reason to require proof of
the stability being experimentally measured. Measured tracking, saturation,
command clipping/delivery, interruption and physical outcome remain separately
audited. A surviving but unexecuted motion cannot support a usable manipulation-
speed claim. The full ledger, tests and least-aggressive matched execution checks
must precede expansion to the full discovery panel. No new limit is chosen to
admit a desired duration or produce controller separation.

## Discovery And Health Protocol

Use one headless, serial, single-worker run per controller/target/validated duration
for each policy. Preserve all 40 targets in the ledger, including exclusions; do
not select only favorable geometries. Controller comparisons require matched
geometry, alpha, duration and protocol. Rotate which controller executes first
across target blocks. Capture health before launch and between sweep blocks,
including competing simulation processes, available memory/swap, storage and
runtime failures. Do not kill unrelated user processes.

Infrastructure failures and initialization failures remain separate from stable,
drift, stumble and fall. A complete physics recording alone does not certify live
controller continuity. Preserve every attempt and any unresolved case; do not
retry physical or solver failures as if they were infrastructure accidents.

Retain the current classifier: infrastructure takes precedence over physical
labels; completed non-survival is fall. Stumble uses foot displacement above
`0.075 m`, or same-foot displacement above `0.03 m` and lift above `0.005 m`,
only for stepping-capable policies. FAME defaults to not stepping-capable, whereas
ALMI-Manip-v2 is stepping-capable. Drift uses peak roll/pitch orientation drift
above `0.1 rad`. Final joint-configuration error `<=0.02 rad` is a separate
precision diagnostic. Do not force identical stumble semantics across policies.

Fall detection retains `0.50 rad` sustained for `0.75 s`, immediate `0.60 rad`
hard tilt or `0.75 m` height, and a two-second post-confirmation record. Explicit
fall cleanup can terminate child processes with code 143; this is not by itself
an independent infrastructure crash. Preserve outer run status and cleanup reason.

## Stability Envelope And Pairing

Define `gamma = 1.5 / T`. Report all categorical Frame-to-3C transitions, including
unchanged states, improvements and regressions. Physical ordering is
`stable < drift < stumble < fall` where stumble is applicable; infrastructure,
initialization and unavailable outcomes have no physical ordering.

For each controller and target, report the largest **observed validated** gamma
with a non-fall outcome, and separately with a stable outcome. A missing or
rejected duration is not a fall and cannot locate a physical boundary. If no
observed duration qualifies, report undefined, not zero speed. If the fastest
tested duration still qualifies, label the boundary right-censored. Preserve
nonmonotonic outcomes instead of inferring unobserved intermediate success.

Compare paired gamma maxima only over a common tested/validated duration domain;
expose differing coverage and exclusions. Treat discovery maxima as single-run
observations until confirmed, not reliable worst-case safety guarantees. Do not
pool FAME and ALMI into one headline or repeated speeds into independent samples.

## Diagnostics And Confirmation

Retain peak pelvis tilt, translation/drift, CoM/support-relative motion, foot
displacement/lift, measured moving-arm tracking, endpoint error, available arm
momentum, requested/applied counter action, collision/backtracking/hold state,
solver/runtime timing and initialization outcomes. These explain margins and
transitions; they do not substitute for physical classification. No straight-line
Segment/Uniform RMS is introduced for endpoint tasks.

Only after complete discovery, preregister a confirmation subset containing
survival/stability improvements, regressions, both policies' boundary transitions
and ambiguous near-threshold cases. Initially use three fresh interleaved
repetitions with rotated controller order. Extend only a small preregistered
headline subset to five fresh repetitions. Never collect until a desired p-value
appears. Select synchronized videos only after discovery and confirmation, showing
both improvements and meaningful regressions.

## Artifacts And Completion Rule

Use benchmark-root `runs/challenge_sweep/iteration6c/` with `preflight/`,
`discovery/`, `confirmation/`, `reports/`, `videos/` and `health_checks/`. Maintain
an artifact index and resumable progress JSON with source/config/target/checkpoint
hashes, run identities, exclusions, health and exact commands. Do not mutate or
copy nominal evidence unnecessarily; link verified baseline records in place.

Final outputs are per-duration classifications, paired transitions, all 40 targets'
non-fall/stable speed envelopes, and policy-separated conclusions. A genuine
executability/safety blocker stops launch and remains explicit; unperformed
experiments are not null results or evidence for a paper claim.
