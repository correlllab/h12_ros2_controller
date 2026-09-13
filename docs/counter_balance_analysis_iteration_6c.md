# Iteration 6C Analysis: Challenge Manipulation-Speed Envelope

## Status

Stage 1 analytic reference and sampled-geometry validation is complete. Planning
is paused by the user pending concurrent controller work, after the source-hash
guard detected an edit to the frozen counter controller. No faster discovery sweep, confirmation trial,
or selected video has yet
been run in Iteration 6C. The
[preregistered design](counter_balance_iteration_6c.md) freezes the comparison and
separates unvalidated commands from physical stability evidence.

## Existing Nominal Context

The following supplied nominal results are the baseline compatibility-audit
targets, not new 6C measurements:

| Policy | Controller | Stable | Drift | Stumble | Fall |
| --- | --- | ---: | ---: | ---: | ---: |
| FAME | Frame | 24 | 4 | 0 | 12 |
| FAME | Frozen robust 3C | 26 | 7 | 0 | 7 |
| ALMI-Manip-v2 | Frame | 37 | 3 | 0 | 0 |
| ALMI-Manip-v2 | Frozen robust 3C | 38 | 2 | 0 | 0 |

FAME has four `fall -> drift`, one `fall -> stable`, one `drift -> stable`, and no
regressions. ALMI has one `drift -> stable` and no regressions. They do not alone
establish an expanded speed envelope. Original roots remain under:

- `runs/challenge_sweep/20260909_205330_iter6b_new_challenge40_frame_vs_3c_fame/`.
- `runs/challenge_sweep/20260909_214657_iter6b_new_challenge40_frame_vs_3c_almi/`.

## Open Execution Gate

The endpoint interpolation, release derivatives, motion-to-hold transition,
publisher limits, effort handling and simulator actuator limits must be verified
before duration acceptance. No straight-line bank acceleration bound or unrelated
counter-controller acceleration parameter is accepted as endpoint evidence.

The source audit confirms the rest-to-rest **reference** formulas with measured
release position, but measured release velocity is not part of that polynomial.
The nominal joint-velocity standing check is disabled. The exact duration gate
available from existing limits is `T >= max_i(1.5*abs(delta_q_i)/V_i)`, plus
position and declared collision checks. Acceleration demand is
`6*abs(delta_q_i)/T^2`; there is no independently justified endpoint acceleration
ceiling in the current catalog, URDF or publisher. Actual actuator-force caps are
not an acceleration or saturation-free-tracking certificate. The implemented
analytic check reports these facts separately rather than substituting the saved
straight-line bank's acceleration envelope.

## Stage 1 Results And Admission Decision

The complete static ledger covers 40 immutable targets times five durations:
200 nominal-zero-home references and 2,800 per-joint records. All pass the actual
model/publication position and velocity intersection, with zero reference
rejections. Static collision revalidation covered all 11,746 catalog segment
samples with zero collisions. The minimum sampled clearance across targets ranges
from `0.004019893` to `0.038964703 m`; it is not a continuous or dynamic clearance
guarantee. Six catalog-documented below-floor paths remain permitted under the
unchanged direct-joint policy, not silently redesigned or excluded.

| T (s) | Gamma | Targets | Reference Admitted | Peak Velocity Range (rad/s) | Peak Acceleration Demand Range (rad/s^2) |
| --- | --- | --- | --- | --- | --- |
| 1.50 | 1.00 | 40 | 40 | 1.000000 to 3.076677 | 2.666667 to 8.204473 |
| 1.25 | 1.20 | 40 | 40 | 1.200000 to 3.692013 | 3.840000 to 11.814441 |
| 1.00 | 1.50 | 40 | 40 | 1.500000 to 4.615016 | 6.000000 to 18.460064 |
| 0.80 | 1.875 | 40 | 40 | 1.875000 to 5.768770 | 9.375000 to 28.843850 |
| 0.60 | 2.50 | 40 | 40 | 2.500000 to 7.691693 | 16.666667 to 51.277956 |

The ranges are per-motion maxima over joints, then minimum/maximum across the
40 targets. They are demands, not physical capability estimates. `0.60 s` is
conditional preflight only. All records still require actual measured-release
validation and retain `acceleration_bound: null` and uncertified dynamics.

**Decision:** absence of a justified acceleration ceiling alone does not block
this bounded-command simulation experiment. The actual question is physical
stability under explicitly bounded reference commands, not a prior proof of that
stability or saturation-free tracking. Admit only measured-release references
passing the source-derived position/velocity intersection and scoped collision
check, preserve actuator-force caps, and measure command delivery, saturation,
arm execution and physical outcome separately. This is not hardware authorization.

The first execution check is the first lexical target, `left_arm_forward`, at
`T=1.25 s`, under both policies and both frozen controllers. It is selected before
observing new outcomes, and its four records belong to the discovery panel rather
than becoming additional repetitions. Full discovery follows only if actual
runtime evidence validates the pipeline; rejected or unhealthy attempts remain
identified, never promoted to physical success. No numerical limit was chosen to
admit a desired duration or manufacture separation.

The final static source snapshot is benchmark-root
`runs/challenge_sweep/iteration6c/preflight/reference_final/`, containing summary,
motion/per-joint tables, collision checks, processed bounds/configs, input hashes
and an artifact index. The initial `preflight/` snapshot is retained as the
pre-instrumentation audit, not overwritten; `reference_v1/` is an intermediate
snapshot. The nominal compatibility audit and
linked 160-record inventory remain alongside it; original nominal roots are
unchanged. Stage-1 tests also verify runtime cubic parity and MuJoCo's distinction
between pre-clamp `actuator_force` and post-joint-clamp `qfrc_actuator`.
Final interface validation passed 163 tests. Endpoint wire ticks retain legacy
floor encoding; both wire and unique rounded physics ticks are logged. Ambiguous
wire headers are resolved only by a unique Float32 q/dq observation match, never
nearest-time guessing. Publisher q/dq buffers are themselves Float32: command
integrity compares the reference rounded to that representation, with exact
encoded equality rather than an arbitrary tolerance.

## Nominal Compatibility Audit

Read-only comparison verified all 160 retained trial configurations against those
generated from the current catalog and nominal sweep configs: execution-relevant
target vectors/poses and trial mappings match exactly. Each policy has 40 unique
geometries at `alpha = 1.0`, a `1.5 s` move and a ten-second hold. Current physical
classification applied to the retained summaries changes no label.

The frozen controller subtree and controller simulation configuration match the
recorded clean controller revision `ea8b138f0e24360e690717d8a19ddd4289323391`.
Corrected ALMI is the v2 65-D, one-history LSTM deployment, normalization off,
`[64,32]` head, action scale `0.25`, lower gains `kp 200/80`, `kd 6/3`, and 50 Hz.
The current checkpoint hashes to freeze for 6C are:

| Artifact | SHA-256 |
| --- | --- |
| FAME `policy_3600.pt` | `dd553331f5bc4be319ce8c6faed0633301bcf61cefb2489f44819ba8d769bad1` |
| FAME `encoder_3600.pt` | `5d9a91ce53ac9640a02bb8872fddff19c5a9d0876a716f6441b08a243bcf2494` |
| ALMI `model_almi_manip_2.pt` | `6432f8af22e6dbff42665d1936cbdcbb48d9a0b2b4b9be85f0d1c76d0a2d21dd` |

Historical records contain checkpoint paths, not hashes of loaded checkpoint
bytes, and record a dirty benchmark source tree. Thus current byte hashes must
not be presented as retroactive historical attestation. The catalog/source-config
mapping hashes also differ despite exact execution-geometry/trial-config parity;
direct `--resume` with the current catalog is incompatible. Reference old rows in
place, do not rewrite their hashes or force a resume. Current safety-layer logging
is disabled whereas the nominal snapshot enabled it; limits are unchanged, but
observability/runtime load differ and must be recorded.

Original physical labels remain authoritative observations, not fresh 6C
confirmations. Historical endpoint logs lack physics-tick linkage. A `0.252 s`
release-clock gap in FAME `right_diagonal_rank6` Frame and a `0.202 s` gap in FAME
`right_extended_down_forward_01` 3C (after nominal 11.5 s) limit runtime-continuity
claims; they are not silently reclassified from those gaps alone. The 19 FAME
falls' child-process code 143 follows documented fall cleanup and is not an
independent crash classification.

The baseline is therefore configuration/geometry-compatible historical evidence
with explicit source/checkpoint/continuity limitations, not a provenance-perfect
current-runtime repetition. No nominal rerun or new speed result is implied by
this compatibility assessment.

## Answers Pending Validated Discovery

FAME envelope expansion, ALMI envelope expansion, per-duration improvements and
regressions, clearest boundary targets, and paper-level speed-robustness evidence
are not yet established. Missing faster runs are not physical failures and do not
support a positive or negative speed-envelope conclusion.

## Source Freeze Blocker

The first campaign-plan attempt failed closed before creating a run: the current
`counter_balance_controller.py` differs from `preflight/reference_final/inputs.json`.
The concurrent edit moves arm-ownership initialization ahead of the superclass
constructor and adds an inherited startup-publication fallback. It was not made
by this experiment task. It is retained untouched, not silently reverted or
accepted as equivalent frozen evidence. The progress record contains both hashes.
There are zero faster-rollout physical outcomes at this point. Controller-source
authorization and a renewed compatibility/source freeze must precede launch.
The user selected **Pause for concurrent work**. No automatic restart is scheduled;
resume only after that work is settled and the experiment source is re-audited.
