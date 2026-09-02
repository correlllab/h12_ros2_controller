# Counter-Balance Iteration 4B Analysis

## Status

Iteration 4B completed the progressive adaptive-authority workflow through the
first justified temporal extension.

No Iteration 4B candidate passed the compact freeze gate. B0 and Iteration 3C
remain the frozen practical controllers. Iteration 4B code is retained in shadow
mode for diagnostics and future research.

## Step 1: Shadow Signal Validation

Artifacts:

- `runs/key_findings/20260901_174522_iter4b_shadow2_fame`.
- `runs/key_findings/20260901_175415_iter4b_shadow2_almi`.
- `runs/key_findings/iter4b_shadow_parameters.json`.

Pooled frozen values:

| Quantity | Value |
|---|---:|
| Moving momentum scale | `2.0500` |
| Moving momentum-rate scale | `6.3290` |
| Preview entry/full risk | `0.9524 / 1.2159` |
| Response entry/full risk | `0.2354 / 0.9413` |
| Authority rise/decay | `0.10 / 0.30 s` |

Observations:

- Preview is policy-independent for the same manipulation, as expected.
- FAME rescue target `11` has near-full preview risk.
- FAME rescue target `09` has only moderate preview risk.
- Ordinary left-arm overhead also has high preview risk.
- FAME rescue measured response is higher than stable ALMI right-boundary
  response.
- ALMI stumble response is high, but stumble rescue is optional.

Decision: retain preview and measured response. Preview alone is unlikely to
preserve every rescue or suppress ordinary false-positive authority.

## Step 2A: Preview-Only Authority

Artifacts:

- `runs/key_findings/20260901_180606_iter4b_preview_fame`.
- `runs/key_findings/20260901_182310_iter4b_preview_almi`.

Preview-only authority:

- Retained FAME target `11` drift outcome.
- Lost target `09`, which changed from 3C drift to fall.
- Preserved stable/drift ALMI guards in the initial screen.
- Did not improve ALMI stumbles.

Decision: reject preview-only scheduling. The failed target has moderate preview
risk but high measured response, supporting feedforward/feedback coupling as the
single next mechanism.

## Step 2B: Combined Authority

Artifacts:

- Initial screen:
  - `runs/key_findings/20260901_183708_iter4b_combined_fame`.
  - `runs/key_findings/20260901_184907_iter4b_combined_almi`.
- Repeated rescue/guard runs:
  - `runs/key_findings/20260901_190236_iter4b_combined_fame_r1` through the
    subsequent FAME repetitions.
  - `runs/key_findings/20260901_192904_iter4b_combined_almi_r1` through the
    subsequent ALMI repetitions.

Repeated physical outcomes:

- FAME target `09`: 3C and 4B were drift in all three complete trials.
- FAME target `11`: 4B was drift in all three complete trials; 3C had two drift
  and infrastructure in the remaining attempts.
- FAME target `06`: remained fall.
- ALMI right-boundary target `11`: 3C and 4B were drift in all three trials.
- ALMI manual-grasp stumble: no severity improvement; foot displacement tended
  to increase.

The median ALMI right-boundary base drift changed from approximately `0.1036` to
`0.1008`, but no meaningful threshold was preregistered. This is not sufficient
for the continuous-metric freeze alternative.

Only three complete rescue repetitions were obtained; repetitions four and five
were infrastructure failures. The required five-repeat gate was not met, which
independently prevents promotion.

Deduplicated combined-authority total-controller p99 was approximately
`14.92 ms` on the FAME repeat panel and `15.33 ms` on the ALMI repeat panel.
ALMI therefore also missed the `<15 ms` timing gate; rare maxima exceeded one
period.

Decision: combined authority retains both rescues and adds no severe regression,
but fails the required ordinary-case improvement gate.

## Step 3: H3-0 Temporal Test

Artifacts:

- `runs/key_findings/20260901_195505_iter4b_h3_fame`.
- `runs/key_findings/20260901_200848_iter4b_h3_almi`.

H3-0 used only:

- Repeated frozen 3C running costs.
- Velocity-level control.
- Known manipulation preview.
- Zero-cost terminal model.

No smoothing, braking, terminal reserve, or weak-response term was active.

Results:

- FAME target `11` remained drift; target `06` remained fall.
- ALMI right-boundary guards regressed from stable/drift to drift/stumble in the
  initial screen.
- H3 lifecycle logs contained approximately 890 solved and 66 solver-failure
  ticks. Failures were significant but did not dominate actual controller
  executions; earlier raw summary counts were inflated by repeated steady-state
  diagnostics.
- Deduplicated total-controller p99 was approximately `18.89 ms` on FAME and
  `17.72 ms` on ALMI, failing the timing gate on both panels.

Decision: reject H3-0. Since the minimal temporal formulation fails, no optional
H3 residual or longer horizon is justified.

The experimental H3 implementation was removed after rejection rather than kept
as a runtime mode.

H3 source was evaluated from a dirty worktree without a retained implementation
content hash. It is treated as exploratory negative evidence, not a reproducible
controller baseline. Its severe regressions and timing failure support rejection,
but no quantitative H3 result is used to promote another mechanism.

## Evidence Limitations

- Step-1 shadow commands were 3C-equivalent only; frame and B0 response
  distributions were not collected under the same scheduler instrumentation.
- The frozen parameter artifact is bound to source logs and fitting procedure,
  but preview entry/full values use logged peak summaries rather than the complete
  per-sample normalized risk sequence.
- Adaptive authority is retained in configuration but intentionally omitted from
  the normal hard-sweep controller list. It must be invoked explicitly for future
  shadow work.

## Final Decision

Iteration 4B does not produce a candidate that surpasses both B0 and Iteration
3C under the frozen compact gates.

Retained findings:

- Preview-only authority is insufficient for rescue target `09`.
- Measured feedback is necessary to retain both repeated rescues.
- Combined adaptive authority reduces some ordinary response magnitude but does
  not produce a preregistered repeated ordinary improvement.
- H3-0 is behaviorally and computationally worse.

Therefore:

- Do not run Iteration 4B hard-group or full-panel promotion sweeps.
- Do not add further scheduler heuristics, H3 terms, or weak shaping.
- Keep B0 and Iteration 3C as separate practical baselines.
- Keep Iteration 4B in shadow-only configuration.
