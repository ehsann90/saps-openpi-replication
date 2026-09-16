# C1-C1 physical validation — live inference under measured-q hold

**Date:** 2026-09-16

**Result:** PASS

Source run: `outputs/physical_c1c1/c1c1_20260916T131945Z`

Run UUID: `aaf87d8e-bab0-45b9-b595-446a8e5a79b8`

A live π0.5/DROID inference request was performed while the FR3 remained under
an explicitly installed measured-q equilibrium hold. The pre-inference hold was
confirmed applied before inference began, no new target was published during
inference, and the installed equilibrium target remained unchanged across
sampled controller-state telemetry during the 3.238 s inference interval. A fresh
post-inference measured-q hold was then applied successfully. No policy action or
gripper command was executed, and no Franka health violation was observed.

## Recorded provenance and sequence

- SAPS branch: `experiment/physical-fr3-saps`.
- Exact SAPS base commit: `dc5ce5e1ee038d793f7b672cc72a5fffce8d06c2`.
  The recorded tree was dirty with the C1-C1 implementation and Makefile target
  being validated; this archive does not substitute the later closeout commit.
- OpenPI pin: `15a9616a00943ada6c20a0f158e3adb39df2ccac` (clean).
- `fr3_lab_stack` pin: `9e535b665626cf8f3894fc4d2ef9136790abad92` (clean).
- Policy: `pi05_droid`, checkpoint `gs://openpi-assets/checkpoints/pi05_droid`.
- Prompt: `Pick up the object`; episode seed: `20260827`; replan index: `0`.

The sequence was fresh live DROID observation → fresh measured-q pre-hold →
confirmed controller application → one live inference → save native `[15,8]`
response without execution → fresh measured-q post-hold → confirmed application
→ post-hold evidence drain and final validation. Both holds used
`q_target = q_measured`; neither was a DROID action. The application-confirmation
timeout was 0.25 s. No controller gains, safety thresholds or warm-up behavior
were changed for this gate.

The reused `configs/physical_pi05_fr3.json` contains historical P0 milestone and
execution-disabled metadata as the observation/policy contract. The top-level
`physical_c1c1` run identity is authoritative for this experiment.

## Hold application and inference timing

| Hold | Controller sequence | T0→T4 |
| --- | ---: | ---: |
| Pre-inference | 1 | 1.495998 ms |
| Post-inference | 2 | 1.019387 ms |

Both holds had exactly one forwarded record, accepted callback and application;
installed `q_desired` matched the published measured-q target. Controller
instance `60141-21689313392677` and activation `21848350486110` were consistent.
T4 denotes equilibrium installation, not physical convergence.

Recorded monotonic ordering (ns): pre-hold T4 `22286949506756` ≤ inference start
`22286964455444` ≤ inference completion `22290202331894` < post-hold T0
`22290248420411` < post-hold T4 `22290249439798`.

| Observed first live request timing | Seconds |
| --- | ---: |
| Client round trip | 3.237829432 |
| Server inference | 3.013742075 |
| Policy inference | 2.738020817 |

This is the observed first C1-C1 request with model-input auditing/cold-start
context, not normal warm latency. [P0's audit documentation](../../physical_pi05_p0.md)
notes first-request copying and device synchronization; analyze this request
separately from later requests.

## Sampled hold retention

The read-only derivation selects `controller_state` records in `timing.jsonl`
whose `receive_monotonic_ns` falls inclusively between the recorded
`policy_inference` start and completion. The controller's state layout supplies
`dq` at indices 7–13, `q_desired` at 14–20, tracking error at 21–27, and last
applied target sequence at 37. Absolute maxima are over those selected samples;
per-joint desired-q span is maximum minus minimum. Seconds are nanoseconds / 1e9;
milliradians are radians × 1000. Exact derived values are in `hold_analysis.json`.

- Inference interval: 3.23787645 s; controller-state samples: 307.
- Target sequences: `[1.0]`; desired-q span: zero for all seven joints.
- Maximum sampled absolute tracking error: 0.038485954616640505 mrad.
- Maximum sampled absolute joint velocity: 0.003944330795423518 rad/s.

The same sequence in every sample, zero desired-q span, and very small sampled
tracking error and velocity support hold retention. These are sampled
controller-state observations, **not continuous-time maxima**.

## Final validation and health

The recorded run reports `status: success`, one completed request,
`policy_actions_executed: 0` and `gripper_commands_issued: 0`.
Delivery and runtime-health validators are accepted. Both publication checks,
`no_publication_during_inference` and
`pre_t4_inference_post_hold_order_valid`, are true.

Franka samples: 7,212; observed health violations: 0.

| Controller-period evidence | Value |
| --- | ---: |
| Samples | 7,380 |
| Mean period | 0.999998 ms |
| p95 | 1.097023 ms |
| p99 | 1.139993 ms |
| Maximum | 1.196238 ms |
| Periods above 1.5 ms | 0 |

Evidence gaps, dropped period samples, dropped application records, controller
publication errors and unmatched forwarder/callback/application records were all
zero. Capture-boundary losses remain outside the analyzer's observability.

## Limitations and evidence

C1-C1 validates holding during one live inference request; it does not validate
live policy-action execution, task performance, repeated replanning, or autonomy
continuation. The first request took 3.238 s end-to-end. That latency does not
invalidate this hold-only gate, but a one-time discarded warm-up request will be
considered before C1-C2 so the first physically executed chunk is not generated
from a cold-start observation. Warm-up is not implemented in C1-C1. No task-success
or C1-C2/live-action validation is claimed.

`run.json` and `response.json` are byte-for-byte copies of the source run's
`run.json` and `request_0000/response.json`. `hold_analysis.json` contains the
supplied derived metrics, independently reproduced from the raw telemetry during
closeout. `SHA256SUMS` covers these three evidence JSON files, excluding README
as in the C1-B convention. Raw timing, images and action archives remain under
`outputs/` and are not committed. No physical experiment was rerun for closeout.
