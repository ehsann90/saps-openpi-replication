# C1-C2 Repeated Arm Runtime — Cold-Start Two-Chunk Validation

## Status

**PASS — repeated arm-runtime validation under reproduced cold-start conditions**

Qualifying physical run:

```text
outputs/physical_c1c2/c1c2_2chunk_cold_20260917T114502Z
```

Run UUID:

```text
22111120-db81-4d66-a7b4-e9a98adccf1a
```

This run closes the C1-C2 repeated **arm-only** runtime gate. It validates one
discarded cold-start policy warm-up followed by two consecutive live
π0.5/DROID replans, each with a confirmed measured-q pre-inference hold,
post-hold observation barrier, eight fresh-state-anchored arm actions at 15 Hz,
and a confirmed fresh measured-q terminal hold.

The run deliberately reproduced the cold-server condition that had exposed the
earlier between-chunk readiness failure. The corrected runtime completed both
replans and terminated only because the explicit two-chunk test cap was
reached.

This is **not** a manipulation-task-success result. Physical gripper actuation
remains disabled because the DROID-to-FR3 gripper command conversion is not yet
verified, and the current outcome provider returns `CONTINUE`.

## Provenance

```text
SAPS repository branch:
experiment/physical-fr3-saps

SAPS base commit at execution:
de40f71202d40038029c0a8fa0e5b1e4a780cb83
Validate live C1-C2 policy chunk execution

Local C1-C2 fix files under physical validation:
M docs/physical_c1c2_runtime.md
M src/saps/physical/policy_execution.py
M tests/unit/test_physical_policy_execution.py

OpenPI:
15a9616a00943ada6c20a0f158e3adb39df2ccac

fr3_lab_stack:
9e535b665626cf8f3894fc4d2ef9136790abad92
```

OpenPI and `fr3_lab_stack` remained clean at their frozen revisions.

Immediately before this qualifying run, the implementation passed:

```text
337 tests plus compilation
git diff --check
```

No controller, forwarder, policy mapping, safety threshold, collision threshold,
application-confirmation timeout, controller-active evidence TTL, or
post-terminal evidence-drain duration was relaxed to obtain this result.

## Regression context

Two earlier two-chunk attempts exposed history-dependent orchestration problems.

### First failure: full-prefix confirmation inside the 0.25 s hold gate

Run:

```text
c1c2_2chunk_20260917T104329Z
```

The first terminal hold was physically applied, but online confirmation timed
out because confirmation repeatedly analyzed the growing full telemetry prefix.
The runtime was changed so pre-hold and terminal-hold confirmation use a
target-local evidence cursor while the full-prefix delivery audit remains
authoritative after the chunk.

### Second failure: stale readiness before replan 1

Cold-server run:

```text
c1c2_2chunk_20260917T111640Z
```

The discarded warm-up absorbed the multi-second cold inference and chunk 0
completed, including terminal-hold confirmation and a valid 10-target
full-prefix audit. The next pre-inference hold was nevertheless rejected because
controller-active evidence had expired during expensive post-chunk history
processing.

The follow-up correction:

- captures one stable post-chunk evidence snapshot;
- copies only the append-only record-reference slice while holding the boundary
  lock, then deep-copies outside the lock;
- reuses the same detached snapshot for delivery, inference-hold, and
  runtime-health validation;
- performs a bounded, command-free live readiness reacquisition before allowing
  the next pre-hold;
- retains the unchanged 0.25 s application-confirmation bound, 300 ms
  controller-active evidence TTL, two-second evidence drain, and full-prefix
  audit.

The qualifying run below was performed with the inference server deliberately
started cold again.

## Validated execution sequence

```text
process startup
→ one seed-separated discarded cold warm-up
→ warm-up source barrier crossed

REPLAN 0
→ fresh measured-q pre-inference hold
→ unique target-local T4 confirmation
→ new full observation after the pre-hold source barrier
→ main inference: seed 20260827, replan_index 0, audit enabled
→ actions 0..7 at independently scheduled 15 Hz deadlines
→ fresh measured-q terminal hold
→ unique target-local T4 confirmation
→ two-second evidence drain
→ one detached full-prefix evidence snapshot
→ full delivery + inference-hold + runtime-health validation
→ outcome CONTINUE
→ command-free readiness reacquisition accepted

REPLAN 1
→ new fresh measured-q pre-inference hold
→ unique target-local T4 confirmation
→ new full observation after the new pre-hold source barrier
→ main inference: seed 20260827, replan_index 1, audit disabled
→ actions 0..7 at independently scheduled 15 Hz deadlines
→ fresh measured-q terminal hold
→ unique target-local T4 confirmation
→ two-second evidence drain
→ one detached full-prefix evidence snapshot
→ full delivery + inference-hold + runtime-health validation
→ outcome CONTINUE
→ explicit two-chunk test cap
→ stop before replan_index 2
```

No gripper command was issued.

## Accounting and termination

```text
warm-up requests:                 1
main policy requests:             2
completed main replans:           2
policy actions scheduled:         16
policy actions published:         16
terminal holds applied:           2
gripper commands issued:          0

runtime validation status:        passed
task outcome:                     continue
termination reason:               test_chunk_limit_reached
```

Final full-prefix delivery validation accepted all 20 arm targets:

```text
2 pre-inference holds
16 policy-action targets
2 terminal holds
-------------------------
20 total
```

Controller application sequences were continuous from `1` through `20`.

## Cold-start warm-up and real inference timing

The qualifying run reproduced the cold-start condition.

Discarded startup warm-up:

```text
client round trip:                3363.529 ms
server inference:                 3153.276 ms
policy inference:                 2856.532 ms
response discarded:               yes
arm target publications:           0
gripper commands:                  0
```

First real action-producing request:

```text
replan_index:                     0
policy episode seed:              20260827
client round trip:                127.813 ms
server inference:                 121.357 ms
policy inference:                  96.145 ms
```

Second real action-producing request:

```text
replan_index:                     1
policy episode seed:              20260827
client round trip:                114.246 ms
server inference:                 112.331 ms
policy inference:                  90.124 ms
```

The warm-up therefore absorbed the multi-second first-request cost in this
process. This is useful mechanism validation, but it is not presented as a
controlled latency experiment.

## Between-chunk post-processing and readiness

After replan 0 terminal-hold confirmation and the fixed two-second drain:

```text
evidence snapshot copy:           211.600 ms
shared validation after snapshot:  15.508 ms
total snapshot + validation:      227.108 ms
```

The new command-free readiness barrier then ran before replan 1:

```text
checks:                           1
accepted:                         true
last readiness reasons:           []
barrier duration:                 0.398 ms
```

Only after this successful live readiness check was the replan-1 pre-inference
hold allowed to publish.

After replan 1, the larger full prefix required:

```text
evidence snapshot copy:           325.883 ms
shared validation after snapshot:  30.130 ms
total snapshot + validation:      356.013 ms
```

No further readiness reacquisition was required because the explicit two-chunk
test cap terminated the process.

These measurements demonstrate why a live readiness barrier is required before
a further replan: full-history audit cost remains intentionally outside the
bounded online confirmation gate and can exceed the 300 ms controller-active
evidence lifetime as history grows.

## Hold confirmation and inference-hold evidence

Target-local confirmation remained bounded and independent of the full history.

Replan 0 terminal hold:

```text
controller sequence:              10
T0→T4:                             0.862 ms
publish→confirmation:             22.046 ms
```

Replan 1 pre-inference hold:

```text
controller sequence:              11
T0→T4:                             1.004 ms
```

Replan 1 terminal hold:

```text
controller sequence:              20
T0→T4:                             0.704 ms
publish→confirmation:             10.687 ms
```

Inference-hold audits accepted:

```text
replan 0 controller-state samples: 12
replan 1 controller-state samples: 10
gripper commands during inference: 0
```

The installed equilibrium target remained the pre-inference hold across sampled
controller-state telemetry during each blocking inference interval.

## Arm action execution

Each replan executed exactly actions `0..7` from its native `[15,8]` policy
chunk. Arm mapping remained:

```text
delta_q = 0.2 * clip(policy_action[0:7], -1, 1)
q_target = fresh_measured_q + delta_q
```

Every policy action was anchored to a newly received measured joint state. The
runtime did not construct an accumulated target-to-target trajectory.

Observed action publication lateness:

```text
replan 0:                         3.240–5.304 ms
replan 1:                         2.287–4.063 ms
```

Minimum target joint-limit margin:

```text
replan 0:                         0.645 rad
replan 1:                         0.463 rad
```

Maximum target Jacobian condition number:

```text
replan 0:                         8.376
replan 1:                         8.856
fixed rejection threshold:        17
```

All per-action safety gates accepted.

The terminal hold remains a timed fresh-state handoff rather than a convergence
criterion. Maximum measured joint speed at the terminal reference was about
`0.042 rad/s` after replan 0 and `0.734 rad/s` after replan 1.

## Controller timing and robot health

Raw timing evidence contains:

```text
controller-period samples:        16000
mean period:                      1.000001 ms
p50:                              0.999694 ms
p95:                              1.105418 ms
p99:                              1.160192 ms
maximum:                          1.205102 ms
samples > 1.5 ms:                0

target publications:             20
forwarder records:               20
controller applications:         20
controller publication errors:   0

Franka-state records:            14342
Franka health violations:        0
```

Per-replan runtime-health validation also accepted with zero Franka health
violations.

## Accepted claim

> Under a reproduced cold-start condition, the C1-C2 arm runtime absorbed the
> multi-second initial π0.5 inference into a discarded warm-up, completed two
> consecutive live policy replans, executed sixteen fresh-state-anchored arm
> actions through the persistent streaming impedance path, applied and uniquely
> confirmed both terminal holds, preserved full-prefix delivery and runtime-
> health validation, reacquired current controller readiness before the second
> replan, and terminated at the explicit two-chunk test limit without an
> observed Franka health violation.

## Scope and limitations

Validated by this archive:

- startup warm-up under a reproduced cold inference-server state;
- warm-up/main seed separation;
- two consecutive main policy requests with `replan_index = 0, 1`;
- target-local pre-hold and terminal-hold confirmation;
- fresh post-hold observation barriers for both replans;
- unchanged pre-inference equilibrium across sampled inference telemetry;
- sixteen live arm actions with fresh measured-state anchoring;
- independently scheduled 15 Hz deadlines;
- two confirmed terminal measured-q holds;
- shared detached post-chunk evidence snapshots;
- full-prefix audits after both chunks;
- live readiness reacquisition before replan 1;
- one continuous 20-target application chain;
- clean controller timing and Franka health evidence;
- explicit test-limit termination before replan 2.

Not validated:

- DROID-to-FR3 gripper actuation;
- manipulation-task success;
- a validated SUCCESS/FAILURE detector;
- the observed policy behavior relative to the intended object;
- long-horizon task completion;
- convergence before terminal holds;
- cold-start latency reduction as a controlled experiment.

A separate deferred behavior issue remains: during physical arm-only execution,
the robot appeared to move toward the workdesk rather than reliably toward the
target object, eventually reducing object visibility. This archive does not
attribute a cause. That issue should be investigated separately now that the
repeated execution path itself is validated.

## Suggested archived evidence

Repository archive:

```text
README.md
qualifying_run.json
qualifying_timing.jsonl.gz
request_0000_request.json
request_0000_response.json
request_0000_actions.npz
request_0000_model_audit.json
request_0001_request.json
request_0001_response.json
request_0001_actions.npz
failed_cold_run.json
SHA256SUMS
```

`failed_cold_run.json` preserves the immediately preceding cold-server
regression in which chunk 0 passed but replan 1 was blocked by stale
controller-active evidence. The successful qualifying run then demonstrates
the corrected behavior under the same cold-start condition.

`qualifying_timing.jsonl.gz` should be produced with deterministic lossless
compression (`gzip -n`) from the raw timing file.

## Next work

After this archive is frozen and the fix is committed/pushed, further arbitrary
three-, five-, or ten-chunk arm-only runtime tests are not required merely to
demonstrate repetition.

The next substantive work is to:

1. establish and validate the DROID-to-FR3 gripper command mapping;
2. establish a genuine task outcome mechanism for SUCCESS/FAILURE/ABORT;
3. investigate the deferred object-directed policy behavior separately from
   runtime transport/control correctness.
