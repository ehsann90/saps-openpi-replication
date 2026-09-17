# C1-C2 Physical Test 1 Validation — 2026-09-17

## Status

**PASS — runtime validation only**

Accepted physical run:

```text
outputs/physical_c1c2/c1c2_20260917T092308Z
```

Run UUID:

```text
cbc7266d-990d-4935-8e94-e6e53aa139fa
```

This validation establishes the first live arm-only C1-C2 execution through the
persistent FR3 streaming impedance path. It validates one discarded policy
warm-up, one real π0.5/DROID inference at main `replan_index = 0`, eight
fresh-state-anchored arm actions at 15 Hz, a fresh measured-q terminal hold at
the tick-8 deadline, unique forwarder/controller application evidence, and a
clean one-chunk test-limit stop.

This run **does not establish manipulation-task success**. Gripper actuation was
disabled because the DROID-to-FR3 gripper command conversion remains
unverified, and the current outcome provider returns `CONTINUE`. The process
terminated because the explicit physical test cap was reached.

## Provenance

```text
SAPS repository branch:
experiment/physical-fr3-saps

SAPS base commit at execution:
cba260c97a19db5aadd99052a16c94cccec3413b
Add one-time physical policy warm-up

OpenPI:
15a9616a00943ada6c20a0f158e3adb39df2ccac

fr3_lab_stack:
9e535b665626cf8f3894fc4d2ef9136790abad92

C1-B reference commit:
dc5ce5e1ee038d793f7b672cc72a5fffce8d06c2
```

The SAPS working tree was intentionally dirty with the reviewed, uncommitted
C1-C2 runtime implementation under physical validation. OpenPI and
`fr3_lab_stack` were clean at their pinned revisions.

`igd_fr3_control` contained unrelated local visualization/test changes and was
not part of the C1-C2 command path.

Before physical execution, the C1-C2 implementation passed 330 tests,
including 18 new policy-execution tests, plus compilation and
`git diff --check`.

## Validated execution sequence

The accepted run exercised:

```text
process startup
→ one seed-separated discarded warm-up
→ warm-up source barrier crossed
→ fresh measured-q pre-inference hold
→ unique T4 application confirmation
→ new full observation with all four source stamps > pre-hold barrier
→ real inference
     policy_episode_seed = 20260827
     replan_index = 0
     model-input audit enabled
→ actions 0..7 at independently scheduled 15 Hz deadlines
     each action anchored to a new measured q
     q_des = q_measured + 0.2 * clip(u[0:7], -1, 1)
→ fresh measured-q terminal hold at scheduled tick 8
→ unique terminal-hold application confirmation
→ evidence drain and runtime validation
→ outcome provider returns CONTINUE
→ explicit one-chunk test limit
→ stop before replan_index = 1
```

No gripper command was issued.

## Accounting and termination

```text
warm-up requests:                 1
main policy requests:             1
completed main replans:           1
policy actions scheduled:         8
policy actions executed:          8
terminal holds applied:           1
gripper commands issued:          0

runtime validation status:        passed
task outcome:                     continue
termination reason:               test_chunk_limit_reached
```

The legacy top-level `completed_requests` field remains `0`; C1-C2 uses the
explicit `main_policy_requests` and `completed_main_replans` counters for its
runtime accounting.

## Warm-up and real inference

Warm-up request:

```text
policy seed:                      20260917
replan index:                     0
audit_model_input:                false
response discarded:               true
target publications:              0
policy actions executed:          0
gripper commands issued:          0

client round trip:                126.555 ms
server inference:                 118.464 ms
policy inference:                  95.442 ms
```

Real main request:

```text
policy seed:                      20260827
replan index:                     0
request index:                    0
chunk index:                      0
returned native action shape:     [15, 8]
finite response:                  true

client round trip:                125.261 ms
server inference:                 114.476 ms
policy inference:                  95.215 ms
```

The real request returned server-side model-input audit evidence. The exterior
camera was mapped to `base_0_rgb`, the wrist camera to `left_wrist_0_rgb`, and
both masks were active. The canonical request arrays matched the audited
server-side input hashes before transformation.

Both the warm-up and real request were already in the approximately 125 ms
regime. Therefore this run does **not** quantify cold-start latency reduction;
it verifies that the real request did not incur the multi-second cold-start
delay observed in the earlier cold C1-C1/C1-C2-W context.

## Pre-inference hold and observation barrier

The pre-inference measured-q hold was uniquely applied before the real policy
request. T4 application preceded inference start by approximately:

```text
189.001 ms
```

The source-time barrier recorded immediately after hold confirmation was:

```text
1789636994.7785368
```

The action-producing observation used:

```text
wrist image:       1789636994.7858365   (+7.300 ms above barrier)
exterior image:    1789636994.7883737   (+9.837 ms above barrier)
joint state:       1789636994.8186476   (+40.111 ms above barrier)
gripper state:     1789636994.8101404   (+31.604 ms above barrier)
```

All four source timestamps therefore strictly postdated the confirmed
pre-inference hold barrier.

Observation freshness at assembly:

```text
wrist source age:                  34.377 ms
exterior source age:               31.840 ms
joint-state source age:             1.566 ms
gripper-state source age:          10.073 ms
cross-source skew:                 32.811 ms
```

Existing camera-pair advancement, source/receive freshness, skew, identity, and
schema checks remained active.

## Inference hold retention

Real inference interval:

```text
125.319 ms
```

Sampled controller-state evidence during that interval:

```text
controller-state samples:         11
q_desired span, all 7 joints:      0 rad
maximum sampled |q-q_desired|:     3.409 mrad
maximum sampled |dq|:              0.015984 rad/s
gripper commands:                  0
```

This supports the narrow conclusion that the installed equilibrium target
remained unchanged across the sampled controller-state telemetry during the
real inference interval.

These are sampled controller-state results, not continuous-time maxima.

## Eight-action live execution

Exactly actions `0..7` were executed from the returned native `[15,8]` chunk.

For every arm action:

```text
delta_q = 0.2 * clip(policy_action[0:7], -1, 1)
q_target = fresh_measured_q + delta_q
```

Each action used a distinct advancing measured-state sample rather than an
accumulated previous target. Every per-action safety gate accepted.

Across actions `0..7`:

```text
minimum target joint-limit margin:     0.536258 rad
maximum target Jacobian condition:     8.580418
fixed singularity threshold:           17
action publication lateness range:     2.030–3.354 ms
```

The physical safety thresholds and collision settings were not changed.

## 15 Hz schedule and terminal hold

The nine independently rounded schedule deadlines represented actions `0..7`
plus the terminal hold at tick 8.

The terminal hold was a new fresh measured-q zero-displacement command and was
not counted as a policy action.

```text
nominal action-0 → terminal-hold interval:   533.333333 ms
actual action-0 → terminal-hold publication: 533.338831 ms
difference:                                    +0.005498 ms

terminal-hold schedule lateness:                2.866786 ms
```

The terminal reference still contained motion:

```text
maximum |dq| at terminal-hold reference:       0.110399 rad/s
```

This is not a convergence failure. C1-C2 Test 1 validates timed delivery and
fresh-state handoff, not settling before the terminal hold.

## Delivery evidence

The run produced exactly ten arm targets:

```text
1 pre-inference hold
8 policy actions
1 terminal hold
```

Delivery validation found:

```text
expected targets:                  10
analyzed targets:                  10
controller sequences:             1..10
single controller instance:       yes
single controller activation:     yes
all applications accepted:        yes
```

The embedded forwarder evidence contains ten forwarded targets with no
rejections or publication errors.

## Controller timing

Lossless raw controller/forwarder/robot telemetry is retained in the archived
compressed `timing.jsonl.gz`.

Unique controller-period samples:

```text
count:                             7640
mean:                              1.000007 ms
p50:                               0.999898 ms
p95:                               1.104759 ms
p99:                               1.163932 ms
maximum:                           1.197362 ms
samples > 1.5 ms:                 0

dropped period samples:            0
dropped application records:       0
controller publication errors:     0
```

## Robot/runtime health

For the validated replan:

```text
Franka state samples:              3336
Franka health violations:          0
post-hold Franka samples:          1999
post-hold controller readiness:    21
post-hold controller-state samples: 197
post-hold joint samples:           2001
```

No observation callback, observation executor, or cleanup errors were recorded.

## Accepted claim

> A live π0.5/DROID replan was executed physically through the persistent FR3
> streaming impedance path after a one-time discarded warm-up and confirmed
> measured-q pre-inference hold. The action-producing observation was acquired
> after the hold barrier, the installed equilibrium target remained unchanged
> across sampled controller-state telemetry during inference, and actions 0–7
> were executed at independently scheduled 15 Hz deadlines using fresh
> measured-state anchoring. A fresh measured-q terminal hold was then uniquely
> applied at the tick-8 deadline. All ten commanded targets were uniquely
> forwarded and applied in order, no Franka health violation was observed, and
> controller-period telemetry contained no sample above 1.5 ms. Execution
> stopped at the explicit one-chunk test limit; task success was not evaluated.

## Scope and limitations

Validated by this test:

- one warm-up request and one real main request;
- seed separation between warm-up and main episode;
- confirmed pre-inference measured-q hold;
- post-hold source-time observation barrier;
- real request model-input audit;
- unchanged sampled equilibrium target during inference;
- eight live arm actions with fresh measured-state anchoring;
- independently scheduled 15 Hz action deadlines;
- fresh measured-q terminal hold at tick 8;
- unique forwarder/controller application for all ten targets;
- clean controller timing and Franka health evidence;
- explicit test-limit termination before any second replan.

Not validated by this test:

- physical gripper actuation;
- manipulation-task success;
- a validated task-success/failure detector;
- repeated live replanning across multiple chunks;
- long-horizon task completion;
- cold-start latency reduction as a controlled experiment;
- convergence at the terminal hold.

These remain subsequent C1-C2 validation objectives.

## Archived evidence

The compact repository archive contains:

```text
README.md
run.json
request.json
response.json
actions.npz
model_audit.json
timing.jsonl.gz
SHA256SUMS
```

`timing.jsonl.gz` is a lossless deterministic (`gzip -n`) compression of the raw
run telemetry to avoid committing the much larger uncompressed JSONL file.

The first main request's model-input bundle and visual audit files remain in the
original run directory; `model_audit.json` records their hashes and mappings.

A separate forwarder console capture is optional because the raw timing archive
already contains the ten structured forwarder evidence records.

## Next validation

After this evidence is frozen and the C1-C2 runtime is committed, the next
physical test should use the **same runtime implementation** with a larger
`C1C2_MAX_EXECUTED_POLICY_CHUNKS` limit to validate repeated replanning.

No redesign is implied by that next test. Gripper actuation should remain
disabled until an authoritative DROID-to-FR3 command conversion is established
and separately validated.
