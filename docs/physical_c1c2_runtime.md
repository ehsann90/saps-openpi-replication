# C1-C2 repeated physical arm runtime

G1B adds an explicitly enabled asynchronous gripper path; see
[the validated G1B integration](physical_g1b_gripper.md).
The arm-only baseline and historical validation below remain unchanged.

The arm-only baseline was implemented from frozen warm-up commit
`cba260c97a19db5aadd99052a16c94cccec3413b`; its validation is recorded
below. OpenPI remains pinned at `15a9616a` and fr3_lab_stack at `9e535b6`.
The separately enabled `physical-c1c2-gripper` path adds G1B hand actuation.
Neither entry point has a task-success detector. C1-B, C1-C1 and C1-C2-W
commands retain their existing behavior.

## Lifecycle and protocol

The C1-C1 ROS lifecycle supplies provenance, URDF limits, graph, camera, server,
controller and Franka checks and raw telemetry capture. The new loop first
runs exactly one discarded warm-up (seed `20260917`, index 0, no model audit).
All four observation sources must then cross its completion barrier. That
observation is archived under `startup/request_0000` and discarded.

Each main replan installs a new fresh measured-q pre-hold and confirms its
unique controller T4 application. Immediately after confirmation it samples a
ROS source-time barrier. Another full observation is acquired: wrist, exterior,
joint and gripper source stamps must all strictly exceed that barrier, in
addition to the existing age, receive-age, skew, camera advancement, camera
identity and schema checks. Late reception of old images cannot qualify.

The main seed is always `20260827`, indices are 0, 1, 2, ... and only index 0
requests model-input audit. The warm-up does not consume a main index. Inference
is synchronous; no target publication or gripper call occurs inside it.
Controller state records must show the installed pre-hold desired q and target
sequence during inference. Delivery analysis also rejects unmatched commands,
identity changes and evidence gaps. Audits run before action 0 and again after
the terminal hold to include delayed evidence.

Only native finite `[15,8]` chunks qualify. Execute actions 0 through 7:

```
delta_q[k] = 0.2 * clip(action[k, 0:7], -1, 1)
q_des[k] = freshly_measured_q[k] + delta_q[k]
deadline[k] = t0 + (k * 1_000_000_000 + 7) // 15
```

Every step waits until its deadline before acquiring its reference. The state
receive time must be at or after eligibility and its source stamp must advance
beyond the previous reference. Targets never accumulate. Existing joint margin,
singularity, finite-value, state/target freshness, ownership and Franka health
gates are unchanged. The exact integer deadline for the fresh measured-q
terminal hold is `t0 + 533333333 ns`, not action 8. Action 7 retains its full
scheduled interval. `t0` is the action-0 **schedule** origin; actual publication
includes state acquisition and preparation latency. This Python/ROS runtime
does not guarantee zero physical publication jitter: it records actual T0 and
lateness and rejects a whole missed interval, as C1-B does.

Pre-hold and terminal-hold confirmation use an evidence cursor captured just
before publication. Within the unchanged 0.25-second default deadline, only
that target and evidence acquired from its cursor onward are validated,
including run/source stamp, T0, desired q, controller identity, and unique
forwarder/callback/application evidence. Confirmation cost therefore does not
grow with prior episode telemetry. The confirmation's target counts describe
one hold; they are not a complete episode audit. In the G2 policy-execution
path, the pre-inference and post-chunk online audits inspect new evidence since
their last accepted cursor. Cross-window sequence, source-stamp, T4, and
evidence-ID continuity is required. The full append-only record is retained
for the final audit after closure. C1-B and its existing drain remain unchanged.
If a safety gate rejects a target before publication, its raw row remains in
the run, while final delivery analysis counts only targets eligible for
publication and reports `safety_rejected_unpublished_targets` separately.
The episode still fails on the original safety rejection.

After terminal T4 confirmation, the G2 policy-execution path waits for complete
incremental target delivery and healthy Franka state, controller state,
controller readiness, and measured-joint evidence received strictly after T4.
The controller-state message is headerless; the comparable timestamp here is
its monotonic callback receive time. The wait fails closed at the existing
two-second drain bound if its evidence does not arrive. Unhealthy Franka or
readiness records stop the replan even if later records recover. This changes
the previous G2 fixed two-second wait into a condition-based wait; it does not
change the hold, the validated arm mapping, or the safety gates. In the archived
14 complete G2 chunks, all four streams and delivery evidence had advanced
within 22–96 ms, but these observations do not guarantee a future duration.

Incremental audits capture append-only record references under the boundary
lock and deep-copy outside it so callbacks can continue. A full episode audit
still runs at finalization. After a successful chunk audit and CONTINUE, if
another replan is permitted, a
command-free readiness barrier waits up to the application-confirmation bound
(default 0.25 s) for all snapshot readiness reasons to clear. The 300 ms active
evidence TTL and publication safety checks remain unchanged. Timeout stops the
episode with the confirmed terminal hold in place, without another hold or
inference request.

When gripper actuation is enabled, a late CLOSE can be queued while an older
Move is cancelling. Before the next pre-hold and observation, the confirmed
terminal arm hold remains installed while the runtime waits for the gripper
replacement to be issued. This condition wait uses the configured gripper
timeout (default 3 seconds); a pending replacement, latched hand failure, or
lost arm readiness fails the episode without starting another inference or
publishing an extra arm hold. Each attempted wait records `gripper_transition`
start, deadline, check count, completion, and acceptance.

Each replan records `post_chunk_timing` with barrier start, deadline, check
count, completion, evidence-snapshot start/end, and validation completion in
monotonic nanoseconds. When another
replan is allowed, `readiness_reacquisition` records start/end monotonic times,
check count, acceptance, and the last readiness reasons. These are additive
diagnostic fields; outcome and action semantics are unchanged.

Unique terminal T4 confirmation and the post-T4 condition barrier
precede outcome evaluation. There is no
convergence wait. CONTINUE establishes a new pre-hold; SUCCESS, FAILURE and
ABORT terminate before another inference. `TaskOutcome` is a separate provider
interface. The installed validation provider returns CONTINUE and does not
infer task completion from images, policy responses or delivery acceptance.

`max_replans` (default 100) is a required positive finite safety bound, in
addition to bounded observation, policy transport and application waits.
Exhaustion is `safety_replan_limit_reached`, not task failure. The independent
test cap defaults to one chunk. A validated chunk with CONTINUE then stops as
`test_chunk_limit_reached`, before submitting replan 1. A terminal task outcome
takes precedence over the test cap.

Any failure stops the episode. Before/during action execution it requests one
safety-gated fresh measured-q abort hold and confirms it if possible. Remaining
actions are suppressed. A failed terminal hold is not retried. Partial action
counts and unsafe/unconfirmed holds are retained; no failed chunk permits
another inference.

## Gripper behavior and remaining task gap

The G1B adapter maps finite `action[7] > 0.5` to CLOSED (Franka Grasp at
width zero) and values at or below 0.5 to OPEN (Franka Move at maximum width).
It is enabled only by `physical-c1c2-gripper`; see
[the G1B integration](physical_g1b_gripper.md) for the validated conversion,
configured hand parameters and asynchronous action lifecycle. The arm-only
`physical-c1c2` entry point issues no hand commands. Physical hand actuation
does not establish that the policy grasps the object at the right time or
completes the task; the runtime still reports CONTINUE until a test cap or
safety abort.

## Progressive physical validation

With the existing supervised physical stack and policy server already ready,
use a new run identifier; existing directories are rejected:

```bash
make physical-c1c2 \
  PHYSICAL_RUN_ID=<unique-run-id> PHYSICAL_PROMPT='<task instruction>' \
  C1C2_MAX_EXECUTED_POLICY_CHUNKS=1
```

The command explicitly enables arm execution. It neither builds images nor
starts the server. Application confirmation defaults to 0.25 seconds; override
`C1C2_APPLICATION_CONFIRMATION_TIMEOUT` explicitly if required by the supervised
protocol. `C1C2_MAX_REPLANS` remains an independent hard bound. The main seed is
fixed in this command, unaffected by `DROID_POLICY_SEED` overrides.

For supervised G2 runs with the validated G1B hand adapter, select the
separately enabled entry point and set the chunk cap explicitly:

```bash
make physical-c1c2-gripper \
  PHYSICAL_RUN_ID=<unique-run-id> \
  PHYSICAL_PROMPT='Pick up the red object' \
  C1C2_MAX_EXECUTED_POLICY_CHUNKS=<reviewed-cap> \
  C1C2_STOP_AFTER_INFERENCE_REPLAN=
```

This enables hand commands, keeps the existing safety gates and records the
raw observation, response, action and runtime evidence in the output directory.
Changing the physical wrist-camera mount changes policy input, so compare
such runs as separate camera configurations rather than controlled policy
repeats.

Inspect Test 1 before increasing `C1C2_MAX_EXECUTED_POLICY_CHUNKS`. Require one
warm-up, one main request at index 0, eight fresh independently anchored safety
accepted and uniquely forwarded/applied actions, correct integer deadlines,
terminal hold at scheduled 8/15 with unique T4, zero gripper commands, no second
inference, no evidence gaps or health/ownership violations. Raising the cap
uses the same loop; no alternative execution implementation is involved.

### Physical Test 1 — 2026-09-17

C1-C2 arm-only Physical Test 1 passed using run
`c1c2_20260917T092308Z`. The process performed one discarded warm-up,
installed and uniquely confirmed a measured-q pre-inference hold, acquired
a new full observation whose four source timestamps postdated the hold
barrier, and submitted the first real request using policy seed `20260827`
and `replan_index = 0`.

The real request completed in `125.3 ms` client round-trip
(`114.5 ms` server inference; `95.2 ms` policy inference). Across sampled
controller-state telemetry during the `125.3 ms` inference interval,
`q_desired` remained unchanged. The runtime then executed exactly actions
`0..7` at independently scheduled 15 Hz deadlines using fresh measured-state
anchoring and applied a fresh measured-q terminal hold at tick 8. All ten
targets (pre-hold, eight policy actions, terminal hold) were uniquely
forwarded and applied in order.

Controller telemetry contained 7,640 unique period samples with mean
`1.000007 ms`, p95 `1.104759 ms`, p99 `1.163932 ms`, maximum
`1.197362 ms`, and zero samples above `1.5 ms`. No Franka health violation
was observed. The process stopped with
`termination_reason = test_chunk_limit_reached` after one validated replan;
the outcome provider remained `CONTINUE`.

This is a runtime-validation result, not manipulation-task success. Physical
gripper actuation and a validated task success/failure detector remain
unimplemented. Repeated replanning also remains to be physically validated.

Frozen evidence is archived under:

`docs/validation/2026-09-17_c1c2_test1/`

### Repeated-runtime cold-start validation — 2026-09-17

The repeated arm runtime was physically validated under a deliberately
reproduced cold inference-server condition using run
`c1c2_2chunk_cold_20260917T114502Z`.

The discarded startup warm-up incurred a 3.364 s client round trip. The two
subsequent action-producing requests completed in 127.8 ms and 114.2 ms,
respectively. The runtime completed `replan_index = 0` and `1`, executed
exactly 16 policy arm actions, applied two terminal measured-q holds, and
terminated with `task_outcome = continue` and
`termination_reason = test_chunk_limit_reached`.

The qualifying run followed two informative failed two-chunk attempts. The
first exposed full-history work inside the bounded T4 confirmation path and
motivated target-local cursor confirmation. A later cold-server run completed
chunk 0 but rejected the next pre-hold because controller-active evidence had
expired during post-chunk history processing. This motivated the shared
detached evidence snapshot and explicit command-free readiness reacquisition
barrier.

In the qualifying cold run, replan 0's post-chunk evidence snapshot required
211.6 ms and shared validation another 15.5 ms. The runtime then explicitly
reacquired current readiness before publishing the replan-1 pre-hold. The
final full-prefix delivery audit accepted all 20 targets with one continuous
controller application sequence, and both per-replan runtime-health checks
reported zero Franka health violations.

This closes the C1-C2 repeated arm-runtime validation gate. It does not
validate physical gripper actuation, manipulation-task success, a task
SUCCESS/FAILURE detector, or the observed task-level direction of policy
motion.

Frozen evidence is archived under:

`docs/validation/2026-09-17_c1c2_repeated_cold/`

### G2 condition-based wait and hand-transition validation — 2026-09-23

Supervised runs exercised the G2 incremental audit, post-terminal-T4
condition barrier and gripper transition wait. The one-chunk run completed
eight actions with final delivery accepted and its post-T4 barrier completed
in 49.36 ms. A two-chunk run after the hand-transition correction completed
16 actions with 20 targets accepted in the final delivery audit; its two
post-T4 barriers completed in 39.86 ms and 88.01 ms. These were bounded
runtime checks, not grasp-success checks.

The subsequent run `g2_wait_chunk15_20260923T092918Z` completed ten chunks
and stopped in replan 10 when a proposed target exceeded the existing
Jacobian-condition safety limit (17.013 >= 17.0). Its rejected target was
never published and an abort hold was applied. That run predates the final
audit correction for rejected, unpublished targets, so its final audit count
is not evidence of a delivery failure.

Two runs with the wrist camera moved to the corrected side of the gripper
used the same prompt, `Pick up the red object`, but different initial arm
positions. The following figures are from their archived `run.json` files:

| Run | Software outcome | Actions and hand commands | Delivery and wait evidence |
| --- | --- | --- | --- |
| `g2_wait_chunk18_20260923T124245Z` | 18 completed chunks; `test_chunk_limit_reached`, outcome CONTINUE | 144 applied actions; 9 hand commands issued | Final audit accepted 180/180 expected targets; all 18 per-chunk delivery and health checks accepted; post-T4 barriers 15.7–136.0 ms |
| `g2_wait_chunk18_20260923T124449Z` | Eight completed chunks, then replan 8/action 3 rejected; `runtime_abort` | 67 applied actions; 2 hand commands issued; confirmed abort hold | Proposed target condition 17.036 >= 17.0; final audit accepted 85/85 published targets and separately counted one rejected, unpublished target; eight completed post-T4 barriers 14.2–104.6 ms |

The first 18-chunk run began away from the usual home pose. The operator
observed smooth motion followed by retraction away from the desk without a
successful pickup. The second began at the usual home pose. The operator
observed smooth approach and a grasp attempt before the gripper was aligned
with the object; the log records a Grasp command but cannot establish why
the policy requested it. Earlier runs did not show the same frequency of
early closing. The corrected-side wrist view still does not include the
gripper as in the DROID examples; moving the camera again will produce a new
input configuration. Camera placement and starting pose both differ between
these runs, so neither a visual causal explanation nor task success follows
from the runtime audits. The 17.0 singularity threshold remains unchanged.

## Evidence

`run.json` separates `warmup`, `episode`, `replans`, `termination` and
`runtime_health`. Each replan records pre-hold, source barrier and observation,
inference timings and hold audit, action rows, terminal hold, delivery/health
validation, and outcome. Rows include q source/receive times, eligibility,
preparation time, deadline, actual publication, source/receive age at
publication, lateness, delta, q reference/target, safety decision, run/stamp,
forwarder/controller joins and unique application identity. `timing.jsonl`
retains raw telemetry. Request directories retain observations, responses,
actions, timing and the first main request's model-input audit.

Counters distinguish warm-up requests, main requests, completed validated
replans, scheduled actions, published actions, controller-applied actions,
gripper commands and confirmed terminal holds. `policy_actions_executed`
counts controller-applied evidence; ambiguous publication is not counted as
confirmed execution. `status=success` means software validation only;
`runtime_validation_status`, `termination.task_outcome` and
`termination.termination_reason` are separate. A capped CONTINUE is never
reported as manipulation-task success or failure.

Compare `warmup.client_round_trip_seconds` with each
`replans[i].inference.response.client_round_trip_seconds` separately. Warm-up
has no model-input audit; real index 0 does. No latency improvement has yet
been demonstrated by physical C1-C2 execution.

Automated verification: `tests/unit/test_physical_policy_execution.py` uses
deterministic fake time, sources, transport and controller evidence with the
real mapper, safety gate, request/response contracts and delivery validators.
Run it with the existing physical regression tests, then `make check`.
