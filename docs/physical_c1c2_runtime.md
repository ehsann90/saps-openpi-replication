# C1-C2 repeated physical arm runtime

Implemented from frozen warm-up commit
`cba260c97a19db5aadd99052a16c94cccec3413b`. No physical execution has been
performed for this implementation. OpenPI remains pinned at `15a9616a` and
fr3_lab_stack at `9e535b6`. C1-B, C1-C1 and C1-C2-W commands retain their
existing behavior. This is arm execution only, not complete manipulation-task
execution: gripper commands are disabled and no task-success detector exists.

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
the drain to include delayed evidence.

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

Unique terminal T4 confirmation and the unchanged two-second evidence drain
precede delivery/health validation and outcome evaluation. There is no
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

## Gripper evidence and remaining gap

Inspection of pinned OpenPI, existing SAPS and the local Franka interface found:

* `third_party/openpi/src/openpi/training/config.py`, `pi05_droid`: horizon 15,
  `DroidInputs`/`DroidOutputs`; no relative gripper transform.
* `third_party/openpi/src/openpi/policies/droid_policy.py`, `DroidOutputs`:
  returns the first eight output dimensions, with no closure conversion.
* `third_party/openpi/examples/droid/convert_droid_data_to_lerobot.py`: the
  action combines seven `joint_velocity` values and `gripper_position`.
  `src/openpi/training/droid_rlds_dataset.py` likewise takes dimension 7 from
  `action_dict/gripper_position`. The repository dataset manifest is DROID raw
  1.0.1 in `configs/droid_m1_sample.json`.
* `third_party/openpi/examples/droid/main.py:80` selects gripper **position**
  mode; lines 140–150 threshold at `> 0.5` into 1 or 0 and clip before
  `RobotEnv.step`. This supports absolute position intent, not a relative
  gripper increment. The backend `droid.robot_env` is not vendored/pinned here;
  these sources alone do not establish its physical width conversion.
* `src/saps/physical/live_observation.py` maps observed finger width to
  `1 - width / maximum_width`. This is an input observation convention, not
  a validated output command adapter. `embodiment.py`'s historical 0=open,
  1=closed validator and `discrete_verifier.py`'s threshold labels do not
  establish current actuation semantics.
* Existing SAPS subscribes to `/franka_gripper/joint_states` but has no current
  physical gripper action client. The inspected local
  `franka_gripper/src/gripper_action_server.cpp` exposes `~/move` and `~/grasp`,
  passing width/speed and width/speed/force/epsilon to libfranka respectively;
  `~/gripper_action` uses `2 * command.position` as total width. These are
  distinct ROS interfaces, not interchangeable normalized policy inputs.

The earlier DROID document states normalized absolute closure and describes
the binary example. However, a pinned DROID backend width/polarity contract and
a validated FR3 command conversion (including Move versus Grasp, width,
speed/force/epsilon, cancellation and action-deadline behavior) are missing.
Consequently this runtime creates no gripper action client, does not guess a
mapping, and records zero gripper commands. Those contracts and their tests
must be established before enabling gripper actuation.

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
