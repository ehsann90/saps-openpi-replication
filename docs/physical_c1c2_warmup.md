# C1-C2 startup warm-up prerequisite

The `physical_c1c2_warmup` runner performs one discarded inference per physical
process invocation, with a distinct warm-up seed, zero actuation, and a fresh
observation afterward. The real episode begins at the main seed / replan index
0. This runner stops after acquiring that observation; it submits no real
request and does not validate C1-C2 policy-action execution.

Warm-up is a runtime-initialization step and is excluded from experimental
episode timing and replanning counts. It does not detect server warmth or repeat
on replans. This runner does not itself quantify cold-start reduction, and the
first real request still incurs its model-input audit overhead.

For a separately supervised, zero-actuation validation with sources and the
policy server already available:

```bash
make physical-c1c2-warmup \
  PHYSICAL_RUN_ID=<unique-run-id> PHYSICAL_PROMPT='<instruction>'
```

The target sources ROS Jazzy and `FRANKA_ROS2_INSTALL/setup.bash`, uses
`.venv-physical/bin/python`, and includes both `src` and the pinned OpenPI client
package in `PYTHONPATH`. It starts no controller or policy service. The dedicated
script accepts the same observation/policy timeout, host, port, configuration,
and provenance directory options as P0; it has no execution or request-count
option. Existing P0 and C1-C1 commands, audits and output schemas are unchanged.
C1-C1's validated measured-q holds do not participate in this runner.

## Seed, lifecycle and observation contracts

`PHYSICAL_WARMUP_POLICY_SEED ?= 20260917` is exported only for the new Make target.
The script also defaults to the module constant `20260917`. The main seed stays
`DROID_POLICY_SEED ?= 20260827`. Both values are passed unchanged. `PolicyWarmup`
rejects equal seeds during construction, before transport creation, and checks
again immediately before inference. Make overrides cannot bypass this check.

The subscriber-only process lifecycle owns one `PolicyWarmup`. Its attempted
flag is set before validation/submission and rejects a second call, including
after failure. No automatic retry occurs. A future C1-C2 lifecycle must retain
this same object across replans; action execution is not integrated here.
There was no existing per-process warm-up state object to reuse.

`PolicyWarmup.run()` receives only an observation, policy client, artifact path,
configuration and clocks. It reuses the command-incapable `infer_live_request()`
with the independent seed, index `0` and `audit_model_input=False`. The helper's
new optional audit override defaults to the previous `index == 0` rule, so the
first real request retains `True`, as do P0 and C1-C1. At the wire boundary,
OpenPiDroidPolicy represents false by omitting the optional audit flag.

The existing client validates finite floating native actions and echoed seed,
index and protocol. Existing live checks enforce `[15,8]`, pinned config,
checkpoint, OpenPI commit, advertised horizon, noise digest and timing evidence.
The returned chunk is archived as diagnostic evidence and discarded from
runtime use; no actions leave the warm-up helper. The ROS lifecycle uses
`SubscriptionBoundary` and audits its owned interfaces. Neither the helper nor
runner creates an arm publisher, gripper client or streaming boundary.

`prepare_warmed_observation()` retains the same `CameraPairGate` across the two
acquisitions. Both camera source stamps must strictly advance. In addition,
all four source timestamps (wrist, exterior, joints, gripper) must be strictly
greater than the warm-up response's `response_completed_ros_seconds`, recorded
as `post_warmup_source_barrier_ros_seconds`. Receive times cannot satisfy this
barrier: queued messages originating before completion are rejected even if
received afterward. The established
collector assembly checks source and receive freshness for cameras, joints and
gripper, plus skew. Acquisition timeout or failed validation terminates the
process without a real inference or fallback to the warm-up observation.

## Separate evidence and accounting

Artifacts are exclusively allocated at `outputs/physical_c1c2_warmup/<run-id>`:

- `warmup/request_0000/`: observation, native images, request, call timing and
  validated native response evidence; no model-input audit.
- `request_0000/`: newly acquired main-seed observation and request metadata.
  There is no response here because validation stops before real inference.
- `start.json`, `run.json`: existing lifecycle/provenance envelope with the new
  milestone and a separate `warmup` object recording attempted/completed status,
  request count, seed, index, audit flag, returned shape, discard status, zero
  action/target/gripper counts, and exclusion from episode timing.

Warm-up records request start and response completion monotonic timestamps,
client round trip, server/model timing dictionaries (`infer_ms`), and observation
ages at request and response. Failed transport calls retain `request_timing.json`
and monotonic call-boundary evidence; unavailable response metrics are absent.
`wall_clock_duration_seconds` covers the whole validation process, including
preflight and warm-up; it is not an experimental episode duration or latency
summary. No episode inference-latency summary is generated by this runner.

`completed_requests` counts successful experimental/shadow requests in the
existing paths: `run_live_loop` sets it to `index + 1`; C1-C1 sets it to `1` after
its one successful inference. Warm-up touches neither counter nor any policy
replan sequence. This validation's top-level `requested_requests` and
`completed_requests` remain `0`, as do `policy_actions_executed` and
`gripper_commands_issued`. `first_real_replan_index: 0` records the prepared
request's index, not a claim that a real request was executed.

The historical P0 observation configuration metadata remains unchanged; the
top-level `physical_c1c2_warmup` milestone identifies this validation. Existing
P0/C1-C1 `request.json`, `response.json`, `run.json` and accounting meanings are
preserved. Unit fixtures cover the first real request's seed/index/audit and
failure paths; they are not physical or C1-C2 execution evidence.

## Physical validation

C1-C2-W startup warm-up prerequisite passed physical validation on 2026-09-17 using run
`c1c2w_20260917T073653Z`. The warm-up issued exactly one π0.5/DROID
request using policy seed `20260917` and `replan_index = 0`, while the
prepared real episode retained policy seed `20260827` and
`replan_index = 0`. The returned warm-up action chunk had native shape
`[15, 8]` and was discarded without publishing an arm target, issuing a
gripper command, or otherwise invoking a robot command interface.

The accepted run also validated the post-warm-up source-time barrier. The
warm-up response completed at ROS time `1789630620.3672678`. The wrist
image, exterior image, joint state, and gripper state used for the next
observation all had source timestamps strictly greater than this barrier.
An earlier candidate observation was rejected because this condition was
not yet satisfied. Existing observation-age, cross-source-skew, and
camera-pair advancement checks remained active.

The accepted post-warm-up observation had a maximum source age of
approximately `43.9 ms` and a cross-source skew of `42.8 ms`.

The accepted warm-up itself took `118.9 ms` client round-trip
(`113.0 ms` server inference; `91.2 ms` policy inference). Because the
persistent policy server had already serviced a preceding request, this
run does not quantify cold-start reduction. It validates the startup
protocol and its zero-actuation boundary. First real post-warm-up
inference latency and live policy-action execution remain C1-C2 work.

Frozen validation evidence is archived under:

`docs/validation/2026-09-17_c1c2_warmup/`
