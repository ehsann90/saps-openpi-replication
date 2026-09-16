# C1-C1 live inference under measured-q hold

**Physical status: PASS on 2026-09-16.** The validated run and representative
evidence are archived in
[2026-09-16 C1-C1 physical validation](validation/2026-09-16_c1c1_physical/README.md).
This gate connects
validated C1-B streaming control with the frozen P0 DROID observation/inference
contract. It tests equilibrium retention during one inference, with no policy
action execution, gripper command, task-success criterion or controller switching.
The existing P0 command remains subscriber-only. C1-B behavior is unchanged.

The accepted physical claim is that one live π0.5/DROID request completed under
an explicitly installed measured-q equilibrium: pre-hold T4 preceded inference,
no target was published during inference, and all 307 sampled controller states
retained sequence 1 with zero desired-q span across the 3.23787645 s interval.
A fresh post-inference measured-q hold was then applied. Pre/post T0→T4 were
1.495998/1.019387 ms. Maximum sampled tracking error was 0.038486 mrad; maximum
sampled absolute joint velocity was 0.003944331 rad/s. These are sampled telemetry
metrics, not continuous-time maxima. Both final validators and the publication
audit passed; 7,212 Franka samples contained no health violation. The 7,380
controller periods had mean 0.999998 ms, p95 1.097023 ms, p99 1.139993 ms and
maximum 1.196238 ms, with no periods above 1.5 ms or reported timing-evidence loss.
Zero policy actions and zero gripper commands were executed. This makes no
task-success claim and does not validate C1-C2/live-action execution.

The observed first live request took 3.238 s end-to-end, with first-request model
auditing/cold-start context; it is not normal warm latency. As described in the
[P0 audit documentation](physical_pi05_p0.md), auditing adds copying and device
synchronization and must be analyzed separately from later requests. A one-time
discarded warm-up request will be considered as C1-C2 startup work so the first
physically executed chunk is not generated from a cold-start observation. No
warm-up mitigation is implemented as part of C1-C1.

`configs/physical_pi05_fr3.json` retains historical P0 milestone and
execution-disabled metadata because it supplies the observation/policy contract.
The top-level C1-C1 run identity is authoritative for this experiment.

The new entry point is `scripts/physical_live_inference_hold.py`. After separate
manual review and physical authorization, run it on the controller host with the
already commissioned streaming controller/forwarder and pinned policy server.
It does not start those services. Both arm commands are explicitly
`q_target = q_measured`, retaining measured double precision. Safety admission
uses the existing C1-B zero-displacement hold path and unchanged limits,
singularity, state freshness, controller ownership and Franka-health checks.

Example command template (not executed during software verification):

```bash
PYTHONPATH="$PWD/src:$PYTHONPATH" .venv-physical/bin/python \
  scripts/physical_live_inference_hold.py --execute \
  --prompt 'EXPLICIT_OPERATOR_PROMPT' \
  --application-confirmation-timeout SECONDS \
  --output-dir outputs/physical_c1c1/UNIQUE_RUN
```

`--application-confirmation-timeout` is required, positive and finite. It bounds
waiting for baseline controller identity and each hold's complete T4 evidence;
it is not a performance qualification threshold. Choose and record it during
manual review. The pinned controller publishes timing batches every 20 ms;
confirmation polls every 10 ms. Discovery and post-hold evidence drain each use
the existing C1-B two-second convention. Observation and policy timeouts retain
P0 defaults of 30 and 120 seconds. Observation freshness remains 0.5 seconds,
including a recheck immediately before inference: a long confirmation wait can
expire the observation and abort without submitting a request. No observation
replacement or inference retry occurs after the pre-hold.

The process validates provenance and server identity, captures a fresh complete
DROID observation and native/canonical image evidence, then acquires a newly
received measured joint state for `pre_inference_hold`. It publishes the hold
through `StreamingBoundary`. Inference cannot begin until the unchanged pinned
C1-A2 analyzer and C1-B delivery checks establish one forwarded, accepted and
applied target, matching source stamp/run, desired q, controller instance and
activation, and same-clock ordered T0–T4 evidence. T4 means equilibrium
installation, not physical convergence.

One seeded request follows (`policy_episode_seed` from the CLI, `replan_index=0`,
model-input audit enabled). The synchronous inference path has no command
interface. Its native `[15,8]` actions, policy/server timing and sampling identity
are saved using the P0 helpers. A newly received joint sample after inference
provides `post_inference_hold`, which must also be uniquely applied. The
observation node uses an explicit executor transferred to a background thread
after observation assembly; the streaming boundary keeps its separate background
executor throughout blocking inference and the post-hold drain.

A failed pre-hold gate or confirmation prevents inference. A failed inference,
invalid response or artifact/audit failure permits at most one separately labelled
fresh `abort_hold`, admitted through the same safety gate. Unsafe abort holds are
not published. Confirmation failure is recorded without retry. A failed post-hold
is not retried. Any failure makes the gate unsuccessful; no recovery or resumption
is attempted. The persistent controller retains its previously installed target
until another target is actually installed; software does not replace independent
robot safety systems.

Each exclusive output directory contains `provenance.json`, `start.json`, deployed
`robot_description.urdf`, `joint_limits.json`, `request_0000/` with the unchanged P0
request/response/image/model-audit artifacts, raw `timing.jsonl`, and `run.json`.
Artifacts completed before an ordinary failure are retained. As in C1-B, streaming
evidence is buffered in memory until shutdown; abrupt host/process termination
can lose evidence. It must never be treated as a successful run.

`run.json` distinguishes both holds from `policy_inference`, records hold source
stamps, measured-state source/receive timestamps, T0–T4 and live confirmation,
and nanosecond monotonic inference start/end times. Final offline analysis remains
mandatory. `publication_audit` joins every recorded `sent` target to the two
labelled holds and checks that no publication falls inside inference. Exactly two
publications, consecutive controller sequences, consistent activation, no
unmatched/ambiguous/rejected/unresolved evidence, no reported evidence gaps or
overflow, and healthy evidence after post-hold T4 are required. This retains C1-A2's stated
capture-boundary limitations; it does not claim to observe losses outside capture.

`status=success` and exit code zero require all software acceptance checks,
including `pre_hold_T4 <= inference_request_start`, a fresh independent post-hold,
exactly one completed request, `policy_actions_executed=0` and
`gripper_commands_issued=0`. Repository provenance must descend from validated
C1-B commit `dc5ce5e1ee038d793f7b672cc72a5fffce8d06c2`; current revision and dirty
state are recorded. OpenPI must be clean at
`15a9616a00943ada6c20a0f158e3adb39df2ccac`, and the analyzer/robot-stack reference
clean at `9e535b665626cf8f3894fc4d2ef9136790abad92`. Deployed binaries must match that
reference. Controller/source/forwarder monotonic clocks must share host, boot and
time namespace, as established by the pinned analyzer. The archived result
validates the recorded deployment and run; later physical runs still require
separate authorization and verification of these assumptions.
