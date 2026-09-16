# C1-B prerecorded streaming playback
**Status:** C1-B physical validation passed on 2026-09-16. The validated run is
documented in [`validation/2026-09-16_c1b_physical/README.md`](validation/2026-09-16_c1b_physical/README.md).
C1-C, live π0.5 inference with inference-time holding, remains unvalidated.

This software gate plays actions 0–7 from
`outputs/physical_pi05_droid_p1b/p1b_execute_20260914T113026Z/actions.npz`.
The archive must have SHA-256
`0a9079ad0b429d16c18f1ef294cf66bced70d727db127a30cd9f7a5180d8dec0`.
The full original `(15, 8)` archive is copied unchanged into each exclusive output
directory. No inference occurs. The historical P1-B MoveIt entry point is unchanged.

Each tick independently snapshots the latest `/franka/joint_states`, preserving
its original double positions, source stamp and receive monotonic timestamp. The
existing `single_action.safety_gate` applies the authoritative mapping:

```text
q_target[k] = q_measured[k] + 0.2 * clip(action[k, :7], -1, 1)
```

Joint limits come from the deployed robot description for execution. The existing
100 ms source/receive freshness, 0.1 rad joint margin and singularity gates remain
unchanged. Controller evidence must identify the streaming controller as active
within 300 ms of request dispatch; its 39-field state must be active and fresh
within 100 ms. Fresh FrankaRobotState evidence is also required at every admission and publication boundary. Robot mode must be IDLE or MOVE, no current Franka error may be active, collision indicators must be clear and finite, and control_command_success_rate must be finite. The streaming controller must be the expected fr3_lab_stack/StreamingJointImpedanceController, own exactly the seven FR3 effort command interfaces, and no other active controller may claim or require an FR3 arm command interface. The C1-B node must also be the sole publisher on /fr3_streaming_joint_target. Invalid observation callbacks and observed target rejections revoke
admission. Freshness/readiness are rechecked before publication. The seventh arm
joint is the last commanded dimension; the eighth policy dimension is logged only.

C1-B intentionally uses **elapsed playback time** for the existing one-second
action-age gate. It does not claim that prerecorded actions are fresh inference
for the current scene. This is an open-loop timing experiment, not a replanning or
policy-success evaluation. Operator assessment of the scene/start pose remains
necessary for any later physical experiment.

The absolute CLOCK_MONOTONIC schedule computes every deadline independently
from the rational 15 Hz cadence:

offset_ns(k) = round(k * 1e9 / 15)

`PERIOD_NS = round(1e9 / 15)` is retained only as the threshold for detecting
that a complete action interval has already been missed; it is not multiplied
to generate the schedule.

| Event | Offset (ms) |
| --- | ---: |
| action 0 | 0.000000 |
| action 1 | 66.666667 |
| action 2 | 133.333333 |
| action 3 | 200.000000 |
| action 4 | 266.666667 |
| action 5 | 333.333333 |
| action 6 | 400.000000 |
| action 7 | 466.666667 |
| terminal hold | 533.333333 |

The hold has no action index and uses its own fresh measured q. It is scheduled
at tick eight, never immediately as a follow-on call to action seven. Ordinary Python/ROS
scheduling cannot guarantee exact actual publication times; deadlines, actual T0
and lateness are distinct evidence fields. A complete missed interval aborts
instead of emitting catch-up targets. There is no physical-convergence/T4 wait.

A rejection/publication failure stops all subsequent policy actions and records
`aborted_before_action_k`. A separately labelled `abort_hold` uses a new state and
the same safety gates, with one attempt only. An unsafe hold is not published.
A failed terminal hold is reported separately and is not retried. A failed
publication may have reached middleware; its T0 and error remain recorded.

## Software dry run

Supply a JSON array of **nine independently chosen diagnostic q[7] rows**, one for
each action and the hold. These states are software fixtures, not measured hardware
evidence. Dry run uses a virtual clock and constructs no ROS node or publisher;
actual T0 and latency fields remain null. Limits come from the saved P1-B URDF.

```bash
PYTHONPATH=src python3 scripts/physical_prerecorded_chunk.py \
  --dry-q-refs outputs/physical_c1b_dry_refs_20260916.json \
  --output-dir outputs/physical_c1b/dry_UNIQUE_RUN
```

The local example fixture varies measured P1-B q's first joint by `0.001 * k` rad
for k=0..8. It varies states only; the canonical policy actions are unchanged.
Generated diagnostics are not formal experimental results.

## Physical execution command

Use the already commissioned, active streaming controller and forwarder on the
controller host. This command performs no bringup or controller switching. The
read-only source checkout must be clean at
`9e535b665626cf8f3894fc4d2ef9136790abad92`; deployed binaries must correspond to that
pin. Source, controller and forwarder ROS clocks must share a clock domain.

The following command was executed once under operator supervision for the
validated C1-B run. The streaming impedance controller and forwarder were already
commissioned and active; the command itself performs no controller switching.

```bash
cd /home/hvl-robotics2404/saps-openpi-replication
source /opt/ros/jazzy/setup.bash
source /home/hvl-robotics2404/franka_ros2_ws/install/setup.bash
PYTHONPATH="$PWD/src:$PYTHONPATH" .venv-physical/bin/python \
  scripts/physical_prerecorded_chunk.py --execute \
  --output-dir "outputs/physical_c1b/c1b_$(date -u +%Y%m%dT%H%M%S)_$$"
```

Targets are `JointState` messages on `/fr3_streaming_joint_target`, with exactly
`fr3_joint1` through `fr3_joint7`, reliable/volatile KEEP_LAST depth 1. A UUID in
`header.frame_id` and unique positive ROS stamps preserve C1-A2 correlation.

`playback.json` contains per-action mapping, reference ages, safety, scheduling,
publication and joined timing evidence, plus actual action-0 T0 to terminal-hold
T0 duration. `timing.jsonl` retains raw forwarder/controller JSON strings unchanged,
decoded evidence, measured q/dq, controller state and readiness observations.
Recording is in memory during playback, written after the two-second drain.
Abrupt process/host termination can therefore lose evidence; this is not a
crash-durable recorder. `provenance.json`, `actions.npz`, `robot_description.urdf`
and `joint_limits.json` preserve inputs and version identity.

The pinned C1-A2 analyzer is read directly without writing bytecode to its source
checkout. It joins run/stamp and controller instance/activation/sequence, rejects
cross-clock latency arithmetic and reports evidence loss/unresolved application.
`published` describes client publication calls only. Physical C1-B acceptance
additionally requires `delivery_validation.accepted == true` and
`runtime_health_validation.accepted == true`. The 2026-09-16 validation satisfied
both conditions: all nine targets were applied in order with no observed evidence
loss or Franka health violation. T4 remains equilibrium installation, not physical
convergence. Live inference and inference-time holding remain unvalidated.
