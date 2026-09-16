# C1-B physical validation — prerecorded 15 Hz streaming playback

**Date:** 2026-09-16
**Result:** PASS

C1-B validates the physical streaming execution path for one prerecorded real
π0.5/DROID action chunk. It does not validate live policy inference, task success,
or policy performance.

## Protocol

The canonical `(15, 8)` policy chunk was loaded from:

`outputs/physical_pi05_droid_p1b/p1b_execute_20260914T113026Z/actions.npz`

Canonical SHA-256:

`0a9079ad0b429d16c18f1ef294cf66bced70d727db127a30cd9f7a5180d8dec0`

Actions 0–7 were executed at independently computed absolute 15 Hz deadlines.
At each action tick:

```text
q_target = q_measured + 0.2 * clip(action[:7], -1, 1)
```

The measured joint state was reacquired independently for every action. Targets
were therefore not accumulated from previous desired targets.

The eighth policy dimension was logged as the gripper output and was not sent to
the arm.

A separate terminal hold was scheduled at exactly tick 8 (`8/15 s`) and used its
own fresh measured joint state. It was not issued immediately after the action-7
publication call and was not treated as a ninth policy action.

The robot-side streaming stack used the pinned `fr3_lab_stack` C1-A2 implementation:

`9e535b665626cf8f3894fc4d2ef9136790abad92`

The active arm controller was
`fr3_lab_stack/StreamingJointImpedanceController`; the conventional
`fr3_arm_controller` was inactive. The persistent streaming forwarder was the sole
subscriber to `/fr3_streaming_joint_target` before the C1-B client started.

## Software verification before physical execution

Focused C1-B unit tests:

```text
26 tests
0 failures
```

`make compile` also passed.

The exact live, non-actuating `StreamingBoundary` preflight passed before motion:
controller readiness, Franka health, graph ownership, target-publisher ownership,
and state freshness all reported no rejection reasons.

## Physical run

Original output directory:

`outputs/physical_c1b/c1b_20260916T092848Z_12651`

Run UUID:

`b79d5e15-7bb1-493a-955d-a89024121709`

SAPS repository branch at execution:

`experiment/physical-fr3-saps`

SAPS repository base commit recorded in provenance:

`b442c551df9d2609bdcab8c3a7335b25497a039e`

The repository was intentionally dirty because the C1-B implementation and
documentation were the uncommitted changes being physically validated.

The run completed with:

```text
status: published
exit_code: 0
delivery_validation.accepted: true
runtime_health_validation.accepted: true
```

All eight policy actions and the terminal hold passed the admission gates.

## Delivery and application

All nine commands were uniquely forwarded, accepted by the streaming controller,
and installed:

```text
controller sequences: 1, 2, 3, 4, 5, 6, 7, 8, 9
target statuses:      applied for all 9
```

For every command:

- forwarder records: 1
- controller callback records: 1
- controller application records: 1
- forwarded target matched installed `q_desired`
- controller instance and activation remained unchanged

Observed evidence integrity:

```text
controller evidence-ID gaps:      0
forwarder evidence-ID gaps:       0
dropped period samples:           0
dropped application records:      0
controller publication errors:    0
unmatched forwarder records:      0
unmatched callback records:       0
unmatched application records:    0
```

## 15 Hz scheduling

Nominal absolute deadline offsets were:

```text
action 0        0.000000 ms
action 1       66.666667 ms
action 2      133.333333 ms
action 3      200.000000 ms
action 4      266.666667 ms
action 5      333.333333 ms
action 6      400.000000 ms
action 7      466.666667 ms
terminal hold 533.333333 ms
```

Observed publish-call T0 lateness relative to those deadlines:

```text
minimum: 1.123196 ms
mean:    1.403490 ms
maximum: 1.862514 ms
```

Observed consecutive publication intervals ranged from:

```text
minimum: 66.179378 ms
maximum: 67.373406 ms
```

Relative to the nominal 66.666667 ms period, the interval error ranged from
approximately -0.487289 ms to +0.706739 ms.

Actual action-0 T0 to terminal-hold T0:

```text
observed: 533.043667 ms
nominal:  533.333333 ms
error:     -0.289666 ms
```

The approximately common positive scheduling offset therefore did not accumulate
across the eight-action window.

## Target-application timing

T0 is the SAPS client publication-call start and T4 is the first successful
software installation of the corresponding equilibrium inside the 1 kHz controller.
T4 does not represent actuator response or physical convergence.

Across the nine commands:

```text
T0 -> T4 minimum: 0.439794 ms
T0 -> T4 mean:    0.966763 ms
T0 -> T4 maximum: 1.407027 ms
```

## Fresh-state anchoring

Measured-q source age at mapping:

```text
minimum: 0.420117 ms
maximum: 1.109359 ms
```

Measured-q source age immediately before publication:

```text
minimum: 1.540246 ms
maximum: 2.647501 ms
```

Thus every policy action and the terminal hold used a recent independently
observed joint state.

## Controller-period evidence

The capture contained 2520 controller periods from one controller activation:

```text
nominal:       1.000000 ms
mean:          0.999989 ms
median:        0.999443 ms
p95:           1.159463 ms
p99:           1.214657 ms
maximum:       1.268126 ms
periods >1.5:  0
periods >2.0:  0
```

No RT timing-evidence overflow was observed.

## Robot health

No Franka health violation was recorded during the capture.

The full runtime-health validation reported:

```text
FrankaRobotState samples:              2536
Franka health violations:                 0
post-hold FrankaRobotState samples:    2002
post-hold controller-readiness samples:  20
post-hold controller-state samples:     189
post-hold measured-joint samples:      2003
```

All post-run manual `current_errors` fields were false and all collision indicators
were zero.

## Motion interpretation

The terminal hold was intentionally issued at the fixed `8/15 s` boundary rather
than waiting for convergence. The robot was therefore still moving when the hold
was created.

Maximum absolute measured joint velocity among the action reference states was:

```text
0.332704 rad/s
```

At the terminal-hold reference state the maximum absolute measured velocity was:

```text
0.320950 rad/s
```

This is expected for this protocol and must not be interpreted as physical settling
by the terminal-hold deadline.

## Conclusion

C1-B passes its intended validation claim:

> A prerecorded real π0.5/DROID eight-action chunk can be physically executed
> through the persistent streaming impedance path using fresh measured-state
> anchoring and independently scheduled 15 Hz deadlines. All eight policy targets
> and the terminal measured-q hold were uniquely forwarded, accepted, and installed
> in order, with no observed evidence loss, controller-period excursion above
> 1.5 ms, or Franka health violation.

This result validates prerecorded moving 15 Hz execution. It does **not** validate
live π0.5 inference, inference-time holding, task success, or physical convergence
at the terminal-hold boundary. Those belong to C1-C and later evaluation.

## Evidence

Committed representative evidence:

- `playback.json` — per-command mappings, scheduling, delivery/application joins,
  timing summary, and health-validation summary.
- `provenance.json` — software/run identity captured by the execution script.
- `SHA256SUMS` — integrity hashes for the committed summary files and the original
  local raw run artifacts.

The original `timing.jsonl` and other generated run artifacts remain under the
original `outputs/` directory and are not committed as source files.

Recorded artifact hashes:

```text
6902ecb8852344e9021a63b22b004a7eaa41a3c9c3d8466fe27b6a8269897d41  playback.json
1bb77e9b7a28b19d70e65ce2a347d7f23c9b498a843a0bfac27e1032187050bb  timing.jsonl
eccf5292fb5039c66f62c55cbbe1eb4819c7808eacbc69fbb89b8fb5255cb599  provenance.json
0a9079ad0b429d16c18f1ef294cf66bced70d727db127a30cd9f7a5180d8dec0  actions.npz
70ebca8364603cc7b0279907e10ca9be2456dc0996b2898d309462f7815fc4dd  robot_description.urdf
2d1c61ba38d3a08a5c8036e196d7608f98a1b2c0ca77f675bbe2478229bfa1d2  joint_limits.json
```
