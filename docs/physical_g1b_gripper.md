# G1B DROID gripper integration

## Status

The DROID-to-FR3 gripper semantics, lower-level Franka Hand lifecycle, SAPS
adapter, and physical grasp/release path are validated. **G1B is closed.**

No saved policy-action replay is required to close this gate. The strict
`> 0.5` policy decision boundary is validated deterministically in software,
while the supervised physical run validates the corresponding OPEN/CLOSED
realization and grasp-release lifecycle on the FR3 hand. Using a saved policy
trajectory here would unnecessarily couple gripper validation to the separate
object-directed policy-behavior problem addressed next in G2.

No manipulation task or task-success detector is part of G1B.

## Accepted policy contract

Pinned OpenPI/DROID inspection established binary absolute closure intent:

```text
action[7] > 0.5  -> CLOSED
action[7] <= 0.5 -> OPEN
```

Nonfinite policy values are rejected.

The policy adapter deliberately does not receive object width. The physical
realization is:

```text
OPEN   -> Franka Move(maximum_width)
CLOSED -> Franka Grasp(width=0)
```

The configured maximum finger position is `0.04 m`, giving `0.08 m` total hand
width.

The physical G1B validation used fixed embodiment parameters:

```text
maximum width:        0.080 m
Move/Grasp speed:     0.100 m/s
Grasp target width:   0.000 m
Grasp force:          20.0 N
epsilon inner:        0.001 m
epsilon outer:        0.080 m
lifecycle timeout:    3.0 s
grasp dwell:          3.0 s
```

These are robot/adapter configuration values, not object-specific task inputs.

## Repository boundary

`fr3_lab_stack` owns the policy-independent physical Franka Hand lifecycle.

Frozen implementation:

```text
frdedynamics/fr3_lab_stack
4bb6cdc58839dcdd94acbe1633b8f361676a4eb6
Add asynchronous Franka Hand Move and Grasp runtime
```

Gripper-enabled SAPS execution is pinned to that exact clean commit. The historical
arm-only C1-C2 runtime retains its original `9e535b6` arm-baseline pin.

The reusable `RosFrankaHand` library attaches to a caller-owned ROS node and
executor. It owns:

- Move and Grasp ROS action clients;
- explicit Stop service handling;
- asynchronous goal/result lifecycle;
- replacement and cancellation ordering;
- duplicate suppression;
- grasp-hold state;
- nonterminal Stop before release Move;
- terminal cleanup Stop;
- latched errors;
- detached lifecycle evidence.

SAPS owns:

- strict `action[7] > 0.5` DROID thresholding;
- OPEN/CLOSED policy semantics;
- fixed physical Grasp parameters supplied to the hand wrapper;
- measured-before evidence;
- command integration with the physical policy loop;
- inference-time command suppression checks;
- validation and episode evidence.

`franka_ros2` remains a read-only dependency. No upstream change was required
for G1B.

Inspected local source revision:

```text
1369a2cb200d0f7b3da11c7728c7ca2e6975ca00
```

Relevant live interfaces:

| Endpoint | Type |
| --- | --- |
| `/franka_gripper/move` | `franka_msgs/action/Move` |
| `/franka_gripper/grasp` | `franka_msgs/action/Grasp` |
| `/franka_gripper/stop` | `std_srvs/srv/Trigger` |
| `/franka_gripper/joint_states` | `sensor_msgs/msg/JointState` |

`control_msgs/action/GripperCommand` is not used for the G1B adapter.

## Lifecycle semantics

The hand wrapper never assumes that sending another goal preempts an existing
one. Replacement waits for cancellation handling and the previous terminal
result before issuing the latest queued command.

Move and Grasp are different commands even at the same target width.

A successful Grasp sets the lower-level hand state to a held grasp. When a later
OPEN intent arrives, the wrapper performs:

```text
successful Grasp
→ queued Move(open)
→ release_stop_requested
→ release_stop_result(success=true)
→ Move(open)
```

The opening Move is therefore not allowed to overlap the force-holding grasp.

Rejected goals, failed action results, lifecycle timeout, transport exception,
failed release Stop, or failed terminal Stop latch failure and suppress unsafe
continuation.

## Software validation

### fr3_lab_stack

The lower-level Move+Grasp implementation passed:

```text
16 focused Franka Hand tests
120 total package tests
0 errors
0 failures
0 skipped
```

### SAPS

The SAPS gripper adapter, ROS integration, physical validator, and existing
physical policy regressions passed:

```text
351 tests
compilation passed
```

The final test count is one lower than the preceding revision because the old
two-test obstruction/reopen validator was replaced with one object-agnostic
grasp-release validator test.

## Isolated physical validation

Qualifying run:

```text
outputs/physical_franka_hand_g1_b/g1b_grasp_release_20260918_144453
```

Frozen archive:

```text
docs/validation/2026-09-18_g1b_grasp_release/
```

Run UUID:

```text
a1319691-4dda-4707-8cfe-4d804fb0862d
```

The arm remained on the existing measured-q hold and executed zero policy arm
actions.

The physical sequence was:

```text
Move(open)
→ Grasp(width=0, speed=0.1 m/s, force=20 N,
        epsilon_inner=0.001 m, epsilon_outer=0.08 m)
→ operator releases the object
→ unsupported object retention for ~3 s
→ Stop
→ Move(open)
→ terminal cleanup Stop
```

Observed results:

```text
initial open:             79.975 mm
grasp width:              46.252 mm
unsupported dwell:        3.0046 s
dwell width span:         0.069 mm
final open:               79.799 mm
release Stop:             success
Franka health violations: 0
arm hold unchanged:       true
```

The object remained physically retained without operator support for the full
dwell.

The measured 46.252 mm width is an observed physical result only. It was never
provided to the adapter, wrapper, or validator as object-width knowledge.

This validates the semantic conversion and physical grasp/release mechanism.
A saved policy-gripper sequence remains an optional diagnostic, not a G1B
acceptance requirement.

## Physical validation entry point

The supervised gripper-only entry point is:

```text
tests/manual/physical_gripper_validation.py
```

The current supported modes are:

```text
grasp-release
archived
```

Because ROS 2 Jazzy's `rclpy` is supplied by the system Python installation,
preserve the ROS environment's existing `PYTHONPATH` and prepend the SAPS source
tree rather than replacing it.

Example:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash

cd ~/saps-openpi-replication

OUT=outputs/physical_g1b/grasp_release_$(date +%Y%m%d_%H%M%S)

PYTHONPATH="$PWD/src:${PYTHONPATH:-}" \
python3 \
tests/manual/physical_gripper_validation.py \
    --execute \
    --mode grasp-release \
    --output-dir "$OUT"
```

Do not use the old `open-close-open`, `interrupted-close`, `reopen`,
`--blocking-width-mm`, or controlled-obstruction workflow. Those belonged to the
discarded Move-only design.

## Gate closure

G1B requires no additional physical replay.

The strict threshold semantics are covered by software tests including the
boundary at `0.5`, values immediately above the boundary, and nonfinite-input
rejection. The physical `grasp-release` validation then exercises the same SAPS
adapter with deterministic OPEN/CLOSED inputs and verifies their actual Franka
Hand realization.

A saved `[15,8]` policy action chunk may still be useful later as a diagnostic
or during task-level execution, but it is not a G1B acceptance criterion. In the
current project state, requiring it here would confound gripper integration with
the separate G2 question of whether the physical policy trajectory is
task-directed.

## Scope

Validated:

- DROID binary absolute gripper semantics;
- strict thresholding and nonfinite rejection;
- no privileged object-width input;
- OPEN -> Move(maximum width);
- CLOSED -> Grasp(width=0);
- explicit fixed Grasp force/speed/epsilon;
- asynchronous Move/Grasp lifecycle;
- successful grasp-hold tracking;
- Stop-before-Move release;
- software failure propagation;
- physical unsupported object retention;
- physical release/reopen;
- unchanged measured-q arm hold;
- zero observed Franka health violations in the qualifying isolated run.

Outside G1B:

- object-directed policy behavior;
- autonomous manipulation-task success;
- SUCCESS/FAILURE/ABORT task outcome detection;
- SAPS operator blending;
- grasp-force optimization across object classes;
- fragile/deformable-object handling.
