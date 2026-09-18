# G1B Physical Gripper — Object-Agnostic Grasp/Release Validation

## Status

**PASS — isolated physical DROID binary gripper semantics and FR3 grasp/release lifecycle**

Qualifying physical run:

```text
outputs/physical_franka_hand_g1_b/g1b_grasp_release_20260918_144453
```

Run UUID:

```text
a1319691-4dda-4707-8cfe-4d804fb0862d
```

This run closes the G1B physical gripper gate. It validates the object-agnostic
conversion from DROID binary absolute gripper intent to explicit Franka Hand
Move/Grasp commands while the arm remains on the existing measured-q hold.

The qualifying test used an unsupported object: the operator initially held the
object between the open fingers and released it as the grasp closed. The gripper
then retained the object without support for the complete approximately
three-second validation dwell before the release sequence reopened the hand.

This is a gripper-command and physical-retention validation. It is not a
manipulation-task-success result and does not validate object-directed policy
behavior or a task SUCCESS/FAILURE detector.

## Provenance

### SAPS repository at execution

```text
branch:
experiment/physical-fr3-saps

HEAD at execution:
e408cd4e955dd7ef90b80011fb4ce8034698311f
Validate repeated C1-C2 cold-start execution
```

The qualifying run was executed with local uncommitted G1B implementation
changes. The run's `saps.status` field records those local modifications and
untracked files. Those changes are frozen by the SAPS commit that includes this
validation archive.

### fr3_lab_stack at execution

The run recorded the previously frozen arm baseline:

```text
9e535b665626cf8f3894fc4d2ef9136790abad92
Add streaming target timing instrumentation
```

with the new Franka Hand implementation present as a dirty local extension.
The run records per-file SHA-256 hashes for that extension.

The validated hand implementation was subsequently committed and pushed as:

```text
4bb6cdc58839dcdd94acbe1633b8f361676a4eb6
Add asynchronous Franka Hand Move and Grasp runtime
```

This commit contains the policy-independent asynchronous Move/Grasp client,
release-stop lifecycle, tests, package dependency updates, and hand-runtime
documentation used for the G1B implementation.

### franka_ros2 contract inspected for G1B

```text
1369a2cb200d0f7b3da11c7728c7ca2e6975ca00
```

The lower-level wrapper uses:

```text
/franka_gripper/move
/franka_gripper/grasp
/franka_gripper/stop
```

No upstream `franka_ros2` modification was introduced for G1B.

## DROID semantic contract

Pinned OpenPI/DROID inspection established binary absolute gripper intent:

```text
raw policy gripper value > 0.5  -> CLOSED
raw policy gripper value <= 0.5 -> OPEN
```

The SAPS policy adapter therefore maps:

```text
OPEN   -> Franka Move(maximum_width)
CLOSED -> Franka Grasp(width=0)
```

The physical embodiment parameters used in the qualifying validation were:

```text
maximum width:        0.080 m
Move/Grasp speed:     0.100 m/s
Grasp target width:   0.000 m
Grasp force:          20.0 N
epsilon inner:        0.001 m
epsilon outer:        0.080 m
grasp dwell:          3.0 s
```

The object width was not supplied to the policy adapter, gripper wrapper,
validator, or success criterion.

These speed/force/epsilon values are fixed embodiment parameters rather than
object-specific task inputs.

## Validated physical sequence

```text
stationary measured-q arm hold
→ DROID OPEN
→ Franka Move(0.080 m)
→ DROID CLOSED
→ Franka Grasp(width=0, force=20 N, epsilon=[0.001, 0.080] m)
→ successful physical object capture
→ operator releases object
→ unsupported retention for ~3 s
→ DROID OPEN
→ nonterminal Franka Stop of successful grasp
→ confirmed release stop
→ Franka Move(0.080 m)
→ object released
→ terminal cleanup stop
```

No policy arm action was executed during this validation.

## Qualifying measurements

### Initial open

```text
target width:             80.000 mm
confirmed measured width: 79.975 mm
action result:            success
```

### Grasp

```text
target width:             0.000 mm
force:                    20.0 N
epsilon inner:            1.0 mm
epsilon outer:            80.0 mm
action result:            success
confirmed measured width: 46.252 mm
```

The measured width is an observed result only. It was not supplied as an object
width target.

### Unsupported retention dwell

```text
duration:                 3.0046 s
samples:                  646
initial width:            46.252 mm
final width:              46.321 mm
minimum width:            46.252 mm
maximum width:            46.321 mm
width span:               0.069 mm
validator threshold:      2.000 mm
```

The operator visually confirmed that the object remained physically retained
without support for the complete dwell.

### Release

The hand runtime recorded:

```text
release_stop_requested
→ release_stop_result(success=true)
→ move_requested(width=0.080 m)
→ action result success
```

Final confirmed measured width:

```text
79.799 mm
```

This validates the normal successful-Grasp release lifecycle. The opening Move
does not overlap the force-holding grasp state.

## Arm hold and robot health

The arm executed no policy action:

```text
policy_arm_actions: 0
```

The pre-inference measured-q hold was accepted and remained unchanged throughout
the gripper validation.

Final runtime evidence reported:

```text
hold validation:               accepted
hold unchanged:                true
Franka samples:                9611
Franka health violations:      0
controller publication errors: 0
evidence ID gaps:              0
```

The arm hold's controller timing evidence contained 9640 period samples:

```text
mean:                          0.999999 ms
p95:                           1.100070 ms
p99:                           1.151417 ms
maximum:                       1.228171 ms
samples > 1.5 ms:              0
samples > 2.0 ms:              0
```

## Software validation

Before physical G1B closure, the lower-level `fr3_lab_stack` hand implementation
passed:

```text
16 focused Franka Hand tests
120 total fr3_lab_stack tests
0 errors
0 failures
0 skipped
```

The SAPS implementation then passed:

```text
351 tests
compilation passed
```

The SAPS count is one lower than the preceding revision because the obsolete
two-test obstruction/reopen validator was replaced with the single
object-agnostic grasp-release validator test.

## Accepted claim

> Under a stationary measured-q arm hold, the physical FR3 hand correctly
> realized DROID binary absolute gripper intent using an object-agnostic
> Move/Grasp adapter: OPEN commanded full opening, CLOSED commanded a
> force-controlled Grasp without object-width input, the hand captured and
> retained an unsupported object for the complete three-second dwell, and a
> subsequent OPEN first stopped the successful grasp and then reopened the hand.
> The validation completed without an observed Franka health violation or arm
> target change.

## Scope and limitations

Validated by this archive:

- strict DROID binary absolute gripper semantics;
- OPEN -> Franka Move(maximum width);
- CLOSED -> Franka Grasp(width=0);
- no privileged object-width input;
- fixed embodiment force/speed/epsilon parameters;
- successful physical object capture;
- unsupported three-second retention;
- measured-width stability during retention;
- successful Grasp -> Stop -> Move(open) release lifecycle;
- one continuous gripper-wrapper session;
- stationary arm hold during gripper execution;
- zero observed Franka health violations;
- clean software regression suites.

Not validated:

- object-directed policy behavior;
- autonomous approach or alignment to the object;
- manipulation-task success;
- a validated SUCCESS/FAILURE/ABORT detector;
- grasp-force optimization across object classes;
- fragile/deformable-object handling;
- long-horizon policy execution with live gripper commands;
- SAPS operator blending with the physical gripper.

The fixed 20 N grasp force is sufficient for this G1B mechanism validation but
is not presented as an optimal universal grasp-force setting.

## Archived evidence

This directory contains:

```text
README.md
qualifying_run.json
qualifying_gripper_feedback.json
qualifying_arm_timing.jsonl.gz
SHA256SUMS
```

`qualifying_run.json` is the authoritative structured record of the G1B run.

`qualifying_gripper_feedback.json` retains measured gripper-state feedback.

`qualifying_arm_timing.jsonl.gz` is a deterministic lossless compression of the
raw arm/controller evidence captured while the arm remained on the measured-q
hold.

## Next work

G1B is closed. Further arbitrary gripper-only repetitions or replay of a saved
policy chunk are not required merely to demonstrate the same command conversion.
The strict `> 0.5` decision boundary is covered by deterministic software tests,
while the physical validation demonstrates the corresponding OPEN/CLOSED
realization and grasp-release lifecycle on hardware.

The next substantive gate is G2: investigate why the physical π0.5/DROID policy
does not yet reliably move toward and interact with the intended object. That
investigation should treat camera observations, initial conditions, task prompt,
policy behavior, and physical scene alignment separately from the now-validated
arm transport/control and gripper-command mechanisms.
