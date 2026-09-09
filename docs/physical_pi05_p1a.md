# P1-A: non-actuating DROID discrete joint targets

P1-A verifies measured-state target construction and scheduling with live FR3
observations and real `pi05_droid` inference. No command publisher, trajectory
action client, gripper client, or controller-switching client is created.
Read-only controller/parameter inspection uses separate ROS CLI processes.
The P0 runtime, checkpoint, seeded sampling and historical outputs are unchanged.

## Source-established DROID boundary

Inspected DROID revision:
`33ae6a67274f36d2e29525b86f23a56616ef43a7`.

- [FrankaRobot.create_action_dict and update_joints](https://github.com/droid-dataset/droid/blob/33ae6a67274f36d2e29525b86f23a56616ef43a7/droid/franka/robot.py)
  read the current robot state, add the joint delta to `joint_positions`, and
  pass the resulting absolute position to `update_desired_joint_positions`.
  The nonblocking DROID path uses a Polymetis impedance policy; its internal
  dynamics are not claimed equivalent to the FR3 ROS effort controller.
- [RobotIKSolver](https://github.com/droid-dataset/droid/blob/33ae6a67274f36d2e29525b86f23a56616ef43a7/droid/robot_ik/robot_ik_solver.py)
  defines all seven relative maximum deltas as `0.2`. Its additional vector
  normalization is inactive for an already componentwise-clipped input.
- [Pinned OpenPI loop](../third_party/openpi/examples/droid/main.py) clips each
  component to `[-1,1]`, thresholds gripper at strictly `>0.5`, and consumes
  eight actions at a nominal 15 Hz before synchronous replanning. Its `(10,8)`
  assertion is obsolete for the pinned `pi05_droid`; actual chunks are `[15,8]`.

For each control step k:

```text
u_k        = clip(policy_action_k[:7], -1, 1)
delta_q_k  = 0.2 * u_k
q_target_k = q_measured_k + delta_q_k
```

There is no accumulation from the previous target. `15 * delta_q` is recorded
only as a rate-equivalent diagnostic, never a velocity command. The gripper
reference is closed for `action[7] > 0.5`, otherwise open. Neither is executed.

## Active controller inspection — 2026-09-09

Read-only live queries established:

| Property | Observed value |
|---|---|
| Controller | active `fr3_arm_controller`, `joint_trajectory_controller/JointTrajectoryController` |
| Controller and manager update rate | 1000 Hz, synchronous |
| Claimed hardware commands | `fr3_joint1/effort` through `fr3_joint7/effort` |
| Required states | position and velocity for all seven joints |
| Other available hardware interfaces | seven position and velocity interfaces, unclaimed |
| Direct command topic | `/fr3_arm_controller/joint_trajectory`, `trajectory_msgs/msg/JointTrajectory` |
| Topic endpoints | one controller subscriber, zero publishers at inspection; subscriber best effort, volatile |
| Action server | `/fr3_arm_controller/follow_joint_trajectory`, `control_msgs/action/FollowJointTrajectory` |
| Existing action client | `/moveit_simple_controller_manager` |
| Feedback | `/fr3_arm_controller/controller_state` |
| Interpolation | `splines`; `interpolate_from_desired_state=false`, `open_loop_control=false` |
| Partial/integrated goals | both disabled; all seven positions required |
| Timeout | `cmd_timeout=0.0` (disabled) |
| Default position tracking/goal tolerances | all zero (disabled); stopped velocity tolerance `0.01` |
| Manager command-limit enforcement | `enforce_command_limits=false` |
| Installed JTC package | `4.39.0-1noble.20260412.063953` |

Interpretation uses the matching upstream
[JTC 4.39.0 source](https://github.com/ros-controls/ros2_controllers/blob/4.39.0/joint_trajectory_controller/src/joint_trajectory_controller.cpp)
and [trajectory sampler](https://github.com/ros-controls/ros2_controllers/blob/4.39.0/joint_trajectory_controller/src/trajectory.cpp).
The binary package was identified; it was not rebuilt or compared byte-for-byte
against upstream source.

A trajectory must contain all seven joint names and at least one point with
seven positions. Derivative fields can be empty. Times must strictly increase
for multiple points. A zero header stamp means start on the first controller
sample. A nonzero stamp whose final point is already in the past is rejected.
The single-point validator permits `time_from_start=0`.

**Interpolation materially affects the reference:** a position-only point at
`time_from_start=1/15 s` ramps linearly from the controller's current measured
state to the target. Supplying endpoint velocities instead selects cubic
interpolation; accelerations can select quintic interpolation. These add a
trajectory time law that the DROID discrete target does not specify.

The smallest representation of the discrete setpoint is one position-only
point at `time_from_start=0`, header stamp zero. The sampler immediately selects
that point and supplies zero desired velocity/acceleration after its endpoint.
This retains the absolute target value without a 66.7 ms ramp. It does **not**
retain Polymetis impedance dynamics: the FR3 controller converts position and
velocity tracking errors to effort using its own PID gains.

Each accepted topic message replaces the current trajectory; there is no
FIFO guarantee for intermediate targets. New action goals cancel the previous
active goal. Topic replacement does not perform the same action-goal
cancellation handshake; do not mix topic streaming with another active goal.
With timeout disabled, stopping messages leaves the final target held; it does
not mean stop at the current measured pose. Explicit action cancellation uses
measured-position hold with the current `decelerate_on_cancel=false` setting.
Cancellation cannot be assumed effective after an action has already completed.

JTC validates representation and timing, not absolute target feasibility against
URDF position bounds. Manager limit enforcement is disabled. The inspected
Franka hardware effort path writes torques, with an optional torque-rate limiter;
that is not a target-position gate. Hardware reflexes do not establish that an
arbitrary target is safe. P1-A separately checks current and candidate joint
positions against the **live robot-description** bounds.

15 Hz replacement is representable by this 1000 Hz controller, but no message
has been submitted to test delivery, tracking, torque transients or stopping.
P1-A establishes target arithmetic and observer scheduling only. A ramped
trajectory cannot be called identical to the DROID discrete setpoint interface.

## Implemented verifier and usage

With the existing FR3, cameras and policy server running:

```bash
make physical-pi05-target-verify \
  PHYSICAL_RUN_ID=p1a_$(date -u +%Y%m%dT%H%M%SZ) \
  PHYSICAL_PROMPT='pick up the object' P1A_REQUESTS=3
```

The default is one request; the finite limit is 100. Existing P0 configuration
supplies observation identity and freshness only. P1-A has its own output
family under `outputs/physical_pi05_droid_p1a/`, allocated without overwriting.
The Makefile now exports `FRANKA_ROS2_INSTALL` after assigning its default:
early export had defined an empty variable and prevented `?=` from taking
effect, causing `/setup.bash` lookup. Explicit workspace overrides still work.

A dedicated ROS spin thread updates the existing observation collector under
a lock. Readers take coherent snapshots; inference, disk output and the 15 Hz
schedule run outside that lock. Every tick samples a fresh measured q. Callback
failures propagate or are recorded; stale and nonadvancing joint states reject
candidates. Both cameras must advance before each policy request.

Eight slots use absolute monotonic deadlines, starting after response validation
and diagnostic persistence. The last slot retains its nominal dwell through
8/15 s. Missed slots are marked rejected; they do not shift later deadlines.
Only after this window and artifact persistence does the next synchronous
request begin. This is not overlapping inference or continuous execution.

CLI diagnostic gates (not approved physical execution limits):

- `--max-state-age=0.1`: source and receive age of arm feedback;
- `--max-action-age=1.0`: elapsed monotonic time since inference request;
- `--max-lateness=0.0666666667`: one nominal period.

Each candidate records raw action, clipped u, measured q and timestamp, delta,
absolute target, lower/upper limits, measured and target margins to both limits,
position-gate result, overall verifier result and reasons, gripper reference,
request/response/observation ages, intended timestamp, actual sample timestamp,
and lateness. `safe_to_execute=false` and `controller_acceptance_tested=false`
remain explicit even for candidates passing all diagnostic gates.

Run artifacts include provenance/config, raw canonical observation and `[15,8]`
action NPZ files, seed/noise and policy timing, first-request model audit,
controller queries, live URDF/hash and limits, eight candidates per request,
and continuity windows. `inference_continuity.json` retains callback receive
and source stamps *during the blocking call*, including empty-window boundary
gaps. These measure delivered subscriber callbacks, not raw camera publishing
rates. `continuity.json` covers subscription monitoring only, excluding CLI
preflight before subscriptions existed. Unit tests require no ROS/hardware.

## Live diagnostic evidence — 2026-09-09

Final run `p1a_discrete_20260909_verified`: three real `[15,8]` chunks,
24/24 candidate slots passed the diagnostic gates, `request_count_reached`.
Zero policy actions, gripper commands, command publishers and command action
clients. The owned node exposed only `/parameter_events` publication and no
service clients at both endpoints. Sources retained the accepted P0 identities.

| Chunk | Client RTT (ms) | Action age, first–last slot (ms) | Maximum lateness (ms) |
|---|---:|---:|---:|
| 0 | 131.3 | 166.6–633.3 | 0.171 |
| 1 | 121.6 | 131.9–598.6 | 0.191 |
| 2 | 125.0 | 137.7–603.4 | 1.445 |

Maximum sampled arm source age was 4.067 ms. The smallest predicted target
margin across all 24 candidates was 0.779856 rad, at joint 4. These measurements
include diagnostic overhead and are not a formal latency or task benchmark.

The preceding run `p1a_discrete_20260909_first` retained a 3.268 s initial
inference and correctly rejected all eight associated candidates as expired;
its next 16 candidates passed. That run's whole-run continuity window included
pre-subscription CLI inspection, so use its **per-inference** windows only.
The final run corrected that window without editing the earlier artifacts.

During final inference windows, arm callbacks numbered 118/117/114; wrist
4/3/4; external 4/2/4; gripper 2/2/2. Maximum arm receive gap within inference
was 3.332 ms; external source gap reached 66.715 ms. Across the 6.258 s
subscription-monitoring window, source gaps reached 333.1 ms wrist, 366.9 ms
external, 4.107 ms arm and 103.7 ms gripper. No source-stamp regressions or
duplicates were observed. Boundary-inclusive receive gaps also include startup
discovery delay; these must not be interpreted as steady-state publisher gaps.
Continuous callbacks worked, but camera irregularity remains relevant to any
future continuous execution gate.

One real chunk's first eight measured-state targets (rad) are below. Every row
uses its own fresh measured q; full raw action, u, q, delta, both signed margins
and timing are in `request_0000/candidates.json`. The remaining seven returned
actions are retained raw in `actions.npz` and are not scheduled.

| Slot | Proposed q_target, joints 1–7 |
|---|---|
| 0 | `-0.281215, -0.002805, 0.117051, -2.233217, -0.086653, 2.201947, 0.677315` |
| 1 | `-0.283137, -0.003043, 0.120451, -2.234465, -0.084662, 2.200477, 0.677817` |
| 2 | `-0.286878, -0.004881, 0.126632, -2.237580, -0.079867, 2.199323, 0.674685` |
| 3 | `-0.289908, -0.007771, 0.133263, -2.241357, -0.074710, 2.199393, 0.671000` |
| 4 | `-0.292089, -0.010002, 0.139598, -2.245850, -0.070109, 2.198517, 0.664573` |
| 5 | `-0.294775, -0.013683, 0.146260, -2.250334, -0.064134, 2.200694, 0.657184` |
| 6 | `-0.296111, -0.018629, 0.152043, -2.255937, -0.059118, 2.203447, 0.650555` |
| 7 | `-0.298107, -0.023197, 0.157808, -2.262244, -0.055036, 2.207334, 0.644674` |

Minimum margins per joint across these eight targets (rad):
`2.445593, 1.760503, 2.742892, 0.779856, 2.719847, 1.654017, 2.338083`.

Validation: nine focused tests passed; `make compile` and `make check` passed
(246 unit tests plus compilation), and `git diff --check` passed.

## First single-action P1-B recommendation — not implemented

Use one `FollowJointTrajectory` goal containing seven named positions equal to
`q_measured + 0.2*clip(action[:7],-1,1)`, one position-only point, zero header
stamp and zero `time_from_start`. This is the candidate route for checking a
single discrete target through the existing controller, with feedback available.
Do not submit the remaining seven actions or gripper commands.

Before that test, agree on one sufficiently small real candidate, fresh-state
and action-age bounds, workspace/collision clearance, tracking/effort abort
criteria, and an independently verified hold/stop procedure. A goal completing
or messages ceasing still leaves a held target. Do not silently shrink the
DROID delta, retime it, or label an interpolated test equivalent. The observed
warm chunk's first target includes a roughly 0.120 rad joint-4 delta; passing
joint-position limits alone is insufficient reason to select it for a first
physical test. P1-B implementation and actuation remain pending user review.
