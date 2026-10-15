# π0.5-DROID public action and controller semantics

This inspection uses OpenPI commit `15a9616a00943ada6c20a0f158e3adb39df2ccac`
and the DROID revision `33ae6a67274f36d2e29525b86f23a56616ef43a7`
already identified in the physical deployment record. The local repository was
at `fe73baa29b229774129ee967d2dbd2ecc191c206` during inspection.

## Complete execution path

- Pinned OpenPI `training/config.py` selects `DroidOutputs` for `pi05_droid`.
  `policies/policy_config.py` applies checkpoint-asset quantile
  `Unnormalize` before `DroidOutputs` returns the first eight dimensions.
  Thus the server response has already been denormalized.
- The [pinned OpenPI example](../../third_party/openpi/examples/droid/main.py)
  instantiates `RobotEnv(action_space="joint_velocity",
  gripper_action_space="position")`. It thresholds the gripper at `> 0.5`,
  clips each arm component to `[-1, 1]`, calls `env.step(action)`, and sleeps
  to match 15 Hz. Current [upstream OpenPI](https://github.com/Physical-Intelligence/openpi/blob/main/examples/droid/main.py)
  has the same behavior. Both [pinned](../../third_party/openpi/docs/norm_stats.md)
  and [current upstream action documentation](https://github.com/Physical-Intelligence/openpi/blob/main/docs/norm_stats.md)
  call the first seven outputs joint velocities but do not assign rad/s to a
  value of one.
- The [pinned DROID `RobotEnv.step`](https://github.com/droid-dataset/droid/blob/33ae6a67274f36d2e29525b86f23a56616ef43a7/droid/robot_env.py)
  requires eight elements and checks all velocity-action values lie in
  `[-1, 1]`; `update_robot` forwards the selected action-space strings to
  `FrankaRobot.update_command`. If a NUC is configured, `ServerInterface`
  forwards the same call over RPC to the robot server. The [current DROID
  source](https://github.com/droid-dataset/droid/blob/main/droid/robot_env.py)
  retains this path.
- [Pinned `FrankaRobot.create_action_dict`](https://github.com/droid-dataset/droid/blob/33ae6a67274f36d2e29525b86f23a56616ef43a7/droid/franka/robot.py)
  labels the seven incoming values `joint_velocity`, calls
  `RobotIKSolver.joint_velocity_to_delta`, and adds the resulting delta to
  measured joint positions. `update_command` then calls
  `update_joints(..., velocity=False)` with this absolute position target.
  The nonblocking `update_joints` path starts Polymetis
  `start_cartesian_impedance` if necessary, then calls
  `update_desired_joint_positions` with the position target. Its gripper
  path independently calls `update_gripper` with normalized closure.
- [Pinned `RobotIKSolver.joint_velocity_to_delta`](https://github.com/droid-dataset/droid/blob/33ae6a67274f36d2e29525b86f23a56616ef43a7/droid/robot_ik/robot_ik_solver.py)
  defines seven `relative_max_joint_delta` values of `0.2`, computes a
  maximum-component normalization, and multiplies by `0.2`. For the
  already-clipped OpenPI action, this yields `0.2 * action` radians per
  reference update. Its `joint_delta_to_velocity` divides by `0.2`, returning
  the dimensionless action coordinate. Current [upstream DROID
  robot code](https://github.com/droid-dataset/droid/blob/main/droid/franka/robot.py)
  and [solver](https://github.com/droid-dataset/droid/blob/main/droid/robot_ik/robot_ik_solver.py)
  retain this behavior.
- Upstream [Polymetis `RobotInterface`](https://github.com/facebookresearch/fairo/blob/main/polymetis/polymetis/python/polymetis/robot_interface.py)
  implements `start_cartesian_impedance` with
  `HybridJointImpedanceControl`; `update_desired_joint_positions` updates its
  `joint_pos_desired` parameter. The [impedance policy](https://github.com/facebookresearch/fairo/blob/main/polymetis/polymetis/python/torchcontrol/policies/impedance.py)
  computes torque from joint position and velocity errors plus inverse
  dynamics. Thus the actual DROID path ends in torque control around a
  position target. The exact commit of DROID's `fairo` submodule was not
  available in this workspace; this last step is verified against upstream
  Polymetis source and the pinned DROID calls, not a locally installed
  pinned Polymetis binary.

The source-established public command and DROID realization are

```text
u = clip(server_action[:7], -1, 1)  # dimensionless action coordinate
q_target = q_measured + 0.2 * u     # DROID's internal position target
```

The `0.2` factor is DROID's controller-interface scaling, not an intrinsic
physical unit assigned to the policy output by OpenPI. The first seven
outputs are best described as **normalized joint-velocity action
coordinates**. The source does not establish that `u=1` means `1 rad/s`.
The existing Isaac position baseline implements DROID's position-target
mapping. The value `15 * 0.2 * u` is only a nominal rate corresponding to
the requested per-update displacement; it is not a DROID velocity setpoint.

Polymetis provides `start_joint_velocity_control` and
`update_desired_joint_velocities`, a genuine alternative API that DROID does
not call. Its upstream [velocity policy](https://github.com/facebookresearch/fairo/blob/main/polymetis/polymetis/python/torchcontrol/policies/velocity.py)
internally integrates the desired velocity into a desired position for PD
torque control. Thus even that Polymetis controller uses an intermediate
position reference internally. It does accept physical desired velocities;
the DROID public action coordinate still requires a specified conversion to
rad/s before use with that API.

## Isaac interface inspection

The installed Isaac Sim 6.1 experimental articulation exposes
`set_dof_velocity_targets`, `get_dof_velocity_targets`, `set_dof_gains`,
`get_dof_gains`, and `get_dof_max_velocities`. Its API says velocity control
requires zero stiffness and nonzero damping. The current FR3 assembly uses
position drives; prior [drive response inspection](droid_deployment_audits.md)
measured arm stiffness `3,437,746.75` and damping `343,774.6875` in the
baseline scenes. A velocity mode would need arm-only runtime gain switching
and a verified position-drive hold under gravity. Zero velocity alone has not
been validated as a safe hold. The original USD and solver settings should
remain intact.

Isaac therefore supports sending seven clipped numbers directly to the
seven arm DOF velocity targets. Isaac interprets those numbers as rad/s.
That call would silently impose the unsupported convention that one DROID
action unit equals `1 rad/s`. It would be a **direct-velocity experiment**,
not exact DROID controller replication. A defensible physical scale needs a
documented policy-unit definition, a controller calibration against measured
DROID responses, or an explicitly chosen experimental gain. Neither the
OpenPI action label nor the pinned DROID controller proves a unity gain.
The source inspection alone does not establish a physical velocity scale.

## Implementation gate

The opt-in Isaac `joint_velocity` mode uses
`qdot_cmd = velocity_scale_rad_s_per_action_unit * clip(action[:7], -1, 1)`.
Its default scale, `1.0 rad/s per action unit`, is an **engineering test
value**, not an established DROID physical unit. It can be set explicitly
on the CLI and is recorded in provenance and execution logs. Position
reference remains the default. The scale is applied before saturation by
the articulation's per-joint velocity limits and the one-interval projected
joint-position limits. Requested, commanded, and saturation values are
recorded separately.

Velocity mode sets only the seven arm drive stiffnesses to zero, retains
their original nonzero damping, and sends articulation velocity targets.
Before inference and at termination, it clears the arm velocity target,
replaces the arm position target with measured joint positions, and restores
the original arm stiffness and damping. Gripper drives and source USD are
unchanged. This is a direct-velocity experiment, not exact DROID controller
replication. The pinned source does not justify treating the original
position increment as a constant velocity command.

## Bounded validation, 2026-10-14

Using the unchanged Robotiq scene, the contact-free
[`velocity_interface_smoke.py`](../scripts/velocity_interface_smoke.py)
run at `outputs/isaac_droid_velocity_interface/20261014T071925Z_31bfa7b1/`
sent `0.03 rad/s` to arm joint 1 for eight 60 Hz steps. Joint 1 moved
`+0.00377 rad`; the maximum arm drift over eight subsequent position-hold
steps was `0.00115 rad`. The velocity and position-drive transitions were
recorded with the gripper at DOF index 7 and arm joints at indices 0–6.

At the experimental scale of `1.0 rad/s per action unit`, the real-policy
one-chunk run at
`outputs/isaac_droid_robotiq_one_chunk/20261014T072233Z_02ce2e79/`
completed eight actions with four physics steps each and no command
saturation or simulation schedule error. Its 32 action steps plus eight
terminal-hold evidence steps are retained in `execution.json`. The largest
per-step difference between simulator-reported and position-difference
velocity was `0.032 rad/s`; the streams remain distinct in the artifact.

The two-replan run at
`outputs/isaac_droid_robotiq_rollout/20261014T072326Z_27554816/`
completed 16 actions and terminated at the configured replan cap. Both
inferences held the simulation state, both execution-to-hold transitions
succeeded, and no saturation or safety rejection was recorded. The gripper
remained open throughout; this run does not test a gripper transition or
establish task success or contact fidelity.

A separate contact-free interface run at
`outputs/isaac_droid_velocity_interface/20261014T073820Z_48e6d01d/`
used the original Robotiq scene and the existing binary gripper controller.
With the arm drive in velocity mode, the driver reached its closed and open
targets in 20 physics steps each; the measured pad width changed from
`0.08708 m` to `0 m` and back. Neither transition reported a gripper fault.
The arm velocity target was cleared and position-drive gains restored after
the cycle. The composed FR3 articulation retained 32 solver position
iterations. This checks the interface only, not manipulation success or
contact fidelity.
