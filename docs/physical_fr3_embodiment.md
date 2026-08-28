# Physical M2: DROID-to-FR3 Cartesian embodiment

## Scope and status

This completed milestone implements and validates the non-actuating embodiment
boundary from one native `pi05_droid` action and one current FR3 joint state to
a six-dimensional Cartesian policy motion. It never starts a ROS graph,
publishes a Servo command, reads a live robot, or commands the arm or gripper.

M2 starts from M1 commit
`ea762d782c68024ce1f2ce3a9f764b2e6122f198`. The LIBERO simulation baseline
and pinned OpenPI submodule are unchanged.

Reproduce the offline diagnostic with a fresh output identity:

```bash
make droid-fr3-m2 M2_RUN_ID=m2_$(date -u +%Y%m%dT%H%M%SZ)
```

`M2_M1_RUN` selects the M1 `run.json`. `FRANKA_DESCRIPTION_DIR`,
`IGD_FR3_CONTROL_DIR`, and `FRANKA_ROS2_INSTALL` select the read-only lab
sources and installed ROS overlay. The diagnostic refuses to overwrite an
existing output directory.

## Laboratory model and local changes

The backend expands the laboratory
`franka_description/robots/fr3/fr3.urdf.xacro` with the Franka Hand enabled,
then uses Pinocchio 3.9.0 offline. The two finger joints are locked at their
neutral value so the resulting model has exactly seven arm configuration and
velocity dimensions. No new robotics dependency was added.

The description repository was inspected at commit
`fac4949828c8b627ccb8593628212f59a8f46d00`. Its local change to
`robots/common/group_definition.xacro` only adds nine named `fr3_arm`
`group_state` postures. It changes no chain definition, group membership,
joint ordering, TCP, URDF geometry, or Jacobian input. The binary diff SHA-256
at validation was
`13eccbebefda713e4a34583a7b0be61baafcc21220aab164b1c8573486ac5a7d`.
Neither external lab repository was modified.

The lab Servo configuration independently confirms:

```text
planning group: fr3_arm
planning frame: fr3_link0
EE frame:       fr3_hand_tcp
publish period: 0.02 s (50 Hz)
```

Its current unitless scales (`0.4 m/s`, `0.8 rad/s`) are execution settings,
not the scientific definition of the SAPS action.

## Verified joint mapping

The selected episode metadata identifies robot serial
`panda-295341-1326372`. DROID launches Polymetis `franka_hardware`; that
configuration controls indices 0 through 6 of a Panda URDF whose serial chain
is `panda_joint1` through `panda_joint7`. Pinned OpenPI copies raw DROID
`joint_positions` and `joint_velocity` arrays without reordering and declares
the robot type `panda`.

The laboratory FR3 xacro defines the serial chain `fr3_joint1` through
`fr3_joint7`, and the `fr3_arm` semantic group uses that chain. The adapter
therefore uses and validates this explicit ordinal mapping:

| Policy dimension | DROID joint | FR3 joint |
|---:|---|---|
| 0 | `panda_joint1` | `fr3_joint1` |
| 1 | `panda_joint2` | `fr3_joint2` |
| 2 | `panda_joint3` | `fr3_joint3` |
| 3 | `panda_joint4` | `fr3_joint4` |
| 4 | `panda_joint5` | `fr3_joint5` |
| 5 | `panda_joint6` | `fr3_joint6` |
| 6 | `panda_joint7` | `fr3_joint7` |

The adapter rejects a provider with any other name or order. Supporting
primary sources are the [DROID Franka implementation](https://github.com/droid-dataset/droid/blob/main/droid/franka/robot.py),
[DROID launch selection](https://github.com/droid-dataset/droid/blob/main/droid/franka/launch_robot.sh),
[Polymetis Panda configuration](https://github.com/facebookresearch/fairo/blob/main/polymetis/polymetis/conf/robot_model/franka_panda.yaml),
and [Polymetis Panda URDF](https://github.com/facebookresearch/fairo/blob/main/polymetis/polymetis/data/franka_panda/panda_arm.urdf).

## Mathematical contract

For native motion `u_native` in policy dimensions 0 through 6:

```text
s = max(1, max_i |u_native_i|)
u_clipped = u_native / s
delta_q = 0.2 u_clipped                         [rad/policy-step]
qdot_nominal = 15 delta_q = 3 u_clipped         [rad/s]
twist_si = J_FR3(q) qdot_nominal                 [m/s; rad/s]
delta_x_linearized = J_FR3(q) delta_q            [m/step; rad/step]
```

The whole seven-dimensional vector shares one clipping scale, preserving its
direction. `qdot_nominal` is the analytical 15 Hz equivalent of DROID's
per-update semantics; it is not an instruction to execute the FR3 at that
speed. The adapter preserves all four joint representations and both SI
Cartesian representations.

`DroidToFr3TaskSpaceAdapter.project(policy_action, joint_position)` handles one
action only. It queries `JacobianProvider.jacobian(q)` on every call. An
eventual chunk executor must call it with the current `q[k]`; it must not
project all 15 actions with the observation-time Jacobian.

The offline 15-step sequences use the explicitly labelled approximation:

```text
q[0] = recorded DROID Panda observation
q[k+1] = q[k] + delta_q[k]
```

These are DROID-command kinematic rollouts through an FR3 model, not measured
FR3 states or executable trajectories.

## Jacobian and finite-step contract

The backend returns `J` with shape `(6, 7)`:

```text
rows 0..2: linear velocity of fr3_hand_tcp, resolved in fr3_link0 [m/s]
rows 3..5: angular velocity, resolved in fr3_link0             [rad/s]
columns:   fr3_joint1 ... fr3_joint7
```

Pinocchio computes a `LOCAL_WORLD_ALIGNED` geometric Jacobian at the TCP and
the backend rotates its components into `fr3_link0`. FK returns the transform
from `fr3_link0` to `fr3_hand_tcp`.

For every projected action, the diagnostic also computes FK at `q` and
`q + delta_q`. Translation is compared as `p1 - p0` in the base frame.
Rotation uses the SE(3)-consistent rotation vector
`Log(R1 R0^T)`, also in base axes. This comparison diagnoses the first-order
approximation; it does not replace the Jacobian adapter with FK differencing.

## Cartesian normalization for SAPS

M2 defines an explicit DROID-reference normalized motion while retaining all
SI values:

```text
D = diag(0.075 m, 0.075 m, 0.075 m,
         0.15 rad, 0.15 rad, 0.15 rad)
a_task = inverse(D) delta_x_linearized
```

These are DROID's own per-update Cartesian scales. The adapter never clips
`a_task`; values outside `[-1, 1]` remain observable. On the 45 M1 actions the
range was `[-0.4032, 0.8146]`, and no component exceeded unit magnitude. That
small diagnostic does not establish a universal bound.

This scaling gives translation coordinates twice the numerical gain of
rotation coordinates (and four times the squared contribution for equal raw
numeric magnitudes) in normalized cosine similarity. M3 must map human motion
through the same diagonal definition before cosine comparisons. For fixed
linear blending, applying the same invertible diagonal scaling to both sources
and not clipping preserves the componentwise SI blend after denormalization.

The representation is defensible as an embodiment reference because it comes
from DROID, not the current Servo tuning. Its suitability for human-control
cosine geometry still requires M3 evidence.

## Empirical projection

The validated local output is:

```text
outputs/physical_pi05_droid_m2/m2_final_20260827T1620Z/run.json
```

It projects all 15 predicted actions for each genuine M1 observation at steps
0, 76, and 152, using a new Jacobian at every rollout state.

| Diagnostic | Observed over 45 actions |
|---|---:|
| Native joint motion range | `[-0.30544, 0.50103]` |
| DROID vector clipping | `0/45` actions |
| `delta_q` range | `[-0.06109, 0.10021] rad/step` |
| Nominal `qdot` range | `[-0.91633, 1.50310] rad/s` |
| Cartesian twist range | `[-0.70798, 0.91646]` SI |
| Linearized Cartesian step range | `[-0.04720, 0.06110]` SI |
| Normalized task-motion range | `[-0.40320, 0.81463]` |
| Normalized components above unit magnitude | `0/270` |
| Nominal velocity-limit violations | `0/315` joint components |
| Rollout next-position-limit violations | `0/315` joint components |
| Worst nominal velocity/limit ratio | `0.57370`, `fr3_joint2` |
| Jacobian condition number | median `10.93`, max `236.14` |
| Minimum singular value | min `0.008389`, max `0.22508` |
| Numerically near-singular actions | `0/45` at tolerance `1e-10` |

The worst conditioning occurs at source step 152, rollout action 14. Although
it is not rank-deficient at the diagnostic tolerance, condition number 236 is
a meaningful warning for later Servo/singularity handling.

## Linearization and null-space loss

FK-versus-Jacobian translation error was `0.000001–0.005004 m`, mean
`0.001307 m`, median `0.000645 m`. Rotation-vector error was
`0.000005–0.006484 rad`, mean `0.001733 rad`, median `0.001155 rad`. Both worst
cases were source step 0, rollout action 11. The maximum errors (about 5 mm and
0.37 degrees) show that finite 0.2-scaled steps are not always negligible;
M2 nevertheless retains the required differential Jacobian method.

For each action the diagnostic computes:

```text
qdot_task = pinv(J) (J qdot_nominal)
qdot_null = qdot_nominal - qdot_task
null_fraction = norm(qdot_null) / norm(qdot_nominal)
```

Null fraction ranged from `0.00011` to `0.31258`, with mean `0.15828` and
median `0.18453`. Task-component norms ranged `0.08204–2.75009 rad/s`; null
component norms ranged `0.00024–0.31455 rad/s`. The worst fraction was source
step 76, action 6: total `0.40944`, task `0.38892`, null `0.12798 rad/s`.

MoveIt Servo will choose its own seven-joint realization of the six-dimensional
TCP motion. Matching `J qdot` therefore does not reproduce the native DROID
joint trajectory or its null-space posture evolution.

## FR3 limits, cadence, and gripper

The expanded FR3 URDF supplies velocity limits
`[2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26] rad/s` and the recorded position
limits retained in `run.json`. No nominal velocity or rollout position exceeded
them in this diagnostic. M2 applies no joint clipping or execution scaling.

The policy semantic cadence is 15 Hz while the current Servo publish period is
0.02 seconds (50 Hz). Holding, interpolation, or resampling, as well as the
physical speed/safety scale, remains an M4 execution decision and must be
logged separately from policy and embodiment conversion.

The physical SAPS gripper representation is continuous normalized closure
`g in [0, 1]`, where `0` is fully open and `1` is fully closed. The adapter
validates this separately from the seven motion dimensions. It neither
thresholds at 0.5 nor maps closure to Franka Hand width, Move, or Grasp calls.

## M3/M4 boundaries

M3 still needs read-only/live FR3 state timing, camera alignment and
preprocessing, and SpaceMouse motion mapped into the same Cartesian axes and
normalization. In particular, current SpaceMouse Servo messages are expressed
in `fr3_hand_tcp`, whereas M2's policy twist components are expressed in
`fr3_link0`; arbitration inputs must use one explicit common expression frame.

M4 still owns 15-to-50 Hz action-rate conversion, base/TCP command-frame
mapping, execution and safety scaling, Servo collision/singularity behavior,
position/velocity safety treatment, stale-state handling, and all actuation.
