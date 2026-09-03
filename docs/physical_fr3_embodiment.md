# Physical FR3 kinematics and DROID action mapping

## Scope and current status

This document is the authoritative non-actuating physical embodiment record.
It fixes the FR3 geometry, the pinned π0.5-DROID joint-action boundary, the
manual NumPy forward kinematics (FK), and the manual geometric Jacobian. It
also records independent Pinocchio and finite-difference validation and a
finite-action diagnostic over the saved M3 observations and policy actions.

Nothing here publishes a ROS command, moves the FR3 or gripper, selects
physical SAPS Cartesian scales, or defines inverse kinematics. Physical
execution and arbitration remain unresolved.

The earlier M2 adapter and its generated artifacts are retained only as
historical provenance. Its global joint-vector rescaling and proposed
`0.075 m`/`0.15 rad` normalization are not authoritative. The legacy Make
paths require `ALLOW_LEGACY_M2=1`, preventing accidental selection by current
diagnostic workflows.

## Source and provenance

The primary geometry source inspected read-only is the local
`~/franka_ros2_ws/src/franka_description` checkout:

```text
commit: 7aeeddc449edf8d62b594f9e36a81da53e7796f9
subject: chore: bump version for release 2.9.0
status: clean
```

The relevant files are:

- `robots/common/franka_arm.xacro`, which applies each joint origin before a
  revolute z-axis motion;
- `robots/fr3/kinematics.yaml`, which supplies the fixed FR3 transforms;
- `robots/fr3/fr3.urdf.xacro`, which mounts the Franka Hand with yaw
  `-pi/4`;
- `robots/common/franka_robot.xacro` and
  `end_effectors/common/franka_hand.xacro`, which supply the default
  `0.1034 m` Hand-to-TCP translation.

The pinned OpenPI submodule remains at
`15a9616a00943ada6c20a0f158e3adb39df2ccac`. Neither external source was
modified during this consolidation.

## DROID policy action semantics

Pinned OpenPI returns a checkpoint-unnormalized eight-dimensional DROID action.
Dimensions 0 through 6 are the DROID joint-motion coordinates and dimension 7
is gripper position. For the arm:

```text
u_pi  = policy_action[0:7]
u_ref = clip(u_pi, -1, 1)       # independently, component by component
delta_q = 0.2 u_ref             # radians per reference controller update
```

This is component-wise clipping. It is not the obsolete global transform
`u / max(1, max_i(abs(u_i)))`. Multiplying `delta_q` by `15 Hz` is only an
equivalent average-velocity diagnostic; it is not the native DROID/OpenPI
command and is not an FR3 execution instruction.

The pinned policy produces 15 actions per inference. The pinned physical
OpenPI DROID workflow is designed around executing the first 8 at 15 Hz before
replanning; the full 15-action chunk is retained here only as a stress
diagnostic.

DROID used a Robotiq 2F-85 for data collection, but π0.5-DROID still emits the
seven joint-motion coordinates plus gripper. No Robotiq-TCP-to-Franka-Hand-TCP
transform is part of the native policy mapping, and none is invented here.

## Panda/FER and FR3 geometry

For each arm joint, the inspected description uses

```text
T_(i-1,i) = T(x_i, y_i, z_i) Rx(alpha_i) Rz(q_i).
```

The fixed chain is:

| Joint | Translation `(x, y, z)` m | `alpha` |
|---|---|---|
| J1 | `(0, 0, 0.333)` | `0` |
| J2 | `(0, 0, 0)` | `-pi/2` |
| J3 | `(0, -0.316, 0)` | `+pi/2` |
| J4 | `(0.0825, 0, 0)` | `+pi/2` |
| J5 | `(-0.0825, 0.384, 0)` | `-pi/2` |
| J6 | `(0, 0, 0)` | `+pi/2` |
| J7 | `(0.088, 0, 0)` | `+pi/2` |
| J7 to flange/link8 | `(0, 0, 0.107)` | `0` |

The inspected FR3 and Panda/FER reference descriptions have the same modeled
serial-arm geometry through the flange: their `kinematics.yaml` files are
byte-identical (SHA-256
`44326830a6773dd6611cdf9947045eeb3276ea4e3ca47a05272816c862c8e386`).
This does not make the robots interchangeable. Their joint limits and
performance specifications differ; robot-specific dynamics and safety limits
must also remain explicit. In particular, the inspected FR3 and FER
`joint_limits.yaml` files have different hashes.

## Franka Hand TCP

The fixed flange-to-`fr3_hand_tcp` transform is

```text
F_T_TCP = Tz(0.1034) Rz(-pi/4).
```

The yaw is introduced at the flange-to-Hand mount and the translation at the
Hand-to-TCP joint; composing the two gives the transform above.

## Manual forward kinematics

[`fr3_forward_kinematics.py`](../src/saps/physical/fr3_forward_kinematics.py)
is the authoritative mathematical implementation. It uses only NumPy and the
fixed constants above:

```text
0_T_F   = 0_T_1 ... 6_T_7 7_T_F
0_T_TCP = 0_T_F F_T_TCP.
```

Inputs are seven joint angles in radians. Translations are metres. Returned
transforms are resolved from `fr3_link0`. Pinocchio is not imported by this
implementation; it is an independent validation backend only.

## Manual geometric Jacobian

For each joint origin `p_i` and base-resolved joint axis `z_i`, the manual
Jacobian at `fr3_hand_tcp`, resolved in `fr3_link0`, is

```text
J_v,i     = z_i cross (p_TCP - p_i)
J_omega,i = z_i.
```

The returned array has shape `(6, 7)`. Rows 0 through 2 are linear motion in
metres per radian; rows 3 through 5 are angular motion in radians per radian.
Columns are `fr3_joint1` through `fr3_joint7`.

## FK and Jacobian validation

Run the deterministic validation against the actual local description:

```bash
make validate-fr3-kinematics
```

The default `FRANKA_ROS2_WS=$HOME/franka_ros2_ws` derives both the description
and install paths. If the workspace is elsewhere, override only its root, for
example `make validate-fr3-kinematics FRANKA_ROS2_WS=/home/franka_ros2_ws`.

With Pinocchio 4.0.0 and expanded-URDF SHA-256
`23d7f794e78ea7947a871cadacc41f0f66444b2a60208b07233b6cfffcc1d766`, the
2026-09-03 validation passed:

| Check | Worst maximum error |
|---|---:|
| Flange FK position, 1000 random valid configurations | `6.83e-16 m` |
| Flange FK orientation | `2.30e-15 rad` |
| Hand TCP FK position | `8.25e-16 m` |
| Hand TCP FK orientation | `2.21e-15 rad` |
| Analytical Jacobian vs Pinocchio | `1.94e-15` |
| Analytical Jacobian vs centred finite difference (`epsilon=1e-7`) | `3.57e-9` |

The recorded M3 state also agreed at machine precision. The epsilon sweep had
maximum errors `1.67e-9`, `2.30e-11`, `1.61e-10`, `1.81e-9`, `1.06e-8`, and
`9.13e-8` for epsilon from `1e-4` through `1e-9`, showing the expected
numerical-error minimum near `1e-5`.

## Finite-action `J delta_q` versus exact FK

The saved accepted M3 input is
`outputs/physical_pi05_droid_m3/m3_live_20260828T1548Z/`, containing five
measured FR3 joint observations and five policy chunks. Reproduce the
non-actuating diagnostic with:

```bash
make validate-droid-fr3-mapping \
  M3_MAPPING_RUN=outputs/physical_pi05_droid_m3/m3_live_20260828T1548Z
```

For each action it compares the differential prediction

```text
delta_x_linear = J(q) delta_q
```

with the manual exact finite step

```text
delta_p     = p(q + delta_q) - p(q)
delta_theta = Log(R(q + delta_q) R(q)^T).
```

The exact manual-FK displacement agrees with Pinocchio's finite-step reference
to numerical precision (worst component error `4.37e-16 m or rad`). In the
intended 8-action kinematic/model-based
rollouts (40 steps total), translation direction cosine had minimum `0.99605`
and median `0.99914`; rotation direction cosine had minimum `0.99771` and
median `0.99927`. Median linearization errors were `1.09 mm` and
`2.97e-3 rad`; maxima were `2.81 mm` and `6.83e-3 rad`. Thus direction
agreement is very high, while finite-step linearization error is small but
nonzero.

The 15-action model rollout (75 steps) is only a full-chunk stress diagnostic.
Its minimum translation and rotation direction cosines were `0.99216` and
`0.99553`, and maximum errors were `2.92 mm` and `1.04e-2 rad`. Neither
rollout is measured FR3 execution. No next-state joint-limit violation occurred
in either idealized rollout.

## Simulation and physical action spaces

The simulation baseline did not use explicit Cartesian scales `s_t` and `s_r`.
OpenPI returned checkpoint-unnormalized LIBERO action coordinates, the
SpaceMouse was mapped into that same simulator-native action space, and SAPS
operated directly there. The committed simulation SpaceMouse gains
`translation=0.40` and `rotation=0.08` are simulation action-space calibration;
they must not be transferred as physical SI gains.

## Current unresolved design issues

The following are open design and validation questions, not decisions:

- dimensionless physical SAPS Cartesian scales `s_t` and `s_r`;
- policy-preserving Cartesian correction to joint-command mapping;
- execution scheduling between the 15 Hz reference semantics and lower-level
  control;
- streaming observation freshness and the policy/action age contract;
- physical safety supervision, including limits, collision handling,
  singularities, workspace constraints, and operator stop behavior.

The lab Servo values `0.4 m/s` and `0.8 rad/s` are existing controller settings,
not SAPS normalization. No Servo gain, pseudoinverse, IK rule, or actuator path
is selected by this document.
