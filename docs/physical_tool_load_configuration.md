# FR3 mounted-tool configuration reference

2026-11-10. Preparation only: no robot settings, controller settings or motion
were changed. Workspace clearance and emergency-stop availability are
operator verified. This procedure addresses the remaining tool/load mismatch;
it does not change the [development rollout](physical_development_rollout.md).

The subsequent [Fusion-v13 assembly audit](validation/2026-11-10_fusion_v13_tool_profile/README.md)
contains the pinned CAD identities, all flange-relative tool transforms,
component inventory, calculation method and incomplete Desk profile. Its source
is `experiment/isaacsim-fr3-droid`; it does not replace missing physical mass
properties with simulation values or certify the old nominal local draft.

**This is not a software prerequisite for joint-space development execution.**
The autonomous execution path does not reject missing tool mass/CoM/inertia,
`assembly_verified=false`, or the historical 0.73 kg readback. `mounted_tool`
records reference data only. Tool/load findings remain informational in
readiness reports; existing runtime motion checks are separate. This document
describes how to correct the robot's dynamics model, not an additional approval
gate. An unverified model is not proof of unsafe motion or proof of safety.

Grasped objects change the physical load, and a fixed model is necessarily an
approximation over that task. Exact payload identification at each grasp is not
required by SAPS. Correcting a persistent tool mismatch can improve model-based
compensation and force estimation; it does not eliminate changing object loads.

## Minimum correction

### Actual editable location

The active EE properties live on the robot. They are not read from SAPS JSON,
the impedance-controller YAML, or the ROS URDF. In Desk, open **Settings →
End-Effector → Mechanical Data (edit)** and use **Download** to export
`endeffector-config.json`. Keep the original as a backup; prepare changes in a
separate copy. **Upload** applies the edited mechanical configuration to the
robot and is a separate supervised operation, not something performed by SAPS.
UI wording can vary with robot software version.

The relevant JSON fields are `mass`, `centerOfMass`, `inertia`, and
`transformation`. The last is the nominal flange-to-tool transform; preserve it
and the existing SDK adjustable transform unless deliberately changing the
controller reference point. The published `F_T_EE` is their composite, so it
must not be substituted blindly for this exported field.

The actual robot uses the newer Profiles/Devices layout. Its supplied export is
`configs/local/Franka Hand.endeffector-profile.json`, with `inertial.mass`,
`inertial.centerOfMass`, `inertial.inertia` (six symmetric tensor entries),
`tcp.rotation`, `tcp.translation`, `id`, `name`, and `deviceId`. Use this schema
for a profile import rather than the older mechanical-data schema described
above. The down-arrow on the profile row exports it; **Profiles → IMPORT** is
distinct from **Devices → INSTALL**.

The original export is preserved. A separate
`configs/local/Robotiq 2F-85.DRAFT-NOT-FOR-IMPORT.json` records the nominal 0.921 kg
and `[0, 0, 0.06]` m CoM and preserves the exported TCP exactly. The CoM is still
manufacturer reference data, pending confirmation of its mounting-frame origin.
This draft intentionally has `inertial.inertia=null` and `deviceId=null` and is
**not import-ready**. The original `ee-gripper` binding belongs to the Franka
Hand device; a profile exported from the installed **Generic Device** is needed
to establish the replacement binding without guessing its identifier. The draft
has a new UUID and name so it does not reuse the original profile identity.
The C4 `mounted_tool` values likewise remain reference data, not a Desk import.

Franka documents the fields and nominal flange transform in its
[end-effector configuration reference](https://download.franka.de/ride-cli-doc-v1.1.1/end_effector_commissioning.html).

Replace the historical Franka Hand **end-effector dynamic configuration** in
the robot's administrator interface with the verified installed fixed assembly:
Robotiq, its actual coupling/adapters, and camera/bracket hardware rigidly
attached to the flange/tool. Verify the ZED Mini's attachment link before
including it: a camera mounted on an upstream arm link is not a fixed flange
payload. Keep the external payload separately allocated, without double
counting. No controller, policy, gripper or ROS package change is needed to
configure these SDK properties. Changing URDF inertials or adding ROS parameters
does not set them.

The November 9 capture in
`~/franka_ehsan_ros2_ws/c4_runtime_inspection_20261109T161032Z/tool_state/inspection.json`
reports:

| SDK quantity | Captured configured value |
|---|---|
| End-effector mass | 0.73 kg |
| End-effector CoM, flange coordinates | approximately `[-0.01, 0, 0.03]` m |
| End-effector inertia about CoM | diagonal `[0.001, 0.0025, 0.0017]` kg m² |
| External load mass/inertia | zero |
| Total mass | 0.73 kg |
| `F_T_EE` | translation `[0, 0, 0.1034]` m; yaw −45° |
| `EE_T_K` | identity |

These are configured values, not measurements. The gripper's supplied 0.921 kg
already exceeds the configured total before accounting for the ZED and mount.
There is not yet enough information to produce a correct numerical replacement.

## Installed interfaces and conventions

The installed libfranka is 0.15.0. Its
[Robot API](https://frankarobotics.github.io/libfranka/0.15.0/classfranka_1_1Robot.html)
and local `/usr/local/include/franka/robot.h` distinguish these settings:

| Setting | Supported configuration route | Meaning |
|---|---|---|
| End-effector mass, CoM, inertia; nominal frame | Robot administrator interface | Fixed tool definition; no setter for these dynamic EE fields in this ROS service server |
| External payload | `/service_server/set_load`, `franka_msgs/srv/SetLoad` → `Robot::setLoad` | Replaces external-load fields, which are then combined with the configured EE; it does not replace EE fields |
| Adjustable end-effector frame | `/service_server/set_tcp_frame`, `franka_msgs/srv/SetTCPFrame` → `Robot::setEE` | Takes **`NE_T_EE`**, not `F_T_EE`, despite the abbreviated service name/comment |
| Stiffness frame | `/service_server/set_stiffness_frame` → `Robot::setK` | Takes `EE_T_K`; not needed for this inertial correction |

`F` is the robot flange frame at its mounting surface; `NE` is the configured
nominal end-effector frame; `EE` is the adjustable end-effector frame; `K` is
the stiffness frame. A matrix `A_T_B` maps B coordinates into A coordinates:
`p_A = R_AB p_B + t_AB`. Thus `F_T_EE = F_T_NE * NE_T_EE`.
SDK transforms are 4×4 column-major arrays; translations are metres. SDK CoMs
are flange-coordinate vectors in metres. Inertias are about the corresponding
CoM, with axes parallel to F, in kg m², as 3×3 column-major arrays:
`[Ixx, Iyx, Izx, Ixy, Iyy, Izy, Ixz, Iyz, Izz]`.
The installed `libfranka/src/load_calculations.cpp` combines the tensors in
these common axes using parallel-axis shifts, without rotating them.

The current hybrid controller calls the SDK **end-effector** Jacobian and
Coriolis model. Its 103.4 mm point need not be a fingertip TCP for joint-space
policy interpretation, but changing it changes the hybrid impedance law.
Preserve the current numeric `F_T_EE`, `EE_T_K` and gains during this correction.
Do not select another nominal frame inadvertently when replacing the tool
definition. If the administrator workflow changes `F_T_NE`, preserving the same
composite requires `NE_T_EE_new = inverse(F_T_NE_new) * F_T_EE_old`; review that
separately with the actual nominal transform. No numerical frame setter is
prepared from the published composite alone.

The ROS state message does not expose `F_T_NE` and `NE_T_EE` separately. Also,
this installed semantic component assigns misleading URDF-derived headers
(`fr3_link6`/`fr3_hand_tcp`) while copying SDK numeric values directly. Interpret
`f_t_ee` and the inertia fields by their SDK semantics above; do not transform
them using those headers. This label issue does not change the raw SDK model.

## Properties still needed

The C4 template now records the confirmed nominal gripper values and sourced
camera mass under `mounted_tool`; prepared profiles preserve operator-supplied
tool evidence. These fields are captured with the requested configuration in
existing provenance. They do not call setters, update the SDK, or certify the
assembly. Existing local profiles are not overwritten automatically.

The [Stereolabs ZED Mini datasheet](https://support.stereolabs.com/hc/article_attachments/27901442262551)
specifies **62.9 g**. The [DROID mounting instructions](https://droid-dataset.github.io/droid/hardware-setup/assembly.html#mounting-hand-camera-on-robot)
identify the custom wrist mount and fasteners, but give no mass, CoM or inertia
for the printed bracket. The two catalog masses sum to **0.9839 kg**, before
resolving coupling overlap and additional hardware; this is not the measured
assembly mass. Do not treat the camera's mass as zero in a verified tool model.

The [Robotiq manual, page 82, section 5.2.3](https://blog.robotiq.com/hubfs/support-files/2F-85_2F-140_General_PDF_20210623.pdf#page=82)
also supplies an approximate inertia for fully open fingers: diagonal
`[4180, 5080, 1250] kg mm²`, or `[0.00418, 0.00508, 0.00125] kg m²`.
That page does not explicitly establish the tensor origin and axes for SDK use.
It is retained as manufacturer reference data, not inserted as a verified
CoM/flange-axis inertia. Do not parallel-axis shift it until its origin is known.

The manufacturer
[Robotiq table, page 1](https://blog.robotiq.com/hubfs/support-files/TCP_20and_20Center_20of_20Mass_20of_20Robotiq_20Products_20V1.1_20210623.pdf)
supports 921 g and CoM `[0, 0, 60]` mm. Its note includes a coupling in
non-Robotiq-wrist-camera configurations. Identify the installed product/coupling
and the table's reference origin before treating this as bare-gripper data.
Its “Camera” combination refers to Robotiq hardware, not the ZED Mini.

Needed for the actual fixed assembly:

- ZED Mini CoM and inertia, including its actual bracket/fasteners and
  any attached accessories not included elsewhere. Establish the physical
  attachment link; a ROS camera frame alone does not prove mechanical attachment.
  If mounted upstream, do not compensate it by inventing a flange-fixed load.
- Coupling/adapter identity, mass properties and included/excluded components
  of the 921 g figure. No verified adapter mass or mounting transform was found
  in the inspected workspace resources; do not substitute Franka Hand data.
- Flange-to-component reference transforms, including coupling thickness and
  orientation. The SDK 103.4 mm frame is not a measured mounting transform.
- CoM-referenced inertia tensors and their axes for the gripper and remaining
  components, or verified properties of the complete assembly. Mass and CoM
  alone do not determine inertia. Use applicable manufacturer data, verified
  CAD mass properties or measurement; do not insert zero inertia or a guessed
  diagonal. Identify finger/pad configuration and any relevant variation.
- Intended grasped-object load properties/allocation for the first task.
  Zero external load describes no additional payload; it does not characterize
  an object once grasped.

For components with verified mass `m_i`, local CoM `c_i`, CoM inertia `I_i`
and transform `(R_i, t_i)` into F:

```text
c_Fi = t_i + R_i c_i
M = sum(m_i)
c_F = sum(m_i c_Fi) / M
d_i = c_Fi - c_F
I_F,CoM = sum(R_i I_i R_i^T + m_i ((d_i^T d_i) Identity - d_i d_i^T))
```

Convert mm → m and g → kg before combining. Confirm symmetry, finite values,
physical principal moments and units. Do not double-count a coupling, camera
mount or object between EE and external load.

## Supervised configuration and verification procedure

1. In the hardware terminal, capture the current state using the read-only
   commands below. Save/export the current administrator tool definition and
   its nominal frame too; record the robot software version and interface units.
2. Complete the missing assembly properties and convert them to the SDK
   conventions. The fixed assembly belongs in the EE definition; an actual
   additional payload belongs in the external-load definition.
3. With rollout/target production stopped, use the existing supervised robot
   stop/deactivation procedure and leave FCI control before editing the robot
   administrator configuration. A command hold is not deactivation. Do not
   change load/frame settings during the running torque-control loop.
   [FCI and Desk control are exclusive](https://frankarobotics.github.io/docs/overview.html).
4. In the robot administrator interface, replace the historical EE dynamics
   with the verified fixed-assembly mass, CoM and inertia, checking that the UI's
   origin/axes/units match the values entered. Preserve the existing nominal
   and adjustable frame composition and stiffness frame. Exact UI field names
   depend on the robot software version, which the saved ROS capture does not
   establish. Save the selected tool definition; do not retain an automatic
   Franka Hand preset that overwrites it.
5. Restore the existing ROS bringup under supervision, initially without a
   rollout. Capture again and compare EE, load, total, `F_T_EE`, `EE_T_K`, and
   controller gains. If load remains correctly zero, **no `set_load` call is
   needed**. Verify again after reconnect/restart rather than assuming persistence.
6. Check `m_total = m_ee + m_load`, the weighted total CoM, and the parallel-axis
   combined inertia against the independently prepared assembly values.
   Confirm that numeric `F_T_EE`, `EE_T_K` and hybrid gains are unchanged within
   readback precision. A service success or a plausible mass alone is insufficient.
   Reuse the existing observation/shadow checks before the supervised one-chunk
   command in the development guide; no execution authorization is added here.

Read-only commands, in the hardware ROS environment (not executed in this task):

```bash
cd ~/franka_ehsan_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 interface show franka_msgs/srv/SetLoad
ros2 interface show franka_msgs/srv/SetTCPFrame
ros2 service type /service_server/set_load
ros2 service type /service_server/set_tcp_frame
capture_dir="/tmp/c4_tool_readback_$(date -u +%Y%m%dT%H%M%SZ)"
PYTHONNOUSERSITE=1 ros2 run fr3_lab_stack inspect_tool_model.py \
  --timeout 10 --state-topic /franka_robot_state_broadcaster/robot_state \
  --output-dir "$capture_dir"
ros2 param dump /fr3_streaming_joint_impedance_controller \
  > "$capture_dir/impedance_controller.yaml"
timeout 10s ros2 topic echo /franka_robot_state_broadcaster/robot_state \
  --once --field inertia_total
```

The inspector's historical `hardware_match_verified=false`/NO-GO label is not
an automatic comparison with these new measurements. Compare its recorded
numeric fields; no report label or runtime guard is changed by this procedure.

Only if a separately characterized external payload must be configured, the
existing service accepts this request shape (placeholders are intentionally
not numerical defaults):

```text
mass: VERIFIED_EXTERNAL_KG
center_of_mass: [VERIFIED_F_X_M, VERIFIED_F_Y_M, VERIFIED_F_Z_M]
load_inertia: [IXX, IYX, IZX, IXY, IYY, IZY, IXZ, IYZ, IZZ]
```

After review, save numerical values as `reviewed_payload_request.yaml`.
The following is a **robot-setting write**, for the operator only, with the
robot stationary and torque control stopped; it was not executed here:

```bash
ros2 service call /service_server/set_load franka_msgs/srv/SetLoad \
  "$(cat reviewed_payload_request.yaml)"
```

Require `success: true`, inspect `error`, and repeat the read-only capture.
Calling this with the gripper's 0.921 kg while retaining the historical EE
would yield 1.651 kg before other hardware, not a corrected EE definition.
Do not use a negative or residual “difference mass” to compensate for a phantom
Hand. No `set_tcp_frame`, stiffness, gain, collision-threshold or gripper write
is required by this procedure.
