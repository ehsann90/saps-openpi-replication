# Physical M3: live observations and SpaceMouse input

## Scope and current status

M3 implements two non-actuating physical input paths:

```text
FR3 state + two serial-pinned RGB cameras -> pi05_droid observation
spnavd SpaceMouse + current FR3 orientation -> normalized fr3_link0 input
```

It also provides a staged shadow path from captured live observations through
`pi05_droid` and the M2 task-space adapter. None of these programs creates a
ROS publisher, service client, or action client. They cannot publish Servo
commands or call Franka Hand actions. M4 execution and arbitration are not
implemented.

The software, mocked contracts, and live acceptance passed on 2026-08-28.
Acceptance used wrist serial `342222073510` and temporary exterior serial
`244222076317`. The latter assignment is provenance for this acceptance run,
not a permanent scientific camera-role assumption. This state is recorded in
[`configs/physical_m3.json`](../configs/physical_m3.json).

## Provenance boundary

M3 begins at M2 commit
`b5a18d1c7c54d8c78afbec71b5f3addbfee60c5b`. The pinned OpenPI submodule
remains `15a9616a00943ada6c20a0f158e3adb39df2ccac`.

The read-only lab repositories remain:

- `franka_description` at
  `fac4949828c8b627ccb8593628212f59a8f46d00`;
- `igd_fr3_control` at
  `1ecd52e310f069d855591ff69c17e5c3412e1722`.

Their pre-existing local modifications are not changed. Each live diagnostic
records the commit, status, and SHA-256 of the full local binary diff.

## Live ROS contract

The laboratory Franka bringup separates arm and hand states:

| Source | Default topic | Message |
|---|---|---|
| measured FR3 arm state | `/franka/joint_states` | `sensor_msgs/msg/JointState` |
| Franka Hand state | `/franka_gripper/joint_states` | `sensor_msgs/msg/JointState` |
| wrist RGB | `/wrist/wrist_camera/color/image_raw` | `sensor_msgs/msg/Image` |
| exterior RGB | `/exterior/exterior_camera/color/image_raw` | `sensor_msgs/msg/Image` |

The topic names are explicit Make overrides. A completed capture records the
actual selected topics and measured source and callback rates. Use `M3_JOINT_TOPIC`,
`M3_GRIPPER_TOPIC`, `M3_WRIST_TOPIC`, and `M3_EXTERIOR_TOPIC` if the launched
names differ.

Arm messages must contain exactly `fr3_joint1` through `fr3_joint7`. Incoming
array order is ignored; the values are selected by name and returned as
`float32 (7,)`. Missing, duplicate, unexpected, non-numeric, or non-finite
states are rejected.

The live hand node publishes `_finger_joint1` and `_finger_joint2`, each at
half the physical width. The pinned hand xacro authoritatively limits each
finger to `0.04 m`, so total maximum width is `0.08 m`. M3 uses:

```text
width = finger_joint1 + finger_joint2
unclipped_closure = 1 - width / 0.08
closure = clip(unclipped_closure, 0, 1)
```

The live open state can exceed the nominal width by a small numerical boundary
error (measured positions were approximately `0.04002 m` each). M3 therefore
clips closure explicitly to `[0, 1]` and logs the physical positions, total
width, maximum width, unclipped closure, clipped closure, and whether clipping
occurred.

## Camera contract and preprocessing

The wrist camera discovered on 2026-08-28 is:

```text
model:    Intel RealSense D435I
serial:   342222073510
firmware: 5.13.0.55
```

The temporary exterior camera for this acceptance run is another D435I,
serial `244222076317`, firmware `5.17.3.10`. `M3_EXTERIOR_SERIAL` remains
mandatory, must differ from the wrist serial, and is cross-checked against
`rs-enumerate-devices -s`. Device enumeration order is never used for
selection.

Launch two RGB-only RealSense nodes with explicit serials in separate
terminals:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=wrist camera_name:=wrist_camera \
  serial_no:="'_342222073510'" \
  enable_color:=true enable_depth:=false \
  enable_infra:=false enable_infra1:=false enable_infra2:=false \
  enable_gyro:=false enable_accel:=false \
  rgb_camera.color_profile:=640x360x30
```

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=exterior camera_name:=exterior_camera \
  serial_no:="'_244222076317'" \
  enable_color:=true enable_depth:=false \
  enable_infra:=false enable_infra1:=false enable_infra2:=false \
  enable_gyro:=false enable_accel:=false \
  rgb_camera.color_profile:=640x480x15
```

During acceptance the wrist negotiated `640x360x30`. The temporary exterior
camera was attached over USB 2.1 and rejected `640x360x30`, then negotiated
`640x480x15`. Its frames were explicitly centre-cropped from 4:3 to 16:9 and
resized; this measured connection/profile is run provenance, not a permanent
camera requirement.

Confirm the serial parameters and topics read-only before capture. RGB inputs
accept only explicit `rgb8` or `bgr8`; padded ROS row strides are handled. Each
frame is converted to RGB `uint8`, centre-cropped to 16:9 if needed, then
resized deterministically to `(180, 320, 3)`. Downscaling uses OpenCV
`INTER_AREA`; upscaling uses `INTER_LINEAR`. The serial, model, topic, native
shape, encoding, crop, resize, final shape, source timestamp, and age are
recorded. M3 performs no depth processing or hand-eye correction.

## Observation timing and schema

Every source records its ROS header stamp and local monotonic receive time.
Assembly records ROS and monotonic time, source ages, receive ages, oldest and
newest source stamps, and cross-source skew. Diagnostic defaults reject a
source older than `0.5 s` or skew above `0.25 s`. These are measurement guards,
not M4 execution thresholds.

The capture and SpaceMouse diagnostics also subscribe to `/tf` and
`/tf_static`, explicitly wait for a fresh `fr3_link0 <- fr3_hand_tcp`
transform, and record readiness attempts, the first lookup error, transform
stamp, age, translation, quaternion, and rotation. SpaceMouse samples use the
live TF orientation; Pinocchio FK is retained only as a recorded cross-check.

The assembled policy dictionary has exactly:

```text
observation/exterior_image_1_left  uint8 (180, 320, 3)
observation/wrist_image_left       uint8 (180, 320, 3)
observation/joint_position         float32 (7,)
observation/gripper_position       float32 (1,)
prompt                             non-empty string
```

No old frame is accepted after it exceeds the configured age. After the first
observation, both camera stamps must advance before another observation is
captured, so rapid joint callbacks cannot duplicate either image pair.

## Shadow inference workflow

First start the pinned DROID policy server in its own terminal:

```bash
make droid-policy-server
```

With the existing read-only FR3 stack and both serial-pinned camera nodes
running, execute a new shadow identity:

```bash
make physical-m3-shadow \
  M3_RUN_ID=m3_$(date -u +%Y%m%dT%H%M%SZ) \
  M3_EXTERIOR_SERIAL=244222076317 \
  M3_PROMPT="pick up the object"
```

The target performs three explicit stages:

1. system ROS Python captures several complete observations into
   `observation_bundle.npz` and `run.json`;
2. the validated runtime container queries `pi05_droid`, requires `(15, 8)`,
   and writes `policy_actions.npz` and `shadow_policy.json`;
3. system ROS Python projects actions at each observation's captured `q` and
   writes `shadow_projection.json`.

The projection stage applies all 15 actions at the same captured state for
distribution diagnostics and logs actions 0, 7, and 14 in detail. It does not
roll forward an imagined robot state. Eventual M4 execution must reacquire `q`
and `J(q)` for every executed action.

All artifacts live below ignored `outputs/physical_pi05_droid_m3/<run-id>/`.
Every stage refuses to overwrite an existing artifact. Stop the policy server
after the run:

```bash
make policy-stop
```

The accepted live run is
`outputs/physical_pi05_droid_m3/m3_live_20260828T1548Z/`. It contains five
physical observations, five `(15, 8)` shadow action chunks, and 75 projections
at captured live FR3 states. Shadow inference records wall-clock observation
age at call start and completion in addition to client and model timing.

## Physical SpaceMouse contract

Physical M3 uses `spnavd`, not the simulation evdev backend. The new backend
opens only the spnavd client socket. It never opens the evdev node, publishes
`/servo_node/delta_twist_cmds`, switches Servo command type, unpauses Servo, or
constructs Franka action clients.

The pinned lab mapping is preserved exactly. For raw spnav translation
`(tx, ty, tz)` and rotation `(rx, ry, rz)`:

```text
raw_normalized = raw / 500
abs(component) < 0.3 -> 0
h_tcp = [-tz, tx, ty, -rz, rx, ry]
```

The deadzone zeros components without rescaling. M3 adds no gain and no clip;
values above nominal unit magnitude remain observable. The latest actual
motion-event receive time is tracked. After `0.25 s`, all six motion dimensions
become zero and `stale_input=true`, while the last raw event remains in the
diagnostic.

Buttons expose explicit transient intent under the existing
`HumanInputSample` convention:

```text
button 0: open  -> -1
no button:      ->  0
button 1: close -> +1
```

Close has deterministic priority if both are pressed. No command is latched,
and no intent is sent to the hand. Live acceptance confirmed physical button 0
as OPEN and button 1 as CLOSE.

With the subscriber-only FR3 stack running, collect deliberate operator motion:

```bash
make physical-m3-spacemouse \
  M3_SPACEMOUSE_RUN_ID=spnav_$(date -u +%Y%m%dT%H%M%SZ) \
  M3_SPACEMOUSE_DURATION=10
```

The output logs connection and physical-device status, raw and mapped axes,
TCP- and base-frame normalized motion, motion activity, stale state, buttons,
event age, current joints, the live TF transform used, and its Pinocchio
rotation disagreement.

Acceptance evidence is in `spnav_live_20260828T1552Z` and
`spnav_buttons_20260828T1554Z`. Across the two runs every raw axis crossed the
deadzone in both directions, button 0 produced open intent, button 1 produced
close intent, and every stale sample had zero six-dimensional motion.

## Common SAPS frame

SpaceMouse components enter M3 expressed along `fr3_hand_tcp` axes. M2 policy
motion is expressed along `fr3_link0` axes. Both refer to motion at the TCP
point, so M3 resolves the components with the current TCP orientation:

```text
h_base = diag(R_base_tcp, R_base_tcp) h_tcp
```

There is no `p x omega` term because the reference point is not changed. With
M2 normalization

```text
S = diag(0.075, 0.075, 0.075, 0.15, 0.15, 0.15)
```

each block is isotropic, so
`S^-1 diag(R,R) S = diag(R,R)`. Translation norm, rotation norm, overall norm,
dot products, and cosine geometry are preserved under a common proper rotation.
Unit tests verify those properties numerically. MoveIt Servo's `0.4` and `0.8`
execution scales are not used.

## Policy/human distribution comparison

After both diagnostics exist, compare their unmodified normalized values:

```bash
make physical-m3-compare \
  M3_PROJECTION=outputs/physical_pi05_droid_m3/M3_RUN/shadow_projection.json \
  M3_SPACEMOUSE_OUTPUT=outputs/physical_pi05_droid_m3/SPNAV_RUN/spnav.json
```

The report includes component ranges, translation/rotation/overall norms,
above-unit frequencies, active-human medians, and human-to-policy median ratios.
It refuses a SpaceMouse log with no active motion. It does not tune either
source; a serious mismatch is recorded for M4.

The accepted comparison is
`outputs/physical_pi05_droid_m3/comparison_live_20260828T1556Z.json`. It records
the unmodified policy and active-human distributions; no gain or normalization
was tuned.

## M4 boundary

M4 still owns all commands and safety decisions: 15 Hz policy to 50 Hz Servo
scheduling, chunk holding or interpolation, physical motion scale, final Servo
command frame, workspace/velocity/singularity clipping, observation/action age
limits, arbitration, and physical gripper mapping. M3 outputs are diagnostic
candidates only and are never robot commands.
