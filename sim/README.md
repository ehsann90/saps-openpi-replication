# Isaac Sim FR3 / DROID Simulation

This directory contains the Isaac Sim development environment for running and
evaluating the OpenPI pi0.5 DROID-finetuned policy with a simulated Franka FR3.

The simulation is intentionally isolated from the physical FR3 runtime. It
reuses the same OpenPI/DROID policy semantics where appropriate, but does not
require ROS, `franka_ros2`, the physical streaming controller, or robot hardware.

Current branch:

```text
experiment/isaacsim-fr3-droid
```

## Scope

The simulator currently provides:
- native Isaac Sim 6.1 Franka FR3 asset;
- reproducible robot, table, and object scene;
- deterministic FR3 home configuration;
- Franka Hand open/close actuation;
- Lula inverse kinematics for deterministic validation motions;
- finger/cube collision inspection;
- PhysX contact reporting;
- validated contact-limited grasp, lift, and release of a simple rigid cube.
- two virtual, RGB-only DROID-like camera views and a capture validator.

Planned next steps are exact DROID/OpenPI observation construction, policy-server
connection, 15 Hz action execution, and closed-loop pi0.5 rollouts.

## Requirements

Validated environment:

```text
Isaac Sim:       6.1
Ubuntu:          24.04
GPU:             NVIDIA RTX 5080 Laptop GPU, 16 GB
NVIDIA driver:   595.91.07
```

Isaac Sim is installed separately under:

```text
~/isaacsim
```

Do not install Isaac Sim as a normal Python dependency of this repository.

The FR3 USD is resolved through the Isaac asset root:

```text
/Isaac/Robots/FrankaRobotics/FrankaFR3/fr3.usd
```

No NVIDIA robot assets are copied into this repository.


## Directory structure

```text
sim/
├── README.md
├── assets/
├── configs/
│   └── fr3_droid_scene.json
├── scenes/
├── scripts/
│   ├── launch_scene.py
│   ├── gripper_contact_test.py
│   └── camera_validation.py
└── src/
    └── isaac_fr3/
        ├── __init__.py
        ├── scene.py
        ├── cameras.py
        └── grasp_poses.py
```

`sim/src/isaac_fr3/scene.py` is the reusable source of truth for the baseline
scene. Scripts should reuse this implementation rather than duplicate scene
construction.

The JSON configuration is the source of truth for scene geometry and robot
configuration. Saved USD files, if added later, are derived/convenience
artifacts rather than the authoritative experiment definition.

## Baseline scene
### FR3 home
Arm configuration:

```text
[0.0, -0.4, 0.0, -1.9, 0.0, 1.5, 0.0] rad
```

Open finger positions:

```text
[0.04, 0.04] m
```

Typical measured TCP pose at home:

```text
position ≈ [0.4184, 0.0001, 0.5542] m
```

### Table

```text
center = [0.55, 0.0, 0.375] m
size   = [0.70, 0.80, 0.05] m
```

### Target object

The current diagnostic target is a red rigid cube:

```text
center = [0.55, 0.0, 0.42] m
size   = [0.04, 0.04, 0.04] m
mass   = 0.05 kg
```

After settling, its center remains approximately:

```text
[0.55, 0.0, 0.42] m
```

## Launching the baseline scene

Run with Isaac Sim's bundled Python:

```bash
cd ~/isaacsim

./python.sh -u \
  ~/MyProjects/saps-openpi-replication/sim/scripts/launch_scene.py \
  --config \
  ~/MyProjects/saps-openpi-replication/sim/configs/fr3_droid_scene.json
```

Headless mode is available with:

```text
--headless
```

### Gripper/contact validation

Run:

```bash
cd ~/isaacsim

./python.sh -u \
  ~/MyProjects/saps-openpi-replication/sim/scripts/gripper_contact_test.py \
  --config \
  ~/MyProjects/saps-openpi-replication/sim/configs/fr3_droid_scene.json
```

The test performs:

```text
HOME
→ PREGRASP
→ GRASP POSE
→ CLOSE
→ LIFT
→ RELEASE
```

The cube remains at the nominal tabletop location. The robot is moved to the
cube; the object is not repositioned into the gripper to manufacture a grasp.

## SIM-P2 validated grasp behavior

The default native finger-drive realization was not accepted.

An abrupt zero-width command with the native 100 N maximum drive effort caused
excessive simulated penetration.

The validated simulated CLOSE realization preserves the high-level DROID intent

```text
CLOSE -> desired total width = 0
```

but applies it as:

```text
total-width closing speed = 0.100 m/s
per-finger target speed   = 0.050 m/s
active finger max effort  = 20 N
```

The speed and effort are aligned with the validated physical Franka Hand grasp
configuration. The 20 N simulated drive limit is a command parameter, not a
calibrated measurement of force at the object.

Typical qualifying result:

```text
initial width:               0.080000 m
first bilateral contact:     ~0.056345 m
final width:                 ~0.055750 m
DROID gripper observation:   ~0.3031

TCP lift:                    ~0.04983 m
cube lift:                   ~0.04983 m
bilateral loaded contact:    retained

release:
finger width ->              0.080000 m
cube returns to table
loaded finger/cube contact -> false
```

Qualifying flags:

```text
PRE_CLOSE_GEOMETRY:     PASS
CONTACT_LIMITED_CLOSE:  PASS
RETENTION:              PASS
RELEASE:                PASS
SIM_P2:                 PASS
```

The validation applies only to this deterministic cube/scene. It does not yet
establish grasp robustness across object sizes, shapes, materials, poses, or
friction conditions.

## DROID arm semantics

The OpenPI/DROID arm command interpretation remains:

```text
u = clip(policy_action[0:7], -1, 1)

delta_q = 0.2 * u

q_target = fresh_measured_q + delta_q
```

Targets are constructed from fresh measured state rather than accumulated
target-to-target.

The policy action cadence is 15 Hz.

The simulator must preserve these semantics when policy execution is added.

## DROID gripper semantics

At the policy boundary:

```text
action[7] > 0.5  -> CLOSED
action[7] <= 0.5 -> OPEN
```

The observed normalized gripper state is derived from measured width:

```text
g_obs = clip(1 - measured_width / 0.08, 0, 1)
```

A CLOSED policy request therefore does not imply that measured width becomes
zero. Contact determines the physical/simulated final width.

## Controller boundary

The current Isaac simulation does NOT reproduce the physical FR3 hybrid
impedance controller.

Isaac currently realizes arm position targets through the native PhysX
articulation joint drives authored for the FR3 asset.

The physical robot uses a custom 1 kHz torque-level hybrid impedance controller.

Therefore simulator and physical robot currently share the DROID/OpenPI
high-level target semantics, but not the same low-level controller dynamics.

This distinction must be retained when comparing simulation and physical
results.

## SIM-P3 virtual RGB cameras

`sim/configs/fr3_droid_scene.json` defines two independent pinhole cameras. The
wrist camera prim is a child of `/World/fr3/fr3_hand`, so its USD transform is
fixed in the hand frame. The external camera is a child of `/World` and stays
world-fixed. Neither camera adds collision geometry. Each camera has one RGB
render product at 320 × 180; no depth, right stereo, or segmentation annotators
are created.

Camera poses use metres and scalar-first `[w, x, y, z]` quaternions. The
configured rotations are **USD camera** rotations: +X image right, +Y image up,
and -Z forward. Conventional optical coordinates are +X right, +Y down, +Z
forward. Captured RGB arrays have top-left origin and are not flipped,
mirrored, or channel swapped.

| Camera | Position and quaternion | Horizontal FOV | Provenance |
| --- | --- | --- | --- |
| Wrist (`zed_mini_droid_like`) | Hand-relative `[-0.079489144607, 0.031927806250, 0.002650643753]` m; `[0.123099065614, 0.696279310194, -0.696337080774, -0.123111381195]` | 66° | CAD-derived DROID ZED Mini mount with nominal left optical center; manufacturer rectified HD1080 reference FOV |
| External (`zed2_droid_like`) | World `[1.05, -0.85, 1.10]` m; `[0.84480517, 0.45397060, 0.13406399, 0.24948300]` | 84° | Representative simulator viewpoint; manufacturer rectified HD1080 ZED 2 FOV |

The FOVs approximate left rectified pinhole views. Isaac uses a *virtual* 36 mm
horizontal aperture, with focal length computed as `aperture / (2 tan(FOV/2))`.
That aperture is a rendering parameter, not a measured ZED sensor width. The
camera config and saved metadata record the realized focal length, apertures,
intrinsics matrix, and poses. The clip range is 0.01 to 10 m; the near plane
must be closer than the wrist camera's hand and tabletop view.

The [DROID dataset description](https://github.com/droid-dataset/droid/blob/main/docs/the-droid-dataset.md)
documents 180 × 320 × 3 left images. The [DROID schema](https://github.com/droid-dataset/droid/blob/main/droid/postprocessing/schema.py)
stores episode camera calibration, and the [DROID hardware description](https://droid-dataset.github.io/)
describes adjustable external ZED 2 viewpoints and a wrist-mounted ZED Mini.
There is no single external DROID pose. A [separate calibrated DROID
extrinsics release](https://github.com/Stanford-TML/cloak/blob/main/examples/render_extrinsics.py)
contains per-episode camera poses in an `attachment_site` end-effector frame,
using OpenCV optical axes. For example, its AUTOLab 2023-07-07 09:42:23 entry
is `[-0.07127, 0.03159, 0.02091, -0.34140, 0.01211, -1.57947]` (metres,
XYZ Euler radians). That real episode's frame and Robotiq mount are not a
transferable transform for Isaac's `fr3_hand` and Franka Hand. The
[Stereolabs rectified FOV table](https://support.stereolabs.com/articles/8809264540-what-is-the-camera-focal-length-and-field-of-view)
supplies the 66° and 84° HD1080 reference values. The [ZED Mini specifications](https://docs.stereolabs.com/docs/products/cameras/zed/specifications)
give 102° horizontal native maximum and a 63 mm stereo baseline. DROID's
180 × 320 image shape alone does not establish its effective rectified or
cropped HFOV. Thus 66° remains a provisional rendering intrinsic pending
episode-specific DROID calibration; no intrinsic has been inferred from the
native maximum. Camera geometry remains explicit for later comparisons.

The wrist extrinsic is `T_fr3_hand_from_usd_camera_nominal`, directly parented
to `/World/fr3/fr3_hand`. The supplied transform package derives mount
placement from the CAD assembly hierarchy, verifies hand alignment against
Isaac's standard FR3 hand frame to numerical precision, and transfers the
nominal ZED Mini left optical center from Stereolabs URDF/mesh registration.
Its nominal camera-center to left-optical-center displacement is 0.0315 m.
The supplied quaternion already uses native USD camera axes; no further
optical-frame conversion or 180° X rotation is applied. This is mechanically
CAD-derived with nominal optical position accuracy around the millimetre scale,
not a measured physical hand-eye calibration. The Isaac
`Stereolabs/ZED_X_mini/ZED_X_Mini.usd` asset represents the distinct ZED X Mini
product. It is not used here, nor are its intrinsics transferred to the
original ZED Mini virtual camera.

Raw captures are `(180, 320, 3)` `uint8` RGB. The validation script calls the
pinned `openpi_client.image_tools.resize_with_pad(image, 224, 224)` used by the
[OpenPI DROID example](https://github.com/Physical-Intelligence/openpi/blob/main/examples/droid/main.py):
PIL bilinear scaling, centered zero padding, yielding `(224, 224, 3)` `uint8`
RGB. The current task only prepares images; it makes no policy request.

With OpenPI off, run from `~/isaacsim`:

```bash
./python.sh -u \
  ~/MyProjects/saps-openpi-replication/sim/scripts/camera_validation.py \
  --config \
  ~/MyProjects/saps-openpi-replication/sim/configs/fr3_droid_scene.json
```

Add `--headless` for a non-GUI capture. Every run creates a unique directory
under `outputs/isaac_camera_validation/` with raw and processed PNGs for HOME,
PREGRASP, and GRASP_POSE_OPEN, plus `metadata.json`. The approach targets use
the SIM-P2 settled cube, finger-tip collider midpoint, home TCP orientation,
and Lula IK. The gripper remains open; there is no close or lift. The validator
prints and records configured-versus-measured hand-relative pose errors and
checks rigid attachment through arm motion. The metadata records each stage's
arm state, camera poses, and projections of the cube and
points 5 cm away in world X/Y. Those projections test FOV coverage, while the
saved images show actual occlusion. Inspect the images before using the camera
setup for policy experiments; visibility is an observation, not a pose-tuning
criterion for the CAD-derived extrinsic.

## Known warnings

Isaac currently reports known warnings for:

```text
/World/fr3/fr3_hand_tcp
/World/fr3/fr3_link8
```

regarding mass/inertia properties, and a TGS articulation velocity-iteration
warning.

These have not produced an observed failure in the current baseline and the
upstream NVIDIA FR3 asset has not been modified.

## Resource/startup note

On the RTX 5080 laptop, pi0.5-DROID and Isaac Sim can coexist, but GPU memory is
tight.

The validated startup order is:

1. fully load the OpenPI pi0.5-DROID server;
2. wait until the server is listening;
3. start Isaac Sim.

Starting Isaac first and restoring the OpenPI checkpoint afterward previously
caused a transient host-memory OOM kill.

For scene and physics development, OpenPI should normally remain off.

## Validation before commits

At minimum:

```bash
python3 -m py_compile \
  sim/src/isaac_fr3/scene.py \
  sim/src/isaac_fr3/cameras.py \
  sim/src/isaac_fr3/grasp_poses.py \
  sim/scripts/launch_scene.py \
  sim/scripts/gripper_contact_test.py \
  sim/scripts/camera_validation.py

git diff --check
```

Physics/contact changes should additionally be validated with Isaac Sim's
bundled Python.
