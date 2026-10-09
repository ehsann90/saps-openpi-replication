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

Planned next steps are task outcome detection and broader rollout evaluation.

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
│   ├── fr3_droid_scene.json
│   └── fr3_presentation_scene.json
├── scenes/
├── scripts/
│   ├── launch_scene.py
│   ├── gripper_contact_test.py
│   ├── camera_validation.py
│   └── policy_shadow.py
└── src/
    └── isaac_fr3/
        ├── __init__.py
        ├── scene.py
        ├── cameras.py
        ├── grasp_poses.py
        └── droid_observation.py
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

Omitting `--config` selects `fr3_droid_scene.json`.

### Illustrative presentation scene

For a workspace overview image, launch the optional presentation config:

```bash
cd ~/isaacsim
./python.sh -u \
  ~/MyProjects/saps-openpi-replication/sim/scripts/launch_scene.py \
  --config \
  ~/MyProjects/saps-openpi-replication/sim/configs/fr3_presentation_scene.json
```

This scene adds four coloured cubes, an open fixed basket, and a plain light
floor for an overview image. It is an illustrative pick-and-place layout for a
presentation, separate from the single-cube baseline experiment. It does not
establish policy execution or evaluation performance in this layout.

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

SIM-P5 applies these semantics to eight actions in one native Isaac position
drive chunk; it does not yet run a closed-loop episode.

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
fixed in the hand frame. The external camera is a child of the stationary FR3
root `/World/fr3` and is fixed relative to the robot base. Neither camera adds
collision geometry. Each camera has one RGB render product at 320 × 180; no
depth, right stereo, or segmentation annotators are created.

Camera poses use metres and scalar-first `[w, x, y, z]` quaternions. The
configured rotations are **USD camera** rotations: +X image right, +Y image up,
and -Z forward. Conventional optical coordinates are +X right, +Y down, +Z
forward. Captured RGB arrays have top-left origin and are not flipped,
mirrored, or channel swapped.

| Camera | Position and quaternion | Horizontal FOV | Provenance |
| --- | --- | --- | --- |
| Wrist (`zed_mini_droid_like`) | Hand-relative `[-0.079489144607, 0.031927806250, 0.002650643753]` m; `[0.123099065614, 0.696279310194, -0.696337080774, -0.123111381195]` | 82.19068145751953° | CAD-derived DROID ZED Mini mount with nominal left optical center; selected episode rectified LEFT SVO intrinsic |
| External (`zed2_droid_like`) | FR3 root-relative `[0.18596379, -0.57591951, 0.74884339]` m; `[0.74966453, 0.59876325, -0.18201043, -0.21530877]` | 101.5525131225586° | DROID-informed side placement adapted to the simulated tabletop; selected episode rectified LEFT SVO intrinsic |

The selected DROID episode's external ZED 2 (serial `23404442`) was physically
side-mounted relative to the robot. Its left-camera calibration was constant
across all 228 frames: base-relative position
`[0.18596379, -0.57591951, 0.34884339]` m and Euler xyz
`[-1.78926414, -0.01505613, -0.57143820]` rad. Applying that position
directly in Isaac placed the camera below or near the tabletop and yielded an
under-table view: this simulator's tabletop workspace is approximately 0.4 m
above the FR3 base reference. The active pose keeps the DROID-like side-view
x/y placement and derived orientation, and raises z by 0.4 m to
`0.74884339` m. This height adaptation is specific to the current tabletop;
the active extrinsic is DROID-informed rather than the exact recorded pose.
Its parent is `/World/fr3`, not `/World`.

The camera intrinsics use the exact rectified LEFT SVO calibration of selected
DROID episode
`IRIS+7dfa2da3+2023-12-04-15h-44m-25s`. The Stereolabs ZED SDK 5.5 read
the episode SVO files and retrieved factory calibration for each camera serial.
Both streams were acquired at 1280 × 720 and 60 fps. The source values are:

| Camera | Serial | fx, fy (px) | cx, cy (px) | HFOV | VFOV |
| --- | --- | --- | --- | --- | --- |
| Wrist ZED Mini | 19824535 | 733.6873779296875, 733.6873779296875 | 650.0894775390625, 354.6379699707031 | 82.19068145751953° | 52.26985549926758° |
| External ZED 2 | 23404442 | 522.412841796875, 522.412841796875 | 640.4915161132812, 353.2311706542969 | 101.5525131225586° | 69.13614654541016° |

Rectified distortion is zero. RAW calibration includes lens distortion and is
not the simulator target. The source-calibration values and serials are also
stored in the camera config. Only HFOV controls the current Isaac projection;
the 16:9 render aspect ratio supplies the vertical aperture. Isaac uses a
*virtual* 36 mm horizontal aperture, with focal length computed as
`aperture / (2 tan(FOV/2))`.
That aperture is a rendering parameter, not a measured ZED sensor width. The
camera config and saved metadata record the realized focal length, apertures,
intrinsics matrix, and poses. The clip range is 0.01 to 10 m; the near plane
must be closer than the wrist camera's hand and tabletop view.

Acquisition was 1280 × 720, simulator and DROID policy observations are
320 × 180, and OpenPI receives 224 × 224 after resize-with-pad. The source
principal points scale to approximately (162.5224, 88.6595) px for wrist and
(160.1229, 88.3078) px for external at 320 × 180. The current centered Isaac
projection does not apply these offsets; they remain calibration provenance.

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
transferable transform for Isaac's `fr3_hand` and Franka Hand. The selected
episode's calibrated intrinsics above replace the earlier
manufacturer-reference FOV values. Camera geometry remains explicit for later
comparisons.

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

## SIM-P4 one-request policy shadow inference

`sim/scripts/policy_shadow.py` captures one settled HOME observation, sends one
seeded request to the existing `pi05_droid` server, validates the returned
action chunk, and archives the request, audit, actions, timing, and provenance
under a unique `outputs/isaac_droid_shadow/<run-id>/` directory. It never
applies policy actions. The simulator adapter in
`sim/src/isaac_fr3/droid_observation.py` reads measured articulation positions
in `fr3_joint1` through `fr3_joint7` order and maps measured finger width to
`clip(1 - width / 0.08, 0, 1)`. It passes raw 180 × 320 RGB `uint8` images to
the shared `prepare_droid_observation`; OpenPI performs the 224 × 224 resize
once on the server.

Start the existing server first with `make droid-policy-server` in a separate
terminal. Wait for checkpoint restoration and the port 8000 listening log.
Isaac's bundled Python may lack `msgpack`, which the pinned OpenPI client
declares as a dependency. For that environment, install it into a temporary
target without changing Isaac's installation:

```bash
cd ~/isaacsim
./python.sh -m pip install --target /tmp/isaac-openpi-client-deps 'msgpack>=1.0.5'
PYTHONPATH=/tmp/isaac-openpi-client-deps ./python.sh -u \
  ~/MyProjects/saps-openpi-replication/sim/scripts/policy_shadow.py \
  --config ~/MyProjects/saps-openpi-replication/sim/configs/fr3_droid_scene.json \
  --host 127.0.0.1 --port 8000 --prompt 'Pick up the red object' --headless
```

The script defaults to seed `20260827` and replan index `0`; both are CLI
arguments. It requires the seeded server's `pi05_droid` checkpoint and pinned
OpenPI commit identity. The server model-input audit checks the received raw
arrays and prompt, then archives the transformed images and sampler input.
Stop this task's server afterward with `make policy-stop` unless it was already
running for another purpose.

## SIM-P5 one-chunk execution

Start `make droid-policy-server` and wait for checkpoint restoration before
starting Isaac. With the pinned OpenPI client dependency available to Isaac
Python as described above, run:

```bash
cd ~/isaacsim
PYTHONPATH=/tmp/isaac-openpi-client-deps ./python.sh -u \
  ~/MyProjects/saps-openpi-replication/sim/scripts/policy_one_chunk.py \
  --config ~/MyProjects/saps-openpi-replication/sim/configs/fr3_droid_scene.json \
  --host 127.0.0.1 --port 8000 --prompt 'Pick up the red object' \
  --policy-episode-seed 20260827 --replan-index 0 --headless
```

This command captures one settled HOME observation, holds the simulator during
one seeded inference, and requires at least eight returned actions. It executes
only actions 0–7. Each arm target is `fresh measured q + 0.2 × clip(action[:7],
-1, 1)` and must satisfy the FR3 articulation limits before the native
position drive receives it. The physical baseline's gripper threshold is
strictly `action[7] > 0.5` for CLOSED. A change of binary intent starts one
rate-limited gripper transition; repeated intent continues the same transition.
Closing uses the SIM-P2 0.100 m/s total-width rate and 20 N active-finger
drive effort. Opening uses the same rate toward 0.08 m width. Policy targets
are issued every four 1/60 s physics steps, with no convergence wait. After
the eighth action period the script commands an explicit fresh-measured-q hold
and advances eight more physics steps for evidence only.

Each run creates `outputs/isaac_droid_one_chunk/<run-id>/` with the raw images,
observation and provenance, native and selected action arrays, per-action
execution evidence, and timing. A repeated run with the same seed can still
receive different actions if Isaac's rendered RGB pixels differ; compare the
archived observation hashes before attributing a difference to policy sampling.
This gate records motion and safety checks, not task success. Stop the server
afterward with `make policy-stop`.

## SIM-P6 repeating closed-loop rollout

Start `make droid-policy-server` and wait for checkpoint restoration before
launching Isaac. Using the pinned OpenPI client dependency described in SIM-P4:

```bash
cd ~/isaacsim
PYTHONPATH=/tmp/isaac-openpi-client-deps ./python.sh -u \
  ~/MyProjects/saps-openpi-replication/sim/scripts/policy_rollout.py \
  --config ~/MyProjects/saps-openpi-replication/sim/configs/fr3_droid_scene.json \
  --host 127.0.0.1 --port 8000 --prompt 'Pick up the red object' \
  --policy-episode-seed 20260827 --headless
```

The default has no replan cap. Each cycle commands a fresh measured-q hold,
renders the held state without stepping physics, captures new RGB images and
measured joints, makes exactly one seeded request, then executes the first eight
actions at four 60 Hz physics steps per action. Each completed chunk advances
exactly 32 physics steps (8/15 s). The gripper's binary intent and rate-limited
transition persist across chunks. After action 7, a fresh measured-q terminal
hold is verified without stepping physics; the next observation is captured
from that held state. Physics does not advance during inference. Omit
`--headless` to watch the scene. An explicit
`--max-replans N` is available for diagnostics; it is not the default.

Stop a normal rollout with Ctrl-C. The runtime attempts a fresh measured-q
hold, preserves the gripper state, and writes the partial episode. Safety,
invalid response, or server/runtime errors also end the episode. Artifacts
under a unique `outputs/isaac_droid_rollout/<run-id>/` include top-level
provenance, episode, timing, and termination JSON, plus raw images, observation,
response, native and selected actions, and execution evidence per replan.
There is no task-success detector in this baseline: the outcome is recorded as
`not_evaluated` even if objects move. Stop the server with `make policy-stop`
after the run.

## SIM-P7 separate FR3 + Robotiq embodiment

`sim/configs/fr3_droid_robotiq_scene.json` selects the derived
`sim/assets/fr3_robotiq_droid.usda` assembly. The original
`fr3_droid_scene.json` still selects the NVIDIA FR3 with Franka Hand. SIM-P7
established this scene with OpenPI off; SIM-P8 adds the policy rollout adapter
described below.

The assembly references the Isaac Sim 6.1 FR3 and an unchanged copy of
[Robotiq's 2F-85 Isaac asset](https://github.com/robotiq/isaacsim_assets/tree/ef313b8416096d50c3ab56d67adb21f92d086660/grippers/Robotiq_2F_85)
at `ef313b8416096d50c3ab56d67adb21f92d086660`. It uses the PhysX
parallel-grip variant and standard fingertips. Source pin, resolved LFS files,
and licenses are recorded in
`sim/assets/robotiq_2f85_upstream/SAPS_PROVENANCE.md`. The derived USD alone
disables the stock hand and Robotiq's standalone root joint, connects
`fr3_link8` to `base_link` with a fixed joint, and limits the drive to 10 N m.
The upstream 26 N m setting caused deep cube penetration in the initial
scripted trial; 10 N m gave shallow bilateral contacts on this scene.

The supplied `sim/assets/FR3_2F85_ZED_transforms/` package defines column-vector
transforms `p_A = T_A_from_B @ p_B`, in metres. The verified FR3 `fr3_link8`
frame is the flange interface; the source STEP flange frame needs the supplied
+107 mm local-Z datum correction. Robotiq's `base_link/visuals` has a 120-degree
rotation about `(1,1,1)` relative to `base_link`, so the authored mount is the
composition `T_flange_from_hand_cad @ inverse(T_usd_base_from_hand_cad)`:
180 degrees about flange Z and translation `(0, 0, 0.013758)` m. CAD and USD
base-mesh bounds agree to within about 0.05 mm; mesh volume
is about 0.56% below the STEP solid, so this is a geometric registration, not
proof of identical source meshes. The source grip-frame TCP lies 0.133717 m
from the USD fixed base along its local Z. It is a nominal grasp reference,
not a measured centre of every grasped object.

The native USD wrist-camera prim is parented to `fr3_link8` and uses the
package's `T_flange_from_usd_camera_nominal` directly: position
`(-0.079491597033, 0.031930911279, 0.002650643753)` m and quaternion
`(w, x, y, z) = (0.123099062742, 0.696279294064, -0.696337096906,
-0.123111384047)`. This is the nominal ZED Mini LEFT optical pose; its
camera-body CAD transform is separate. The 82.19068145751953-degree wrist
HFOV, 101.5525131225586-degree external HFOV, SVO-derived LEFT intrinsics,
320 × 180 RGB captures, and OpenPI 224 × 224 resize-with-pad stay as in the
baseline. The external view remains DROID-informed and tabletop adapted.

The Robotiq articulation has seven arm joints, one driven `finger_joint`
(0 to 0.820305 rad), and five passive/mimic joints. Open and closed are driver
targets 0 and 0.820305 rad. The measured opening projects the two moving pad
origins onto the fixed base's lateral axis. An unconstrained full sweep of this
asset gives about 0.08708 m at open and 0 m at closed; a 0.4-rad driver pose
gives about 0.04735 m. Thus the normalized DROID observation is
`clip(1 - measured_width / 0.08708, 0, 1)`. Policy intent `> 0.5` commands
closed; `<= 0.5` commands open. Repeated intent leaves the current joint drive
active. This separate controller does not copy the Franka finger gains or
rate limiter. The arm's 15 Hz, 0.2-rad policy mapping is unchanged.

Run the independent validation with OpenPI stopped:

```bash
~/isaacsim/python.sh -u \
  sim/scripts/robotiq_validation.py \
  --config sim/configs/fr3_droid_robotiq_scene.json \
  --headless
```

Each run creates a unique directory under
`outputs/isaac_robotiq_validation/`, containing per-stage raw and OpenPI-sized
RGB images, transform and joint measurements, PhysX contact events, and pass
flags. The scripted HOME → pregrasp → grasp → close → lift → open sequence uses
Lula IK and the existing 40 mm, 50 g cube. The 2026-10-08 run
`20261008T073122Z_4cb0c4ac` passed: the measured flange datum was
0.106999953 m, both pads carried loaded contact during
close and lift, cube centre rose from 0.42003 to 0.48979 m, and opening
returned to 0.08708 m after release. The worst reported contact separation
was -0.000121 m. The maximum flange-to-base translation/rotation errors
were `3.92e-8 m` / `2.99e-8 rad`; flange-to-camera errors were `3.51e-9 m`
/ `2.10e-8 rad` through the sampled poses. The grasp/lift wrist images show
the red target between the fingers; the HOME wrist image is dominated by the
gripper and its shadow, so the target is not clearly visible there. The
external images show the arm and tabletop. A selected
DROID episode frame also has gripper fingers at the lower edge and a central
work surface, but scene content, lighting, and material appearance differ;
this qualitative comparison is not camera-pose ground truth.

**Validation status:** The SIM-P7 focused unit tests (17/17), standalone Robotiq grasp validation, and Franka-Hand mechanical regression passed. The repository-wide `make check` did not pass: two archived-sweep tests use `str.removeprefix`, which is unavailable in the Docker Python 3.8 runtime, and one presentation-scene fixture expects a different HOME joint value. These failures remain unresolved and were not addressed by SIM-P7.

### SIM-P11 soft-light inspection scene

`sim/configs/fr3_droid_robotiq_soft_light_scene.json` copies the Robotiq
baseline and changes only its lighting. It broadens Isaac's existing default
ground-plane SphereLight from 0.25 to 1.0 m radius, lowers its USD intensity
from 100000 to 6250, and reduces its specular contribution from 1.0 to 0.25.
This softens concentrated shadows and highlights while keeping the robot,
cube, table, cameras, and control settings identical. No global renderer
settings change. Inspect it with the existing scene launcher; no policy server
is needed:

```bash
~/isaacsim/python.sh -u sim/scripts/launch_scene.py \
  --config sim/configs/fr3_droid_robotiq_soft_light_scene.json
```

Run the rollout with the same command as before, changing only the config file while the policy server is running:

```bash
cd ~/isaacsim
PYTHONPATH=/tmp/isaac-openpi-client-deps ~/isaacsim/python.sh -u \
  sim/scripts/policy_rollout.py \
  --config sim/configs/fr3_droid_robotiq_soft_light_scene.json \
  --host 127.0.0.1 --port 8000 --prompt 'Pick up the object' \
  --policy-episode-seed 20260827
```

### SIM-P12 household-box pick-and-place scene

`sim/configs/fr3_droid_robotiq_pick_place_scene.json` keeps the SIM-P11 robot,
gripper, cameras, lighting, and policy settings. It places a small white
household storage box at the original target position (0.55, 0, 0.42 m). The
box has a 40 mm cuboid collision body and 50 g mass. A textured open-top basket
is centered at (0.70, -0.18 m) on the table. Its fixed collision floor and four
walls leave a 112 x 104 mm clear opening; the visual meshes have no collision.
The meshes come from the pinned local LIBERO assets and are documented in
`sim/assets/p12_household/README.md`.

Inspect the scene with the existing launcher from the repository root:

```bash
cd ~/MyProjects/saps-openpi-replication
~/isaacsim/python.sh -u sim/scripts/launch_scene.py \
  --config sim/configs/fr3_droid_robotiq_pick_place_scene.json
```

For a manual policy rollout, start `make droid-policy-server` in a separate
terminal, then use the exact task prompt:

```bash
cd ~/MyProjects/saps-openpi-replication
PYTHONPATH=/tmp/isaac-openpi-client-deps ~/isaacsim/python.sh -u \
  sim/scripts/policy_rollout.py \
  --config sim/configs/fr3_droid_robotiq_pick_place_scene.json \
  --host 127.0.0.1 --port 8000 \
  --prompt 'Pick up the small box and place it in the basket.' \
  --policy-episode-seed 20260827
```

### SIM-P12 Isaac catalog household-asset variant

`sim/configs/fr3_droid_robotiq_pick_place_catalog_scene.json` keeps the
earlier SIM-P12 scene for comparison and changes only the target package,
basket, and disabled optional mug. The active target is an upright 37.27 x
16.62 x 73.97 mm mac-and-cheese package at (0.55, 0, 0.436985) m with a
50 g cuboid rigid body. The blue basket is centered at (0.70, -0.18) m,
measures 138.5 x 176.0 x 74.4 mm, and has a fixed floor and four separate
walls. Its collision opening is 126.5 x 164.0 mm. The mug is disabled in this
condition. Asset sources and local dependencies are listed in
`sim/assets/p12_household/README.md`.

Inspect this scene from the repository root:

```bash
cd ~/MyProjects/saps-openpi-replication
~/isaacsim/python.sh -u sim/scripts/launch_scene.py \
  --config sim/configs/fr3_droid_robotiq_pick_place_catalog_scene.json
```

For a manual rollout, start `make droid-policy-server` in another terminal,
then use this scene's exact task instruction:

```bash
cd ~/MyProjects/saps-openpi-replication
PYTHONPATH=/tmp/isaac-openpi-client-deps ~/isaacsim/python.sh -u \
  sim/scripts/policy_rollout.py \
  --config sim/configs/fr3_droid_robotiq_pick_place_catalog_scene.json \
  --host 127.0.0.1 --port 8000 \
  --prompt 'Pick up the mac and cheese box and place it in the blue basket.' \
  --policy-episode-seed 20260827
```

`sim/configs/fr3_droid_robotiq_mug_target_scene.json` selects the yellow mug
instead: `target_object.enabled` is false and `optional_mug.enabled` is true.
The mug is the only tracked dynamic target, at (0.55, 0, 0.426792) m. A
22.23 mm radius, 53.58 mm tall cylinder collides with its cup body; the
visible handle has no separate collider. The basket and all robot, camera,
lighting, and policy settings are unchanged. Both flags cannot have the same
value. Existing rollout `objects[0]`, `initial_objects[0]`, and
`handles.target` refer to the selected mug. Legacy `cube_*` fields in
one-chunk and episode records retain their names but measure this target.

Inspect the mug scene with:

```bash
cd ~/MyProjects/saps-openpi-replication
~/isaacsim/python.sh -u sim/scripts/launch_scene.py \
  --config sim/configs/fr3_droid_robotiq_mug_target_scene.json
```

For a manual mug-target rollout, start `make droid-policy-server` separately:

```bash
cd ~/MyProjects/saps-openpi-replication
PYTHONPATH=/tmp/isaac-openpi-client-deps ~/isaacsim/python.sh -u \
  sim/scripts/policy_rollout.py \
  --config sim/configs/fr3_droid_robotiq_mug_target_scene.json \
  --host 127.0.0.1 --port 8000 \
  --prompt 'Pick up the yellow mug and place it in the blue basket.' \
  --policy-episode-seed 20260827
```

## SIM-P8 Robotiq policy rollout

Start `make droid-policy-server` and wait for the pinned `pi05_droid` server to
listen. Use the Isaac client dependency setup in SIM-P4, then run the same
one-chunk or repeated scripts with
`--config sim/configs/fr3_droid_robotiq_scene.json`. For example:

```bash
cd ~/isaacsim
PYTHONPATH=/tmp/isaac-openpi-client-deps ./python.sh -u \
  ~/MyProjects/saps-openpi-replication/sim/scripts/policy_rollout.py \
  --config ~/MyProjects/saps-openpi-replication/sim/configs/fr3_droid_robotiq_scene.json \
  --host 127.0.0.1 --port 8000 --prompt 'Pick up the red object' \
  --policy-episode-seed 20260827 --headless
```

The default remains unlimited; Ctrl-C finalizes the episode. The one-chunk
script is `policy_one_chunk.py` with the same configuration and a
`--replan-index`. Robotiq output goes to unique
`outputs/isaac_droid_robotiq_one_chunk/` and
`outputs/isaac_droid_robotiq_rollout/` paths, separate from Franka-Hand runs.
The policy request still contains exactly seven measured FR3 arm joints, one
normalized pad-opening scalar, two RGB images, and the prompt. The gripper
controller commands only `finger_joint` on a change of binary intent and lets
the joint drive continue across actions and replans. Its 10 N m authored
torque is not equivalent to the Franka Hand's 20 N active-finger effort.
The five passive Robotiq joints receive no position commands.

The arm mapping, first-eight selection, 15 Hz timing, four physics steps per
action, terminal hold, and zero simulation stepping during inference are shared
with SIM-P6. Each completed repeated chunk advances exactly 32 steps, or
0.533333 s. Per-action and episode evidence contains driver radians, measured
pad width, normalized observation, intent/transition state, faults, cube pose,
and loaded pad/cube contact counts. `bilateral_loaded_reports` requires both
pads to carry load in the same PhysX contact report; left and right contacts
at different times are not a grasp. A one-chunk run retains SIM-P5's separate
eight-step terminal evidence settle. Task outcome remains `not_evaluated`;
commanded closure alone is never labeled successful grasp.

In the 2026-10-08 pilot
`outputs/isaac_droid_robotiq_rollout/20261008T081653Z_78a9f018`, 13 complete
replans executed 104 actions at exactly 0.533333 s per chunk. The fourteenth
replan stopped after six actions because its next measured-q arm target would
exceed a joint limit. One pad contacted the cube, but there was no bilateral
loaded contact. The cube moved about 0.076 m laterally and its final height
did not increase; this is neither a grasp nor a lift result. The earlier long
pilot demonstrated continuous execution but had a cumulative side-contact
flag that could be mistaken for simultaneous bilateral contact. The current
monitor records bilateral contact only within one PhysX report. These pilots
are diagnostic evidence, not task-success measurements.

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
