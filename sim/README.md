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

Planned next steps are:
1. external and wrist RGB cameras;
2. exact DROID/OpenPI observation construction;
3. OpenPI policy-server connection;
4. 15 Hz DROID policy execution;
5. closed-loop pi0.5 simulation rollouts.

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
│   └── gripper_contact_test.py
└── src/
    └── isaac_fr3/
        ├── __init__.py
        └── scene.py
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
  sim/scripts/launch_scene.py \
  sim/scripts/gripper_contact_test.py

git diff --check
```

Physics/contact changes should additionally be validated with Isaac Sim's
bundled Python.