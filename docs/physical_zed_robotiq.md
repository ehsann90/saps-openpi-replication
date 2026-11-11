# Physical FR3 deployment

This is the operating guide for the FR3, Robotiq 2F-85, ZED Mini wrist camera
and ZED 2i external camera. SAPS owns observations, policy execution, action
mapping, readiness, provenance and experiment logging. The independent
`~/franka_ehsan_ros2_ws/src/franka_droid_stack` repository owns hardware:

| ROS2 package | Responsibility |
|---|---|
| `franka_droid_stack` | Unified hardware launch and joint-target forwarder |
| `franka_robotiq` | Exclusive Modbus serial owner and gripper services |
| `franka_zed_cameras` | Camera configuration, launch and validation |

SAPS starts no hardware drivers. Follow those package READMEs for installation
and the site's independent robot-bringup/controller workflow. Never start a
second camera, serial owner, forwarder or robot driver alongside an existing one.
Physical operator blending and automatic manipulation-success detection are
not implemented. The [first successful manipulation record](physical_rollout_20261111.md)
separates observed task success from its subsequent runtime abort.

## Environment and configurations

Use ROS2 Jazzy, the commissioned workspace `install/` overlay and SAPS
`.venv-physical` (ROS-compatible Python 3.12). The installed camera stack was
ZED SDK 5.5.1 / wrapper 5.5.0, HD720/30. Preserve the Docker-pinned simulation
runtime; it is independent of this physical environment. Hardware dependencies
and driver installation belong to the ROS2 packages.

```bash
cd ~/saps-openpi-replication
source tools/activate_physical_env.sh
export PHYSICAL_CONFIG="$PWD/configs/local/c4_zed_robotiq.json"
```

`FRANKA_DROID_INSTALL` selects an explicit overlay; its default is the current
workspace `install/`. `FRANKA_LAB_STACK_DIR` overrides the outer hardware Git
root. An explicit `PHYSICAL_CONFIG` is preserved by activation; its default is
the local C4 file. Local profiles are ignored and never overwritten by startup.

| Configuration | Responsibility |
|---|---|
| `configs/local/c4_zed_robotiq.json` | Authoritative machine-local commissioned C4 execution profile; calibration/device identity must be actual measured settings. |
| `configs/physical_pi05_fr3_c4.json` | Versioned C4 template and read-only CLI default; incomplete calibration/serial-port placeholders intentionally prevent readiness. |
| `configs/physical_pi05_fr3_zed_robotiq.json` | Distinct older commissioning-profile contract, used by profile regression tests; lacks C4's reliable-camera interface/stage. Not interchangeable with C4. |
| `configs/zed_robotiq_lab_sources.json` | Immutable reviewed historical hardware-source manifest used by strict qualification; not a current hardware configuration. |

To prepare a **new** C4 file from an existing complete hardware JSON:

```bash
"$PHYSICAL_PYTHON" tools/prepare_c4_config.py \
  --hardware-config /absolute/path/to/operator_hardware.json \
  --output configs/local/c4_NEW_PROFILE.json
```

This preserves supplied gripper and mounted-tool evidence, uses the C4 camera
contract, and rejects an existing output file. It does not invent calibration
or apply robot settings. Review the new profile before use. The exact November
11 configuration is immutable inside that run's `provenance.json`; the current
local file still matched its SHA-256 at cleanup. No configuration was changed.

Development mode records dirty SAPS/hardware trees, actual commits, source/diff
hashes and manifest differences. Strict mode additionally enforces clean trees,
reviewed source hashes, ancestry and the clean OpenPI pin. The current hardware
tree does **not** match the historical strict manifest. Do not regenerate it or
bypass the strict check merely to pass startup. Development execution requires
explicit `--execute` and positive finite replan/chunk limits. Neither mode
certifies physical safety. Requested config, effective device readback,
observation readiness, inference and experiment qualification remain distinct.

## Hardware startup and non-actuating checks

Start the commissioned Franka bringup and impedance controller separately,
retaining its existing gains, tool/load settings and safety supervision. The
hardware launch below does not load/switch that controller:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ehsan_ros2_ws/install/setup.bash
ros2 launch franka_droid_stack hardware.launch.py \
  gripper_config:="$HOME/franka_ehsan_ros2_ws/src/franka_droid_stack/franka_robotiq/config/robotiq.json"
```

It starts cameras, the read-only gripper server and the joint-target forwarder.
Use `start_cameras:=false`, `start_gripper:=false` or `start_forwarder:=false`
when retaining existing owners. Separately supervised gripper command service
availability requires `authorize_gripper:=true`; startup does not activate/reset
or move the gripper. Consult the hardware README for activation/recovery.

In SAPS, start the actual DROID inference server in a separate terminal:

```bash
make droid-policy-server
```

It serves `pi05_droid`, checkpoint `gs://openpi-assets/checkpoints/pi05_droid`,
OpenPI pin `15a9616a00943ada6c20a0f158e3adb39df2ccac`. Use unique IDs for
subscriber-only checks against the existing hardware owners:

```bash
make physical-c4-observations C4_CONFIG="$PHYSICAL_CONFIG" \
  PHYSICAL_RUN_ID=observations_UNIQUE_ID PHYSICAL_PROMPT='Put the marker in the white cup'
make physical-c4-readiness C4_CONFIG="$PHYSICAL_CONFIG" \
  PHYSICAL_RUN_ID=readiness_UNIQUE_ID PHYSICAL_PROMPT='Put the marker in the white cup'
make physical-c4-shadow C4_CONFIG="$PHYSICAL_CONFIG" \
  PHYSICAL_RUN_ID=shadow_UNIQUE_ID PHYSICAL_PROMPT='Put the marker in the white cup'
```

Inspect readiness and source/config provenance, not just exit status. Readiness
handshake is not inference proof; shadow performs discarded warm-up then fresh
seeded inference. Hardware findings remain unverified unless independently
established. A successful inference or graph query is not permission to move.

## Observation and action contract

| Source | Required current interface |
|---|---|
| Arm | `/franka/joint_states`, canonical `fr3_joint1..7`, radians |
| Wrist | ZED Mini `zedm`, serial 15330503, `/wrist_camera/zed_node/rgb/color/rect/image` |
| External | ZED 2i `zed2i`, serial 32919961, `/external_camera/zed_node/rgb/color/rect/image` |
| Gripper | `/robotiq/status`, JSON measured motor state from one exclusive server; OPEN/CLOSE/STOP via asynchronous Trigger services |

Camera messages are native 1280×720 BGRA8. Decoding respects padded `step`,
drops alpha and converts to RGB. Existing centre crop to 16:9 and INTER_AREA
downsize (INTER_LINEAR upsize) yields `[180,320,3]` RGB uint8 HWC. Preserve
orientation: no flip, mirror, rotation, camera swap or acquisition change.
Policy input keys are `observation/exterior_image_1_left`,
`observation/wrist_image_left`, `observation/joint_position` (`[7]` float32),
`observation/gripper_position` (`[1]` float32 measured closure), and `prompt`.
OpenPI performs model resize/normalization; the websocket does not reinterpret
channels. Observation dimensions do not prove training-view compatibility.

C4 uses latest **advancing** camera frames, RELIABLE KEEP_LAST depth 2 for
cameras, and sensor-compatible state QoS. Preserve the 0.5 s maximum source age
and 0.25 s cross-source skew. Post-warm-up and pre-inference barriers require
advancing source stamps; delayed pre-barrier messages cannot qualify. C3's
camera-pairing thresholds are not substituted into this policy path.

The response is finite `[15,8]`; only rows 0–7 execute. For each 15 Hz tick:

```text
u = clip(action[:7], -1, 1)
delta_q_rad = 0.2 * u
q_target = fresh_measured_q + delta_q_rad
action[7] > 0.5 -> CLOSE; otherwise OPEN
```

No extra `dt` factor, accumulated previous target, IK or fingertip TCP enters
this arm mapping. JointState targets pass SAPS → forwarder → streaming impedance
controller → effort interfaces. Native future predictions 8–14 remain logged.
Each full chunk ends with a fresh measured-q hold, application confirmation and
health audit before replanning. Holds keep effort control active.

Robotiq speed/force values are registers 0–255, not m/s or Newtons; historical
`--gripper-speed` applies to Franka Hand only. Verified calibration computes
`closure=clip((actual-open)/(closed-open),0,1)` and
`estimated_width=measured_max_width*(1-closure)`. This is a motor-position
estimate, not a direct contact-width sensor. Missing calibration blocks
observations. Keep measured endpoints/evidence, register targets, speed and
force unchanged. A write acknowledgement is not completed motion: requested
position, gGTO and gOBJ are checked; gOBJ 1/2 indicates contact, 3 target reached.
Contact, command completion and task success are distinct. Duplicate intents
are suppressed; reversal waits for STOP acknowledgement and prior command
completion. Faults/stale feedback latch errors. There is no automatic OPEN,
reset, activation or emergency auto-release on error.

## Safety prerequisites and bounded execution

| Layer | Existing boundary and important limitation |
|---|---|
| SAPS | Finite canonical seven-joint targets, deployed URDF position limits, 0.1 rad margin, source/receive age at most 0.1 s, action-age limit 1 s from execution-window origin, readiness and Franka fault/contact checks. No prospective MoveIt collision query or numerical plant speed/acceleration bound. |
| Scheduling | Eight actions at independently rounded `k/15` deadlines; no burst catch-up. A full missed 66,666,667 ns period aborts scheduled actions/terminal hold even when the observation timeout is longer. |
| Singularity | Direct unweighted flange Jacobian, `fr3_link0` → `fr3_link8`; measured and target condition checks. Make C1C2 default is 17; direct Python constant/default is 30 (CLI help still says 17); historical hard threshold is 80. Always record the explicit chosen limit. Cleanup changes none of these. |
| Forwarder/controller | Names, finiteness, timestamps and active-controller/activation identity; controller target age 0.25 s/future tolerance 0.05 s in the inspected configuration. Latest accepted equilibrium remains held on publisher loss. Graph wiring alone is not delivery evidence. |
| Effort path | Inspected libfranka 0.15.0 torque slew: 999.999 Nm/s, approximately 0.999999 Nm per nominal 1 ms write. This is neither an absolute torque clamp nor a velocity/acceleration limiter. Position/velocity-branch limiters do not establish protection on this effort route. |

The deployed URDF lists velocity `[2.62,2.62,2.62,2.62,5.26,4.18,5.26] rad/s`
and effort `[87,87,87,87,12,12,12] Nm`; metadata is not enforcement. The captured
controller-manager empty parameter map did not establish enabled effort-limit
coverage. Do not infer streamed-path protection from separate MoveIt/Servo.
The retained hybrid law is
`tau = Kq*e - Dq*dq + J_EE.T*(Kx*(J_EE*e) - Dxd*(J_EE*dq)) + coriolis`,
`e=q_target-q`. November 9 captured 1000 Hz configured update rate, hybrid true,
`Kq=[40,30,50,25,35,25,10]`, `Dq=[4,6,5,5,3,2,1]`,
`Kx=[750,750,750,15,15,15]`, `Dxd=[37,37,37,2,2,2]`. These are historical
readbacks, not authorization to apply gains or proof of today's effective state.

Before supervised motion, establish the actual assembly/finger/cable swept
volume, all-link self/table/fixture clearances, allowed region and independent
protection for this effort route. Include tracking overshoot, sensing/reaction
latency, torque-slew-limited braking, terminal-hold settling and held-last-target
failure. Verify native protection and accessible E-stop/abort arrangements.
A small chunk or feasible endpoint does not establish a safe swept volume.

The controller uses the configured SDK end-effector Jacobian and Coriolis model.
A fingertip TCP is unnecessary for joint arithmetic, but controller-frame,
mounted-tool and load compatibility remain material. Preserve numeric `F_T_EE`,
`EE_T_K`, controller gains and calibration. The [tool/load reference](physical_tool_load_configuration.md)
retains exact transforms, units, inertia combination, missing properties and
supervised readback procedures. Missing tool data is reported as unverified;
it is not an unconditional joint-space software veto or a safety certificate.

Only after independent safety review and operator authorization, the existing
one-chunk command is:

```bash
"$PHYSICAL_PYTHON" scripts/physical_policy_execution.py --execute --enable-gripper \
  --provenance-mode development --config "$PHYSICAL_CONFIG" \
  --prompt 'Put the marker in the white cup' --host 127.0.0.1 --port 8000 \
  --max-replans 1 --max-executed-policy-chunks 1 --singularity-limit 17 \
  --observation-timeout 30 --policy-timeout 120 \
  --application-confirmation-timeout 2 \
  --lab-stack-dir "$HOME/franka_ehsan_ros2_ws/src/franka_droid_stack" \
  --igd-control-dir "$HOME/franka_ehsan_ros2_ws/src/igd_fr3_control" \
  --output-dir outputs/physical_c4/execution_UNIQUE_ID
```

This is a documented operator command, not an automated validation step. Review
the entire output before increasing budgets. Default outcome provider returns
CONTINUE, with no deployed success detector or policy completion signal. Bounds,
diagnostic stop, intervention or runtime/safety errors terminate execution;
software status `success` means runtime validation, not manipulation success.
No timeout is mapped to task success. Do not automatically resume after faults.

Stop policy target production before hardware shutdown. Follow the commissioned
robot stop/deactivation procedure; a held equilibrium is not a disabled arm.
Do not change SDK load/frames during torque control. Shut down only owned
processes; stop the policy server with `make policy-stop` if this session started
it. Preserve raw artifacts and use new IDs for retries.

## Offline video export and diagnostics

```bash
PYTHONPATH=src .venv-physical/bin/python tools/analysis/export_physical_video.py \
  outputs/physical_c4/c4_dev_long_20261111T083244Z \
  --output-dir outputs/physical_c4/derived_UNIQUE_ID/videos --fps 10 --side-by-side
```

Uses existing NumPy/OpenCV; no ROS, GPU, server or robot connection. A new output
directory outside the original run is required. Generates wrist/exterior MP4s,
optional wrist-left side-by-side, and a manifest; encoded frame counts are
verified by decoding. Saved `native_rgb.npz` is already RGB even for BGRA8
sources: hashes/shapes are checked before explicit RGB→BGR encoding.

These are sparse policy-request observations, not continuous 30 Hz footage.
Each camera uses its own source ROS stamps on a common fixed-fps grid starting
at the later first timestamp. Latest past frames are held without interpolation;
the endpoint rounds up to a grid tick with one frame's duration. Manifest hashes,
source selections, fps, held duplicates, skipped and unavailable frames make
resampling explicit. Invalid/missing/nonadvancing evidence fails; no unrelated
frames fill gaps. Labels show source time and held duration. Cameras are aligned
by clock, not asserted to be hardware synchronized. A single sample is one
frame. Higher-rate recording would require a separately reviewed bounded
asynchronous recorder; none is added to inference/control here.

| Retained utility | Scope |
|---|---|
| `scripts/physical_c4.py`, `c4_observation.py`, `c4_readiness.py` | Current non-actuating C4 observations, shadow and hardware/config readiness; also reused by execution assembly. |
| `tools/prepare_c4_config.py` | New-profile preparation preserving operator-owned calibration/tool evidence. |
| `tools/diagnostics/audit_c4_saved_chunk.py`, `c4_target_audit.py` | Offline saved-q joint-margin/singularity and independent Pinocchio flange-model check. Passing is not trajectory/safety approval; no fresh timestamps are synthesized. |
| `tools/analysis/audit_fr3_desk_profile.py` | Pinned CAD/frame provenance and complete-input rigid-body inertia calculations; rejects missing/nonphysical inputs, never uploads or activates a profile. See tool/load reference. |
| `tools/diagnostics/native_limits_probe/` | Relocated offline C++ libfranka/joint-limits probe; creates no robot objects. |

For the last probe, source ROS, then use a new build directory:
`cmake -S tools/diagnostics/native_limits_probe -B /tmp/native_limits_UNIQUE_ID`,
`cmake --build /tmp/native_limits_UNIQUE_ID`,
`/tmp/native_limits_UNIQUE_ID/c4_native_limits`. It tests available library
behavior, not enabled runtime protection. For Python regressions use
`make check`; tests require no hardware and bind the independent Robotiq package
read-only. Historical Hand/RealSense scientific baselines remain separate.
