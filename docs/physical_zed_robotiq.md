# FR3 + ZED Mini/ZED 2 + Robotiq 2F-85

Current package migration (2026-11-10): source the existing ROS workspace and
`~/franka_ehsan_ros2_ws/install-droid-current/setup.bash` before these workflows.
The independent repository is `~/franka_ehsan_ros2_ws/src/franka_droid_stack`;
pass that Git root as `--lab-stack-dir`. The ROS package/module names are now
`franka_droid_stack` / `franka_droid_stack_runtime`. Robotiq hardware commands
use `python -m franka_robotiq.robotiq_node`; the Make target names and existing
C4 local JSON are retained. The policy-specific adapter remains in SAPS.
The controller plugin and message types keep their original identifiers.
Historical pins/evidence below remain historical; strict source qualification
is not refreshed by this migration. See the independent repository's README
and `franka_robotiq/README.md` for build, interfaces and supervised commands.


For the commissioned **ZED 2i / wrapper 5.5.0 C4 workflow**, use
[physical_c4.md](physical_c4.md). The earlier profile and procedures below remain
available as implementation/commissioning history. C4 does not lift the geometry gate.

Implementation branch: `implement/physical-zed-robotiq`, based on
`experiment/physical-fr3-saps` at
`ee29c2c7d931a44ea1878c44c27228ac2f752c1d`. The lab branch is `implement/zed-robotiq`, based
on main at `4bb6cdc58839dcdd94acbe1633b8f361676a4eb6`.
Historical RealSense/Franka configuration, G1B pin, results and validation
artifacts are unchanged. Codex has not independently validated any H1–H7
hardware stage. Researcher-reported gripper tests and follow-up offline checks
are recorded below. Codex has not started hardware nodes, contacted devices,
started a policy server, performed inference or issued robot/gripper commands.

## Implemented boundaries

- Lab `launch/dual_zed.launch.py` and `fr3_lab_stack_runtime/zed_cameras.py`
  compose official Stereolabs components in independent camera containers.
  Exact installed wrapper version, required YAML API, explicit serials and
  models are checked before process creation. Parameters and config-file hashes
  are logged. RGB startup reception is bounded to 30 seconds. See the lab
  `docs/zed_cameras.md` for dependencies and direct launch commands.
- SAPS `camera_launch.py` selects the camera driver from the new profile;
  historical RealSense behavior remains available. Publisher uniqueness,
  node binding and read-only serial-parameter checks remain mandatory.
  A requested-serial parameter plus image publication is the software identity
  check; H1 must confirm actual physical camera-to-role assignment. No serial
  is inferred from an Image message.
- `live_observation.py` decodes RGB8, BGR8, RGBA8 and BGRA8, handles padded rows,
  discards alpha, converts to RGB and preserves the existing deterministic
  centre crop to 16:9 and OpenCV resize to `[180,320,3]` uint8. There is no
  flip, rotation, mirror or camera swap. Source stamp, frame ID, encoding,
  decoded native shape and preprocessing are retained in evidence. Native
  profile is checked at reception and before inference; freshness/skew gates
  retain their configured 0.5s source-age and 0.25s cross-source bounds.
- `robotiq.py` implements only the required Modbus RTU register subset, using
  pyserial: function 03 at 0x07D0, function 16 at 0x03E8, CRC/device/function/
  acknowledgement checks and bounded serial timeouts. No suitable local
  Robotiq driver was found. There are no automatic serial retries.
- `robotiq_node.py` owns serial I/O outside the policy process. Status-only
  mode sends read requests only, including at shutdown. Serving status does
  not activate or move the device. OPEN/CLOSE/STOP services exist only with
  explicit `--authorize-gripper`. Explicit activation is a separate command
  and may physically move the fingers during device calibration.
- `/robotiq/status` is a `std_msgs/String` JSON record, stamped after a successful
  status read. It includes raw measured position, requested position, fault,
  activation/object status, adapter identity and configured speed/force/targets.
  Read errors are latched; the collector rejects faulted, stale, missing,
  uncalibrated or mismatched feedback. It no longer requires Franka finger
  JointStates for this profile. The gripper protocol does not supply a unique
  gripper serial/model in this register subset: inspect wiring/model physically.
- `robotiq_hand.py` uses asynchronous ROS service futures, duplicates suppression,
  bounded transitions, error latching and stop-confirmed reversal. A replacement
  waits for both the stop response and the old command result. Readiness requires
  fresh measured feedback. Lifecycle logs include requested/issued times,
  write acceptance and completion times from the serial node, before/after
  feedback, cancellation and contact evidence. Serial operations cannot block
  the 15 Hz scheduler. A CLOSE is not labeled a successful grasp or task outcome.

The wire and register semantics follow the manufacturer's
[2F-85/140 manual, communication and control sections](https://assets.robotiq.com/website-assets/support_documents/document/2F-85_2F-140_Instruction_Manual_e-Series_PDF_20190206.pdf).
The ZED architecture follows the [official composition approach](https://docs.stereolabs.com/docs/integrations/ros-2).
Both still require release/device commissioning.

## Exact mappings

`action[7] > 0.5` → CLOSED → configured `close_command` (default 255).
`action[7] <= 0.5` → OPEN → configured `open_command` (default 0).
Non-finite actions are rejected. Targets are distinct from the policy threshold.
Speed and force are configured register values 0–255, **not m/s or Newtons**.
The historical `--gripper-speed` argument applies to Franka only; Robotiq uses
its JSON register settings. Do not mistake a write acknowledgement for motion
completion: echoed requested position, gGTO and gOBJ are checked. gOBJ 1/2 is
contact in opening/closing respectively; 3 means requested position reached
without contact. Contact is recorded separately from command success.

No direct physical width sensor is assumed. After measured two-point calibration:

```text
raw_closure = (actual_position - open_position) / (closed_position - open_position)
closure = clip(raw_closure, 0, 1)
estimated_width_m = measured_maximum_width_m * (1 - closure)
policy observation = 1 - estimated_width_m / measured_maximum_width_m = closure
```

This is a calibrated **motor-position width estimate**, not an accurate width
measurement for all contact/adaptive finger configurations. Raw register,
calibration evidence, clipping and estimate source are logged. The default
85 mm is a nominal upper bound; replace it with measured usable opening.
Missing calibration (`verified=false`, null endpoints/evidence) blocks observation
readiness. Do not mark calibration verified without measured evidence.

STOP clears rGTO with rACT retained; rATR emergency auto-release is never sent.
Errors never cause an automatic OPEN. Serial faults latch and inhibit commands;
there is a bounded stop attempt on command failure and stop on normal shutdown
if this process issued OPEN/CLOSE motion. Activation-only commands send no
extra STOP/reset write on failure or shutdown (see recovery below). A failed
stop is unconfirmed, not safe completion. Robot-native supervision and emergency stop remain independent.

## Geometry and arm execution blocker

The seven-joint DROID target mapping, measured-state anchor, action scale,
clipping, chunk semantics, 15 Hz schedule, warm-up, finite rollout limits and
terminal holds were not changed. `single_action.singularity_metric` evaluates
the flange (`fr3_link8`) Jacobian by shifting the old TCP Jacobian back to the
flange; that metric and pure joint limits remain arm-only calculations.
`fr3_forward_kinematics` TCP FK, TCP Jacobian and Cartesian transforms in
`fr3_kinematics` still describe the historical Franka Hand. They must not be
used as Robotiq geometry. Existing URDF hand collision geometry, tool payload,
mounting and workspace clearances are unverified for the replacement.

The new config sets `tcp_frame=null`. All config-accepting arm execution entry
points (single action, verifier, inference hold, policy episode and historical
G1B validator) reject this profile before creating arm interfaces. Changing a
JSON `execution_ready` flag cannot bypass the guard. H7 needs measured mounting/
TCP, reviewed URDF/collision/payload integration and review of affected gates;
that cannot be completed with the information supplied. H6 subscriber-only
shadow inference is available once camera identity and feedback calibration pass.
Legacy commands run without this config still describe historical hardware;
do not use them to circumvent this block on the installed replacement.

## Provenance

Historical G1B retains its exact clean lab commit check. The new profile uses
`hardware_provenance.py` plus `configs/zed_robotiq_lab_sources.json`: the complete
tracked lab file set and SHA256 contents must match the reviewed manifest, lab
HEAD must descend from the historical baseline, and lab and SAPS trees must be
clean and committed. This permits a new actual Git commit identity without
broadly accepting arbitrary descendants or dirty sources. Branch/commit,
manifest hash, file hashes and effective hardware config are recorded. The arm
geometry block still applies first. No commits were created by Codex. Review and
commit the intended changes in both repositories before attempting later
execution. Do not regenerate the manifest merely to silence a mismatch.

## Dependencies and configuration

The inspected host has Ubuntu 24.04.5/Jazzy but no ZED SDK at `/usr/local/zed`,
ZED wrapper in the ROS installation, or ZED/Robotiq driver sources in the lab
workspace. Installed SDK/wrapper compatibility is therefore unresolved.
Install an official Jazzy-compatible SDK/CUDA/wrapper pair using Stereolabs'
release compatibility instructions. New lab dependencies: `zed_wrapper`,
`rclcpp_components`, `python3-yaml`. New serial dependency: pyserial 3.5
(`requirements-robotiq.txt`). ROS interfaces use existing `rclpy`, `std_msgs`,
`std_srvs`; there is no custom interface build or policy dependency change.
Use the ROS-enabled physical Python environment documented for historical runs.
No installation was performed.

Create a machine-local copy (already ignored by Git):

```bash
cd /home/hvl-robotics2404/saps-openpi-replication
mkdir -p configs/local
cp configs/physical_pi05_fr3_zed_robotiq.json configs/local/zed_robotiq.json
export PHYSICAL_CONFIG="$PWD/configs/local/zed_robotiq.json"
export PYTHONPATH="$PWD/src:$PWD/third_party/openpi/packages/openpi-client/src"
export PHYSICAL_PYTHON="$PWD/.venv-physical/bin/python"
source /opt/ros/jazzy/setup.bash
source /home/hvl-robotics2404/franka_ros2_ws/install/setup.bash
# Dependency installation, only when chosen by researcher:
"$PHYSICAL_PYTHON" -m pip install -r requirements-robotiq.txt
```

Edit the local JSON using real values: each camera serial, reviewed installed
wrapper version, SDK/CUDA versions (manually recorded and cross-checked against
startup logs), actual resolution/rate/encoding, persistent RS-485 adapter
`/dev/serial/by-id/...` identity, slave address/baud/wiring, speed/force registers,
command timeouts, measured calibration endpoints/maximum opening and evidence.
Defaults are commissioning starting values, not validated device settings.
Two serial placeholders intentionally cannot pass camera identity checks.
Native `shape` describes decoded RGB, even when source encoding has alpha.
Keep camera orientation explicit and preprocessing unchanged.

## Ordered researcher commissioning guide — commands prepared, not executed

1. **H1 read-only camera discovery and identity** (after dependency installation):

   ```bash
   "$PHYSICAL_PYTHON" -c 'import pyzed.sl as sl; print(sl.Camera.get_sdk_version())'
   ros2 pkg prefix --share zed_wrapper
   # Read package.xml at the returned path; record wrapper version and SDK/CUDA compatibility.
   ZED_Explorer --all
   ```

   Bind the actual Mini serial to wrist and actual ZED 2 serial to exterior in
   the local JSON. Discovery must confirm model and physical mounting.

2. **H3 read-only gripper discovery/status**, before any motion commands:

   ```bash
   ls -l /dev/serial/by-id/
   # Inspect adapter identity/wiring, then set port/slave/baud in the local JSON.
   "$PHYSICAL_PYTHON" -m franka_robotiq.robotiq_node --config "$PHYSICAL_CONFIG" --mode status
   ```

   This does not activate the gripper. Confirm actual device model, slave,
   fault/activation state and raw feedback; an adapter name is not gripper identity.

3. **H2 camera streams/content/rates/timestamps**:

   ```bash
   make physical-cameras PHYSICAL_CONFIG="$PHYSICAL_CONFIG"
   # In another configured terminal:
   ros2 param get /wrist_camera/zed_node general.serial_number
   ros2 param get /external_camera/zed_node general.serial_number
   ros2 topic info -v /wrist_camera/zed_node/rgb/image_rect_color
   ros2 topic info -v /external_camera/zed_node/rgb/image_rect_color
   ros2 run franka_droid_stack check_rgb_rates.py --seconds 15 --topics \
     /wrist_camera/zed_node/rgb/image_rect_color \
     /external_camera/zed_node/rgb/image_rect_color
   ros2 topic echo /wrist_camera/zed_node/rgb/image_rect_color --once --field header
   ros2 topic echo /external_camera/zed_node/rgb/image_rect_color --once --field header
   ros2 run rqt_image_view rqt_image_view
   ```

   Inspect each role's image, orientation, actual encoding and resolution.
   Optional rqt requires `ros-jazzy-rqt-image-view`. Do not compensate for
   incorrect mounting or roles using silent image transforms.

4. **H4 supervised gripper only**, stationary arm, clear fingers, one command
   at a time. Do not run the serial server concurrently with these commands:

   ```bash
   "$PHYSICAL_PYTHON" -m franka_robotiq.robotiq_node --config "$PHYSICAL_CONFIG" --mode activate --authorize-gripper
   "$PHYSICAL_PYTHON" -m franka_robotiq.robotiq_node --config "$PHYSICAL_CONFIG" --mode open --authorize-gripper
   "$PHYSICAL_PYTHON" -m franka_robotiq.robotiq_node --config "$PHYSICAL_CONFIG" --mode status
   "$PHYSICAL_PYTHON" -m franka_robotiq.robotiq_node --config "$PHYSICAL_CONFIG" --mode close --authorize-gripper
   "$PHYSICAL_PYTHON" -m franka_robotiq.robotiq_node --config "$PHYSICAL_CONFIG" --mode status
   # Explicit stop if needed (does not release automatically):
   "$PHYSICAL_PYTHON" -m franka_robotiq.robotiq_node --config "$PHYSICAL_CONFIG" --mode stop --authorize-gripper
   ```

   Activation itself may move fingers. Completion/contact is not proof of
   grasp success. Inspect timeout, fault and stop behavior before proceeding.

5. **H5 measured feedback calibration and action mapping**: measure usable
   opening at both endpoints, capture raw position and document the estimate's
   limitations. Populate calibration and evidence only after measurement:

   ```bash
   "$PHYSICAL_PYTHON" -m franka_robotiq.robotiq_node --config "$PHYSICAL_CONFIG" --mode observation
   "$PHYSICAL_PYTHON" -c 'from saps.physical.droid_gripper import droid_gripper_decision as d; print([d(x) for x in (0., .5, .50001, 1.)])'
   # Subscriber-only status server for H6; no activation or command services:
   make physical-robotiq-serve PHYSICAL_CONFIG="$PHYSICAL_CONFIG"
   # Another terminal:
   ros2 topic echo /robotiq/status --once
   ```

6. **H6 complete observation/shadow inference**, with cameras and read-only
   status server running and a separately approved existing policy server:

   ```bash
   make physical-pi05-shadow PHYSICAL_CONFIG="$PHYSICAL_CONFIG" \
     PHYSICAL_RUN_ID=zed_robotiq_h6_UNIQUE_ID PHYSICAL_REQUESTS=1 \
     PHYSICAL_PROMPT='YOUR_EXPLICIT_TASK'
   ```

   Inspect `outputs/physical_pi05_droid_p0/` (or the output root printed by
   Make), camera/native RGB evidence, crop, timestamp/skew rejection, actual
   gripper register/estimate, policy schema and shape `[15,8]`. Do not proceed
   to arm commands from a shadow inference result alone. For the existing
   warm-up shadow path use `make physical-c1c2-warmup` with the same explicit
   config/run/prompt; it remains subscriber-only.

7. **H7 bounded physical policy execution is blocked** until measured geometry
   is integrated and reviewed. After that follow-up, the existing finite command
   shape is shown below for later review. It is intentionally rejected today:

   ```bash
   # Separate gripper server terminal, only at an authorized later H7 session:
   "$PHYSICAL_PYTHON" -m franka_robotiq.robotiq_node --config "$PHYSICAL_CONFIG" --mode serve --authorize-gripper
   # After verified geometry integration, clean committed sources and prior gates:
   "$PHYSICAL_PYTHON" scripts/physical_policy_execution.py --execute --enable-gripper \
     --config "$PHYSICAL_CONFIG" --output-dir outputs/physical_c1c2/H7_UNIQUE_ID \
     --prompt 'YOUR_EXPLICIT_TASK' --max-replans 1 --max-executed-policy-chunks 1 \
     --application-confirmation-timeout RESEARCHER_SELECTED_POSITIVE_SECONDS
   ```

   Retain the established warm-up, native supervision, explicit authorization,
   finite bounds and terminal holds. Never bypass the geometry/provenance gates.

## Explicit activation recovery

Researcher-reported status is `gACT=1`, `gSTA=0`, initially `gFLT=9`;
status reads clear the communication fault but do not complete activation.
The existing `activate` mode deliberately refuses an already-set gACT unless
ready. Use the new **explicitly authorized**, standalone `reset-activate` mode
for a single reset-and-activation attempt. It is not an automatic fallback and
is not exposed through the policy or ROS command services.

```bash
# Prepared for the researcher; not executed by Codex. Stationary arm, clear fingers.
# Stop any serial-owning status server first.
"$PHYSICAL_PYTHON" -m franka_robotiq.robotiq_node \
  --config "$PHYSICAL_CONFIG" --mode reset-activate --authorize-gripper
```

This follows the manufacturer's [rACT reset/activation sequence and fault
semantics](https://assets.robotiq.com/website-assets/support_documents/document/2F-85_2F-140_General_PDF_20210623.pdf):

1. Read and record the initial status, including any existing fault.
2. Send `output(action=0)`; the existing transport verifies the FC16 ACK.
3. Poll for `gACT=0`, `gSTA=0`, `gGTO=0` before permitting activation.
4. Send `output(action=1)` and verify its ACK.
5. Poll until `gACT=1`, `gSTA=3`, `gFLT=0`.

Both phases share `gripper.command_timeout_seconds` (existing range 0–30 seconds,
exclusive of zero), rather than starting a fresh timeout per phase. An in-flight
serial transaction retains its own bounded read/write timeout. Priority notices
0x05/0x07 may persist while waiting; they never count as ready. Other nonzero
faults after the reset write, communication errors, failed acknowledgements and
deadlines terminate the attempt. There are no activation/write retries and no
OPEN/CLOSE or auto-release commands. Reset/activation itself can move fingers.

The CLI prints JSON containing `initial_state`, timestamped `transitions`,
`final_state` (last successfully read feedback), `success` and `error`. A failed
attempt exits with code 1 and retains its trace; a missing authorization exits
before opening the serial port. Reset failure cannot advance to `rACT=1`.

Activation uses rGTO=0, and clearing rGTO does not stop the activation routine.
The existing `stop()` also writes rACT=1: emitting it after a reset failure could
initiate activation again. Both activation-only modes therefore close the serial
port without that extra shutdown/error write. After a timeout or interruption,
activation may still be underway; do not infer physical stop or readiness from
process exit. Inspect read-only feedback and use independent physical supervision.
OPEN/CLOSE shutdown/error stopping and error latching remain unchanged.

Offline regression command (included in the follow-up suite below):

```bash
PYTHONPATH=src "$PHYSICAL_PYTHON" -m unittest discover -s tests/unit -p test_physical_robotiq_activation.py -v
```

The new fixture uses fake serial devices and a fake clock. It covers successful
recovery, reset ACK/feedback failures, activation timeout, activation/fault notices,
communication failure, one shared deadline, authorization and absence of extra
activation writes during CLI cleanup. Codex has not performed physical recovery; later researcher reports appear below.

## Shutdown and measured observation follow-up

The researcher reports successful activation, OPEN/CLOSE, object contact,
release and repeated-operation tests, with healthy continuous server polling.
Reported motor endpoints are approximately 3 open and 227–228 empty-close;
one object-contact reading was 102. These reports are not automatically entered
as verified calibration. Record verified endpoints, measured usable opening
and calibration evidence in the local configuration before conversion.
No calibration, speed, force, policy threshold or arm settings were changed.

`--mode serve` now handles intentional Ctrl+C and an externally stopped ROS
context without an unhandled interrupt or duplicate context shutdown. It still
cancels command waits, preserves the existing STOP behavior for issued motion,
drains the executor and closes the serial port. Context cleanup uses Jazzy's
idempotent `rclpy.try_shutdown()`. A server that has issued no motion never writes
output registers during shutdown, including a read-only server. Cleanup also
covers failure after ROS initialization but before node creation.

Standalone `--mode observation` now has a short read-only settling period:

```bash
# Researcher command only; stop the serial-owning server first.
"$PHYSICAL_PYTHON" -m franka_robotiq.robotiq_node --config "$PHYSICAL_CONFIG" \
  --mode observation --feedback-settle-seconds 1
```

The default is 1 second; the explicit bound must be positive and at most 5
seconds. Existing per-transaction serial timeouts still apply, so an in-flight
read can outlast the settling deadline by its bounded transaction duration.
Only successful status reads are repeated. Watchdog fault 0x09 and normal
activation notices may settle; other device faults fail immediately. Read
errors/timeouts fail immediately without transaction retries. Conversion
requires healthy ready feedback and verified calibration. No reset, activation,
fault-clearing write, OPEN, CLOSE or STOP is sent by observation mode.

Failure exits with code 1 and JSON containing `success=false`, `reason`, `error`,
`attempts` and `last_status`. The last successful status includes `activated`,
`activation_state`, `fault` and `ready`; it is null if no status was obtained.
A healthy device with missing/unverified calibration reports
`reason=invalid_calibration` and remains blocked. Connection/ownership errors
are also structured and never cause another connection attempt.

With the persistent server running, use SAPS's ROS subscription path, **not**
a standalone serial diagnostic. `RosPhysicalObservationCollector` creates only
subscriptions, reads measured `actual_position`, uses the existing verified
two-point mapping and records contact separately in feedback metadata. It now
also rejects invalid latest gripper feedback at direct assembly (even if an
older valid sample remains cached) and rejects stale receive timestamps.
Existing source-time freshness/skew checks remain mandatory. The serial owner
continues to request exclusive port access; no second Modbus owner was added.

Offline checks executed for this follow-up:

```bash
PYTHONPATH=src:tests/unit .venv-physical/bin/python -m unittest \
  test_physical_robotiq test_physical_robotiq_activation \
  test_physical_robotiq_observation test_physical_live_observation \
  test_physical_live_shadow test_gripper_ros test_droid_gripper \
  test_gripper_validation test_physical_zed_config -q
make compile
```

All 97 selected tests passed; compile checks passed. An initial regression run
exposed four Franka mock-interface errors and one outdated image-error regex.
The fixtures now explicitly expose the Franka interface and expect the existing
four supported image encodings; no production action/threshold behavior changed.
The new tests use fake devices, ROS modules and clocks to cover normal/interrupted/
already-stopped shutdown, serial cleanup and preserved STOP, read-only settling,
fault/communication failures, calibration refusal, measured conversion, stale
feedback and single ownership. Expected rejection messages in test output are
negative fixtures. Full `make check` and hardware tests were not run.

Remaining researcher checks: Ctrl+C on the real server, read-only settling after
an idle watchdog interval, calibration evidence/usable opening, and comparison
of standalone versus persistent measured observations. Keep arm execution blocked
by the existing geometry prerequisites; no hardware stage is approved by these
offline tests.

## Additional offline commands

```bash
PYTHONPATH=src "$PHYSICAL_PYTHON" -m unittest discover -s tests/unit -p test_physical_robotiq.py
PYTHONPATH=src "$PHYSICAL_PYTHON" -m unittest discover -s tests/unit -p test_physical_zed_config.py
make compile
make check
```

Observation evidence adds camera `source_frame_id` and gripper
`feedback_metadata` (null for historical Franka sources); canonical policy keys
and tensor schemas are unchanged. Existing result files are untouched.

Fixtures cover protocol CRC, status/contact interpretation, calibrated feedback,
missing calibration, padded alpha images, threshold boundary, duplicates,
stop-before-reversal, stale-feedback failure, persistent device identity and
geometry blocking. Lab version/identity guards have a separate offline pytest.
The selected tests above have now been executed offline. Physical stages have
not been executed by Codex; researcher reports are distinguished above.
