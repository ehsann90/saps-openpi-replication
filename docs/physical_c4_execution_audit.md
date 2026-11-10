# C4 single-chunk execution audit — 2026-11-06

**Decision: NO-GO for physical execution.** CAD is excluded at the researcher's
request. No measured tool transform, combined payload or collision-envelope
evidence has replaced it. Ignoring CAD does not approve the historical Franka
Hand geometry for the installed Robotiq. No execution guard was changed.

## Evidence inspected

- `outputs/physical_c4/observations_002/run.json`: five observation captures
  passed the non-actuating checks in the unified Jazzy workspace.
- `outputs/physical_c4/shadow_004/run.json`: one main request after discarded
  warm-up passed; metadata identifies `pi05_droid`, checkpoint
  `gs://openpi-assets/checkpoints/pi05_droid`, OpenPI
  `15a9616a00943ada6c20a0f158e3adb39df2ccac`. This is saved researcher evidence;
  no new live inference was performed during this audit.
- That server's saved Robotiq status was healthy, open at actual position 3,
  exclusively owned, and **read-only** (`commands_authorized=false`, no command
  services). This establishes observations at capture time, not current command
  readiness or verified physical fingertip width.
- The saved shadow preview explicitly says `joint_limits_evaluated=false`.
  The new offline audit fills that gap for its recorded observation snapshot.
- Both SAPS and lab working trees remain dirty. The lab source manifest still
  differs from the current C3 implementation. Neither manifest nor revision
  guards were repinned to make these changes pass.

## Model and saved-target validation

The unified `franka_description` xacro expanded successfully with the existing
bringup's `hand:=false` mapping (`load_gripper:=false`). `check_urdf` parsed it
successfully. Its chain ends at `fr3_link8`: it contains no Robotiq, bracket,
wrist camera/mount or tool TCP. Disabling Franka gripper bringup does not add
the replacement tool to the robot model or configure libfranka's load/tool frames.

A bounded read-only attempt to obtain `/robot_state_publisher`'s current
`robot_description` returned **Node not found** in the queried ROS graph.
Consequently the generated source model has not been verified against the
deployed model or robot state. No robot bringup/controller/server was started.

Using Pinocchio 3.9.0 and the generated arm model:

| Check | Saved evidence result |
|---|---|
| Prediction shape/type | `[15,8]`, finite float64 |
| Flange model vs runtime FK | Maximum matrix-element error `9.09e-16` |
| Flange model vs runtime Jacobian | Maximum element error `1.09e-15` |
| Positions compared | Recorded measured q and all 15 snapshot targets |
| First 8 targets: minimum joint-limit margin | `0.498576` rad; existing gate `0.1` rad |
| Measured flange-Jacobian condition number | `8.44149`; existing limit `30` |
| First 8 target condition numbers | Approximately `8.2448–8.2555` |
| Largest single-joint offset in first chunk | Approximately `0.060446` rad |
| q5 offsets in first chunk | Approximately `+0.007164` to `+0.016065` rad |
| Gripper predictions | All OPEN, including all eight executable-window rows |
| Physical commands issued by audit | Zero |

The [derived evidence](validation/2026-11-06_c4_saved_chunk/summary.json)
contains exact input hashes, source revision/dirty state and eight proposed
targets. The full local diagnostic is `/tmp/c4_saved_chunk_audit_20261106.json`.
All targets use **the same saved measured q** independently; no accumulation or
future measured trajectory is assumed. Their validity today cannot be inferred
from these old timestamps. A future policy response may also differ.

The new diagnostic exits nonzero on a first-window snapshot rejection or arm
model mismatch. It always records `safe_to_execute=false` and `NO-GO`, even on
exit 0. It does not evaluate live deadlines, future q, tool/workspace collision,
payload, dynamic limits, controller delivery or operator safety. No age values
are fabricated. A deliberately altered joint-origin model was correctly rejected.

Reproduce the offline checks (use new output filenames if these already exist):

```bash
cd ~/saps-openpi-replication
source tools/activate_physical_env.sh
xacro "$HOME/franka_ehsan_ros2_ws/src/franka_description/robots/fr3/fr3.urdf.xacro" \
  hand:=false ros2_control:=true robot_ip:=192.170.10.101 \
  > /tmp/c4_fr3_arm_only_repeat.urdf
check_urdf /tmp/c4_fr3_arm_only_repeat.urdf
"$PHYSICAL_PYTHON" tools/diagnostics/audit_c4_saved_chunk.py \
  --urdf /tmp/c4_fr3_arm_only_repeat.urdf \
  --request-dir outputs/physical_c4/shadow_004/shadow/request_0000 \
  --output /tmp/c4_saved_chunk_audit_repeat.json
```

After a measured tool model is integrated, run this same audit on that model
and a fresh shadow capture. It compares arm flange kinematics; it deliberately
does not certify the added tool model from a successful numerical comparison.

## Minimum work still needed before execution

| Missing evidence/check | Minimum required resolution |
|---|---|
| Flange/tool/TCP frames | Measure flange-to-adapter, adapter-to-tool and chosen TCP translation **and rotation**, axes, units and uncertainty. Record reference points and compare with actual SDK `F_T_EE` and `EE_T_K`. A CAD file is optional; measurements are mandatory. |
| Mounted load | Measure combined gripper, adapter, wrist-camera/mount and attached load mass, centre of mass and inertia with frame conventions; review robot-native load configuration. Include the intended object/payload range. Do not invent or copy Franka Hand values. |
| Collision/workspace | Measure a conservative envelope covering open/closed fingers, linkage sweep, bracket, camera and cables. Integrate URDF/SRDF geometry and verify table/workspace/self-collision clearance and the independent enforcement used during torque streaming. RViz geometry alone does not enforce collision avoidance. |
| Tool-dependent runtime assumptions | Review historical `fr3_hand_tcp` FK/displacement consumers and SDK end-effector frame used by the unchanged hybrid impedance controller. Bind the measured model to deployed frames; retain the arm-flange singularity gate. Only then implement a reviewed evidence-bound replacement for the current unconditional Robotiq rejection. |
| Opening calibration | Confirm usable inner fingertip opening and uncertainty. The saved evidence estimates 85.5 mm from outer width minus approximate thickness, while the config uses nominal 85 mm. Keep actual-register endpoint closure separate from physical width and contact/grasp claims. |
| Source integrity | Review existing changes, make explicitly approved clean commits, then review/update the exact lab manifest. No commits or repinning were done by this task. |
| Deployment readiness | Verify the unified overlay and actual published model, fresh driver/state, exclusive authorized gripper server, controller/forwarder health and application evidence, visual camera-role approval, fresh warmed shadow predictions, robot-native safety/reflex settings, E-stop and supervision. |

The existing geometry guard is unconditional for `zed_robotiq_2f85` in both the
ROS entry point and policy episode. Configuring `geometry.execution_ready=true`
cannot unblock it. With the measurements absent, there is **no verified runtime
geometry change to apply**. No fabricated URDF, load or collision parameters,
alternative controller or direct controller publisher have been introduced.

## Single supervised chunk command — conditional, blocked today

Use the [terminal-by-terminal C4 guide](physical_c4.md) for independent bringup,
the existing impedance controller, forwarder, cameras and one gripper owner.
The gripper server must be explicitly motion-authorized in its own terminal;
the saved read-only server configuration cannot execute commands. No automatic
activation, reset, OPEN, CLOSE or robot-error recovery is part of this audit.

**Only after** the measured geometry integration above is implemented and
reviewed, all existing integrity/readiness gates pass, and the researcher
authorizes motion at the emergency stop, use this existing execution command:

```bash
cd ~/saps-openpi-replication
source tools/activate_physical_env.sh
export PHYSICAL_CONFIG="$PWD/configs/local/c4_zed_robotiq.json"
export PHYSICAL_PROMPT='pick up the object and hold it'
"$PHYSICAL_PYTHON" scripts/physical_policy_execution.py --execute --enable-gripper \
  --config "$PHYSICAL_CONFIG" --prompt "$PHYSICAL_PROMPT" \
  --host 127.0.0.1 --port 8000 \
  --max-replans 1 --max-executed-policy-chunks 1 \
  --observation-timeout 30 --policy-timeout 120 \
  --application-confirmation-timeout 2 \
  --lab-stack-dir "$HOME/franka_ehsan_ros2_ws/src/fr3_lab_stack" \
  --igd-control-dir "$HOME/franka_ehsan_ros2_ws/src/igd_fr3_control" \
  --output-dir "outputs/physical_c1c2_gripper/c4_one_chunk_$(date -u +%Y%m%dT%H%M%SZ)"
```

The prompt is the saved shadow prompt; review it explicitly for the actual
supervised task rather than silently substituting another. The budget remains
one replan and at most **8 actions at 15 Hz**, preserving
`q_target=q_measured+0.2*clip(action[:7],-1,1)` and the `>0.5` CLOSE threshold.
Warm-up, pre-holds, inference, terminal hold and bounded confirmation waits add
wall time. Terminal hold does not disable torque control or make the arm inert.
The program may stop before completing all eight actions if any gate fails.
Do not replay the saved predictions or automatically continue after a fault.

## Changes and verification

Only the offline `c4_target_audit.py` module, diagnostic CLI, focused tests,
this report/guide link and the small derived evidence file were added/updated.
Runtime execution, geometry guards, thresholds, controller/forwarder/gripper
behavior and all existing work were preserved. No commits were made.

Checks executed:

```bash
PYTHONNOUSERSITE=1 PYTHONPATH=src:tests/unit .venv-physical/bin/python -m unittest \
  test_physical_c4_target_audit test_physical_c4 test_physical_policy_execution \
  test_single_action test_fr3_forward_kinematics -q
make compile
make check
git diff --check
```

The focused suite passed **77 tests**; compilation passed. Full `make check`
passed **453 tests** and compilation. Whitespace checks passed and nine
previously hashed historical C3 artifacts remain unchanged. The generated URDF
passed `check_urdf`, and the real saved chunk passed the offline audit.
The first focused invocation misspelled an existing module name; the corrected
command above passed. An initial ROS parameter query hit sandbox socket
permissions; the permitted read-only retry returned `Node not found`.
No physical motion or fresh hardware experiment was performed.
