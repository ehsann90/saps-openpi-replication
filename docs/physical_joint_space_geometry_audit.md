# Geometry dependencies of the autonomous joint-space rollout

Audit date: 2026-11-09. Branch: `implement/physical-zed-robotiq`.

**Historical dependency audit:** the later
[development workflow](physical_development_rollout.md) removes the unconditional
joint-space Robotiq profile veto described here. No fingertip TCP/IK is required;
actual runtime checks remain, and unresolved tool/load/clearance findings are
reported without inventing hardware values or granting physical qualification.

**A named/measured Robotiq fingertip TCP is not required to interpret the seven
arm actions or evaluate the flange singularity gate.** This corrects the broader
TCP language in earlier C4 reports. It does not establish collision safety or
validate the controller's configured end-effector frame and load.

**Physical execution remains NO-GO** for the unresolved physical-safety and
source-integrity requirements below. No motion, live hardware test or policy
service was started during this audit. Earlier researcher observation/shadow
success is useful evidence but does not settle those requirements.

## Exact action interpretation and execution chain

```mermaid
flowchart TD
  P[Pinned pi05-DROID server: finite 15 x 8 actions] --> S[Select native rows 0 through 7]
  S --> Q[Fresh measured q for each tick]
  Q --> T[q_target = q_measured + 0.2 clip action]
  T --> G[Joint bounds/margin, flange singularity, age/readiness checks]
  G --> B[StreamingBoundary: stamped JointState]
  B --> F[/fr3_streaming_joint_target]
  F --> W[Existing target forwarder]
  W --> C[Existing hybrid streaming impedance controller]
  C --> E[Seven effort commands through franka_hardware]
```

1. `scripts/physical_policy_execution.py` calls
   `live_inference_hold_ros.run_inference_hold(policy_execution=True)`, which
   assembles `StreamingBoundary`, the ROS observation collector and the existing
   single-owner gripper adapter, then calls `policy_execution.run_policy_episode`.
2. `infer_live_request` uses the bounded client and strictly checks checkpoint,
   protocol and native finite floating `[15,8]` actions. The frozen pretrained
   config is `pi05_droid`, checkpoint
   `gs://openpi-assets/checkpoints/pi05_droid`; model transformations/normalization
   occur on the server. `DroidOutputs` returns the first eight action dimensions.
3. `streaming_playback.selected_actions` validates the whole native chunk and
   selects exactly rows 0–7. `policy_execution.send` obtains newly advancing
   measured joint positions for each tick. `prepare` → `safety_gate` →
   `candidate_target` performs:

   ```text
   u[k]       = clip(action[k, 0:7], -1, 1)
   delta_q[k] = 0.2 * u[k]                  # radians
   q_target[k]= q_measured[k] + delta_q[k]  # absolute joint radians
   ```

   The arm action is a relative, normalized joint-velocity-style DROID control
   value, **not an SI rad/s command**. The adapter applies no extra `dt=1/15`
   factor, integrates no previous target, performs no IK, and publishes no joint
   velocity command. `15*delta_q` is only a rate-equivalent diagnostic, not a
   guaranteed physical velocity or a velocity safety limit. This is consistent
   with the inspected pinned [DROID conversion](https://github.com/droid-dataset/droid/blob/33ae6a67274f36d2e29525b86f23a56616ef43a7/droid/robot_ik/robot_ik_solver.py)
   and [joint update path](https://github.com/droid-dataset/droid/blob/33ae6a67274f36d2e29525b86f23a56616ef43a7/droid/franka/robot.py).
4. `StreamingBoundary.publish` creates `sensor_msgs/JointState` with canonical
   `fr3_joint1..7` names and seven absolute positions in radians; velocity/effort
   fields are unused. `header.frame_id` is the run identifier, not a TCP frame.
   The current ROS stamp, state age, readiness and deadline are checked again
   immediately before the one publication to `/fr3_streaming_joint_target`.
5. The unchanged forwarder checks names, count, finiteness, stamp and fresh
   controller-active evidence, then forwards the same message to
   `/fr3_streaming_joint_impedance_controller/target_joint`. It performs no
   Cartesian conversion or geometric collision test.
6. The controller installs the latest absolute joint equilibrium. In the existing
   `hybrid_impedance_enabled=true` configuration its effort law is:

   ```text
   tau = Kq*e - Dq*dq + J_EE.T*(Kx*(J_EE*e) - Dxd*(J_EE*dq)) + coriolis
   e   = q_target - q_measured
   ```

   It uses the configured SDK end-effector Jacobian, not the policy's flange
   singularity Jacobian. It does not solve for a Cartesian pose. No controller
   law, gains, interfaces or native safety settings were changed.

Absolute deadlines are independently rounded `k/15` seconds; actions occupy
ticks 0–7 and a fresh measured-q terminal hold is requested at tick 8. Late
intervals are rejected rather than replayed in a burst. Pre-holds, discarded
warm-up, all-source barriers, bounded inference, target-delivery/T4 confirmation,
post-hold health checks and abort behavior are retained. Terminal hold preserves
effort control; it does not make the arm inert. Gripper intent remains CLOSE
only for `action[7] > 0.5`, otherwise OPEN, through the existing async server
adapter with measured completion/contact/fault handling.

## Dependency inventory and classification

| Invocation/resource | Classification and actual requirement |
|---|---|
| `candidate_target` absolute joint mapping | Required for interpretation. Pure seven-element arithmetic; no FK, IK, TCP, TF or model masses. |
| `limits_from_urdf` in ROS startup | Required safety boundary. Reads only the seven bounded arm joints and position limits, plus an XML hash. Hand links, finger joints and a TCP are not needed for this calculation. |
| `safety_gate` joint margin/freshness/readiness | Required physical safety. Existing 0.1 rad target margin, 0.1 s state/receive age, 1 s action-age gate and passed readiness reasons remain. Action-age basis is the existing execution-window origin; no timing semantics were changed. |
| `singularity_metric(q)` on measured and target q | Required existing admissibility gate. Base `fr3_link0`, tip `fr3_link8`, unweighted `sigma_max/sigma_min`; independent of Robotiq TCP and tool load. |
| Former `fr3_tcp_jacobian` + TCP→flange shift inside that metric | Unnecessary computational detour through the historical hand TCP. The shift cancels the TCP offset mathematically. Replaced with a direct arm-flange Jacobian. |
| `fr3_flange_fk`/arm joint transforms | Used to construct the direct flange Jacobian; requires the FR3 arm kinematic constants, not a hand model. |
| `fr3_tcp_fk`, `fr3_tcp_finite_displacement`, hand TCP Jacobian | Not invoked by the revised autonomous path. Historical Cartesian diagnostics retain them unchanged. |
| `Fr3PinocchioKinematics`, `c4_target_audit.compare_arm_model` | Offline validation only, not imported/invoked for runtime target generation. The diagnostic can explicitly use an arm-only `fr3_link8` model. |
| IK / Cartesian pose tracking | No invocation on this autonomous path. No IK implementation was added. |
| TF listener/provider classes in `ros_observation.py` | Generic Cartesian utilities; this rollout does not instantiate them. Its observation contract does not include `tcp_frame`. |
| Controller `getZeroJacobian(kEndEffector)` at activation/update | Required for the retained hybrid control law and physical behavior. Current SDK `F_T_EE` and the gain/frame interpretation need review; no requirement that the chosen controller point coincide with the Robotiq fingertip TCP. |
| Controller `getCoriolisForceVector` | Required for retained effort law. Hardware wrapper uses q, dq, `I_total`, `m_total`, `F_x_Ctotal`; configured mounted-tool/load inertial properties remain material. |
| `EE_T_K` passed by the hardware model wrapper | SDK interface carries it, but the inspected libfranka 0.15.0 `zeroJacobian(kEndEffector)` branch uses F_T_EE; EE_T_K is multiplied only for `kStiffness`. This controller does not request that branch. Do not turn an unused stiffness-frame calculation into a fingertip-TCP prerequisite. |
| `franka_health` robot mode/errors/collision indicators | Required runtime rejection of fault/contact-reflex indicators and invalid feedback. Preserved. Reactive indicators are not predictive collision or workspace clearance checks. |
| MoveIt collision meshes, SRDF, Cartesian workspace checking | No prospective query/enforcement is invoked by this policy/forwarder/controller path. MoveIt/Servo collision checking cannot be assumed to supervise separately streamed targets. A wrong hand mesh invalidates any claim relying on that mesh, but does not change seven-joint arithmetic. |
| Mounted tool/coupling/camera envelope and collision/workspace supervision | Still required physical safety. Must cover the actual assembly and finger/linkage sweep, environment, uncertainty and independently enforced bounds. A full articulated Robotiq URDF is not inherently required if an adequate verified conservative representation/supervision arrangement is provided. |

The controller inspection used the unified lab sources, its unchanged YAML,
`franka_hardware/include/franka_hardware/model.hpp`, and the local clean
libfranka `0.15.0` source (`~/libfranka/src/model.cpp`). The hardware build's
Franka_DIR points at `/usr/local/lib/cmake/Franka`, whose installed version is
0.15.0. No running process's tool/load values are assumed from source inspection.

## What `tcp_frame=null` and the guard actually mean

`load_shadow_config` explicitly **accepts and requires null** for the current
Robotiq profile. It prevents silently substituting `fr3_hand_tcp`; it does not
reject null. `observation_contract` omits that field. The ROS entry point stops
at `require_supported_arm_geometry`, before ROS initialization or output/command
creation. The episode repeats the guard defensively. The old unconditional
error bundled TCP, collision and payload, obscuring their different roles.

The minimal separation implemented here is:

- Direct `fr3_flange_jacobian` for the runtime singularity gate and offline
  flange-model comparison. No hand transform is evaluated then cancelled.
- Explicit `joint_space=True` scope at the existing autonomous/hold entry point,
  policy episode and C4 blocker diagnostic. Its rejection names the three
  remaining categories: collision/clearance/workspace supervision, controller
  frame/gain compatibility, and end-effector/payload inertial settings.
- Other callers retain the conservative Cartesian-tool scope. Historical
  Cartesian helpers and profiles remain available.

This changes dependency/diagnostic precision, **not physical authorization**.
Both guards still reject the Robotiq profile regardless of config approval
booleans. A reviewed evidence-bound integration of the remaining safety
requirements is not implemented by setting a field in JSON. No fresh runtime
collision-enforcement mechanism or collision approval has been invented.

## Source integrity, independently of geometry

| Repository/contract | Inspected revision and status |
|---|---|
| SAPS working branch | `ee29c2c7d931a44ea1878c44c27228ac2f752c1d`, `implement/physical-zed-robotiq`, dirty with existing integration work |
| Lab | `4bb6cdc58839dcdd94acbe1633b8f361676a4eb6`, `implement/zed-robotiq`, dirty |
| OpenPI required/current | `15a9616a00943ada6c20a0f158e3adb39df2ccac`, clean; pretrained checkpoint pin retained |
| IGD source evidence | `438e4ae3145944042033395940848bf107cb9965`, `commissioning/fr3-spacemouse`, dirty; recorded by rollout provenance |
| Existing SAPS ancestry check | `dc5ce5e1ee038d793f7b672cc72a5fffce8d06c2` is an ancestor of current HEAD |
| Robotiq lab manifest baseline | `4bb6cdc58839dcdd94acbe1633b8f361676a4eb6` is an ancestor of lab HEAD |

`configs/zed_robotiq_lab_sources.json` is unchanged, SHA256
`402b4d66027b056fcc70464c10ffef3a3d809863fa75ba9787a7c6889c4365ec`.
Its full-file hashes describe a ZED extension to the baseline, not the baseline
Git tree itself: four added ZED files and four changed baseline files.
The present lab tree differs from the manifest in **8 changed, 11 added, 0
missing paths**. Exact paths, identities and comparison results are in the
[source-integrity evidence](validation/2026-11-09_joint_space_geometry/source_integrity.json).
Both controller/forwarder YAML files and their implementation paths remain
outside those changed/added sets.

`hardware_timing_helpers` still requires a clean lab checkout, baseline ancestry,
an exact full tracked-file hash match, and a clean SAPS checkout before loading
the timing helper. Gripper-enabled Robotiq execution routes through that same
helper; historical G1B/arm pins remain unchanged. Dirty IGD is recorded evidence,
not an additional clean-check gate claimed to exist here. No manifest was
regenerated, branch switched, commit made, or provenance bypass added.

The intended **new reviewed integration commit does not yet exist**. Review
the preserved changes, obtain explicit commit authorization, commit the chosen
integration state, and review a manifest matching that state. Merely returning
to baseline HEAD would remove the required C3 integration and still would not
match the extension manifest. Resolving geometry does not resolve provenance.

## Offline verification and remaining physical requirements

Tests cover direct-vs-previous shifted Jacobians at 42 poses, finite-difference
flange FK/Jacobian agreement, hand helpers deliberately broken/offset set NaN,
arm-only limits with null TCP, unchanged target preparation, retained joint and
runtime rejection reasons, both physical entry guards, and dirty/hash-mismatched
source loading rejection. Existing threshold-boundary tests were updated only
to inject their synthetic Jacobian at the new direct function.

The saved `shadow_004` chunk was independently checked against a newly expanded
arm-only model: Pinocchio 3.9.0 agreed with runtime flange FK/Jacobian to about
1e-15 across recorded q plus all 15 snapshot targets. This is archived snapshot
analysis, not a new live trajectory validation. No tool collision or dynamic
safety approval follows. The [small offline evidence](validation/2026-11-09_joint_space_geometry/snapshot_checks.json)
records the input hashes, comparison and unchanged NO-GO outcome.

Commands executed:

```bash
cd ~/saps-openpi-replication
PYTHONNOUSERSITE=1 PYTHONPATH=src:tests/unit .venv-physical/bin/python -m unittest \
  test_physical_joint_geometry test_single_action test_fr3_forward_kinematics \
  test_physical_c4 test_physical_c4_target_audit test_physical_policy_execution -q
make check
git diff --check
```

Focused result: **85 tests passed**. Full result: **461 tests passed**, plus
successful compilation. Arm-only expansion also passed `check_urdf` and the
existing saved-chunk CLI. No ROS package source was changed by this task.

Remaining requirements are specifically:

1. Verify actual FR3 arm kinematics/joint limits and the current SDK controller
   frame F_T_EE against the retained hybrid gains/behavior. A measured Robotiq
   fingertip TCP, IK model and Cartesian task-frame transform are not prerequisites
   solely for joint-space interpretation.
2. Establish the real mounted-tool/coupling/camera/finger-sweep collision and
   clearance envelope, workspace bounds, uncertainty and effective independent
   supervision for the streamed motions. Do not equate joint bounds, reactive
   collision indicators or visible meshes with complete collision safety.
3. Verify installed end-effector and payload mass/CoM/inertia and robot-native
   tool/load configuration. URDF inertials do not configure libfranka. No
   setter was called, and no nominal Franka tool values were assumed correct.
4. Review/resolve the independent clean-source and manifest requirements; verify
   the unified overlay, fresh observations, controller/forwarder readiness and
   delivery, one authorized calibrated gripper owner, native safety/reflex
   settings and operator/E-stop supervision. Keep the verified binary gripper
   convention, speed/force and policy identity.
5. Implement/review acceptance of that concrete safety evidence in the existing
   hardware guard. It remains intentionally blocking today; this audit is not an
   alternative permission flag.

## One supervised autonomous chunk, conditional on all remaining gates

There is an existing threshold-default discrepancy: Makefile C1C2 targets pass
**17.0**, while the Python `SINGULARITY_LOWER`/direct CLI default is **30.0**
(the CLI help text still says 17). Neither value was changed. To retain the
Makefile autonomous gate explicitly, use `--singularity-limit 17.0` below.
The controller/forwarder are unchanged. Do not use a default change as an
implicit way to relax a safety gate.

After all requirements above pass and the researcher explicitly authorizes
the run, use the existing independent-service startup procedure in
[the C4 guide](physical_c4.md), then:

```bash
cd ~/saps-openpi-replication
source tools/activate_physical_env.sh
export PHYSICAL_CONFIG="$PWD/configs/local/c4_zed_robotiq.json"
export PHYSICAL_PROMPT='pick up the object and hold it'
"$PHYSICAL_PYTHON" scripts/physical_policy_execution.py --execute --enable-gripper \
  --config "$PHYSICAL_CONFIG" --prompt "$PHYSICAL_PROMPT" \
  --host 127.0.0.1 --port 8000 --singularity-limit 17.0 \
  --max-replans 1 --max-executed-policy-chunks 1 \
  --observation-timeout 30 --policy-timeout 120 \
  --application-confirmation-timeout 2 \
  --lab-stack-dir "$HOME/franka_ehsan_ros2_ws/src/fr3_lab_stack" \
  --igd-control-dir "$HOME/franka_ehsan_ros2_ws/src/igd_fr3_control" \
  --output-dir "outputs/physical_c1c2_gripper/c4_joint_chunk_$(date -u +%Y%m%dT%H%M%SZ)"
```

This is still rejected today. It permits at most eight actions at 15 Hz plus
the existing holds, warm-up, inference and bounded confirmation waits. Approve
the actual prompt, keep the operator at the E-stop, inspect the complete first
run before increasing any budget, and never automatically resume after a fault.

Changed for this audit: `fr3_forward_kinematics.py`, `single_action.py`,
`c4_target_audit.py`, `hardware_provenance.py`, the scope argument in
`live_inference_hold_ros.py`, `policy_execution.py`, `c4_readiness.py`,
`test_physical_joint_geometry.py`, `test_single_action.py`, this report and its
C4-guide pointer/evidence. All earlier uncommitted changes were preserved.
