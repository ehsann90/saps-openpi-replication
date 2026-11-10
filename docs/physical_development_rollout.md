# Bounded C4 development rollout and experiment qualification

2026-11-09. This is the current development workflow; it supersedes earlier
C4 statements that dirty SAPS/lab trees or an outdated lab manifest must block
every physical invocation. The autonomous joint-space path also no longer has
an unconditional Robotiq geometry/documentation approval gate. Existing runtime
checks and explicit operator execution authorization remain.

The subsequent [targeted motion-safety audit](physical_motion_safety_audit.md)
includes the fresh active-controller/tool-state capture and verifies the existing
torque-slew mechanism. It distinguishes native effort-interface protection from
an additional software trajectory limiter. Its hardware findings remain relevant;
its earlier statements about an unconditional software veto are superseded here.

For the mounted Robotiq/ZED assembly, use the targeted
[tool/load configuration procedure](physical_tool_load_configuration.md).
It distinguishes fixed end-effector properties from additive external payload
and preserves the hybrid controller's configured reference frame and gains.
That reference procedure is not a software execution prerequisite: missing
assembly inertial values and the historical 0.73 kg readback do not veto this
joint-space development command. They remain recorded hardware limitations.

## Two explicit provenance modes

| Mode | Source requirements | Physical requirements |
|---|---|---|
| `--provenance-mode development` | Record actual SAPS/lab revisions, dirty diffs, source/config hashes and manifest differences; no clean-tree, baseline-ancestry or exact-manifest gate for those two repositories. | Explicit `--execute`, positive finite replan/chunk limits, and existing per-target/runtime checks; default chunk cap remains one. No blanket hardware-profile approval gate. |
| `--provenance-mode strict` | Retain clean-source, reviewed lab manifest, baseline ancestry and historical pin checks. Source rejection is recorded as experiment qualification failure. | Same physical requirements; qualified source bytes do not establish hardware safety. |

The physical execution CLI defaults to **strict**, preserving historical
commands and qualified-run source behavior. Development execution is scoped
to the explicit C4 profile. The non-actuating C4 CLI defaults to development,
as its observation/shadow workflow already allowed dirty integration sources;
it also accepts `--provenance-mode strict` for source-qualified checks.

The unchanged clean pinned OpenPI checkout and pretrained policy/server identity
remain required in both modes. Developing SAPS/lab adapters is distinct from
changing the frozen policy implementation or checkpoint. No Git commit, reset,
checkout, manifest regeneration, ROS-package refactor or hardware command is
performed by selecting either mode.

## Four readiness levels

Every C4 invocation records these separately in `readiness.json` and `run.json`:

| Level | Evidence/status |
|---|---|
| `observation` | Complete camera/joint/measured-gripper observation with existing identity, encoding, calibration, ownership and freshness checks. Reports not checked, checking, passed or failed. |
| `policy_inference` | Bounded warmed inference with the unchanged pretrained server. Readiness-only handshake reports `identity_verified_inference_not_tested`, not a successful inference. Observation-only mode never contacts the policy server. |
| `physical_runtime` | Actual runtime check outcome is reported separately as `runtime_checks`. Tool/load/clearance findings stay in `unverified_conditions`, with status `unverified`; they do not act as a profile blacklist or prove physical safety. |
| `reproducible_experiment_qualification` | Source qualification is nested under `sources`; dirty/unreviewed sources are unqualified here. Complete experiment qualification also requires physical and experimental evidence. It is not inferred from a successful observation or source check. |

These levels describe evidence collected during this invocation, not a stored
authorization for later motion. A dirty-source shadow run may pass observation
and inference checks while hardware qualification remains `unverified` and
experiment qualification remains `not_qualified`. Observation/shadow prints
`physical_rollout_readiness=UNVERIFIED`, not an automatic permission to move.
Clean source metadata does not clear the unresolved hardware findings.

## Evidence retained without premature commits

`provenance.json` is an immutable startup snapshot. It includes:

- HEAD, branch, short status and dirty flag for SAPS, lab, IGD and OpenPI.
- SHA256 hashes of staged, unstaged and combined HEAD diffs, without storing
  the raw diff text. Relevant tracked **and untracked** source/configuration
  files are individually hashed; untracked content is not lost just because
  Git diff omits it.
- Requested hardware configuration and the exact selected config-file hash,
  including an ignored local hardware config when explicitly selected.
- The existing manifest hash, changed/added/missing paths, baseline ancestry
  and qualification reasons. A missing/invalid/stale manifest disqualifies
  strict qualification but is not a development source approval gate.

Later evidence is separate: `runtime_provenance.json` records loaded helper
identity/hash and clock evidence; `effective_hardware.json` records available
URDF hash and observed ROS camera/controller/gripper information. A blocked
startup explicitly says `not_observed`. Requested configuration is never
represented as verified device readback. Neither file is a load/collision
approval. Existing raw observations, prediction chunks, timing/delivery audits,
gripper command/completion evidence and error logs remain available.

Source capture occurs before dynamic helper loading and before any ROS command
interface is created. Thus a failed development startup still
saves provenance and readiness. Existing output directories are never overwritten.
All source hashing/qualification work is outside the 15 Hz action loop.

## Physical safety audit

| Requirement | Existing mechanism / remaining limitation |
|---|---|
| Joint and target limits | Canonical seven-joint URDF limits, target margins and flange singularity checks are retained. No scaling or bound was relaxed. |
| Joint velocity, acceleration and torque | The boundary checks recorded dq finiteness when available. The hardware effort path already applies native torque-slew processing. No new 15 Hz trajectory limiter is added; `15*delta_q` is not a speed guarantee. Actual native/manager protection and configured limits must not be inferred from unused position/velocity interfaces. |
| Freshness and timing | Existing observation/state/action ages, advancing measurements, camera/source barriers, 15 Hz absolute deadlines, no catch-up, forwarder timestamps and target application confirmations remain. |
| Controller and forwarder health | Existing active-controller/activation identity, graph ownership, runtime telemetry and delivery/T4 checks remain. Source hashes do not replace these checks. |
| Robot faults/collision reflex | Existing Franka mode/errors and collision-indicator monitoring remains. These reactive signals do not establish predictive geometric collision safety. |
| Workspace/collision/clearance protection | No prospective MoveIt collision/workspace query was found on the streaming path. Actual assembly envelope, environmental clearances and independent effective protection must be verified. A picture/mesh or config approval flag is insufficient. |
| Controller/tool/load configuration | Existing hybrid gains use the configured SDK end-effector Jacobian and Coriolis model. Verify actual frame interpretation and mounted-tool/payload inertial settings. A fingertip TCP is not required just to interpret joint actions. No tool/load setter is called by the new modes. |
| Gripper | Single serial owner, measured closure/calibration, authorization, async command acknowledgement/completion/contact, stale/fault rejection, reversal/STOP/error-latching behavior are retained. No automatic activation, OPEN/release, retry, speed or force change was added. |
| Operator/native safety | Emergency-stop capability, supervision, native limits/reflex settings and the safe hold/shutdown arrangement require actual verification. A software flag cannot attest them. |

The duplicate unconditional Robotiq profile gates were removed from ROS assembly
and the policy episode. Software still rejects invalid/stale commands, unsafe
joint targets, controller/forwarder faults, missing delivery evidence and invalid
gripper state. `--execute` remains mandatory, and development requires positive finite
integer replan and chunk limits. No `geometry.execution_ready` Boolean grants permission.
Older standalone diagnostic/validation entry points retain their conservative
geometry guard; they are not used by this autonomous joint-space entry point.

Before using `--execute`, the researcher must review the real mounted-hardware
settings and clearances. The fresh capture confirms the controller frame at
103.4 mm / -45 degrees and total configured mass 0.73 kg, but does not establish
their suitability for the Robotiq/coupling/camera assembly. No mass, inertia,
URDF, load or robot setting is changed automatically. There is no additional
software approval document or guard implementation required to run joint control.

### Remaining Franka Hand assumptions

| Dependency | Current joint-space use / minimum correction |
|---|---|
| Deployed URDF and finger meshes | Still describe Franka Hand. Autonomous startup extracts only the seven arm limits; it does not use finger links or this mesh for collision protection. No invented Robotiq/coupling model replaces it. |
| Policy-side FK/Jacobian | Direct arm-flange Jacobian at `fr3_link8`; no Hand TCP, IK or fingertip geometry is used. Historical Cartesian tools remain separate. |
| Controller SDK frame | Captured F_T_EE is 103.4 mm Z and -45 degrees yaw, matching the historical Hand transform. It affects the unchanged hybrid law and may be retained as an intentional virtual control point only with an appropriate hardware review; it is not required to represent the fingertips. |
| Controller mass/CoM/inertia | Captured total mass 0.73 kg, external load 0 kg; SDK CoM approximately `[-0.01,0,0.03] m`, inertia diagonal `[0.001,0.0025,0.0017] kg m²`. Suitability for the installed assembly is unverified. URDF inertials do not set these SDK values. No value is guessed or automatically rewritten. |
| Gripper commands/feedback | C4 selects the existing Robotiq ROS server, register targets and measured actual-position calibration. Franka Move/Grasp/Stop and finger JointState conversion are not used for this backend. Contact remains separate from closure. |

Thus the genuine remaining hardware issue is matching the actual mounted load,
control-point choice and safe clearances to the existing robot configuration,
not adding a TCP/IK model or collecting approval documents. Review those settings
before explicitly running the command; native reflex monitoring is not proof
that an incorrect load model or collision model is harmless.

## Five existing runtime command groups

Reuse running owners. Do not start duplicates or restart a working robot stack
merely to switch provenance modes. These are five existing launch/service groups,
not a claim that ROS launches produce exactly five operating-system PIDs.
The bounded client is a temporary invocation, not another persistent supervisor.

Common SAPS terminal setup:

```bash
cd ~/saps-openpi-replication
source tools/activate_physical_env.sh
export PHYSICAL_CONFIG="$PWD/configs/local/c4_zed_robotiq.json"
export PHYSICAL_PROMPT='pick up the object and hold it'
```

1. **FR3 bringup and existing impedance controller:** retain the validated
   running hardware stack and active streaming controller. Verify read-only
   with `ros2 control list_controllers --controller-manager /controller_manager`.
   For cold bringup use the existing [C4 terminal procedure](physical_c4.md);
   controller switching/activation remains a separate supervised operation.
2. **Existing streaming forwarder:** reuse the one running forwarder, or start
   its unchanged command after sourcing the unified workspace:

   ```bash
   ros2 run fr3_lab_stack fr3_streaming_joint_target_forwarder --ros-args \
     --params-file "$HOME/franka_ehsan_ros2_ws/src/fr3_lab_stack/config/streaming_joint_target_forwarder.yaml"
   ```
3. **Dual ZED launcher:** reuse the validated C3 owner, or:

   ```bash
   ros2 launch fr3_lab_stack dual_zed.launch.py \
     wrist_serial:=15330503 exterior_serial:=32919961 exterior_model:=zed2i \
     wrapper_version:=5.5.0 resolution:=HD720 fps:=30
   ```
4. **One Robotiq server:** observation/shadow can use existing read-only serve
   mode. Combined execution requires the explicitly authorized existing server:

   ```bash
   "$PHYSICAL_PYTHON" -m saps.physical.robotiq_node \
     --config "$PHYSICAL_CONFIG" --mode serve --authorize-gripper
   ```

   Stop the old owner before a separately supervised restart. Do not run a
   standalone serial observation command alongside the persistent server.
5. **Pretrained policy server:** reuse the correctly identified running server.
   If absent, the existing command is `make droid-policy-server`. It retains
   `gs://openpi-assets/checkpoints/pi05_droid`; resolve port/server ownership
   rather than starting a competing checkpoint instance.

No new daemon, driver, controller, inference stack or orchestration process is
introduced. Source requirements do not require restarting any of these groups.

## Non-actuating development checks

Use new output directories and explicitly review the actual task prompt:

```bash
"$PHYSICAL_PYTHON" scripts/physical_c4.py --mode observations \
  --provenance-mode development --config "$PHYSICAL_CONFIG" \
  --samples 1 --prompt "$PHYSICAL_PROMPT" \
  --output-dir "outputs/physical_c4/dev_observation_$(date -u +%Y%m%dT%H%M%SZ)"

"$PHYSICAL_PYTHON" scripts/physical_c4.py --mode shadow \
  --provenance-mode development --config "$PHYSICAL_CONFIG" \
  --samples 1 --prompt "$PHYSICAL_PROMPT" \
  --output-dir "outputs/physical_c4/dev_shadow_$(date -u +%Y%m%dT%H%M%SZ)"
```

These allow dirty SAPS/lab trees and stale manifests, record the actual sources,
and issue zero physical commands. Inspect saved images, measured states, raw
predictions and readiness levels. `--mode readiness` adds existing read-only
controller/service checks and a policy identity handshake; it does not perform
inference or clear physical blockers.

## Exact one-chunk development command — conditional on physical safety

After reviewing the actual hardware configuration and clearances, with the five
existing runtime groups ready and the researcher supervising at the emergency stop:

```bash
"$PHYSICAL_PYTHON" scripts/physical_policy_execution.py --execute --enable-gripper \
  --provenance-mode development \
  --config "$PHYSICAL_CONFIG" --prompt "$PHYSICAL_PROMPT" \
  --host 127.0.0.1 --port 8000 --singularity-limit 17.0 \
  --max-replans 1 --max-executed-policy-chunks 1 \
  --observation-timeout 30 --policy-timeout 120 \
  --application-confirmation-timeout 2 \
  --lab-stack-dir "$HOME/franka_ehsan_ros2_ws/src/fr3_lab_stack" \
  --igd-control-dir "$HOME/franka_ehsan_ros2_ws/src/igd_fr3_control" \
  --output-dir "outputs/physical_c1c2_gripper/c4_dev_chunk_$(date -u +%Y%m%dT%H%M%SZ)"
```

This command now reaches the existing runtime checks; it is not rejected solely
for being a Robotiq profile, having `tcp_frame=null`, an unset geometry approval
flag, dirty development sources or an outdated development manifest. The robot
will move only when explicitly invoked and its runtime checks permit publication.
The unresolved hardware findings are not automatically certified by this change.
Omitting the mode retains strict source checks. For an explicitly source-qualified experiment,
use `--provenance-mode strict` after review and qualification; it still does not
approve physical safety. Existing Makefile commands remain strict by default.

The explicit singularity limit 17 retains the existing Makefile choice; the
existing direct-CLI default 30 was not changed. Action mapping remains
`q_target=q_measured+0.2*clip(action[:7],-1,1)`, at most eight actions at 15 Hz,
then terminal hold. Warm-up/inference/pre-holds and bounded waits add wall time.
After final delivery validation and cleanup, the execution runner prints one
JSON summary with status, executed-action/gripper-command/terminal-hold counts,
termination reason and output directory (and error on failure). Earlier runs
saved `run.json` without a console summary; quiet return did not mean no motion.
If readiness is lost while awaiting a gripper transition, the error includes
the individual readiness reasons. That chunk's `gripper_transition` record
also retains the last reasons, snapshot start/end times and measured-joint
source/receive ages. These diagnostics do not relax freshness limits or retry
the failed transition automatically.
Terminal hold is active effort control, not an inert arm. Existing failure,
gripper STOP confirmation and no-auto-continuation procedures remain in force.

## Longer supervised development runs

Development mode now accepts larger positive integer limits. The default
executed-chunk cap stays **one**. For a longer run, use the same command above
with `--max-replans 1000 --max-executed-policy-chunks 1000` and a new output
directory. Both limits must be positive finite integers; zero, negative and
unlimited sentinels are rejected. The first reached limit stops execution.

A 1,000-chunk cap allows at most **8,000 policy actions**. This is an action
budget, not a duration guarantee: inference, readiness reacquisition and holds
add time. Each chunk still uses at most eight actions at 15 Hz. Terminal holds,
measured-state reacquisition, gripper supervision and all per-target/runtime
checks continue between chunks. Faults, timeouts, interruption or explicit task
outcomes can stop the run earlier. This budget change does not add automatic
object-pick success detection. It does not relax the singularity threshold;
retain the explicit `--singularity-limit 17.0` from the validated command.

## Changed files and offline verification

Module filenames below are under `src/saps/physical/`. This simplification changed:

- `rollout_provenance.py`: separates unverified findings from runtime blockers.
- `hardware_provenance.py`: source logging/strict qualification retained;
  joint-space findings are informational, Cartesian-tool rejection remains.
- `live_inference_hold_ros.py`, `policy_execution.py`: mode selection, logging
  before runtime startup, finite rollout-budget enforcement, removal of duplicate profile vetoes.
- `c4_observation.py`: `UNVERIFIED` observation/shadow hardware status.
- `tests/unit/test_physical_development_provenance.py`,
  `test_physical_c4.py`, `test_physical_joint_geometry.py`,
  `test_physical_policy_execution.py`: actual runtime fault rejection, explicit
  execution and mocked C4 eight-action/Robotiq completion without Hand geometry.
- `docs/physical_development_rollout.md`, `docs/physical_c4.md`,
  `docs/physical_joint_space_geometry_audit.md`: current workflow and historical
  audit pointers. Existing script CLI flags remain unchanged.

Offline tests use fake ROS/server/source evidence and temporary files. They
cover dirty observation/shadow success, preserved provenance, strict rejection,
clean-source qualification without physical approval, bounded helper loading,
finite rollout-budget enforcement, native fault rejection before policy action zero, plus existing invalid/stale
observation, command, fault and historical qualified-path regressions.

```bash
cd ~/saps-openpi-replication
PYTHONNOUSERSITE=1 PYTHONPATH=src:tests/unit .venv-physical/bin/python -m unittest \
  test_physical_development_provenance test_physical_joint_geometry \
  test_physical_c4 test_physical_policy_execution test_live_inference_hold \
  test_gripper_ros test_single_action test_physical_robotiq \
  test_physical_robotiq_observation test_physical_live_observation -q
make check
git diff --check
```

Results: **163 focused tests passed; 473 full-suite tests passed**, plus
compilation and whitespace checks. These include the mock C4 rollout and
retained historical regressions. No hardware motion was performed by these tests.

A direct read-only source capture of the actual dirty working trees also
succeeded, recording 127 relevant SAPS and 74 lab file hashes. Its diagnostic
is `/tmp/c4_development_source_capture_20261109.json`; sources remain
`unqualified_sources`; that earlier capture predates removal of the profile veto.
Effective hardware in that source-only diagnostic is
explicitly `not_observed`. This was not a physical test. No source manifest,
historical report, controller setting or Git commit was changed.
