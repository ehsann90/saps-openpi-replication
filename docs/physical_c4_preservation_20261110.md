# Preserve the working physical C4 deployment — 2026-11-10

This is a source preservation snapshot before refactoring or package migration.
Existing runtime files, launch behavior, controller settings, safety limits,
policy parameters, observation preprocessing, experiment semantics, seed handling,
and output schemas were preserved byte-for-byte. No physical robot commands,
services, camera processes, builds, policy servers, pushes, history rewrites,
cleanups, or package migrations were performed during preservation.

## Starting repositories and recovery

| Repository | Branch | Starting HEAD |
| --- | --- | --- |
| SAPS | `implement/physical-zed-robotiq` | `ee29c2c7d931a44ea1878c44c27228ac2f752c1d` |
| fr3_lab_stack | `implement/zed-robotiq` | `4bb6cdc58839dcdd94acbe1633b8f361676a4eb6` |

Both existing development branches are used for preservation commits. Obtain the
snapshot SHA with `git log -1` in each checkout; the local preservation directory
also records both full SHAs and contains verified Git bundles after commit.
The SAPS commit adds this record to the 64 existing changed/new files; the lab
commit preserves its 19 existing changed/new files.

Local recovery directory (ignored, outside Git):

`outputs/physical_c4/preservation_20261110T101822Z`

It contains reviewed file inventories, original status/diffs, validation logs,
`rollout_evidence_inventory.json` (per-file SHA-256, original provenance,
configuration, startup readback, and source comparisons), local-profile hashes,
`local_hardware_profiles.tar.gz`, and a supporting dependency snapshot
`igd_fr3_control_worktree.tar.gz` plus `igd_fr3_control_HEAD.bundle`.
The local profiles archive includes all six files under `configs/local/`,
including the commissioned C4 profile and robot tool profiles, without altering
the originals. These local recovery artifacts require a separate backup when
moving machines; cloning the source repositories alone does not restore them
or the raw rollout evidence.

## Physical rollout evidence

The original archive remains at:

`outputs/physical_c4/c4_dev_long_20261110T09.zip`

- Size: 455,251,233 bytes.
- SHA-256: `fa813a9f1ce28e675c31b94e8e25e6ac7e50dff5b3a56cab56fbdd464f255197`.
- All 823 ZIP entries passed CRC verification. Every archived file matches its
  local counterpart; no files are missing locally.
- No standalone `manifest.json`, `summary.json`, or named configuration file is
  in the archive. The ZIP directory is its member manifest. `run.json` is the
  finalized summary; `start.json`, `provenance.json`, `runtime_provenance.json`,
  `effective_hardware.json`, `joint_limits.json`, and readiness records provide
  configuration and source evidence. No substitute raw summary was fabricated.

| Local directory under `outputs/physical_c4/` | In ZIP | Executed policy actions | Recorded outcome |
| --- | --- | ---: | --- |
| `c4_dev_long_20261110T093404Z` | yes | 176 | abort: gripper transition arm readiness lost |
| `c4_dev_long_20261110T094147Z` | yes | 224 | abort: post-hold readiness failed |
| `c4_dev_long_20261110T094704Z` | yes | unknown | no finalized `run.json`; startup evidence only |
| `c4_dev_long_20261110T095014Z` | yes | 154 | abort: no new measured q for step |
| `c4_dev_long_20261110T095745Z` | no | 125 | abort: safety gate rejected |
| `c4_dev_long_20261110T101318Z` | no | 80 | abort: ROS node context invalid |

Times in run directory names are UTC. The latest archived finalized run is
09:50:14 UTC; the latest local extended run is 10:13:18 UTC (11:13:18 Oslo).
Raw logs, images, model snapshots, timing files, and policy predictions remain
in their original ignored directories. Execution evidence establishes a
functional hardware/policy pipeline, not manipulation success, physical safety
qualification, or a formal successful experiment. All finalized extended runs
record `status=failed`, `task_outcome=abort`, and `termination_reason=runtime_abort`.

## Recorded revisions and effective configuration

The finalized extended runs record these starting revisions:

| Component | Recorded revision | Recorded state |
| --- | --- | --- |
| SAPS | `ee29c2c7d931a44ea1878c44c27228ac2f752c1d` | dirty |
| fr3_lab_stack | `4bb6cdc58839dcdd94acbe1633b8f361676a4eb6` | dirty |
| igd_fr3_control | `438e4ae3145944042033395940848bf107cb9965` | dirty |
| OpenPI | `15a9616a00943ada6c20a0f158e3adb39df2ccac` | clean |

HEAD alone does not identify the dirty rollout sources. Each original
`provenance.json` retains source-file and diff hashes. Current files match all
128 recorded SAPS source hashes, 74 lab hashes, 33 IGD hashes, and 81 OpenPI
hashes for every listed run from 09:41 onward. For 09:34, only
`src/saps/physical/policy_execution.py` differs; retain its original source hash
in the evidence inventory rather than claiming an exact match for that run.
These preservation commits identify the current working source state, not
historical clean revisions retroactively assigned to the runs.

The recorded lab manifest is `configs/zed_robotiq_lab_sources.json`, SHA-256
`402b4d66027b056fcc70464c10ffef3a3d809863fa75ba9787a7c6889c4365ec`.
Its baseline and hash map were preserved unchanged. It does not match the
working lab integration. A clean preservation commit does not qualify strict
experiment mode; no manifest refresh or safety authorization is implied.

The recorded local config is `configs/local/c4_zed_robotiq.json`, SHA-256
`cc0032c875ff4138157c6ba6eaeb7c72f07ff7ff4144d4cb5eff8940d457a7a2`.
The current local file matches. Exact config objects for each run are retained
in their raw records and the local evidence inventory; the tracked template
retains placeholders and is not the commissioned profile.

Recorded finalized-run settings:

- Policy: `pi05_droid`, `gs://openpi-assets/checkpoints/pi05_droid`, native action
  shape `[15, 8]`, reference future open-loop horizon 8; episode seed 20260827.
  The pre-rollout readiness record specifies warmup seed 20260917.
- Mode: `development`; max replans 1000; max executed policy chunks 1000;
  singularity limit 60.0; no inference-stop replan; outcome provider
  `unevaluated_outcome`. Recorded timeouts: observation 30 s, policy 120 s,
  application confirmation 2 s.
- Wrist: ZED Mini/`zedm`, serial 15330503,
  `/wrist_camera/zed_node/rgb/color/rect/image`.
- Exterior: ZED 2i/`zed2i`, serial 32919961,
  `/external_camera/zed_node/rgb/color/rect/image`.
- Wrapper 5.5.0, SDK 5.5.1; HD720, 30 Hz, BGRA8; native image shape
  `[720,1280,3]` describes decoded RGB with alpha removed. Cameras are not
  hardware synchronized. Camera interface: latest advancing, reliable,
  history depth 2, SYSTEM_CLOCK. Freshness remains 0.5 s age / 0.25 s skew.
- Client images: `[180,320,3]` uint8 RGB, existing 16:9 center crop,
  INTER_AREA downsize / INTER_LINEAR upsize, no flip/rotate/mirror.
- Gripper: Robotiq 2F-85, slave 9, 115200 baud, serial timeout 0.2 s,
  command timeout 3 s, speed register 64, force register 32, open/close 0/255.
  Commissioned calibration: positions 3/227, maximum width 0.085 m.
  Exact machine serial port remains in local config/provenance, outside Git.
- Robot state `/franka/joint_states`; gripper state `/robotiq/status`;
  base `fr3_link0`; TCP null. Config records `execution_enabled=false` and
  geometry `execution_ready=false`; those fields do not override recorded
  executed-action evidence or establish measured hardware qualification.
- Latest recorded robot description SHA-256:
  `0d2660f7c5b7b9726dc00d7581539fa31890b6844e66fb23e56c5ce5bcecccde`.
  Exact startup readback remains in each `effective_hardware.json` and the
  recovery inventory. No controller setting was inferred or changed here.

The local upstream ZED wrapper is clean at
`431dcf4b1ea39caf894ca6ea294437437dbdb034`; this is an inspection-time revision,
not an additional revision recorded by the rollout. Its observed installed
version/source hashes remain in the original camera identity evidence.
No upstream Stereolabs package was changed.

## Files deliberately outside Git and ignore review

Existing ignore rules cover all currently observed generated artifacts:
`outputs/`, generated `results/` except the already tracked historical pilot,
`data/`, `videos/`, `logs/`, `*.log`, `*.tmp`, Python caches, physical virtualenv,
and `configs/local/`. No relevant source is hidden by the SAPS ignored-file
inventory outside these categories. The lab ignores build/install/log,
validation output, Python bytecode and pytest cache. Neither ignore file was
changed. Existing ignore patterns are not a general credential filter; staging
was restricted to the reviewed files below and scanned for common credential
formats and credential-bearing paths. No credential files were identified in
that set.

Deliberate exclusions:

- Raw rollout ZIP, timing/log/image/prediction output and new preservation
  artifacts: generated, large, and kept outside Git.
- Local commissioned profiles, device identities and backup launch file:
  machine-specific; backed up locally with exact hashes.
- Python/pytest caches, virtualenv dependencies and other generated data:
  reproducible or machine-specific, not implementation source.
- Lab ignored September acceptance raw evidence: historical generated logs,
  left untouched at its existing location.
- Small existing November audit summaries/probe sources under
  `docs/validation/` are included as reviewable supporting documentation,
  not promoted to successful manipulation evidence.

## Non-actuating validation

- `git diff --check` passed for both original working trees; staged checks are
  required again immediately before each commit.
- SAPS focused host unittest run: 184 tests passed. The exact argument list is
  in `saps_focused_command.json` in the recovery directory; it covers every
  changed/new unit test file in the preservation inventory.
- `make check` and `make compile` were attempted separately; both could not
  access `/var/run/docker.sock` (permission denied). No escalation, image
  build, GPU execution, persistent service, or robot access was attempted.
- Host full discovery:
  `PYTHONNOUSERSITE=1 PYTHONPATH=src .venv-physical/bin/python -m unittest discover -s tests/unit -p 'test_*.py' -v`:
  430 tests discovered/run, with eight import errors from missing `imageio` or
  `tyro`; no test assertion failures. Full Docker validation remains unverified.
- Initial host compile encountered unwritable existing bytecode caches.
  `PYTHONNOUSERSITE=1 PYTHONPYCACHEPREFIX=<recovery>/pycache .venv-physical/bin/python -m compileall -q src scripts tests tools`
  passed. Existing cache ownership was not changed.
- `bash -n tools/activate_physical_env.sh` passed.
- Lab after sourcing ROS Jazzy and the existing workspace install:
  `ROS_LOG_DIR=<recovery>/ros_logs ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1 PYTHONPATH="$PWD:$PYTHONPATH" /usr/bin/python3 -m pytest -q -p no:cacheprovider test/test_camera_pairing.py test/test_camera_validation.py test/test_check_rgb_rates.py test/test_zed_cameras.py test/test_tool_model.py -k 'not test_read_only_capture_with_isolated_fake_state'`:
  76 passed, one ROS fixture test deliberately deselected to avoid starting ROS
  nodes. The first attempt had two ROS log-path permission errors; redirecting
  only the log path resolved them. No camera container was started by tests.
- Lab `/usr/bin/python3 -m compileall -q fr3_lab_stack_runtime launch scripts test`
  passed. No C++/colcon build or hardware smoke test was run.
- ZIP CRC validation and SHA-256 local/archive comparisons passed.

## Ownership, remotes and remaining limitations

Both repositories and their `.git` directories are owned by the current local
user; no repository ownership conflict was found. Existing root/unwritable SAPS
bytecode caches and Docker socket permissions limit standard verification.
SAPS origin is `https://github.com/ehsann90/saps-openpi-replication.git`;
lab origin is `https://github.com/frdedynamics/fr3_lab_stack.git`.
Remote write permission was not tested, and nothing was pushed.

The supporting `igd_fr3_control` checkout remains dirty on
`commissioning/fr3-spacemouse`: modified `igd_fr3_control/view_capture.py` and
untracked `display_grasp_rviz.py`, `test_grasp.py`, `view_capture2.py` under that
package. It is outside the requested commit scope; its working files, original
diff/status and HEAD history are preserved locally for recovery. Source
preservation does not resolve historical physical-readiness findings or
establish manipulation success. Robotiq/ZED package migration has not begun.

## Exact source preservation inventory

### SAPS: 64 existing changed/new files

- `Makefile`
- `README.md`
- `configs/physical_pi05_fr3_c4.json`
- `configs/physical_pi05_fr3_zed_robotiq.json`
- `configs/zed_robotiq_lab_sources.json`
- `docs/physical_c4.md`
- `docs/physical_c4_execution_audit.md`
- `docs/physical_development_rollout.md`
- `docs/physical_joint_space_geometry_audit.md`
- `docs/physical_motion_safety_audit.md`
- `docs/physical_tool_load_configuration.md`
- `docs/physical_zed_robotiq.md`
- `docs/validation/2026-11-06_c4_saved_chunk/summary.json`
- `docs/validation/2026-11-09_joint_space_geometry/snapshot_checks.json`
- `docs/validation/2026-11-09_joint_space_geometry/source_integrity.json`
- `docs/validation/2026-11-09_motion_restrictions/CMakeLists.txt`
- `docs/validation/2026-11-09_motion_restrictions/evidence.json`
- `docs/validation/2026-11-09_motion_restrictions/native_limits_probe.cpp`
- `docs/validation/2026-11-09_motion_restrictions/native_limits_result.json`
- `requirements-robotiq.txt`
- `scripts/physical_c4.py`
- `scripts/physical_policy_execution.py`
- `scripts/physical_single_action.py`
- `src/saps/physical/c4_observation.py`
- `src/saps/physical/c4_readiness.py`
- `src/saps/physical/c4_target_audit.py`
- `src/saps/physical/camera_launch.py`
- `src/saps/physical/droid_gripper.py`
- `src/saps/physical/fr3_forward_kinematics.py`
- `src/saps/physical/gripper_ros.py`
- `src/saps/physical/gripper_validation.py`
- `src/saps/physical/hardware_provenance.py`
- `src/saps/physical/live_inference_hold_ros.py`
- `src/saps/physical/live_observation.py`
- `src/saps/physical/live_shadow.py`
- `src/saps/physical/policy_execution.py`
- `src/saps/physical/robotiq.py`
- `src/saps/physical/robotiq_hand.py`
- `src/saps/physical/robotiq_node.py`
- `src/saps/physical/rollout_provenance.py`
- `src/saps/physical/ros_observation.py`
- `src/saps/physical/shadow_config.py`
- `src/saps/physical/shadow_ros.py`
- `src/saps/physical/single_action.py`
- `src/saps/physical/verifier_ros.py`
- `tests/unit/test_droid_gripper.py`
- `tests/unit/test_fr3_desk_profile_audit.py`
- `tests/unit/test_g2_archived_gripper_sweep.py`
- `tests/unit/test_physical_c4.py`
- `tests/unit/test_physical_c4_target_audit.py`
- `tests/unit/test_physical_development_provenance.py`
- `tests/unit/test_physical_joint_geometry.py`
- `tests/unit/test_physical_live_observation.py`
- `tests/unit/test_physical_policy_execution.py`
- `tests/unit/test_physical_robotiq.py`
- `tests/unit/test_physical_robotiq_activation.py`
- `tests/unit/test_physical_robotiq_observation.py`
- `tests/unit/test_physical_zed_config.py`
- `tests/unit/test_single_action.py`
- `tools/activate_physical_env.sh`
- `tools/analysis/audit_fr3_desk_profile.py`
- `tools/diagnostics/audit_c4_saved_chunk.py`
- `tools/diagnostics/sweep_archived_droid_gripper_state.py`
- `tools/prepare_c4_config.py`

Additional file created by this task: `docs/physical_c4_preservation_20261110.md`.

### fr3_lab_stack: 19 existing changed/new files

- `CMakeLists.txt`
- `README.md`
- `docs/robotiq_model_audit.md`
- `docs/zed_cameras.md`
- `fr3_lab_stack_runtime/camera_pairing.py`
- `fr3_lab_stack_runtime/camera_validation.py`
- `fr3_lab_stack_runtime/tool_model.py`
- `fr3_lab_stack_runtime/zed_cameras.py`
- `launch/dual_zed.launch.py`
- `package.xml`
- `rviz/fr3_lab_stack_zed.rviz`
- `scripts/check_camera_pairing.py`
- `scripts/check_rgb_rates.py`
- `scripts/inspect_tool_model.py`
- `test/test_camera_pairing.py`
- `test/test_camera_validation.py`
- `test/test_check_rgb_rates.py`
- `test/test_tool_model.py`
- `test/test_zed_cameras.py`
