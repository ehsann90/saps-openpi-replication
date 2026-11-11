# First operator-observed physical task success — 2026-11-11

Task/prompt: **Put the marker in the white cup**. Hardware: FR3, Robotiq 2F-85,
ZED Mini wrist and ZED 2i exterior, using the independent ROS2 hardware stack.
Run: `outputs/physical_c4/c4_dev_long_20261111T083244Z/`;
UUID `0327da03-a196-4180-8a93-e2582901e626`.
Recorded interval: 08:32:44.288927–08:33:23.934783 UTC (09:32–09:33 Oslo).

| Field | Recorded finding |
|---|---|
| Operator-observed task outcome | Success, reported by operator; first observed successful manipulation on the migrated infrastructure. |
| Visual evidence | Final exterior observation `request_0018`, source 1794385994.4127998 s, shows the marker at/in the white cup and the open gripper withdrawn, before the failed action deadline. |
| Policy completion | No explicit completion interface found in this deployed pin/checkpoint interface. |
| Runner status / task-outcome field | `failed` / `abort`, unchanged; legacy runtime-abort classification is separate from operator annotation. |
| Execution termination | `runtime_abort`, `TimeoutError: no_new_measured_q_for_step`, phase `actions`. |
| Counts | 150 applied policy actions, 3 gripper commands, 18 terminal holds; 19 main requests, 18 completed chunks. |

This is development/pilot evidence, not a success-rate estimate. No success
timestamp was recorded; sparse images do not establish exact release time,
settled-object retention or every intervening movement. Physical repeatability
and automatic task-outcome detection remain unvalidated.

## Evidence, configuration and preservation

Completed preservation verified all **138 original files** against the ZIP,
with no missing/extra files. `outputs/physical_c4/review_20261111/` holds the
inventory, all-file `SHA256SUMS`, archive verification, analysis/script, test
receipts and three exported videos. There are 19 main request directories with
native RGB pairs, policy observations, action chunks, requests/responses and
timing; startup also retains warm-up and post-warm-up observations. Other files
record URDF/joint limits, start/run, readiness, effective hardware and provenance.
No original continuous video or independent terminal transcript was available.

| Evidence | SHA-256 |
|---|---|
| `c4_dev_long_20261111T083244Z.zip` | `5008f11a5d5c82324b95b76c68d62c755a9e2da1fa2053ae76bbab00a8fdcdc9` |
| `run.json` | `bcf8c3de705a24e35fa3e72dea1220c3ab4292433da31bcd4c6c38e52b8289e6` |
| `timing.jsonl` | `c4836db2c669e88a5561902dbf6cb55feaec426e1da112d833e576217ff45f51` |
| `provenance.json` | `d1ba3792a761845d7e8c457672a3bc06a81f7bf07013337d11deb42c484d0422` |
| Exact local C4 config | `cc0032c875ff4138157c6ba6eaeb7c72f07ff7ff4144d4cb5eff8940d457a7a2` |

The immutable requested configuration is inside the run's `provenance.json`;
the local `configs/local/c4_zed_robotiq.json` still matched it during cleanup.
Run settings: seed 20260827, max replans/chunks 1000, singularity limit 60,
observation/policy/application timeouts 30/120/2 s. These are historical settings,
not new defaults or recommended budgets. Camera identities, formats, calibration,
controller evidence and preprocessing are preserved in raw records.

| Recorded clean source | Revision |
|---|---|
| SAPS migration | `f8b62482a35bc8d5ee328651ca132bef2555efff` |
| Hardware stack, main | `3dc23e39f5470ef929c5bfbb0f476b50d54fff03` |
| IGD control | `438e4ae3145944042033395940848bf107cb9965` |
| OpenPI | `15a9616a00943ada6c20a0f158e3adb39df2ccac` |

The supplied published hardware revision was `e34c74c1…`; both run provenance
and inspected local checkout instead identify `3dc23e39…`. Do not substitute
one for the other. The server handshake identifies `pi05_droid` and
`gs://openpi-assets/checkpoints/pi05_droid`; no independent full weight digest
was recorded. Strict hardware-source qualification remains unresolved.

At the operator's request, a second local copy of the verified ZIP, checksums,
review evidence and videos is under
`docs/validation/2026-11-11_physical_rollout_backup/`, explicitly ignored by Git.
The copied ZIP hash was verified. Original run/archive/videos remain accessible
and untouched outside Git. The backup shares the same disk: it protects against
`outputs/` cleanup but is **not a verified independent-storage backup**. Do not
use broad ignored-file cleanup commands on either location. Raw rollout data
and videos are not committed; a Git checkout cannot recover this local backup.

## Completion and runtime-abort investigation

Pinned inference trace: OpenPI `training/config.py:pi05_droid` →
`policies/policy.py:Policy.infer/sample_actions` →
`droid_policy.py:DroidOutputs` (only `actions[..., :8]`) → SAPS seeded websocket
server (timing/sampling/audit metadata) → `OpenPiDroidPolicy.infer` →
`selected_actions` (rows 0–7) → `run_policy_episode`. No `done`, success estimate,
completion probability, dedicated stop or distinguished hold action appears.
Ordinary near-zero predictions have no explicit completion semantics.

All 19 inferences completed. The last returned at 08:33:14.704572 UTC in about
115 ms, with actions/timing/sampling keys. Chunks 14–18 have selected raw arm
L2 means approximately 0.215, 0.303, 0.333, 0.171, **1.301**: the final prediction
does not converge to stationary actions. The final gripper prediction is OPEN
(0.011–0.021); issued hardware commands were OPEN, CLOSE with object detection,
OPEN, all completed. Duplicate intents did not issue extra commands.

The default `unevaluated_outcome` returns CONTINUE. Execution ends on configured
bounds, diagnostic stop, interruption or runtime/safety abort, not automatic
manipulation completion. A custom outcome provider could stop it, but none is
installed by this CLI. Runtime status `success` is software validation; abort
sets the legacy task-outcome enum to ABORT. Human annotation, policy completion,
execution termination and safety/intervention evidence must remain distinct;
simulation outcome metrics are unchanged. Future task-general assessment should
start with human video annotation and uncertainty/occlusion labels. Learned
visual/language-conditioned success or progress estimation requires independent
validation; model completion is usable only if a future interface supports it.

The nested `send` in `src/saps/physical/policy_execution.py` requires callback
receive time **at or after eligibility** and a source ROS stamp **strictly newer
than the previous target's measured-state stamp**. Identical joint positions
can pass if stamps advance. Scheduled steps also abort after one missed
66,666,667 ns period, even with the 30 s observation timeout.

| Event | Monotonic ns |
|---|---:|
| Last ordinary action 5 measured receive / publish | 3875417102525 / 3875419529164 |
| Action 5 applied T4, sequence 187 | 3875420420841 |
| Last measured callback before missing action 6 | 3875469695609 |
| Missing action 6 scheduled / one-period deadline | 3875483062550 / 3875549729217 |
| Next callback, used by abort hold | 3875608357189 |
| Abort hold publish / applied T4, sequence 188 | 3875609845973 / 3875610547776 |

Receive gap: **138.662 ms**; source gap: **134.016 ms**
(1794385995139702794 → 1794385995273718986 ns). The next callback is
125.295 ms after the missing action's schedule. Last source stamp actually used
by an ordinary action: 1794385995086684445 ns. Its measured `dq_ref` includes
about −0.58/−0.82 rad/s at joints 2/7; accepted/applied equilibrium targets are
not proof the target pose was reached. The original failed attempt left no row
or exact exception timestamp, so the raise time is bounded, not directly known.

Feedback resumes and continues through source 1794386001315750150 ns. There
are 30,287 measured-joint records, 30,242 Franka-state records, 3,072 controller
state records, 332 readiness records and 188 sent/forwarded targets. No logged
invalid-evidence, Franka health or readiness violation was found. Controller
application evidence confirms the final policy target and abort hold. A separate
observation subscription's lack of errors does not prove this boundary was fresh.

**Supported:** transient fresh-state unavailability at the executor and continued
execution after apparent completion. **Unsupported:** explicit policy completion
or stationary final behavior as the cause. Publishing, DDS transport, callback
scheduling and host processing remain competing underlying causes; no independent
publisher trace establishes which. A recurrence can abort future execution.
The first-failure `joint_state_wait_failure` diagnostic now records both deadlines
and actual/source/receive times without changing fail-closed behavior.

The **18 terminal holds** are one per full chunk. Chunk 18 (zero-based) applied
six actions before failing action 6, never reaching its terminal hold. The single
abort hold is separate. No timeout threshold was increased or mapped to success.

## Commissioning conclusions and repository cleanup

Full predecessor records remain recoverable at Git commit
`b2e951d6230d8b3d5b10a39fd0b346c48336bb53`. Redundant documents and generated
November 6/9 snapshots are also byte-preserved in
`outputs/physical_c4/final_cleanup_20261111/commissioning_b2e951d.tar.gz`
and the ignored backup directory. They are historical evidence, not current
operating instructions. Selected conclusions:

- November 6 saved `shadow_004`: finite `[15,8]`, first-eight target margin
  0.498576 rad, measured flange condition 8.44149; Pinocchio 3.9.0 FK/Jacobian
  agreed to about 1e-15. Arm-description revision `fac4949828c8b627ccb8593628212f59a8f46d00`
  had a modified group-definition xacro. Snapshot feasibility was not trajectory,
  collision, load or current-state approval. November 9 arm-only comparison
  confirmed flange geometry, not a fingertip TCP requirement.
- November 9 controller/tool capture is at
  `~/franka_ehsan_ros2_ws/c4_runtime_inspection_20261109T161032Z/`.
  Effort-path torque slew exists; enabled manager velocity/effort protection was
  not established by an empty parameter map. Configured tool mass was 0.73 kg,
  external load zero, `F_T_EE` 103.4 mm Z / −45° yaw, `EE_T_K` identity.
  Actual Robotiq/camera assembly inertia/clearance was not established.
  Exact units, transforms, gain assumptions, safety limits and procedures are
  retained in the [operating guide](physical_zed_robotiq.md) and
  [tool/load reference](physical_tool_load_configuration.md).
- Historical manifest SHA-256
  `402b4d66027b056fcc70464c10ffef3a3d809863fa75ba9787a7c6889c4365ec`,
  hardware baseline `4bb6cdc58839dcdd94acbe1633b8f361676a4eb6`, did not match
  the later working tree. No manifest was repinned to bypass qualification.
- Git warnings came from installed ZED share directories, not a false hardware
  commit. SAPS now resolves package-manifest symlinks and outer Git roots,
  records dirty state/revision/diff fingerprints and explicit unavailability,
  and captures stderr. Inspected wrapper/components commit `431dcf4b…`, messages
  `24f8d513…`, wrapper 5.5.0 and SDK 5.5.1 were clean. The separate hardware CLI's
  old provenance helper was not modified.

| Cleanup decision | Scope / reason |
|---|---|
| Retain implementation, profiles and all tests | Active execution/readiness or reusable regression coverage; C4 names reflect the supported interface, not obsolescence. |
| Retain offline target and CAD/inertia tools | Reproducible snapshot/model and complete-input physical-property checks; no generated JSON is a runtime/test dependency. |
| Relocate native probe | `tools/diagnostics/native_limits_probe/`; still useful for offline library safety checks, creates no robot objects. |
| Consolidate documentation | Deployment/procedure/video/safety in `physical_zed_robotiq.md`; outcome/history/investigation here; distinct detailed tool/load reference retained. |
| Remove active generated snapshots / redundant guides | Conclusions/pins retained above, exact bytes archived and historical commits unchanged. Older qualifying simulation/Hand baseline evidence remains untouched. |

The per-file audit covers all 80 migration paths in
`outputs/physical_c4/final_cleanup_20261111/file_audit.tsv`. Cleanup does not
change runtime code, configuration values, policy protocol, calibration,
controller settings, safety thresholds, observations, logging or outcome metrics.
No test coverage is removed. The three existing MP4s contain 209 frames each at
10 fps from 19 sparse camera pairs, 190 held duplicates per stream, no skipped
input observations; use the operating guide's exporter for a new review copy.
The earlier simulation imageio and pinned Isaac `2fac880…` video implementations
were inspected; physical export uses the existing OpenCV stack and explicit
irregular timestamps instead of importing the simulation framework.

Cleanup verification: **81 focused host tests passed**; `make check` discovered
**492 tests**, OK with the same single Docker Git-executable skip (covered on
host), and compilation passed. No tests or assertions were removed. Runtime
imports/CLI help, exact configuration hashes and hardware helper resolution
passed; the relocated native C++ probe built and passed without robot objects.
Offline export from the preserved observations again decoded all three streams
(209 frames each). All 90 retained local Markdown paths resolved; shell syntax
and `git diff --check` passed. Receipts and new derived videos are under the
cleanup output directory. No ROS2 stack edits, live inference, hardware commands
or robot motion were performed. No policy/hardware service was started.
