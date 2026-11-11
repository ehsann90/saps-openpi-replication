# First operator-observed physical task success — 2026-11-11

Run directory: `outputs/physical_c4/c4_dev_long_20261111T083244Z/`.
Run UUID: `0327da03-a196-4180-8a93-e2582901e626`.
Recorded interval: 08:32:44.288927–08:33:23.934783 UTC (09:32–09:33 Oslo).
Task/prompt: **Put the marker in the white cup**.

| Field | Evidence |
|---|---|
| Operator-observed task outcome | Success, reported by operator; first observed success on the migrated infrastructure. |
| Recorded runner status | `failed` (unchanged). |
| Recorded task outcome | `abort` (legacy runtime-abort classification, unchanged). |
| Execution termination | `runtime_abort`, `TimeoutError: no_new_measured_q_for_step`, phase `actions`. |
| Policy completion | No explicit completion interface found. |
| Execution counts | 150 policy actions applied, 3 gripper commands, 18 confirmed terminal holds; 19 main requests, 18 completed chunks. |

This is development/pilot evidence, not a formal success-rate estimate. No
success timestamp was recorded. Task outcome and runtime termination are
separate observations; neither replaces the original runner classification.

## Preservation and inventory

The original directory and ZIP were read only. All **138 files** match the
archive byte for byte, with no missing or extra files. The separate review root
`outputs/physical_c4/review_20261111/` contains `inventory.json` (sizes/hashes),
`SHA256SUMS` (all source files and archive), `archive_verification.json`,
`analysis.json`, and `preservation_analysis.py` (the exact analysis performed).
Checksums refer to original bytes, not regenerated records. No raw rollout,
images, videos or bulk timing data are committed.

| Evidence | SHA-256 |
|---|---|
| `c4_dev_long_20261111T083244Z.zip` | `5008f11a5d5c82324b95b76c68d62c755a9e2da1fa2053ae76bbab00a8fdcdc9` |
| `run.json` | `bcf8c3de705a24e35fa3e72dea1220c3ab4292433da31bcd4c6c38e52b8289e6` |
| `timing.jsonl` | `c4836db2c669e88a5561902dbf6cb55feaec426e1da112d833e576217ff45f51` |
| `provenance.json` | `d1ba3792a761845d7e8c457672a3bc06a81f7bf07013337d11deb42c484d0422` |
| `effective_hardware.json` | `96dae9e75fec402c22bb26dc11b1529aa01f66dd0f8605575bd1174688e350f2` |
| `request_0018/actions.npz` | `035ce11e719b7b1914967826adff8ae913be68c5434aa004986c4b3d596ecc3b` |

Inventory: 19 top-level request directories, each with native RGB camera pairs,
policy observation NPZ, action NPZ, request/response JSON and request timing.
Request zero also includes model-input audit. Startup includes a discarded warm-up request and a saved fresh post-warm-up
observation; neither is included in the 19-frame video sequence. Other files
include start/run, source/runtime provenance, startup/readiness,
effective hardware, robot-description URDF, joint limits and timing JSONL.
There was no original MP4, continuous camera recording or independent terminal
transcript in this directory. Exceptions, gripper feedback, callback errors and
termination are recorded in JSON. The inventory includes nested startup files.

Hardware: FR3, Robotiq 2F-85, ZED Mini wrist serial 15330503, ZED 2i exterior
serial 32919961, BGRA8 messages decoded to RGB. Native images are 1280×720;
policy client images are 320×180 with existing preprocessing. Effective
camera, gripper calibration, URDF and controller evidence remain in the run.
The C4 development run used episode seed 20260827, max replans/chunks 1000,
singularity threshold 60, observation timeout 30 s, policy timeout 120 s,
hold-application confirmation timeout 2 s. These are recorded run settings,
not changes to defaults or recommended future bounds.

| Recorded clean source | Revision |
|---|---|
| SAPS (`migrate/franka-robotiq`) | `f8b62482a35bc8d5ee328651ca132bef2555efff` |
| Hardware stack (`main`) | `3dc23e39f5470ef929c5bfbb0f476b50d54fff03` |
| IGD control | `438e4ae3145944042033395940848bf107cb9965` |
| OpenPI | `15a9616a00943ada6c20a0f158e3adb39df2ccac` |

The supplied published hardware revision was `e34c74c1c5f14d1fdc8a5e862437db5d93de4762`.
Both the preserved run and the inspected local checkout instead identify
`3dc23e39…`. Do not substitute the supplied revision into the historical record.
The hardware repository was not modified. Strict source-manifest mismatch is
still a qualification blocker.

Policy identity: server handshake confirms `pi05_droid`, checkpoint
`gs://openpi-assets/checkpoints/pi05_droid`, horizon 15 and latent width 32.
The checkpoint URI/handshake is available; an independent complete checkpoint
weight digest is not. Saved responses have eight action dimensions.

## Completion interface and final behavior

Actual pinned inference path:

1. OpenPI `training/config.py` selects `pi05_droid`; `policies/policy.py:Policy.infer`
   calls `model.sample_actions`, applies output transforms and adds timing.
2. `policies/droid_policy.py:DroidOutputs` returns only `actions[..., :8]`.
3. SAPS `scripts/serve_seeded_policy.py` adds seed/audit and server timing;
   neither it nor the pinned websocket server adds task completion.
4. `src/saps/policies/openpi_droid.py:OpenPiDroidPolicy.infer` validates actions,
   sampling identity and timing; all response keys are retained as evidence.
5. `streaming_playback.py:selected_actions` selects actions 0–7 from `[15,8]`.
   `policy_execution.py:run_policy_episode` maps arm values against fresh measured
   joints, executes eight actions, installs a terminal hold, audits delivery and
   requests another chunk after outcome/readiness/gripper barriers.

**Classification: No explicit completion interface found.** No `done`, success
estimate, completion probability, stop token/action, or distinguished policy hold
exists in this deployed interface. Ordinary near-zero arm actions are possible
but have no completion semantics. The runtime constructs its own measured-state
holds independently of model predictions. This conclusion concerns this pin,
checkpoint interface and deployment, not all VLA models.

All 19 main requests completed inference. The last request returned at
08:33:14.704572 UTC in about 115 ms, with keys `actions`, `policy_timing`,
`saps_sampling`, `server_timing`; request zero additionally carries audit data.
No explicit completion field appears. The default `unevaluated_outcome` provider
returns CONTINUE; the physical CLI has no installed success detector. A rollout
stops on configured chunk/replan bounds, diagnostic inference-stop bounds,
external interruption, or a runtime/safety abort. A separately supplied task
outcome provider can also stop it; the deployed default does not detect success.

Predicted arm-value L2 means over the eight selected actions for chunks 14–18
are approximately **0.215, 0.303, 0.333, 0.171, 1.301**. These are raw
normalized policy values, not measured motion or end-effector displacement.
The final chunk therefore does not converge to repeated stationary predictions.
It requests OPEN throughout (gripper values 0.011–0.021). Earlier issued
commands were OPEN, CLOSE (reported object detection), OPEN, all completed;
duplicate intents did not issue extra hardware commands.

Accepted targets remain distinct from measured state: final action index 5
was forwarded, accepted and applied as controller sequence 187; its measured
`dq_ref` includes roughly -0.58 rad/s at joint 2 and -0.82 rad/s at joint 7.
This is measured motion at the action anchor, not proof that the full target
was reached. The controller continued receiving ordinary targets until the
abort, followed by sequence 188 for the abort hold. No next inference was
attempted after the abort.

The inspected final exterior observation (`request_0018`, source
1794385994.4127998 s) shows the marker at/in the white cup with the open gripper
withdrawn. It supports the operator's reported manipulation outcome before the
failed action deadline. It does not establish an exact release/success time,
settled-object retention or a quantitative task metric. Sparse images cannot
reconstruct every intervening movement.

## Timeout reconstruction and competing hypotheses

The exception is raised by the nested `send` function in
`src/saps/physical/policy_execution.py`. It requires both:

- `state.receive_monotonic_ns >= eligible_monotonic_ns`;
- `state.source_stamp_ns > previous_row.q_ref_source_stamp_ns`.

These are callback receive time and ROS source stamp, not joint displacement or
a policy sequence/done signal. Even identical joint positions pass if stamps
advance. The observation bound is 30 s, but scheduled policy/terminal steps
also abort after one full **66,666,667 ns** period is missed. The shorter bound
applies here. There is no timeout-threshold change.

| Event | Monotonic nanoseconds |
|---|---:|
| Last executed action 5, measured receive | 3875417102525 |
| Action 5 published / applied T4 | 3875419529164 / 3875420420841 |
| Last measured callback before action 6 deadline | 3875469695609 |
| Unexecuted action 6 scheduled | 3875483062550 |
| One-period deadline | 3875549729217 |
| Next measured callback (used by abort hold) | 3875608357189 |
| Abort hold published / applied T4 | 3875609845973 / 3875610547776 |

The pre-gap source stamp is 1794385995139702794 ns; the next is
1794385995273718986 ns. Receive gap is **138.662 ms**, source gap **134.016 ms**.
The next measured callback arrives **125.295 ms** after action 6's scheduled
time, beyond its allowed period. The last source stamp actually used for an
ordinary action is 1794385995086684445 ns. The failed attempt itself left no
row/exact exception timestamp; timing bounds are reconstructed from the schedule
and adjacent evidence. This is not a claim to know the precise raise time.

Joint feedback subsequently continues through source stamp
1794386001315750150 ns (about six seconds after the abort hold). The timing file
has 30,287 measured-joint records, 30,242 Franka-state records, 3,072 controller
state records, 332 readiness records and 188 sent/forwarded targets. No logged
invalid-evidence, Franka health or readiness violation was found. Controller
application evidence confirms both final policy target and abort hold. No
independent publisher-side trace or hardware terminal log establishes whether
the gap originated at publishing, DDS transport, callback scheduling, or host
processing. The final observation collector also reports no callback/executor
errors; that separate subscription does not prove this boundary stayed fresh.

- **H1 explicit completion:** unsupported by interface/source and response keys.
- **H2 implicit stationary completion behavior:** not supported by final arm
  predictions or measured motion. Gripper stabilization alone is insufficient.
- **H3 joint-state availability failure:** confirmed at the executor's freshness
  boundary; transient gap, not established permanent publishing failure.
- **H4 continued execution after apparent completion:** consistent with operator
  report, final camera evidence, default CONTINUE and subsequent action/abort.
  Exact completion-to-abort interval remains unannotated.

The 18 terminal holds are one per fully completed chunk, not 18 success retries.
Chunk 18 (zero-based) executes six actions and aborts before action 6; it never
reaches its terminal hold. The single applied abort hold is separate.

Reliability remains unresolved: a repeat of this gap can abort future runs even
when inference and controller delivery work. Retain fail-closed behavior. The
minimal correction now is instrumentation, not a speculative controller fix:
`joint_state_wait_failure` records the first failed wait's action, actual
observation time, source/receive stamps and both deadlines; abort-hold failures
do not overwrite it. This adds a failure-diagnostic field; provenance also gains repository-root,
source-directory and availability metadata. Existing task/outcome fields are
unchanged. Action mapping, timing, bounds, outcome classification and historical
evaluation metrics remain unchanged. A subsequent separately authorized run should
correlate this with independent publisher/DDS receive tracing and executor/host
scheduling evidence. No robot motion was run during this investigation.

## Outcome semantics and future evaluation

The physical episode already has a task-outcome enum/provider and a separate
termination reason, but default CONTINUE means **not automatically evaluated**;
legacy runtime abort overwrites `task_outcome` with ABORT. Top-level `success`
is software/runtime validation, not manipulation success. Intervention and safety
are represented by mode/input/runtime evidence, not a single universal outcome
schema. The simulation task-success metrics have their own established meaning.

The smallest correction is this documented distinction and a separate evidence
annotation with `operator_observed_task_outcome` and `execution_termination`;
no historical metric or enum was changed. Policy completion remains absent.
Future research should start with human video annotation using explicit unknown
and occlusion categories. Language-conditioned visual success assessment or
learned task-progress estimation would require held-out validation against human
labels, uncertainty/false-positive assessment and temporal persistence criteria.
Model-generated completion is usable only if a future deployed interface
explicitly supports and validates it. These approaches are not implemented.
Do not use small actions, gripper state or timeout as task-general success rules.

## Provenance and cleanup findings

Warnings originated in the external camera helper's `identity(share)` calls for
installed `zed_wrapper`, `zed_components`, `zed_msgs` directories. Their recorded
`revision` is null; stderr escaped despite caught Git errors. It was not an
incorrect outer hardware checkout in this run. SAPS now gathers the same
camera versions, SDK/header and helper hashes locally, resolves `package.xml`
symlinks to actual source directories, then resolves outer Git roots. Checkout
commit/dirty state is explicit; binary-only package directories record unavailable
Git identity without stderr. Source capture hashes paths relative to the resolved
root. Strict manifest/ancestry/policy pins are unchanged.

Read-only inspection verified clean actual identities: hardware `3dc23e39…`,
wrapper/components `431dcf4b1ea39caf894ca6ea294437437dbdb034`, ZED messages
`24f8d513ac8d4145a3bb6d72258f7e030443c1d8`, wrapper 5.5.0, SDK 5.5.1.
The independent hardware CLI's own provenance helper is outside this task;
its standalone commands may still emit the old warnings until fixed there.

Removed SAPS camera driver launcher/module and its two launcher-only tests,
serial/camera Make launch wrappers, and unused `requirements-robotiq.txt`.
Modbus and duplicate Robotiq ROS implementations had already moved out in
migration commits; no remaining copy was found. Retained `robotiq_hand.py`
(policy-to-ROS adapter), DROID mapping/tests, C4 checks, simulation baselines,
RealSense/Franka Hand observation/configuration support and frozen C1/G1B
execution/diagnostic paths: they still have callers and scientific baseline
coverage, not duplicate current hardware ownership. Legacy `fr3_lab_stack`
identifiers in pins, protocol records and controller/message types are retained
for compatibility. Historical commissioning content is archived with an explicit
label; operational deployment docs use only the current hardware launcher.

See [video convention](physical_video.md) for the exported evidence and its
sparse-recording limits. Validation and local branch integration are reported
with the final review; no raw evidence or remote publication is part of commits.

Existing video implementations were inspected: LIBERO episode runners use
`imageio.mimwrite`; historical Isaac commit
`2fac8807619bd9a1595a2adc0d58fecc6909933e` adds
`sim/src/isaac_fr3/policy_video.py` and an offline exporter. Its timeline assumes
fixed simulation cadence and its encoder requires external ffmpeg/ffprobe.
The physical exporter reuses the existing saved-observation convention and
NumPy/OpenCV stack; it does not import the simulation framework or introduce a
video dependency. Physical timestamps need explicit irregular-frame resampling.

Verification before integration: 80 focused host tests passed; `make check`
passed 491 Docker tests with one skip (Git executable absent in that image),
and compile checks passed. The real nested-checkout test passed on the host;
mock Git-root/installed-package tests also passed in Docker. Offline export
and full decoding passed for all three videos, and representative final images
were visually inspected. `bash -n tools/activate_physical_env.sh`, `make help`
and `git diff --check` passed. No hardware/interactive smoke test, live inference
request or actuation was attempted. No service was started. Independent
publisher/DDS timing and physical repeatability remain unverified.
