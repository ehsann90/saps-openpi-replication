# Live π0.5-DROID P0 shadow inference

P0 collects a fresh physical observation, immediately requests a native
`[15, 8]` policy chunk, and saves the paired evidence. It executes **zero**
actions. Physical acceptance requires an operator-supervised run; automated
tests alone do not establish acceptance or policy performance.

## Current interface and historical provenance

`configs/physical_pi05_fr3.json` is the current P0 contract. The historical
`configs/physical_m3.json`, M3 capture/replay commands, and saved artifacts
retain their original topics, profiles, and acceptance meaning.

| Live source | Identity | Canonical request |
|---|---|---|
| `/camera/external_camera/color/image_raw` | external serial `244222076317` | `observation/exterior_image_1_left` |
| `/camera/wrist_camera/color/image_raw` | wrist serial `342222073510` | `observation/wrist_image_left` |
| `/franka/joint_states` | `fr3_joint1` … `fr3_joint7` | `observation/joint_position` |
| `/franka_gripper/joint_states` | Franka Hand fingers | `observation/gripper_position` |
| Explicit CLI instruction | verbatim string | `prompt` |

Camera serial-role assignments are frozen. Topics and camera node names are
explicit configuration fields, verified against the running graph. Exactly
one publisher is required per source. Camera topic publishers must belong to
the configured camera nodes. Read-only `ros2 param get --hide-type <camera>
serial_no` queries verify their serial selection, including RealSense's
leading underscore convention. The same graph endpoint IDs and camera serials
are checked at completion. Image messages contain no serial; identity evidence
comes from the driver selection parameter combined with publisher binding.
It is not an independent optical identification of the device.

Base/TCP provenance is `fr3_link0` / `fr3_hand_tcp`. P0 needs no TF, Servo,
SpaceMouse, Cartesian projection, or calibration. Joint positions are ordered
by names into `(7,) float32`, never by incoming array order. Both accepted
finger pairs remain supported: `_finger_joint1/2` and `fr3_finger_joint1/2`.
The adapter orders finger positions canonically and computes

```text
width_m = finger_joint1 + finger_joint2
unclipped_closure = 1 - width_m / 0.08
DROID_closure = clip(unclipped_closure, 0, 1)
```

Closure is `(1,) float32`, with zero open and one closed. Logs retain incoming
names, canonical position order, both positions, width, maximum width,
unclipped/clipped closure, and whether clipping occurred.

## Three image boundaries

1. Native ROS: both expected streams are `1280×720 RGB8 @ 30 Hz`. The existing
   decoder explicitly supports RGB/BGR and row padding; P0 rejects a runtime
   profile differing from its configured RGB8 baseline. Decoded native RGB
   arrays are retained separately from the canonical request.
2. Client: reuse `preprocess_policy_rgb`: centre crop to 16:9 if needed, then
   OpenCV `INTER_AREA` downsize (`INTER_LINEAR` when upsizing) to
   `(180, 320, 3) uint8 RGB`. Current native input records
   `no crop (native 16:9); resize 1280x720->320x180 with INTER_AREA`.
3. OpenPI: the pinned policy owns its existing full transform pipeline.
   There is no new client normalization or model resize implementation.

Confirmed in OpenPI `15a9616a00943ada6c20a0f158e3adb39df2ccac`:

- `DroidInputs(PI05)` concatenates seven joints and gripper into eight state
  coordinates; external maps to `base_0_rgb`, wrist to `left_wrist_0_rgb`, and
  a zero image supplies `right_wrist_0_rgb`. Masks are true, true, false.
- `create_trained_policy` loads normalization statistics from the checkpoint's
  `assets/droid`. PI05 uses quantile normalization; P0 does not reproduce it.
- `ModelTransformFactory` applies OpenPI padded `ResizeImages(224, 224)`,
  PaliGemma prompt tokenization with discrete normalized state, and state
  padding to 32. PI05's maximum token length is 200. The exact submitted
  prompt is logged; OpenPI's own tokenizer cleans whitespace/underscores and
  formats its task/state text as defined upstream.
- `Policy.infer` adds batch dimension one and converts to JAX arrays.
  `Observation.from_dict` converts uint8 images to float32 `[-1, 1]`.
  The actual sampler receives `(1,224,224,3)` images, `(1,)` Boolean masks,
  `(1,32)` state, and `(1,200)` token arrays/mask.
- Output transforms perform checkpoint unnormalization and `DroidOutputs`
  truncation to eight coordinates. Configuration `pi05_droid` has horizon 15.
  The upstream DROID example's old `(10,8)` assertion is not used.

For the first seeded request, `audit_model_input=true` opts into a project
server hook. It temporarily wraps the policy instance's existing
`_input_transform` and `_sample_actions`, copies their actual outputs/inputs,
then calls the original sampler once with identical RNG/noise arguments.
Both hooks are restored in `finally`. The server's synchronous inference
section serializes requests; hooks cannot overlap another inference. Nothing
under `third_party/openpi` is edited, and no normalization/transform is replayed.
The client validates request hashes, prompt, model keys/shapes/dtypes/masks,
state/tokens, and the masked placeholder, then saves NPZ arrays plus visual
PNG copies of all three transformed images. The mapping is additionally
covered by executing the pinned `DroidInputs` class in an isolated unit test.

Auditing adds copies and device-to-host synchronization on the first request;
that request's latency must be analyzed separately. Later requests have no
audit overhead. Upstream `policy_timing.infer_ms` is retained as reported: its
JAX timing brackets sampling dispatch and does not explicitly block for GPU
completion. Server and client timing include wider boundaries; none is
renamed into an independently measured GPU execution time.

## Finite loop and zero actuation

Only a complete observation passing the existing source-age/skew rules can
trigger inference. Defaults are maximum age `0.5 s` and cross-source skew
`0.25 s`; these are diagnostic gates, not execution safety limits. A second
request requires **both** camera ROS stamps to be strictly greater than the
previous accepted pair. Repeated, partially advanced, and regressed pairs are
rejected. Invalid callback state prevents use of previously cached data.
Observation age is checked again immediately before calling the policy.

The ROS loop spins synchronously between requests with best-effort depth-one
subscriptions, dropping backlog while inference runs. It records measured
callback counts and source/receive rates. These are rates of consumed samples,
not uninterrupted 30-Hz throughput measurements during blocking inference.
Advancing source stamps establish continuity across requests; the run does
not claim hardware synchronization. The prior approximately 19-ms camera offset
is historical context, not a substituted live measurement.

The default is ten requests, at most 30 seconds waiting for each fresh pair,
and a 120-second connection/response timeout. There is one outstanding policy
request, no retries or action execution between requests, and monotonically
increasing replan indices `0..N-1`. Episode seed defaults to the existing
`DROID_POLICY_SEED=20260827`. Protocol v1, seed, index, and noise SHA-256 are
preserved and checked. This is not the latency-aware scheduler or an open-loop
controller. Transport errors and Ctrl+C terminate with diagnostic evidence.

`SubscriptionBoundary` exposes only `create_subscription` and `get_clock` to
the existing collector. The runtime contains no robot command publishers,
robot service clients, action clients, gripper controllers, or execution
callbacks. ROS's own diagnostic infrastructure (such as `/parameter_events`)
is distinct from robot command topics. Camera serial preflight alone uses
read-only camera parameter services through the ROS CLI; it never calls robot
services. Run records explicitly contain:

```json
{
  "published_robot_command_topics": [],
  "called_robot_services": [],
  "called_robot_actions": [],
  "policy_actions_executed": 0,
  "gripper_commands_issued": 0,
  "robot_command_action_clients": 0
}
```

These are properties of this process's implementation, not a claim that other
processes on the lab ROS graph cannot issue commands. The unit suite checks
the subscription capability boundary and absence of command creation and
projection calls in the P0 runtime. The running node's owned publishers and
service clients are also inspected and recorded at startup and completion;
anything other than `/parameter_events` publication or any service client
causes failure. These checks are separate from the camera CLI preflight.

## Operator-supervised run

The policy server continues using the validated Docker image. The subscriber
runs with ROS Jazzy's system Python ABI on the host. The inspected system
Python has NumPy/OpenCV but lacks the OpenPI networking dependencies. Prepare
an isolated client environment once; the versions below are already pinned in
OpenPI's `uv.lock`, not dependency upgrades:

```bash
/usr/bin/python3 -m venv --system-site-packages .venv-physical
.venv-physical/bin/python -m pip install \
  websockets==15.0.1 msgpack==1.1.0
```

The Make target supplies the pinned `openpi-client` source through PYTHONPATH;
no model package, JAX, checkpoint, or GPU is needed in this client environment.
No global Python packages or Docker images need changing. `PHYSICAL_PYTHON`
can select an existing compatible ROS/client environment. This setup is an
operator prerequisite and is not installed automatically by the run target.

Terminal 1: keep the already-working FR3/MoveIt bringup. No change to its
procedure and no extra teleoperation qualification is required for P0.
Servo need not be active.

Terminal 2: use the existing cameras or, if they are not already running,
source the lab workspace and start its validated launch:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash
ros2 launch fr3_lab_stack dual_realsense.launch.py
```

Terminal 3: start the project server with the new diagnostic hook available:

```bash
make droid-policy-server
```

The handshake must advertise `pi05_droid`,
`gs://openpi-assets/checkpoints/pi05_droid`, horizon 15, and model audit schema 1
with the frozen OpenPI commit. The P0 client enforces these conditions; an
older already-running server without the hook must be restarted by the
operator. The unchanged `make droid-policy-server` target supplies the model
identity explicitly.

Terminal 4, from this repository:

```bash
make physical-pi05-shadow \
  PHYSICAL_RUN_ID="pi05_shadow_$(date -u +%Y%m%dT%H%M%SZ)" \
  PHYSICAL_PROMPT="pick up the object" \
  PHYSICAL_REQUESTS=10
```

Run identity and prompt are required. The directory is created exclusively;
reusing any existing run fails before connecting to ROS or the policy server.
The direct script also accepts `--host`, `--port`, `--observation-timeout`,
`--policy-timeout`, `--lab-stack-dir`, and `--igd-control-dir`. External
checkout paths are optional provenance: missing paths are recorded, and dirty
repositories are never cleaned. No hardware run is part of automated tests.
After an approved run, stop a server started for it with `make policy-stop`
unless it is intentionally being kept running.

## Output schema (version 1)

Every run is under `outputs/physical_pi05_droid_p0/<unique-run>/`:

| Artifact | Contents |
|---|---|
| `start.json` | Start UTC, run ID, full config, prompt, count, seed, timeouts, runtime, repository/OpenPI/lab/control provenance, zero-actuation contract |
| `run.json` | Same run contract, final UTC/duration, completed count, server handshake, initial/final source graph and camera serial evidence, rates/callback errors, rejected-pair checks, termination reason/error |
| `request_NNNN/request.json` | Request/chunk/replan identity, seed, exact prompt, canonical array shapes/dtypes/hashes, observation timing, both camera records, robot/gripper transforms, pair stamps, continuity counts/rates, artifact hashes |
| `request_NNNN/observation.npz` | Exactly the canonical five DROID keys, including scalar Unicode prompt; load with `allow_pickle=False` |
| `request_NNNN/native_rgb.npz` | Decoded native RGB arrays `wrist` and `exterior`, before client preprocessing |
| `request_NNNN/request_timing.json` | Request start UTC/ROS/monotonic, oldest-source age, call end UTC/ROS/monotonic; also written for a failed policy call |
| `request_NNNN/response.json` | Actual request/response times, oldest-source response age, client round trip, original model/server timing, sampling protocol/seed/index/noise hash, response keys, raw action shape/dtype/hash/finite status, per-dimension min/max/mean/std, artifact hash |
| `request_NNNN/actions.npz` | Complete unmodified native action chunk as `actions`, preserving original floating dtype and values outside unit range |
| `request_0000/model_audit.json` | Actual boundary, canonical hashes/prompt, image mapping/masks, transformed/model array shapes/dtypes/hashes, token count, artifact references |
| `request_0000/model_audit/model_input.npz` | `transformed/<image-name>`, `model/<image-name>`, `mask/<image-name>`, `state`, `tokenized_prompt`, `tokenized_prompt_mask` |
| `request_0000/model_audit/*.png` | Visual copies of the actual post-transform uint8 images, including the masked placeholder |

Timing includes assembly ROS/monotonic and assembly-record UTC, all four source
ROS stamps and source/receive ages, explicit camera/joint/gripper receive
stamps, oldest/newest source and cross-source skew. Request/response ages use
the **oldest** source and the same ROS clock, not Unix-minus-ROS subtraction.
No action is rescaled, clipped, thresholded, or Cartesian-transformed. Every
chunk records returned horizon 15, future reference open-loop horizon 8, and
executed actions zero. The reference value 8 is not executed in P0.

Artifacts are written per request, so previous completed records survive a
later failure. `run.json` finalizes failures after allocation, including
preflight errors. If initial provenance itself fails, `start.json` can be
absent while `run.json` records the failure. A policy call that fails native
response validation has request/timing evidence but no accepted action
artifact. Unexpected finite horizons are saved for diagnostics and rejected.
Incomplete/failed runs are never labeled as physical acceptance or formal
experimental results.

## Physical P0 acceptance — 2026-09-09

An operator-supervised one-request smoke run
(`pi05_shadow_20260909T083241Z`) and ten-request run
(`pi05_shadow_20260909T083508Z`) both completed with
`request_count_reached`. The accepted run used the configured FR3 arm and
gripper topics and the fixed wrist/external serial-to-topic assignments. Its
first-request audit and visual inspection confirmed external → `base_0_rgb`,
wrist → `left_wrist_0_rgb`, and the black, masked right-wrist placeholder.
All ten policy responses were finite native `[15,8]` chunks, with zero policy
actions or gripper commands executed and zero robot-command publishers or
action clients created by the P0 process.

Warm-server client round trips in this short run were approximately
115–133 ms (request 0: 126.5 ms client, 118.1 ms server, 96.6 ms policy).
These requests are diagnostic acceptance evidence, not a formal latency or
task-performance benchmark. P0 is accepted as the live physical observation
and inference boundary.

The run rejected non-advancing observations and one stale external-camera
sample; separate `ros2 topic hz` inspection also showed occasional external
delivery gaps. Rejection worked as designed and ten valid requests completed,
so this does not invalidate P0. Camera delivery continuity remains an explicit
execution-stage evaluation item. The P0 callback rates are rates consumed by
the synchronous loop, which does not spin ROS callbacks while inference is
blocking; they are not raw camera publishing rates.

## Acceptance and remaining P1 work

Review `run.json` for `request_count_reached`, the intended completed count,
correct identities, unchanged publisher endpoints, and zero actuation. Review
both native/client images and first-request transformed PNGs for correct
physical roles. Verify all chunks are finite `[15,8]`, both camera source
timestamps advance throughout the run, timing/audit evidence is complete, and
no source substitution or missing field occurred. A timestamp regression,
stale input, identity failure, or missing audit must not be accepted as a pass.

Direct autonomous controller selection, exact execution semantics/cadence,
clipping, scheduling, limits, action-age rules, gripper execution, and physical
safety supervision remain P1 work. Direct execution may remain in joint space.
Later SAPS must reuse the established joint-to-Cartesian/TCP mapping and
`0.30 m/rad` cosine characteristic length. P0 does not redesign either.

Implementation inspection used research branch `experiment/physical-fr3-saps`
at `459df132ab83451a57944c8dfbb8c1df34ddafa2` (initially clean), OpenPI at the
frozen pin, `fr3_lab_stack` main at
`ff12254b4f85a939d453200355c96b5bc00102c1` (clean), and `igd_fr3_control`
`commissioning/fr3-spacemouse` at
`438e4ae3145944042033395940848bf107cb9965` (unrelated local edits present).
Lab launch sources agree with the new config. M3's older camera topics and
profiles remain historical. The infrastructure README still mentions a pending
15-minute teleoperation qualification; that is not a prerequisite for this
non-actuating P0 task. The actual live ROS graph and hardware/model behavior
remain to be verified during the supervised run.

## Implementation and automated verification report

| File | Purpose of change |
|---|---|
| `configs/physical_pi05_fr3.json` | New current interface/provenance contract; preserves historical M3 config |
| `src/saps/physical/shadow_config.py` | Parse and enforce frozen policy, image, state, and serial-role semantics |
| `src/saps/physical/live_shadow.py` | Reusable finite observation/inference loop, advancing-pair gate, strict schema checks, per-request logging |
| `src/saps/physical/shadow_ros.py` | Subscriber capability boundary, graph/serial/interface preflight, provenance, exclusive run allocation and failure cleanup |
| `src/saps/physical/shadow_audit.py` | Validate and persist actual model-boundary evidence and PNG previews |
| `src/saps/physical/live_observation.py` | Optional immutable native RGB retention; old default and preprocessing are unchanged |
| `src/saps/physical/ros_observation.py` | Pass optional native-image retention through the existing collector |
| `src/saps/policies/model_input_audit.py` | Temporarily tap actual OpenPI transform/sampler calls without replay or RNG changes |
| `src/saps/policies/bounded_websocket.py` | Finite policy connection/receive transport using pinned OpenPI msgpack encoding |
| `src/saps/policies/openpi_droid.py` | Optional seeded audit request and response field; existing defaults preserved |
| `scripts/serve_seeded_policy.py` | DROID-only audit capability/pin/runtime handshake and opt-in hook dispatch |
| `scripts/physical_live_shadow_inference.py` | Thin CLI for the reusable P0 runtime |
| `Makefile` | Explicit finite `physical-pi05-shadow` target and help entry |
| `.gitignore` | Exclude the optional isolated physical client environment |
| `tests/unit/test_physical_live_shadow.py` | Config, native RGB, pair gate, loop/logging, audit, transport timeout, graph/interface and failure regression tests |
| `docs/physical_pi05_p0.md` | Current protocol, operator command, output schema, acceptance limits, implementation report |
| `docs/physical_pi05_droid.md` | Link the historical M1 document to current P0 instructions |
| `docs/testing.md` | Distinguish automated P0 coverage from supervised physical acceptance |

Exact verification commands executed:

```bash
PYTHONPATH=src:tests/unit /usr/bin/python3 -m unittest \
  test_physical_live_shadow test_physical_live_observation test_openpi_droid
make unit-test
make compile
make check
make -n physical-pi05-shadow PHYSICAL_RUN_ID=review_only \
  PHYSICAL_PROMPT='pick up the object'
git diff --check
```

Final focused verification passed **52 tests**. The standalone Docker unit
suite passed **234 tests** before the last three failure/transport checks were
added; final `make check` passed **237 tests** and compilation. Standalone
`make compile` also passed. Make command wiring was inspected with `make -n`;
it did not run hardware. `git diff --check` passed.

The initial sandboxed `make unit-test` could not access `/var/run/docker.sock`;
the required command was rerun with the existing Docker permission. During
implementation, the new source-mapping test exposed Python 3.8 incompatibilities
with upstream `match` and `zip(strict=True)`. The test now extracts and executes
the original PI05 branch, with an equivalent length-checked zip for the older
test interpreter. Those failures were caused by the new test and were fixed;
no existing test was disabled or suppressed. Robosuite's existing private-macro
warnings remained visible and did not fail checks.

No live FR3/camera run, checkpoint inference, GPU model audit, or live ROS graph
inspection was performed automatically. The host client environment setup is
documented but not installed by this change. Actual checkpoint transforms and
hardware acceptance remain unverified until the finite supervised run;
unit fixtures are not physical evidence. In particular, timing and camera
identity queries must be reviewed from that run, not inferred from the local
launch files.

OpenPI dependency files, `fr3_lab_stack`, `igd_fr3_control`, historical M3
configuration/artifacts, simulation evaluation, action scaling/normalization,
and the established FR3 Cartesian mapping were not changed. The new P0 output
schema and camera profiles identify a separate diagnostic run family and must
not be relabeled as historical M3 or formal evaluation data. No commit, push,
merge, or pull request was created.
