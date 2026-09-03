# Physical M1: Offline π0.5-DROID

## Scope and status

This completed milestone establishes an offline-only path from genuine DROID
records to the pinned OpenPI `pi05_droid` checkpoint. It does not contain or
exercise an FR3 adapter, ROS, MoveIt Servo, SpaceMouse input, live cameras,
Jacobian conversion, arbitration, gripper execution, or any robot command.

The LIBERO policy client and seven-dimensional simulation action path are not
used or changed. Native DROID actions remain eight-dimensional and are only
inspected and logged.

## Reproduce the sample

The official full DROID RLDS release is approximately 1.8 TB, the official
`droid_100` debugging set is approximately 2 GB, and OpenPI's documented raw
30-episode subset is approximately 1.6 GB. None is required for this milestone.

Prepare the selected object-level subset:

```bash
make droid-sample
```

This reads [`configs/droid_m1_sample.json`](../configs/droid_m1_sample.json),
downloads exactly four public objects from the official `gresearch` bucket,
and verifies their GCS size and MD5 fields before extraction:

| Object role | Bytes |
|---|---:|
| Episode metadata | 1,754 |
| Low-dimensional HDF5 trajectory | 1,219,948 |
| Wrist RGB MP4 | 4,169,356 |
| Exterior-1 RGB MP4 | 8,479,227 |
| Total retained source data | 13,870,285 |

The tool also streams the official aggregated annotation index to verify the
selected language instruction. It does not retain that approximately 12 MB
index. Generated raw files and the extracted NumPy bundle are placed below
`data/droid_m1/`, which is excluded from Git.

Selected source identity:

```text
dataset: DROID raw 1.0.1
episode: IRIS+7dfa2da3+2023-12-04-15h-44m-25s
steps: 0, 76, 152
prompt: Remove the glass lid from the silver pot
```

For each step, the bundle contains exterior-1 and wrist RGB, seven observed
joint positions, observed gripper position, and a 15-step recorded-action
reference. The reference uses the same end-of-episode repetition rule as the
pinned OpenPI DROID loader. It is logged only for representation and scale
inspection; no prediction error or policy-quality metric is computed.

If all raw files are already present, require a network-free rebuild with:

```bash
docker run --rm \
  -v "$PWD":/workspace \
  -w /workspace \
  --entrypoint /bin/bash \
  openpi_server:latest -lc \
  'source /.venv/bin/activate && python \
   tools/datasets/prepare_droid_m1_sample.py --skip-download \
   --skip-annotation-verification --force-rebuild'
```

`--force-rebuild` replaces only the derived bundle. Raw files with a size or
hash mismatch are never overwritten silently.

## Run offline inference

Start the same deterministic OpenPI server implementation used by the
simulation baseline, but select the DROID configuration and checkpoint:

```bash
make droid-policy-server
```

In a second terminal, run three samples twice with identical seeded sampling
state:

```bash
make droid-inference
```

Stop the server afterward:

```bash
make policy-stop
```

`DROID_NUM_SAMPLES`, `DROID_REPEAT_COUNT`, `DROID_POLICY_SEED`,
`DROID_OUTPUT`, and `DROID_RUN_ID` are Make overrides. Every run must use a new
output directory; the diagnostic refuses to overwrite an existing identity.
The checkpoint is downloaded by pinned OpenPI into the existing OpenPI asset
cache on first use.

The output `run.json` contains:

- repository, submodule, checkpoint, policy, dataset, sample, server, and
  container provenance;
- exact policy-input keys, image and state contracts, and state values;
- full returned and recorded-reference chunks, first actions, per-dimension
  summaries, and gripper sequences;
- preprocessing, client round-trip, server, and model timing;
- seeded noise metadata and exact repeatability comparisons; and
- one aggregate empirical action contract for the run.

This is a small diagnostic, not a Gate-1 benchmark or latency study.

## Validated empirical contract

The final 2026-08-27 diagnostic is stored locally at:

```text
outputs/physical_pi05_droid_m1/validation_final_20260827T1318Z/run.json
```

That generated output remains excluded from Git. It used all three selected
steps, two calls per step, episode seed `20260827`, pinned OpenPI commit
`15a9616a00943ada6c20a0f158e3adb39df2ccac`, and checkpoint
`gs://openpi-assets/checkpoints/pi05_droid`.

Every runtime response had:

```text
shape: [15, 8]
horizon: 15
action dimension: 8
dtype leaving DroidOutputs: float64
finite: true
```

The `float64` dtype is an observed adapter result, not a client coercion. The
model samples latent floating actions, then NumPy quantile unnormalization uses
the checkpoint statistics loaded from JSON and the DROID output transform
preserves the resulting dtype.

Across the 45 predicted actions and their 45 recorded-reference actions, the
observed ranges were:

| Dimension | Predicted minimum | Predicted maximum | Recorded minimum | Recorded maximum |
|---:|---:|---:|---:|---:|
| 0 | -0.034410 | 0.314773 | -0.020776 | 0.255855 |
| 1 | -0.081175 | 0.501035 | -0.042183 | 0.390940 |
| 2 | -0.158451 | 0.331061 | -0.153904 | 0.270935 |
| 3 | -0.086984 | 0.376949 | 0.013115 | 0.437794 |
| 4 | -0.052220 | 0.426937 | -0.087544 | 0.358219 |
| 5 | -0.305443 | 0.429243 | -0.283369 | 0.371113 |
| 6 | -0.081543 | 0.326047 | -0.067412 | 0.299397 |
| 7, gripper | 0.024370 | 0.965963 | 0.000000 | 1.000000 |

These values describe three diagnostic samples from one episode. They do not
measure policy quality or establish general dataset ranges.

The first inference after server startup took `5.713 s` client round-trip,
including JAX compilation (`5.267 s` model time). In the final warmed six-call
run, client round-trip was `97.3–108.5 ms` with mean `99.5 ms`; server inference
was `96.2–97.3 ms`, and model inference was `78.4–79.3 ms`. These are diagnostic
values from this machine and not a benchmark.

Each same-observation, same-seed, same-replan repeat was bitwise identical with
maximum absolute difference `0.0`. The full predicted chunks also matched
bitwise between the initial and final client runs. Each repeated pair reported
the same latent-noise SHA-256.

## Pinned input transform contract

[`src/saps/policies/openpi_droid.py`](../src/saps/policies/openpi_droid.py)
constructs the five raw keys consumed by pinned OpenPI:

```text
observation/exterior_image_1_left
observation/wrist_image_left
observation/joint_position
observation/gripper_position
prompt
```

The M1 client requires RGB `uint8` HWC images and passes the extracted DROID
resolution `(180, 320, 3)` without client-side resizing. It requires joint
shape `(7,)`, gripper shape `(1,)`, and converts both state arrays to
`float32`. The prompt must be a non-empty string.

Pinned `DroidInputs` then:

1. concatenates seven joint positions and one gripper position into an
   eight-dimensional state;
2. maps exterior-1 to `base_0_rgb` and wrist to `left_wrist_0_rgb`;
3. supplies a zero right-wrist image with its mask false;
4. applies checkpoint DROID quantile normalization using `q01` and `q99`;
5. performs padded resizing to `(224, 224, 3)`;
6. tokenizes the prompt with π0.5's discrete-state input; and
7. zero-pads state to the model's latent action dimension of 32.

The upstream adapter can also coerce floating images and CHW images for
LeRobot use. M1 deliberately uses the narrower canonical raw-DROID contract so
invalid image layout or dtype is rejected locally and provenance stays clear.

## Pinned output transform and action meaning

The `pi05_droid` inference config declares latent model output
`[15, 32]`. OpenPI applies checkpoint DROID quantile unnormalization first,
then `DroidOutputs` returns `actions[..., :8]`. There is no delta-to-absolute
joint transform in the inference configuration.

Pinned OpenPI explicitly states that the original π0.5-DROID checkpoint was
trained with:

```text
dimensions 0..6: DROID joint_velocity
dimension 7:     DROID gripper_position
```

Despite its name, DROID `joint_velocity` is not directly a physical rad/s
command. The reference boundary is

```text
u_pi = policy_action[0:7]
u_ref = clip(u_pi, -1, 1)       # component by component
delta_q = 0.2 u_ref             # rad/reference update
```

This is not global vector rescaling. The `15 * delta_q` quantity is only an
equivalent average-velocity diagnostic at DROID's 15 Hz reference cadence; it
is not the native policy command or an FR3 command. See the authoritative
[FR3 kinematics and mapping record](physical_fr3_embodiment.md).

DROID gripper position is absolute and normalized by physical width:

```text
gripper_position = 1 - current_width / maximum_width
0 = open
1 = closed
```

The pinned live DROID example thresholds the policy gripper value at `0.5`,
binarizes it to zero or one, and then clips every action component to `[-1, 1]`
before execution. M1 records the continuous value leaving the policy adapter
and does not threshold, clip, or execute it.

The checkpoint action quantiles used for the inverse scaling are:

| Dimension | q01 | q99 |
|---:|---:|---:|
| 0 | -0.4580 | 0.4476 |
| 1 | -0.8076 | 0.7652 |
| 2 | -0.4472 | 0.4480 |
| 3 | -0.9268 | 0.7944 |
| 4 | -0.6456 | 0.6484 |
| 5 | -0.6460 | 0.6628 |
| 6 | -0.7616 | 0.7344 |
| 7, gripper | 0.0000 | 0.9998 |

For each dimension, pinned `Unnormalize(use_quantiles=True)` applies
`(x + 1) / 2 * (q99 - q01 + 1e-6) + q01`. It does not clip normalized model
outputs before this transform, so these quantiles define scale rather than hard
output bounds.

## Horizon discrepancy

The pinned sources contain a real inconsistency:

- the `pi05_droid` inference configuration sets `action_horizon=15`;
- the DROID robot example comments and asserts a returned shape of `(10, 8)`.

The diagnostic never accepts either horizon as a client constant. It validates
only a positive runtime horizon and eight returned dimensions, then records the
actual shape. The seeded server separately reports its latent model horizon and
dimension in the handshake and sampling metadata.

The actual pinned runtime returned `(15, 8)` for all six calls. The example's
`(10, 8)` assertion is therefore stale for `pi05_droid` at this pinned revision;
the configured horizon of 15 is the observed model output. The intended pinned
physical DROID execution horizon is the first 8 actions at 15 Hz before
replanning; all 15 actions are used only for full-chunk diagnostics here.

## Determinism

The shared seeded server constructs flow-matching noise with shape
`[latent_action_horizon, latent_action_dimension]` from a JAX episode key and
folded-in replan index. For each selected observation, M1 repeats inference
with the same episode seed and replan index, logs the noise SHA-256, and checks
exact array equality plus maximum absolute difference. OpenPI sampling
semantics are not modified.

## Boundary to physical embodiment

M1 establishes the OpenPI boundary only. The manual FR3 FK, Jacobian, and
finite-action diagnostic are documented in
[`physical_fr3_embodiment.md`](physical_fr3_embodiment.md). Cartesian SAPS
scales, Cartesian-correction-to-joint mapping, execution scheduling, streaming
freshness, gripper execution, and physical safety supervision remain
unresolved; none is an M1 result.
