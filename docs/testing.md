# Testing and Validation

This document separates fast automated checks, interactive smoke tests, and
formal experiments. A successful smoke test demonstrates functional behavior;
it is not a statistical result.

## 1. Automated regression checks

Run the full unit suite and compile every project Python file:

```bash
make check
```

Equivalent commands:

```bash
make unit-test
make compile
```

The unit suite covers:

- deterministic policy action chunks and seed protocol checks;
- keyboard-to-action mapping;
- perturbation and output helpers;
- autonomous runner integration;
- autonomous, takeover, fixed, and cosine arbitration;
- asynchronous scheduler behavior during human activity;
- invalid action, weight, gain, and protocol inputs.

Do not rely on a hard-coded expected test count. New regression tests should
increase the count while every test still reports `ok`.

## 2. Browser input smoke test

This test does not create LIBERO and does not require the policy server:

```bash
make operator-smoke DURATION=60
```

Confirm:

- the browser connects;
- **Arm controls** enables input;
- key press and release are visible immediately;
- changing tabs or losing focus clears pressed keys;
- gripper and speed-mode commands update;
- Escape aborts cleanly.

## 3. Deterministic policy probe

Start the policy server first:

```bash
make policy-server
```

In a second terminal:

```bash
make seeded-probe CONDITION=nominal TRIAL=0
```

Repeat the same probe before and after a complete policy-server restart. The
same episode seed and replan index should reproduce the same action chunk and
latent-noise hash.

## 4. Perturbation preview

```bash
make perturbation-preview \
  DX=0.10 \
  DY=0.08 \
  LABEL=p02
```

Inspect the saved initial, perturbed, and settled images and the recorded object
pose. The operation must preserve object height and orientation while changing
only planar position.

## 5. Autonomous smoke test

With the policy server running:

```bash
make autonomous-smoke \
  CONDITION=nominal \
  INITIAL_STATE=0 \
  AUTONOMOUS_OUTPUT=outputs/autonomous_smoke
```

Expected evidence:

- deterministic policy seed in `summary.json`;
- monotonically increasing replan indices;
- seven-dimensional finite actions;
- a success or explicit timeout/termination reason;
- no human-action influence.

## 6. Pure teleoperation smoke test

The policy server is not required:

```bash
make teleop \
  CONDITION=nominal \
  TRIAL=0 \
  INITIAL_STATE=0 \
  TELEOP_MAX_STEPS=1800 \
  TELEOP_OUTPUT=outputs/teleoperation_smoke
```

Use the displayed browser interface. A nominal pilot previously completed in
796 control steps, but completion time is operator-dependent and is not an
acceptance threshold.

## 7. Shared-autonomy smoke tests

Keep the policy server running. Use unique trial indices or output roots for
repeated attempts.

### Hard takeover

```bash
make takeover \
  CONDITION=nominal \
  TRIAL=0 \
  SHARED_MAX_STEPS=280 \
  SHARED_OUTPUT=outputs/takeover_smoke
```

Confirm transitions through `takeover_started`, `human_takeover`,
`takeover_released`, and `takeover_resync`. Policy requests should pause during
active takeover, and stale pre-takeover inference should be rejected.

### Fixed/equal blending

```bash
make fixed-blend \
  FIXED_AUTONOMY_WEIGHT=0.5 \
  CONDITION=nominal \
  TRIAL=0 \
  SHARED_MAX_STEPS=280 \
  SHARED_OUTPUT=outputs/fixed_blend_smoke
```

When human motion is active, verify:

```text
executed_motion =
    0.5 * autonomous_motion
    + 0.5 * human_motion
```

When the human is idle, the effective autonomy weight must be `1.0`.

### Cosine-similarity blending

```bash
make cosine-blend \
  COSINE_GAIN=6.0 \
  CONDITION=nominal \
  TRIAL=0 \
  SHARED_MAX_STEPS=280 \
  SHARED_OUTPUT=outputs/cosine_blend_smoke
```

Apply varied human commands. Confirm that logs contain computed similarities,
dynamic weights, continuous policy requests during human activity, and explicit
`cosine_blend_policy_wait` records when inference is pending.

## 8. Output-integrity checks

Every completed episode should contain:

```text
summary.json
steps.jsonl
perturbation.json
rollout_<termination>.mp4
```

Shared-autonomy episodes also contain:

```text
scheduler_waits.jsonl
```

Basic inspection:

```bash
find outputs -name summary.json -print
python3 -m json.tool <path-to-summary.json>
wc -l <path-to-steps.jsonl>
```

Check that action arrays are length seven and finite, and that the output path
matches the requested mode, condition, initial state, and trial.

## 9. Milestone checklist

Before committing a functional or documentation milestone:

```bash
git status --short
git diff --check
make check
```

For documentation-only changes, `make check` remains recommended because command
examples and Makefile targets can drift even when Python code is unchanged.
