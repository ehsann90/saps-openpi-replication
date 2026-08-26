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
- SpaceMouse normalization, mapping, device discovery, exclusive access,
  stale/lost-input safety, cleanup, and diagnostic CLI wiring.
- calibration profile validation/round-trip, per-axis sensitivity and enable
  masks, safe live apply/reset/save behavior, and normal-runner defaults.
- manifest-session profile propagation, immutable profile provenance, resume
  protection, and Make-to-runner argument contracts.
- exact Gate-2 v2 40-row shared and 20-row autonomous coverage, deterministic
  two-mode counterbalancing, 20 mode-independent matched triplets, unchanged
  legacy scheduling, protocol/profile rejection, immutable resume state, and
  Gate-2 Make-to-CLI contracts.
- Gate-2 completion validity for success/timeout, termination and operator-input
  integrity failures, neutral stale input, redo selection preservation, and
  unchanged legacy acceptance semantics.
- Gate-2 analysis with independently incomplete collections, failed and timeout outcomes,
  invalid-attempt audit exclusion, selected redos, seed/profile mismatch
  rejection, fixed/cosine diagnostics,
  policy-wait timing and human overlap, and exact autonomous pairing.

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

## 3. SpaceMouse input diagnostic

This manual diagnostic requires hardware but starts no LIBERO environment or
policy server:

```bash
make spacemouse-diagnostic \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick
```

The Make target loads `configs/spacemouse_profile.json` by default, so mapped
axes and actions should match the physically validated calibration. Set
`SPACEMOUSE_PROFILE=` only for an intentional raw/default diagnostic.

Confirm all six raw axes change, mapped axes and signs match the intended robot
frame, the final action respects gains/deadzone, both configured buttons work,
and unplugging zeros motion and requires re-arming after reconnection. Follow
the ownership diagnostics in [`human_input.md`](human_input.md) if acquisition
fails.

For application-level calibration after the low-level diagnostic passes:

```bash
make spacemouse-calibrate \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick
```

This mode is interactive and is not part of automated verification. Confirm
that Apply/Reset/Save disarm, translation/rotation isolation works, resetting
restores the nominal scene, and the saved profile matches the browser values.
The same `SPACEMOUSE_PROFILE` variable selects the diagnostic, calibration, and
runtime profile.

## Gate-2 preflight

Gate-2 preflight is non-interactive and does not launch an episode:

```bash
make gate2-preflight \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick
```

Inspect the reported manifest/profile/config hashes, exact counts, parameters,
and 40-row shared schedule plus 20 intended matched autonomous identities. This validates protocol wiring, not physical device access;
run the SpaceMouse diagnostic separately. Do not invoke `gate2-session` as part
of automated testing.

The autonomous preflight is also non-launching and output-independent:

```bash
make gate2-autonomous-preflight
```

## 4. Deterministic policy probe

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

## 5. Perturbation preview

```bash
make perturbation-preview \
  DX=0.10 \
  DY=0.08 \
  LABEL=p02
```

Inspect the saved initial, perturbed, and settled images and the recorded object
pose. The operation must preserve object height and orientation while changing
only planar position.

## 6. Autonomous smoke test

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

## 7. Pure teleoperation smoke test

The policy server is not required. Verify the keyboard path with:

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

Verify the calibrated SpaceMouse path separately with a unique output root:

```bash
make teleop \
  INPUT_SOURCE=spacemouse \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick \
  CONDITION=nominal \
  TRIAL=0 \
  TELEOP_OUTPUT=outputs/spacemouse_teleop_smoke
```

The summary must contain the calibrated profile identity. Every step must log
the SpaceMouse source, selected device, `0.40/0.08` gains, calibrated
mapping/signs, and finite seven-dimensional actions. Stale neutral samples are
expected when the puck is idle, but their six motion dimensions must be zero.

## 8. Shared-autonomy smoke tests

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
  INPUT_SOURCE=spacemouse \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick \
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
  INPUT_SOURCE=spacemouse \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick \
  COSINE_GAIN=6.0 \
  CONDITION=nominal \
  TRIAL=0 \
  SHARED_MAX_STEPS=280 \
  SHARED_OUTPUT=outputs/cosine_blend_smoke
```

Apply varied human commands. Confirm that logs contain computed similarities,
dynamic weights, continuous policy requests during human activity, and explicit
`cosine_blend_policy_wait` records when inference is pending.

## 9. Output-integrity checks

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

Profile-backed SpaceMouse episodes also record `spacemouse_profile` in
`summary.json`. Its path, schema version, and SHA-256 must match the configured
profile and the session-level `human_input.json` when launched by a manifest
session.

Basic inspection:

```bash
find outputs -name summary.json -print
python3 -m json.tool <path-to-summary.json>
wc -l <path-to-steps.jsonl>
```

Check that action arrays are length seven and finite, and that the output path
matches the requested mode, condition, initial state, and trial.

## 10. Milestone checklist

Before committing a functional or documentation milestone:

```bash
git status --short
git diff --check
make check
```

For documentation-only changes, `make check` remains recommended because command
examples and Makefile targets can drift even when Python code is unchanged.
