# Command Runbook

Run commands from the repository root unless a section says otherwise.

```bash
cd saps-openpi-replication
```

Detailed explanations are in:

- [`setup.md`](setup.md)
- [`testing.md`](testing.md)
- [`shared_autonomy.md`](shared_autonomy.md)
- [`human_input.md`](human_input.md)
- [`simulation_saps_baseline.md`](simulation_saps_baseline.md)
- [`gate2_operator_pilot.md`](gate2_operator_pilot.md)
- [`experiment_protocol.md`](experiment_protocol.md)
- [`analysis.md`](analysis.md)
- [`physical_m3_inputs.md`](physical_m3_inputs.md)

## 1. Bootstrap

```bash
git submodule update --init --recursive
make apply-patch
make build-images
make check
```

## 2. Policy server

Required for autonomous and shared-autonomy modes:

```bash
make policy-server
```

Stop it with:

```bash
make policy-stop
```

Not required for:

```text
make operator-smoke
make spacemouse-diagnostic
make spacemouse-calibrate
make teleop
```

## 3. Compact help

```bash
make help
```

The compact help lists the supported Make entry points and their common
overrides. Python entry points also expose `--help` when invoked directly in the
validated runtime container.

## 4. Automated checks

```bash
make check
```

Individual targets:

```bash
make unit-test
make compile
make clean-python
```

## 5. Diagnostics

```bash
make operator-smoke DURATION=60
make spacemouse-diagnostic \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick
make spacemouse-calibrate \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick
make seeded-probe CONDITION=nominal TRIAL=0
make scene-inspect
make perturbation-preview DX=0.10 DY=0.08 LABEL=p02
```

Both SpaceMouse targets load `configs/spacemouse_profile.json` by default. Set
`SPACEMOUSE_PROFILE=<path>` to inspect or edit another validated profile. For
the diagnostic only, `SPACEMOUSE_PROFILE=` intentionally selects the
unprofiled raw/default settings. Fresh calibration without loading an existing
profile is available from the Python runner's `--no-load-existing-profile`
option.

## 6. Physical M3 shadow diagnostics

M3 is live-validated and non-actuating. It requires an already running
read-only FR3 stack and two RealSense color nodes selected by distinct serials.
For the 2026-08-28 live
acceptance only, serial `244222076317` is the temporary exterior camera; this
does not establish its permanent scientific role. See
[`physical_m3_inputs.md`](physical_m3_inputs.md) before running it.

Start the pinned DROID policy server:

```bash
make droid-policy-server
```

Capture live inputs with a unique identity:

```bash
make physical-m3-observation \
  M3_RUN_ID=m3_$(date -u +%Y%m%dT%H%M%SZ) \
  M3_EXTERIOR_SERIAL=244222076317 \
  M3_PROMPT="pick up the object"
```

Run shadow inference for that captured identity, then validate the saved
actions with the hand-derived FK/Jacobian mapping:

```bash
make physical-m3-shadow-inference M3_RUN_ID=<captured-run-id>
make validate-droid-fr3-mapping \
  M3_MAPPING_RUN=outputs/physical_pi05_droid_m3/<captured-run-id>
```

The old normalized M2 projection is a provenance-only path and requires
`ALLOW_LEGACY_M2=1`; do not use it to define physical SAPS scales.

Collect a separate current-state SpaceMouse diagnostic:

```bash
make physical-m3-spacemouse \
  M3_SPACEMOUSE_RUN_ID=spnav_$(date -u +%Y%m%dT%H%M%SZ)
```

Stop the policy server afterward with `make policy-stop`. These targets never
launch or command the robot, but the surrounding laboratory bringup is
stateful and must follow its own safety procedure.

## 7. Autonomous execution

Single-condition smoke test:

```bash
make autonomous-smoke \
  CONDITION=nominal \
  INITIAL_STATE=0 \
  AUTONOMOUS_OUTPUT=outputs/autonomous_smoke
```

Resumable condition sweep:

```bash
make autonomous-sweep \
  NUM_TRIALS=20 \
  INITIAL_STATE=0 \
  AUTONOMOUS_MAX_STEPS=280 \
  AUTONOMOUS_OUTPUT=outputs/autonomous_n20_state0
```

Selected conditions:

```bash
make autonomous-sweep \
  CONDITION_IDS=p02,p03 \
  NUM_TRIALS=5 \
  AUTONOMOUS_OUTPUT=outputs/autonomous_selected
```

## 8. Human input selection

Keyboard is the default input. For SpaceMouse operation, add
`INPUT_SOURCE=spacemouse` to the Make invocation. The optional
`SPACEMOUSE_DEVICE=/dev/input/by-id/...-event-joystick` override selects a
specific device when auto-discovery is not sufficient. All normal and manifest-driven
Make targets automatically use the committed
`configs/spacemouse_profile.json`; override `SPACEMOUSE_PROFILE` only when a
different validated profile is intentionally required.

## 9. Archived matched simulation pilot

The matched pilot is complete. Its raw roots and historical experiment IDs are
frozen; do not launch or resume collection against them. Regenerate and validate
the lightweight derived archive read-only with:

```bash
make gate2-analysis
```

The historical preflight and collection entry points remain available to make
the frozen protocol auditable. The commands below describe the original
workflow and must not be used to overwrite or extend the completed roots.

Validate the 40 shared rows and intended 20 autonomous identities without
creating outputs or launching an episode:

```bash
make gate2-preflight \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick
```

The original shared collection command was:

```bash
make gate2-session \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick
```

The session is fixed to `configs/gate2_shared_autonomy_pilot_manifest.json`, the
committed SpaceMouse profile, and the now-immutable
`outputs/gate2_shared_autonomy_pilot_v2`. The autonomous protocol commands were:

```bash
make gate2-autonomous-preflight
make gate2-autonomous
```

See the
[Gate-2 protocol](gate2_operator_pilot.md) for counterbalancing, provenance,
redo, and scope restrictions.

## 10. Pure teleoperation

Manifest-driven 20-trial operator schedule (280 steps, 14 simulated seconds):

```bash
make teleoperation-session \
  INPUT_SOURCE=spacemouse \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick
```

Single-episode development run:

```bash
make teleop \
  INPUT_SOURCE=spacemouse \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick \
  CONDITION=nominal \
  TRIAL=0 \
  INITIAL_STATE=0 \
  TELEOP_MAX_STEPS=1800 \
  SPEED_MODE=fine \
  TELEOP_OUTPUT=outputs/teleoperation_smoke
```

Omit the two SpaceMouse overrides to run with the keyboard.

## 11. Shared autonomy

Manifest-driven shared-autonomy schedule (280 steps, 14 simulated seconds):

```bash
make shared-autonomy-session \
  INPUT_SOURCE=spacemouse \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick
```

Redo a completed attempt while preserving its history:

```bash
make shared-autonomy-session \
  REDO_EPISODES=trial_000__condition_p08__mode_cosine_blend
```

The commands below are single-episode development runs and may use a longer
horizon. They use keyboard input unless the SpaceMouse overrides from the
previous section are supplied.

Hard takeover:

```bash
make takeover \
  CONDITION=nominal \
  TRIAL=0 \
  SHARED_MAX_STEPS=1200 \
  SPEED_MODE=fine \
  SHARED_OUTPUT=outputs/takeover_smoke
```

Fixed/equal blending:

```bash
make fixed-blend \
  FIXED_AUTONOMY_WEIGHT=0.5 \
  CONDITION=nominal \
  TRIAL=0 \
  SHARED_MAX_STEPS=1200 \
  SPEED_MODE=fine \
  SHARED_OUTPUT=outputs/fixed_blend_smoke
```

Cosine-similarity blending:

```bash
make cosine-blend \
  COSINE_GAIN=6.0 \
  CONDITION=nominal \
  TRIAL=0 \
  SHARED_MAX_STEPS=1200 \
  SPEED_MODE=fine \
  SHARED_OUTPUT=outputs/cosine_blend_smoke
```

The underlying shared-autonomy Python runner still accepts
`--arbitration-mode autonomous` for programmatic experiment orchestration. For
manual autonomous runs, use `make autonomous-smoke` or `make autonomous-sweep`.

## 12. Browser controls

| Key | Command |
|---|---|
| `W / S` | screen forward / backward |
| `A / D` | screen left / right |
| `Space / Shift` | up / down |
| `Q / E` | yaw |
| `Up / Down` | pitch |
| `Left / Right` | roll |
| `Z / X` | open / close gripper |
| `1 / 2 / 3` | fine / normal / fast speed |
| `Escape` | abort episode |

Click **Arm controls** before using the keyboard. Losing browser focus clears
pressed keys.

## 13. Common variables

| Variable | Default | Meaning |
|---|---:|---|
| `CONDITION` | `nominal` | Single perturbation condition |
| `CONDITION_IDS` | empty | Selected autonomous sweep conditions |
| `TRIAL` | `0` | Trial identity and output index |
| `INITIAL_STATE` | `0` | LIBERO initial-state index |
| `ENVIRONMENT_SEED` | `7` | LIBERO environment seed |
| `POLICY_BASE_SEED` | `20260724` | Deterministic policy seed base |
| `INPUT_SOURCE` | `keyboard` | `keyboard` or `spacemouse` |
| `SPACEMOUSE_DEVICE` | empty | Optional stable runtime device path |
| `SPACEMOUSE_PROFILE` | `configs/spacemouse_profile.json` | Validated calibration profile |
| `SPEED_MODE` | `fine` | Initial keyboard gain profile |
| `AUTONOMOUS_MAX_STEPS` | `280` | Autonomous horizon |
| `TELEOP_MAX_STEPS` | `1800` | Teleoperation horizon |
| `SHARED_MAX_STEPS` | `1200` | Shared-autonomy horizon |
| `FIXED_AUTONOMY_WEIGHT` | `0.5` | Active-human fixed blend weight |
| `COSINE_GAIN` | `6.0` | Logistic cosine gain |

## 14. Analysis tools

Analyze an autonomous sweep:

```bash
make analyze \
  SUMMARY=outputs/autonomous_n20_state0/sweep_summary.json \
  ANALYSIS_OUTPUT=results/autonomous_n20_state0
```

Analyze autonomous, teleoperation, and shared-autonomy results together:

```bash
make analyze-comparison
```

The default operator roots are the versioned `saps_libero_teleoperation_v2` and
`saps_libero_shared_autonomy_v2` session outputs. Override the result-root
variables when analyzing another collection.

See [`analysis.md`](analysis.md) for outputs and interpretation constraints.

## 15. Output conventions

Generated data:

```text
outputs/
results/
```

Both are ignored by Git.

The exception is the small validated derived archive at
`results/gate2_shared_autonomy_pilot_v2`, which is intentionally tracked. The
completed raw roots under `outputs/` remain ignored and immutable.

Autonomous and teleoperation episodes use:

```text
<root>/<condition>/task_<id>/init_<index>/trial_<index>/
```

Shared-autonomy episodes add the arbitration mode. Fixed and cosine modes also
add their parameter component:

```text
<root>/takeover/<condition>/...
<root>/fixed_blend/alpha_0p500/<condition>/...
<root>/cosine_blend/k_6p000/<condition>/...
```

Do not reuse a completed output directory during versioned experiments.

Manifest sessions add immutable provenance and attempt history under their
session root:

```text
<session>/manifest.json
<session>/schedule.json
<session>/human_input.json
<session>/perturbation_config.json
<session>/repository_provenance.json
<session>/session_protocol.json
<session>/attempts/<episode_id>/attempt_<number>/...
```

## 16. Commit checklist

```bash
git status --short
git diff --check
make check

git diff --cached --stat
git diff --cached --check
```
