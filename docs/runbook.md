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
- [`experiment_protocol.md`](experiment_protocol.md)
- [`analysis.md`](analysis.md)

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
make teleop
```

## 3. Compact help

```bash
make help
```

CLI-specific help:

```bash
make help-operator
make help-probe
make help-teleop
make help-autonomous
make help-shared-autonomy
```

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

## 6. Autonomous execution

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
  CONDITION_IDS="p02 p03" \
  NUM_TRIALS=5 \
  AUTONOMOUS_OUTPUT=outputs/autonomous_selected
```

## 7. Pure teleoperation

Formal 20-trial operator schedule (280 steps, 14 simulated seconds):

```bash
make teleoperation-session
```

Single-episode development run:

```bash
make teleop \
  CONDITION=nominal \
  TRIAL=0 \
  INITIAL_STATE=0 \
  TELEOP_MAX_STEPS=1800 \
  SPEED_MODE=fine \
  TELEOP_OUTPUT=outputs/teleoperation_smoke
```

## 8. Shared autonomy

Formal shared-autonomy schedule (280 steps, 14 simulated seconds):

```bash
make shared-autonomy-session
```

Redo a completed attempt while preserving its history:

```bash
make shared-autonomy-session \
  REDO_EPISODES=trial_000__condition_p08__mode_cosine_blend
```

The commands below are single-episode development runs and may use a longer
horizon.

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

## 9. Browser controls

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

## 10. Common variables

| Variable | Default | Meaning |
|---|---:|---|
| `CONDITION` | `nominal` | Single perturbation condition |
| `CONDITION_IDS` | empty | Selected autonomous sweep conditions |
| `TRIAL` | `0` | Trial identity and output index |
| `INITIAL_STATE` | `0` | LIBERO initial-state index |
| `ENVIRONMENT_SEED` | `7` | LIBERO environment seed |
| `POLICY_BASE_SEED` | `20260724` | Deterministic policy seed base |
| `SPEED_MODE` | `fine` | Initial operator gain profile |
| `AUTONOMOUS_MAX_STEPS` | `280` | Autonomous horizon |
| `TELEOP_MAX_STEPS` | `1800` | Teleoperation horizon |
| `SHARED_MAX_STEPS` | `1200` | Shared-autonomy horizon |
| `FIXED_AUTONOMY_WEIGHT` | `0.5` | Active-human fixed blend weight |
| `COSINE_GAIN` | `6.0` | Logistic cosine gain |

## 11. Current analysis tool

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

See [`analysis.md`](analysis.md) for outputs and interpretation constraints.

## 12. Output conventions

Generated data:

```text
outputs/
results/
```

Both are ignored by Git.

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

Do not reuse a completed output directory during formal experiments.

## 13. Commit checklist

```bash
git status --short
git diff --check
make check

git diff --cached --stat
git diff --cached --check
```
