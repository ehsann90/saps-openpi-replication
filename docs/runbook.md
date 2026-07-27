# SAPS OpenPI Replication Runbook

This runbook records the commands used to run the main replication processes, tests, diagnostics, and analysis tools.

Run all commands from the repository root:

~~~bash
cd ~/MyProjects/saps-openpi-replication
~~~

## 1. Repository layout

### Main experiment entry points

| Path | Purpose |
|---|---|
| `scripts/serve_seeded_policy.py` | Deterministic OpenPI policy server |
| `scripts/run_libero.py` | Single autonomous LIBERO run |
| `scripts/run_autonomous_sweep.py` | Resumable autonomous condition sweep |
| `scripts/run_teleoperation_episode.py` | Browser-controlled pure teleoperation |

### Tests

| Path | Purpose |
|---|---|
| `tests/unit/test_keyboard_mapping.py` | Automated tests for keyboard-to-action mapping |
| `tests/manual/keyboard_operator_smoke.py` | Interactive browser input smoke test |

### Supporting tools

| Path | Purpose |
|---|---|
| `tools/diagnostics/inspect_libero_scene.py` | Inspect task objects, observations, and cameras |
| `tools/diagnostics/preview_libero_perturbation.py` | Preview one controlled planar object offset |
| `tools/diagnostics/probe_seeded_policy.py` | Validate deterministic policy sampling |
| `tools/analysis/analyze_autonomous_results.py` | Analyze a completed autonomous sweep |
| `tools/monitoring/watch_autonomous_progress.py` | Legacy autonomous progress monitor |

### Patches

| Path | Purpose |
|---|---|
| `patches/apply_openpi_patch.sh` | Apply the OpenPI Python 3.8 compatibility patch |
| `patches/openpi-libero-python38-build.patch` | Stored compatibility patch |

---

## 2. Quick command reference

Display the available Makefile commands:

~~~bash
make help
~~~

Common commands:

```bash
make apply-patch
make policy-server
make autonomous-smoke
make autonomous-sweep NUM_TRIALS=20 AUTONOMOUS_MAX_STEPS=280
make operator-smoke
make teleop CONDITION=nominal TRIAL=4 TELEOP_MAX_STEPS=1800
make seeded-probe
make unit-test
make compile
```

---

## 3. Apply the OpenPI compatibility patch

```bash
make apply-patch
```

Equivalent direct command:

```bash
./patches/apply_openpi_patch.sh
```

The patch script is idempotent. It reports whether the patch was newly applied or was already present.

---

## 4. Deterministic OpenPI policy server

The policy server is required for autonomous runs, autonomous sweeps, deterministic-policy probes, and future shared-autonomy arbitration modes.

It is not required for pure teleoperation or the browser-input smoke test.

Start the server in a dedicated terminal:

```bash
make policy-server
```

Stop it with:

```bash
make policy-stop
```

Default endpoint:

```text
host: 0.0.0.0
port: 8000
```

The server uses deterministic GPU execution through:

```text
XLA_FLAGS=--xla_gpu_deterministic_ops=true
```

---

## 5. Automated checks

### Keyboard mapping unit tests

```bash
make unit-test
```

Expected result:

```text
Ran 6 tests
OK
```

### Compile all project Python code

```bash
make compile
```

Expected result:

```text
Compilation passed
```

### Remove Python cache files

```bash
make clean-python
```

If cache files were created by a root-owned Docker process and cannot be deleted normally:

```bash
sudo rm -rf scripts/__pycache__ tests/**/__pycache__ tools/**/__pycache__ src/**/__pycache__
```

The repository ignores:

```text
__pycache__/
*.py[cod]
.pytest_cache/
```

---

## 6. Browser operator smoke test

Run the browser interface without creating a LIBERO environment:

```bash
make operator-smoke
```

Override its duration:

```bash
make operator-smoke DURATION=300
```

Open the URL shown in the terminal, normally:

```text
http://127.0.0.1:8766
```

### Controls

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

### Speed profiles

| Mode | Translation gain | Rotation gain |
|---|---:|---:|
| Fine | 0.07 | 0.10 |
| Normal | 0.14 | 0.18 |
| Fast | 0.25 | 0.30 |

The operator must click **Arm controls** before motion commands are accepted.

Releasing a key immediately removes its motion command. Switching browser tabs or losing focus clears all pressed keys. Only one browser client is accepted at a time.

Show all manual smoke-test arguments:

```bash
make help-operator
```

---

## 7. Pure teleoperation

Pure teleoperation does not require the OpenPI policy server.

Example nominal trial:

```bash
make teleop \
  CONDITION=nominal \
  TRIAL=4 \
  INITIAL_STATE=0 \
  TELEOP_MAX_STEPS=1800
```

Example perturbed trial:

```bash
make teleop \
  CONDITION=p02 \
  TRIAL=0 \
  INITIAL_STATE=0 \
  TELEOP_MAX_STEPS=1800 \
  TELEOP_OUTPUT=outputs/teleoperation_pilot
```

The browser displays:

```text
Agent view | Wrist view
```

Recommended operating strategy:

- use `3` for coarse movement through free space;
- use `2` while approaching the object or basket;
- use `1` for final alignment, grasping, and release.

### Important Make variables

| Variable | Default | Meaning |
|---|---:|---|
| `CONDITION` | `nominal` | Perturbation condition ID |
| `TRIAL` | `0` | Trial index used in seeds and output paths |
| `INITIAL_STATE` | `0` | LIBERO initial-state index |
| `TELEOP_MAX_STEPS` | `1800` | Teleoperation control horizon |
| `SPEED_MODE` | `fine` | Initial operator speed |
| `ENVIRONMENT_SEED` | `7` | LIBERO environment seed |
| `POLICY_BASE_SEED` | `20260724` | Matched seed derivation base |
| `TELEOP_OUTPUT` | `outputs/teleoperation_smoke` | Output root |

The Python script currently defaults to:

```text
max_steps = 1200
control_frequency_hz = 20
video_fps = 20
resolution = 256
http_port = 8766
websocket_port = 8765
```

Show every CLI argument:

```bash
make help-teleop
```

### Successful Phase 2.2 pilot

The validated nominal pure-teleoperation pilot achieved:

```text
success = true
termination_reason = success
control_steps = 796
simulated_control_seconds = 39.8
```

---

## 8. Autonomous single-condition smoke test

Start the policy server in terminal 1:

```bash
make policy-server
```

Run one nominal autonomous trial in terminal 2:

```bash
make autonomous-smoke \
  CONDITION=nominal \
  INITIAL_STATE=0 \
  AUTONOMOUS_OUTPUT=outputs/autonomous_smoke
```

The autonomous smoke target uses deterministic policy sampling.

---

## 9. Autonomous condition sweep

Run every condition from `configs/libero_cream_cheese_offsets.json`:

```bash
make autonomous-sweep \
  NUM_TRIALS=20 \
  INITIAL_STATE=0 \
  AUTONOMOUS_MAX_STEPS=280 \
  AUTONOMOUS_OUTPUT=outputs/autonomous_n20_state0
```

Run one selected condition:

```bash
make autonomous-sweep \
  CONDITION_IDS=p02 \
  NUM_TRIALS=5 \
  INITIAL_STATE=0 \
  AUTONOMOUS_MAX_STEPS=280 \
  AUTONOMOUS_OUTPUT=outputs/p02_pilot
```

Leave `CONDITION_IDS` empty to run all configured conditions.

### Important Make variables

| Variable | Default | Meaning |
|---|---:|---|
| `CONDITION_IDS` | empty | Selected sweep conditions; empty means all |
| `NUM_TRIALS` | `1` | Trials per condition |
| `INITIAL_STATE` | `0` | Fixed LIBERO initial-state index |
| `AUTONOMOUS_MAX_STEPS` | `280` | Autonomous control horizon |
| `POLICY_BASE_SEED` | `20260724` | Deterministic policy seed base |
| `AUTONOMOUS_OUTPUT` | `outputs/autonomous_sweep` | Output root |

### Phase 1 autonomous protocol

```text
conditions = nominal + p01 ... p09
trials_per_condition = 20
initial_state_index = 0
max_steps = 280
control_frequency_hz = 20
settling_steps = 10
```

Show every autonomous-sweep CLI argument:

```bash
make help-autonomous
```

---

## 10. Single autonomous LIBERO run

`scripts/run_libero.py` runs a single condition directly.

Show all arguments:

```bash
LOCAL_UID="$(id -u)" \
LOCAL_GID="$(id -g)" \
docker compose -f compose.yml run --rm --no-deps runtime \
  /bin/bash -lc \
  'source /.venv/bin/activate && \
   python /workspace/scripts/run_libero.py --help'
```

Important defaults:

```text
task_suite_name = libero_object
task_id = 1
initial_state_index = 0
condition_id = nominal
seed = 7
resolution = 256
resize_size = 224
replan_steps = 5
num_steps_wait = 10
max_steps = 280
```

---

## 11. Deterministic-policy probe

The policy server must already be running.

Nominal probe:

```bash
make seeded-probe \
  CONDITION=nominal \
  TRIAL=0
```

Perturbed probe:

```bash
make seeded-probe \
  CONDITION=p02 \
  TRIAL=3 \
  PROBE_OUTPUT=outputs/p02_seed_probe
```

Important variables:

| Variable | Default |
|---|---:|
| `CONDITION` | `nominal` |
| `TRIAL` | `0` |
| `INITIAL_STATE` | `0` |
| `POLICY_BASE_SEED` | `20260724` |
| `ENVIRONMENT_SEED` | `7` |
| `PROBE_OUTPUT` | `outputs/seeded_policy_probe` |

Show all arguments:

```bash
make help-probe
```

---

## 12. Scene diagnostics

### Inspect the LIBERO scene

```bash
make scene-inspect
```

Override output location:

```bash
make scene-inspect \
  INITIAL_STATE=0 \
  SCENE_OUTPUT=outputs/scene_inspection
```

### Preview a controlled perturbation

```bash
make perturbation-preview \
  DX=0.10 \
  DY=0.08 \
  LABEL=p02
```

Common variables:

| Variable | Default |
|---|---:|
| `DX` | `0.0` |
| `DY` | `0.0` |
| `LABEL` | `preview` |
| `INITIAL_STATE` | `0` |
| `PREVIEW_OUTPUT` | `outputs/perturbation_preview` |

---

## 13. Analyze autonomous results

```bash
make analyze \
  SUMMARY=outputs/autonomous_n20_state0/sweep_summary.json \
  ANALYSIS_OUTPUT=results/autonomous_n20_state0
```

The positional `SUMMARY` value must point to a completed `sweep_summary.json`.

---

## 14. Autonomous progress monitor

The legacy monitor is located at:

```text
tools/monitoring/watch_autonomous_progress.py
```

It currently does not have a conventional CLI parser. Passing `--help` starts monitoring rather than displaying usage, so it is not yet exposed as a Makefile target.

Direct invocation:

```bash
python3 tools/monitoring/watch_autonomous_progress.py
```

This tool should receive a proper argument parser before being treated as a stable command-line entry point.

---

## 15. Output conventions

Development and evaluation outputs are written below `outputs/`, which is ignored by Git.

### Teleoperation output structure

```text
<output-root>/
└── <condition-id>/
    └── task_<task-id>/
        └── init_<initial-state>/
            └── trial_<trial-index>/
                ├── 01_nominal_initial.png
                ├── 02_perturbed_before_settle.png
                ├── 03_perturbed_after_settle.png
                ├── perturbation.json
                ├── steps.jsonl
                ├── summary.json
                └── rollout_<termination>.mp4
```

Use a new `TRIAL` value for each attempt. Reusing a trial index writes to the same episode directory.

### Teleoperation step log

Each control step includes fields such as:

```text
human_action
autonomous_action
autonomy_weight
executed_action
operator_speed_mode
operator_translation_gain
operator_rotation_gain
operator_gripper_command
gripper_qpos
eef_position
object_position
policy_episode_seed
policy_replan_index
reward
done
```

---

## 16. Direct CLI help commands

```bash
make help
make help-operator
make help-probe
make help-teleop
make help-autonomous
```

---

## 17. Before committing a milestone

```bash
git status --short
git diff --check
make unit-test
make compile
```

Inspect staged changes:

```bash
git diff --cached --stat
git diff --cached --check
```

Then commit and push:

```bash
git commit -m "<descriptive milestone message>"
git push origin phase2-human-input-arbitration
```
