# Autonomous LIBERO Perturbations and Deterministic Policy Sampling

**Status:** Complete  
**Completed:** July 2026

The filename retains the historical Phase-1 label for existing links. In the
current project narrative, this is the autonomous robustness and deterministic
sampling foundation of the completed simulation SAPS baseline.

## 1. Purpose

This phase establishes the autonomous experimental foundation for the replication of:

> SAPS: Shared Autonomy for Policy Steering by Blending Teleoperation with a Pretrained VLA.

The phase had three objectives:

1. reproduce the controlled LIBERO cream-cheese object perturbations;
2. measure the degradation of the autonomous π0.5 policy across those perturbations;
3. make π0.5 action sampling deterministic so that autonomous, teleoperation, takeover, fixed-blending, and cosine-blending trials can later be compared using matched policy randomness.

Human input and arbitration were intentionally deferred to Phase 2.

---

## 2. Software and Model Baseline

The implementation uses:

- OpenPI commit: `15a9616a00943ada6c20a0f158e3adb39df2ccac`
- LIBERO commit: `f78abd68ee283de9f9be3c8f7e2a9ad60246e95c`
- model configuration: `pi05_libero`
- checkpoint: `openpi-assets/checkpoints/pi05_libero`
- task suite: `libero_object`
- task ID: `1`
- task instruction: `pick up the cream cheese and place it in the basket`

OpenPI is kept as a pinned Git submodule. Project-specific behavior is implemented in the outer repository rather than by modifying OpenPI source files.

The retained OpenPI compatibility change is stored separately as:

```text
patches/openpi-libero-python38-build.patch
```

---

## 3. Perturbation Implementation

### 3.1 Target Object

The cream-cheese object is identified by:

```text
joint: cream_cheese_1_joint0
body:  cream_cheese_1_main
```

The perturbation modifies only the planar position components of the free-joint state:

```text
qpos[0] += delta_x
qpos[1] += delta_y
```

The original height and orientation are preserved.

After applying the perturbation, the environment executes ten dummy steps to allow the object to settle before the policy receives its first observation.

### 3.2 Conditions

The configuration is stored in:

```text
configs/libero_cream_cheese_offsets.json
```

The implemented conditions are:

| Condition | Δx (m) | Δy (m) | Radial distance (m) |
|---|---:|---:|---:|
| nominal | 0.00 | 0.00 | 0.000 |
| p01 | 0.00 | 0.08 | 0.080 |
| p02 | 0.10 | 0.08 | 0.128 |
| p03 | 0.00 | 0.18 | 0.180 |
| p04 | 0.18 | 0.05 | 0.187 |
| p05 | 0.13 | 0.15 | 0.198 |
| p06 | 0.05 | 0.23 | 0.235 |
| p07 | 0.06 | 0.23 | 0.238 |
| p08 | 0.20 | 0.16 | 0.256 |
| p09 | 0.08 | 0.28 | 0.291 |

Appendix Table A1 of SAPS lists nine offset pairs, although the paper text refers to eight perturbed configurations. This repository preserves all nine listed offsets and adds the nominal condition.

### 3.3 Verification

The implementation records:

- the requested offset;
- the object joint state immediately after perturbation;
- the object pose after settling;
- initial, perturbed, and settled scene images.

The settled planar positions were checked for every condition and matched the requested translations.

---

## 4. Autonomous Evaluation Protocol

The autonomous baseline used:

| Parameter | Value |
|---|---:|
| Initial-state index | 0 |
| Conditions | 10 |
| Trials per condition | 20 |
| Total episodes | 200 |
| Control frequency | 20 Hz |
| Maximum control steps | 280 |
| Active horizon | 14.0 simulated seconds |
| Settling steps | 10 |
| Executed actions per policy chunk | 5 |
| Arbitration mode | Autonomous |

Conditions were executed using a cyclic round-robin schedule. The starting condition was rotated in each trial round to distribute execution order across conditions.

The scheduler supports:

- interruption and resumption;
- compatibility checks before reusing completed episodes;
- per-condition summaries;
- root sweep summaries;
- episode videos;
- step-level JSON logging;
- ownership repair for container-generated outputs.

The official LIBERO task-success signal determines success. Contact with or movement of an incorrect object does not immediately terminate an episode. The policy may continue until official task success or timeout.

The 280-step horizon was inherited from the standard OpenPI LIBERO evaluation. It was not specified by the SAPS paper.

---

## 5. Autonomous 200-Episode Results

The completed autonomous sweep produced:

| Condition | Successes | Trials | Success rate |
|---|---:|---:|---:|
| nominal | 20 | 20 | 100% |
| p01 | 20 | 20 | 100% |
| p02 | 13 | 20 | 65% |
| p03 | 15 | 20 | 75% |
| p04 | 0 | 20 | 0% |
| p05 | 0 | 20 | 0% |
| p06 | 5 | 20 | 25% |
| p07 | 1 | 20 | 5% |
| p08 | 0 | 20 | 0% |
| p09 | 0 | 20 | 0% |

For the nine perturbed conditions, the relationship between radial offset distance and condition-level success rate was approximately:

```text
Pearson r = -0.820
p = 0.0067
```

For perturbations with radial distance of at least 0.15 m:

```text
21 successes / 140 episodes = 15.0%
```

The results show strong degradation as the target object is moved away from its nominal training distribution. However, radial distance alone does not explain all behavior. For example, `p03` and `p04` have similar radial distances but substantially different success rates, showing that perturbation direction and scene geometry also matter.

### Important Experimental Distinction

The 200-episode sweep was completed before deterministic policy sampling was integrated. It is therefore retained as the Phase 1 autonomous degradation baseline.

It should not be treated as the matched autonomous condition for later human-arbitration experiments. The shared-autonomy study will run a new deterministic autonomous control using the same matched seeds as the human-assisted modes.

---

## 6. Deterministic Policy Sampling

### 6.1 Motivation

OpenPI normally advances an internal JAX random-number state each time the policy is queried. Therefore, using the same checkpoint and initial environment state does not by itself guarantee matched autonomous action samples across separate experimental runs.

A matched arbitration experiment requires each experimental unit:

```text
(condition_id, trial_index)
```

to receive one stable autonomous-policy seed that is reused across:

- deterministic autonomous control;
- pure teleoperation, where policy logging remains relevant;
- takeover;
- fixed 50/50 blending;
- cosine-similarity blending.

The arbitration mode is deliberately excluded from seed derivation.

### 6.2 Seed Protocol

The current protocol is:

```text
saps-policy-seed-v1
```

The episode seed is derived from:

- base policy seed;
- task ID;
- initial-state index;
- condition ID;
- trial index.

A SHA-256 digest is used rather than Python's process-randomized `hash()` function.

At each policy replan:

```text
noise_key = fold_in(episode_key, replan_index)
```

The server generates an explicit latent noise tensor and passes it to the OpenPI policy.

The replan index:

- starts at zero for every episode;
- increments once per newly sampled action chunk;
- does not increment for individual actions taken from the existing chunk.

### 6.3 Deterministic GPU Execution

Explicitly seeded latent noise was not sufficient for bitwise equality across policy-server restarts because GPU kernel selection introduced small numerical differences.

The policy-server container therefore uses:

```text
XLA_FLAGS=--xla_gpu_deterministic_ops=true
```

With this setting, action chunks were bit-for-bit identical across complete server restarts.

### 6.4 Validation

The following tests passed:

1. the same observation, episode seed, and replan index produced identical action chunks within one server process;
2. a different episode seed produced a different action chunk;
3. a different replan index produced a different action chunk;
4. restarting the policy server reproduced identical action arrays and hashes;
5. two full nominal episodes reproduced identical:
   - policy actions;
   - end-effector trajectories;
   - object trajectories;
   - rewards;
   - success signals;
   - policy replan indices;
   - latent-noise hashes;
   - completion step.

The full deterministic nominal episode completed successfully in:

```text
151 control steps
31 policy replans
```

### 6.5 Interpretation for Matched Arbitration Trials

Matched seeds ensure that arbitration modes receive the same autonomous sampling sequence.

Before human input affects the executed action, identical observations and identical policy noise should produce identical autonomous action chunks.

After human input changes the robot or environment state, subsequent observations naturally differ. The autonomous policy may then produce different actions even though the same deterministic replan-noise sequence is being used. This divergence is expected and scientifically appropriate.

---

## 7. Main Implementation Files

### Perturbations and Environment

```text
configs/libero_cream_cheese_offsets.json
src/saps/environments/libero_env.py
src/saps/environments/perturbations.py
tools/diagnostics/inspect_libero_scene.py
tools/diagnostics/preview_libero_perturbation.py
```

### Autonomous Execution and Logging

```text
src/saps/evaluation/runner.py
scripts/run_libero.py
scripts/run_autonomous_sweep.py
tools/monitoring/watch_autonomous_progress.py
```

### Deterministic Policy Sampling

```text
src/saps/policies/openpi_client.py
src/saps/policies/seeding.py
scripts/serve_seeded_policy.py
tools/diagnostics/probe_seeded_policy.py
compose.yml
```

---

## 8. Output Structure

A typical episode is stored under:

```text
outputs/<experiment>/
  <condition>/
    task_01/
      init_000/
        trial_000/
```

Episode artifacts include:

```text
01_nominal_initial.png
02_perturbed_before_settle.png
03_perturbed_after_settle.png
perturbation.json
steps.jsonl
summary.json
rollout_success.mp4
```

Sweep-level outputs include:

```text
schedule.json
sweep_summary.json
<condition>/run_summary.json
```

The `outputs/` directory is intentionally excluded from Git.

---

## 9. Autonomous-baseline conclusions

This stage established that:

1. the SAPS cream-cheese perturbations can be reproduced in the pinned LIBERO environment;
2. π0.5 performs reliably near the nominal object pose but degrades substantially for larger or directionally difficult offsets;
3. the autonomous evaluation can be executed and resumed robustly over hundreds of episodes;
4. deterministic policy sampling can provide matched autonomous randomness across future arbitration conditions;
5. full-episode reproducibility is achievable on the current hardware and software stack.

---

## 10. Subsequent simulation work

The next simulation work completed the human-input and action-arbitration stack:

1. browser keyboard input and seven-dimensional LIBERO action mapping;
2. pure teleoperation;
3. a common structured arbitration interface;
4. autonomous execution through that interface;
5. hard takeover with stale-policy rejection and post-release resynchronization;
6. fixed/equal blending;
7. cosine-similarity blending;
8. asynchronous policy inference with explicit non-stepping scheduler waits;
9. step-level human, policy, executed-action, weight, and latency logging.

The reproducibility infrastructure then added:

1. an immutable experiment manifest;
2. deterministic and counterbalanced episode schedules;
3. a resumable operator-session runner for all non-autonomous modes;
4. artifact validation and duplicate protection;
5. unified paired analysis across modes and perturbations;
6. latency characterization on the intended operator-experiment hardware.

The later matched multi-condition operator collection is complete and archived
in [`simulation_saps_baseline.md`](simulation_saps_baseline.md). The earlier
200-episode sweep, smoke episodes, and calibration runs retain their own
identities and must not be relabeled as matched-pilot results.

See:

- [`shared_autonomy.md`](shared_autonomy.md)
- [`experiment_protocol.md`](experiment_protocol.md)
- [`analysis.md`](analysis.md)
