# Latency-Aware Scheduler Experiment Record

## Status and scope

This branch is an experimental extension, not the faithful SAPS replication
baseline. The baseline remains `strict_pause` with five executed actions per
policy request. The latency-aware scheduler explores whether buffered policy
actions can hide inference latency without predicting through operator input.

All results below are single-run development observations on the local ROG-G16
system. They are not powered comparisons and must not be reported as final
performance results.

## Mechanism

`latency_aware` submits inference while valid actions remain in the current
chunk. Simulation continues using those buffered actions. A returned plan is
accepted only if generation, age, translation, rotation, and gripper-freshness
checks pass. Accepted plans may use a motion handoff; if the buffer is exhausted
first, the conservative `pause` fallback still freezes simulation.

This does not predict operator inputs or advance a model-derived state. Every
request uses an observed state, but the observation can become stale while the
operator and buffered policy actions continue changing the environment.

## Runs and observations

| Run | Result | Steps | Simulated time | Control wall time | Replans |
|---|---:|---:|---:|---:|---:|
| Strict, 20 Hz, replan 5 | success | 170 | 8.50 s | 25.12 s | 35 |
| Latency-aware, 20 Hz, replan 10, handoff 3 | success | 227 | 11.35 s | 16.67 s | 24 |
| Latency-aware, 10 Hz, replan 10, handoff 3 | timeout | 280 | 28.00 s | 28.00 s | 37 |
| Handoff 0, 10 Hz | success | 207 | 20.70 s | 20.62 s | 27 |
| Handoff 1, 10 Hz | success | 216 | 21.60 s | 21.52 s | 28 |
| Handoff 3, 10 Hz | timeout | 280 | 28.00 s | 28.00 s | 36 |

The operator observed freezing in all scheduler comparisons: most prominently
in strict 20 Hz, less in latency-aware 20 Hz, and least at 10 Hz. Latency-aware
20 Hz showed more end-effector overshoot than 10 Hz. An early short-horizon
handoff comparison appeared to favor three steps, but a 280-step repeat made
handoff 3 the worst of 0, 1, and 3. This reversal means no handoff value has
been selected.

The 10 Hz rows above used 280 steps and therefore have a 28-second horizon.
They are not completion-time comparable with a 20 Hz, 280-step, 14-second run.
A fair 14-second comparison must use 140 steps at 10 Hz and 280 at 20 Hz.

## Reproduction commands

Representative latency-aware configuration:

```bash
make cosine-blend \
  CONTROL_FREQUENCY_HZ=20 \
  SCHEDULER_MODE=latency_aware \
  REPLAN_STEPS=10 \
  PREFETCH_REMAINING_ACTIONS=7 \
  HANDOFF_STEPS=3 \
  COSINE_GAIN=6 \
  SHARED_MAX_STEPS=280 \
  SHARED_OUTPUT=outputs/cosine_latency_aware_20hz
```

For a fair 10 Hz horizon, change the frequency to 10 and `SHARED_MAX_STEPS` to
140. Preserve separate output directories for every scheduler configuration.

## Required next work

1. Add a deterministic mock-operator trace so scheduler and arbitration modes
   receive repeatable interventions.
2. Add a noninteractive shared-autonomy runner for matched sweeps.
3. Compare strict and latency-aware scheduling with equal simulated horizons,
   matched policy seeds, and identical mock inputs.
4. Report freeze duration, rejected-plan rate, plan age, overshoot, path length,
   success, and completion time—not success alone.
5. Keep exclusive GPU scheduling as a separate system-control factor.
6. Treat any operator-priority or predictive-state design as a new method, not
   a SAPS replication result.

Local result directories are ignored by Git. The table records the summaries
currently stored under `outputs/scheduler_comparison/`,
`outputs/handoff_comparison/`, and `outputs/cosine_latency_aware_*`.
