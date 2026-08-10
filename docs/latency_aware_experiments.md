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

## Mock-operator development sweep

The following single-run development observations use
`configs/mock_operator_trace_example.json`, cosine blending with gain 6, a
nominal condition, matched policy noise within each frequency, and a 14-second
simulated horizon. The trace supplies a +x intervention for 15 executed
control steps. They are diagnostic only; no scheduler setting is selected from
them.

| Frequency | Scheduler | Handoff | Result | Steps | Freeze estimate | Wall time | Rejected plans |
|---:|---|---:|---|---:|---:|---:|---:|
| 20 Hz | strict pause | 0 | success | 157 | 13.50 s | 21.31 s | 0 |
| 20 Hz | latency-aware | 0 | timeout | 280 | 9.35 s | 23.35 s | 0 |
| 20 Hz | latency-aware, 3 cm translation gate | 0 | success | 218 | 15.00 s | 25.87 s | 10 |
| 10 Hz | strict pause | 0 | timeout | 140 | 5.70 s | 19.70 s | 0 |
| 10 Hz | latency-aware | 0 | timeout | 140 | 0.50 s | 14.50 s | 0 |
| 10 Hz | latency-aware | 1 | timeout | 140 | 0.50 s | 14.50 s | 0 |
| 10 Hz | latency-aware | 3 | timeout | 140 | 0.60 s | 14.60 s | 0 |
| 10 Hz, time-matched trace | strict pause | 0 | timeout | 140 | 9.30 s | 23.30 s | 0 |
| 10 Hz, time-matched trace | latency-aware, prefetch 5, 4 cm gate | 1 | timeout | 140 | 5.30 s | 19.30 s | 6 |

At 20 Hz, the ten-action policy horizon covers only 0.5 seconds, while
observed inference takes roughly 0.7 seconds. Consequently, plans arrived only
after the active buffer exhausted and the 20 Hz handoff values never activated.
Latency-aware nevertheless reduced waiting but changed the trajectory enough to
lose task completion. Tightening the translation gate to 3 cm restored success,
but produced more waiting than strict pause, so it does not satisfy the target
tradeoff.

At 10 Hz, the buffer covers one second and the handoffs did activate: 0, 16,
and 36 executed handoff actions for settings 0, 1, and 3 respectively. None of
the one-run trace conditions completed, and their autonomous-action RMSE from
the matched strict trace was approximately 0.57. Thus the current latency-aware
settings do not yet preserve strict-pause behavior closely enough for selection.

The trace is control-step indexed. It provides identical interventions for
scheduler comparisons at one frequency, but must be retimed when comparing
different frequencies at the same simulated time.

A final focused 10 Hz check retimed the intervention to 2.0--2.8 simulated
seconds, matching the 20 Hz trace. Strict pause still timed out. A later
prefetch (five actions remaining) and a 4 cm translation gate reduced waiting,
but also timed out, rejected six plans, and never reached a motion handoff.
This is not progress toward the strict-pause success/freeze target, so no
further parameter sweeps were run.

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

1. Compare strict and latency-aware scheduling with equal simulated horizons,
   matched policy seeds, and identical mock inputs.
2. Report freeze duration, rejected-plan rate, plan age, overshoot, path length,
   success, and completion time—not success alone.
3. Keep exclusive GPU scheduling as a separate system-control factor.
4. Treat any operator-priority or predictive-state design as a new method, not
   a SAPS replication result.

## Deterministic mock-operator sweeps

The shared-autonomy runner accepts `--mock-operator-trace <path>` and skips the
browser connection and arming workflow. This is for matched development sweeps,
not for human-subject data. The supplied trace is copied into each episode
directory as `mock_operator_trace.json` and its source path is recorded in
`summary.json`.

Trace actions are direct seven-dimensional LIBERO actions. A segment starts at
`start_control_step` (inclusive) and ends at `end_control_step` (exclusive).
The trace advances only after a successful environment step, so scheduler waits
do not shift an intervention. Segments must be ordered, non-overlapping, and
contain finite values in `[-1, 1]`. Any gap supplies idle motion with an open
gripper.

```json
{
  "trace_format_version": 1,
  "segments": [
    {
      "start_control_step": 40,
      "end_control_step": 55,
      "action": [0.10, 0.00, 0.00, 0.00, 0.00, 0.00, -1.0],
      "label": "translation_intervention"
    }
  ]
}
```

For example, use a new output directory for each combination:

```bash
make cosine-blend \
  MOCK_OPERATOR_TRACE=configs/mock_operator_trace_example.json \
  SCHEDULER_MODE=latency_aware \
  CONTROL_FREQUENCY_HZ=20 \
  SHARED_MAX_STEPS=280 \
  SHARED_OUTPUT=outputs/mock_latency_aware_20hz
```

Local result directories are ignored by Git. The table records the summaries
currently stored under `outputs/scheduler_comparison/`,
`outputs/handoff_comparison/`, and `outputs/cosine_latency_aware_*`.
