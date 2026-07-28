# Shared-Autonomy Runtime

This document describes the Phase 2.3 autonomous and hard-takeover
runtime for the SAPS–OpenPI replication.

## Supported arbitration modes

The current implementation supports:

- `autonomous`
- `takeover`

Fixed-weight and cosine-similarity blending are intentionally not
enabled yet.

### Autonomous

The OpenPI action is executed. Human input may still be logged, but it
does not affect the robot.

### Takeover

When operator motion is idle, the OpenPI action is executed.

When operator translational or rotational motion is active, the human
motion command is executed. Gripper arbitration follows the SAPS rule
implemented by `ActionArbitrator`.

## Runtime architecture

The runtime has two execution contexts.

### Main thread

The main thread owns:

- LIBERO and MuJoCo
- browser input sampling
- arbitration
- environment stepping
- browser image publication
- step logging

### Policy worker

A persistent worker thread owns OpenPI inference.

Only one inference request may be active at a time.

In hard-takeover mode, new inference requests are paused while human
motion is active. An inference result based on a pre-takeover
observation is discarded if it becomes stale.

After the operator releases control, a fresh inference request is made
from the post-intervention observation.

## Shared-control states

The logged `shared_control_state` has four possible values.

- `autonomous`: executing genuine buffered policy actions
- `human_takeover`: executing operator motion
- `policy_wait`: waiting for normal initial or periodic policy inference
- `takeover_resync`: waiting for fresh inference after human release

LIBERO advances only in `autonomous` and `human_takeover`.

During `policy_wait` and `takeover_resync`, the browser scheduler
continues running, but the simulation does not receive artificial hold
actions.

## Replanning semantics

The OpenPI `pi05_libero` model predicts an action horizon of 10.

The official OpenPI LIBERO evaluator uses `replan_steps=5`: it executes
the first five actions from the predicted chunk and then requests a new
chunk from the latest observation.

This project retains that default for baseline compatibility.

At a 20 Hz control rate, five actions represent 0.25 seconds of
simulated control. If inference takes longer than 0.25 seconds, the
current conservative runtime pauses simulation between policy chunks.

## Launch

Start the deterministic policy server:

```bash
make policy-server
```

In another terminal, run hard takeover:

```bash
make shared-control \
  ARBITRATION_MODE=takeover \
  CONDITION=nominal \
  TRIAL=0 \
  INITIAL_STATE=0 \
  SHARED_MAX_STEPS=280 \
  SPEED_MODE=fine \
  SHARED_OUTPUT=outputs/shared_autonomy
```

Open the displayed operator URL and click **Arm controls**.

## Outputs

Each episode directory contains:

- summary.json: episode-level result and timing
- steps.jsonl: records for actual LIBERO environment steps
- scheduler_waits.jsonl: browser-scheduler ticks where LIBERO did not
  advance while waiting for policy inference
- perturbation.json: requested and measured perturbation
- rollout video and diagnostic images

## Timing interpretation

Two rates must be distinguished.

- **Operator scheduler frequency**

    The browser and keyboard scheduler targets 20 Hz, including during
    policy inference.

- **LIBERO step frequency**

    LIBERO advances only when a genuine policy action or active human
    action is available. Its wall-clock step frequency may therefore be
    lower than 20 Hz.

This behavior preserves the validated five-action OpenPI trajectory and
avoids inserting artificial zero-motion actions.

## Validated behavior

The current implementation has been validated with:

- a matched nominal autonomous regression that completed successfully;
- repeated hard takeover and release transitions;
- a 20 Hz operator scheduler;
- no policy requests during active takeover;
- stale pre-takeover result rejection;
- no arbitration violations;
- no artificial non-policy autonomous LIBERO steps.

## Known limitations

- Autonomous simulation pauses when inference exceeds the five-action 
  execution window.
- Hard takeover switches abruptly between full autonomy and full human 
  motion.
- Fixed and cosine blending are not yet implemented.
- Faster inference hardware can shorten policy waits but does not 
  remove the discontinuity inherent in hard takeover.