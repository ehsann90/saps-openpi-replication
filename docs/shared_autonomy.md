# Shared-Autonomy Runtime

This document describes the Phase 2 autonomous, hard-takeover, and
fixed-weight blending runtime for the SAPS-OpenPI replication.

## Paper-defined arbitration semantics

SAPS uses seven-dimensional expert and VLA actions. The first six
dimensions are translational and rotational end-effector motion. The
seventh dimension controls the gripper.

Motion is blended as:

```text
executed_motion =
    alpha * autonomous_motion
    + (1 - alpha) * human_motion
```

`alpha` is the autonomy weight. Human motion is active when the L2 norm
of the first six human-action dimensions exceeds `0.001`.

The paper's equal-blending condition uses `alpha = 0.5` while human
motion is active and `alpha = 1.0` while human motion is idle. This
replication generalizes the active-human coefficient through
`--fixed-autonomy-weight`, retaining `0.5` as the default.

The gripper is not blended. It follows the SAPS closing-biased rule:

```text
executed_gripper = max(autonomous_gripper, human_gripper)
```

with this project's `-1=open`, `+1=close` convention.

## Supported arbitration modes

### `autonomous`

The OpenPI action is executed. Human input is logged but does not alter
the robot command.

### `takeover`

When operator motion is idle, the OpenPI action is executed. When
operator motion is active, human motion is executed. Policy requests are
paused during active takeover, stale pre-takeover inference is rejected,
and fresh inference is required after release.

### `fixed_blend`

When operator motion is idle, effective autonomy is `1.0`. When operator
motion is active, the configured fixed autonomy weight is used.

The policy worker continues replanning while human motion is active.
LIBERO advances only when a valid buffered policy action exists. No
fabricated zero policy action is blended or executed.

Cosine-similarity blending remains intentionally disabled.

## Shared-control states

The runtime logs:

- `autonomous`: executing buffered policy actions in autonomous or
  takeover-idle operation;
- `human_takeover`: executing active operator motion in takeover mode;
- `policy_wait`: waiting for normal policy inference outside fixed
  blending;
- `takeover_resync`: waiting for fresh post-takeover inference;
- `fixed_blend`: executing a fixed human-policy blend;
- `fixed_blend_policy_wait`: retaining browser responsiveness while
  waiting for policy data required by fixed blending.

LIBERO advances only in `autonomous`, `human_takeover`, and
`fixed_blend`.

## Replanning and latency semantics

The OpenPI `pi05_libero` model predicts an action horizon of 10. The
official evaluator executes the first five actions before replanning, so
this project retains `replan_steps=5`.

At 20 Hz, five actions cover 0.25 seconds. If inference takes longer,
the conservative runtime pauses LIBERO between chunks while keeping the
browser scheduler responsive. It does not reuse an invalidated chunk or
insert artificial hold actions.

Deterministic episode seeds and monotonically increasing replan indices
are preserved across arbitration modes.

## Logging

Every LIBERO step records human, autonomous, and executed actions;
activity state and motion norm; configured and effective autonomy
weights; policy action freshness; replan and chunk indices; inference
timing; worker state; generation; and result disposition.

For compatibility, `autonomy_weight` remains an alias of
`effective_autonomy_weight`.

`scheduler_waits.jsonl` records non-stepping browser ticks, including the
wait state, human activity, worker state, and the weight that would apply
once fresh policy data is available.

## Launch

Start the deterministic policy server:

```bash
make policy-server
```

Run fixed blending with the paper coefficient:

```bash
make fixed-blend \
  FIXED_AUTONOMY_WEIGHT=0.5 \
  CONDITION=nominal \
  TRIAL=0 \
  INITIAL_STATE=0 \
  SHARED_MAX_STEPS=280 \
  SPEED_MODE=fine \
  SHARED_OUTPUT=outputs/fixed_blend_validation
```

Equivalent explicit mode selection:

```bash
make shared-control \
  ARBITRATION_MODE=fixed_blend \
  FIXED_AUTONOMY_WEIGHT=0.5
```

Fixed-blend output paths include an `alpha_...` component so different
weights do not overwrite one another.

## Boundary validation

For active human motion, weight `1.0` reproduces autonomous motion,
weight `0.0` reproduces human motion, and weight `0.5` produces the
six-dimensional element-wise midpoint. Human-idle fixed blending always
uses effective autonomy weight `1.0`. The gripper remains independent.

## Existing behavior preserved

Autonomous and takeover retain matched deterministic policy seeds,
policy replan indices, stale-generation rejection, takeover-specific
inference pausing and resynchronization, genuine-policy-only environment
steps, and separate action and scheduler-wait logs.
