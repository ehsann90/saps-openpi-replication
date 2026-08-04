# Shared-Autonomy Runtime

This document describes the implemented action-level arbitration runtime for the
SAPS–OpenPI replication.

## 1. Reference

The implementation follows the comparison conditions described in:

> Crystal Zhou, Jehan Yang, Douglas J. Weber, and Zackory Erickson,
> “SAPS: Shared Autonomy for Policy Steering by Blending Teleoperation with a
> Pretrained VLA,” arXiv:2606.15568, 2026.
>
> https://arxiv.org/abs/2606.15568

SAPS blends real-time human commands with a pretrained policy at the action
level without retraining or modifying the policy architecture.

## 2. Action convention

Actions are seven-dimensional:

```text
[translation_xyz, rotation_xyz, gripper]
```

The first six dimensions are end-effector motion. The seventh controls the
gripper.

Human motion is active when:

```text
norm(human_action[:6]) > 0.001
```

The motion blend is:

```text
executed_motion =
    alpha * autonomous_motion
    + (1 - alpha) * human_motion
```

The gripper is handled independently:

```text
executed_gripper = max(autonomous_gripper, human_gripper)
```

This repository uses `-1=open` and `+1=close`, so `max()` lets either source
initiate closing and biases conflicts toward closing.

## 3. Modes

### `autonomous`

The current OpenPI action is executed. Human input may be logged but cannot
change the command.

### `takeover`

- human idle: execute autonomous motion;
- human active: execute human motion;
- pause new inference requests during takeover;
- invalidate stale pre-takeover policy results;
- request fresh policy inference after release.

### `fixed_blend`

- human idle: effective autonomy weight `1.0`;
- human active: use `--fixed-autonomy-weight`;
- paper comparison value: `0.5`;
- policy inference continues during human activity.

### `cosine_blend`

For active human motion:

```text
cosine_similarity =
    dot(human_motion, autonomous_motion)
    / (norm(human_motion) * norm(autonomous_motion))

alpha = sigmoid(cosine_gain * cosine_similarity)
```

The paper uses `cosine_gain = 6`. Aligned actions approach full autonomy,
opposing actions approach full human authority, and orthogonal actions give
`alpha = 0.5`.

Human-idle behavior remains full autonomy.

The paper does not specify the zero autonomous-motion norm case. This
replication uses neutral similarity `0.0`, resulting in `alpha = 0.5`, and logs:

```text
cosine_similarity_status = autonomous_motion_below_threshold
```

## 4. Asynchronous runtime

The main thread owns:

- LIBERO and MuJoCo;
- browser input sampling;
- arbitration;
- environment stepping;
- frame publication;
- step logging.

A persistent worker thread owns OpenPI inference. Only one inference request may
be pending at a time.

The policy predicts a ten-action horizon. The runtime executes five actions per
chunk to match the validated OpenPI LIBERO setup. At 20 Hz, those actions cover
0.25 simulated seconds.

If inference is slower than chunk consumption, the browser scheduler remains
responsive but LIBERO does not advance until a genuine policy action is
available. The runtime never fabricates a zero autonomous action for blending.

This distinction matters on slower hardware: wall-clock operator experience can
include visible waits even though simulated time is paused. Formal
operator-assisted experiments should use hardware where this latency is small,
while still retaining the logged latency and wait metrics.

### 4.1 Latency-aware experimental scheduler

The implementation history, preliminary local measurements, known comparison
confounds, and next experiments are recorded in
[`latency_aware_experiments.md`](latency_aware_experiments.md).

The strict scheduler remains the default reproduction baseline. An opt-in
latency-aware scheduler can request the next plan while buffered actions remain:

```bash
make fixed-blend \
  SCHEDULER_MODE=latency_aware \
  REPLAN_STEPS=20 \
  PREFETCH_REMAINING_ACTIONS=12
```

The active chunk continues during inference. A returned plan is accepted only
when its generation, age, end-effector translation and rotation, and gripper
divergence pass configured limits. Accepted plans use a short motion-action
handoff; rejected plans are logged and replanned. `EXHAUSTION_FALLBACK=pause`
retains the conservative freeze if inference outlasts the buffer. The
experimental `hold` fallback advances with zero autonomous motion and should
not be used as a formal SAPS condition without separate validation.

Relevant overrides are `MAX_PLAN_AGE_SECONDS`, `MAX_PLAN_TRANSLATION_M`,
`MAX_PLAN_ROTATION_RADIANS`, `MAX_PLAN_GRIPPER_DELTA`, and `HANDOFF_STEPS`.
Latency-aware results are not directly comparable with strict runs unless the
scheduler mode and all thresholds are included in the experiment manifest.

## 5. Shared-control states

| State | Meaning |
|---|---|
| `autonomous` | consuming buffered policy actions |
| `human_takeover` | executing active human motion in takeover mode |
| `policy_wait` | waiting for normal policy inference |
| `takeover_resync` | waiting for fresh inference after release |
| `fixed_blend` | executing fixed human-policy blending |
| `fixed_blend_policy_wait` | waiting for policy data required for fixed blending |
| `cosine_blend` | executing dynamic cosine blending |
| `cosine_blend_policy_wait` | waiting for policy data required for cosine blending |

LIBERO advances only while executing a valid action. Scheduler-wait states are
logged separately in `scheduler_waits.jsonl`.

## 6. Determinism and freshness

Every episode receives a deterministic policy seed derived from:

```text
base policy seed
task ID
initial-state index
condition ID
trial index
```

Arbitration mode is excluded from seed derivation. Replan indices increase once
per sampled policy chunk. Generation counters reject stale inference across
hard-takeover transitions.

Matched seeds do not force identical policy actions after human input changes
the state. They preserve the same deterministic noise sequence; observation
changes can and should produce different policy outputs.

## 7. Logging

Every LIBERO step records:

- human, autonomous, and executed actions;
- human activity and both motion norms;
- configured and effective autonomy weights;
- cosine gain, similarity, and status;
- policy action freshness and chunk position;
- policy seed, replan index, and latent-noise hash;
- inference latency and worker state;
- generation and result disposition;
- operator keys, gains, timing, and connection state;
- robot and object state;
- reward, success, and control timing.

`autonomy_weight` is retained as an alias of `effective_autonomy_weight` for
compatibility.

Wait records include the state, human activity, policy-worker status, latency,
and any weight that can be known without a fresh policy action. In active cosine
waits, the effective weight is intentionally `null` because the policy direction
is not yet available.

## 8. Launch commands

Start the policy server:

```bash
make policy-server
```

Hard takeover:

```bash
make takeover \
  CONDITION=nominal \
  TRIAL=0 \
  SHARED_OUTPUT=outputs/takeover_smoke
```

Fixed/equal blending:

```bash
make fixed-blend \
  FIXED_AUTONOMY_WEIGHT=0.5 \
  CONDITION=nominal \
  TRIAL=0 \
  SHARED_OUTPUT=outputs/fixed_blend_smoke
```

Cosine blending:

```bash
make cosine-blend \
  COSINE_GAIN=6.0 \
  CONDITION=nominal \
  TRIAL=0 \
  SHARED_OUTPUT=outputs/cosine_blend_smoke
```

Fixed output paths include `alpha_...`; cosine paths include `k_...`. This keeps
parameter variants separate.

## 9. Functional validation

The completed implementation has passed:

- automated arbitration and scheduler regression tests;
- full Python compilation;
- live weight `1.0`, `0.0`, and `0.5` fixed-blend equation checks;
- live cosine equation, similarity, norm, and dynamic-weight checks;
- continuous policy requests during active fixed and cosine input;
- takeover start, release, resynchronization, and stale-generation rejection;
- finite seven-dimensional action checks;
- explicit scheduler waits without artificial LIBERO steps.

These checks establish functional correctness. They do not replace the formal,
manifest-driven multi-condition experiment described in
[`experiment_protocol.md`](experiment_protocol.md).
