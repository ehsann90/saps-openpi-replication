# Formal Arbitration Experiment Protocol

**Status:** implemented for teleoperation and shared-autonomy operator sessions.
The autonomous condition continues to use the unattended autonomous sweep.

## 1. Purpose

The formal study will compare autonomous execution, pure teleoperation, hard
takeover, equal/fixed blending, and cosine-similarity blending under the
controlled LIBERO cream-cheese perturbations.

Functional pilot episodes are not part of the formal dataset. Formal collection
must use an immutable experiment manifest and a recorded repository commit.

## 2. Experimental unit

A matched experimental unit is identified by:

```text
condition_id
trial_index
initial_state_index
policy_episode_seed
```

The policy seed is derived independently of arbitration mode. This preserves the
same deterministic replan-noise sequence across modes while allowing subsequent
policy actions to diverge naturally after human input changes the state.

## 3. Conditions

The canonical conditions are defined only in:

```text
configs/libero_cream_cheese_offsets.json
```

They include `nominal` and `p01` through `p09`. The configuration records that
SAPS Appendix Table A1 lists nine offset pairs even though the main text refers
to eight perturbed configurations. The repository retains all listed pairs.

## 4. Modes

| Mode | Operator required | Policy required | Main comparison |
|---|---:|---:|---|
| `autonomous` | No | Yes | unassisted VLA baseline |
| `teleoperation` | Yes | No for control | full human baseline |
| `takeover` | Yes | Yes | abrupt human/policy switching |
| `fixed_blend` | Yes | Yes | equal action blending |
| `cosine_blend` | Yes | Yes | dynamic geometric agreement |

An operator must be present for every non-autonomous episode. Shared-autonomy
modes may include idle periods, but the operator must monitor the task and apply
the intervention protocol consistently.

## 5. Hardware and latency

Formal operator-assisted collection should run on the more capable system where
policy latency is small enough not to dominate the interaction. The runtime
already records inference latency and scheduler wait ticks, so residual latency
can be quantified rather than hidden.

Changing hardware must not change:

- repository and submodule revisions;
- checkpoint;
- task and perturbation configuration;
- policy seed protocol;
- initial state;
- control frequency;
- arbitration parameters;
- manifest ordering.

Record the hardware and software inventory described in [`setup.md`](setup.md).

## 6. Manifest

The session runner will consume an immutable JSON manifest similar to:

```json
{
  "schema_version": 1,
  "experiment_id": "saps_libero_operator_v1",
  "repository_commit": "<commit>",
  "conditions": ["nominal", "p01", "p02"],
  "modes": [
    "autonomous",
    "teleoperation",
    "takeover",
    "fixed_blend",
    "cosine_blend"
  ],
  "trials_per_condition": 20,
  "initial_state_index": 0,
  "environment_seed": 7,
  "policy_base_seed": 20260724,
  "fixed_autonomy_weight": 0.5,
  "cosine_gain": 6.0,
  "control_frequency_hz": 20.0,
  "autonomous_max_steps": 280,
  "operator_max_steps": 1200,
  "ordering_seed": 20260801
}
```

The finalized manifest is copied into the output root and must not be edited
after data collection begins. Changes require a new experiment ID.

Start from
`configs/operator_experiment_manifest.example.json`, copy it to a new file, and
replace `repository_commit` with the exact clean commit returned by
`git rev-parse HEAD`. The operator runner rejects commit mismatches and dirty
repositories for formal collection.

## 7. Episode schedule

The generated schedule should store one row per episode:

```text
episode_id
mode
condition_id
trial_index
initial_state_index
policy_episode_seed
order_index
status
attempt_count
output_directory
started_at
finished_at
termination_reason
success
```

Requirements:

- deterministic schedule generation;
- counterbalanced mode and condition order;
- resumable execution;
- no overwrite of completed episodes;
- explicit operator acknowledgement between episodes;
- validation of artifacts before marking an episode complete;
- recorded aborts, timeouts, and invalid attempts;
- a preserved history of reruns.

The session runner must launch one browser-mediated episode at a time. It should
not attempt unattended loops for operator-controlled conditions.

Generate or resume the session with the policy server already running:

```bash
make operator-session \
  MANIFEST=configs/operator_experiment_manifest.json \
  SESSION_OUTPUT=outputs/saps_libero_operator_v1
```

The first invocation freezes `manifest.json`, creates `schedule.json`, and asks
for acknowledgement before every episode. Completed valid episodes are skipped
on resume. Every attempt receives a unique directory; invalid attempts are
retained and never overwritten.

Each schedule row derives `policy_episode_seed` with the same
`make_policy_episode_seed` inputs used by `run_autonomous_sweep.py`:

```text
policy_base_seed
condition_id
trial_index
task_id
initial_state_index
```

Arbitration mode is deliberately excluded. Consequently, a condition/trial
unit has one seed shared by autonomous, teleoperation, takeover, fixed blending,
and cosine blending.

## 8. Operator procedure

Before a collection session:

1. verify `make check`;
2. record the repository and submodule commits;
3. start the policy server and confirm the checkpoint;
4. run one excluded warm-up episode;
5. verify browser focus and key release;
6. calibrate the operator and document the intervention instruction;
7. start or resume the immutable schedule.

The intervention policy must be written before formal data collection. Examples
include intervening only when predicted failure is apparent or continuously
steering toward task success. Mixing strategies within one experiment would
confound arbitration-mode comparisons.

## 9. Episode validity

An episode is valid only if:

- the manifest identity matches the output path and summary;
- the expected policy seed is present where applicable;
- action arrays are finite and seven-dimensional;
- the mode-specific arbitration fields are present;
- policy replan indices are consistent;
- the operator was connected and armed when required;
- termination reason is recorded;
- required JSON logs are readable;
- no prior completed output was overwritten.

An operator abort can be a valid recorded outcome or an excluded attempt, but
the choice must be declared in the analysis plan.

## 10. Primary outcomes

Candidate episode-level outcomes:

- official LIBERO task success;
- simulated completion time and control steps;
- wall-clock completion time;
- intervention time and fraction;
- number and duration of intervention segments;
- mean and distribution of effective autonomy weight;
- cosine-similarity distribution;
- policy inference latency and wait duration;
- invalid, aborted, and timed-out episode rates.

Condition-level and paired mode comparisons should use the full matched key, not
file order.

## 11. Data handling

`outputs/` and `results/` are intentionally ignored by Git. Archive formal data
with:

- manifest and generated schedule;
- repository and submodule revisions;
- hardware/software inventory;
- session logs;
- episode summaries and step logs;
- analysis configuration and generated tables;
- checksums for immutable artifacts.
