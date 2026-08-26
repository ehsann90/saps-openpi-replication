# Formal Arbitration Experiment Protocol

The fixed 60-outcome Gate-2 v2 matched pilot is implemented separately in
[`gate2_operator_pilot.md`](gate2_operator_pilot.md). Its operator scheduler
contains 40 Fixed/Cosine episodes and its dedicated autonomous sweep contains
20 matched episodes. Gate 2 reuses the infrastructure below but is not the
final powered experiment described by this document and must remain labeled as
pilot data.

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

Formal operator-assisted collection must run on hardware that satisfies the
validated latency requirements in
[`gate1_rtx5080_ac_performance.md`](gate1_rtx5080_ac_performance.md). The runtime
records inference latency and scheduler wait ticks, so residual latency can be
quantified rather than hidden.

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

The session runner consumes an immutable JSON manifest. The shared-autonomy
manifest currently has this structure:

```json
{
  "schema_version": 3,
  "experiment_id": "saps_libero_shared_autonomy_v2",
  "config_path": "configs/libero_cream_cheese_offsets.json",
  "conditions": ["nominal", "p01", "p02"],
  "modes": [
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
  "operator_max_steps": 280,
  "fine_translation_gain": 0.25,
  "fine_rotation_gain": 0.1,
  "normal_translation_gain": 0.5,
  "normal_rotation_gain": 0.2,
  "fast_translation_gain": 1.0,
  "fast_rotation_gain": 0.4,
  "default_speed_mode": "normal",
  "ordering_seed": 20260801
}
```

The finalized manifest is copied into the output root and must not be edited
after data collection begins. Changes require a new experiment ID.

The version-controlled protocols are
`configs/operator_teleoperation_manifest.json` and
`configs/operator_shared_autonomy_manifest.json`. They are separate because
pure teleoperation and corrective shared autonomy impose different operator
attention requirements. Commit any protocol changes before collection. The
Make target rejects a dirty worktree, obtains the exact Git commit on the host,
and records it alongside the frozen manifest in `repository_provenance.json`.
The session also freezes the perturbation configuration path, contents, and
canonical hash in `perturbation_config.json`; `session_protocol.json` records
any required versioned protocol guard. Git is therefore not required in the
runtime container. On resume, the immutable schedule fields are checked against
deterministic regeneration while attempt status and history are preserved.

When `INPUT_SOURCE=spacemouse`, the Python session runner requires an explicit
profile path. Make supplies `configs/spacemouse_profile.json` by default through
`SPACEMOUSE_PROFILE`. The session validates and freezes that profile in
`human_input.json`, then passes the same path to every teleoperation, takeover,
fixed-blend, and cosine-blend child episode. Manifest gains remain the keyboard
speed profile; they do not replace calibrated SpaceMouse gains. The runtime
device path is recorded separately. Resuming with different profile contents,
profile path, or device path is rejected.

## 7. Episode schedule

The generated schedule stores one row per episode:

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
attempts
termination_reason
success
```

Each attempt records its number, start/finish timestamps, return code, output
root, summary path, validity, analysis selection, redo status, and error.

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

Generate or resume the teleoperation session without a policy server:

```bash
make teleoperation-session \
  INPUT_SOURCE=spacemouse \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick
```

Run shared autonomy separately with the policy server already running:

```bash
make shared-autonomy-session \
  INPUT_SOURCE=spacemouse \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick
```

The first invocation freezes `manifest.json`, creates `schedule.json`, and asks
for acknowledgement before every episode. Completed valid episodes are skipped
on resume. Every attempt receives a unique directory; invalid attempts are
retained and never overwritten.

Formal autonomous and operator experiments share a 280-step horizon at 20 Hz,
equal to 14 simulated seconds. The earlier `v1` operator pilot used a 1200-step
horizon. Pilot episodes that completed by step 280 are dynamically unaffected
by that larger ceiling, but later completions are not directly comparable.

Redo a completed scheduled episode without overwriting its earlier attempt by
passing its exact ID from `schedule.json`:

```bash
make shared-autonomy-session \
  REDO_EPISODES=trial_000__condition_p08__mode_cosine_blend
```

Multiple comma-separated IDs are accepted. A successful redo becomes the
attempt selected for analysis; every previous attempt remains recorded with
`selected_for_analysis: false`. An invalid redo does not silently replace the
last valid selected attempt.

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

The SAPS paper also excludes the gripper from motion arbitration and applies
`max(policy_gripper, operator_gripper)`. Under the LIBERO convention
`-1 = open` and `+1 = close`, either source can initiate closing, while a
conflict is intentionally biased toward closing. An operator Open command
therefore cannot override a simultaneous policy Close command.

## 8. Operator procedure

Before a collection session:

1. verify `make check`;
2. record the repository and submodule commits;
3. start the policy server and confirm the checkpoint;
4. run one excluded warm-up episode;
5. run the calibrated SpaceMouse diagnostic and verify the profile/device;
6. verify browser focus, arming, and SpaceMouse stale-input behavior;
7. calibrate the operator and document the intervention instruction;
8. start or resume the immutable schedule.

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
