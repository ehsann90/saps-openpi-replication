# Gate-2 Excluded Operator Pilot

Gate 2 is a 60-episode excluded pilot for validating the SAPS-style operator
baseline before physical-robot work. It is not a powered experiment, and its
outputs must not be presented as formal statistical evidence.

## Fixed protocol

The single source of schedule parameters is
`configs/gate2_operator_pilot_manifest.json`.

| Parameter | Fixed value |
|---|---|
| Experiment ID | `saps_libero_gate2_operator_pilot_v1` |
| Conditions | `nominal`, `p02`, `p06`, `p09` |
| Modes | `teleoperation`, `fixed_blend`, `cosine_blend` |
| Trials per mode-condition | `5`, with indices `0` through `4` |
| Total episodes | `60` |
| Initial state | `0` |
| Environment seed | `7` |
| Policy base seed | `20260724` |
| Fixed autonomy weight | `0.5` |
| Cosine gain | `6.0` |
| Control frequency | `20.0 Hz` |
| Collection horizon | `280` control steps |
| Ordering seed | `20260825` |
| SpaceMouse profile | `configs/spacemouse_profile.json` |
| Output root | `outputs/gate2_operator_pilot_v1` |

The keyboard-gain fields retained by the schema do not configure Gate-2
SpaceMouse motion. The validated SpaceMouse profile is authoritative.

## Unified counterbalanced schedule

Gate 2 uses the dedicated ordering method
`gate2_constrained_counterbalance_v1`; generic and earlier experiment schedules
retain their existing cyclic behavior. For each condition, the Gate-2 generator
shuffles all six permutations of the three modes and assigns five distinct
permutations to the five trial rounds. Taking five of all six permutations
guarantees that each pair of modes occurs in either direction two or three
times. A deterministic backtracking search then interleaves the four condition
queues within each round while preserving those mode orders.

Every 12-episode round contains all mode-condition units exactly once. Across
the complete 60-episode schedule:

- every pairwise mode precedence count is `2/5` or `3/5` per condition;
- consecutive episodes never use the same condition;
- a same-mode run contains at most two episodes; and
- the three modes for one condition/trial have at least one intervening
  episode between consecutive occurrences.

The ordering seed initializes both permutation selection and interleaving.
Recreating the schedule from the same manifest, repository code, and ordering
seed therefore produces the same order, episode IDs, and policy seeds. The
ordering-method identifier is stored in `schedule.json` and is immutable on
resume.

Policy seeds use `saps-policy-seed-v1` and depend only on the policy base seed,
task ID, initial-state index, condition, and trial. Arbitration mode is excluded.
Teleoperation retains this seed as matching metadata; fixed and cosine modes
execute with the same seed for a given condition/trial identity. This matching
does not depend on the current RTX 5080 autonomous outputs. Autonomous episodes
rerun on the test system can use the same protocol identity and seed derivation
when cross-dataset matching is required.

## Preflight

Run preflight from a clean checkout of the intended collection commit:

```bash
make gate2-preflight \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick
```

Preflight validates the exact manifest, perturbation-config identity, committed
calibration profile and hash, device argument, output namespace, schedule
coverage, uniqueness, deterministic regeneration, matched policy seeds, and all
ordering constraints. It reports maximum mode and condition runs, minimum
matched condition/trial separation, and every per-condition pairwise precedence
count. It then prints all 60 rows as schedule index, mode, condition, trial, and
policy seed. It does not open the SpaceMouse, create a session output, contact
the policy server, or launch an episode.

Device access and physical behavior must still be checked separately with
`make spacemouse-diagnostic` before collection.

## Collection and resume

Start the policy server in a separate terminal, then run:

```bash
make gate2-session \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick
```

The target requires a clean repository and fixes the manifest, output root,
input source, profile, and Gate-2 protocol identifier. It does not invoke
preflight or start the policy server automatically. The session runner displays
the next mode, condition, trial, and matched seed, then waits for explicit
operator confirmation before every episode. Enter `q` at the prompt to stop;
rerunning the same command resumes the frozen schedule.

Request a redo without deleting the earlier attempt:

```bash
make gate2-session \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick \
  REDO_EPISODES=trial_000__condition_p02__mode_fixed_blend
```

Only the newest valid redo remains selected for later analysis. Earlier valid,
failed, or aborted attempts remain in the attempt history.

## Frozen provenance

The session root records:

```text
manifest.json
schedule.json
human_input.json
perturbation_config.json
repository_provenance.json
session_protocol.json
session_events.jsonl
session_summary.json
attempts/
```

These files freeze the manifest and canonical hash, deterministic schedule,
repository commit, perturbation configuration path/contents/hash, SpaceMouse
profile path/contents/hash, runtime device path, and required protocol. The
manifest itself freezes the blending parameters, frequency, horizon, seeds,
ordering seed, and ordering-method identifier. Resume rejects immutable
schedule drift or changed provenance instead of mixing runs.

Do not commit generated Gate-2 outputs. Gate-2 analysis is intentionally outside
the scope of this protocol implementation.
