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

The existing deterministic scheduler is reused. It forms the 12
mode-condition units, shuffles that base list once with `ordering_seed`, and
uses a cyclic left shift by the trial index for each of five trial rounds.
Every round therefore contains all 12 units exactly once, every
mode-condition-trial cell occurs once, and the five rounds begin with different
units. Recreating the schedule from the same manifest produces identical order,
episode IDs, and seeds.

Policy seeds use `saps-policy-seed-v1` and depend only on the policy base seed,
task ID, initial-state index, condition, and trial. Arbitration mode is excluded.
Teleoperation retains this seed as matching metadata; fixed and cosine modes
execute with the same seed as each other and as the corresponding existing
autonomous trial.

## Preflight

Run preflight from a clean checkout of the intended collection commit:

```bash
make gate2-preflight \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick
```

Preflight validates the exact manifest, perturbation-config identity, committed
calibration profile and hash, device argument, output namespace, schedule
coverage, uniqueness, deterministic regeneration, and matched policy seeds. It
prints all 60 rows as schedule index, mode, condition, trial, and policy seed.
It does not open the SpaceMouse, create a session output, contact the policy
server, or launch an episode.

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
and ordering seed. Resume rejects immutable schedule drift or changed
provenance instead of mixing runs.

Do not commit generated Gate-2 outputs. Gate-2 analysis is intentionally outside
the scope of this protocol implementation.
