# Frozen Matched-Pilot Protocol (historical Gate-2 v2)

This lower-level record preserves the immutable protocol historically named
Gate-2 v2. Collection and validation are complete: 60/60 outcomes are analyzable,
20/20 exact matched triplets are present, `collection_complete = true`, and
`analysis_valid = true` with no blocking errors. The experiment compares
autonomous, Fixed, and Cosine execution with five repetitions per
condition-mode cell. It is descriptive evidence, not a powered method study.

The [Simulation SAPS Baseline](simulation_saps_baseline.md) is the canonical
project-level archive and interpretation. This page retains historical names,
commands, and paths because they are part of frozen provenance.

The earlier `saps_libero_gate2_operator_pilot_v1` design and
`configs/gate2_operator_pilot_manifest.json` are retained only as superseded
history. No v1 outcome or autonomous result collected on another machine is a
formal v2 input.

## Frozen identities and design

| Collection | Experiment ID | Output root | Episodes |
|---|---|---|---:|
| Shared | `saps_libero_gate2_shared_autonomy_pilot_v2` | `outputs/gate2_shared_autonomy_pilot_v2` | 40 |
| Autonomous | `saps_libero_gate2_autonomous_pilot_v2` | `outputs/gate2_autonomous_pilot_v2` | 20 |

| Provenance identity | Value |
|---|---|
| Collection commit | `d4013d7998b9843bf1e1a5fb25c7bbce515d0fdb` |
| Accounting-analysis commit | `2d2d8fe5efa0a59a05ce8e59a6814f1c1895209f` |
| OpenPI | `15a9616a00943ada6c20a0f158e3adb39df2ccac` |
| LIBERO | `f78abd68ee283de9f9be3c8f7e2a9ad60246e95c` |
| Checkpoint | `gs://openpi-assets/checkpoints/pi05_libero` |
| Shared manifest SHA-256 | `61c3d346af87ffdef16b378fed9383a395b3d27947eabf768da1bd314491383a` |
| Autonomous protocol SHA-256 | `47d84fed0dcb1909d9d99412af2515989fe46559e0e9fb0a06bbf70d1d10bd18` |
| Perturbation config SHA-256 | `43c88fe649362303ec599c6397155380d0de1ece84dbdcf614a2a952829447c5` |

The shared manifest is
`configs/gate2_shared_autonomy_pilot_manifest.json`. The autonomous parameters
are independently frozen before shared collection in
`configs/gate2_autonomous_pilot_protocol.json`.

Both collections use exactly:

- conditions `nominal`, `p02`, `p06`, and `p09`;
- trials `0` through `4`, initial state `0`, and environment seed `7`;
- policy base seed `20260724` and `saps-policy-seed-v1` derivation;
- LIBERO `libero_object` task 1 and the committed perturbation configuration;
- policy configuration `pi05_libero` and checkpoint
  `gs://openpi-assets/checkpoints/pi05_libero`;
- 5-step replanning, 10 settling steps, 20 Hz LIBERO control semantics, and a
  280 environment/control-step horizon.

Shared execution additionally freezes fixed autonomy weight `0.5`, cosine gain
`6.0`, ordering seed `20260825`, and ordering method
`gate2_v2_two_mode_counterbalance_v1`.

The v2 manifest has one keyboard translation gain and one keyboard rotation
gain. It does not contain fine/normal/fast alternatives. Gate-2 uses the
SpaceMouse, so these keyboard values are not applied. SpaceMouse motion comes
directly from `configs/spacemouse_profile.json`: translation `0.40`, rotation
`0.08`, profile SHA-256
`3bae6c547e2eec8d33c68a860d65eea4c0b1c39c7fb993dd2f033323b0994afc`.
The profile does not multiply manifest keyboard gains.

## Shared counterbalancing

The shared schedule is five consecutive eight-episode trial rounds. Every
round contains each Fixed/Cosine by condition cell exactly once. Across the
five trials, Fixed precedes Cosine two or three times independently for every
condition. The deterministic backtracking interleaver also enforces:

- no consecutive episodes from the same condition;
- no run longer than two episodes from the same mode;
- at least one intervening episode between Fixed and Cosine for the same
  condition/trial; and
- exact regeneration from the ordering seed.

Policy seed derivation excludes arbitration mode. The combined preflight
validator constructs 20 autonomous identities without reading autonomous
outputs and proves that Autonomous, Fixed, and Cosine form exactly 20 triplets
matched on condition, trial, initial state, policy episode seed, and seed
protocol.

## Environment-step and wall-clock semantics

The autonomous and shared runners use the same `create_libero_task` path. The
pinned LIBERO environment defaults to a 20 Hz control frequency. Both runners
reset, restore the same saved initial state, apply the same planar perturbation,
and execute 10 dummy settling steps before control. Both use 5-step policy
chunks, and one recorded control step means exactly one `env.step` call. Their
task horizon is therefore the same 280 environment/control steps.

Autonomous policy inference is synchronous: inference completes before its
next environment step. Shared policy inference is asynchronous, but scheduler
wait ticks do not call `env.step`; simulated progression pauses until a fresh
policy action is available. The shared 20 Hz scheduler also paces operator wall
time. Autonomous is not given artificial sleeps. Consequently the modes have
matched simulated-step semantics but can have different human-observed wall
durations.

Analysis keeps these quantities separate:

- environment/simulated execution time: `control_steps / 20 Hz`;
- wall-control and total wall time: measured elapsed runtime;
- shared waiting: wait ticks, reconstructed events, nominal wait duration,
  wait wall duration, wait fraction, inference latency, and human activity
  during waits; and
- autonomous inference: per-replan inference latency from step logs and its
  contribution to autonomous wall time.

Shared wait ticks never count as simulated steps or simulated task time.

The reported `human_active_duration_seconds` and `human_active_fraction` also
use only environment/control steps. Human activity during policy waits is
separate in `human_active_policy_wait_ticks`,
`human_active_policy_wait_seconds`, and
`human_active_policy_wait_fraction`. Including that wait activity does not
explain the roughly 50% step-based intervention fractions. These values should
not be compared directly with SAPS LIBERO-PRO intervention rates because the
benchmark and protocol differ.

This simulation pause does not imply that a physical external environment can
freeze. A conventional physical chunked-VLA baseline may hold or stop the robot
while waiting for inference, but wall clock and the external environment
continue. That ordinary stop/replan/continue behavior is the next baseline;
real-time chunking is not part of it.

## Non-launching preflights

Run the shared preflight with the intended stable device path:

```bash
make gate2-preflight \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick
```

It validates all immutable identities and ordering rules, prints all 40 shared
rows and each matched autonomous seed, and reports the complete 60-outcome
design. It does not open the device, create outputs, or contact the policy
server.

Run the separate autonomous preflight:

```bash
make gate2-autonomous-preflight
```

It prints all 20 condition/trial/initial-state/seed rows and verifies task,
perturbation, environment, seed, replan, settling, frequency, and horizon
settings. It does not depend on an autonomous output directory.

## Historical collection and resume commands

The commands below document how the frozen collections were produced. Do not
rerun them against the completed roots or reuse the frozen experiment IDs.

Commit the complete protocol first and start the seeded policy server in a
separate terminal. The server handshake advertises its policy configuration and
checkpoint; both collectors reject a server that differs from the v2 protocol.

Collect or resume the 40 operator episodes:

```bash
make gate2-session \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick
```

The session runner waits for confirmation before every episode. Enter `q` to
stop. Rerunning the same command resumes the immutable schedule. Gate-2 accepts
only a success within 280 steps or a complete 280-step timeout. Disconnect,
disarm, abort, environment termination, or invalid runtime output requires an
explicit retained redo:

```bash
make gate2-session \
  SPACEMOUSE_DEVICE=/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Wireless-event-joystick \
  REDO_EPISODES=trial_000__condition_p02__mode_fixed_blend
```

After all shared episodes, collect or resume the dedicated autonomous sweep:

```bash
make gate2-autonomous
```

The target fixes all 20 cells and rejects condition overrides, protocol drift,
or a non-clean checkout. It freezes the protocol, perturbation config, exact
schedule, and repository commit before creating an environment. Compatible
completed summaries are skipped. An existing incompatible completed summary is
an error and is never silently overwritten.

## Reproducible final analysis

Run analysis at any point:

```bash
make gate2-analysis
```

The analyzer retains all 60 planned rows and can still diagnose a partial copy,
but the archived roots are complete. It reports observed coverage independently
by mode.
`matched_triplets.csv` contains only identities with all three valid outcomes;
missing episodes are never fabricated or replaced by near matches. A seed or
identity mismatch is blocking.

Outputs under `results/gate2_shared_autonomy_pilot_v2` include:

```text
episode_metrics.csv
mode_summary.csv
condition_mode_summary.csv
matched_triplets.csv
fixed_blend_diagnostics.csv
cosine_blend_diagnostics.csv
policy_wait_summary.csv
policy_accounting_diagnostics.csv
validation_report.json
REPORT.md
```

`policy_accounting_diagnostics.csv` preserves the runtime distinction between
submitted requests and completed/logged results. For shared asynchronous modes,
`summary.policy_replan_count` is the number of requests submitted to the worker;
latency statistics include only results returned through `poll()` and written to
an episode log. Completed results must have a one-to-one measured-latency count.
Submitted minus completed must be zero, except that one final contiguous request
is accepted when the terminal step records that exact request as still pending.
The accepted terminal request remains explicit in
`terminal_unobserved_request_count` and has no fabricated latency. Autonomous
episodes use separate synchronous replan accounting and require exact equality
between submitted replans, logged replans, and measured latencies.

Historical shared scheduler-wait records contain worker state and latency but
not request/result replan-index fields. Therefore
`logged_request_submission_count` counts explicit step-record submission events;
the initial request (normally index `0`) is evidenced by its contiguous logged
result. Validation reconstructs the request sequence from the summary submitted
count plus the deduplicated step/wait evidence, while still rejecting duplicate
results, non-contiguous results, unexplained gaps, or executed actions without a
logged result.

Fixed blending is audited at alpha `0.5` with absolute tolerance `1e-9`.
Cosine diagnostic thresholds remain predeclared: near zero `alpha <= 0.10`,
near one `alpha >= 0.90`, and material consecutive change
`|delta alpha| >= 0.05`. Results remain descriptive pilot evidence.

The final generated report records the accepted success table, matched
interpretation, human-effort semantics, arbitration diagnostics, accounting
clarification, limitations, and transition to physical SAPS. It is generated by
the analyzer rather than edited independently.
