# Simulation SAPS Baseline

## Purpose and status

The completed LIBERO stage reproduces and explains the SAPS shared-autonomy
baseline sufficiently to support transfer to a fixed physical robot. It
established π0.5/OpenPI deployment, deterministic evaluation, runtime and
inference-latency characterization, calibrated SpaceMouse input, Fixed and
Cosine action blending, human recovery of autonomous failures, and a complete
logging and analysis pipeline.

This work is not currently being developed as an independent SAPS-method paper.
The matched experiment is a descriptive excluded pilot and infrastructure
baseline, not a powered comparison of arbitration methods.

The project progression is:

```text
Autonomous π0.5 baseline
        ↓
Robustness and perturbation characterization
        ↓
Latency and runtime characterization
        ↓
SpaceMouse human-input integration
        ↓
Fixed and Cosine shared autonomy
        ↓
Matched descriptive LIBERO pilot
        ↓
Physical SAPS deployment
```

Historical identifiers such as Gate-1, Gate-2, and Gate-2 v2 are retained in
frozen commands, source constants, protocols, and output roots where changing
them would break reproducibility. Gate-1 identifies runtime/autonomous
characterization, while Gate-2 identifies the completed matched shared-autonomy
simulation pilot. Future-facing documentation uses descriptive names.

## Reference implementation

The simulation reference includes:

- deterministic π0.5 sampling through the pinned OpenPI server;
- the frozen LIBERO cream-cheese task and planar perturbations;
- autonomous, takeover, Fixed, and Cosine arbitration;
- calibrated SpaceMouse Cartesian input and gripper commands;
- stale-result rejection and explicit asynchronous policy waits;
- separate human, policy, arbitration, and executed-action records;
- manifest-driven, resumable collection with immutable attempt history; and
- read-only coverage, matching, accounting, timing, and arbitration analysis.

The [runtime semantics](shared_autonomy.md), [human-input guide](human_input.md),
and [testing guide](testing.md) describe the implementation. The historical
[matched-pilot protocol](gate2_operator_pilot.md) records the frozen scheduler,
validity rules, and artifact layout without renaming its provenance identities.

## Completed matched pilot

The collection contains 60 outcomes:

| Mode | Outcomes |
|---|---:|
| Autonomous | 20 |
| Fixed | 20 |
| Cosine | 20 |
| Total | 60 |

Conditions are `nominal`, `p02`, `p06`, and `p09`, with five trials per
condition-mode cell. Condition, trial, initial state, policy episode seed, and
seed protocol form 20 exact matched triplets.

Validation is final:

- 60/60 outcomes are analyzable;
- 20/20 exact matched triplets are complete;
- `collection_complete = true`;
- `analysis_valid = true`; and
- there are no blocking validation errors.

The generated [final report](../results/gate2_shared_autonomy_pilot_v2/REPORT.md)
contains the descriptive result tables, human-effort and arbitration
diagnostics, accounting findings, limitations, and conclusion. Its CSV and JSON
companions remain the machine-readable source for those derived summaries.

## Immutable provenance

| Identity | Value |
|---|---|
| Collection commit | `d4013d7998b9843bf1e1a5fb25c7bbce515d0fdb` |
| Accounting-analysis commit | `2d2d8fe5efa0a59a05ce8e59a6814f1c1895209f` |
| OpenPI submodule commit | `15a9616a00943ada6c20a0f158e3adb39df2ccac` |
| LIBERO submodule commit | `f78abd68ee283de9f9be3c8f7e2a9ad60246e95c` |
| Policy configuration | `pi05_libero` |
| Checkpoint | `gs://openpi-assets/checkpoints/pi05_libero` |
| SpaceMouse profile SHA-256 | `3bae6c547e2eec8d33c68a860d65eea4c0b1c39c7fb993dd2f033323b0994afc` |
| Shared manifest SHA-256 | `61c3d346af87ffdef16b378fed9383a395b3d27947eabf768da1bd314491383a` |
| Autonomous experiment ID | `saps_libero_gate2_autonomous_pilot_v2` |
| Autonomous protocol SHA-256 | `47d84fed0dcb1909d9d99412af2515989fe46559e0e9fb0a06bbf70d1d10bd18` |
| Perturbation config SHA-256 | `43c88fe649362303ec599c6397155380d0de1ece84dbdcf614a2a952829447c5` |

These are canonical experiment identities recorded or validated by the
collectors; they are not raw file-byte hashes. Both raw collections record the
collection commit. The later analysis commit changed only asynchronous policy
request/result accounting and remains explicitly distinguishable.

Frozen raw roots:

```text
outputs/gate2_shared_autonomy_pilot_v2
outputs/gate2_autonomous_pilot_v2
```

Tracked lightweight derived archive:

```text
results/gate2_shared_autonomy_pilot_v2
```

The raw roots contain frozen manifests, schedules, provenance, attempt histories,
step and scheduler-wait logs, summaries, and media. They must not be regenerated,
renamed, edited, or overwritten. The derived archive is regenerated read-only
from those roots with `make gate2-analysis`.

## Accepted descriptive interpretation

| Condition | Autonomous | Fixed | Cosine |
|---|---:|---:|---:|
| nominal | 5/5 | 5/5 | 5/5 |
| p02 | 5/5 | 5/5 | 3/5 |
| p06 | 2/5 | 4/5 | 5/5 |
| p09 | 0/5 | 1/5 | 5/5 |
| Overall | 12/20 (60%) | 15/20 (75%) | 18/20 (90%) |

These are descriptive pilot observations only. Fixed recovered some autonomous
failures while preserving the autonomous successes in this sample. Cosine
recovered all eight matched autonomous failures. Two `p02` identities that
succeeded autonomously failed under Cosine, but this does not establish an
intrinsic regression: the experiment cannot distinguish operator input,
intervention timing, post-intervention policy state, arbitration,
trajectory-specific effects, or their interactions.

The experiment covers one LIBERO task, four selected perturbations, five
repetitions per cell, one nonexpert operator, and one SpaceMouse interface. It
contains no multi-operator study. That operator profile is not by itself
inconsistent with the SAPS within-benchmark setup, but the task, participant,
and sample coverage are insufficient for a paper-level method comparison.

## Timing and policy accounting

The reported `human_active_duration_seconds` and `human_active_fraction` use
only actual environment/control steps. Human input during a frozen simulation
policy wait is instead represented by
`human_active_policy_wait_ticks`, `human_active_policy_wait_seconds`, and
`human_active_policy_wait_fraction`. Including this wait activity changes the
overall intervention picture only modestly and does not explain the roughly
50% step-based Fixed and Cosine fractions.

Those fractions should not be compared directly with the roughly 10.8% and 30%
SAPS LIBERO-PRO rates because the benchmark and protocol differ. SAPS real-world
fractions are more comparable in magnitude, without implying exact equivalence.

Policy accounting found 23 shared episodes with complete asynchronous
request/result accounting and 17 with one terminal request still in flight.
Those terminal requests have no logged latency and are correctly excluded from
latency statistics; no trajectory is invalid. All 20 autonomous episodes have
complete synchronous accounting.

During simulation waits, robot state, environment state, and simulation time do
not advance, while wall clock does. On a physical robot, a conventional
chunked-VLA controller may hold the commanded state or stop motion during
inference, but wall clock and the physical external environment continue. This
stop/replan/continue behavior is the intended first physical baseline. Real-time
chunking or another continuous-inference extension may be evaluated later; RTC
is not part of this baseline.

## Physical SAPS next stage

The next implementation target is a fixed physical robot. This section is a
specification, not a claim of existing hardware support.

The baseline should provide:

- a physical fixed-arm interface;
- π0.5/DROID-compatible policy deployment;
- fixed and wrist/external camera observations as the available hardware permits;
- ordinary chunked policy inference;
- robot hold or stopped motion during inference when required;
- SpaceMouse Cartesian operator correction;
- Fixed `alpha = 0.5` and Cosine `k = 6` baselines;
- gripper arbitration consistent with SAPS and the robot's command convention;
- latency and policy-wait logging; and
- separate operator, policy, arbitration, and executed-action logging for
  controlled real-world perturbation tasks.

Physical safety supervision must be independent of learned policy confidence,
arbitration weight, and operator activity. The deployment should use workspace
and velocity limits, force/torque or collision supervision where available, an
emergency stop, and all robot-native safety mechanisms. Hardware-specific
limits and stop behavior must be validated before shared-autonomy trials.

The initial physical milestone is functional and reproducible deployment of the
ordinary chunked baseline. It should not introduce RTC, new arbitration
parameters, or research extensions before the reference behavior is stable.

## Research after the physical baseline

Once physical SAPS is stable, subsequent work addresses:

1. short-horizon autonomous-continuation risk;
2. collaboration evidence, recovery, assistance reduction, and autonomy
   resumption; and
3. selective learning from intervention.

The simulation archive provides infrastructure and reference behavior for those
questions; it does not answer them.
