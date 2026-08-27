# LIBERO matched simulation pilot report

This completed pilot closes the LIBERO simulation-baseline stage of the SAPS–OpenPI replication. It establishes reference behavior and validates the deployment, operator-control, arbitration, logging, provenance, and analysis pipeline before transfer to a fixed physical robot. It is descriptive infrastructure evidence, not a powered comparison of arbitration methods.

## Experimental design

The frozen matched design contains 20 autonomous, 20 Fixed, and 20 Cosine outcomes. Conditions `nominal`, `p02`, `p06`, and `p09` each have five trials per mode. The 20 condition/trial identities are exact matched triplets with the same initial state and policy seed protocol. Fixed uses autonomy weight `0.5`; Cosine uses gain `k = 6`.

## Frozen provenance

| Identity | Frozen value |
|---|---|
| Collection commit | `d4013d7998b9843bf1e1a5fb25c7bbce515d0fdb` |
| Accounting-analysis commit | `2d2d8fe5efa0a59a05ce8e59a6814f1c1895209f` |
| OpenPI submodule | `15a9616a00943ada6c20a0f158e3adb39df2ccac` |
| LIBERO submodule | `f78abd68ee283de9f9be3c8f7e2a9ad60246e95c` |
| Policy checkpoint | `gs://openpi-assets/checkpoints/pi05_libero` |
| SpaceMouse profile SHA-256 | `3bae6c547e2eec8d33c68a860d65eea4c0b1c39c7fb993dd2f033323b0994afc` |
| Shared manifest SHA-256 | `61c3d346af87ffdef16b378fed9383a395b3d27947eabf768da1bd314491383a` |
| Autonomous protocol SHA-256 | `47d84fed0dcb1909d9d99412af2515989fe46559e0e9fb0a06bbf70d1d10bd18` |
| Perturbation config SHA-256 | `43c88fe649362303ec599c6397155380d0de1ece84dbdcf614a2a952829447c5` |

The collection commit is the repository revision recorded by both raw collections. The later accounting-analysis commit is kept separate because it changed validation of asynchronous request/result accounting, not the collected trajectories.

Raw data remain immutable at `outputs/gate2_shared_autonomy_pilot_v2` and `outputs/gate2_autonomous_pilot_v2`. This report and its CSV/JSON companions are derived artifacts.

## Coverage and validation

- Expected outcomes: `60`
- Analyzable outcomes: `60/60`
- Exact complete triplets: `20/20`
- Complete collection: `True`
- Analysis valid: `True`
- Blocking validation errors: `0`

## Descriptive success outcomes

| Condition | Autonomous | Fixed | Cosine |
|---|---:|---:|---:|
| nominal | 5/5 | 5/5 | 5/5 |
| p02 | 5/5 | 5/5 | 3/5 |
| p06 | 2/5 | 4/5 | 5/5 |
| p09 | 0/5 | 1/5 | 5/5 |
| Overall | 12/20 (60.0%) | 15/20 (75.0%) | 18/20 (90.0%) |

These are descriptive observations only; no statistical-significance claim is attached.

## Matched outcome interpretation

Fixed recovered some autonomous failures while preserving the autonomous successes in this small sample. Cosine recovered all eight matched autonomous failures. Two `p02` identities that succeeded autonomously failed under Cosine. Those two failures do not establish an intrinsic Cosine regression: operator input, intervention timing, post-intervention policy state, arbitration, trajectory-specific effects, and their interactions cannot be distinguished with this experiment.

## Human effort

`human_active_duration_seconds` and `human_active_fraction` use only actual environment/control steps. They exclude human input during policy-wait ticks, when the simulation is paused. That wait-period activity is logged separately.

| Metric | Fixed | Cosine |
|---|---:|---:|
| Mean step-based human-active fraction | 51.7% | 50.1% |
| Mean step-based human-active duration (s) | 5.4 | 4.7 |
| Mean correction segments per episode | 3.60 | 4.15 |
| Policy-wait duration, all episodes (s) | 72.6 | 70.9 |
| Human-active policy-wait duration (s) | 43.3 | 37.3 |
| Human-active share of policy-wait ticks | 59.6% | 52.6% |

Including wait-period activity modestly changes the overall intervention picture but does not explain the roughly 50% step-based rates. These fractions must not be compared directly with the roughly 10.8% and 30% LIBERO-PRO rates reported by SAPS, which use a different benchmark and protocol. SAPS real-world fractions are more comparable in magnitude, but this pilot does not establish exact cross-paper equivalence.

## Arbitration diagnostics

All `20/20` analyzable Fixed episodes used active-human autonomy weight `0.5` within absolute tolerance `1e-9`; no deviation was detected.

Cosine produced `1896` defined active weights and `0` undefined weights. Across active steps, the aggregate mean weight was `0.658`, with `14.5%` near zero, `42.7%` near one, and `42.8%` intermediate. These are implementation diagnostics, not evidence of method superiority.

## Timing semantics

In this simulation baseline, policy waits do not advance robot simulation state, environment state, or simulation/environment time; wall clock does advance. Simulated execution time is `control_steps / 20 Hz`, while wall-control and total wall time are retained separately. Autonomous synchronous inference also contributes to wall time.

For the next conventional physical chunked-VLA baseline, the robot may hold its commanded state or stop motion while waiting for the next action chunk, but the physical external environment and wall clock continue evolving. Stop/replan/continue is a valid baseline. Continuous execution methods such as real-time chunking may later reduce pauses, but RTC is outside this baseline.

## Policy accounting

Shared `policy_replan_count` counts submitted asynchronous requests. Completed/logged results must equal measured latency samples. A difference of one is accepted only for a contiguous final request logged as pending on the terminal step.

- Shared complete accounting: `23`
- Shared terminal-unobserved requests: `17`
- Autonomous synchronous complete accounting: `20`

The 17 terminal requests have no logged latency and are correctly excluded from latency statistics. This does not invalidate any raw trajectory. All 20 autonomous episodes have complete synchronous accounting.

## Limitations

- One LIBERO task.
- Four selected perturbation conditions.
- Five repetitions per condition-mode cell.
- One nonexpert operator and one SpaceMouse interface.
- No multi-operator study.
- Descriptive excluded pilot, not a powered method comparison.

A single nonexpert operator is not by itself inconsistent with the SAPS within-benchmark setup, but this sample and its task and participant coverage are insufficient for a paper-level comparison of arbitration methods.

## Conclusion and transition

The simulation study established a reproducible π0.5/SAPS baseline and validated the complete shared-autonomy execution and logging pipeline. Autonomous performance degraded under larger object-position perturbations, while Fixed and Cosine action blending enabled recovery in several cases where autonomous execution failed. Because the pilot used a single task, one operator, selected perturbations, and five repetitions per condition-method cell, it is not intended as a powered comparison between arbitration methods. It provides reference behavior and deployment infrastructure for the next stage: ordinary chunked-VLA SAPS on a fixed physical robot, followed by research on autonomous-continuation risk, intervention, recovery, autonomy resumption, and selective learning from intervention.

## Warnings

- This is a descriptive excluded pilot, not a powered experiment.
- 17 shared episodes ended with one terminal policy request still in flight; those requests have no logged latency and are excluded from latency statistics.
