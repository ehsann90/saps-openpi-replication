# Gate 1 RTX 5080 AC/Performance Characterization

## Status and conclusion

This report records the 2026-08-21 Gate 1 latency and asynchronous-scheduler
characterization of the RTX 5080 Laptop GPU with AC power and the firmware
performance profile explicitly verified. It supersedes the earlier RTX 5080
run as the primary laptop comparison because that run did not record these two
power-state controls.

The confirmed AC/performance state approximately halved RTX 5080 inference
latency relative to the earlier exploratory run. It did not eliminate scheduler
waiting: the unchanged five-action, 20 Hz scheduler waited six ticks at nearly
every policy boundary. The supplied RTX 5090 measurements remain substantially
faster and less distorted by scheduler waiting.

## Experimental provenance

| Item | Recorded value |
|---|---|
| Repository branch | `main` |
| Repository commit | `70b6d2b1832e2788da4d9e9d51276d5b990f28d4` |
| `origin/main` | `70b6d2b1832e2788da4d9e9d51276d5b990f28d4` |
| OpenPI commit | `15a9616a00943ada6c20a0f158e3adb39df2ccac` |
| LIBERO commit | `f78abd68ee283de9f9be3c8f7e2a9ad60246e95c` |
| OpenPI checkpoint | `pi05_libero` |
| GPU | NVIDIA GeForce RTX 5080 Laptop GPU, 16,303 MiB |
| NVIDIA driver | `595.84` |
| Docker server | `29.1.3` |
| Docker Compose | `2.40.3` |
| Runtime CUDA | `12.2.2` |
| `/sys/class/power_supply/ADP0/online` | `1` |
| `/sys/firmware/acpi/platform_profile` | `performance` |

The two power-state values were checked before and after Gate 1A, before and
after Gate 1B, and after stopping the policy server. Both remained unchanged.
They are also stored in the generated Gate 1A and Gate 1B JSON metadata.

The local `pi05_libero` cache occupied approximately 12 GB. Server startup logs
confirmed checkpoint restoration, LIBERO normalization-stat loading, CUDA/JAX
initialization, and the listener on port 8000. The health endpoint returned
`OK` before measurement.

Initial idle GPU state was P4, 29.28 W, 49 degrees Celsius, 17 percent
utilization, and 1,020 MiB used. With the checkpoint loaded it was P4, 32.03 W,
52 degrees Celsius, 19 percent utilization, and 13,153 MiB used. No GPU power,
clock, or firmware settings were changed during collection.

## Fixed policy and environment configuration

| Parameter | Value |
|---|---|
| Task suite / task | `libero_object` / task 1 |
| Task description | `pick up the cream cheese and place it in the basket` |
| Condition / trial | `nominal` / 0 |
| Initial state | 0 |
| Environment seed | 7 |
| Policy base seed | 20260724 |
| Derived policy episode seed | 1594108130 |
| Replan steps | 5 |
| Control frequency | 20 Hz |
| Observation / policy image size | 256 / 224 |
| Maximum Gate 1B control steps | 280 |

The production `OpenPiLiberoPolicy`, deterministic server protocol,
`AsyncPolicyWorker`, `run_shared_episode_loop`, autonomous arbitration, and
LIBERO environment construction were used unchanged. Temporary scripts under
`/tmp` performed external instrumentation and supplied a connected, armed,
zero-motion operator for Gate 1B. No tracked source was modified for data
collection.

## Deterministic seeded probe

The seeded probe passed all checks:

| Probe | Result |
|---|---|
| Same observation, seed, and replan | Bitwise-identical actions |
| Different episode seed | Different actions |
| Different replan index | Different actions |
| First action hash | `b201ea1fcc337bedd5d6e42d49e2bebc1ff84becbe11b0ca98a99efb4d4ae23b` |
| Different-seed hash | `e7b125dcb326283005e552cae60ddb38b09956150e70528c43f635d1ac069e30` |
| Next-replan hash | `ddbdc30ba6ce71e4f07276505dd465e2ebea02349798e5c2a94078818365f63a` |

The first hash exactly matched the supplied RTX 5090 comparison hash.

## Gate 1A: steady-state latency

The benchmark used one fixed nominal observation after ten LIBERO settling
steps. Ten policy calls were treated as warm-up and excluded, followed by 200
measured calls with sequential replan indices. The first server compilation
occurred during the earlier seeded probe and was not included.

All table values are milliseconds.

| Metric | n | Mean | Min | p50 | p95 | p99 | Max |
|---|---:|---:|---:|---:|---:|---:|---:|
| Server inference | 200 | 397.324 | 348.992 | 396.493 | 419.374 | 431.413 | 455.360 |
| Client-to-client | 200 | 399.370 | 351.543 | 398.563 | 421.754 | 433.571 | 457.617 |
| Non-inference end-to-end | 200 | 2.046 | 1.344 | 2.005 | 2.655 | 2.876 | 3.063 |
| Client packing | 200 | 0.096 | 0.067 | 0.092 | 0.126 | 0.134 | 0.139 |
| Client unpacking | 200 | 0.044 | 0.032 | 0.043 | 0.055 | 0.062 | 0.070 |
| Transport/server residual | 200 | 1.872 | 1.166 | 1.831 | 2.480 | 2.686 | 2.853 |
| Observation preprocessing | 200 | 1.178 | 1.044 | 1.074 | 1.629 | 2.810 | 3.218 |

The reported quantities are defined as:

```text
non_inference_e2e = client_to_client - server_inference

transport_server_residual =
    raw_websocket_total
    - server_inference
    - client_pack
    - client_unpack
```

The residual is not pure network latency. It includes loopback and WebSocket
transport, server request unpacking, deterministic noise generation, response
packing, scheduling, and related protocol work. Model inference overwhelmingly
dominates the end-to-end result.

## GPU telemetry and thermal behavior

`nvidia-smi` sampled GPU state at approximately 200 ms intervals throughout
Gate 1A. The post-warm-up measurement window contained 400 samples.

| Metric | Mean | Min | p50 | p95 | p99 | Max |
|---|---:|---:|---:|---:|---:|---:|
| Temperature, degrees Celsius | 59.43 | 57 | 59 | 61 | 61 | 61 |
| GPU utilization, percent | 97.62 | 16 | 100 | 100 | 100 | 100 |
| Power, W | 79.32 | 64.96 | 79.43 | 80.89 | 81.33 | 81.47 |
| VRAM, MiB | 13,842.8 | 13,131 | 13,836 | 13,899 | 14,091 | 14,099 |
| SM clock, MHz | 1,034.6 | 742 | 1,057 | 1,230 | 1,290 | 1,477 |
| Memory clock, MHz | 14,001 | 14,001 | 14,001 | 14,001 | 14,001 | 14,001 |

All selected samples reported P0. The 16 percent utilization minimum was a
measurement-boundary sample; the median and upper percentiles were 100 percent.

The long-run thermal check did not show progressive clock or latency decline:

| Run quartile | Mean temperature | Mean SM clock | Median SM clock | Mean server latency |
|---|---:|---:|---:|---:|
| Q1 | 59.42 C | 1,028 MHz | 1,035 MHz | 396.77 ms |
| Q2 | 59.86 C | 1,030 MHz | 1,050 MHz | 397.73 ms |
| Q3 | 59.28 C | 1,050 MHz | 1,091 MHz | 398.22 ms |
| Q4 | 59.16 C | 1,030 MHz | 1,057 MHz | 396.58 ms |

Temperature remained at or below 61 degrees Celsius. Fourth-quartile clocks
were comparable to first-quartile clocks, and fourth-quartile latency did not
increase. The temperature/SM-clock correlation was `-0.346`, but temperature
varied over only four degrees and there was no time-dependent degradation.

## Gate 1B: asynchronous scheduler behavior

Gate 1B ran the real asynchronous five-step scheduler against the warm policy
server. Environment construction, reset, ten settling steps, checkpoint load,
video writing, and analysis were outside `control_elapsed_seconds`. The initial
asynchronous request was submitted immediately before the control-loop timer.

| Metric | RTX 5080 AC/performance result |
|---|---:|
| Success / termination | Yes / `success` |
| Control / total simulation steps | 151 / 161 |
| Scheduler / wait ticks | 336 / 185 |
| Wait fraction | 55.06% |
| Waits per control step | 1.225 |
| Wait episodes | 31 |
| Wait ticks/episode, mean / p50 / p95 / max | 5.97 / 6 / 6 / 6 |
| Nominal wait duration, mean / p50 / p95 / max | 298.39 / 300 / 300 / 300 ms |
| Policy latency, mean / p50 / p95 / p99 / max | 302.52 / 303.22 / 312.33 / 323.56 / 328.10 ms |
| Policy replans | 31 |
| Control deadline misses | 0 |
| Simulated / wall control time | 7.550 / 16.761 s |
| Wall/sim ratio | 2.2200x |

The observed boundary pattern was:

```text
initial request -> 6 wait ticks -> execute 5 actions
periodic request -> normally 6 wait ticks -> execute 5 actions
```

One boundary waited five ticks and the other 30 boundaries waited six ticks.

## Comparison with the earlier RTX 5080 run

The earlier run is retained as exploratory evidence. It did not record the AC
adapter or platform-profile state, so the difference cannot be attributed to a
single setting with experimental certainty. Its much lower observed power and
SM clocks nevertheless show that it was not an equivalent hardware state.

| Metric | Earlier unconfirmed run | AC/performance run | Observed change |
|---|---:|---:|---:|
| Server inference p50 | 762.315 ms | 396.493 ms | 48.0% lower; 1.92x faster |
| Client-to-client p50 | 765.713 ms | 398.563 ms | 48.0% lower; 1.92x faster |
| Gate 1B policy p50 | 634.068 ms | 303.219 ms | 52.2% lower; 2.09x faster |
| Median wait ticks | 12 | 6 | Halved |
| Wait fraction | 71.78% | 55.06% | 16.72 percentage points lower |
| Wall/sim ratio | 3.5381x | 2.2200x | 37.3% lower |
| Mean GPU power | 54.87 W | 79.32 W | 44.6% higher |
| Median SM clock | 502 MHz | 1,057 MHz | 110.6% higher |

The AC/performance run is therefore the appropriate RTX 5080 result for future
hardware comparisons.

## Comparison with the supplied RTX 5090 reference

For latency quantities, the factor is:

```text
factor = RTX5080_latency / RTX5090_latency
```

Thus a factor above one states how many times faster the RTX 5090 was for that
quantity.

| Metric | RTX 5080 AC/performance | RTX 5090 | Factor |
|---|---:|---:|---:|
| Server inference p50 | 396.493 ms | 90.295 ms | 4.39x |
| Server inference p95 | 419.374 ms | 96.759 ms | 4.33x |
| Server inference p99 | 431.413 ms | 97.346 ms | 4.43x |
| Client-to-client p50 | 398.563 ms | 91.346 ms | 4.36x |
| Client-to-client p95 | 421.754 ms | 97.894 ms | 4.31x |
| Client-to-client p99 | 433.571 ms | 98.613 ms | 4.40x |
| Gate 1B policy p50 | 303.219 ms | 98.49 ms | 3.08x |
| Wait fraction | 55.06% | 29.11% | 1.89x |
| Median wait ticks per boundary | 6 | 2 | 3.00x |
| Wall/sim ratio | 2.2200x | 1.4049x | 1.58x |

The GPU telemetry is not power-equivalent: the RTX 5080 is a laptop GPU that
averaged approximately 79 W, while the supplied RTX 5090 desktop trace was
approximately 475--485 W. The comparison measures the complete systems in their
recorded operating states, not equal-power GPU silicon.

The RTX 5090 remains the better reference machine for evaluation throughput and
reduced wall-time distortion. It is not a wait-free real-time baseline: its
unchanged scheduler still waited two ticks per chunk and spent 29.11 percent of
ticks waiting.

## Generated artifacts

The following local directories contain the raw samples and derived reports:

```text
outputs/gate1_latency_rtx5080_ac_performance/
outputs/gate1b_scheduler_rtx5080_ac_performance_autonomous/
```

Important files include:

```text
latency_samples.jsonl
warmup_samples.jsonl
preprocessing_samples.jsonl
latency_report.json
gpu_trace.csv
gpu_report.json
seeded_probe/report.json

steps.jsonl
scheduler_waits.jsonl
episode_summary.json
scheduler_report.json
```

`outputs/` is intentionally ignored by Git. These paths describe the local raw
evidence used to prepare this tracked report; the generated data is not included
in the documentation commit. Archive it separately when preserving the study.

Non-fatal runtime warnings included missing optional Robosuite private macros,
absent LIBERO dataset directories, a Numba deprecation warning, and unavailable
ROCm/TPU JAX backends. CUDA inference, deterministic sampling, both benchmarks,
and JSON validation completed successfully. The initial and final tracked
worktree was clean, and the policy server was stopped after collection.
