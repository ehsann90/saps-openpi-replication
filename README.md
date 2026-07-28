# SAPS–OpenPI Replication

An independent, research-oriented replication of **SAPS: Shared Autonomy for
Policy Steering by Blending Teleoperation with a Pretrained VLA** using the
OpenPI `pi05_libero` policy, LIBERO, Robosuite, and MuJoCo.

> Crystal Zhou, Jehan Yang, Douglas J. Weber, and Zackory Erickson,
> “SAPS: Shared Autonomy for Policy Steering by Blending Teleoperation with a
> Pretrained VLA,” arXiv:2606.15568, 2026.
>
> Paper: https://arxiv.org/abs/2606.15568

This repository is not an official implementation from the SAPS or OpenPI
authors. It focuses on methodological transparency, deterministic policy
sampling, controlled LIBERO perturbations, operator-visible shared autonomy,
and reproducible experiment tooling.

## Current status

The functional replication stack is complete through action-level arbitration.
Formal multi-condition, operator-assisted experiments and unified statistical
analysis are the next phase.

| Component | Status |
|---|---|
| Pinned OpenPI and LIBERO environment | Implemented and validated |
| `pi05_libero` policy server | Implemented and validated |
| SAPS cream-cheese perturbations | Implemented and validated |
| Deterministic per-episode and per-replan sampling | Implemented and validated |
| Autonomous baseline and resumable sweeps | Implemented |
| Browser keyboard teleoperation | Implemented and validated |
| Hard takeover | Implemented and validated |
| Fixed/equal action blending | Implemented and validated |
| Cosine-similarity blending | Implemented and validated |
| Manifest-driven operator experiment sessions | Planned for Phase 3 |
| Unified multi-mode analysis | Planned for Phase 3 |

The completed Phase 1 autonomous degradation study contains 200 episodes:
20 trials for the nominal condition and each of nine perturbation conditions.
The human-input and arbitration stack has passed unit, compilation, scheduler,
and live LIBERO validation, but those pilot runs are not formal statistical
experiments.

## What is reproduced

The repository implements the SAPS action-level comparison conditions:

- autonomous policy execution;
- pure teleoperation;
- hard takeover;
- fixed blending, with `0.5` as the paper coefficient;
- cosine-similarity blending, with gain `k = 6` as in the paper.

Human activity is detected from the first six action dimensions. Motion is
arbitrated separately from the gripper, which uses the SAPS closing-biased
`max()` rule under this project's `-1=open`, `+1=close` convention.

## Reproducibility baseline

| Dependency | Pinned revision |
|---|---|
| OpenPI | `15a9616a00943ada6c20a0f158e3adb39df2ccac` |
| LIBERO | `f78abd68ee283de9f9be3c8f7e2a9ad60246e95c` |
| Policy config | `pi05_libero` |
| Task suite | `libero_object` |
| Task | `pick up the cream cheese and place it in the basket` |

OpenPI is included as the `third_party/openpi` Git submodule. Project-specific
code remains in the outer repository; the retained OpenPI compatibility change
is stored as a patch rather than an undocumented modification.

## Quick start

### 1. Clone the complete repository

```bash
git clone --recurse-submodules \
  https://github.com/ehsann90/saps-openpi-replication.git
cd saps-openpi-replication

git submodule status
```

For an existing clone:

```bash
git submodule update --init --recursive
```

### 2. Apply the pinned compatibility patch and build images

```bash
make apply-patch
make build-images
```

The two expected local images are:

```text
libero
openpi_server
```

### 3. Run the automated checks

```bash
make check
```

### 4. Start the deterministic policy server

```bash
make policy-server
```

Keep this terminal running for autonomous or shared-autonomy modes. Pure
teleoperation and the browser input smoke test do not require the policy server.

### 5. Run one mode in a second terminal

```bash
# Autonomous smoke test
make autonomous-smoke CONDITION=nominal

# Pure teleoperation
make teleop CONDITION=nominal TRIAL=0 TELEOP_MAX_STEPS=1800

# Hard takeover
make takeover \
  CONDITION=nominal \
  TRIAL=0

# Equal/fixed blending
make fixed-blend \
  FIXED_AUTONOMY_WEIGHT=0.5 \
  CONDITION=nominal \
  TRIAL=0

# Cosine-similarity blending
make cosine-blend \
  COSINE_GAIN=6.0 \
  CONDITION=nominal \
  TRIAL=0
```

For operator-controlled modes, open the displayed browser URL and click
**Arm controls** before issuing commands.

## Documentation

- [Installation and environment setup](docs/setup.md)
- [Command runbook](docs/runbook.md)
- [Testing and validation](docs/testing.md)
- [Shared-autonomy semantics and runtime](docs/shared_autonomy.md)
- [Phase 1 perturbations and deterministic sampling](docs/phase1_libero_perturbations_and_determinism.md)
- [Formal experiment protocol](docs/experiment_protocol.md)
- [Analysis plan and current tools](docs/analysis.md)
- [Repository structure](docs/repository_structure.md)

Run `make help` for the compact command reference.

## Output policy

Generated episodes are written below `outputs/`; analysis products belong below
`results/`. Both directories are ignored by Git. Preserve experiment manifests,
software revisions, and output summaries outside the repository when archiving
formal studies.

## Citation

When using this replication, cite the original SAPS paper and identify the exact
repository commit used for the experiment. A machine-readable citation template
is provided in [`CITATION.cff`](CITATION.cff).
