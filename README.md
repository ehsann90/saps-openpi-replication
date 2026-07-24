# SAPS OpenPI Replication

A methodological replication of:

**SAPS: Shared Autonomy for Policy Steering by Blending Teleoperation
with a Pretrained VLA**

The first objective is to reproduce the simulation experiments using:

- Physical Intelligence OpenPI
- the `pi05_libero` checkpoint
- LIBERO
- Robosuite
- MuJoCo

The subsequent objective is to implement the SAPS action-level
arbitration methods:

- autonomous execution
- pure teleoperation
- takeover
- fixed 50/50 blending
- cosine-similarity blending

## Current status

The nominal OpenPI-LIBERO pipeline has been validated locally.

- OpenPI policy server: working
- `pi05_libero` checkpoint: working
- LIBERO / Robosuite simulation: working
- NVIDIA GPU inference: working
- EGL rendering: working
- LIBERO-Object smoke test: 10/10 successful episodes

The SAPS arbitration methods and paper-specific object perturbations have
not yet been implemented.

## Tested environment

- Ubuntu 24.04.4 LTS
- NVIDIA GeForce RTX 5080 Laptop GPU, 16 GB
- NVIDIA driver 595.71.05
- Docker 29.1.3
- Docker Compose 2.40.3
- OpenPI commit:
  `15a9616a00943ada6c20a0f158e3adb39df2ccac`
- LIBERO commit:
  `f78abd68ee283de9f9be3c8f7e2a9ad60246e95c`

## Clone

```bash
git clone --recurse-submodules <repository-url>
cd saps-openpi-replication
```

For an existing clone:

```bash
git submodule update --init --recursive
```


## Patch

The pinned LIBERO runtime uses Python 3.8. Some legacy source packages require Python-3.8-compatible build tools.

```bash
./scripts/apply_openpi_patch.sh
```

Then build the runtime:

```bash
cd third_party/openpi
docker compose -f examples/libero/compose.yml build runtime
```

## Checkpoint

The checkpoint is not stored in this repository.

Expected cache location:

```bash
~/.cache/openpi/openpi-assets/checkpoints/pi05_libero
```

It can be synchronized from the public bucket using:

```bash
gcloud storage rsync \
  gs://openpi-assets/checkpoints/pi05_libero \
  "$HOME/.cache/openpi/openpi-assets/checkpoints/pi05_libero" \
  --recursive \
  --checksums-only
```

## Baseline evaluation

```bash
cd third_party/openpi

SERVER_ARGS="--env LIBERO" \
CLIENT_ARGS="--args.task-suite-name libero_object \
--args.num-trials-per-task 1 \
--args.replan-steps 5" \
docker compose -f examples/libero/compose.yml up
```

## Cleanly Stopping Server

```bash
cd ~/MyProjects/saps-openpi-replication/third_party/openpi

docker compose \
  -f examples/libero/compose.yml \
  down
```

## Repository structure

```bash
saps-openpi-replication/
├── docs/
├── patches/
├── scripts/
└── third_party/
    └── openpi/
```
