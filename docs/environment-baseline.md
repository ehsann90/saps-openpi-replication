# Environment Baseline

## Host

- OS: Ubuntu 24.04.4 LTS
- GPU: NVIDIA GeForce RTX 5080 Laptop GPU, 16 GB
- NVIDIA driver: 595.71.05
- Docker: 29.1.3
- Docker Compose: 2.40.3

## Pinned repositories

- OpenPI: `15a9616a00943ada6c20a0f158e3adb39df2ccac`
- LIBERO: `f78abd68ee283de9f9be3c8f7e2a9ad60246e95c`

## Compatibility change

The pinned OpenPI LIBERO Docker image creates a Python 3.8 environment.
Legacy source packages failed when temporary isolated environments used
incompatible build tools.

The stored patch:

`patches/openpi-libero-python38-build.patch`

installs Python-3.8-compatible Setuptools and Wheel versions and performs
the dependency synchronization without isolated source builds.

## Checkpoint

- Remote: `gs://openpi-assets/checkpoints/pi05_libero`
- Local file count: 16
- Approximate local size: 12 GB
- Verified with checksum-based `gcloud storage rsync`

## Smoke-test result

Task suite: `libero_object`

- Episodes: 10
- Successes: 10
- Success rate: 100%
- Replan steps: 5
- Approximate policy-server GPU memory: 12.2 GB
- Approximate complete GPU usage: 14 GB
