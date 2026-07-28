# Installation and Environment Setup

This guide describes the supported Docker workflow for the SAPS–OpenPI
replication. It also records the earlier OpenPI-first path from which the
replication environment was developed.

## 1. Recommended installation path

Clone the outer replication repository with its pinned OpenPI submodule:

```bash
git clone --recurse-submodules \
  https://github.com/ehsann90/saps-openpi-replication.git
cd saps-openpi-replication
```

If the repository was cloned without submodules:

```bash
git submodule update --init --recursive
```

Verify the recorded revisions:

```bash
git rev-parse HEAD
git submodule status
git -C third_party/openpi rev-parse HEAD
git -C third_party/openpi/third_party/libero rev-parse HEAD
```

Expected dependency revisions:

```text
OpenPI: 15a9616a00943ada6c20a0f158e3adb39df2ccac
LIBERO: f78abd68ee283de9f9be3c8f7e2a9ad60246e95c
```

Do not update the submodule during a formal experiment unless the experiment
manifest and documentation are also updated.

## 2. System requirements

The project uses NVIDIA GPU containers. A practical setup requires:

- a Linux host;
- an NVIDIA GPU and compatible driver;
- Docker Engine with the Compose plugin;
- NVIDIA Container Toolkit;
- enough local disk space for Docker images, the OpenPI checkpoint, and videos.

Upstream OpenPI currently lists more than 8 GB of GPU memory for inference and
recommends Docker for the LIBERO example. This replication was validated on:

```text
Ubuntu 24.04.4 LTS
NVIDIA GeForce RTX 5080 Laptop GPU, 16 GB
NVIDIA driver 595.71.05
Docker 29.1.3
Docker Compose 2.40.3
```

These versions are a validated configuration, not a strict requirement.

Confirm GPU access before building:

```bash
nvidia-smi
docker version
docker compose version

docker run --rm --gpus all \
  nvidia/cuda:12.2.2-base-ubuntu22.04 \
  nvidia-smi
```

## 3. Why OpenPI is a submodule

The work originally began by cloning OpenPI directly and validating its official
LIBERO Docker example:

```bash
git clone --recurse-submodules \
  https://github.com/Physical-Intelligence/openpi.git
cd openpi

git checkout 15a9616a00943ada6c20a0f158e3adb39df2ccac
git submodule update --init --recursive

SERVER_ARGS="--env LIBERO" \
  docker compose -f examples/libero/compose.yml up --build
```

That upstream-first workflow established that the checkpoint, policy server,
LIBERO client, MuJoCo rendering, and GPU inference worked together. The project
was then reorganized into this outer repository, with the validated OpenPI
revision retained as `third_party/openpi`. New experiment code therefore remains
separate from upstream OpenPI while the exact dependency history stays pinned.

New users should use the recommended outer-repository clone rather than manually
reconstructing this migration.

## 4. Apply the OpenPI compatibility patch

The pinned LIBERO image uses Python 3.8. Some legacy source packages require
Python-3.8-compatible build tooling, so the repository retains one explicit
patch:

```bash
make apply-patch
```

Equivalent command:

```bash
./patches/apply_openpi_patch.sh
```

The patch is idempotent. It reports whether it was newly applied or already
present. It must apply cleanly to the pinned OpenPI revision; a failure usually
means the submodule was changed.

## 5. Build the runtime and policy-server images

```bash
make build-images
```

Equivalent direct commands:

```bash
cd third_party/openpi

docker compose \
  -f examples/libero/compose.yml \
  build runtime openpi_server

cd ../..
```

The upstream Compose file names the resulting images `libero` and
`openpi_server`. The root `compose.yml` reuses those images while mounting the
outer replication repository at `/workspace`.

Inspect them with:

```bash
docker image inspect libero openpi_server >/dev/null
docker images | grep -E '^(libero|openpi_server)[[:space:]]'
```

## 6. Checkpoint cache

The `pi05_libero` checkpoint is not stored in Git. OpenPI downloads public
assets when needed and caches them under:

```text
~/.cache/openpi
```

The root Compose configuration mounts that cache into the policy-server
container. Override the host location with:

```bash
export OPENPI_DATA_HOME=/path/to/openpi-cache
```

Optional explicit synchronization:

```bash
gcloud storage rsync \
  gs://openpi-assets/checkpoints/pi05_libero \
  "$HOME/.cache/openpi/openpi-assets/checkpoints/pi05_libero" \
  --recursive \
  --checksums-only
```

The first policy-server start can take longer if the checkpoint must be fetched.
Do not include the checkpoint in this repository.

## 7. Rendering

The default is headless EGL:

```text
MUJOCO_GL=egl
PYOPENGL_PLATFORM=egl
```

If EGL initialization fails and the machine has a working X server, try GLX:

```bash
MUJOCO_GL=glx make operator-smoke
```

For local X11 workflows, the upstream example may require:

```bash
sudo xhost +local:docker
```

The browser operator itself is served over HTTP and WebSocket and does not
require the MuJoCo window to be displayed locally.

## 8. Remote operation

The operator server normally uses:

```text
HTTP:      8766
WebSocket: 8765
Policy:    8000
```

When the browser is on another machine, forward the operator ports:

```bash
ssh \
  -L 8766:127.0.0.1:8766 \
  -L 8765:127.0.0.1:8765 \
  user@remote-host
```

Then open:

```text
http://127.0.0.1:8766
```

The policy port is used by processes on the remote host and normally does not
need to be forwarded to the operator computer.

For formal operator studies, use a stable network path and verify that browser
focus, key-release events, and video updates are reliable before collecting
data.

## 9. Verify the installation

```bash
make check
make operator-smoke DURATION=30
```

Start the deterministic server in a dedicated terminal:

```bash
make policy-server
```

Then, in another terminal:

```bash
make seeded-probe CONDITION=nominal TRIAL=0
make autonomous-smoke CONDITION=nominal
```

Stop the server with:

```bash
make policy-stop
```

## 10. Moving to a more capable machine

A new experiment host should reproduce the software state, not copy an
untracked development environment:

```bash
git clone --recurse-submodules \
  https://github.com/ehsann90/saps-openpi-replication.git
cd saps-openpi-replication

git checkout <experiment-commit>
git submodule update --init --recursive
make apply-patch
make build-images
make check
```

Record at least:

```text
repository commit
OpenPI submodule commit
LIBERO submodule commit
GPU model and memory
NVIDIA driver
Docker and Compose versions
OPENPI_DATA_HOME
```

Policy latency is logged by the shared-autonomy runtime. A faster system should
reduce wall-clock waits, but it must not change the deterministic trial identity
or overwrite completed outputs.
