FROM mambaorg/micromamba:1.5.10@sha256:e3797091302382ea841498bc93a7b0a50f7c1448333d5e946d2d1608d0c5f43d

USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential git ca-certificates libgomp1 && \
    rm -rf /var/lib/apt/lists/*

COPY conda-lock-linux-64.txt /tmp/conda-lock-linux-64.txt
RUN micromamba create -y -n polymetis-pinned -f /tmp/conda-lock-linux-64.txt && \
    micromamba clean --all --yes

ARG FAIRO_REV=0a01a7fa7a7c65b2f9a3aebf5e79040940daf9d2
RUN git clone --filter=blob:none https://github.com/facebookresearch/fairo.git /opt/fairo && \
    cd /opt/fairo && git checkout --detach "$FAIRO_REV" && \
    test "$(git rev-parse HEAD)" = "$FAIRO_REV"

COPY patches/0001-include-cstddef.patch /tmp/0001-include-cstddef.patch
COPY patches/0002-pybullet-only-imports.patch /tmp/0002-pybullet-only-imports.patch
RUN cd /opt/fairo && git apply --check /tmp/0001-include-cstddef.patch && \
    git apply /tmp/0001-include-cstddef.patch && \
    git apply --check /tmp/0002-pybullet-only-imports.patch && \
    git apply /tmp/0002-pybullet-only-imports.patch

RUN micromamba run -n polymetis-pinned pip install --no-deps -e /opt/fairo/polymetis/polymetis
RUN cd /opt/fairo/polymetis/polymetis && \
    micromamba run -n polymetis-pinned /bin/bash -c \
    'cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_FRANKA=OFF -DBUILD_TESTS=OFF -DBUILD_DOCS=OFF && cmake --build build --target run_server -j 2'

RUN apt-get update && apt-get install -y --no-install-recommends procps && \
    rm -rf /var/lib/apt/lists/*

ENV PYTHONUNBUFFERED=1
WORKDIR /workspace
ENTRYPOINT ["micromamba", "run", "-n", "polymetis-pinned"]
