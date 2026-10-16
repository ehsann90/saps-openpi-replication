FROM openpi_server:latest

# The locked OpenCV wheel needs these libraries to decode the DROID MP4 files.
RUN apt-get update && \
    apt-get install -y --no-install-recommends libgl1 libglib2.0-0 && \
    rm -rf /var/lib/apt/lists/*
