# Offline physical observation video

From SAPS, after a completed run:

```bash
PYTHONPATH=src .venv-physical/bin/python tools/analysis/export_physical_video.py \
  outputs/physical_c4/c4_dev_long_20261111T083244Z \
  --output-dir outputs/physical_c4/derived_UNIQUE_ID/videos \
  --fps 10 --side-by-side
```

Requires the existing NumPy/OpenCV stack and its MP4 encoder; no ROS, GPU,
policy server or robot connection. The new output directory must be outside
the original run. Existing output is rejected. Generates `wrist.mp4`,
`exterior.mp4`, optional `side_by_side.mp4` (wrist left), and
`video_manifest.json`. Every MP4 is decoded to verify frame count after writing.
Failed encodes retain an explicit failed manifest; use a new destination to retry.

The input is `request_*/native_rgb.npz` plus `request.json`. These are already
decoded RGB arrays, even for BGRA8 camera messages; native array hashes and
shapes are verified before export. OpenCV receives BGR only after explicit RGB
conversion. Source files and timestamps remain unchanged.

Saved images are sparse policy-request observations, not 30 Hz acquisition or
15 Hz execution footage. Each stream uses its own source ROS timestamps on a
common constant-fps grid starting at the later first camera timestamp. At each
tick, the latest past observation is held without interpolation. The endpoint
is rounded up to the next grid tick, whose frame lasts one output period.
Asynchronous cameras are aligned by source clock, not asserted to be hardware
synchronized. Labels show request index, source time and held duration.

The manifest records hashes, timestamps, output fps, per-frame source selections,
stream checksums, duplicates, skipped request indices and unavailable frames.
Missing, invalid, nonadvancing or inconsistent camera evidence fails explicitly;
no unrelated or synthetic frames fill gaps. Separate source-clock starts can
exclude an early sample; the manifest reports this. Large gaps remain visible
as long holds. A single sample exports one frame.

The November 11 run contains 19 usable native camera pairs. Its three derived
videos are in `outputs/physical_c4/review_20261111/videos/`: 209 frames each at
10 fps, 190 held duplicates per stream, no dropped input samples. This supports
qualitative review, not precise motion or task-completion timing. No continuous
recorder was needed or added. Future higher-rate video would require a separate
optional bounded asynchronous recorder with source stamps and drop accounting;
it must remain outside the controller/inference timing path.
