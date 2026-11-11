"""Offline, timestamped MP4 review of saved physical policy observations."""
from __future__ import annotations

from bisect import bisect_right
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np

ROLES = ('wrist', 'exterior')


def frame_plan(stamps: dict[str, list[float]], fps: float) -> dict:
    """Resample by holding the latest past frame on a common source-clock grid."""
    if not math.isfinite(fps) or not 0 < fps <= 120:
        raise ValueError('fps must be finite and in (0, 120]')
    for role in ROLES:
        values = stamps[role]
        if (not values or not all(math.isfinite(v) for v in values)
                or any(b <= a for a, b in zip(values, values[1:]))):
            raise ValueError(f'{role}: source timestamps must strictly advance')
    start = max(stamps[role][0] for role in ROLES)
    end = max(stamps[role][-1] for role in ROLES)
    count = math.ceil((end - start) * fps) + 1
    if count > 1_000_000:
        raise ValueError('Video exceeds one million frames; check timestamps/fps')
    times = [start + index / fps for index in range(count)]
    # Round to nanoseconds so a decimal grid boundary does not hold an old
    # frame merely because of binary floating-point subtraction.
    grid_ns = [round(index / fps * 1e9) for index in range(count)]
    selections = {role: [bisect_right(
        [round((stamp - start) * 1e9) for stamp in stamps[role]], t) - 1
        for t in grid_ns] for role in ROLES}
    return dict(start_source_ros_seconds=start, last_source_ros_seconds=end,
                output_source_ros_seconds=times, selections=selections,
                fps=fps, output_frames=count)


def _rgb(path: Path, role: str, metadata: dict) -> np.ndarray:
    with np.load(path, allow_pickle=False) as bundle:
        rgb = bundle[role]
    if (rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3
            or min(rgb.shape[:2]) < 2):
        raise ValueError(f'{path}: expected decoded HxWx3 uint8 RGB')
    if list(rgb.shape) != metadata['shape']:
        raise ValueError(f'{path}: recorded native shape mismatch')
    if hashlib.sha256(rgb.tobytes()).hexdigest() != metadata['sha256']:
        raise ValueError(f'{path}: recorded native RGB hash mismatch')
    return rgb


def export_physical_video(run: Path, output: Path, *, fps: float = 10.,
                          side_by_side: bool = False) -> dict[str, Any]:
    """Export into a new derived directory; never change or overwrite evidence.

    native_rgb.npz already contains decoded RGB, including for BGRA8 sources.
    This exporter does not decode the source encoding a second time.
    """
    run = run.resolve()
    output = output.resolve()
    if output == run or run in output.parents:
        raise ValueError('Use a separate derived-output directory')
    requests = sorted(run.glob('request_*/request.json'))
    if not requests:
        raise ValueError('No recorded physical observations found')
    samples = [json.loads(path.read_text()) for path in requests]
    indices = [sample['request_index'] for sample in samples]
    if any(b <= a for a, b in zip(indices, indices[1:])):
        raise ValueError('Request indices must strictly advance')
    stamps = {role: [s['cameras'][role]['stamp']['ros_seconds'] for s in samples]
              for role in ROLES}
    plan = frame_plan(stamps, fps)
    # Validate every saved frame before creating any output, including frames
    # the output grid will skip. Only one pair is retained at a time.
    shape = None
    sources = []
    for path, sample in zip(requests, samples):
        bundle = path.parent / 'native_rgb.npz'
        for role in ROLES:
            rgb = _rgb(bundle, role, sample['cameras'][role]['native_decoded_rgb'])
            if shape is None:
                shape = rgb.shape
            if rgb.shape != shape or any(d % 2 for d in rgb.shape[:2]):
                raise ValueError('MP4 needs consistent, even camera dimensions')
        sources.append(dict(request_index=sample['request_index'],
                            request_path=str(path.relative_to(run)),
                            request_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                            bundle_sha256=hashlib.sha256(bundle.read_bytes()).hexdigest(),
                            source_encoding={role: sample['cameras'][role]['source_encoding']
                                             for role in ROLES}))
    output.mkdir(parents=True, exist_ok=False)
    report = dict(schema_version=1, status='encoding', source_run=str(run),
                  scope='Sparse policy-request observations; not continuous video',
                  color_order='native_rgb.npz RGB -> OpenCV BGR; no alpha',
                  convention='Common source ROS clock; latest past frame held; '
                  'no interpolation; start at later first camera stamp; '
                  'end rounded up to output grid plus one frame duration',
                  source_timestamps=stamps, sources=sources, **plan,
                  missing_frames=[], interpolation_count=0, streams={})
    writers = {}
    try:
        height, width = shape[:2]
        for role in (*ROLES, *(('side_by_side',) if side_by_side else ())):
            size = (width * (2 if role == 'side_by_side' else 1), height)
            writer = cv2.VideoWriter(str(output / f'{role}.mp4'),
                                     cv2.VideoWriter_fourcc(*'mp4v'), fps, size)
            writers[role] = writer
            if not writer.isOpened():
                raise RuntimeError(f'MP4 encoder unavailable for {role}')
        cache: dict[str, tuple[int, np.ndarray]] = {}
        for n, stamp in enumerate(plan['output_source_ros_seconds']):
            pair = {}
            for role in ROLES:
                index = plan['selections'][role][n]
                if role not in cache or cache[role][0] != index:
                    rgb = _rgb(requests[index].parent / 'native_rgb.npz', role,
                               samples[index]['cameras'][role]['native_decoded_rgb'])
                    cache[role] = (index, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                image = cache[role][1].copy()
                text = (f'{role} request {indices[index]} | source {stamps[role][index]:.3f} '
                        f'| held {stamp - stamps[role][index]:.2f}s')
                cv2.putText(image, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                            .5, (0, 255, 255), 1, cv2.LINE_AA)
                cv2.putText(image, 'Sparse policy observations', (8, 48),
                            cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 255), 1)
                pair[role] = image
                writers[role].write(image)
            if side_by_side:
                writers['side_by_side'].write(np.concatenate(
                    [pair[role] for role in ROLES], axis=1))
        for writer in writers.values():
            writer.release()
        for role in writers:
            path = output / f'{role}.mp4'
            capture = cv2.VideoCapture(str(path))
            try:
                decoded = 0
                while capture.read()[0]:
                    decoded += 1
            finally:
                capture.release()
            if decoded != plan['output_frames']:
                raise RuntimeError(f'{role}: encoded frame count mismatch')
            report['streams'][role] = dict(path=path.name, frames=decoded,
                sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        report['resampling'] = {}
        for role in ROLES:
            selected = plan['selections'][role]
            report['resampling'][role] = dict(
                duplicated_output_frames=len(selected) - len(set(selected)),
                dropped_request_indices=[indices[i] for i in range(len(samples))
                                         if i not in set(selected)],
                unavailable_frames=0)
        report['status'] = 'complete'
    except Exception as error:
        report.update(status='failed', error=f'{type(error).__name__}: {error}')
        raise
    finally:
        for writer in writers.values():
            writer.release()
        with (output / 'video_manifest.json').open('x') as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
            stream.write('\n')
    return report
