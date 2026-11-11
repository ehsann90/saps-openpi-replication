"""Deterministic offline RGB export, irregular source clocks and evidence safety."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from saps.physical.video_export import export_physical_video, frame_plan


class PhysicalVideoTests(unittest.TestCase):
    def test_irregular_stamps_hold_past_frames_and_report_skips(self):
        plan = frame_plan({'wrist': [10., 10.01, 10.5],
                           'exterior': [10.1, 10.3, 10.6]}, 10)
        self.assertEqual(plan['selections']['wrist'], [1, 1, 1, 1, 2, 2])
        self.assertEqual(plan['selections']['exterior'], [0, 0, 1, 1, 1, 2])
        for stamps in ([1., 1.], [2., 1.], [float('nan')], []):
            with self.assertRaises(ValueError):
                frame_plan(dict(wrist=stamps, exterior=[1.]), 10)
        for fps in (0, float('inf'), -1, 121):
            with self.assertRaises(ValueError):
                frame_plan(dict(wrist=[1.], exterior=[1.]), fps)

    def fixture(self, root: Path) -> Path:
        run = root / 'run'
        run.mkdir()
        for index in range(2):
            request = run / f'request_{index:04d}'
            request.mkdir()
            rgb = np.zeros((96, 160, 3), dtype=np.uint8)
            rgb[:, :, 0] = 240  # red in saved RGB; would be blue if mishandled
            np.savez_compressed(request / 'native_rgb.npz', wrist=rgb, exterior=rgb)
            metadata = dict(shape=list(rgb.shape),
                            sha256=hashlib.sha256(rgb.tobytes()).hexdigest())
            (request / 'request.json').write_text(json.dumps(dict(
                request_index=index, cameras={role:dict(
                    stamp=dict(ros_seconds=10. + index * .5),
                    source_encoding='bgra8', native_decoded_rgb=metadata)
                    for role in ('wrist', 'exterior')})))
        return run

    def test_export_decodes_rgb_and_preserves_sources_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            run = self.fixture(root)
            before = {p: p.read_bytes() for p in run.rglob('*') if p.is_file()}
            report = export_physical_video(run, root / 'videos', fps=10,
                                           side_by_side=True)
            self.assertEqual(report['status'], 'complete')
            self.assertEqual(report['output_frames'], 6)
            self.assertEqual(report['resampling']['wrist']['duplicated_output_frames'], 4)
            capture = cv2.VideoCapture(str(root / 'videos/wrist.mp4'))
            try:
                ok, bgr = capture.read()
                self.assertTrue(ok)
                self.assertGreater(int(bgr[80, 80, 2]), 200)
                self.assertLess(int(bgr[80, 80, 0]), 30)
            finally:
                capture.release()
            self.assertEqual(before, {p: p.read_bytes() for p in before})
            with self.assertRaises(FileExistsError):
                export_physical_video(run, root / 'videos')
            with self.assertRaises(ValueError):
                export_physical_video(run, run / 'videos')

    def test_missing_or_invalid_native_frames_fail_without_fabricating_video(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            run = self.fixture(root)
            np.savez_compressed(run / 'request_0001/native_rgb.npz',
                                wrist=np.zeros((2, 2, 4), dtype=np.uint8))
            with self.assertRaises(ValueError):
                export_physical_video(run, root / 'videos')
            self.assertFalse((root / 'videos').exists())
