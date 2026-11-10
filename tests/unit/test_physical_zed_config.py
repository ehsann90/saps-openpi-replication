"""Historical profile and new profile remain distinct; no ROS imports."""
import json
from pathlib import Path
import tempfile
import unittest

from saps.physical.shadow_config import load_shadow_config


class ZedConfigTests(unittest.TestCase):
    def test_both_profiles_and_explicit_roles(self):
        root = Path(__file__).resolve().parents[2]
        old = load_shadow_config(root / 'configs/physical_pi05_fr3.json')
        self.assertEqual(old['robot']['tcp_frame'], 'fr3_hand_tcp')
        new = json.loads((root / 'configs/physical_pi05_fr3_zed_robotiq.json').read_text())
        new['wrist_camera']['serial'] = '12345'
        new['exterior_camera']['serial'] = '67890'
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'config.json'
            path.write_text(json.dumps(new))
            self.assertEqual(load_shadow_config(path)['native_images']['encoding'], 'bgra8')
            new['wrist_camera']['orientation'] = 'flipped'
            path.write_text(json.dumps(new))
            with self.assertRaises(ValueError):
                load_shadow_config(path)
