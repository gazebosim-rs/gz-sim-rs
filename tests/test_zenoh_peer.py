# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Fail-closed client selection; these checks do not qualify a native build."""
import importlib.util
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('zenoh_peer', ROOT / 'scripts/zenoh_peer.py')
peer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(peer)


class ZenohPeerTests(unittest.TestCase):
    def test_unknown_client_cannot_select_a_source_or_reference(self):
        with self.assertRaisesRegex(ValueError, 'unknown conformance client'):
            peer.compile_client('../foreign', Path('/missing'))

    def test_camera_cannot_use_historical_profile_or_json_contract(self):
        result = subprocess.run([
            'python3', str(ROOT / 'scripts/native-peer.py'), '--client', 'camera',
            '--profile', 'transport-client-v1', '--json',
        ], capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertEqual(result.stdout, '')
        self.assertIn('require the explicit Zenoh profile', result.stderr)

    def test_explicit_zenoh_never_falls_back_to_an_old_prefix(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            old = root / 'historical'
            old.mkdir()
            marker = old / 'environment.json'
            marker.write_text('{"profile":"transport-client-v1"}')
            env = dict(os.environ, GZ_ZENOH_REFERENCE_ROOT=str(root / 'missing'),
                       GZ_REFERENCE_PREFIX=str(old / 'install'))
            result = subprocess.run([
                'python3', str(ROOT / 'scripts/native-peer.py'), '--client', 'camera',
                '--profile', 'gazebo-zenoh-client-v1', '--json',
            ], env=env, capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(result.stdout, '')
            self.assertIn('native reference root must be a real directory', result.stderr)
            self.assertEqual(marker.read_text(), '{"profile":"transport-client-v1"}')


if __name__ == '__main__':
    unittest.main()
