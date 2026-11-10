# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Fail-closed client selection; these checks do not qualify a native build."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('zenoh_peer', ROOT / 'scripts/zenoh_peer.py')
peer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(peer)


class ZenohPeerTests(unittest.TestCase):
    def test_real_cli_dispatches_registered_clients_and_refuses_wrong_profiles_before_build(self):
        # Run the actual CLI/parser with only the compiler/reference boundary
        # replaced. No native SDK, compiler or network is used by this control.
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scripts = root / 'scripts'
            scripts.mkdir()
            command = scripts / 'native-peer.py'
            command.write_bytes((ROOT / 'scripts/native-peer.py').read_bytes())
            (scripts / 'native_reference.py').write_text(
                "LEGACY='transport-client-v1'\nSDK='clean-headless-sdk-v1'\n"
                "builder=compiler_identity=validate_reference=None\n")
            (scripts / 'zenoh_peer.py').write_text(
                "import json,os\nfrom pathlib import Path\n"
                "def compile_client(client, reference):\n"
                "    result={'client':client,'reference':str(reference),'native_build':False}\n"
                "    Path(os.environ['PEER_DISPATCH_RECORD']).write_text(json.dumps(result))\n"
                "    return result\n")
            record = root / 'dispatch.json'
            reference = root / 'not-an-sdk'
            env = dict(os.environ, GZ_ZENOH_REFERENCE_ROOT=str(reference),
                       GZ_REFERENCE_PREFIX=str(root / 'no-historical-prefix'),
                       PEER_DISPATCH_RECORD=str(record), PYTHONDONTWRITEBYTECODE='1')
            for client in sorted(peer.CLIENTS):
                with self.subTest(client=client):
                    result = subprocess.run([
                        sys.executable, str(command), '--client', client,
                        '--profile', 'gazebo-zenoh-client-v1', '--json',
                    ], env=env, capture_output=True, text=True, timeout=10)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertEqual(result.stderr, '')
                    expected = {'client': client, 'reference': str(reference), 'native_build': False}
                    self.assertEqual(json.loads(result.stdout), expected)
                    self.assertEqual(json.loads(record.read_text()), expected)
            marker = record.read_bytes()
            for client in ['camera', 'robot_sensors', 'odometry_publisher', 'apply_link_wrench', 'wrench_odometry', 'odometry_3d', 'odometry_translation_offsets', 'air_speed_wind', 'joint_force_feedback']:
                for profile in ['transport-client-v1', 'clean-headless-sdk-v1']:
                    with self.subTest(client=client, profile=profile):
                        result = subprocess.run([
                            sys.executable, str(command), '--client', client, '--profile', profile,
                        ], env=env, capture_output=True, text=True, timeout=10)
                        self.assertEqual(result.returncode, 2)
                        self.assertEqual(result.stdout, '')
                        self.assertIn('require the explicit Zenoh profile', result.stderr)
                        self.assertEqual(record.read_bytes(), marker)
            result = subprocess.run([
                sys.executable, str(command), '--client', '../foreign',
                '--profile', 'gazebo-zenoh-client-v1', '--json',
            ], env=env, capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 2)
            self.assertEqual(result.stdout, '')
            self.assertIn('invalid choice', result.stderr)
            self.assertEqual(record.read_bytes(), marker)

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
