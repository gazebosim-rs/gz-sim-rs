# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Fail-closed client selection; these checks do not qualify a native build."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('zenoh_peer', ROOT / 'scripts/zenoh_peer.py')
peer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(peer)


class ZenohPeerTests(unittest.TestCase):
    def setUp(self):
        environment = patch.dict(os.environ)
        environment.start()
        self.addCleanup(environment.stop)
        # A real selected attestation must not leak into fake/default references.
        os.environ.pop('GZ_ZENOH_RUNTIME_PROBE_MANIFEST', None)

    def test_real_cli_dispatches_registered_clients_and_refuses_wrong_profiles_before_build(self):
        self.assertEqual(peer.CLIENTS['multiple_lidars'],
                         'crates/gz-sim-server/tests/native_multiple_lidars_peer.cc')
        self.assertEqual(peer.CLIENTS['multiple_pose_publishers'],
                         'crates/gz-sim-server/tests/native_multiple_pose_publishers_peer.cc')
        self.assertEqual(peer.CLIENTS['multiple_odometry_publishers'],
                         'crates/gz-sim-server/tests/native_multiple_odometry_publishers_peer.cc')
        self.assertEqual(peer.CLIENTS['multiple_joint_states'],
                         'crates/gz-sim-server/tests/native_multiple_joint_states_peer.cc')
        self.assertEqual(peer.CLIENTS['multiple_joint_controllers'],
                         'crates/gz-sim-server/tests/native_multiple_joint_controllers_peer.cc')
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
            for client in ['camera', 'robot_sensors', 'odometry_publisher', 'apply_link_wrench', 'wrench_odometry', 'odometry_3d', 'odometry_translation_offsets', 'odometry_rotation_offsets', 'odometry_noise', 'air_speed_wind', 'joint_force_feedback', 'imu_reference_frames', 'imu_custom_world', 'imu_world_heading', 'multiple_altimeters', 'multiple_air_pressures', 'multiple_air_speeds', 'multiple_magnetometers', 'multiple_logical_cameras', 'multiple_lidars', 'multiple_pose_publishers', 'multiple_odometry_publishers', 'multiple_joint_states', 'multiple_joint_controllers']:
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


    def _compile_fixture(self, root, selected, mutate=None):
        reference = root / 'reference'
        reference.mkdir()
        metadata = {'lock': {'vendor': {'prefix': 'vendor-prefix'}},
                    'tools': {'pkg-config': {'path': '/mock/pkg-config'},
                              'c++': {'path': '/mock/c++'}}}
        base = reference / 'environment.json'
        base.write_text(json.dumps(metadata))
        manifest = root / 'runtime-selection.json'
        manifest.write_text('reviewed selection fixture; no actual native result')
        helper = root / 'runtime-helper.py'
        helper.write_text('# retained helper Source fixture\n')
        source = root / peer.CLIENTS['clock']
        source.parent.mkdir(parents=True)
        source.write_text('// immutable canonical peer Source fixture\n')
        evidence = root / 'evidence'
        evidence.mkdir()

        def artifact(path):
            blob = path.read_bytes()
            return {'path': str(path), 'sha256': hashlib.sha256(blob).hexdigest(),
                    'bytes': len(blob)}

        def selection(_reference):
            self.assertEqual(_reference, reference)
            return {'metadata': json.loads(base.read_text()),
                    'base_environment': artifact(base),
                    'runtime_probe': artifact(manifest) if selected else None,
                    'helper_source': artifact(helper)}

        def query(command, **_kwargs):
            if command == ['/mock/pkg-config', '--variable=prefix', 'gz-transport']:
                return str(reference / 'install') + '\n'
            if command == ['/mock/pkg-config', '--cflags', '--libs', 'gz-transport']:
                return ''
            self.assertEqual(command[0], '/usr/bin/ldd')
            self.assertTrue(Path(command[1]).is_relative_to(evidence))
            return 'retained linkage fixture'

        def compile_only(command, **_kwargs):
            self.assertEqual(command[0], '/mock/c++')
            Path(command[-1]).write_bytes(b'nonexecuted client ELF fixture')
            if mutate is not None:
                {'base': base, 'manifest': manifest, 'helper': helper}[mutate].write_text('changed')
            return subprocess.CompletedProcess(command, 0, '', '')

        with patch.object(peer, 'ROOT', root), \
             patch.dict(os.environ, {'GZ_ZENOH_PEER_EVIDENCE_DIR': str(evidence)}), \
             patch.object(peer.runtime_probe, 'validate_selection', side_effect=selection) as validate, \
             patch.object(peer.bootstrap, 'build_environment', return_value={}), \
             patch.object(peer.bootstrap, 'validate_linkage', return_value={'retained': 'fixture'}), \
             patch.object(peer.subprocess, 'check_output', side_effect=query), \
             patch.object(peer.subprocess, 'run', side_effect=compile_only):
            result = peer.compile_client('clock', reference)
            self.assertEqual(validate.call_count, 2)
        return json.loads(Path(result['provenance']).read_text()), artifact(base), artifact(manifest), artifact(helper)

    def test_client_provenance_keeps_base_and_selected_runtime_bindings_separate(self):
        for selected in (False, True):
            with self.subTest(selected=selected), tempfile.TemporaryDirectory() as directory:
                result, base, manifest, helper = self._compile_fixture(Path(directory), selected)
                self.assertEqual(result['reference_sha256'], base['sha256'])
                self.assertEqual(result['runtime_probe'], manifest if selected else None)
                self.assertEqual(result['runtime_probe_helper_sha256'], helper['sha256'])
                self.assertEqual(result['libraries'], {'retained': 'fixture'})

    def test_base_manifest_or_runtime_helper_mutation_cannot_publish_client_provenance(self):
        for role in ('base', 'manifest', 'helper'):
            with self.subTest(role=role), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                with self.assertRaises(ValueError):
                    self._compile_fixture(root, True, mutate=role)
                self.assertEqual(list((root / 'evidence').glob('*/provenance.json')), [])

    def test_present_invalid_runtime_selection_refuses_before_compiler_or_output(self):
        for selector in ('', '/missing-runtime-selection.json'):
            with self.subTest(selector=selector), \
                 patch.dict(os.environ, {'GZ_ZENOH_RUNTIME_PROBE_MANIFEST': selector}), \
                 patch.object(peer.runtime_probe, 'validate_selection',
                              side_effect=ValueError('invalid explicit runtime selection')) as validate, \
                 patch.object(peer.bootstrap, 'build_environment') as environment, \
                 patch.object(peer.subprocess, 'run') as compile_only:
                with self.assertRaisesRegex(ValueError, 'invalid explicit runtime selection'):
                    peer.compile_client('clock', Path('/missing-sdk'))
                validate.assert_called_once_with(Path('/missing-sdk'))
                environment.assert_not_called()
                compile_only.assert_not_called()


if __name__ == '__main__':
    unittest.main()
