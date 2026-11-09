# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Temporary verifier integrity controls, not coverage or SDK qualification.

External compilation/instrumentation is mocked. Mutations affect only copied
fixtures and synthetic staged evidence; the actual SDK and sources are untouched.
"""
from contextlib import ExitStack, redirect_stdout
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import shutil
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('selected_native_verify', ROOT / 'scripts/verify.py')
verify = importlib.util.module_from_spec(spec)
spec.loader.exec_module(verify)


def digest(data):
    return hashlib.sha256(data).hexdigest()


class SelectedNativeGate(unittest.TestCase):
    def coverage_fixture(self, mutate=False):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name) / 'workspace'
        root.mkdir()
        ignore = shutil.ignore_patterns('artifacts', 'target', '__pycache__', '.git')
        for name in ['Cargo.toml', 'Cargo.lock', 'rust-toolchain.toml', '.cargo',
                     'scripts', 'planning', 'third_party', 'reference', 'LICENSE', 'NOTICE',
                     'crates', 'tests/support']:
            source, target = ROOT / name, root / name
            target.parent.mkdir(parents=True, exist_ok=True)
            if source.is_dir():
                shutil.copytree(source, target, ignore=ignore)
            else:
                shutil.copyfile(source, target)
        crates = verify.RENDER_PACKAGES | {'gz-sim-core', 'gz-sim-scene', 'gz-sim-cli', 'gz-sim-proto', 'gz-sim-server'}
        sources = {}
        for name in crates:
            path = root / 'crates' / name / 'src/gate_control.rs'
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('// temporary source identity fixture, never compiled\n')
            sources[name] = [path]
        native = Path(temporary.name) / 'native'
        native.mkdir()
        blob = b'{"scope":"synthetic native binding; not an SDK"}\n'
        (native / 'environment.json').write_bytes(blob)
        metadata = {'workspace_members': ['fixture'], 'packages': [
            {'id': 'fixture', 'manifest_path': str(root / 'crates/gz-sim-server/Cargo.toml')}]}
        version = 'rustc 1.97.1 (mocked gate control)\nrelease: 1.97.1\nhost: x86_64-unknown-linux-gnu\n'

        def run(command, **kwargs):
            if command[:2] == ['cargo', 'metadata']:
                return SimpleNamespace(stdout=json.dumps(metadata))
            if command == ['rustc', '--print', 'sysroot']:
                return SimpleNamespace(stdout='/not-executed/toolchain\n')
            if command == ['rustc', '-vV']:
                return SimpleNamespace(stdout=version)
            raise AssertionError('unexpected external command: ' + repr(command))

        profiles = []

        def instrumentation(name, crates, command, directory, *args):
            profiles.append(name)
            directory.mkdir(parents=True)
            for filename in ['test-output.log', 'evidence.json', 'summary.json']:
                (directory / filename).write_text('mocked instrumentation; no coverage result\n')
            if mutate and len(profiles) == 1:
                # This runs after coverage() captures its inputs. No actual
                # native fixture is modified and no compiler is called.
                fixture = root / verify.zenoh_peer.CLIENTS['clock']
                fixture.write_bytes(fixture.read_bytes() + b'\n// copied native fixture changed\n')
            return {}, frozenset()

        def execute():
            with ExitStack() as stack:
                stack.enter_context(patch.object(verify, 'ROOT', root))
                stack.enter_context(patch.object(verify, 'run', side_effect=run))
                stack.enter_context(patch.object(verify, 'production_sources', return_value=sources))
                stack.enter_context(patch.object(verify, 'validate_test_support'))
                stack.enter_context(patch.object(verify, 'check_headless_dependencies'))
                stack.enter_context(patch.object(verify, 'changed_sources', return_value={}))
                stack.enter_context(patch.object(verify, 'selected_native_reference', return_value=(native, {'synthetic': True})))
                stack.enter_context(patch.object(verify, 'profile_coverage', side_effect=instrumentation))
                stack.enter_context(patch.object(verify, 'check_profile_reports', return_value={'scope': 'mocked instrumentation'}))
                stack.enter_context(patch.dict(verify.os.environ, {'RUSTC': '', 'CARGO_BUILD_RUSTC': '', 'RUSTC_WRAPPER': ''}))
                stack.enter_context(redirect_stdout(io.StringIO()))
                verify.coverage()
        return root, blob, profiles, execute

    def test_integrated_coverage_binds_all_clients_and_retains_native_metadata(self):
        root, blob, profiles, execute = self.coverage_fixture()
        execute()
        self.assertEqual(profiles, ['headless', 'renderer'])
        index = json.loads((root / 'artifacts/coverage/profile-index.json').read_text())
        for path in verify.zenoh_peer.CLIENTS.values():
            self.assertEqual(index['inputs'][path], digest((root / path).read_bytes()))
        self.assertEqual(index['zenoh_reference']['environment_sha256'], digest(blob))
        self.assertEqual((root / 'artifacts/coverage' / index['run'] / 'zenoh-native-reference.json').read_bytes(), blob)

    def test_native_fixture_drift_fails_integrated_coverage(self):
        _, _, profiles, execute = self.coverage_fixture(mutate=True)
        with self.assertRaisesRegex(ValueError, 'verification inputs changed between profiles'):
            execute()
        self.assertEqual(profiles, ['headless', 'renderer'])

    def staged_fixture(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        candidate = root / 'candidate'
        coverage = candidate / 'artifacts/coverage'
        run = coverage / 'profiles-control'
        run.mkdir(parents=True)
        (candidate / 'planning').mkdir()
        shutil.copyfile(ROOT / 'planning/coverage-profiles.json', candidate / 'planning/coverage-profiles.json')
        blob = b'{"scope":"synthetic retained metadata, not an SDK"}\n'
        (run / 'zenoh-native-reference.json').write_bytes(blob)
        helper = digest(b'synthetic helper identity')
        fixtures = {client: digest(('source: ' + client).encode()) for client in verify.zenoh_peer.CLIENTS}
        binding = {'reference_sha256': digest(blob), 'helper_sha256': helper, 'fixtures': fixtures}
        index = {'run': run.name, 'profiles': ['headless', 'renderer'],
                 'zenoh_reference': {'environment_sha256': digest(blob)},
                 'inputs': {'scripts/zenoh_peer.py': helper, **{
                     path: fixtures[client] for client, path in verify.zenoh_peer.CLIENTS.items()}}}
        (coverage / 'profile-index.json').write_text(json.dumps(index))
        required = ['test-output.log', 'coverage.json', 'coverage.lcov', 'summary.json',
                    'evidence.json', 'merged.profdata', 'proto-generated.json', 'proto-facade-audit.json',
                    'proto-facade-audit-output.log', 'proto-generated.rs', 'proto-descriptor.pb', 'proto-coverage.json', 'coverage-unfiltered.lcov']
        for filename in ['test-output.log', 'evidence.json', 'summary.json']:
            (coverage / filename).write_text('synthetic retention control, not coverage')
        for profile in index['profiles']:
            base = run / profile
            (base / 'raw').mkdir(parents=True)
            for filename in required + (['facade-audit.json', 'renderer-build-output.log'] if profile == 'renderer' else []):
                (base / filename).write_text('synthetic retention control, not coverage')
            (base / 'raw/control.profraw').write_text('not an LLVM profile')
            peers = base / 'native-peers'
            peers.mkdir()
            expected = {'camera'} if profile == 'renderer' else {'clock', 'rate', 'force', 'robot_sensors', 'odometry_publisher', 'apply_link_wrench'}
            for client in sorted(expected):
                output = peers / (client + '-control')
                output.mkdir()
                source = ('source: ' + client).encode()
                executable = b'not an executable; retained artifact integrity control'
                (output / 'source.cc').write_bytes(source)
                (output / (client + '-client')).write_bytes(executable)
                (output / 'provenance.json').write_text(json.dumps({
                    'client': client, 'profile': verify.zenoh_peer.PROFILE,
                    'reference_sha256': digest(blob), 'helper_sha256': helper,
                    'source': verify.zenoh_peer.CLIENTS[client],
                    'source_sha256': digest(source), 'binary_sha256': digest(executable)}))
            records = verify.check_native_peer_evidence(peers, expected, binding)
            (base / 'native-peers.json').write_text(json.dumps(records))
        return root, candidate, coverage, run, index

    def test_current_stage_retains_clients_and_rejects_missing_metadata_contract(self):
        root, candidate, coverage, run, index = self.staged_fixture()
        with patch.object(verify, 'ROOT', root), redirect_stdout(io.StringIO()):
            verify.retain_staged_evidence('a' * 40, candidate, 'passed')
            output = next((root / 'artifacts/staged').iterdir())
            self.assertEqual((output / 'zenoh-native-reference.json').read_bytes(), (run / 'zenoh-native-reference.json').read_bytes())
            preparation = Path(run.name) / 'renderer/renderer-build-output.log'
            self.assertEqual((output / preparation).read_bytes(), (coverage / preparation).read_bytes())
            for profile, clients in [('headless', ['clock', 'rate', 'force', 'robot_sensors', 'odometry_publisher', 'apply_link_wrench']), ('renderer', ['camera'])]:
                for client in clients:
                    relative = Path(run.name) / profile / 'native-peers' / (client + '-control')
                    for filename in ['source.cc', client + '-client', 'provenance.json']:
                        self.assertEqual((output / relative / filename).read_bytes(), (coverage / relative / filename).read_bytes())
            index.pop('zenoh_reference')
            (coverage / 'profile-index.json').write_text(json.dumps(index))
            (run / 'zenoh-native-reference.json').unlink()
            with self.assertRaisesRegex(ValueError, 'missing retained profile evidence'):
                verify.retain_staged_evidence('b' * 40, candidate, 'passed')

    def test_current_stage_rejects_missing_native_fixture_input_binding(self):
        root, candidate, coverage, _, index = self.staged_fixture()
        index['inputs'].pop(verify.zenoh_peer.CLIENTS['force'])
        (coverage / 'profile-index.json').write_text(json.dumps(index))
        with patch.object(verify, 'ROOT', root), redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(ValueError, 'missing retained profile evidence'):
                verify.retain_staged_evidence('a' * 40, candidate, 'passed')

    def test_current_stage_rejects_missing_robot_sensor_peer_input(self):
        root, candidate, coverage, _, index = self.staged_fixture()
        index['inputs'].pop(verify.zenoh_peer.CLIENTS['robot_sensors'])
        (coverage / 'profile-index.json').write_text(json.dumps(index))
        with patch.object(verify, 'ROOT', root), redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(ValueError, 'missing retained profile evidence'):
                verify.retain_staged_evidence('a' * 40, candidate, 'passed')

    def test_current_stage_rejects_each_missing_new_system_peer_input(self):
        for client in ['odometry_publisher', 'apply_link_wrench']:
            with self.subTest(client=client):
                root, candidate, coverage, _, index = self.staged_fixture()
                index['inputs'].pop(verify.zenoh_peer.CLIENTS[client])
                (coverage / 'profile-index.json').write_text(json.dumps(index))
                with patch.object(verify, 'ROOT', root), redirect_stdout(io.StringIO()):
                    with self.assertRaisesRegex(ValueError, 'missing retained profile evidence'):
                        verify.retain_staged_evidence('a' * 40, candidate, 'passed')

    def test_current_rgb_stage_requires_the_renderer_build_output(self):
        root, candidate, _, run, _ = self.staged_fixture()
        (run / 'renderer/renderer-build-output.log').unlink()
        with patch.object(verify, 'ROOT', root), redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(ValueError, 'missing retained profile evidence'):
                verify.retain_staged_evidence('a' * 40, candidate, 'passed')
        output = next((root / 'artifacts/staged').iterdir())
        result = json.loads((output / 'stage-result.json').read_text())
        self.assertEqual(result['evidence_state'], 'partial')
        self.assertIn(str(Path(run.name) / 'renderer/renderer-build-output.log'), result['missing_artifacts'])


if __name__ == '__main__':
    unittest.main()
