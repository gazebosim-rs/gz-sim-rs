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
import subprocess
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
    def coverage_fixture(self, mutate=False, runtime=False, runtime_mutation=None, failures=None,
                         compiler_drift=False):
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
        helper = root / 'scripts/zenoh_runtime_probe.py'
        selection = {'metadata': {'synthetic': True},
                     'base_environment': {'path': str(native / 'environment.json'),
                                          'bytes': len(blob), 'sha256': digest(blob)},
                     'runtime_probe': None,
                     'helper_source': {'path': str(helper), 'bytes': helper.stat().st_size,
                                       'sha256': digest(helper.read_bytes())}}
        proof_files = {}
        if runtime:
            for role in verify.zenoh_peer.runtime_probe.PROOF_ROLES:
                path = native / role
                payload = ('synthetic proof role: ' + role).encode()
                path.write_bytes(payload)
                proof_files[role] = {'path': str(path), 'bytes': len(payload), 'sha256': digest(payload)}
            selection['runtime_probe'] = proof_files['selection.json']
        metadata = {'workspace_members': ['fixture'], 'packages': [
            {'id': 'fixture', 'manifest_path': str(root / 'crates/gz-sim-server/Cargo.toml')}]}
        version = 'rustc 1.97.1 (mocked gate control)\nrelease: 1.97.1\nhost: x86_64-unknown-linux-gnu\n'

        def run(command, **kwargs):
            if command[:2] == ['cargo', 'metadata']:
                return SimpleNamespace(stdout=json.dumps(metadata))
            if command == ['rustc', '--print', 'sysroot']:
                return SimpleNamespace(stdout='/not-executed/toolchain\n')
            if command == ['rustc', '-vV']:
                return SimpleNamespace(stdout=version + ('changed' if compiler_drift and profiles else ''))
            raise AssertionError('unexpected external command: ' + repr(command))

        profiles = []

        def instrumentation(name, crates, command, directory, *args):
            profiles.append(name)
            directory.mkdir(parents=True)
            (directory / 'test-output.log').write_text('mocked instrumentation; no coverage result\n')
            (directory / 'tests.stdout').write_text(name + ': synthetic child stdout\n')
            (directory / 'tests.stderr').write_text(name + ': synthetic child stderr\n')
            (directory / 'raw').mkdir()
            (directory / 'raw/control.profraw').write_bytes(b'synthetic RAW, never merged')
            if mutate and len(profiles) == 1:
                # This runs after coverage() captures its inputs. No actual
                # native fixture is modified and no compiler is called.
                fixture = root / (mutate if isinstance(mutate, str) else verify.zenoh_peer.CLIENTS['clock'])
                fixture.write_bytes(fixture.read_bytes() + b'\n// copied native fixture changed\n')
            if runtime_mutation and len(profiles) == 1:
                if runtime_mutation == 'copy':
                    (directory.parent / 'zenoh-runtime-probe/selection.json').write_bytes(b'changed captured proof')
                else:
                    selection['runtime_probe'] = {**selection['runtime_probe'], 'sha256': '0' * 64}
            if failures and name in failures:
                error = failures[name]
                if isinstance(error, ValueError):
                    (directory / 'coverage.json').write_text('{"synthetic": "failed coverage gate"}\n')
                    (directory / 'coverage.lcov').write_text('synthetic failed coverage, never qualified\n')
                raise error
            for filename in ['evidence.json', 'summary.json']:
                (directory / filename).write_text('mocked instrumentation; no coverage result\n')
            return {}, frozenset()

        def execute():
            with ExitStack() as stack:
                stack.enter_context(patch.object(verify, 'ROOT', root))
                stack.enter_context(patch.object(verify, 'run', side_effect=run))
                stack.enter_context(patch.object(verify, 'production_sources', return_value=sources))
                stack.enter_context(patch.object(verify, 'validate_test_support'))
                stack.enter_context(patch.object(verify, 'check_headless_dependencies'))
                stack.enter_context(patch.object(verify, 'changed_sources', return_value={}))
                stack.enter_context(patch.object(verify, 'selected_native_reference',
                                                side_effect=lambda: (native, json.loads(json.dumps(selection)))))
                stack.enter_context(patch.object(verify.zenoh_peer.runtime_probe, 'proof_files', return_value=proof_files))
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
        self.assertEqual(index['inputs']['scripts/profile_process.py'],
                         digest((root / 'scripts/profile_process.py').read_bytes()))
        self.assertEqual(index['inputs']['scripts/zenoh_runtime_probe.py'],
                         digest((root / 'scripts/zenoh_runtime_probe.py').read_bytes()))
        self.assertIsNone(index['zenoh_reference']['runtime_probe'])
        self.assertEqual(index['zenoh_reference']['runtime_probe_files'], {})
        self.assertEqual(index['zenoh_reference']['environment_sha256'], digest(blob))
        self.assertEqual((root / 'artifacts/coverage' / index['run'] / 'zenoh-native-reference.json').read_bytes(), blob)

    def test_native_fixture_drift_fails_integrated_coverage(self):
        _, _, profiles, execute = self.coverage_fixture(mutate=True)
        with self.assertRaisesRegex(ValueError, 'verification inputs changed between profiles'):
            execute()
        self.assertEqual(profiles, ['headless'])

    def test_process_helper_drift_fails_integrated_coverage(self):
        _, _, profiles, execute = self.coverage_fixture(mutate='scripts/profile_process.py')
        with self.assertRaisesRegex(ValueError, 'verification inputs changed between profiles'):
            execute()
        self.assertEqual(profiles, ['headless'])

    def test_runtime_selector_source_drift_fails_integrated_coverage(self):
        _, _, profiles, execute = self.coverage_fixture(mutate='scripts/zenoh_runtime_probe.py')
        with self.assertRaisesRegex(ValueError, 'verification inputs changed between profiles'):
            execute()
        self.assertEqual(profiles, ['headless'])

    def test_selected_runtime_proof_is_retained_once_separately_from_the_sdk(self):
        root, blob, profiles, execute = self.coverage_fixture(runtime=True)
        execute()
        self.assertEqual(profiles, ['headless', 'renderer'])
        coverage = root / 'artifacts/coverage'
        index = json.loads((coverage / 'profile-index.json').read_text())
        reference = index['zenoh_reference']
        self.assertEqual(reference['environment_sha256'], digest(blob))
        self.assertEqual(set(reference['runtime_probe_files']), set(verify.zenoh_peer.runtime_probe.PROOF_ROLES))
        for role, entry in reference['runtime_probe_files'].items():
            self.assertEqual(entry['file'], 'zenoh-runtime-probe/' + role)
            self.assertEqual((coverage / index['run'] / entry['file']).read_bytes(), Path(entry['source']['path']).read_bytes())
        self.assertEqual(reference['runtime_probe'], reference['runtime_probe_files']['selection.json']['source'])

    def test_changed_runtime_selection_or_retained_copy_cannot_qualify_coverage(self):
        for mutation in ('selection', 'copy'):
            _, _, profiles, execute = self.coverage_fixture(runtime=True, runtime_mutation=mutation)
            expected = 'selected Zenoh reference changed' if mutation == 'selection' else 'retained runtime probe proof changed'
            with self.subTest(mutation=mutation), self.assertRaisesRegex(ValueError, expected):
                execute()
            self.assertEqual(profiles, ['headless'])

    def test_failed_headless_tests_do_not_suppress_renderer_or_become_success(self):
        for actual_exit in (101, 137):
            failure = subprocess.CalledProcessError(actual_exit, ['cargo', 'test', '--no-fail-fast'])
            root, _, profiles, execute = self.coverage_fixture(failures={'headless': failure})
            with self.subTest(actual_exit=actual_exit), self.assertRaisesRegex(ValueError, 'headless: CalledProcessError'):
                execute()
            self.assertEqual(profiles, ['headless', 'renderer'])
            coverage = root / 'artifacts/coverage'
            index = json.loads((coverage / 'profile-index.json').read_text())
            self.assertEqual(index['profiles'], ['headless', 'renderer'])
            self.assertFalse((coverage / index['run'] / 'headless/evidence.json').exists())
            self.assertTrue((coverage / index['run'] / 'renderer/evidence.json').is_file())
            self.assertFalse((coverage / 'evidence.json').exists())
            self.assertFalse((coverage / 'summary.json').exists())

    def test_both_profile_failures_are_reported_with_exact_partial_evidence(self):
        for renderer_failure in (subprocess.CalledProcessError(101, ['cargo', 'test']),
                                 ValueError('renderer regions 94/100 must exceed 95%')):
            failures = {'headless': ValueError('gz-sim-server: regions 19281/20312 must exceed 95%'),
                        'renderer': renderer_failure}
            root, _, profiles, execute = self.coverage_fixture(failures=failures)
            with self.subTest(renderer=type(renderer_failure).__name__), self.assertRaises(ValueError) as caught:
                execute()
            text = str(caught.exception)
            self.assertIn('gz-sim-server: regions 19281/20312 must exceed 95%', text)
            self.assertIn('renderer: ' + type(renderer_failure).__name__, text)
            self.assertLess(text.index('headless:'), text.index('renderer:'))
            self.assertEqual(profiles, ['headless', 'renderer'])
            coverage = root / 'artifacts/coverage'
            index = json.loads((coverage / 'profile-index.json').read_text())
            # Exercise the maintained retention path with synthetic profile data;
            # unrelated native-off qualification has no fixture in this control.
            with patch.object(verify, 'ROOT', root), patch.object(verify, 'retain_rapier_evidence'), \
                 redirect_stdout(io.StringIO()):
                verify.retain_staged_evidence('a' * 40, root, 'checks_failed')
            packets = list((root / 'artifacts/staged').iterdir())
            self.assertEqual(len(packets), 1)
            packet = packets[0]
            stage = json.loads((packet / 'stage-result.json').read_text())
            self.assertEqual(stage['checks_state'], 'checks_failed')
            self.assertEqual(stage['evidence_state'], 'partial')
            for profile in profiles:
                for name in ('test-output.log', 'tests.stdout', 'tests.stderr', 'raw/control.profraw'):
                    relative = index['run'] + '/' + profile + '/' + name
                    source = coverage / relative
                    self.assertEqual((packet / relative).read_bytes(), source.read_bytes())
                    self.assertEqual(stage['artifacts_sha256'][relative], digest(source.read_bytes()))
                self.assertIn(index['run'] + '/' + profile + '/evidence.json', stage['missing_artifacts'])
            relative = index['run'] + '/headless/coverage.json'
            self.assertEqual((packet / relative).read_bytes(), (coverage / relative).read_bytes())

    def test_profile_runtime_failures_are_fatal_before_renderer(self):
        failures = (verify.profile_process.CommandInterrupted(15),
                    subprocess.CalledProcessError(-15, ['cargo', 'test']),
                    subprocess.CalledProcessError(-9, ['cargo', 'test']),
                    subprocess.TimeoutExpired(['cargo', 'test'], 3600),
                    RuntimeError('profile stdout exceeded its retained output cap'),
                    OSError('resource controller stopped the command'), KeyboardInterrupt())
        for error in failures:
            _, _, profiles, execute = self.coverage_fixture(failures={'headless': error})
            with self.subTest(error=type(error).__name__), self.assertRaises(type(error)):
                execute()
            self.assertEqual(profiles, ['headless'])

    def test_failed_profile_does_not_allow_source_or_native_custody_drift(self):
        cases = ({'mutate': 'scripts/profile_process.py'}, {'compiler_drift': True},
                 {'runtime': True, 'runtime_mutation': 'selection'},
                 {'runtime': True, 'runtime_mutation': 'copy'})
        for change in cases:
            _, _, profiles, execute = self.coverage_fixture(
                failures={'headless': ValueError('ordinary coverage failure')}, **change)
            with self.subTest(change=change), self.assertRaises(ValueError) as caught:
                execute()
            self.assertNotIn('Required coverage profiles failed:', str(caught.exception))
            self.assertIn('changed between profiles', str(caught.exception))
            self.assertEqual(profiles, ['headless'])

    def test_runtime_capture_refuses_a_reader_detected_growth_before_profiles(self):
        _, _, profiles, execute = self.coverage_fixture(runtime=True)
        with patch.object(verify.zenoh_peer.runtime_probe, 'read_file',
                          side_effect=ValueError('file grew beyond cap')) as read:
            with self.assertRaisesRegex(ValueError, 'file grew beyond cap'):
                execute()
        self.assertEqual(profiles, [])
        self.assertEqual(read.call_args.args[1], 16 * 1024**2)

    def test_staged_runtime_copy_uses_bounded_captured_bytes_not_the_original(self):
        root, candidate, _, run, index = self.staged_fixture(runtime=True)
        original_reader = verify.zenoh_peer.runtime_probe.read_file
        seen = []
        def bounded(path, cap):
            seen.append(Path(path))
            self.assertEqual(cap, 16 * 1024**2)
            if Path(path) == run / 'zenoh-runtime-probe/probe.stdout':
                raise ValueError('file grew beyond cap')
            return original_reader(path, cap)
        with patch.object(verify, 'ROOT', root), redirect_stdout(io.StringIO()), \
             patch.object(verify.zenoh_peer.runtime_probe, 'read_file', side_effect=bounded):
            with self.assertRaisesRegex(ValueError, 'file grew beyond cap'):
                verify.retain_staged_evidence('a' * 40, candidate, 'passed')
        self.assertIn(run / 'zenoh-runtime-probe/probe.stdout', seen)
        self.assertTrue(all(path.is_relative_to(run) for path in seen))

    def staged_fixture(self, runtime=False):
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
                 'zenoh_reference': {'environment_sha256': digest(blob), 'runtime_probe': None,
                                     'runtime_probe_files': {}},
                 'inputs': {'scripts/zenoh_peer.py': helper, **{
                     path: fixtures[client] for client, path in verify.zenoh_peer.CLIENTS.items()}}}
        process_helper = candidate / 'scripts/profile_process.py'
        process_helper.parent.mkdir()
        process_helper.write_text('# synthetic source identity, never executed\n')
        index['inputs']['scripts/profile_process.py'] = digest(process_helper.read_bytes())
        runtime_helper = candidate / 'scripts/zenoh_runtime_probe.py'
        runtime_helper.write_text('# synthetic selector identity, never executed\n')
        runtime_digest = digest(runtime_helper.read_bytes())
        index['inputs']['scripts/zenoh_runtime_probe.py'] = runtime_digest
        index['zenoh_reference']['runtime_probe_helper_sha256'] = runtime_digest
        if runtime:
            originals = root / 'original-runtime-proof'
            originals.mkdir()
            for role in verify.zenoh_peer.runtime_probe.PROOF_ROLES:
                source = originals / role
                payload = ('synthetic integrity fixture only: ' + role).encode()
                source.write_bytes(payload)
                copied = run / 'zenoh-runtime-probe' / role
                copied.parent.mkdir(exist_ok=True)
                copied.write_bytes(payload)
                index['zenoh_reference']['runtime_probe_files'][role] = {
                    'source': {'path': str(source), 'bytes': len(payload), 'sha256': digest(payload)},
                    'file': 'zenoh-runtime-probe/' + role}
            index['zenoh_reference']['runtime_probe'] = index['zenoh_reference']['runtime_probe_files']['selection.json']['source']
        binding['runtime_probe'] = index['zenoh_reference']['runtime_probe']
        binding['runtime_probe_helper_sha256'] = runtime_digest
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
            expected = {'camera'} if profile == 'renderer' else {'clock', 'rate', 'force', 'robot_sensors', 'odometry_publisher', 'apply_link_wrench', 'wrench_odometry', 'odometry_3d', 'odometry_translation_offsets', 'odometry_rotation_offsets', 'odometry_noise', 'air_speed_wind', 'joint_force_feedback', 'imu_reference_frames', 'imu_custom_world', 'imu_world_heading', 'multiple_altimeters', 'multiple_air_pressures', 'multiple_air_speeds', 'multiple_magnetometers', 'multiple_logical_cameras', 'multiple_lidars', 'spatial_lidar', 'lidar_points', 'multiple_pose_publishers', 'multiple_odometry_publishers', 'multiple_joint_states', 'multiple_joint_controllers', 'multiple_diff_drives', 'velocity_control'}
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
                    'runtime_probe': binding['runtime_probe'],
                    'runtime_probe_helper_sha256': runtime_digest,
                    'source': verify.zenoh_peer.CLIENTS[client],
                    'source_sha256': digest(source), 'binary_sha256': digest(executable)}))
            records = verify.check_native_peer_evidence(peers, expected, binding)
            (base / 'native-peers.json').write_text(json.dumps(records))
        return root, candidate, coverage, run, index

    def test_missing_or_changed_process_helper_binding_cannot_qualify_retention(self):
        for alteration in ('missing', 'changed', 'missing_file', 'dangling_symlink'):
            root, candidate, coverage, _, index = self.staged_fixture()
            if alteration == 'missing':
                index['inputs'].pop('scripts/profile_process.py')
            elif alteration == 'changed':
                (candidate / 'scripts/profile_process.py').write_text('# changed source\n')
            else:
                helper = candidate / 'scripts/profile_process.py'
                helper.unlink()
                if alteration == 'dangling_symlink':
                    helper.symlink_to(candidate / 'absent.py')
            (coverage / 'profile-index.json').write_text(json.dumps(index))
            with self.subTest(alteration=alteration), patch.object(verify, 'ROOT', root), redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(ValueError, 'missing retained profile evidence'):
                    verify.retain_staged_evidence('a' * 40, candidate, 'passed')

    def test_runtime_selector_input_and_complete_selected_bundle_are_required(self):
        for alteration in ('missing_input', 'changed_helper', 'missing_helper', 'dangling_helper',
                           'wrong_helper_binding', 'missing_bundle_role', 'missing_copy', 'dangling_copy'):
            root, candidate, coverage, run, index = self.staged_fixture(runtime=True)
            helper = candidate / 'scripts/zenoh_runtime_probe.py'
            proof = run / 'zenoh-runtime-probe/probe.stdout'
            if alteration == 'missing_input':
                index['inputs'].pop('scripts/zenoh_runtime_probe.py')
            elif alteration == 'changed_helper':
                helper.write_bytes(helper.read_bytes() + b'# changed\n')
            elif alteration in ('missing_helper', 'dangling_helper'):
                helper.unlink()
                if alteration == 'dangling_helper':
                    helper.symlink_to(candidate / 'missing-selector.py')
            elif alteration == 'wrong_helper_binding':
                index['zenoh_reference']['runtime_probe_helper_sha256'] = '0' * 64
            elif alteration == 'missing_bundle_role':
                index['zenoh_reference']['runtime_probe_files'].pop('probe.stdout')
            else:
                proof.unlink()
                if alteration == 'dangling_copy':
                    proof.symlink_to(candidate / 'missing-probe-output')
            (coverage / 'profile-index.json').write_text(json.dumps(index))
            with self.subTest(alteration=alteration), patch.object(verify, 'ROOT', root), redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(ValueError, 'missing retained profile evidence'):
                    verify.retain_staged_evidence('a' * 40, candidate, 'passed')

    def test_selected_runtime_bundle_retains_original_bytes_and_separate_reference_identity(self):
        root, candidate, _, run, index = self.staged_fixture(runtime=True)
        with patch.object(verify, 'ROOT', root), redirect_stdout(io.StringIO()):
            verify.retain_staged_evidence('a' * 40, candidate, 'passed')
        output = next((root / 'artifacts/staged').iterdir())
        for role in verify.zenoh_peer.runtime_probe.PROOF_ROLES:
            self.assertEqual((output / 'zenoh-runtime-probe' / role).read_bytes(),
                             (run / 'zenoh-runtime-probe' / role).read_bytes())
        self.assertEqual((output / 'zenoh-native-reference.json').read_bytes(),
                         (run / 'zenoh-native-reference.json').read_bytes())
        self.assertNotEqual(index['zenoh_reference']['environment_sha256'],
                            index['zenoh_reference']['runtime_probe']['sha256'])

    def test_selected_runtime_proof_corruption_and_path_escape_fail_retention(self):
        for alteration in ('bytes', 'escape'):
            root, candidate, coverage, run, index = self.staged_fixture(runtime=True)
            if alteration == 'bytes':
                (run / 'zenoh-runtime-probe/probe.stdout').write_bytes(b'changed retained result')
            else:
                index['zenoh_reference']['runtime_probe_files']['probe.stdout']['file'] = '../outside'
                (coverage / 'profile-index.json').write_text(json.dumps(index))
            with self.subTest(alteration=alteration), patch.object(verify, 'ROOT', root), redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(ValueError, 'staged runtime probe proof'):
                    verify.retain_staged_evidence('a' * 40, candidate, 'passed')

    def test_native_client_must_bind_the_selected_runtime_proof_and_selector_source(self):
        for field, replacement in (('runtime_probe', None), ('runtime_probe', {'path': '/wrong', 'bytes': 0, 'sha256': '0' * 64}),
                                   ('runtime_probe_helper_sha256', '0' * 64)):
            root, candidate, _, run, _ = self.staged_fixture(runtime=True)
            provenance = run / 'headless/native-peers/clock-control/provenance.json'
            value = json.loads(provenance.read_text())
            value[field] = replacement
            provenance.write_text(json.dumps(value))
            with self.subTest(field=field, replacement=replacement), patch.object(verify, 'ROOT', root), redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(ValueError, 'missing retained profile evidence'):
                    verify.retain_staged_evidence('a' * 40, candidate, 'passed')

    def test_current_stage_retains_clients_and_rejects_missing_metadata_contract(self):
        root, candidate, coverage, run, index = self.staged_fixture()
        with patch.object(verify, 'ROOT', root), redirect_stdout(io.StringIO()):
            verify.retain_staged_evidence('a' * 40, candidate, 'passed')
            output = next((root / 'artifacts/staged').iterdir())
            self.assertEqual((output / 'zenoh-native-reference.json').read_bytes(), (run / 'zenoh-native-reference.json').read_bytes())
            preparation = Path(run.name) / 'renderer/renderer-build-output.log'
            self.assertEqual((output / preparation).read_bytes(), (coverage / preparation).read_bytes())
            for profile, clients in [('headless', ['clock', 'rate', 'force', 'robot_sensors', 'odometry_publisher', 'apply_link_wrench', 'wrench_odometry', 'odometry_3d', 'odometry_translation_offsets', 'odometry_rotation_offsets', 'odometry_noise', 'air_speed_wind', 'joint_force_feedback', 'imu_reference_frames', 'imu_custom_world', 'imu_world_heading', 'multiple_altimeters', 'multiple_air_pressures', 'multiple_air_speeds', 'multiple_magnetometers', 'multiple_logical_cameras', 'multiple_lidars', 'spatial_lidar', 'lidar_points', 'multiple_pose_publishers', 'multiple_odometry_publishers', 'multiple_joint_states', 'multiple_joint_controllers', 'multiple_diff_drives', 'velocity_control']), ('renderer', ['camera'])]:
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

    def test_current_stage_rejects_missing_wrench_odometry_peer_input(self):
        root, candidate, coverage, _, index = self.staged_fixture()
        source = verify.zenoh_peer.CLIENTS['wrench_odometry']
        self.assertEqual(source, 'crates/gz-sim-server/tests/native_wrench_odometry.cc')
        index['inputs'].pop(source)
        (coverage / 'profile-index.json').write_text(json.dumps(index))
        with patch.object(verify, 'ROOT', root), redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(ValueError, 'missing retained profile evidence'):
                verify.retain_staged_evidence('a' * 40, candidate, 'passed')

    def test_current_stage_rejects_missing_three_dimensional_odometry_peer_input(self):
        root, candidate, coverage, _, index = self.staged_fixture()
        source = verify.zenoh_peer.CLIENTS['odometry_3d']
        self.assertEqual(source, 'crates/gz-sim-server/tests/native_odometry_3d.cc')
        index['inputs'].pop(source)
        (coverage / 'profile-index.json').write_text(json.dumps(index))
        with patch.object(verify, 'ROOT', root), redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(ValueError, 'missing retained profile evidence'):
                verify.retain_staged_evidence('a' * 40, candidate, 'passed')

    def test_current_stage_rejects_missing_translation_offset_odometry_peer_input(self):
        root, candidate, coverage, _, index = self.staged_fixture()
        source = verify.zenoh_peer.CLIENTS['odometry_translation_offsets']
        self.assertEqual(source, 'crates/gz-sim-server/tests/native_odometry_translation_offsets.cc')
        index['inputs'].pop(source)
        (coverage / 'profile-index.json').write_text(json.dumps(index))
        with patch.object(verify, 'ROOT', root), redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(ValueError, 'missing retained profile evidence'):
                verify.retain_staged_evidence('a' * 40, candidate, 'passed')

    def test_current_stage_rejects_missing_rotation_offset_odometry_peer_input(self):
        root, candidate, coverage, _, index = self.staged_fixture()
        source = verify.zenoh_peer.CLIENTS['odometry_rotation_offsets']
        self.assertEqual(source, 'crates/gz-sim-server/tests/native_odometry_rotation_offsets.cc')
        index['inputs'].pop(source)
        (coverage / 'profile-index.json').write_text(json.dumps(index))
        with patch.object(verify, 'ROOT', root), redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(ValueError, 'missing retained profile evidence'):
                verify.retain_staged_evidence('a' * 40, candidate, 'passed')

    def test_current_stage_rejects_missing_gaussian_odometry_peer_input(self):
        root, candidate, coverage, _, index = self.staged_fixture()
        source = verify.zenoh_peer.CLIENTS['odometry_noise']
        self.assertEqual(source, 'crates/gz-sim-server/tests/native_odometry_noise.cc')
        index['inputs'].pop(source)
        (coverage / 'profile-index.json').write_text(json.dumps(index))
        with patch.object(verify, 'ROOT', root), redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(ValueError, 'missing retained profile evidence'):
                verify.retain_staged_evidence('a' * 40, candidate, 'passed')

    def test_new_wind_and_force_peer_sources_cannot_be_missing_from_retained_inputs(self):
        for client in ("air_speed_wind", "joint_force_feedback", "imu_reference_frames", "imu_custom_world", "imu_world_heading", "multiple_altimeters", "multiple_air_pressures", "multiple_air_speeds", "multiple_magnetometers", "multiple_logical_cameras", "multiple_lidars", "spatial_lidar", "lidar_points", "multiple_pose_publishers", "multiple_odometry_publishers", "multiple_joint_states", "multiple_joint_controllers", "multiple_diff_drives", "velocity_control"):
            root, candidate, coverage, _, index = self.staged_fixture()
            source = verify.zenoh_peer.CLIENTS[client]
            index['inputs'].pop(source)
            (coverage / 'profile-index.json').write_text(json.dumps(index))
            with self.subTest(client=client), patch.object(verify, 'ROOT', root), redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(ValueError, 'missing retained profile evidence'):
                    verify.retain_staged_evidence('a' * 40, candidate, 'passed')

        # A complete input map cannot replace either required lidar capture.
        # Keep each index internally consistent so the required-client check
        # must detect the missing peer, rather than a stale index entry.
        for client in ('spatial_lidar', 'lidar_points'):
            root, candidate, _, run, _ = self.staged_fixture()
            peer_index = run / 'headless/native-peers.json'
            records = json.loads(peer_index.read_text())
            self.assertEqual(sum(record['client'] == client for record in records), 1)
            shutil.rmtree(run / 'headless/native-peers' / (client + '-control'))
            peer_index.write_text(json.dumps([
                record for record in records if record['client'] != client
            ]))
            with self.subTest(client=client, missing='captured_peer'), \
                 patch.object(verify, 'ROOT', root), redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(ValueError, 'missing retained profile evidence'):
                    verify.retain_staged_evidence('a' * 40, candidate, 'passed')
            output = next((root / 'artifacts/staged').iterdir())
            result = json.loads((output / 'stage-result.json').read_text())
            self.assertEqual(result['evidence_state'], 'partial')
            self.assertIn(str(Path(run.name) / 'headless/native-peers evidence'),
                          result['missing_artifacts'])

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
