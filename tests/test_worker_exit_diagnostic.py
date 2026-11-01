# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Controlled bookkeeping/process tests; these do not qualify renderer execution."""
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock

SPEC = importlib.util.spec_from_file_location(
    'worker_diagnostic', Path(__file__).resolve().parents[1] / 'scripts/diagnostic-worker-exit.py')
diag = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(diag)


class WorkerDiagnosticTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name).resolve()
        self.main = self.root / 'crates/gz-render-worker/src/main.rs'
        self.test = self.root / 'crates/gz-render-worker/tests/direct_worker.rs'
        for path in [self.main, self.test]:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('// controlled source identity fixture\n')
        (self.root / 'Cargo.toml').write_text('[workspace.package]\nversion="0.1.0"\n')
        self.git('init', '--quiet')
        self.git('add', '.')
        self.git('-c', 'user.name=Diagnostic test', '-c', 'user.email=test@example.invalid',
                 'commit', '--quiet', '-m', 'controlled fixture')
        self.env = dict(os.environ, GITHUB_ACTIONS='true', GITHUB_REF=diag.REF,
                        GITHUB_EVENT_NAME='push', GITHUB_REPOSITORY='gazebosim-rs/gz-sim-rs',
                        GITHUB_WORKSPACE=str(self.root), GITHUB_SHA=self.git('rev-parse', 'HEAD').strip(),
                        GZ_RENDER_REQUIRE_LLVMPIPE='1', VK_ICD_FILENAMES='/controlled/icd.json')
        self.coverage = self.root / 'artifacts/coverage'
        self.directory = self.coverage / 'profiles-fixture' / 'renderer'
        self.directory.mkdir(parents=True)
        worker = self.root / 'target/coverage/debug/gz-render-worker'
        tests = worker.parent / 'deps/direct_worker-0123abcd'
        for path in [worker, tests]:
            path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile('/bin/true', path)
            path.chmod(0o755)
        self.paths = [worker, tests]
        self.profile_start_ns = min(path.stat().st_mtime_ns for path in self.paths) - 1
        self.index = {'profiles': ['headless', 'renderer'], 'run': 'profiles-fixture',
                      'rustc': 'controlled compiler identity',
                      'sources': {str(self.main.relative_to(self.root)): diag.digest(self.main)},
                      'inputs': {'Cargo.toml': diag.digest(self.root / 'Cargo.toml')}}
        self.objects = []
        self.cargo = []
        for path, source, name, kind, test in [
                (worker, self.main, 'gz-render-worker', 'bin', False),
                (tests, self.test, 'direct_worker', 'test', True)]:
            target = {'name': name, 'kind': [kind], 'src_path': str(source)}
            obj = {'path': str(path), 'sha256': diag.digest(path), 'target': target,
                   'package_id': f'path+file://{self.root}/crates/gz-render-worker#0.1.0',
                   'profile': {'test': test}}
            self.objects.append(obj)
            self.cargo.append({'reason': 'compiler-artifact', 'executable': str(path),
                               'target': target, 'package_id': obj['package_id'],
                               'profile': obj['profile'], 'fresh': False})
        self.evidence = {'profile': 'renderer', 'rustc': self.index['rustc'],
                         'sources': self.index['sources'].copy(), 'objects': self.objects}
        self.success = (f'test {diag.CASE} ... ok\n'
                        'test result: ok. 20 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s\n')
        self.write_records()

    def git(self, *args):
        return subprocess.check_output(['git', '-C', str(self.root), *args], text=True,
                                       stderr=subprocess.PIPE)

    def write_records(self):
        index = self.coverage / 'profile-index.json'
        index.write_text(json.dumps(self.index))
        os.utime(index, ns=(self.profile_start_ns, self.profile_start_ns))
        (self.directory / 'evidence.json').write_text(json.dumps(self.evidence))
        (self.directory / 'test-output.log').write_text(
            '\n'.join(json.dumps(x) for x in self.cargo) + '\n' + self.success)

    def test_current_normal_cargo_binding_selects_exact_two_objects(self):
        binding = diag.bind(self.root, self.env)
        self.assertEqual(set(binding['executables']), {'worker', 'tests'})
        self.assertEqual(binding['executables']['tests']['path'], str(self.paths[1]))
        # A unit-test version of the worker is legitimate, not a second worker executable.
        unit = copy.deepcopy(self.objects[0])
        unit['profile']['test'] = True
        self.objects.append(unit)
        self.write_records()
        after = diag.bind(self.root, self.env)
        self.assertEqual(after['executables'], binding['executables'])
        self.assertEqual(after['commit'], binding['commit'])

    def test_non_diagnostic_context_and_mismatched_checkout_refuse_before_output(self):
        for key, value in [('GITHUB_REF', 'refs/heads/main'), ('GITHUB_ACTIONS', 'false'),
                           ('GITHUB_EVENT_NAME', 'pull_request'), ('GITHUB_REPOSITORY', 'other/repo'),
                           ('GITHUB_WORKSPACE', ''), ('GITHUB_SHA', '0' * 40),
                           ('GZ_RENDER_REQUIRE_LLVMPIPE', ''), ('VK_ICD_FILENAMES', '')]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                diag.batch(self.root, self.env | {key: value})
            self.assertFalse((self.root / 'artifacts/worker-exit-stress').exists())

    def test_missing_mutated_and_symlinked_executables_refuse(self):
        path = self.paths[1]
        original = path.read_bytes()
        path.write_bytes(original + b'changed')
        with self.assertRaisesRegex(ValueError, 'mutated'):
            diag.bind(self.root, self.env)
        path.unlink()
        with self.assertRaises(FileNotFoundError):
            diag.bind(self.root, self.env)
        path.symlink_to(self.paths[0])
        with self.assertRaisesRegex(ValueError, 'noncanonical'):
            diag.bind(self.root, self.env)
        path.unlink()
        path.write_bytes(original)
        with self.assertRaisesRegex(ValueError, 'mutated'):
            diag.bind(self.root, self.env)  # Lost executable mode is also rejected.

    def test_ambiguous_wrong_source_and_missing_cargo_records_refuse(self):
        original = copy.deepcopy(self.evidence)
        self.objects.append(copy.deepcopy(self.objects[0]))
        self.write_records()
        with self.assertRaisesRegex(ValueError, 'ambiguous'):
            diag.bind(self.root, self.env)
        self.evidence = original
        self.evidence['objects'][0]['target']['src_path'] = str(self.test)
        self.write_records()
        with self.assertRaisesRegex(ValueError, 'source/package'):
            diag.bind(self.root, self.env)
        self.evidence = original = copy.deepcopy(original)
        self.evidence['objects'][0]['target']['src_path'] = str(self.main)
        self.cargo = []
        self.write_records()
        with self.assertRaisesRegex(ValueError, 'unique normal Cargo'):
            diag.bind(self.root, self.env)

    def test_stale_sources_duplicate_json_and_escaping_run_refuse(self):
        self.index['sources'][str(self.main.relative_to(self.root))] = '0' * 64
        self.write_records()
        with self.assertRaisesRegex(ValueError, 'stale'):
            diag.bind(self.root, self.env)
        self.index['sources'][str(self.main.relative_to(self.root))] = diag.digest(self.main)
        self.index['run'] = '../renderer'
        self.write_records()
        with self.assertRaisesRegex(ValueError, 'normal combined'):
            diag.bind(self.root, self.env)
        (self.coverage / 'profile-index.json').write_text('{"run":"one","run":"two"}')
        with self.assertRaisesRegex(ValueError, 'duplicate JSON'):
            diag.bind(self.root, self.env)

    def test_stale_executable_evidence_refuses_even_after_index_is_refreshed(self):
        self.main.write_text('// newly committed controlled source\n')
        self.git('add', str(self.main))
        self.git('-c', 'user.name=Diagnostic test', '-c', 'user.email=test@example.invalid',
                 'commit', '--quiet', '-m', 'changed controlled source')
        self.env['GITHUB_SHA'] = self.git('rev-parse', 'HEAD').strip()
        self.index['sources'][str(self.main.relative_to(self.root))] = diag.digest(self.main)
        self.write_records()
        with self.assertRaisesRegex(ValueError, 'stale source bindings'):
            diag.bind(self.root, self.env)

    def test_cached_targets_and_test_only_source_changes_are_refused(self):
        self.cargo[1]['fresh'] = True
        self.write_records()
        with self.assertRaisesRegex(ValueError, 'freshly compiled'):
            diag.bind(self.root, self.env)
        self.cargo[1]['fresh'] = False
        self.test.write_text('// changed test-source fixture, absent from production index\n')
        self.git('add', str(self.test))
        self.git('-c', 'user.name=Diagnostic test', '-c', 'user.email=test@example.invalid',
                 'commit', '--quiet', '-m', 'changed controlled test')
        self.env['GITHUB_SHA'] = self.git('rev-parse', 'HEAD').strip()
        newer = self.paths[1].stat().st_mtime_ns + 1000000
        os.utime(self.test, ns=(newer, newer))
        self.write_records()
        with self.assertRaisesRegex(ValueError, 'freshly compiled'):
            diag.bind(self.root, self.env)

    def test_binary_outside_normal_compile_interval_is_refused(self):
        original = self.paths[1].stat().st_mtime_ns
        earlier = self.profile_start_ns - 1000000
        os.utime(self.paths[1], ns=(earlier, earlier))
        with self.assertRaisesRegex(ValueError, 'freshly compiled'):
            diag.bind(self.root, self.env)
        os.utime(self.paths[1], ns=(original, original))
        evidence = self.directory / 'evidence.json'
        os.utime(evidence, ns=(earlier, earlier))
        with self.assertRaisesRegex(ValueError, 'freshly compiled'):
            diag.bind(self.root, self.env)

    def controlled_attempt(self, root, command, directory, env, deadline):
        self.assertEqual(root, self.root)
        self.assertNotIn('RUST_TEST_THREADS', env)
        self.assertNotIn('GZ_RENDER_WORKER_DIAGNOSTICS', env)
        self.assertTrue(Path(env['LLVM_PROFILE_FILE']).is_relative_to(directory / 'raw'))
        self.assertFalse(Path(env['LLVM_PROFILE_FILE']).is_relative_to(self.coverage))
        (directory / 'stdout.log').write_text(self.success)
        (directory / 'stderr.log').write_text('controlled execution bookkeeping\n')
        (directory / 'raw/controlled.profraw').write_bytes(b'non-LLVM bookkeeping fixture')
        passed = directory.name != 'attempt-02'
        return {'passed': passed, 'classification': 'passed' if passed else 'other_failure',
                'leader_reaped': True, 'returncode': 0 if passed else 101}

    def test_all_ten_retained_after_failure_without_acceptance_profile_mutation(self):
        before = {str(p): diag.digest(p) for p in self.coverage.rglob('*') if p.is_file()}
        with mock.patch.object(diag, 'execute', side_effect=self.controlled_attempt) as execute:
            self.assertFalse(diag.batch(self.root, self.env | {'RUST_TEST_THREADS': '99'}))
        self.assertEqual(execute.call_count, 10)
        output = self.root / 'artifacts/worker-exit-stress'
        result = json.loads((output / 'result.json').read_text())
        self.assertEqual([x['threads'] for x in result['attempts']], [None] * 5 + [20] * 5)
        self.assertFalse(result['passed'])
        self.assertEqual(sum(x['passed'] for x in result['attempts']), 9)
        for number in range(1, 11):
            directory = output / f'attempt-{number:02}'
            self.assertTrue((directory / 'stdout.log').is_file())
            record = json.loads((directory / 'result.json').read_text())
            for relative, sha in record['files_sha256'].items():
                self.assertEqual(diag.digest(directory / relative), sha)
        self.assertEqual(before, {str(p): diag.digest(p) for p in self.coverage.rglob('*') if p.is_file()})
        with self.assertRaisesRegex(ValueError, 'already exists'):
            diag.batch(self.root, self.env)

    def test_mutation_after_first_attempt_refuses_remaining_executions_but_records_ten(self):
        def mutated(*args):
            result = self.controlled_attempt(*args)
            self.paths[0].write_bytes(b'mutated executable')
            return result
        with mock.patch.object(diag, 'execute', side_effect=mutated) as execute:
            self.assertFalse(diag.batch(self.root, self.env))
        self.assertEqual(execute.call_count, 1)
        result = json.loads((self.root / 'artifacts/worker-exit-stress/result.json').read_text())
        self.assertEqual(len(result['attempts']), 10)
        self.assertTrue(all(not x['passed'] for x in result['attempts']))

    def test_real_outer_bound_escalates_and_reaps_only_owned_child(self):
        directory = self.root / 'process'
        directory.mkdir()
        command = [sys.executable, '-c',
                   'import signal,time; signal.signal(signal.SIGTERM,signal.SIG_IGN); '
                   'print("ready",flush=True); time.sleep(30)']
        with mock.patch.object(diag, 'ATTEMPT_SECONDS', 2), \
                mock.patch.object(diag, 'TERM_SECONDS', .2), \
                mock.patch.object(diag, 'KILL_SECONDS', .2):
            record = diag.execute(self.root, command, directory, dict(os.environ), time.monotonic() + 5)
        self.assertEqual(record['classification'], 'outer_timeout')
        self.assertEqual(record['signals'], ['SIGTERM', 'SIGKILL'])
        self.assertTrue(record['leader_reaped'])
        self.assertEqual(record['returncode'], -9)
        self.assertIn('ready', (directory / 'stdout.log').read_text())
        with self.assertRaises(ChildProcessError):
            os.waitpid(record['pid'], os.WNOHANG)

    def test_only_the_exact_target_panic_is_classified_as_original_timeout(self):
        panic = (f"thread '{diag.CASE}' (123) panicked at tests/direct_worker.rs:80:21:\n"
                 'worker shutdown timed out\n')
        self.assertEqual(diag.classification(101, False, '', panic), 'original_target_timeout')
        for code, expired, text in [(-9, False, panic), (101, True, panic),
                                     (101, False, panic.replace(diag.CASE, 'different_case')),
                                     (101, False, panic.replace('worker shutdown timed out', 'adapter unavailable'))]:
            with self.subTest(code=code, expired=expired, text=text):
                self.assertNotEqual(diag.classification(code, expired, '', text), 'original_target_timeout')
        self.assertEqual(diag.classification(0, False, self.success, ''), 'passed')
        for output, stderr in [(self.success.replace('20 passed', '19 passed'), ''),
                               (self.success.replace('0 ignored', '1 ignored'), ''),
                               (self.success, 'LLVM Profile Error: controlled write failure')]:
            self.assertEqual(diag.classification(0, False, output, stderr), 'other_failure')


if __name__ == '__main__':
    unittest.main()
