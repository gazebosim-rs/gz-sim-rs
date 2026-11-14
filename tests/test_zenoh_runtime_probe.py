# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Synthetic metadata/process controls; never a native qualification campaign."""
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import MagicMock, patch

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('runtime_probe_test', ROOT / 'scripts/zenoh_runtime_probe.py')
runtime = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runtime)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def metadata():
    libraries = {name: {'path': path, 'sha256': old}
                 for name, (path, old, _) in runtime.DELTAS.items()}
    libraries.update({f'unchanged-{i}': {'path': f'/synthetic/library-{i}', 'sha256': 'a' * 64}
                      for i in range(21)})
    return {'profile': 'synthetic-not-native-evidence', 'lock': {'vendor': {'prefix': 'vendor'}},
            'probe': {'libraries': libraries, 'transport_dynamic_sha256': digest(b'dynamic')}}


class RuntimeSelectionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.reference = self.root / 'reference'
        (self.reference / 'probe').mkdir(parents=True)
        (self.reference / 'probe/probe').write_bytes(b'nonexecuted synthetic binary')
        (self.reference / 'probe/probe.cc').write_bytes(b'synthetic Source')
        self.base = self.reference / 'environment.json'
        self.base.write_text(json.dumps(metadata()))
        self.packet = self.root / 'packet'
        self.packet.mkdir()
        self.loader = self.root / 'loader'
        self.loader.write_bytes(b'nonexecuted synthetic loader')
        self.addCleanup(patch.stopall)
        patch.dict(os.environ, {}, clear=True).start()
        patch.object(runtime, 'BASE_SHA', digest(self.base.read_bytes())).start()
        patch.object(runtime, 'PROBE_SHA', digest((self.reference / 'probe/probe').read_bytes())).start()
        patch.object(runtime, 'LOADER', str(self.loader)).start()
        patch.object(runtime, 'LOADER_SHA', digest(self.loader.read_bytes())).start()

    def put_json(self, name, value):
        path = self.packet / name
        path.write_text(json.dumps(value))
        return runtime.card(path)

    def completed(self):
        source = runtime.card(Path(runtime.__file__).absolute())
        result = {**runtime.NOTICES, 'schema_version': 1,
                  'state': 'ZENOH_RUNTIME_PROBE_COMPLETED_NOT_SELECTED',
                  'reference_root': str(self.reference), 'base_environment': runtime.card(self.base),
                  'helper_source': source, 'transition': self.put_json('transition.json', {}),
                  'probe_binary': runtime.card(self.reference / 'probe/probe'),
                  'probe_source': runtime.card(self.reference / 'probe/probe.cc'),
                  'loader': runtime.card(self.loader), 'config': self.put_json('config.json', runtime.loopback_config()),
                  'partition': 'runtime-probe-' + '1' * 32, 'stages': [],
                  'resource_minima': {str(self.packet): 2 * 1024 ** 3, '/dev/shm': 2 * 1024 ** 3,
                                      '/run/user/1000': 2 * 1024 ** 3, 'MemAvailable': 16 * 1024 ** 3},
                  'errors': [], 'all_children_reaped': True, 'internal_exit_code': 0}
        result['probe_environment'] = {'GZ_TRANSPORT_IMPLEMENTATION': 'zenoh',
            'GZ_PARTITION': result['partition'], 'ZENOH_CONFIG': result['config']['path'],
            'LD_LIBRARY_PATH': str(self.reference / 'install/lib') + os.pathsep
                + str(self.reference / 'vendor/vendor/lib')}
        for role in ['linkage-before', 'dynamic-before', 'probe', 'linkage-after', 'dynamic-after']:
            if role == 'probe':
                argv, output = [str(self.reference / 'probe/probe')], b'zenoh client ready\n'
            elif role.startswith('linkage'):
                argv, output = ['/usr/bin/ldd', str(self.reference / 'probe/probe')], b'synthetic linkage\n'
            else:
                argv, output = ['/usr/bin/readelf', '-d', str(self.reference / 'install/lib/libgz-transport.so.16')], b'dynamic\n'
            stdout, stderr = self.packet / (role + '.stdout'), self.packet / (role + '.stderr')
            stdout.write_bytes(output)
            stderr.write_bytes(b'')
            result['stages'].append({'role': role, 'argv': argv, 'actual_exit_code': 0,
                                     'reaped': True, 'error': None, 'elapsed_seconds': 0.01,
                                     'stdout': runtime.card(stdout), 'stderr': runtime.card(stderr)})
        return result

    def select(self, result=None, session=None):
        result = self.completed() if result is None else result
        result_card = self.put_json('result.json', result)
        receipt = {'state': 'ZENOH_RUNTIME_PROBE_ROOT_ACTUAL0',
                   'method_path': result['helper_source']['path'],
                   'method_sha256': result['helper_source']['sha256'],
                   'result_path': result_card['path'], 'result_sha256': result_card['sha256'],
                   'result_bytes': result_card['bytes'], 'base_environment': result['base_environment'],
                   'reference_root': str(self.reference), 'actual_root_exit_code': 0,
                   'tool_exit_basis': {'session': session, 'completion_chunk': 'synthetic-external-completion',
                                       'actual_exit_code': 0}}
        selection = {**runtime.NOTICES, 'schema_version': 1, 'state': 'ZENOH_RUNTIME_PROBE_SELECTED',
                     'reference_root': str(self.reference), 'base_environment': result['base_environment'],
                     'result': result_card, 'actual_exit': self.put_json('actual-exit.json', receipt)}
        return Path(self.put_json('selection.json', selection)['path'])

    def validate(self, manifest):
        with patch.object(runtime, 'guard_reference', return_value=metadata()), \
             patch.object(runtime.bootstrap, 'validate_linkage', return_value=runtime.projection(metadata())['probe']['libraries']):
            return runtime.validate_selection(self.reference, manifest)

    def test_default_calls_unchanged_bootstrap_and_returns_old_metadata(self):
        old = metadata()
        with patch.object(runtime.bootstrap, 'validate_reference', return_value=old) as validate:
            selection = runtime.validate_selection(self.reference)
        validate.assert_called_once_with(self.reference)
        self.assertIs(selection['metadata'], old)
        self.assertEqual(set(selection), {'metadata', 'base_environment', 'runtime_probe', 'helper_source'})
        self.assertIsNone(selection['runtime_probe'])
        self.assertEqual(selection['base_environment'], runtime.card(self.base))
        self.assertEqual(runtime.proof_files(selection), {})

    def test_invalid_or_empty_explicit_selection_never_falls_back(self):
        for value in ['', str(self.root / 'missing')]:
            with self.subTest(value=value), patch.dict(os.environ, {runtime.ENVIRONMENT: value}), \
                 patch.object(runtime.bootstrap, 'validate_reference') as validate:
                with self.assertRaises((ValueError, FileNotFoundError)):
                    runtime.validate_selection(self.reference)
                validate.assert_not_called()

    def test_projection_changes_only_three_sha_leaves_and_preserves_original(self):
        old = metadata()
        before = copy.deepcopy(old)
        changed = runtime.projection(old)
        self.assertEqual(old, before)
        for name, (_, previous, current) in runtime.DELTAS.items():
            self.assertEqual(changed['probe']['libraries'][name]['sha256'], current)
            changed['probe']['libraries'][name]['sha256'] = previous
        self.assertEqual(changed, old)

    def test_projection_rejects_missing_extra_wrong_path_and_unreviewed_sha(self):
        for kind in ['missing', 'extra', 'path', 'sha']:
            changed = metadata()
            libraries = changed['probe']['libraries']
            if kind == 'missing':
                libraries.pop('libc.so.6')
            elif kind == 'extra':
                libraries['extra'] = {'path': '/extra', 'sha256': 'a' * 64}
            else:
                libraries['libc.so.6'][kind if kind == 'path' else 'sha256'] = '/foreign' if kind == 'path' else '0' * 64
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                runtime.projection(changed)

    def test_internal_result_alone_is_not_selection_eligible(self):
        result = self.put_json('result.json', self.completed())
        with self.assertRaises(ValueError):
            self.validate(result['path'])

    def test_root_zero_session_or_direct_completion_binds_original_paths_and_current_source_hash(self):
        for session in [None, 123]:
            with self.subTest(session=session):
                path = self.select(session=session)
                selected = self.validate(path)
                self.assertEqual(selected['metadata'], metadata())
                self.assertEqual(selected['runtime_probe'], runtime.card(path))
                self.assertEqual(selected['helper_source']['sha256'], runtime.card(Path(runtime.__file__).absolute())['sha256'])

    def test_staged_source_relocation_preserves_original_actual_method_path(self):
        result = self.completed()
        result['helper_source']['path'] = '/preserved/original/zenoh_runtime_probe.py'
        selected = self.validate(self.select(result))
        self.assertEqual(selected['helper_source']['path'], str(Path(runtime.__file__).absolute()))

    def test_four_delta_transition_and_diagnostic_root_receipt_are_closed(self):
        diagnostic = self.put_json('diagnostic.json', {
            'internal_exit_code': 0, 'errors': [], 'observed_count': 351,
            'matching_count': 347, 'delta_count': 4, 'child_present_after_cleanup': False})
        receipt = self.put_json('diagnostic-actual.json', {'actual_root_exit_code': 0,
            'tool_exit_basis': {'actual_exit_code': 0}, 'report': diagnostic, 'all_children_reaped': True})
        deltas = [{'path': p, 'previous_sha256': old, 'current_sha256': new}
                  for p, old, new in runtime.DELTAS.values()]
        # The loader delta is independently guarded, not invented as an ldd => row.
        deltas.append({'path': runtime.LOADER,
            'previous_sha256': 'c20a2dc8917c755f02b94049356320fe1f62ac7d9f8994731f807d9df39302da',
            'current_sha256': runtime.LOADER_SHA})
        original = {'environment_update_M58': {'changed_files': deltas,
            'retained_complete_inventory': diagnostic, 'retained_inventory_actual_Root_receipt': receipt}}
        for fault in ['valid', 'missing', 'extra', 'wrong']:
            value = copy.deepcopy(original)
            rows = value['environment_update_M58']['changed_files']
            if fault == 'missing':
                rows.pop()
            elif fault == 'extra':
                rows.append(copy.deepcopy(rows[0]))
            elif fault == 'wrong':
                rows[0]['current_sha256'] = '0' * 64
            source = self.put_json('transition-test.json', value)
            with self.subTest(fault=fault), patch.object(runtime, 'TRANSITION_SHA', source['sha256']):
                if fault == 'valid':
                    self.assertEqual(set(runtime.transition_proofs(source)),
                        {'transition.json', 'dependency-diagnostic.json', 'dependency-diagnostic-actual-exit.json'})
                else:
                    with self.assertRaises(ValueError):
                        runtime.transition_proofs(source)

    def test_fixed_proof_bundle_detects_transcript_mutation(self):
        selected = self.validate(self.select())
        proof = self.put_json('synthetic-dependency-proof.json', {})
        retained = {key: proof for key in ['transition.json', 'dependency-diagnostic.json',
                                           'dependency-diagnostic-actual-exit.json']}
        with patch.object(runtime, 'transition_proofs', return_value=retained):
            files = runtime.proof_files(selected)
            self.assertIn('probe.stdout', files)
            self.assertEqual(set(files), set(runtime.PROOF_ROLES))
            self.assertEqual(len(files), 17)
            (self.packet / 'probe.stdout').write_bytes(b'mutated proof')
            with self.assertRaises(ValueError):
                runtime.proof_files(selected)

    def test_wrong_base_probe_loader_and_helper_are_refused(self):
        for field in ['base_environment', 'probe_binary', 'loader', 'helper_source']:
            result = self.completed()
            result[field]['sha256'] = '0' * 64
            path = self.select(result)
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.validate(path)

    def test_nonzero_timeout_partial_stages_and_false_boolean_exit_are_refused(self):
        for fault in ['nonzero', 'timeout', 'partial', 'unreaped', 'boolean', 'stderr', 'floors']:
            result = self.completed()
            if fault == 'nonzero':
                result['internal_exit_code'] = 1
            elif fault == 'timeout':
                result['stages'][2]['elapsed_seconds'] = 15.01
            elif fault == 'partial':
                result['stages'].pop()
            elif fault == 'unreaped':
                result['stages'][2]['reaped'] = False
            elif fault == 'boolean':
                result['stages'][2]['actual_exit_code'] = False
            elif fault == 'stderr':
                path = self.packet / 'probe.stderr'
                path.write_bytes(b'actual retained failure')
                result['stages'][2]['stderr'] = runtime.card(path)
            else:
                result['resource_minima']['MemAvailable'] = 0
            with self.subTest(fault=fault), self.assertRaises(ValueError):
                self.validate(self.select(result))

    def test_external_root_nonzero_missing_completion_and_mutated_transcript_fail(self):
        for fault in ['root', 'chunk', 'transcript']:
            path = self.select()
            selection = json.loads(path.read_text())
            if fault == 'transcript':
                (self.packet / 'probe.stdout').write_bytes(b'changed after selection')
            else:
                receipt = json.loads(Path(selection['actual_exit']['path']).read_text())
                if fault == 'root':
                    receipt['actual_root_exit_code'] = 1
                else:
                    receipt['tool_exit_basis']['completion_chunk'] = ''
                selection['actual_exit'] = self.put_json('actual-exit.json', receipt)
                self.put_json('selection.json', selection)
            with self.subTest(fault=fault), self.assertRaises(ValueError):
                self.validate(path)

    def test_card_bytes_and_ordinary_file_bounds_are_enforced(self):
        file = self.packet / 'ordinary'
        file.write_bytes(b'abc')
        value = runtime.card(file)
        file.write_bytes(b'abd')
        with self.assertRaises(ValueError):
            runtime.checked_card(value)
        with self.assertRaises(ValueError):
            runtime.read_file(file, 2)
        alias = self.packet / 'alias'
        alias.symlink_to(file)
        with self.assertRaises(OSError):
            runtime.read_file(alias)

    def test_stage_timeout_reaps_owned_child_and_retains_partial_streams(self):
        child = MagicMock()
        child.pid = 12345
        child.poll.return_value = None
        child.wait.return_value = -15
        stages = []
        def create(*args, **kwargs):
            kwargs['stdout'].write(b'partial actual stream')
            kwargs['stdout'].flush()
            return child
        with patch.object(runtime.subprocess, 'Popen', side_effect=create), \
             patch.object(runtime.os, 'killpg') as kill, \
             patch.object(runtime.time, 'monotonic', side_effect=[0, 16, 17]):
            with self.assertRaisesRegex(ValueError, 'deadline'):
                runtime.run_stage('probe', ['/nonexecuted'], {}, self.packet, stages, {})
        kill.assert_called_once_with(child.pid, runtime.signal.SIGTERM)
        child.wait.assert_called_once_with(timeout=2)
        self.assertTrue(stages[0]['reaped'])
        self.assertEqual(stages[0]['actual_exit_code'], -15)
        self.assertEqual(runtime.checked_card(stages[0]['stdout']), b'partial actual stream')
        self.assertIn('deadline', stages[0]['error'])

    def test_probe_failure_still_persists_internal_failure_and_never_selection(self):
        output = self.root / 'fresh-packet'
        with patch.object(runtime, 'floors', return_value={}), \
             patch.object(runtime, 'guard_reference', side_effect=ValueError('current dependency changed')):
            self.assertEqual(runtime.attest(self.reference, output, self.base), 1)
        result = json.loads((output / 'result.json').read_text())
        self.assertEqual(result['internal_exit_code'], 1)
        self.assertIn('current dependency changed', result['errors'][0])
        self.assertEqual(result['stages'], [])
        self.assertFalse((output / 'selection.json').exists())
        self.assertFalse((output / 'actual-exit.json').exists())

    def test_cli_termination_is_controlled_and_previous_handlers_are_restored(self):
        for number in (runtime.signal.SIGTERM, runtime.signal.SIGHUP, runtime.signal.SIGINT):
            previous = {value: object() for value in
                        (runtime.signal.SIGTERM, runtime.signal.SIGHUP, runtime.signal.SIGINT)}
            handlers = dict(previous)
            output = self.root / ('late-signal-' + str(number))
            output.mkdir()

            def install(value, handler):
                handlers[value] = handler

            def interrupted_run(*_args):
                self.assertIsNone(handlers[number](number, None))
                return 0

            argv = ['zenoh_runtime_probe.py', '--reference', str(self.reference),
                    '--output', str(output), '--transition', str(self.base)]
            with self.subTest(signal=number), patch('sys.argv', argv), \
                 patch.object(runtime.signal, 'getsignal', side_effect=previous.__getitem__), \
                 patch.object(runtime.signal, 'signal', side_effect=install), \
                 patch.object(runtime, 'attest', side_effect=interrupted_run):
                self.assertEqual(runtime.main(), 1)
            self.assertEqual(handlers, previous)
            self.assertIsNone(runtime._CLI_INTERRUPTION)
            self.assertTrue((output / runtime.STOPPED_MARKER).is_file())

    def test_controlled_interruption_reaps_child_and_keeps_partial_failure_stream(self):
        child = MagicMock()
        child.pid = 12345
        child.poll.side_effect = [RuntimeError('runtime probe interrupted by signal 15'), None]
        child.wait.return_value = -15
        stages = []

        def create(*_args, **kwargs):
            kwargs['stderr'].write(b'partial interrupted stream')
            kwargs['stderr'].flush()
            return child

        with patch.object(runtime.subprocess, 'Popen', side_effect=create), \
             patch.object(runtime.os, 'killpg') as kill, \
             patch.object(runtime.time, 'monotonic', side_effect=[0, 1]):
            with self.assertRaisesRegex(RuntimeError, 'interrupted by signal'):
                runtime.run_stage('probe', ['/nonexecuted'], {}, self.packet, stages, {})
        kill.assert_called_once_with(child.pid, runtime.signal.SIGTERM)
        self.assertTrue(stages[0]['reaped'])
        self.assertEqual(stages[0]['actual_exit_code'], -15)
        self.assertEqual(runtime.checked_card(stages[0]['stderr']), b'partial interrupted stream')
        self.assertIn('interrupted by signal', stages[0]['error'])

    def test_fast_exit_samples_floor_before_success_and_retains_actual_low_minimum(self):
        child = MagicMock()
        child.poll.return_value = 0
        child.wait.return_value = 0
        stages, minima = [], {}

        def low_floor(_path, saved):
            saved['MemAvailable'] = 0
            raise ValueError('post-exit resource floor')

        with patch.object(runtime.subprocess, 'Popen', return_value=child), \
             patch.object(runtime, 'floors', side_effect=low_floor) as sample, \
             patch.object(runtime.time, 'monotonic', side_effect=[0, 1]):
            with self.assertRaisesRegex(ValueError, 'post-exit resource floor'):
                runtime.run_stage('probe', ['/nonexecuted'], {}, self.packet, stages, minima)
        sample.assert_called_once_with(self.packet, minima)
        self.assertEqual(minima['MemAvailable'], 0)
        self.assertTrue(stages[0]['reaped'])
        self.assertIn('post-exit resource floor', stages[0]['error'])

    def test_floor_records_actual_values_before_refusing_low_observation(self):
        minima = {}
        usage = MagicMock()
        usage.free = 2 * 1024 ** 3
        with patch.object(runtime.shutil, 'disk_usage', return_value=usage), \
             patch.object(Path, 'read_text', return_value='MemAvailable: 0 kB\n'):
            with self.assertRaisesRegex(ValueError, 'resource floor'):
                runtime.floors(self.packet, minima)
        self.assertEqual(minima['MemAvailable'], 0)
        self.assertEqual(minima[str(self.packet)], 2 * 1024 ** 3)

    def test_late_persistence_floor_failure_is_nonzero_with_actual_minimum_veto(self):
        output = self.root / 'late-packet'
        samples = []

        def sample(_path, minima):
            samples.append(None)
            minima['MemAvailable'] = 0 if len(samples) == 3 else 16 * 1024 ** 3
            if len(samples) == 3:
                raise ValueError('late resource floor')
            return dict(minima)

        def stage(_role, _argv, _env, _directory, rows, _minima):
            # Only finalization is under test; no native or subprocess work occurs.
            rows.append({'reaped': True})

        env = {'GZ_TRANSPORT_IMPLEMENTATION': 'zenoh', 'GZ_PARTITION': 'replaced',
               'ZENOH_CONFIG': 'synthetic', 'LD_LIBRARY_PATH': 'synthetic'}
        with patch.object(runtime, 'floors', side_effect=sample), \
             patch.object(runtime, 'guard_reference', return_value=metadata()), \
             patch.object(runtime.bootstrap, 'build_environment', return_value={}), \
             patch.object(runtime.bootstrap, 'probe_environment', return_value=env), \
             patch.object(runtime, 'run_stage', side_effect=stage), \
             patch.object(runtime, 'validate_stage'), patch.object(runtime, 'validate_result'):
            self.assertEqual(runtime.attest(self.reference, output, self.base), 1)
        saved = json.loads((output / 'result.json').read_text())
        stopped = json.loads((output / runtime.STOPPED_MARKER).read_text())
        self.assertEqual(saved['internal_exit_code'], 0)  # Prefix, vetoed after persistence.
        self.assertEqual(stopped['internal_exit_code'], 1)
        self.assertEqual(stopped['resource_minima']['MemAvailable'], 0)
        self.assertIn('late resource floor', stopped['errors'][0])

    def test_existing_and_dangling_stopped_markers_veto_selection_before_sdk_queries(self):
        marker = self.packet / runtime.STOPPED_MARKER
        for dangling in (False, True):
            path = self.select()
            if dangling:
                marker.symlink_to(self.packet / 'absent-target')
            else:
                marker.write_text('{}')
            with self.subTest(dangling=dangling), \
                 patch.object(runtime, 'guard_reference', side_effect=AssertionError('must not query SDK')):
                with self.assertRaisesRegex(ValueError, 'stopped marker'):
                    runtime.validate_selection(self.reference, path)
            marker.unlink()

    def test_noncanonical_reference_and_physically_nested_output_refuse_before_mkdir(self):
        alias = self.root / 'reference-alias'
        alias.symlink_to(self.reference, target_is_directory=True)
        unused = self.root / 'unused'
        unused.mkdir()
        for reference, output in [
            (alias, self.reference / 'alias-packet'),
            (unused / '..' / 'reference', self.reference / 'dotdot-packet'),
            (self.reference, alias / 'ancestor-packet'),
            (self.reference, self.reference / 'nested-packet'),
        ]:
            with self.subTest(reference=reference, output=output), \
                 self.assertRaisesRegex(ValueError, 'canonical and physically separate'):
                runtime.attest(reference, output, self.base)
            self.assertFalse(output.exists())

    def test_signal_inside_popen_before_handle_return_is_reaped_and_retained(self):
        for number in (runtime.signal.SIGTERM, runtime.signal.SIGHUP, runtime.signal.SIGINT):
            output = self.root / ('spawn-signal-' + str(number))
            previous = {value: object() for value in
                        (runtime.signal.SIGTERM, runtime.signal.SIGHUP, runtime.signal.SIGINT)}
            handlers = dict(previous)
            child = MagicMock()
            child.pid = 12345
            child.poll.return_value = None
            child.wait.return_value = -15

            def install(value, handler):
                handlers[value] = handler

            def create(*_args, **kwargs):
                kwargs['stderr'].write(b'partial stream before handle assignment')
                kwargs['stderr'].flush()
                # The parent owns no assigned handle until this factory returns.
                self.assertIsNone(handlers[number](number, None))
                return child

            argv = ['zenoh_runtime_probe.py', '--reference', str(self.reference),
                    '--output', str(output), '--transition', str(self.base)]
            env = {'GZ_TRANSPORT_IMPLEMENTATION': 'zenoh', 'GZ_PARTITION': 'replaced',
                   'ZENOH_CONFIG': 'synthetic', 'LD_LIBRARY_PATH': 'synthetic'}
            with self.subTest(signal=number), patch('sys.argv', argv), \
                 patch.object(runtime.signal, 'getsignal', side_effect=previous.__getitem__), \
                 patch.object(runtime.signal, 'signal', side_effect=install), \
                 patch.object(runtime, 'floors', return_value={}), \
                 patch.object(runtime, 'guard_reference', return_value=metadata()), \
                 patch.object(runtime.bootstrap, 'build_environment', return_value={}), \
                 patch.object(runtime.bootstrap, 'probe_environment', return_value=env), \
                 patch.object(runtime.subprocess, 'Popen', side_effect=create), \
                 patch.object(runtime.os, 'killpg') as kill:
                self.assertEqual(runtime.main(), 1)
            kill.assert_called_once_with(child.pid, runtime.signal.SIGTERM)
            child.wait.assert_called_once_with(timeout=2)
            result = json.loads((output / 'result.json').read_text())
            self.assertEqual(result['internal_exit_code'], 1)
            self.assertTrue(result['all_children_reaped'])
            self.assertEqual(result['stages'][0]['actual_exit_code'], -15)
            self.assertTrue(result['stages'][0]['reaped'])
            self.assertEqual(runtime.checked_card(result['stages'][0]['stderr']),
                             b'partial stream before handle assignment')
            self.assertIn('interrupted by signal', result['errors'][0])
            self.assertTrue((output / runtime.STOPPED_MARKER).is_file())
            self.assertEqual(handlers, previous)
            self.assertIsNone(runtime._CLI_INTERRUPTION)

    def test_signal_during_real_rejected_preflight_never_writes_protected_output(self):
        actual_attest = runtime.attest
        before = {str(path.relative_to(self.reference)): path.read_bytes()
                  for path in self.reference.rglob('*') if path.is_file()}
        for number in (runtime.signal.SIGTERM, runtime.signal.SIGHUP, runtime.signal.SIGINT):
            previous = {value: object() for value in
                        (runtime.signal.SIGTERM, runtime.signal.SIGHUP, runtime.signal.SIGINT)}
            handlers = dict(previous)

            def install(value, handler):
                handlers[value] = handler

            def interrupted_preflight(*args):
                self.assertIsNone(handlers[number](number, None))
                return actual_attest(*args)

            argv = ['zenoh_runtime_probe.py', '--reference', str(self.reference),
                    '--output', str(self.reference), '--transition', str(self.base)]
            with self.subTest(signal=number), patch('sys.argv', argv), \
                 patch.object(runtime.signal, 'getsignal', side_effect=previous.__getitem__), \
                 patch.object(runtime.signal, 'signal', side_effect=install), \
                 patch.object(runtime, 'attest', side_effect=interrupted_preflight):
                with self.assertRaisesRegex(ValueError, 'canonical and physically separate'):
                    runtime.main()
            after = {str(path.relative_to(self.reference)): path.read_bytes()
                     for path in self.reference.rglob('*') if path.is_file()}
            self.assertEqual(after, before)
            self.assertFalse((self.reference / runtime.STOPPED_MARKER).exists())
            self.assertEqual(handlers, previous)
            self.assertIsNone(runtime._CLI_INTERRUPTION)


if __name__ == '__main__':
    unittest.main()
