# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Synthetic protocol/selection controls, never physics observations or goldens."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import sys
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('b03_physics_benchmark', ROOT / 'scripts/benchmark-b03-physics.py')
benchmark = importlib.util.module_from_spec(spec)
spec.loader.exec_module(benchmark)


class B03RuntimeClosure(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.base = Path(self.directory.name).resolve()
        # Only the ELF marker is needed for metadata selection controls. These
        # files are never executed or described as valid dynamic libraries.
        self.executable = self.write('actor', b'\x7fELF synthetic actor')
        self.first = self.write('library one.so', b'\x7fELF synthetic first DSO')
        self.second = self.write('library-two.so', b'\x7fELF synthetic second DSO')
        self.descriptor = self.base / 'known-role.json'
        self.value = {'role': 'rust-scene-physics', 'executable_sha256': benchmark.digest(self.executable),
                      'files': {str(p): benchmark.digest(p) for p in [self.first, self.second]}}
        self.descriptor.write_text(json.dumps(self.value))

    def write(self, name, value):
        path = self.base / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(value)
        return path

    def maps(self, paths):
        lines = []
        for index, path in enumerate(paths, start=1):
            lines.append(f'{index*4096:08x}-{(index+1)*4096:08x} r-xp 00000000 00:01 {index} {path}')
        lines += ['10000000-10001000 rw-p 00000000 00:00 0 [heap]',
                  '10001000-10002000 r--p 00000000 00:00 0']
        return self.write('actor.maps', ('\n'.join(lines)+'\n').encode())

    def known(self):
        return benchmark.known_role_closure(self.descriptor, 'rust-scene-physics', self.executable)

    def test_exact_role_files_binary_hash_and_raw_map_are_bound(self):
        alias = self.base / 'observed-soname.so'
        alias.symlink_to(self.first)
        raw = self.maps([self.executable, alias, self.second, self.first])
        result = benchmark.verify_role_maps(raw, self.known(), self.executable)
        self.assertEqual(result['files_sha256'], self.value['files'])
        self.assertEqual(result['executable_sha256'], self.value['executable_sha256'])
        self.assertEqual(result['maps_sha256'], hashlib.sha256(raw.read_bytes()).hexdigest())
        self.assertEqual(result['observed_paths'][str(alias)], str(self.first))
        self.assertEqual(result['mapped_binary_paths'], [str(self.executable)])

    def test_role_or_build_swap_is_refused_before_any_observation(self):
        with self.assertRaisesRegex(RuntimeError, 'another workload'):
            benchmark.known_role_closure(self.descriptor, 'native-public-physics', self.executable)
        other = self.write('other-actor', b'\x7fELF different actor')
        with self.assertRaisesRegex(RuntimeError, 'another executable'):
            benchmark.known_role_closure(self.descriptor, 'rust-scene-physics', other)

    def test_missing_extra_and_changed_selected_libraries_are_refused(self):
        known = self.known()
        with self.assertRaisesRegex(RuntimeError, 'DSO set differs'):
            benchmark.verify_role_maps(self.maps([self.executable, self.first]), known, self.executable)
        extra = self.write('extra.so', b'\x7fELF unapproved DSO')
        with self.assertRaisesRegex(RuntimeError, 'DSO set differs'):
            benchmark.verify_role_maps(self.maps([self.executable, self.first, self.second, extra]), known, self.executable)
        self.first.write_bytes(b'\x7fELF mutated dependency')
        observed = {}
        with self.assertRaisesRegex(RuntimeError, 'DSO hash differs'):
            benchmark.verify_role_maps(self.maps([self.executable, self.first, self.second]), known, self.executable, observed)
        self.assertEqual(observed['files_sha256'][str(self.first)], benchmark.digest(self.first))
        self.assertEqual(len(observed['maps_sha256']), 64)

    def test_absent_or_changed_actor_and_deleted_missing_or_nonelf_maps_are_refused(self):
        known = self.known()
        for paths in [[self.first, self.second],
                      [self.executable, str(self.first)+' (deleted)', self.second],
                      [self.executable, self.base/'missing.so', self.second],
                      [self.executable, self.write('data', b'not an ELF'), self.second]]:
            with self.subTest(paths=paths), self.assertRaises((RuntimeError, OSError)):
                benchmark.verify_role_maps(self.maps(paths), known, self.executable)
        self.executable.write_bytes(b'\x7fELF mutated actor')
        with self.assertRaisesRegex(RuntimeError, 'executable changed'):
            benchmark.verify_role_maps(self.maps([self.executable, self.first, self.second]), known, self.executable)

    def test_unverified_descriptor_paths_hashes_and_malformed_maps_are_refused(self):
        variants = [{**self.value, 'files': {}}, {**self.value, 'files': {'relative.so': '0'*64}},
                    {**self.value, 'files': {str(self.first): 'not-a-hash'}},
                    {**self.value, 'unexpected': 1}]
        for variant in variants:
            self.descriptor.write_text(json.dumps(variant))
            with self.subTest(variant=variant), self.assertRaises((RuntimeError, OSError)):
                self.known()
        self.descriptor.write_text(json.dumps(self.value))
        for text in [b'', b'not a maps record\n', b'00002000-00001000 r-xp 00 00:01 1 /missing\n',
                     b'00001000-00002000 r-xp 00 00:01 1 relative.so\n']:
            raw = self.write('actor.maps', text)
            with self.subTest(text=text), self.assertRaises((RuntimeError, OSError)):
                benchmark.verify_role_maps(raw, self.known(), self.executable)

    def test_main_loads_both_role_closures_and_refuses_wrong_second_role_before_launch(self):
        root = self.base/'source'
        native = self.write('native-actor', b'\x7fELF native actor')
        native_role = {**self.value, 'role': 'native-public-physics', 'executable_sha256': benchmark.digest(native)}
        native_descriptor = self.write('native-role.json', json.dumps(native_role).encode())
        # Passing the native-role descriptor for Rust must stop the real main
        # wiring after native validation and before any process launch.
        build = self.write('build.json', b'{}')
        arguments = ['benchmark-b03-physics.py', '--native', str(native), '--rust', str(self.executable),
                     '--native-build-record', str(build), '--rust-build-record', str(build),
                     '--native-closure', str(native_descriptor), '--rust-closure', str(native_descriptor),
                     '--plugin', str(self.first), '--cpu', '0', '--output', str(self.base/'attempt')]
        with mock.patch.object(benchmark, 'ROOT', root), mock.patch.object(sys, 'argv', arguments), \
             mock.patch.object(benchmark.os, 'sched_getaffinity', return_value={0}), \
             mock.patch.object(benchmark, 'known_role_closure', wraps=benchmark.known_role_closure) as load, \
             mock.patch.object(benchmark.subprocess, 'Popen', side_effect=AssertionError('must not launch')) as launch:
            with self.assertRaisesRegex(RuntimeError, 'another workload'):
                benchmark.main()
        self.assertEqual([call.args[1] for call in load.call_args_list], ['native-public-physics', 'rust-scene-physics'])
        launch.assert_not_called()

    def test_build_record_retains_real_input_hashes_and_exact_optimized_assertions(self):
        compiler = self.write('compiler', b'synthetic compiler identity only')
        actor = self.write('actor-source.rs', b'// synthetic source')
        path = self.base / 'build.json'
        record = {'role': 'rust-scene-physics', 'executable_sha256': benchmark.digest(self.executable),
                  'compile_argv': [str(compiler), '-C', 'opt-level=3', '-Cdebug-assertions=yes', '-Coverflow-checks=yes'],
                  'inputs_sha256': {str(p): benchmark.digest(p) for p in [compiler, actor]}}
        path.write_text(json.dumps(record))
        self.assertEqual(benchmark.build_record(path, 'rust', self.executable, actor), record)
        # Cargo's actual release argv commonly spells enabled booleans `on`.
        for enabled in ['on', 'true', 'y']:
            cargo = {**record, 'compile_argv': [str(compiler), '-C', 'opt-level=3',
                     '-C', 'debug-assertions='+enabled, '-C', 'overflow-checks='+enabled]}
            path.write_text(json.dumps(cargo))
            with self.subTest(enabled=enabled):
                self.assertEqual(benchmark.build_record(path, 'rust', self.executable, actor), cargo)
        for key in ['debug-assertions', 'overflow-checks']:
            for disabled in ['off', 'false', '0', 'no', 'n', 'enabled', 'unknown']:
                options = {'debug-assertions': 'on', 'overflow-checks': 'on', key: disabled}
                argv = [str(compiler), '-Copt-level=3'] + ['-C'+k+'='+v for k, v in options.items()]
                path.write_text(json.dumps({**record, 'compile_argv': argv}))
                with self.subTest(key=key, disabled=disabled), self.assertRaises(RuntimeError):
                    benchmark.build_record(path, 'rust', self.executable, actor)
        for argv in [[str(compiler), '-Copt-level=30', '-Cdebug-assertions=yes', '-Coverflow-checks=yes'],
                     record['compile_argv'] + ['-Cdebug-assertions=no'],
                     [str(compiler), '-Copt-level=3'],
                     record['compile_argv'] + ['-ffast-math']]:
            path.write_text(json.dumps({**record, 'compile_argv': argv}))
            with self.subTest(argv=argv), self.assertRaises(RuntimeError):
                benchmark.build_record(path, 'rust', self.executable, actor)
        native = {**record, 'role': 'native-public-physics',
                  'compile_argv': [str(compiler), '-O3', '-UNDEBUG']}
        path.write_text(json.dumps(native))
        benchmark.build_record(path, 'native', self.executable, actor)
        for argv in [native['compile_argv'] + ['-O0'], native['compile_argv'] + ['-DNDEBUG'],
                     native['compile_argv'] + ['-D', 'NDEBUG=1']]:
            path.write_text(json.dumps({**native, 'compile_argv': argv}))
            with self.subTest(argv=argv), self.assertRaises(RuntimeError):
                benchmark.build_record(path, 'native', self.executable, actor)
        path.write_text(json.dumps(record))
        actor.write_bytes(b'// altered source')
        with self.assertRaisesRegex(RuntimeError, 'build input changed'):
            benchmark.build_record(path, 'rust', self.executable, actor)


class PhysicsWorkContract(unittest.TestCase):
    def state(self, role='native'):
        names = benchmark.expected_entities()
        identities = {name: i for i, name in enumerate(names, 1)}
        entities = [{'name': name, 'family': family, 'id': identities[name],
                     'parent_id': identities[name.rsplit('::', 1)[0]] if '::' in name else None}
                    for name, family in names.items()]
        frames = [{'tick': tick, 'time_ns': tick * 1000000,
                   'bodies': [{'name': name, 'id': identities[name], 'values': [0.0]*22}
                              for name, family in names.items() if family == 'BODY'],
                   'joints': [{'name': name, 'id': identities[name], 'values': [0.0]*2}
                              for name, family in names.items() if family == 'JOINT']}
                  for tick in range(1, 101)]
        return {'schema_version': 1, 'contract': benchmark.CONTRACT, 'role': benchmark.ROLES[role],
                'declared_uri': 'b03.sdf', 'body_fields': benchmark.FIELDS,
                'entities': entities, 'frames': frames}

    def test_all100_frames_and16200_values_are_compared_by_full_name(self):
        native = benchmark.normalized(self.state(), 'native')
        rust = self.state('rust')
        for entity in rust['entities']:
            entity['id'] += 1000
            if entity['parent_id'] is not None:
                entity['parent_id'] += 1000
        for frame in rust['frames']:
            frame['bodies'].reverse()
            frame['joints'].reverse()
            for row in frame['bodies'] + frame['joints']:
                row['id'] += 1000
        paired = benchmark.comparison(benchmark.normalized(rust, 'rust'), native)
        self.assertEqual(paired, {'values': 16200, 'maximum_delta': 0.0,
                                 'outside_1e_12': 0, 'first_outside': None})
        # An unsampled step must not disappear behind selected1/10/100 checks.
        rust['frames'][36]['bodies'][0]['values'][8] = 2e-12
        paired = benchmark.comparison(benchmark.normalized(rust, 'rust'), native)
        self.assertEqual(paired['outside_1e_12'], 1)
        self.assertEqual(paired['first_outside']['tick'], 37)

    def test_missing_duplicate_nonfinite_and_wrong_owner_rows_fail_closed(self):
        mutations = [lambda s: s['frames'].pop(1),
                     lambda s: s['frames'][1].update(tick=1),
                     lambda s: s['frames'][1].update(time_ns=0),
                     lambda s: s['frames'][50]['bodies'].pop(),
                     lambda s: s['frames'][50]['bodies'][1].update(name=s['frames'][50]['bodies'][0]['name']),
                     lambda s: s['frames'][50]['joints'][0].update(id=999999),
                     lambda s: s['frames'][50]['bodies'][0]['values'].pop(),
                     lambda s: s['frames'][50]['bodies'][0]['values'].__setitem__(4, float('nan')),
                     lambda s: s['entities'][0].update(parent_id=1),
                     lambda s: s['entities'][2].update(id=s['entities'][1]['id'])]
        for mutate in mutations:
            state = self.state()
            mutate(state)
            with self.subTest(mutate=mutate), self.assertRaises(RuntimeError):
                benchmark.normalized(state, 'native')

    def settings(self, role):
        evidence = {'gravity': 'public-readback', 'detector': 'public-readback', 'solver': 'public-readback',
                    'pair_max_contacts': 'public-readback', 'global_max_contacts': 'source-established',
                    'dt_ns': 'forward-step-input', 'solver_iterations': 'unsupported'} if role == 'native' else {
                        'gravity': 'sdf-input', 'detector': 'source-established', 'solver': 'source-established',
                        'pair_max_contacts': 'source-established', 'global_max_contacts': 'source-established',
                        'dt_ns': 'scene-readback', 'solver_iterations': 'unsupported'}
        values = {'gravity': [0,0,-9.8], 'detector': 'ode', 'solver': 'DantzigBoxedLcpSolver',
                  'pair_max_contacts': 'unlimited', 'global_max_contacts': 10000, 'dt_ns': 1000000,
                  'solver_iterations': None}
        result = {k: {'value': v, 'evidence': evidence[k]} for k, v in values.items()}
        if role == 'native':
            result['pair_max_contacts']['raw'] = '18446744073709551615'
        return result

    def test_unlimited_pair_is_not_global20_or_zero_solver_iterations(self):
        self.assertEqual(benchmark.settings(self.settings('native'), 'native'),
                         benchmark.settings(self.settings('rust'), 'rust'))
        for key, replacement in [('pair_max_contacts', 20), ('global_max_contacts', 20),
                                 ('solver_iterations', 0)]:
            settings = self.settings('native')
            settings[key]['value'] = replacement
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                benchmark.settings(settings, 'native')
        settings = self.settings('native')
        settings['global_max_contacts']['evidence'] = 'public-readback'
        with self.assertRaises(RuntimeError):
            benchmark.settings(settings, 'native')
        settings = self.settings('rust')
        settings['pair_max_contacts']['evidence'] = 'public-readback'
        with self.assertRaises(RuntimeError):
            benchmark.settings(settings, 'rust')
        settings = self.settings('rust')
        settings['gravity']['evidence'] = 'scene-readback'
        with self.assertRaises(RuntimeError):
            benchmark.settings(settings, 'rust')

    def test_sampled_default_compatibility_does_not_replace_full100_pairing(self):
        native = benchmark.normalized(self.state(), 'native')
        fixture = {'schema_version': 1, 'xml_sha256': benchmark.XML_SHA, 'body_fields': benchmark.FIELDS,
                   'frames': [copy.deepcopy(self.state()['frames'][i-1]) for i in [1,10,100]]}
        self.assertEqual(benchmark.default_comparison(native, fixture)['values'], 486)
        fixture['frames'][2]['bodies'][0]['values'][0] = 1e-8
        actual = benchmark.default_comparison(native, fixture)
        self.assertEqual(actual['outside_1e_12'], 1)
        self.assertEqual(actual['first_outside']['tick'], 100)
        fixture['frames'].pop()
        with self.assertRaises(RuntimeError):
            benchmark.default_comparison(native, fixture)

    def test_actual_work_counts_and_no_network_are_required(self):
        value = {'schema_version': 1, 'contract': benchmark.CONTRACT, 'role': benchmark.ROLES['rust'],
                 'mode': 'measure', 'steps': 100, 'dt_ns': 1000000, 'body_queries': 700,
                 'joint_queries': 400, 'worlds': 1, 'bodies': 7, 'joints': 4, 'colliders': 15,
                 'sensors': 0, 'rendering': False, 'network': False, 'window_wall_ns': 1}
        benchmark.metrics(value, 'rust', 'measure')
        for key, replacement in [('steps', 99), ('body_queries', 699), ('joint_queries', 399),
                                 ('network', True), ('window_wall_ns', 0), ('mode', 'correctness')]:
            changed = {**value, key: replacement}
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                benchmark.metrics(changed, 'rust', 'measure')
        with self.assertRaises(RuntimeError):
            benchmark.parse('{"value":1,"value":2}')
        with self.assertRaises(RuntimeError):
            benchmark.parse('{"value":NaN}')


if __name__ == '__main__':
    unittest.main()
