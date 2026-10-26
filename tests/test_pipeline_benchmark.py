# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import hashlib
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import subprocess
import sys
import shutil
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('pipeline', Path(__file__).resolve().parents[1] / 'scripts/benchmark-headless-pipeline.py')
pipeline = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pipeline)


class PipelineEvidence(unittest.TestCase):
    def build_fixture(self, directory):
        root = Path(directory) / 'source'
        for name in pipeline.SDK_BUILD_INPUTS | {'crates/example/src/lib.rs', 'crates/example/Cargo.toml'}:
            path = root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('synthetic build-record input\n')
        (root / 'rust-toolchain.toml').write_text('[toolchain]\nchannel="1.95.0"\n')
        build_json = Path(directory) / 'build.jsonl'
        build_json.write_text(json.dumps({'reason': 'build-finished', 'success': True}) + '\n')
        binary = Path(directory) / 'unexecuted-rust-fixture'
        binary.write_bytes(b'\x7fELFsynthetic executable identity; never executed')
        record = {'git_base': 'a'*40, 'source_tree': 'b'*40, 'rustc_vv': 'rustc 1.95.0 (retained fixture)\n',
                  'build_json_sha256': pipeline.digest(build_json), 'binary_sha256': pipeline.digest(binary),
                  'source_sha256': {str(path.relative_to(root)): pipeline.digest(path)
                                    for path in root.rglob('*') if path.is_file()}}
        record_path = Path(directory) / 'build-info.json'
        record_path.write_text(json.dumps(record))
        return root, record_path, build_json, binary, record

    def sdk_fixture(self, directory, root, compiler='/usr/bin/c++'):
        prefix = Path(directory)/'reference/install'
        prefix.mkdir(parents=True)
        builds, rows = {}, {}
        for name in ['sim','physics','sensors']:
            build = prefix.parent/'builds'/('gz-'+name)
            source = prefix.parent/'sources'/('gz-'+name)
            source.mkdir(parents=True)
            (build/'lib').mkdir(parents=True)
            (build/'lib'/('libgz-'+name+'.so')).write_bytes(b'nonexecuted SDK library fixture')
            (build/'CMakeCache.txt').write_text('CMAKE_BUILD_TYPE:STRING=Release\nCMAKE_HOME_DIRECTORY:INTERNAL='+str(source)+'\nCMAKE_CXX_FLAGS_RELEASE:STRING=-O3 -DNDEBUG\n')
            rows['gz-'+name] = {'directory':'builds/gz-'+name, 'source':'sources/gz-'+name,
                               'cmake_cache_sha256':pipeline.digest(build/'CMakeCache.txt'),
                               'library_files':pipeline.client_reference.builder.files_under(build/'lib')}
            builds[name] = build
        target = builds['sim']/'src/CMakeFiles/UNIT_ServerConfig_TEST.dir'
        target.mkdir(parents=True)
        (target/'flags.make').write_text('CXX_INCLUDES =\n')
        (target/'link.txt').write_text(str(compiler)+' -o native-fixture\n')
        sdk = {'repositories':[{'name':name, 'cmake_cache_sha256':row['cmake_cache_sha256']}
                               for name,row in rows.items()], 'installed_files':{}}
        (root/'planning/reference/headless-sdk-environment.json').write_text(json.dumps(sdk))
        registry = {'image_id':'sha256:'+'1'*64,
                    'sdk_environment_sha256':pipeline.digest(root/'planning/reference/headless-sdk-environment.json'),
                    'builds':rows, 'server_compiler_inputs':{name:pipeline.digest(target/name) for name in ['flags.make','link.txt']}}
        (root/'planning/reference/headless-sdk-build-libraries.json').write_text(json.dumps(registry))
        return prefix, builds, sdk

    def test_sdk_build_and_compiler_inputs_are_bound_before_timing(self):
        with tempfile.TemporaryDirectory() as directory:
            root, _, _, _, _ = self.build_fixture(directory)
            prefix, builds, sdk = self.sdk_fixture(directory,root)
            with patch.object(pipeline,'ROOT',root):
                libraries = pipeline.validate_sdk_builds(builds,prefix,sdk)
                self.assertEqual(len(libraries),3)
                target = builds['sim']/'src/CMakeFiles/UNIT_ServerConfig_TEST.dir'
                for path in [builds['sim']/'lib/libgz-sim.so', target/'flags.make', target/'link.txt',
                             builds['sim']/'CMakeCache.txt']:
                    original = path.read_bytes()
                    path.write_bytes(original+b'changed')
                    with self.subTest(path=path), self.assertRaises(ValueError):
                        pipeline.validate_sdk_builds(builds,prefix,sdk)
                    path.write_bytes(original)
                with self.assertRaises(ValueError):
                    pipeline.validate_sdk_builds(builds|{'sim':Path(directory)/'decoy'},prefix,sdk)
                pipeline.validate_sdk_loaded_libraries(libraries,libraries)
                changed = dict(libraries)
                changed[next(iter(changed))] = '0'*64
                with self.assertRaises(ValueError):
                    pipeline.validate_sdk_loaded_libraries(changed,libraries)
                outside = Path(directory)/'libgz-sim.so'
                outside.write_bytes(b'outside the qualified build')
                with self.assertRaises(ValueError):
                    pipeline.validate_sdk_loaded_libraries({str(outside):pipeline.digest(outside)},libraries)
                with self.assertRaises(ValueError):
                    pipeline.validate_sdk_loaded_libraries({str(outside):None},libraries)

    def test_sdk_installed_plugin_role_preserves_legacy_location_and_duplicate_guards(self):
        with tempfile.TemporaryDirectory() as directory:
            root, _, _, _, _ = self.build_fixture(directory)
            prefix, builds, sdk = self.sdk_fixture(directory,root)
            build_library=builds['sim']/'lib/libgz-sim.so'
            installed=prefix/'lib/plugins/libgz-sim.so'
            installed.parent.mkdir(parents=True)
            shutil.copy2(build_library,installed)
            required={'libgz-sim.so':build_library.parent}
            build_map={str(build_library):pipeline.digest(build_library)}
            installed_map={str(installed):pipeline.digest(installed)}
            allowed=build_map|installed_map
            pipeline.require_native_libraries(build_map,required)
            pipeline.require_native_libraries(installed_map,required,allowed)
            with self.assertRaises(ValueError):
                pipeline.require_native_libraries(installed_map,required)
            with self.assertRaises(ValueError):
                pipeline.require_native_libraries(installed_map,required,build_map)
            with self.assertRaises(ValueError):
                pipeline.require_native_libraries({str(installed):None},required,build_map)
            with self.assertRaises(ValueError):
                pipeline.require_native_libraries({str(installed):'0'*64},required,allowed)
            with self.assertRaises(ValueError):
                pipeline.require_native_libraries(build_map|installed_map,required,allowed)

    def test_sdk_build_record_needs_no_local_compiler_or_git_process(self):
        with tempfile.TemporaryDirectory() as directory:
            root, record_path, build_json, binary, record = self.build_fixture(directory)
            with patch.object(pipeline, 'ROOT', root), patch.object(pipeline.subprocess, 'run') as child:
                self.assertEqual(pipeline.validate_rust_build(record_path, build_json, binary), record)
            child.assert_not_called()

    def test_stale_build_binary_and_source_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root, record_path, build_json, binary, _ = self.build_fixture(directory)
            with patch.object(pipeline, 'ROOT', root):
                for path in [build_json, binary, root/'crates/example/src/lib.rs']:
                    original = path.read_bytes()
                    path.write_bytes(original + b'changed')
                    with self.subTest(path=path), self.assertRaises(ValueError):
                        pipeline.validate_rust_build(record_path, build_json, binary)
                    path.write_bytes(original)

    def test_failed_missing_or_duplicate_build_completion_cannot_qualify_an_elf(self):
        with tempfile.TemporaryDirectory() as directory:
            root, record_path, build_json, binary, record = self.build_fixture(directory)
            with patch.object(pipeline, 'ROOT', root):
                for messages in [[], [{'reason': 'build-finished', 'success': False}],
                                 [{'reason': 'build-finished', 'success': True}] * 2]:
                    build_json.write_text(''.join(json.dumps(row)+'\n' for row in messages))
                    changed = record | {'build_json_sha256': pipeline.digest(build_json)}
                    record_path.write_text(json.dumps(changed))
                    with self.subTest(messages=messages), self.assertRaisesRegex(ValueError, 'did not complete'):
                        pipeline.validate_rust_build(record_path, build_json, binary)

    def test_incomplete_escaping_and_wrong_toolchain_build_records_fail(self):
        with tempfile.TemporaryDirectory() as directory:
            root, record_path, build_json, binary, record = self.build_fixture(directory)
            variants = [record | {'git_base': 'invalid'}, record | {'source_tree': 'invalid'},
                        record | {'rustc_vv': 'rustc 1.94.0 (wrong compiler)\n'}]
            missing = copy.deepcopy(record)
            missing['source_sha256'].pop('crates/example/src/lib.rs')
            variants.append(missing)
            outside = Path(directory) / 'outside'
            outside.write_text('outside source')
            escape = copy.deepcopy(record)
            escape['source_sha256']['../outside'] = pipeline.digest(outside)
            variants.append(escape)
            with patch.object(pipeline, 'ROOT', root):
                for changed in variants:
                    record_path.write_text(json.dumps(changed))
                    with self.subTest(changed=changed), self.assertRaises(ValueError):
                        pipeline.validate_rust_build(record_path, build_json, binary)
                record_path.write_text(json.dumps(record))
                source = root/'crates/example/src/lib.rs'
                outside.write_bytes(source.read_bytes())
                source.unlink()
                source.symlink_to(outside)
                with self.assertRaises(ValueError):
                    pipeline.validate_rust_build(record_path, build_json, binary)

    def test_synthetic_compiler_mutations_are_rejected_before_any_measurement(self):
        # The compiler is controlled and the first timing subprocess is never run.
        class TimingReached(Exception):
            pass
        for mutation in ['none', 'source', 'captured', 'metadata', 'compiler', 'binary', 'fixture',
                         'build-source', 'wrong-compiler', 'build-library', 'build-flags', 'build-link',
                         'build-registry', 'binary-record']:
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as directory:
                root, record_path, build_json, binary, record = self.build_fixture(directory)
                compiler = Path(directory)/'c++'
                compiler.write_text('#!/bin/sh\nexit 99\n')
                compiler.chmod(0o755)
                identity = {'profile': pipeline.client_reference.SDK, 'libraries_sha256': {},
                            'expected_compiler_sha256': pipeline.digest(compiler)}
                if mutation == 'wrong-compiler':
                    identity['expected_compiler_sha256'] = '0'*64
                (root/'planning/baseline.json').write_text(json.dumps({'repositories': [
                    {'name': 'gz-'+name, 'commit': 'c'*40} for name in ['sim', 'physics', 'sensors']]}))
                (root/'planning/transport-reference-lock.json').write_text('{}')
                (root/'scripts/probe-headless-imu.py').write_text('def checked_sources(*args): return {}\n')
                prefix, builds, sdk = self.sdk_fixture(directory,root,compiler)
                metadata = prefix.parent/'environment.json'
                metadata.write_text(json.dumps(sdk))
                target = builds['sim']/'src/CMakeFiles/UNIT_ServerConfig_TEST.dir'
                rust_source = root/'crates/gz-sim-server/src/lib.rs'
                rust_source.parent.mkdir(parents=True)
                rust_source.write_text('// Synthetic source identity.\n')
                artifact = {'reason': 'compiler-artifact', 'executable': str(binary),
                            'target': {'name': 'gz_sim_server', 'src_path': str(rust_source) if mutation != 'build-source' else '/different/export/src/lib.rs'},
                            'profile': {'test': True, 'opt_level': '3', 'debug_assertions': False}}
                build_json.write_text(json.dumps(artifact)+'\n'+json.dumps({'reason':'build-finished','success':True})+'\n')
                record.update(build_json_sha256=pipeline.digest(build_json),
                              source_sha256={str(p.relative_to(root)): pipeline.digest(p) for p in root.rglob('*') if p.is_file()})
                record_path.write_text(json.dumps(record))
                work = Path(directory)/'measurement'
                source = root/'crates/gz-sim-server/tests/native_stream_benchmark.cc'
                fixture = root/'crates/gz-sim-scene/tests/fixtures/headless_imu.sdf'
                def child(command, **kwargs):
                    if command[0] == str(compiler):
                        captured = Path(command[command.index('-o')-1])
                        self.assertEqual(captured.read_bytes(), source.read_bytes())
                        Path(command[command.index('-o')+1]).write_bytes(b'\x7fELFsynthetic native binary')
                        changed = {'source': source, 'captured': captured, 'metadata': metadata,
                                   'compiler': compiler, 'binary': binary, 'fixture': fixture,
                                   'build-library':builds['sim']/'lib/libgz-sim.so',
                                   'build-flags':target/'flags.make', 'build-link':target/'link.txt',
                                   'build-registry':root/'planning/reference/headless-sdk-build-libraries.json',
                                   'binary-record':binary}.get(mutation)
                        if changed is not None:
                            changed.write_bytes(changed.read_bytes()+b'changed')
                        if mutation == 'binary-record':
                            changed_record = json.loads(record_path.read_text())
                            changed_record['binary_sha256'] = pipeline.digest(binary)
                            record_path.write_text(json.dumps(changed_record))
                        return subprocess.CompletedProcess(command, 0)
                    self.assertEqual(command[0], 'taskset')
                    raise TimingReached()
                arguments = ['benchmark']
                for name,build in builds.items():
                    arguments.extend(['--'+name+'-build',str(build)])
                arguments.extend(['--dependency-prefix',str(prefix),'--reference-prefix',str(prefix),
                                  '--rust-build-json',str(build_json),'--rust-build-info',str(record_path),
                                  '--work-dir',str(work),'--output',str(Path(directory)/'result.json'),
                                  '--path','streams','--reference-profile',pipeline.client_reference.SDK])
                with patch.object(pipeline,'ROOT',root), patch.object(sys,'argv',arguments), \
                     patch.object(pipeline.client_reference,'validate_reference',return_value=identity), \
                     patch.object(pipeline.subprocess,'check_output',return_value='synthetic compiler\n'), \
                     patch.object(pipeline.subprocess,'run',side_effect=child):
                    with self.assertRaises(TimingReached if mutation=='none' else ValueError):
                        pipeline.main()
                self.assertFalse((Path(directory)/'result.json').exists())

    def row(self):
        return dict(steps=2, nanos=10, logical_bytes=42, time_sum=5_000_000,
                    force_sum=19.62, final_x=0.003, fixture_hex=b'fixture'.hex())

    def decode(self, row):
        return pipeline.decode('PIPELINE ' + json.dumps(row), 2, hashlib.sha256(b'fixture').hexdigest())

    def test_complete_work_is_required(self):
        self.assertEqual(self.decode(self.row()), self.row())
        for key, value in [('steps', 1), ('nanos', 0), ('nanos', float('nan')),
                           ('logical_bytes', 0), ('time_sum', 0), ('force_sum', 0),
                           ('force_sum', float('nan')), ('final_x', float('nan')),
                           ('final_x', 0.004), ('fixture_hex', b'other'.hex())]:
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                self.decode(dict(self.row(), **{key: value}))

    def test_missing_and_duplicate_results_fail(self):
        expected = hashlib.sha256(b'fixture').hexdigest()
        for output in ['', 'PIPELINE {}\nPIPELINE {}']:
            with self.subTest(output=output), self.assertRaises(ValueError):
                pipeline.decode(output, 2, expected)

    def test_full_server_requires_native_warmup(self):
        row = self.row()
        row['warmup_force_x'] = 0.0
        fixture_hash = hashlib.sha256(b'fixture').hexdigest()
        row['fixture_hex'] = b'fixture'.hex()
        pipeline.decode('PIPELINE ' + json.dumps(row), 2, fixture_hash, warmup_force_x=0.0)
        for wrong in [1000.0, float('nan'), float('inf'), -float('inf'), 1e-9]:
            row['warmup_force_x'] = wrong
            with self.subTest(wrong=wrong), self.assertRaises(ValueError):
                pipeline.decode('PIPELINE ' + json.dumps(row), 2, fixture_hash, warmup_force_x=0.0)

    def test_complete_stream_evidence_and_opaque_id_accounting(self):
        row = self.row()
        row.update(warmup_force_x=0.0, wall_nanos=40_000_100, pacing_wait_nanos=40_000_000, request_period_us=20000, clock_count=2, pose_count=2,
                   clock_time_sum=5_000_000, pose_time_sum=5_000_000,
                   clock_bytes=60, pose_bytes=100, pose_bytes_without_id_values=92,
                   pose_ids=dict(ground=4, ball=7, base=5, body=8))
        fixture_hash = hashlib.sha256(bytes.fromhex(row['fixture_hex'])).hexdigest()
        def decode(value):
            return pipeline.decode('PIPELINE ' + json.dumps(value), 2, fixture_hash,
                                   warmup_force_x=0.0, streams=True, request_period_us=20000)
        decode(row)
        for key, wrong in [('request_period_us', 1000), ('wall_nanos', 10), ('pacing_wait_nanos', 50_000_000), ('clock_count', 1), ('pose_count', 3),
                           ('clock_time_sum', 4_000_000), ('pose_time_sum', 0),
                           ('clock_bytes', 0), ('pose_bytes_without_id_values', 91),
                           ('pose_ids', dict(ground=4, ball=7, base=5, body=7)),
                           ('pose_ids', dict(ground=4, ball=7, base=5, body=True)),
                           ('pose_ids', dict(ground=4, ball=7, base=5, body=2**32)),
                           ('pose_ids', dict(ground=4, ball=7, base=5))]:
            with self.subTest(key=key, wrong=wrong), self.assertRaises(ValueError):
                decode(dict(row, **{key: wrong}))
        larger_ids = dict(row, pose_ids=dict(ground=4, ball=7, base=5, body=128),
                          pose_bytes_without_id_values=90)
        decode(larger_ids)
