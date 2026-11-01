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


class ZenohAppPrerequisites(unittest.TestCase):
    # These are local rejection controls, never a successful app environment.
    def test_client_reference_cannot_qualify_an_application(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for profile in ['gazebo-zenoh-client-v1', pipeline.client_reference.SDK, 'unknown']:
                (root / 'environment.json').write_text(json.dumps({'schema_version': 1, 'profile': profile}))
                with self.subTest(profile=profile), self.assertRaisesRegex(ValueError, 'full-app profile'):
                    pipeline.validate_app_reference(root, {}, root, root)
            with self.assertRaisesRegex(ValueError, 'real directory'):
                pipeline.validate_app_reference(root / 'missing', {}, root, root)

    def test_profile_selection_refuses_missing_or_cross_profile_app_before_execution(self):
        arguments = ['benchmark', '--rust-build-info', '/unexecuted/record']
        for name in ['sim-build', 'physics-build', 'sensors-build', 'dependency-prefix',
                     'reference-prefix', 'rust-build-json', 'work-dir', 'output']:
            arguments += ['--' + name, '/unexecuted/' + name]
        for extra, message in [
            (['--reference-profile', pipeline.FULL_APP], 'requires a full-app reference'),
            (['--reference-profile', pipeline.client_reference.SDK, '--app-reference', '/unexecuted/app'],
             'requires the explicit Zenoh full-app profile'),
        ]:
            with self.subTest(extra=extra), patch.object(sys, 'argv', arguments + extra), \
                 patch.object(pipeline.subprocess, 'run') as run, \
                 patch.object(pipeline.subprocess, 'check_output') as output:
                with self.assertRaisesRegex(ValueError, message):
                    pipeline.main()
                run.assert_not_called()
                output.assert_not_called()

    def test_app_base_and_selected_client_mismatch_stop_before_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            client = root / 'client'
            client.mkdir()
            (client / 'environment.json').write_text('{"profile":"not-a-built-reference"}')
            metadata = {'schema_version': 1, 'profile': pipeline.FULL_APP,
                        'base_sdk_environment_sha256': '0' * 64}
            (root / 'environment.json').write_text(json.dumps(metadata))
            with self.assertRaisesRegex(ValueError, 'base SDK identity'):
                pipeline.validate_app_reference(root, {}, root, root)
            metadata['base_sdk_environment_sha256'] = pipeline.digest(pipeline.ROOT / 'planning/reference/headless-sdk-environment.json')
            metadata['client_reference'] = {'root': str(client), 'environment_sha256': '0' * 64}
            (root / 'environment.json').write_text(json.dumps(metadata))
            with self.assertRaisesRegex(ValueError, 'selected-client identity'):
                pipeline.validate_app_reference(root, {}, root, root)

    def test_app_files_and_directories_reject_missing_changed_or_escaping_inputs(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / 'app'
            root.mkdir()
            value = root / 'input'
            value.write_bytes(b'controlled nonexecuted input')
            expected = pipeline.digest(value)
            pipeline.app_files(root, {'input': expected})
            for files in [{}, {'input': '0' * 64}, {'missing': expected}, {'../outside': expected}]:
                with self.subTest(files=files), self.assertRaises(ValueError):
                    pipeline.app_files(root, files)
            outside = root.parent / 'outside'
            outside.write_bytes(value.read_bytes())
            value.unlink()
            value.symlink_to(outside)
            with self.assertRaises(ValueError):
                pipeline.app_files(root, {'input': expected})
            outside_dir = root.parent / 'other'
            outside_dir.mkdir()
            (root / 'linked').symlink_to(outside_dir)
            for name in ['../other', 'linked', 'absent']:
                with self.subTest(name=name), self.assertRaises(ValueError):
                    pipeline.app_directory(root, name)

    def transport_fixture(self, root):
        app, client = root / 'app', root / 'client prefix'
        app.mkdir()
        headers = {}
        for name in pipeline.TRANSPORT_HEADERS:
            path = client / 'include/gz/transport16/gz/transport' / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('#define HAVE_ZENOH 1\n' if name == 'config.hh' else '// controlled header\n')
            headers[str(path.relative_to(client))] = pipeline.digest(path)
        records = {}
        for name, repository in pipeline.TRANSPORT_CONSUMERS.items():
            source = app / 'sources' / name.replace(':', '/', 1)
            source.parent.mkdir(parents=True, exist_ok=True)
            source.write_text('// controlled source, not compiled\n')
            build = app / pipeline.APP_BUILD_DIRECTORIES[repository]
            build.mkdir(parents=True, exist_ok=True)
            obj = build / (source.stem + '.o')
            depfile = obj.with_suffix('.o.d')
            flags = obj.with_suffix('.flags')
            obj.write_bytes(b'nonexecuted object identity fixture')
            flags.write_text('controlled flags identity\n')
            escape = lambda path: str(path).replace(' ', '\\ ')
            deps = [source, *(client / header for header in headers)]
            depfile.write_text(escape(obj) + ': ' + ' \\\n '.join(escape(path) for path in deps) + '\n')
            records[name] = {key: str(path.relative_to(app)) for key, path in
                             [('object', obj), ('depfile', depfile), ('flags', flags)]}
            records[name]['files'] = {str(path.relative_to(app)): pipeline.digest(path)
                                      for path in [obj, depfile, flags]}
        return app, client, headers, records

    def test_off_headers_stale_objects_and_wrong_compiler_dependencies_reject(self):
        for change in ['off', 'missing-header', 'wrong-source', 'wrong-target', 'object', 'flags', 'missing-consumer']:
            with self.subTest(change=change), tempfile.TemporaryDirectory() as directory:
                app, client, headers, records = self.transport_fixture(Path(directory))
                # Baseline tests only this compiler-input checker, not an app manifest.
                pipeline.validate_transport_compilation(app, client, records, headers)
                name = next(iter(records))
                record = records[name]
                if change == 'missing-consumer':
                    records.pop(name)
                elif change == 'off':
                    header = next(key for key in headers if key.endswith('/config.hh'))
                    (client / header).write_text('/* #undef HAVE_ZENOH */\n')
                    headers[header] = pipeline.digest(client / header)
                elif change in ['object', 'flags']:
                    with (app / record[change]).open('ab') as output:
                        output.write(b'changed')
                else:
                    depfile = app / record['depfile']
                    text = depfile.read_text()
                    if change == 'missing-header':
                        header = next(iter(headers))
                        text = text.replace(str(client / header).replace(' ', '\\ '), '/foreign/header.hh')
                    elif change == 'wrong-source':
                        text = text.replace(str(app / 'sources' / name.replace(':', '/', 1)), '/foreign/source.cc')
                    else:
                        text = text.replace(str(app / record['object']), str(app / 'wrong.o'), 1)
                    depfile.write_text(text)
                    record['files'][record['depfile']] = pipeline.digest(depfile)
                with self.assertRaises(ValueError):
                    pipeline.validate_transport_compilation(app, client, records, headers)

    def test_old_lower_layer_compiler_objects_cannot_qualify_new_overlay_builds(self):
        with tempfile.TemporaryDirectory() as directory:
            app, client, headers, records = self.transport_fixture(Path(directory))
            name = 'gz-sensors:src/Sensor.cc'
            record = records[name]
            old = app / 'builds/gz-sensors'
            old.mkdir()
            changed = {}
            for field in ['object', 'depfile', 'flags']:
                source = app / record[field]
                target = old / source.name
                target.write_bytes(source.read_bytes())
                changed[field] = str(target.relative_to(app))
            changed['files'] = {changed[field]: pipeline.digest(app / changed[field])
                                for field in ['object', 'depfile', 'flags']}
            records[name] = changed
            with self.assertRaisesRegex(ValueError, 'outside its build'):
                pipeline.validate_transport_compilation(app, client, records, headers)
            self.assertEqual(pipeline.APP_BUILD_DIRECTORIES['gz-physics'], 'builds/gz-physics')

    def test_app_compiler_settings_cannot_change_work_or_optimization(self):
        flags = {'CMAKE_BUILD_TYPE': 'Release', 'BUILD_TESTING': 'ON'}
        cache = flags | {'CMAKE_C_FLAGS_RELEASE': '-O3 -DNDEBUG', 'CMAKE_CXX_FLAGS_RELEASE': '-O3 -DNDEBUG'}
        pipeline.validate_app_cache(cache, flags, flags)
        for key, value in [('CMAKE_BUILD_TYPE', 'Debug'), ('BUILD_TESTING', 'OFF'),
                           ('CMAKE_CXX_FLAGS_RELEASE', '-Ofast'), ('CMAKE_C_FLAGS', '-march=native'),
                           ('CMAKE_CXX_FLAGS', '-ffast-math'), ('CMAKE_SHARED_LINKER_FLAGS', '-foreign')]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                pipeline.validate_app_cache(cache | {key: value}, flags, flags)
        with self.assertRaises(ValueError):
            pipeline.validate_app_cache(cache, flags | {'ENABLE_GUI': 'ON'}, flags)

    def test_runtime_selection_is_child_only_and_excludes_old_transport_injection(self):
        original = {'GZ_TRANSPORT_IMPLEMENTATION': 'zeromq', 'GZ_RELAY': 'foreign',
                    'GZ_TRANSPORT_ZENOH_CONFIG_OVERRIDE': 'foreign', 'ZENOH_CONFIG': 'foreign',
                    'GZ_DISCOVERY_MSG_PORT': '10317', 'GZ_TRANSPORT_RCVHWM': '1',
                    'LD_PRELOAD': 'foreign', 'LD_AUDIT': 'foreign', 'PATH': '/controlled'}
        saved = dict(original)
        env = pipeline.app_runtime_environment(original, [Path('/selected/lib'), Path('/vendor/lib')], Path('/case/zenoh.json'))
        self.assertEqual(original, saved)
        self.assertEqual(env, {'PATH': '/controlled', 'LD_LIBRARY_PATH': '/selected/lib:/vendor/lib',
                              'GZ_TRANSPORT_IMPLEMENTATION': 'zenoh', 'ZENOH_CONFIG': '/case/zenoh.json'})

    def test_unresolved_foreign_and_unqualified_loaded_libraries_reject(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            selected = root / 'libzenohc.so'
            selected.write_bytes(b'nonexecuted library fixture')
            expected = {str(selected): pipeline.digest(selected)}
            for text in ['libzenohc.so => not found\n', '']:
                with self.assertRaises(ValueError):
                    pipeline.validate_app_linkage(text, expected, {'libzenohc.so': root})
            with self.assertRaises(ValueError):
                pipeline.validate_app_loaded_libraries({str(selected): '0' * 64}, expected)
            extra = root / 'libc.so.6'
            extra.write_bytes(b'unknown system closure fixture')
            with self.assertRaises(ValueError):
                pipeline.validate_app_loaded_libraries({str(extra): pipeline.digest(extra)}, expected, all_libraries=True)

    def test_missing_or_benchmark_smoke_is_not_app_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            app = Path(directory)
            for smoke in [{'files': {}, 'status': 0}, {'files': dict.fromkeys(['source', 'binary', 'config', 'stdout', 'stderr', 'maps']), 'status': 1}]:
                with self.assertRaisesRegex(ValueError, 'incomplete'):
                    pipeline.validate_app_smoke(app, smoke, {}, {})
            paths = {}
            fake_root = app / 'source-root'
            (fake_root / 'scripts').mkdir(parents=True)
            (fake_root / 'scripts/zenoh_headless_probe.cc').write_text('// controlled probe identity\n')
            for name in ['source', 'binary', 'config', 'stdout', 'stderr', 'maps']:
                path = app / name
                path.write_bytes(b'')
                paths[name] = path
            paths['source'].write_bytes((fake_root / 'scripts/zenoh_headless_probe.cc').read_bytes())
            paths['config'].write_text(json.dumps({'mode': 'peer', 'listen': {'endpoints': ['tcp/127.0.0.1:0']},
                                                 'connect': {'endpoints': []}, 'scouting': {'multicast': {'enabled': False}, 'gossip': {'enabled': False}},
                                                 'transport': {'shared_memory': {'enabled': False}}}))
            paths['stdout'].write_bytes(b'PIPELINE {"nanos":100}\n')
            smoke = {'status': 0, 'files': {name: {'path': path.name, 'sha256': pipeline.digest(path)} for name, path in paths.items()}}
            smoke.update(command=[str(paths['binary']), str(fake_root / 'crates/gz-sim-scene/tests/fixtures/headless_imu.sdf'), str(paths['maps'])], timeout_seconds=15)
            with patch.object(pipeline, 'ROOT', fake_root), self.assertRaisesRegex(ValueError, 'committed streams'):
                pipeline.validate_app_smoke(app, smoke, {}, {})


class AppInputCorrections(unittest.TestCase):
    def test_declared_versioned_derive_and_app_builder_inputs_cannot_be_omitted_or_changed(self):
        with tempfile.TemporaryDirectory() as directory:
            root, record_path, build_json, binary, record = PipelineEvidence().build_fixture(directory)
            for name in pipeline.APP_BUILD_INPUTS:
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text('controlled producer input\n')
            (root / 'Cargo.toml').write_text('[patch.crates-io]\nprost-derive = { path = "third_party/prost-derive-0.14.4" }\n')
            repair = root / 'third_party/prost-derive-0.14.4/src/field/scalar.rs'
            repair.parent.mkdir(parents=True)
            repair.write_text('// controlled signed-zero repair identity, not executed\n')
            schema = root / 'crates/gz-sim-proto/schemas/imu.proto'
            schema.parent.mkdir(parents=True)
            schema.write_text('syntax = "proto2";\n')
            (root / '.cargo').mkdir()
            (root / '.cargo/config.toml').write_text('[env]\nTCP_LINGER_TIMEOUT="0"\n')
            record['source_sha256'] = {str(path.relative_to(root)): pipeline.digest(path)
                                      for path in root.rglob('*') if path.is_file()}
            record_path.write_text(json.dumps(record))
            with patch.object(pipeline, 'ROOT', root):
                pipeline.validate_rust_build(record_path, build_json, binary, pipeline.FULL_APP)
                for path in [repair, root / 'scripts/bootstrap-zenoh-headless.py',
                             root / 'planning/reference/headless-sdk-build-libraries.json',
                             root / 'scripts/native_reference.py']:
                    name = str(path.relative_to(root))
                    missing = copy.deepcopy(record)
                    missing['source_sha256'].pop(name)
                    record_path.write_text(json.dumps(missing))
                    with self.subTest(missing=name), self.assertRaisesRegex(ValueError, 'omits required'):
                        pipeline.validate_rust_build(record_path, build_json, binary, pipeline.FULL_APP)
                    record_path.write_text(json.dumps(record))
                    original = path.read_bytes()
                    path.write_bytes(original + b'changed')
                    with self.subTest(changed=name), self.assertRaisesRegex(ValueError, 'source changed'):
                        pipeline.validate_rust_build(record_path, build_json, binary, pipeline.FULL_APP)
                    path.write_bytes(original)
                repair.parent.parent.parent.rename(root / 'third_party/wrong-unversioned-directory')
                with self.assertRaisesRegex(ValueError, 'real directory'):
                    pipeline.validate_rust_build(record_path, build_json, binary, pipeline.FULL_APP)

    def test_candidate_metadata_uses_same_profile_guards_without_publishing_a_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            candidate = {'schema_version': 1, 'profile': 'gazebo-zenoh-client-v1'}
            with self.assertRaisesRegex(ValueError, 'full-app profile'):
                pipeline.validate_app_reference(root, {}, root, root, metadata=candidate)
            candidate = {'schema_version': 1, 'profile': pipeline.FULL_APP,
                         'base_sdk_environment_sha256': '0' * 64}
            with self.assertRaisesRegex(ValueError, 'base SDK identity'):
                pipeline.validate_app_reference(root, {}, root, root, metadata=candidate)
            self.assertFalse((root / 'environment.json').exists())


class AppStaticAndRuntimeLibraries(unittest.TestCase):
    def test_static_core_linkage_still_requires_full_plugins_in_runtime_maps(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            required = {name: root for name in pipeline.APP_STATIC_ROLES}
            required['libgz-sim-imu-system.so.'] = root
            libraries = {}
            linkage = []
            for name in required:
                filename = name + ('1' if name.endswith('.') else '')
                path = root / filename
                path.write_bytes(b'controlled nonexecuted library identity')
                libraries[str(path)] = pipeline.digest(path)
                if name in pipeline.APP_STATIC_ROLES:
                    linkage.append(f'{filename} => {path} (0x0)')
            loaded = pipeline.validate_app_linkage('\n'.join(linkage), libraries,
                                                   {name: required[name] for name in pipeline.APP_STATIC_ROLES})
            with self.assertRaisesRegex(ValueError, 'missing or duplicate'):
                pipeline.require_native_libraries(loaded, required, libraries)
            pipeline.require_native_libraries(libraries, required, libraries)


class SeparateSourceContexts(unittest.TestCase):
    def full_rust_fixture(self, directory):
        root, record_path, build_json, binary, record = PipelineEvidence().build_fixture(directory)
        for name in pipeline.APP_BUILD_INPUTS:
            path = root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('controlled producer input\n')
        (root / 'Cargo.toml').write_text('[patch.crates-io]\nprost-derive = { path = "third_party/prost-derive-0.14.4" }\n')
        for name in ['third_party/prost-derive-0.14.4/src/field/scalar.rs',
                     'crates/gz-sim-proto/schemas/imu.proto', '.cargo/config.toml']:
            path = root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('controlled build input\n')
        record['source_sha256'] = {str(p.relative_to(root)): pipeline.digest(p)
                                  for p in root.rglob('*') if p.is_file()}
        record_path.write_text(json.dumps(record))
        return root, record_path, build_json, binary, record

    def test_context_defaults_profile_and_directory_refusals(self):
        self.assertEqual(pipeline.benchmark_source_roots(pipeline.FULL_APP),
                         (pipeline.ROOT, pipeline.ROOT))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            rust, app = root / 'rust', root / 'native'
            rust.mkdir()
            app.mkdir()
            self.assertEqual(pipeline.benchmark_source_roots(pipeline.FULL_APP, rust, app),
                             (rust, app))
            for profile in [pipeline.client_reference.SDK, pipeline.client_reference.LEGACY]:
                with self.subTest(profile=profile), self.assertRaisesRegex(ValueError, 'full-app'):
                    pipeline.benchmark_source_roots(profile, rust, app)
            with self.assertRaisesRegex(ValueError, 'real directory'):
                pipeline.benchmark_source_roots(pipeline.FULL_APP, root / 'missing', app)
            alias = root / 'symlink'
            alias.symlink_to(app, target_is_directory=True)
            with self.assertRaisesRegex(ValueError, 'real directory'):
                pipeline.benchmark_source_roots(pipeline.FULL_APP, rust, alias)

    def test_rust_record_uses_actual_export_without_relabeling_tool_context(self):
        with tempfile.TemporaryDirectory() as directory:
            root, record_path, build_json, binary, record = self.full_rust_fixture(directory)
            other = Path(directory) / 'tool'
            shutil.copytree(root, other)
            changed = other / 'crates/example/src/lib.rs'
            changed.write_text('different source export\n')
            with patch.object(pipeline, 'ROOT', other):
                self.assertEqual(pipeline.validate_rust_build(
                    record_path, build_json, binary, pipeline.FULL_APP, source_root=root), record)
                with self.assertRaisesRegex(ValueError, 'source changed'):
                    pipeline.validate_rust_build(record_path, build_json, binary, pipeline.FULL_APP)
            original = root / 'crates/example/src/lib.rs'
            original.write_text('actual Rust input changed\n')
            with self.assertRaisesRegex(ValueError, 'source changed'):
                pipeline.validate_rust_build(record_path, build_json, binary,
                                             pipeline.FULL_APP, source_root=root)

    def test_native_smoke_keeps_exact_historical_argv_and_source_checks(self):
        # Controlled files exercise validation only, never native qualification.
        with tempfile.TemporaryDirectory() as directory:
            parent = Path(directory)
            native, rust, app = parent / 'native-source', parent / 'rust-source', parent / 'app'
            for root in [native, rust]:
                (root / 'scripts').mkdir(parents=True)
                (root / 'scripts/zenoh_headless_probe.cc').write_text('// same probe\n')
            app.mkdir()
            paths = {}
            for name in ['source', 'binary', 'config', 'stdout', 'stderr', 'maps']:
                paths[name] = app / name
                paths[name].write_bytes(b'')
            paths['source'].write_bytes((native / 'scripts/zenoh_headless_probe.cc').read_bytes())
            paths['config'].write_text(json.dumps({'mode': 'peer', 'listen': {'endpoints': ['tcp/127.0.0.1:0']},
                'connect': {'endpoints': []}, 'scouting': {'multicast': {'enabled': False}, 'gossip': {'enabled': False}},
                'transport': {'shared_memory': {'enabled': False}}}))
            paths['stdout'].write_bytes(b'zenoh headless ready clock=1 pose=1 imu=1 sim_ns=1000000\n')
            smoke = {'status': 0, 'files': {name: {'path': path.name, 'sha256': pipeline.digest(path)}
                                          for name, path in paths.items()},
                     'command': [str(paths['binary']), str(native / 'crates/gz-sim-scene/tests/fixtures/headless_imu.sdf'), str(paths['maps'])],
                     'timeout_seconds': 15, 'libraries_sha256': {}}
            with patch.object(pipeline, 'ROOT', rust):
                self.assertEqual(pipeline.validate_app_smoke(app, smoke, {}, {}, source_root=native), {})
                with self.assertRaisesRegex(ValueError, 'input-bound'):
                    pipeline.validate_app_smoke(app, smoke, {}, {})
            changed = copy.deepcopy(smoke)
            changed['timeout_seconds'] = 16
            with self.assertRaisesRegex(ValueError, 'bounded'):
                pipeline.validate_app_smoke(app, changed, {}, {}, source_root=native)
            (native / 'scripts/zenoh_headless_probe.cc').write_text('// mutated source\n')
            with self.assertRaisesRegex(ValueError, 'source differs'):
                pipeline.validate_app_smoke(app, smoke, {}, {}, source_root=native)

    def test_context_fixture_and_all_native_driver_bytes_must_agree(self):
        with tempfile.TemporaryDirectory() as directory:
            parent = Path(directory)
            rust, native = parent / 'rust', parent / 'native'
            names = ['crates/gz-sim-scene/tests/fixtures/headless_imu.sdf',
                     *('crates/gz-sim-server/tests/' + p for p in ['native_pipeline_benchmark.cc',
                        'native_lockstep_benchmark.cc', 'native_stream_benchmark.cc'])]
            for root in [rust, native]:
                for name in names:
                    path = root / name
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.write_text('identical workload bytes\n')
            pipeline.check_benchmark_context_inputs(rust, native)
            for name in names:
                path = native / name
                path.write_text('different workload bytes\n')
                with self.subTest(name=name), self.assertRaisesRegex(ValueError, 'contexts differ'):
                    pipeline.check_benchmark_context_inputs(rust, native)
                path.write_text('identical workload bytes\n')

    def test_native_context_inputs_are_bound_before_importing_its_helper(self):
        with tempfile.TemporaryDirectory() as directory:
            parent = Path(directory)
            native, app, client = parent / 'native', parent / 'app', parent / 'client'
            app.mkdir()
            client.mkdir()
            for name in pipeline.APP_BUILD_INPUTS:
                path = native / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text('controlled native input\n')
            base = native / 'planning/reference/headless-sdk-environment.json'
            base.write_text('{}')
            (client / 'environment.json').write_text('{}')
            helper = native / 'scripts/bootstrap-zenoh-native.py'
            helper.write_text('raise RuntimeError("selected native context import boundary")\n')
            metadata = {'schema_version': 1, 'profile': pipeline.FULL_APP,
                'base_sdk_environment_sha256': pipeline.digest(base),
                'client_reference': {'root': str(client), 'environment_sha256': pipeline.digest(client / 'environment.json')},
                'inputs_sha256': {name: pipeline.digest(native / name) for name in pipeline.APP_BUILD_INPUTS}}
            with self.assertRaisesRegex(RuntimeError, 'selected native context import boundary'):
                pipeline.validate_app_reference(app, {}, client, app, metadata=metadata, source_root=native)
            helper.write_text('raise AssertionError("changed helper must never execute")\n')
            with self.assertRaisesRegex(ValueError, 'changed'):
                pipeline.validate_app_reference(app, {}, client, app, metadata=metadata, source_root=native)

    def test_main_keeps_actual_cargo_source_and_tool_drift_guards_before_measurement(self):
        class TimingReached(Exception):
            pass
        # No reference is qualified by this wiring stub; taskset is never run.
        for mutation in ['none', 'wrong-cargo-context', 'changed-tool']:
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as directory:
                root, record_path, build_json, binary, record = self.full_rust_fixture(directory)
                compiler = Path(directory) / 'c++'
                compiler.write_text('unexecuted compiler fixture\n')
                prefix, builds, _ = PipelineEvidence().sdk_fixture(directory, root, compiler)
                native, tool = Path(directory) / 'native-context', Path(directory) / 'tool-context'
                shutil.copytree(root, native)
                shutil.copytree(root, tool)
                (tool / 'scripts/probe-headless-imu.py').write_text('def checked_sources(*args): return {}\n')
                (native / 'planning/baseline.json').write_text(json.dumps({'repositories': [
                    {'name': 'gz-' + name, 'commit': 'c' * 40} for name in ['sim', 'physics', 'sensors']]}))
                # The tool tree is deliberately not a native source inventory.
                (tool / 'planning/baseline.json').unlink()
                for name in ['scripts/native_reference.py', 'reference/build-reference.py']:
                    (tool / name).write_text('controlled current measurement tool\n')
                actual_source = root / 'crates/gz-sim-server/src/lib.rs'
                actual_source.parent.mkdir(parents=True, exist_ok=True)
                actual_source.write_text('// current Rust source\n')
                artifact = {'reason': 'compiler-artifact', 'executable': str(binary),
                    'target': {'name': 'gz_sim_server', 'src_path': str(actual_source if mutation != 'wrong-cargo-context' else native / 'crates/gz-sim-server/src/lib.rs')},
                    'profile': {'test': True, 'opt_level': '3', 'debug_assertions': False}}
                build_json.write_text(json.dumps(artifact) + '\n' + json.dumps({'reason': 'build-finished', 'success': True}) + '\n')
                record.update(build_json_sha256=pipeline.digest(build_json),
                    source_sha256={str(p.relative_to(root)): pipeline.digest(p) for p in root.rglob('*') if p.is_file()})
                record_path.write_text(json.dumps(record))
                app_root = Path(directory) / 'app'
                app_root.mkdir()
                (app_root / 'environment.json').write_text('controlled in-memory validation boundary\n')
                app = {'metadata': {'base_sdk_environment_sha256': 'controlled', 'client_reference': {}},
                       'environment': {}, 'libraries': {}, 'vendor': Path(directory) / 'vendor', 'compiler': str(compiler)}
                def validate_native(*args, source_root=None, **kwargs):
                    self.assertEqual(source_root, native)
                    return app
                def child(command, **kwargs):
                    if command[0] == str(compiler):
                        Path(command[command.index('-o') + 1]).write_bytes(b'\x7fELFnonexecuted native fixture')
                        if mutation == 'changed-tool':
                            path = tool / 'scripts/native_reference.py'
                            path.write_text('changed actual tool\n')
                        return subprocess.CompletedProcess(command, 0)
                    self.assertEqual(command[0], 'taskset')
                    raise TimingReached()
                arguments = ['benchmark', '--reference-profile', pipeline.FULL_APP,
                    '--app-reference', str(app_root), '--app-source-root', str(native), '--rust-source-root', str(root),
                    '--rust-build-json', str(build_json), '--rust-build-info', str(record_path),
                    '--reference-prefix', str(prefix), '--dependency-prefix', str(prefix),
                    '--work-dir', str(Path(directory) / 'work'), '--output', str(Path(directory) / 'result.json'), '--path', 'streams']
                for name, build in builds.items():
                    arguments += ['--' + name + '-build', str(build)]
                with patch.object(pipeline, 'ROOT', tool), patch.object(sys, 'argv', arguments), \
                     patch.object(pipeline, 'validate_app_reference', side_effect=validate_native), \
                     patch.object(pipeline, 'validate_app_linkage', return_value={}), \
                     patch.object(pipeline.subprocess, 'check_output', return_value='controlled compiler\n'), \
                     patch.object(pipeline.subprocess, 'run', side_effect=child):
                    with self.assertRaises(TimingReached if mutation == 'none' else ValueError):
                        pipeline.main()
                self.assertFalse((Path(directory) / 'result.json').exists())
