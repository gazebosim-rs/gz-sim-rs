# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Isolated builder command/failure controls, never a built app environment."""
import importlib.util
import json
from pathlib import Path
import signal
import subprocess
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('headless_build', ROOT / 'scripts/bootstrap-zenoh-headless.py')
build = importlib.util.module_from_spec(spec)
spec.loader.exec_module(build)


class HeadlessBuildTests(unittest.TestCase):
    def test_container_plan_uses_immutable_image_network_none_and_readonly_inputs(self):
        with tempfile.TemporaryDirectory() as directory:
            client = Path(directory)
            metadata = {'lock': {'vendor': {'url': 'https://controlled.invalid/vendor.deb'}}}
            with patch.object(build.selected, 'validate_reference', return_value=metadata) as validate:
                command = build.container_command(client, 2, 'bounded-reference')
            validate.assert_called_once_with(client.resolve())
            self.assertEqual(command[:4], ['docker', 'run', '--pull=never', '--network=none'])
            self.assertIn(build.base_image(), command)
            self.assertNotIn('--rm', command)  # Retain container/source/probe evidence.
            self.assertNotIn('--privileged', command)
            mounts = [command[index + 1] for index, item in enumerate(command[:-1]) if item == '-v']
            self.assertEqual(len(mounts), 3)
            self.assertTrue(all(value.endswith(':ro') for value in mounts))
            self.assertIn(f'{client}/sources:/client-sources:ro', mounts)
            self.assertEqual(command[command.index('--inside-sdk'):], ['--inside-sdk', '--root', '/reference', '--client-reference', '/zenoh-client',
                                            '--client-sources', '/client-sources', '--vendor-archive', '/zenoh-vendor.deb',
                                            '--jobs', '2'])

    def test_container_plan_rejects_tags_changed_base_and_ambiguous_arguments(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            base = root / 'base.json'
            base.write_bytes(b'controlled base identity')
            registry = root / 'registry.json'
            with patch.object(build, 'BASE_PATH', base), patch.object(build, 'REGISTRY_PATH', registry):
                for image, checksum in [('mutable:latest', build.selected.digest(base)), ('sha256:' + 'a' * 64, '0' * 64)]:
                    registry.write_text(json.dumps({'image_id': image, 'sdk_environment_sha256': checksum}))
                    with self.subTest(image=image), self.assertRaises(ValueError):
                        build.base_image()
            for jobs, name in [(0, 'ok'), (5, 'ok'), (2, '--foreign')]:
                with self.subTest(jobs=jobs, name=name), patch.object(build.selected, 'validate_reference') as validate:
                    with self.assertRaises(ValueError):
                        build.container_command(root, jobs, name)
                    validate.assert_not_called()

    def test_wrong_root_packages_or_tools_refuse_before_build_or_source_mutation(self):
        with patch.object(build.selected.builder, 'run') as run:
            with self.assertRaisesRegex(ValueError, 'disposable SDK'):
                build.preflight_base(Path('/tmp/not-an-sdk'))
            run.assert_not_called()
        base = json.loads(build.BASE_PATH.read_text())
        for packages, tools in [('wrong packages', base['tools_sha256']), (base['packages'], {})]:
            with self.subTest(packages=packages == base['packages']), \
                 patch.object(build.selected, 'digest', return_value='controlled-identity'), \
                 patch.object(build.selected, 'build_environment', return_value={}), \
                 patch.object(build, 'packages', return_value=packages), \
                 patch.object(build, 'tools', return_value=tools), \
                 patch.object(Path, 'exists', return_value=False), \
                 patch.object(Path, 'is_symlink', return_value=False), \
                 patch.object(build.pipeline.client_reference, 'validate_reference') as validate:
                with self.assertRaisesRegex(ValueError, 'package/tool identity'):
                    build.preflight_base(Path('/reference'))
                validate.assert_not_called()

    def test_fresh_overlay_builds_leave_all_old_build_bytes_in_place(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            client = root / 'client'
            for package in ['gz-utils', 'gz-math', 'gz-msgs', 'gz-transport']:
                (client / 'install/lib/cmake' / package).mkdir(parents=True)
            old = root / 'builds/gz-sensors'
            old.mkdir(parents=True)
            (old / 'retained.o').write_bytes(b'nonexecuted old object fixture')
            fresh = build.fresh_build_directory(root, 'gz-sensors')
            self.assertEqual(fresh, root / 'builds/zenoh-gz-sensors')
            self.assertEqual(list(fresh.iterdir()), [])
            self.assertEqual((old / 'retained.o').read_bytes(), b'nonexecuted old object fixture')
            with self.assertRaisesRegex(ValueError, 'must be fresh'):
                build.fresh_build_directory(root, 'gz-sensors')
            with self.assertRaisesRegex(ValueError, 'only affected'):
                build.fresh_build_directory(root, 'gz-physics')
            flags = {'CMAKE_BUILD_TYPE': 'Release', 'BUILD_TESTING': 'ON'}
            command = build.configure_command(root, client, 'gz-sensors', flags)
            self.assertIn('-Dgz-transport_DIR=' + str(client / 'install/lib/cmake/gz-transport'), command)
            self.assertIn('-DCMAKE_BUILD_TYPE=Release', command)
            self.assertIn('-DBUILD_TESTING=ON', command)
            for name in build.selected.LAUNCHERS:
                self.assertIn('-D' + name + '=', command)
            (client / 'install/lib/cmake/gz-msgs').rmdir()
            with self.assertRaisesRegex(ValueError, 'package is missing'):
                build.configure_command(root, client, 'gz-sensors', flags)

    def test_smoke_compile_keeps_exact_fixture_and_existing_application_link_inputs(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / 'builds/zenoh-gz-sim/src/CMakeFiles/UNIT_ServerConfig_TEST.dir'
            target.mkdir(parents=True)
            (target / 'flags.make').write_text('CXX_INCLUDES = -I/selected/include\n')
            (target / 'link.txt').write_text('/usr/bin/g++ -o old-test ../lib/libgz-sim.so -lgtest -lprotobuf\n')
            command = build.smoke_command(root, '/controlled/g++', root / 'source.cc', root / 'probe')
            self.assertEqual(command[:7], ['/controlled/g++', '-std=c++17', '-O3', '-DNDEBUG', '-Wall', '-Wextra', '-Werror'])
            self.assertIn('-I/selected/include', command)
            self.assertIn(str((root / 'builds/zenoh-gz-sim/lib/libgz-sim.so').resolve()), command)
            self.assertNotIn('-lgtest', command)
            self.assertIn('-lprotobuf', command)

    def test_smoke_timeout_kills_only_its_child_group_and_reaps_with_raw_output(self):
        class Child:
            pid = 424242
            returncode = -9
            calls = 0
            def communicate(self, **kwargs):
                self.calls += 1
                if self.calls == 1:
                    self.timeout = kwargs['timeout']
                    raise subprocess.TimeoutExpired(['nonexecuted-probe'], self.timeout)
                return b'partial output', b'partial failure'
        with tempfile.TemporaryDirectory() as directory:
            child = Child()
            with patch.object(build.subprocess, 'Popen', return_value=child) as popen, patch.object(build.os, 'killpg') as kill:
                with self.assertRaises(subprocess.TimeoutExpired):
                    build.run_smoke(['nonexecuted-probe'], {}, Path(directory))
                self.assertTrue(popen.call_args.kwargs['start_new_session'])
                kill.assert_called_once_with(child.pid, signal.SIGKILL)
            self.assertEqual(child.timeout, 15)
            self.assertEqual(child.calls, 2)
            self.assertEqual((Path(directory) / 'stdout').read_bytes(), b'partial output')
            self.assertEqual((Path(directory) / 'stderr').read_bytes(), b'partial failure')

    def test_separate_client_uses_unchanged_offline_bootstrap_and_failure_stops_app_build(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'environment.json').write_bytes(b'original base fixture')
            client = root / 'new-client'
            with patch.object(build, 'preflight_base', return_value={}), \
                 patch.object(build.selected, 'build_environment', return_value={}), \
                 patch.object(build, 'owned_source_cache', return_value=root / 'new-client-source-cache') as copy, \
                 patch.object(build.selected.builder, 'run', side_effect=RuntimeError('controlled SDK build failure')) as run:
                with self.assertRaisesRegex(RuntimeError, 'SDK build failure'):
                    build.build(root, client, root / 'offline-sources', root / 'vendor.deb', 2)
            command = run.call_args.args[0]
            self.assertEqual(list(map(str, command)), ['python3', str(ROOT / 'scripts/bootstrap-zenoh-native.py'),
                '--root', str(client), '--local-sources', str(root / 'new-client-source-cache'),
                '--vendor-archive', str(root / 'vendor.deb'), '--jobs', '2'])
            self.assertEqual(run.call_count, 1)
            copy.assert_called_once_with(root / 'offline-sources', root / 'new-client-source-cache', {})
            self.assertEqual((root / 'environment.json').read_bytes(), b'original base fixture')
            self.assertFalse((root / 'base-builds').exists())
            self.assertFalse((root / 'environment.pending.json').exists())

    def test_builder_rebuilds_sensors_then_sim_and_retains_physics_without_publication(self):
        class CommandsCaptured(Exception):
            pass
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            client = root / 'client'
            (client / 'install').mkdir(parents=True)
            (client / 'environment.json').write_text('controlled client marker, never validated as SDK')
            for package in ['gz-utils', 'gz-math', 'gz-msgs', 'gz-transport']:
                (client / 'install/lib/cmake' / package).mkdir(parents=True)
            for name in build.APP_NAMES:
                old = root / 'builds' / name
                old.mkdir(parents=True)
                (old / 'retained').write_bytes(name.encode())
            (root / 'environment.json').write_bytes(b'original base fixture')
            base = {'tools_sha256': {'c++': 'controlled'}, 'repositories': [
                {'name': name, 'sha': 'a' * 40, 'flags': {'CMAKE_BUILD_TYPE': 'Release'}}
                for name in build.APP_NAMES]}
            commands = []
            with patch.object(build, 'preflight_base', return_value=base), \
                 patch.object(build.selected, 'validate_reference', return_value={'tools': {'c++': {'sha256': 'controlled'}}}), \
                 patch.object(build, 'environment', return_value={}), \
                 patch.object(build, 'tools', return_value=base['tools_sha256']), \
                 patch.object(build.selected.builder, 'verify_source'), \
                 patch.object(build.selected.builder, 'run', side_effect=lambda command, *args: commands.append(list(map(str, command)))), \
                 patch.object(build.pipeline, 'validate_app_cache'), \
                 patch.object(build.selected, 'cache_values', return_value={}), \
                 patch.object(build.selected, 'digest', return_value='controlled'), \
                 patch.object(build.selected, 'inventory', return_value={}), \
                 patch.object(build, 'compiler_records', side_effect=CommandsCaptured):
                with self.assertRaises(CommandsCaptured):
                    build.build(root, client, root / 'offline-sources', root / 'archive', 2)
            self.assertEqual(len(commands), 6)
            for offset, name in [(0, 'gz-sensors'), (3, 'gz-sim')]:
                self.assertIn(str(root / 'sources' / name), commands[offset])
                self.assertEqual(commands[offset + 1], ['/usr/bin/cmake', '--build', str(root / build.pipeline.APP_BUILD_DIRECTORIES[name]), '--parallel', '2'])
                self.assertEqual(commands[offset + 2], ['/usr/bin/cmake', '--install', str(root / build.pipeline.APP_BUILD_DIRECTORIES[name])])
                self.assertEqual((root / 'builds' / name / 'retained').read_bytes(), name.encode())
            self.assertEqual((root / 'builds/gz-physics/retained').read_bytes(), b'gz-physics')
            self.assertEqual((root / 'environment.json').read_bytes(), b'original base fixture')
            self.assertFalse((root / 'zenoh-headless-smoke').exists())

    def test_owned_cache_copies_canonical_sources_without_host_changes_or_git_bypass(self):
        lock = json.loads((ROOT / 'planning/zenoh-native-lock.json').read_text())
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            sources = root / 'readonly-input'
            destination = root / 'container-owned-cache'
            for row in lock['repositories']:
                source = sources / row['name']
                source.mkdir(parents=True)
                (source / 'tracked').write_bytes(row['sha'].encode())
                (source / 'link').symlink_to('tracked')
            with patch.object(build.selected.builder, 'verify_source') as verify:
                self.assertEqual(build.owned_source_cache(sources, destination, {}), destination)
            self.assertEqual(verify.call_count, len(lock['repositories']))
            for row in lock['repositories']:
                original, copied = sources / row['name'], destination / row['name']
                self.assertEqual((original / 'tracked').read_bytes(), (copied / 'tracked').read_bytes())
                self.assertNotEqual((original / 'tracked').stat().st_ino, (copied / 'tracked').stat().st_ino)
                self.assertTrue((original / 'link').is_symlink())
                self.assertTrue((copied / 'link').is_symlink())
                verify.assert_any_call(copied, row, {})
            with self.assertRaisesRegex(ValueError, 'must be fresh'):
                build.owned_source_cache(sources, destination, {})
            broken = root / 'bad-input'
            broken.mkdir()
            (broken / lock['repositories'][0]['name']).symlink_to(sources / lock['repositories'][0]['name'])
            with self.assertRaisesRegex(ValueError, 'real directory'):
                build.owned_source_cache(broken, root / 'bad-cache', {})

    def test_environment_has_exact_library_suffixes_and_no_inherited_search_roots(self):
        root, client = Path('/application'), Path('/selected-client')
        vendor = client / 'vendor/opt/selected-vendor'
        metadata = {'lock': {'vendor': {'prefix': 'opt/selected-vendor'}}}
        inherited = {'PATH': '/controlled-tools', 'LD_LIBRARY_PATH': '/unselected/library',
                     'CMAKE_PREFIX_PATH': '/unselected/cmake', 'PKG_CONFIG_PATH': '/unselected/pkg'}
        with patch.object(build.selected, 'build_environment', return_value=dict(inherited)) as selected:
            env = build.environment(root, client, metadata)
        selected.assert_called_once_with(client / 'install', vendor)
        self.assertEqual(env['CMAKE_PREFIX_PATH'], '/selected-client/install:/selected-client/vendor/opt/selected-vendor:/application/install')
        self.assertEqual(env['PKG_CONFIG_PATH'], '/selected-client/install/lib/pkgconfig:/selected-client/vendor/opt/selected-vendor/lib/pkgconfig:/application/install/lib/pkgconfig')
        self.assertEqual(env['LD_LIBRARY_PATH'], '/application/builds/gz-physics/lib:/application/builds/zenoh-gz-sensors/lib:/application/builds/zenoh-gz-sim/lib:/selected-client/install/lib:/application/install/lib:/selected-client/vendor/opt/selected-vendor/lib')
        self.assertNotIn('/unselected', ':'.join(env.values()))

    def test_consumed_registry_or_imported_validator_drift_blocks_publication(self):
        names = ['planning/reference/headless-sdk-build-libraries.json', 'scripts/native_reference.py']
        self.assertTrue(set(names).issubset(build.pipeline.APP_BUILD_INPUTS))
        for changed in names:
            with self.subTest(changed=changed), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                source = root / 'source'
                for name in names:
                    path = source / name
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.write_bytes((ROOT / name).read_bytes())
                initial = {name: build.selected.digest(source / name) for name in names}
                app = root / 'app'
                app.mkdir()
                (app / 'environment.json').write_bytes(b'preserved original base manifest')
                path = source / changed
                path.write_bytes(path.read_bytes() + b'changed after capture')
                with patch.object(build, 'ROOT', source), \
                     patch.object(build.pipeline, 'validate_app_reference') as validate:
                    with self.assertRaisesRegex(ValueError, 'app file changed'):
                        build.publish(app, {}, root / 'client', initial, {'rejected': True})
                validate.assert_not_called()
                self.assertEqual((app / 'environment.json').read_bytes(), b'preserved original base manifest')
                self.assertFalse((app / 'environment.pending.json').exists())

    def test_failed_candidate_validation_never_overwrites_original_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            original = root / 'environment.json'
            original.write_bytes(b'preserved original base manifest')
            with patch.object(build.pipeline, 'app_files'), \
                 patch.object(build.pipeline, 'validate_app_reference', side_effect=ValueError('controlled rejected candidate')):
                with self.assertRaisesRegex(ValueError, 'rejected candidate'):
                    build.publish(root, {}, root / 'client', {}, {'rejected': True})
            self.assertEqual(original.read_bytes(), b'preserved original base manifest')
            self.assertFalse((root / 'environment.pending.json').exists())
            with patch.object(build.pipeline, 'app_files'), \
                 patch.object(build.pipeline, 'validate_app_reference', side_effect=[None, ValueError('controlled late drift')]):
                with self.assertRaisesRegex(ValueError, 'late drift'):
                    build.publish(root, {}, root / 'client', {}, {'rejected': True})
            self.assertEqual(original.read_bytes(), b'preserved original base manifest')
            self.assertTrue((root / 'environment.pending.json').exists())
