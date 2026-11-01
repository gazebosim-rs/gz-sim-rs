# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Recipe/refusal tests. None builds an SDK or executes a native application."""
import copy
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('zenoh_dart', ROOT / 'scripts/bootstrap-zenoh-dart.py')
build = importlib.util.module_from_spec(spec)
spec.loader.exec_module(build)


class ZenohDartBuildTests(unittest.TestCase):
    def test_actual_lock_pins_separate_image_signed_dart_and_unchanged_fixture(self):
        lock = build.lock_data()
        self.assertNotEqual(lock['base_image_id'], build.common.base_image())
        self.assertEqual(lock['builds'], ['gz-physics', 'gz-sensors', 'gz-sim'])
        self.assertEqual(lock['fixture']['sha256'], '24e85ff944b5efe2d758fdda543e392f34d59f26297a0a87fafbb41d5a3b16aa')
        dart_lock = json.loads((ROOT / 'reference/headless-dart.lock.json').read_text())
        result = build.dart.verify_metadata(ROOT, dart_lock)
        self.assertEqual(result['packages'], 10)
        self.assertEqual(result['source']['Version'], '6.13.2+ds1-1~osrf2~noble')

    def test_lock_rejects_profile_layout_fixture_pin_and_input_drift(self):
        lock = build.lock_data()
        for key, value in [('profile', build.pipeline.FULL_APP), ('base_image_id', 'mutable:tag'),
                           ('prefix', '/reference/dart-install'), ('client_root', '/zenoh-client'),
                           ('builds', ['gz-sim']), ('smoke_steps', 10), ('smoke_timeout_seconds', 30),
                           ('repositories', {}), ('fixture', lock['fixture'] | {'sha256': '0' * 64}),
                           ('input_sha256', {'planning/baseline.json': '0' * 64})]:
            with self.subTest(key=key), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'lock.json'
                path.write_text(json.dumps(lock | {key: value}))
                with patch.object(build, 'LOCK', path), self.assertRaises(ValueError):
                    build.lock_data()

    def test_container_has_exact_base_offline_readonly_inputs_and_no_prefix_overlay(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            client = {'lock': {'vendor': {'url': 'https://fixture.invalid/vendor.deb'}}}
            with patch.object(build.selected, 'validate_reference', return_value=client):
                cmd = build.container_command(root, 2, 'dart-build')
            self.assertEqual(cmd[:4], ['docker', 'run', '--pull=never', '--network=none'])
            self.assertIn(build.lock_data()['base_image_id'], cmd)
            mounts = [cmd[i + 1] for i, v in enumerate(cmd[:-1]) if v == '-v']
            self.assertEqual(len(mounts), 3)
            self.assertTrue(all(p.endswith(':ro') for p in mounts))
            self.assertNotIn('--privileged', cmd)
            self.assertNotIn('--rm', cmd)
            self.assertFalse(any(':/reference/install' in p for p in mounts))
            self.assertEqual(cmd[-3:], ['--inside-sdk', '--jobs', '2'])

    def test_invalid_plan_arguments_refuse_before_client_validation(self):
        for jobs, name in [(0, 'ok'), (3, 'ok'), (2, '-option'), (2, 'a,b')]:
            with self.subTest(jobs=jobs, name=name), patch.object(build.selected, 'validate_reference') as validate:
                with self.assertRaises(ValueError):
                    build.container_command(Path('/tmp'), jobs, name)
                validate.assert_not_called()

    def test_base_mismatch_stops_before_new_client_or_application_mutation(self):
        with patch.object(build.selected.builder, 'run') as run:
            with self.assertRaisesRegex(ValueError, 'disposable'):
                build.build(root=Path('/tmp/not-the-reference'))
            run.assert_not_called()
        with patch.object(build, 'base_identity', side_effect=ValueError('controlled base changed')), \
             patch.object(build.common, 'owned_source_cache') as copy_sources:
            with self.assertRaisesRegex(ValueError, 'controlled base changed'):
                build.build()
            copy_sources.assert_not_called()

    def test_client_failure_prevents_application_configuration(self):
        lock = build.lock_data()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with patch.object(build, 'base_identity', return_value={}), \
                 patch.object(build, 'BUILD_ROOT', root / 'build'), patch.object(build, 'PREFIX', root / 'install'), \
                 patch.object(build, 'CLIENT', root / 'client'), patch.object(build, 'lock_data', return_value=lock), \
                 patch.object(build.common, 'owned_source_cache', return_value=root / 'sources'), \
                 patch.object(build.selected.builder, 'run', side_effect=RuntimeError('client failed')) as run, \
                 patch.object(build, 'configure_command') as configure:
                with self.assertRaisesRegex(RuntimeError, 'client failed'):
                    build.build()
                self.assertIn('bootstrap-zenoh-native.py', str(run.call_args.args[0][1]))
                configure.assert_not_called()

    def test_build_flags_enable_dart_and_preserve_release_tests(self):
        sdk = json.loads((ROOT / 'planning/reference/headless-sdk-environment.json').read_text())
        old = copy.deepcopy(sdk)
        flags = build.flags_for('gz-physics', sdk)
        self.assertEqual(flags['SKIP_dartsim'], 'OFF')
        self.assertEqual(flags['DART_DIR'], '/usr/share/dart/cmake')
        for key in ['SKIP_bullet', 'SKIP_bullet-featherstone', 'SKIP_mujoco']:
            self.assertEqual(flags[key], 'ON')
        for name in build.NAMES:
            f = build.flags_for(name, sdk)
            self.assertEqual((f['CMAKE_BUILD_TYPE'], f['BUILD_TESTING']), ('Release', 'ON'))
            self.assertEqual(f['CMAKE_INSTALL_PREFIX'], '/reference/zenoh-dart-install')
        self.assertEqual(sdk, old)

    def test_configure_pins_client_and_new_application_packages_without_launchers(self):
        with tempfile.TemporaryDirectory() as directory:
            client = Path(directory)
            for name in ['gz-utils', 'gz-math', 'gz-msgs', 'gz-transport']:
                (client / 'install/lib/cmake' / name).mkdir(parents=True)
            with patch.object(build, 'CLIENT', client):
                cmd = build.configure_command(Path('/reference'), 'gz-sim', {'BUILD_TESTING': 'ON'})
            self.assertIn('-Dgz-transport_DIR=' + str(client / 'install/lib/cmake/gz-transport'), cmd)
            self.assertIn('-Dgz-physics_DIR=/reference/zenoh-dart-install/lib/cmake/gz-physics', cmd)
            self.assertIn('-Dgz-sensors_DIR=/reference/zenoh-dart-install/lib/cmake/gz-sensors', cmd)
            for key in build.selected.LAUNCHERS:
                self.assertIn('-D' + key + '=', cmd)
            self.assertIn('-DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF', cmd)

    def test_environment_cannot_inherit_old_dart_ros_loader_or_compiler_hooks(self):
        hostile = {key: '/opt/ros/hostile' for key in ['LD_PRELOAD', 'LD_AUDIT', 'PYTHONPATH', 'PYTHONHOME',
                   'CMAKE_TOOLCHAIN_FILE', 'CMAKE_FIND_ROOT_PATH', 'CMAKE_PREFIX_PATH', 'CPATH',
                   'PKG_CONFIG_LIBDIR', 'LD_LIBRARY_PATH', *build.selected.LAUNCHERS]}
        metadata = {'lock': {'vendor': {'prefix': 'vendor-prefix'}}}
        with patch.dict(os.environ, hostile):
            env = build.environment(Path('/reference'), metadata)
        for key in hostile:
            self.assertNotIn('/opt/ros/hostile', env.get(key, ''), key)
        self.assertNotIn('/reference/dart-install', env['CMAKE_PREFIX_PATH'])
        self.assertEqual(env['PATH'], '/zenoh-dart-client/install/bin:/usr/bin:/bin')
        self.assertTrue(env['CMAKE_PREFIX_PATH'].startswith('/reference/zenoh-dart-install:'))

    def test_runtime_selection_drops_inherited_config_and_uses_only_new_plugin_roots(self):
        metadata = {'lock': {'vendor': {'prefix': 'vendor-prefix'}}}
        hostile = {'GZ_TRANSPORT_IMPLEMENTATION': 'zeromq', 'ZENOH_CONFIG': '/foreign/config',
                   'GZ_SIM_SYSTEM_PLUGIN_PATH': '/reference/dart-install',
                   'GZ_SIM_PHYSICS_ENGINE_PATH': '/foreign', 'LD_PRELOAD': 'hostile.so', 'LD_AUDIT': 'audit.so'}
        child, controlled = build.smoke_environment(Path('/reference'), metadata, hostile)
        self.assertEqual(child, controlled)
        self.assertNotIn('LD_PRELOAD', child)
        self.assertNotIn('LD_AUDIT', child)
        self.assertEqual(child['GZ_TRANSPORT_IMPLEMENTATION'], 'zenoh')
        self.assertEqual(child['ZENOH_CONFIG'], '/reference/zenoh-dart-smoke/zenoh.json')
        self.assertEqual(child['GZ_SIM_SYSTEM_PLUGIN_PATH'], '/reference/zenoh-dart-install/lib/gz-sim/plugins')
        self.assertEqual(child['GZ_SIM_PHYSICS_ENGINE_PATH'], '/reference/zenoh-dart-install/lib/gz-physics/engine-plugins')

    def test_compiler_witness_requires_real_object_source_and_all_selected_headers(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / 'gz-sim/src/CMakeFiles/core.dir'
            target.mkdir(parents=True)
            obj = target / 'SimulationRunner.cc.o'
            dep = target / 'SimulationRunner.cc.o.d'
            header = root / 'selected/Node.hh'
            header.parent.mkdir()
            header.write_text('controlled header')
            obj.write_bytes(b'controlled object, not an ELF')
            (target / 'flags.make').write_text('CXX_INCLUDES = -I/selected')
            source = '/reference/sources/gz-sim/src/SimulationRunner.cc'
            valid = f'{obj}: {source} {header}\n'
            dep.write_text(valid)
            with patch.object(build, 'BUILD_ROOT', root):
                good = build.compiler_record('gz-sim', 'src/SimulationRunner.cc', [header])
                self.assertEqual(good[str(obj)], build.selected.digest(obj))
                for invalid in [valid.replace(str(header), '/foreign/Node.hh'),
                                valid.replace(source, '/foreign/SimulationRunner.cc'),
                                valid.replace(str(obj), '/foreign/object.o'), 'not a dependency rule']:
                    dep.write_text(invalid)
                    with self.subTest(value=invalid), self.assertRaises(ValueError):
                        build.compiler_record('gz-sim', 'src/SimulationRunner.cc', [header])
                dep.write_text(valid)
                obj.unlink()
                with self.assertRaises(FileNotFoundError):
                    build.compiler_record('gz-sim', 'src/SimulationRunner.cc', [header])

    def loaded_fixture(self):
        roles = [(build.BUILD_ROOT / 'gz-sim/lib', 'libgz-sim.so.11'),
                 (build.PREFIX / 'lib', 'libgz-sim-physics-system.so.11'),
                 (build.BUILD_ROOT / 'gz-physics/lib', 'libgz-physics.so.10'),
                 (build.PREFIX / 'lib', 'libgz-physics-dartsim-plugin.so.10'),
                 *[(build.CLIENT / 'install/lib', f'libgz-{name}.so.1') for name in ['transport', 'msgs', 'math', 'utils']],
                 (build.CLIENT / 'vendor/vendor-prefix/lib', 'libzenohc.so.1')]
        loaded, inventories = {}, {}
        for root, name in roles:
            loaded[str(root / name)] = 'a' * 64
            inventories.setdefault(root, {})[name] = {'sha256': 'a' * 64}
        dart_files = json.loads((ROOT / 'reference/headless-dart.lock.json').read_text())['dart_files_sha256']
        loaded.update({'/' + name: value for name, value in dart_files.items() if '.so.' in name})
        return loaded, inventories, {'lock': {'vendor': {'prefix': 'vendor-prefix'}}}

    def test_dart_witness_selects_dart_object_among_identically_named_tpe_features(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            build_root = root / 'gz-physics'
            header = root / 'config.hpp'
            header.write_text('selected header')
            for backend in ['dartsim', 'tpe/plugin']:
                target = build_root / backend / 'CMakeFiles/plugin.dir/src'
                target.mkdir(parents=True)
                obj = target / 'SimulationFeatures.cc.o'
                obj.write_text(backend + ' object')
                (target.parent / 'flags.make').write_text('controlled target flags')
                (target / 'SimulationFeatures.cc.o.d').write_text(
                    f'{obj}: /reference/sources/gz-physics/{backend}/src/SimulationFeatures.cc {header}\n')
            dart_dep = build_root / 'dartsim/CMakeFiles/plugin.dir/src/SimulationFeatures.cc.o.d'
            with patch.object(build, 'BUILD_ROOT', root):
                witness = build.compiler_record('gz-physics', 'dartsim/src/SimulationFeatures.cc', [header])
                self.assertIn(str(dart_dep), witness)
                flags = dart_dep.parent.parent / 'flags.make'
                self.assertIn(str(flags), witness)
                self.assertFalse(any('/tpe/' in p for p in witness))
                valid = dart_dep.read_text()
                dart_dep.write_text(valid.replace('/dartsim/src/', '/tpe/plugin/src/'))
                with self.assertRaisesRegex(ValueError, 'selected source/headers'):
                    build.compiler_record('gz-physics', 'dartsim/src/SimulationFeatures.cc', [header])
                dart_dep.write_text(valid)
                flags.unlink()
                with self.assertRaises(FileNotFoundError):
                    build.compiler_record('gz-physics', 'dartsim/src/SimulationFeatures.cc', [header])
                outside = root / 'outside-flags'
                outside.write_text('foreign flags')
                flags.symlink_to(outside)
                with self.assertRaisesRegex(ValueError, 'escapes its fresh build'):
                    build.compiler_record('gz-physics', 'dartsim/src/SimulationFeatures.cc', [header])
                flags.unlink()
                dart_dep.unlink()
                with self.assertRaisesRegex(ValueError, 'missing or ambiguous'):
                    build.compiler_record('gz-physics', 'dartsim/src/SimulationFeatures.cc', [header])

    def test_loaded_closure_rejects_old_off_modules_wrong_dart_and_other_backends(self):
        loaded, inventories, client = self.loaded_fixture()
        self.assertEqual(build.validate_loaded(loaded, inventories, client), loaded)
        for role in ['libgz-sim.so.11', 'libgz-physics-dartsim-plugin.so.10', 'libgz-transport.so.1', 'libgz-math.so.1']:
            original = next(p for p in loaded if Path(p).name == role)
            wrong = dict(loaded)
            wrong['/reference/install/lib/' + role] = wrong.pop(original)
            with self.subTest(role=role), self.assertRaisesRegex(ValueError, 'missing or mixed-prefix|historical'):
                build.validate_loaded(wrong, inventories, client)
        for role in ['libdart.so.6.13.2', 'libdart-collision-ode.so.6.13.2', 'libdart-external-odelcpsolver.so.6.13.2']:
            p = '/usr/lib/x86_64-linux-gnu/' + role
            for wrong in [{k: v for k, v in loaded.items() if k != p}, loaded | {p: '0' * 64}]:
                with self.subTest(role=role), self.assertRaises(ValueError):
                    build.validate_loaded(wrong, inventories, client)
        wrong = loaded | {'/unqualified/libgz-physics-tpe-plugin.so.10': 'b' * 64}
        with self.assertRaises(ValueError):
            build.validate_loaded(wrong, inventories, client)
        wrong = loaded | {'/reference/install/lib/libgz-sim-imu-system.so.11': 'b' * 64}
        inventories[Path('/reference/install/lib')] = {'libgz-sim-imu-system.so.11': {'sha256': 'b' * 64}}
        with self.assertRaisesRegex(ValueError, 'historical application'):
            build.validate_loaded(wrong, inventories, client)

    def test_validation_does_not_accept_tpe_manifest_or_missing_inputs(self):
        for metadata in [{'profile': build.pipeline.FULL_APP}, {'profile': build.PROFILE, 'schema_version': 1}]:
            with patch.object(build, 'base_identity'), patch.object(build.selected, 'validate_reference') as client:
                with self.assertRaisesRegex(ValueError, 'contract differs'):
                    build.validate_reference(metadata=metadata)
                client.assert_not_called()


if __name__ == '__main__':
    unittest.main()
