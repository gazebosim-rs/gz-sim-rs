# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Bootstrap integrity controls; synthetic artifacts are not SDK qualification."""
import copy
import importlib.util
import io
import json
import lzma
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('bootstrap_zenoh_native', ROOT / 'scripts/bootstrap-zenoh-native.py')
bootstrap = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bootstrap)
LAUNCHERS = ('CMAKE_C_COMPILER_LAUNCHER', 'CMAKE_CXX_COMPILER_LAUNCHER',
             'CMAKE_C_LINKER_LAUNCHER', 'CMAKE_CXX_LINKER_LAUNCHER')


class BootstrapZenohTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.lock = json.loads((ROOT / 'planning/zenoh-native-lock.json').read_text())
        self.baseline = json.loads((ROOT / 'planning/baseline.json').read_text())

    def test_separate_profile_cannot_admit_off_flags_wrong_sources_or_vendor(self):
        bootstrap.validate_lock(self.lock, self.baseline)
        changes = [
            ('profile', 'transport-client-v1'), ('functional_implementation', 'zeromq'),
            ('cmake', dict(self.lock['cmake'], CMAKE_BUILD_TYPE='Debug')),
            ('transport_cmake', dict(self.lock['transport_cmake'], GZ_TRANSPORT_ENABLE_ZENOH='OFF')),
            ('platform', dict(self.lock['platform'], ubuntu_snapshot='latest')),
            ('packages', {'--unsafe-option': '1.0'}),
            ('vendor', dict(self.lock['vendor'], sha256='unbound')),
            ('vendor', dict(self.lock['vendor'], url='https://example.invalid/decoy.deb')),
            ('vendor', dict(self.lock['vendor'], zenoh_version='1.7.2')),
            ('vendor', dict(self.lock['vendor'], version='../escape')),
        ]
        for key, value in changes:
            changed = dict(self.lock, **{key: value})
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                bootstrap.validate_lock(changed, self.baseline)
        for action in ['missing', 'duplicate', 'floating', 'foreign']:
            changed = copy.deepcopy(self.lock)
            if action == 'missing': changed['repositories'].pop()
            if action == 'duplicate': changed['repositories'].append(changed['repositories'][0])
            if action == 'floating': changed['repositories'][0]['sha'] = 'main'
            if action == 'foreign': changed['repositories'][0]['url'] = 'https://example.invalid/source'
            with self.subTest(action=action), self.assertRaises(ValueError):
                bootstrap.validate_lock(changed, self.baseline)

    def test_build_environment_drops_compiler_loader_and_foreign_git_overrides(self):
        overrides = {name: 'not-used' for name in ['CC', 'CXX', 'CFLAGS', 'CXXFLAGS', 'LDFLAGS',
            'CPATH', 'CPLUS_INCLUDE_PATH', 'LIBRARY_PATH', 'CMAKE_TOOLCHAIN_FILE', 'PKG_CONFIG_SYSROOT_DIR',
            'LD_PRELOAD', 'LD_AUDIT', 'ROS_DISTRO', 'AMENT_PREFIX_PATH', 'GIT_DIR', 'GIT_WORK_TREE', 'GIT_INDEX_FILE',
            *LAUNCHERS]}
        with patch.dict(os.environ, overrides | {'GIT_SSH_COMMAND': 'false'}):
            env = bootstrap.build_environment(self.root / 'install', self.root / 'vendor')
        for name in overrides:
            self.assertNotIn(name, env)
        self.assertEqual(env['GIT_SSH_COMMAND'], 'false')
        self.assertEqual(env['CMAKE_PREFIX_PATH'], str(self.root/'install') + os.pathsep + str(self.root/'vendor'))
        self.assertEqual(env['PKG_CONFIG_PATH'], str(self.root/'install/lib/pkgconfig'))

    def test_functional_child_always_selects_zenoh_with_explicit_isolation(self):
        env = bootstrap.probe_environment({'GZ_IP': 'invalid', 'GZ_TRANSPORT_IMPLEMENTATION': 'zeromq',
                'GZ_TRANSPORT_ZENOH_CONFIG_OVERRIDE': 'foreign', 'ZENOH_CONFIG': 'foreign', 'PATH': '/usr/bin'}, self.root/'config.json')
        self.assertEqual(env['GZ_TRANSPORT_IMPLEMENTATION'], 'zenoh')
        self.assertEqual(env['ZENOH_CONFIG'], str(self.root/'config.json'))
        self.assertNotIn('GZ_IP', env)
        self.assertNotIn('GZ_TRANSPORT_ZENOH_CONFIG_OVERRIDE', env)
        self.assertEqual(env['PATH'], '/usr/bin')

    def test_inventory_rejects_root_links_escapes_broken_links_and_special_entries(self):
        install = self.root/'install'
        install.mkdir()
        (install/'lib').write_bytes(b'synthetic library')
        (install/'alias').symlink_to('lib')
        before = bootstrap.inventory(install)
        self.assertEqual(before['alias'], {'symlink': 'lib'})
        (install/'lib').write_bytes(b'changed')
        self.assertNotEqual(bootstrap.inventory(install), before)
        alias = self.root/'root-alias'
        alias.symlink_to(install)
        with self.assertRaisesRegex(ValueError, 'real directory'):
            bootstrap.inventory(alias)
        (install/'alias').unlink()
        (install/'alias').symlink_to('/etc/passwd')
        with self.assertRaisesRegex(ValueError, 'escapes'):
            bootstrap.inventory(install)
        (install/'alias').unlink()
        (install/'alias').symlink_to('missing')
        with self.assertRaises(FileNotFoundError):
            bootstrap.inventory(install)
        (install/'alias').unlink()
        os.mkfifo(install/'pipe')
        with self.assertRaisesRegex(ValueError, 'unsupported'):
            bootstrap.inventory(install)

    def test_configure_request_is_not_evidence_that_zenoh_was_found(self):
        source, prefix, vendor = [self.root/name for name in ['source', 'install', 'vendor']]
        expected = bootstrap.COMMON | bootstrap.TRANSPORT | {
            'CMAKE_INSTALL_PREFIX': str(prefix), 'CMAKE_HOME_DIRECTORY': str(source),
            'CMAKE_C_COMPILER': '/usr/bin/gcc', 'CMAKE_CXX_COMPILER': '/usr/bin/g++',
            'HAVE_ZENOH': 'ON', 'zenohc_DIR': str(vendor/'lib/cmake/zenohc'),
            'zenohcxx_DIR': str(vendor/'lib/cmake/zenohcxx')}
        cache = self.root/'CMakeCache.txt'
        def write(values): cache.write_text(''.join(f'{key}:STRING={value}\n' for key, value in values.items()))
        write(expected)
        bootstrap.validate_cache(cache, source, prefix, vendor, True)
        for key, value in [('HAVE_ZENOH', 'OFF'), ('GZ_TRANSPORT_ENABLE_ZENOH', 'OFF'),
                           ('zenohc_DIR', '/opt/foreign'), ('CMAKE_CXX_COMPILER', '/tmp/foreign'),
                           ('CMAKE_INSTALL_PREFIX', '/tmp/foreign'), ('CMAKE_BUILD_TYPE', 'Debug')]:
            write(expected | {key: value})
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, key):
                bootstrap.validate_cache(cache, source, prefix, vendor, True)
        for key in LAUNCHERS:
            self.assertEqual(self.lock['cmake'][key], '')
            write(expected | {key: '/tmp/unqualified-wrapper;--rewrite'})
            with self.subTest(launcher=key), self.assertRaisesRegex(ValueError, key):
                bootstrap.validate_cache(cache, source, prefix, vendor, True)
            missing = dict(expected)
            missing.pop(key)
            write(missing)
            with self.subTest(missing_launcher=key), self.assertRaisesRegex(ValueError, key):
                bootstrap.validate_cache(cache, source, prefix, vendor, True)

    def test_real_cmake_sanitized_environment_and_empty_flags_block_launchers(self):
        # A tiny native build tests CMake's launcher initialization, not the SDK.
        source = self.root/'tiny-source'; source.mkdir()
        (source/'CMakeLists.txt').write_text('cmake_minimum_required(VERSION 3.22)\nproject(launcher_control LANGUAGES C CXX)\nadd_executable(control main.cc)\n')
        (source/'main.cc').write_text('int main() { return 0; }\n')
        marker = self.root/'unqualified-launcher-ran'
        launcher = self.root/'launcher'
        launcher.write_text('#!' + sys.executable + '\nimport os,sys\nfrom pathlib import Path\nPath(' + repr(str(marker)) + ').touch()\nos.execvp(sys.argv[1],sys.argv[1:])\n')
        launcher.chmod(0o755)
        contaminated = bootstrap.git_environment() | dict.fromkeys(LAUNCHERS, str(launcher))
        with patch.dict(os.environ, dict.fromkeys(LAUNCHERS, str(launcher))):
            sanitized = bootstrap.build_environment(self.root/'install', self.root/'vendor')
        for name, env in [('contaminated', contaminated), ('sanitized', sanitized)]:
            build = self.root/name
            command = ['/usr/bin/cmake', '-S', str(source), '-B', str(build),
                       '-DCMAKE_C_COMPILER=/usr/bin/gcc', '-DCMAKE_CXX_COMPILER=/usr/bin/g++',
                       *(f'-D{key}={self.lock["cmake"][key]}' for key in LAUNCHERS)]
            for command in [command, ['/usr/bin/cmake', '--build', str(build), '--parallel', '2']]:
                result = subprocess.run(command, env=env, capture_output=True)
                self.assertEqual(result.returncode, 0, result.stdout.decode(errors='replace') + result.stderr.decode(errors='replace'))
            if name == 'contaminated':
                self.assertTrue(marker.exists(), 'the positive control must actually invoke the unqualified wrapper')
                marker.unlink()
        self.assertFalse(marker.exists())
        values = bootstrap.cache_values(self.root/'sanitized/CMakeCache.txt')
        for key in LAUNCHERS:
            self.assertEqual(values[key], '')

    def test_vendor_version_and_kernel_feature_disclosure_are_exact(self):
        for name in ['zenohc', 'zenohcxx']:
            path = self.root/f'lib/cmake/{name}/{name}ConfigVersion.cmake'
            path.parent.mkdir(parents=True)
            path.write_text('set(PACKAGE_VERSION "1.8.0")\n')
        config = self.root/'lib/cmake/zenohc/zenohcConfig.cmake'
        text = 'set(ZENOHC_BUILD_WITH_UNSTABLE_API TRUE)\nset(ZENOHC_BUILD_WITH_SHARED_MEMORY TRUE)\n'
        config.write_text(text)
        self.assertEqual(bootstrap.vendor_versions(self.root, '1.8.0'), {'UNSTABLE_API': True, 'SHARED_MEMORY': True})
        config.write_text(text.replace('TRUE', 'FALSE', 1))
        with self.assertRaisesRegex(ValueError, 'features'):
            bootstrap.vendor_versions(self.root, '1.8.0')
        config.write_text(text)
        version = self.root/'lib/cmake/zenohcxx/zenohcxxConfigVersion.cmake'
        version.write_text('set(PACKAGE_VERSION "1.9.0")\n')
        with self.assertRaisesRegex(ValueError, 'zenohcxx'):
            bootstrap.vendor_versions(self.root, '1.8.0')

    def test_linkage_requires_both_generated_message_and_zenoh_libraries(self):
        prefix, vendor = self.root/'install', self.root/'vendor'
        (prefix/'lib').mkdir(parents=True)
        (vendor/'lib').mkdir(parents=True)
        names = ['libgz-transport.so.16', 'libgz-msgs.so.13', 'libzenohc.so']
        paths = [prefix/'lib'/names[0], prefix/'lib'/names[1], vendor/'lib'/names[2]]
        for path in paths: path.write_bytes(b'synthetic nonexecuted artifact')
        rows = [f'{name} => {path} (0x01)' for name, path in zip(names, paths)]
        self.assertEqual(set(bootstrap.validate_linkage('\n'.join(rows), prefix, vendor)), set(names))
        for missing in range(3):
            with self.subTest(missing=missing), self.assertRaisesRegex(ValueError, 'missing'):
                bootstrap.validate_linkage('\n'.join(rows[:missing]+rows[missing+1:]), prefix, vendor)
        with self.assertRaisesRegex(ValueError, 'unresolved'):
            bootstrap.validate_linkage('\n'.join(rows)+'\nother => not found', prefix, vendor)
        foreign = self.root/'foreign.so'
        foreign.write_bytes(b'foreign')
        for index in range(3):
            changed = rows.copy(); changed[index] = f'{names[index]} => {foreign} (0x02)'
            with self.subTest(index=index), self.assertRaisesRegex(ValueError, 'escapes'):
                bootstrap.validate_linkage('\n'.join(changed), prefix, vendor)

    def test_exact_vendor_archive_control_and_checksum_precede_extraction(self):
        package = self.root/'package'
        (package/'DEBIAN').mkdir(parents=True)
        (package/'DEBIAN/control').write_text('Package: fixture-zenoh\nVersion: 1.0\nArchitecture: amd64\nMaintainer: Test <test@example.invalid>\nDescription: Nonexecuted extraction fixture\n')
        (package/'payload').write_text('fixture')
        archive = self.root/'fixture.deb'
        subprocess.run(['dpkg-deb', '--build', package, archive], check=True, capture_output=True)
        vendor = {'url': 'https://unused.invalid/fixture.deb', 'sha256': bootstrap.digest(archive),
                  'package': 'fixture-zenoh', 'version': '1.0', 'architecture': 'amd64'}
        root = self.root/'reference'
        env = bootstrap.builder.git_environment()
        with patch.object(bootstrap.urllib.request, 'urlretrieve', side_effect=AssertionError('no network')):
            bootstrap.vendor_archive(root, vendor, archive, self.root/'extract.log', env)
        self.assertEqual((root/'vendor/payload').read_text(), 'fixture')
        with self.assertRaisesRegex(ValueError, 'already exists'):
            bootstrap.vendor_archive(root, vendor, archive, self.root/'extract.log', env)
        for changed in [vendor | {'sha256': '0'*64}, vendor | {'version': '2.0'}]:
            with self.subTest(changed=changed), self.assertRaisesRegex(ValueError, 'differs'):
                bootstrap.vendor_archive(self.root/'reject', changed, archive, self.root/'extract.log', env)
        self.assertFalse((self.root/'reject/vendor').exists())

    def test_transport_elf_must_directly_link_zenoh_not_only_a_transitive_peer(self):
        with patch.object(bootstrap.builder, 'output', return_value='(NEEDED) Shared library: [libzenohc.so]'):
            bootstrap.transport_dynamic(self.root, {})
        with patch.object(bootstrap.builder, 'output', return_value='(NEEDED) Shared library: [libgz-msgs.so.13]'):
            with self.assertRaisesRegex(ValueError, 'direct Zenoh'):
                bootstrap.transport_dynamic(self.root, {})

    def test_wrong_host_package_or_protobuf_is_not_accepted(self):
        versions = self.lock['packages']
        def output(command, env):
            return versions[command[-1]] if command[0] == 'dpkg-query' else 'libprotoc 3.21.12'
        with patch.object(bootstrap.platform, 'machine', return_value='x86_64'), patch.object(bootstrap.builder, 'output', side_effect=output):
            self.assertEqual(bootstrap.verify_host(self.lock, {}), versions)
        for value in ['wrong package version', 'libprotoc 36.2']:
            with patch.object(bootstrap.builder, 'output', return_value=value), self.assertRaises(ValueError):
                bootstrap.verify_host(self.lock, {})

    def test_input_mutation_and_legacy_manifest_fail_without_builds(self):
        initial = bootstrap.inputs()
        changed = dict(initial)
        changed['scripts/bootstrap-zenoh-native.py'] = '0'*64
        with patch.object(bootstrap, 'inputs', return_value=changed), self.assertRaisesRegex(ValueError, 'input changed'):
            bootstrap.verify_inputs(initial)
        root = self.root/'reference'
        root.mkdir()
        (root/'environment.json').write_text(json.dumps({'profile': 'transport-client-v1'}))
        with self.assertRaisesRegex(ValueError, 'contract differs'):
            bootstrap.validate_reference(root)
        alias = self.root/'alias'; alias.symlink_to(root)
        with self.assertRaisesRegex(ValueError, 'real directory'):
            bootstrap.validate_reference(alias)

    def test_real_foreign_git_environment_and_dirty_ignored_inputs_fail_closed(self):
        source = self.root/'source'; source.mkdir()
        env = bootstrap.builder.git_environment()
        def git(*args):
            return subprocess.check_output(['git', '-C', source, *args], env=env, stderr=subprocess.PIPE).decode().strip()
        git('init', '-q'); (source/'input').write_text('pin')
        git('add', 'input')
        git('-c', 'user.name=Reference test', '-c', 'user.email=test@example.invalid', '-c', 'core.hooksPath=/dev/null', 'commit', '-qm', 'pin')
        row = {'sha': git('rev-parse', 'HEAD')}
        bogus = self.root/'never-created-index'
        with patch.dict(os.environ, {'GIT_DIR': str(ROOT/'.git'), 'GIT_WORK_TREE': str(ROOT), 'GIT_INDEX_FILE': str(bogus)}):
            bootstrap.builder.verify_source(source, row, bootstrap.builder.git_environment())
        self.assertFalse(bogus.exists())
        (source/'.git/info/exclude').write_text('ignored-payload\n')
        (source/'ignored-payload').write_text('untracked build input')
        with self.assertRaisesRegex(ValueError, 'dirty'):
            bootstrap.builder.verify_source(source, row, env)

    def test_failed_build_command_retains_streams(self):
        log = self.root/'failure.log'
        with self.assertRaises(subprocess.CalledProcessError):
            bootstrap.builder.run([sys.executable, '-c', 'import sys;print("out");print("err",file=sys.stderr);sys.exit(7)'], log, bootstrap.builder.git_environment())
        self.assertIn('out\n', log.read_text())
        self.assertIn('err\n', log.read_text())
        self.assertFalse((self.root/'environment.json').exists())

    def test_package_proof_covers_exact_versions_and_all_direct_dependencies(self):
        pins = json.loads((ROOT/'planning/zenoh-native-packages.json').read_text())
        bootstrap.validate_package_pins(pins, self.lock)
        for action in ['missing', 'version', 'snapshot', 'key', 'architecture']:
            changed = copy.deepcopy(pins)
            if action == 'missing':
                name = changed['packages'][0]['Package']
                changed['packages'] = [row for row in changed['packages'] if row['Package'] != name]
            if action == 'version': changed['packages'][0]['Version'] = '0.0'
            if action == 'snapshot': changed['snapshot'] = 'latest'
            if action == 'key': changed['signing_key'] = '0'*40
            if action == 'architecture': changed['packages'][0]['Architecture'] = 'arm64'
            with self.subTest(action=action), self.assertRaises(ValueError):
                bootstrap.validate_package_pins(changed, self.lock)

    def test_metadata_cache_and_fresh_download_require_exact_hashes(self):
        path = self.root/'index'
        payload = b'synthetic package index bytes'
        expected = bootstrap.hashlib.sha256(payload).hexdigest()
        with patch.object(bootstrap.urllib.request, 'urlopen', return_value=io.BytesIO(payload)):
            bootstrap.checked_download('https://unused.invalid/index', path, expected)
        with patch.object(bootstrap.urllib.request, 'urlopen', side_effect=AssertionError('cache is local')):
            bootstrap.checked_download('https://unused.invalid/index', path, expected)
        path.write_bytes(b'changed cache')
        with self.assertRaisesRegex(ValueError, 'cached metadata changed'):
            bootstrap.checked_download('https://unused.invalid/index', path, expected)
        path.unlink()
        with patch.object(bootstrap.urllib.request, 'urlopen', return_value=io.BytesIO(b'wrong')):
            with self.assertRaisesRegex(ValueError, 'download checksum differs'):
                bootstrap.checked_download('https://unused.invalid/index', path, expected)
        self.assertFalse(path.exists())
        target = self.root/'target'; target.write_bytes(payload)
        path.symlink_to(target)
        with self.assertRaisesRegex(ValueError, 'symlink'):
            bootstrap.checked_download('https://unused.invalid/index', path, expected)

    def test_release_and_package_parsers_keep_exact_digest_size_and_last_record(self):
        text = 'Origin: fixture\nSHA256:\n ' + 'a'*64 + ' 123 main/binary-amd64/Packages.xz\nOther: field\n'
        self.assertEqual(bootstrap.release_hashes(text), {'main/binary-amd64/Packages.xz': ('a'*64, 123)})
        path = self.root/'Packages.xz'
        path.write_bytes(lzma.compress(b'Package: first\nVersion: 1\nDescription: test\n continued\n\n\nPackage: second\nVersion: 2\n'))
        self.assertEqual(list(bootstrap.package_records(path)), [
            {'Package': 'first', 'Version': '1', 'Description': 'test'}, {'Package': 'second', 'Version': '2'}])

    def test_real_gpgv_refuses_a_checksum_bound_but_unsigned_release(self):
        pins = json.loads((ROOT/'planning/zenoh-native-packages.json').read_text())
        fake_project = self.root/'project'
        (fake_project/'planning').mkdir(parents=True)
        metadata = self.root/'metadata'; metadata.mkdir()
        bad = metadata/'noble.InRelease'
        bad.write_text('Origin: untrusted fixture, deliberately not signed\n')
        pins['releases'][0]['sha256'] = bootstrap.digest(bad)
        (fake_project/'planning/zenoh-native-packages.json').write_text(json.dumps(pins))
        with patch.object(bootstrap, 'ROOT', fake_project), patch.object(bootstrap.urllib.request, 'urlopen', side_effect=AssertionError('no network')):
            with self.assertRaises(subprocess.CalledProcessError):
                bootstrap.verify_package_indexes(metadata, self.lock)
        self.assertTrue((metadata/'noble.gpgv.stderr').read_text())
        self.assertFalse((metadata/'environment.json').exists())

    def test_manifest_is_not_published_after_validation_input_or_tool_drift(self):
        expected = bootstrap.inputs()
        tool = self.root/'tool'; tool.write_bytes(b'current executable fixture')
        metadata = {'inputs_sha256': expected, 'tools': {'fixture': {'path': str(tool), 'sha256': bootstrap.digest(tool)}}}
        with patch.object(bootstrap, 'validate_reference', side_effect=ValueError('invalid installed inventory')):
            with self.assertRaisesRegex(ValueError, 'inventory'):
                bootstrap.publish_manifest(self.root, self.lock, metadata)
        self.assertFalse((self.root/'environment.json').exists())
        with patch.object(bootstrap, 'validate_reference'), patch.object(bootstrap, 'inputs', return_value={}):
            with self.assertRaisesRegex(ValueError, 'input changed'):
                bootstrap.publish_manifest(self.root, self.lock, metadata)
        self.assertFalse((self.root/'environment.json').exists())
        tool.write_bytes(b'changed after validation')
        with patch.object(bootstrap, 'validate_reference'):
            with self.assertRaisesRegex(ValueError, 'tool changed'):
                bootstrap.publish_manifest(self.root, self.lock, metadata)
        self.assertFalse((self.root/'environment.json').exists())


if __name__ == '__main__':
    unittest.main()
