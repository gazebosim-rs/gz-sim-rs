# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Actual APT resolver checks with private synthetic data and no package actions.

No update, sudo, download, dpkg execution, host status or host APT configuration
is used. Cached one-byte payloads are nonexecuted solver fixtures, not packages.
This qualifies dependency selection only, not signatures or a hosted CI run.
"""
import hashlib
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class NativeProvisionResolver(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='gz-ci-apt-resolver-')
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        for name in ['sources', 'preferences.d', 'config.d', 'lists/partial', 'archives/partial']:
            (self.root / name).mkdir(parents=True)
        # No network is contacted: only this local index is read by --simulate.
        (self.root / 'sources/snapshot.list').write_text(
            'deb [trusted=yes] https://snapshot.ubuntu.com/ubuntu/20261024T000000Z/ noble main\n')
        self.status = self.root / 'status'
        self.status.write_text(
            'Package: review-native-runtime\nStatus: install ok installed\n'
            'Priority: optional\nSection: libs\nInstalled-Size: 1\n'
            'Maintainer: Review <review@example.invalid>\nArchitecture: amd64\n'
            'Version: 2\nDescription: private installed-state fixture\n\n')
        (self.root / 'extended_states').write_bytes(b'')
        payload = b'x'
        checksum = hashlib.sha256(payload).hexdigest()
        records = []
        for name, dependency in [('review-native-runtime', ''),
                                 ('review-native-dev', 'Depends: review-native-runtime (= 1)\n')]:
            records.append(
                f'Package: {name}\nVersion: 1\nArchitecture: amd64\n'
                'Maintainer: Review <review@example.invalid>\n' + dependency +
                f'Filename: pool/{name}_1_amd64.deb\nSize: 1\nSHA256: {checksum}\n'
                'Description: private nonexecuted archive fixture\n\n')
            (self.root / 'archives' / f'{name}_1_amd64.deb').write_bytes(payload)
        (self.root / 'lists/snapshot.ubuntu.com_ubuntu_20261024T000000Z_dists_noble_main_binary-amd64_Packages').write_text(''.join(records))
        self.original_status = self.status.read_bytes()
        self.original_archives = {path.name: path.read_bytes() for path in (self.root / 'archives').glob('*.deb')}

    def plan(self, priority=None, origin='snapshot.ubuntu.com'):
        preferences = self.root / 'preferences'
        preferences.write_text('' if priority is None else
                               f'Package: *\nPin: origin "{origin}"\nPin-Priority: {priority}\n')
        options = {
            'Debug::NoLocking': '1',
            'Dir::Etc::main': '/dev/null',
            'Dir::Etc::parts': str(self.root / 'config.d'),
            'Dir::Etc::sourcelist': '/dev/null',
            'Dir::Etc::sourceparts': str(self.root / 'sources'),
            'Dir::Etc::preferences': str(preferences),
            'Dir::Etc::preferencesparts': str(self.root / 'preferences.d'),
            'Dir::State::status': str(self.status),
            'Dir::State::extended_states': str(self.root / 'extended_states'),
            'Dir::State::lists': str(self.root / 'lists'),
            'Dir::Cache::archives': str(self.root / 'archives'),
            'Dir::Cache::pkgcache': str(self.root / 'pkgcache.bin'),
            'Dir::Cache::srcpkgcache': str(self.root / 'srcpkgcache.bin'),
        }
        command = ['/usr/bin/apt-get', '--simulate', '--no-download']
        for key, value in options.items():
            command += ['-o', f'{key}={value}']
        command += ['install', '-y', '--allow-downgrades', 'review-native-dev=1']
        environment = dict(os.environ, LC_ALL='C')
        environment.pop('APT_CONFIG', None)
        result = subprocess.run(command, env=environment, capture_output=True, text=True, timeout=15)
        self.assertEqual(self.status.read_bytes(), self.original_status)
        self.assertEqual((self.root / 'extended_states').read_bytes(), b'')
        self.assertEqual({path.name: path.read_bytes() for path in (self.root / 'archives').glob('*.deb')}, self.original_archives)
        self.assertEqual(list((self.root / 'archives/partial').iterdir()), [])
        return result

    def assert_older_dependency_refused(self, result):
        self.assertEqual(result.returncode, 100, result.stdout + result.stderr)
        self.assertIn('Depends: review-native-runtime (= 1) but 2 is to be installed', result.stdout)
        self.assertNotIn('Inst review-native-runtime', result.stdout)

    def test_allow_downgrades_alone_does_not_select_required_older_dependency(self):
        self.assert_older_dependency_refused(self.plan())

    def test_snapshot_priority_1001_selects_the_required_older_dependency(self):
        result = self.plan(1001)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('Inst review-native-runtime [2] (1 ', result.stdout)
        self.assertIn('Inst review-native-dev (1 ', result.stdout)
        self.assertIn('0 upgraded, 1 newly installed, 1 downgraded, 0 to remove', result.stdout)
        self.assertNotIn('Remv ', result.stdout)

    def test_default_priority_or_unrelated_origin_does_not_force_the_dependency(self):
        for priority, origin in [(500, 'snapshot.ubuntu.com'), (1001, 'unrelated.invalid')]:
            with self.subTest(priority=priority, origin=origin):
                self.assert_older_dependency_refused(self.plan(priority, origin))
