# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import copy
import importlib.util
import hashlib
import json
import shutil
import sys
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('native_reference', Path(__file__).resolve().parents[1] / 'scripts/native_reference.py')
reference = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reference)


class NativeReferenceTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.prefix = self.root / 'install'
        (self.prefix / 'lib').mkdir(parents=True)
        library = self.prefix / 'lib/libfixture.so.1'
        library.write_bytes(b'nonexecuted library inventory fixture')
        (self.prefix / 'lib/libfixture.so').symlink_to(library.name)
        source = self.root / 'sources/fixture'
        source.mkdir(parents=True)
        (source / 'NOTICE').write_text('local source identity fixture\n')
        env = reference.builder.git_environment()
        def git(*args):
            return subprocess.check_output(['git', '-C', str(source), *args], env=env, stderr=subprocess.PIPE)
        git('init', '--quiet')
        git('add', 'NOTICE')
        git('-c', 'user.name=Reference test', '-c', 'user.email=reference@example.invalid', 'commit', '--quiet', '-m', 'fixture')
        self.source = source
        (self.root / 'input').write_bytes(b'copied input fixture')
        self.sdk = {'inputs_sha256': {'input': reference.builder.digest(self.root / 'input')},
                    'installed_files': reference.builder.files_under(self.prefix),
                    'repositories': [{'name': 'fixture', 'sha': git('rev-parse', 'HEAD').decode().strip()}],
                    'tools_sha256': {'c++': 'expected compiler fixture'}}
        self.lock = {'profile': 'legacy fixture'}
        self.legacy = {'source_lock': self.lock, 'libraries_sha256': {'lib/libfixture.so.1': reference.builder.digest(library)}}

    def validate(self, metadata=None, profile=reference.SDK):
        return reference.validate_reference(self.prefix, self.sdk if metadata is None else metadata,
                                            self.lock, self.sdk, profile)

    def test_profiles_remain_distinct_and_require_explicit_sdk_selection(self):
        sdk = self.validate()
        self.assertEqual(sdk['profile'], reference.SDK)
        self.assertNotIn('source_lock', sdk)
        legacy = self.validate(self.legacy, reference.LEGACY)
        self.assertEqual(legacy['source_lock'], self.lock)
        for metadata, profile in [(self.sdk, reference.LEGACY), (self.legacy, reference.SDK),
                                  (self.sdk, 'unknown'), (self.legacy | {'installed_files': {}}, reference.LEGACY)]:
            with self.subTest(profile=profile), self.assertRaises(ValueError):
                self.validate(metadata, profile)

    def test_changed_manifest_and_copied_input_fail(self):
        changed = copy.deepcopy(self.sdk)
        changed['repositories'][0]['sha'] = '0' * 40
        with self.assertRaisesRegex(ValueError, 'qualified manifest'):
            self.validate(changed)
        (self.root / 'input').write_bytes(b'changed input')
        with self.assertRaisesRegex(ValueError, 'SDK input changed'):
            self.validate()

    def test_modified_extra_missing_and_escaping_installed_files_fail(self):
        library = self.prefix / 'lib/libfixture.so.1'
        original = library.read_bytes()
        library.write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'inventory changed'):
            self.validate()
        library.write_bytes(original)
        extra = self.prefix / 'extra'
        extra.write_bytes(b'extra')
        with self.assertRaisesRegex(ValueError, 'inventory changed'):
            self.validate()
        extra.unlink()
        link = self.prefix / 'lib/libfixture.so'
        link.unlink()
        with self.assertRaisesRegex(ValueError, 'inventory changed'):
            self.validate()
        link.symlink_to(self.root / 'input')
        with self.assertRaisesRegex(ValueError, 'escapes prefix'):
            self.validate()

    def test_dirty_source_and_unsupported_installed_type_fail(self):
        (self.source / 'NOTICE').write_text('modified source')
        with self.assertRaisesRegex(ValueError, 'source is dirty'):
            self.validate()
        (self.source / 'NOTICE').write_text('local source identity fixture\n')
        os.mkfifo(self.prefix / 'unexpected-pipe')
        with self.assertRaisesRegex(ValueError, 'unsupported'):
            self.validate()

    def test_legacy_changed_library_and_path_escape_fail(self):
        changed = copy.deepcopy(self.legacy)
        changed['source_lock'] = {'profile': 'different'}
        with self.assertRaises(ValueError):
            self.validate(changed, reference.LEGACY)
        changed = self.legacy | {'libraries_sha256': {'../input': self.sdk['inputs_sha256']['input']}}
        with self.assertRaisesRegex(ValueError, 'escapes'):
            self.validate(changed, reference.LEGACY)
        (self.prefix / 'lib/libfixture.so.1').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'library changed'):
            self.validate(self.legacy, reference.LEGACY)

    def test_compiler_identity_requires_the_qualified_executable(self):
        compiler = self.root / 'c++'
        compiler.write_text('#!/bin/sh\nprintf "compiler fixture\\n"\n')
        compiler.chmod(0o755)
        env = dict(os.environ, PATH=str(self.root))
        expected = reference.builder.digest(compiler)
        _, record = reference.compiler_identity(env, expected)
        self.assertEqual(record['sha256'], expected)
        with self.assertRaisesRegex(ValueError, 'differs'):
            reference.compiler_identity(env, 'wrong')
        with self.assertRaisesRegex(ValueError, 'unavailable'):
            reference.compiler_identity(dict(env, PATH=''))

    def test_synthetic_compile_mutations_cannot_publish_provenance(self):
        # The controlled compiler copies a system executable; this is not native credit.
        repo = self.root / 'peer-source'
        (repo / 'scripts').mkdir(parents=True)
        (repo / 'reference').mkdir()
        (repo / 'planning').mkdir()
        cpp = repo / 'crates/gz-sim-server/tests/clock_client.cc'
        cpp.parent.mkdir(parents=True)
        cpp.write_text('int main() {}\n')
        project = Path(__file__).resolve().parents[1]
        for name in ['native-peer.py', 'native_reference.py']:
            shutil.copy2(project / 'scripts' / name, repo / 'scripts' / name)
        shutil.copy2(project / 'reference/build-reference.py', repo / 'reference/build-reference.py')
        (repo / 'planning/transport-reference-lock.json').write_text(json.dumps(self.lock))
        tools = self.root / 'tools'
        tools.mkdir()
        pkg = tools / 'pkg-config'
        pkg.write_text('#!' + sys.executable + '\nimport sys\nif "--variable=prefix" in sys.argv: print(' + repr(str(self.prefix)) + ')\n')
        pkg.chmod(0o755)
        compiler = tools / 'c++'
        true = shutil.which('true')
        self.assertIsNotNone(true)
        code = """#!{python}
import os,sys,shutil
from pathlib import Path
if sys.argv[1:] == ['--version']:
    print('synthetic compiler fixture');raise SystemExit()
out=Path(sys.argv[sys.argv.index('-o')+1]);shutil.copyfile({executable},out);out.chmod(0o755)
kind=os.environ['REFERENCE_TEST_MUTATION']
if kind=='library': Path({library}).write_bytes(b'changed library')
if kind=='source': Path({source}).write_text('changed source')
if kind=='metadata': Path({metadata}).write_text('{{}}')
if kind=='compiler': Path(__file__).write_text(Path(__file__).read_text()+chr(10)+'# changed'+chr(10))
""".format(python=sys.executable, executable=repr(true), library=repr(str(self.prefix/'lib/libfixture.so.1')),
           source=repr(str(cpp)), metadata=repr(str(self.root/'environment.json')))
        original_library = (self.prefix / 'lib/libfixture.so.1').read_bytes()
        for mutation in ['none', 'library', 'source', 'metadata', 'compiler']:
            with self.subTest(mutation=mutation):
                compiler.write_text(code)
                compiler.chmod(0o755)
                cpp.write_text('int main() {}\n')
                (self.prefix / 'lib/libfixture.so.1').write_bytes(original_library)
                (self.root / 'environment.json').write_text(json.dumps(self.legacy))
                output = repo / 'artifacts/native-peer/provenance.json'
                output.unlink(missing_ok=True)
                env = dict(os.environ, PATH=str(tools)+os.pathsep+os.environ['PATH'],
                           GZ_REFERENCE_PROFILE=reference.LEGACY, GZ_REFERENCE_PREFIX=str(self.prefix),
                           REFERENCE_TEST_MUTATION=mutation)
                result = subprocess.run([sys.executable, str(repo/'scripts/native-peer.py')], env=env,
                                        capture_output=True, timeout=20)
                self.assertEqual(result.returncode == 0, mutation == 'none', result.stderr)
                self.assertEqual(output.exists(), mutation == 'none')
                if mutation == 'library':
                    self.assertEqual((self.prefix/'lib/libfixture.so.1').read_bytes(), b'changed library')
                    self.assertIn(b'library changed', result.stderr)
                elif mutation == 'source':
                    self.assertEqual(cpp.read_text(), 'changed source')
                    self.assertIn(b'input changed during compilation', result.stderr)
                elif mutation == 'metadata':
                    self.assertEqual((self.root/'environment.json').read_text(), '{}')
                    self.assertIn(b'input changed during compilation', result.stderr)
                elif mutation == 'compiler':
                    self.assertNotEqual(compiler.read_text(), code)
                    self.assertIn(b'compiler differs', result.stderr)
                if mutation == 'none':
                    data = json.loads(output.read_text())
                    self.assertEqual(data['peer_source_sha256'], hashlib.sha256(cpp.read_bytes()).hexdigest())
