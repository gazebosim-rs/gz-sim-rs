# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import struct
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
with patch.object(sys, 'path', [str(ROOT / 'scripts'), *sys.path]):
    module_spec = importlib.util.spec_from_file_location('preview_package', ROOT / 'scripts/package-preview.py')
    package = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(package)
runtime_spec = importlib.util.spec_from_file_location('installed_preview', ROOT / 'scripts/preview_runtime.py')
runtime = importlib.util.module_from_spec(runtime_spec)
runtime_spec.loader.exec_module(runtime)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class PreviewPackage(unittest.TestCase):
    """Synthetic input admission controls; actual ELF/browser installation is separate."""
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='gz-preview-package-')
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.source = self.directory / 'source'
        self.source.mkdir()
        for name in ['scripts/preview_runtime.py', 'examples/m8_diff_drive.sdf',
                     'docs/M12_INSTALL.md', 'docs/M12_CAPABILITIES.md', 'LICENSE', 'NOTICE']:
            path = self.source / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('source fixture ' + name)
        shutil.copyfile(ROOT / 'scripts/preview_runtime.py', self.source / 'scripts/preview_runtime.py')
        self.viewer = self.directory / 'viewer'
        self.viewer.mkdir()
        self.viewer_hashes = {}
        for name in package.VIEWER_FILES:
            path = self.viewer / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b'\0asm\x01\0\0\0' if name.endswith('.wasm') else b'fixture browser text')
            self.viewer_hashes[name] = sha(path)
        self.binaries, artifacts, packages = {}, [], []
        for target, crate in [('gz-sim-rs', 'gz-sim-cli'), ('drive_command', 'gz-sim-server')]:
            path = self.directory / target
            header = bytearray(64)
            header[:6] = b'\x7fELF\x02\x01'
            struct.pack_into('<H', header, 18, 62)
            path.write_bytes(header)
            path.chmod(0o755)
            crate_root = self.source / 'crates' / crate
            crate_root.mkdir(parents=True)
            (crate_root / 'Cargo.toml').write_text('fixture')
            identity = 'path+file://' + str(crate_root) + '#0.1.0'
            artifacts.append({'reason': 'compiler-artifact', 'executable': str(path),
                              'package_id': identity, 'target': {'name': target}, 'features': [],
                              'profile': {'test': False, 'opt_level': '3'}})
            packages.append({'id': identity, 'name': crate, 'version': '0.1.0', 'source': None,
                             'license': 'Apache-2.0', 'manifest_path': str(crate_root / 'Cargo.toml')})
            self.binaries[target] = path
        self.transcript = self.directory / 'cargo.jsonl'
        self.transcript.write_text(''.join(json.dumps(event) + '\n' for event in [*artifacts, {'reason': 'build-finished', 'success': True}]))
        self.artifacts = artifacts
        metadata = self.directory / 'native-metadata.json'
        metadata.write_text(json.dumps({'workspace_root': str(self.source), 'packages': packages}))
        viewer_crate = self.viewer / 'crates/gzweb'
        viewer_crate.mkdir(parents=True)
        (viewer_crate / 'Cargo.toml').write_text('fixture')
        (viewer_crate / 'LICENSE-MIT').write_text('actual test MIT notice')
        viewer_metadata = self.directory / 'viewer-metadata.json'
        viewer_metadata.write_text(json.dumps({'workspace_root': str(self.viewer), 'packages': [
            {'id': 'viewer-id', 'name': 'gzweb', 'version': '0.1.0', 'source': None,
             'license': 'MIT', 'manifest_path': str(viewer_crate / 'Cargo.toml')}]}))
        extra_names = ['licenses/Apache-2.0.txt', 'OSRF-NOTICE.txt', 'Geist-OFL.txt',
                       'Rust-COPYRIGHT.html', 'Rust-LICENSE-APACHE.txt', 'Rust-LICENSE-MIT.txt',
                       'Viewer-mesh-loader-LICENSE.txt', 'Viewer-proto-LICENSE.txt']
        extras = {}
        for number, name in enumerate(extra_names):
            path = self.directory / ('notice-' + str(number))
            path.write_text('actual test attribution ' + name)
            extras[name] = str(path)
        self.spec = {'schema_version': 1, 'target': 'ubuntu-24.04-x86_64',
                     'repositories': {'gz-sim-rs': 'a' * 40, 'gzweb-rs': 'b' * 40},
                     'simulator': self.binary_record('gz-sim-rs'),
                     'drive_command': self.binary_record('drive_command'),
                     'viewer_root': str(self.viewer), 'viewer_files': self.viewer_hashes,
                     'cargo_metadata': [
                         {'path': str(metadata), 'sha256': sha(metadata), 'repository': 'gz-sim-rs',
                          'commit': 'a' * 40, 'configuration': 'native',
                          'selected_package_ids': [entry['id'] for entry in packages]},
                         *[{'path': str(viewer_metadata), 'sha256': sha(viewer_metadata),
                            'repository': 'gzweb-rs', 'commit': 'b' * 40,
                            'configuration': flavor, 'selected_package_ids': ['viewer-id']}
                           for flavor in ['viewer-webgpu', 'viewer-webgl2']]],
                     'extra_notices': extras}
        self.output = self.directory / 'preview-v1'
        root_patch = patch.object(package, 'ROOT', self.source)
        root_patch.start()
        self.addCleanup(root_patch.stop)

    def binary_record(self, target):
        path = self.binaries[target]
        return {'path': str(path), 'sha256': sha(path), 'cargo_messages': str(self.transcript),
                'cargo_messages_sha256': sha(self.transcript)}

    def refresh_transcript(self):
        self.transcript.write_text(''.join(json.dumps(event) + '\n' for event in [*self.artifacts, {'reason': 'build-finished', 'success': True}]))
        for key in ['simulator', 'drive_command']:
            self.spec[key]['cargo_messages_sha256'] = sha(self.transcript)

    def refuse(self, expression):
        with self.assertRaisesRegex(ValueError, expression):
            package.assemble(self.spec, self.output)
        self.assertFalse(self.output.exists())

    def test_owned_bytes_notices_manifest_relocation_and_safe_archive(self):
        archive = self.directory / 'preview-v1.tar.gz'
        manifest = package.assemble(self.spec, self.output, archive)
        self.assertEqual(manifest['repositories'], self.spec['repositories'])
        self.assertEqual(len(manifest['viewer_gzip_bytes']), 2)
        self.assertTrue(all(value <= 14_000_000 for value in manifest['viewer_gzip_bytes'].values()))
        self.assertFalse(any(path.is_symlink() for path in self.output.rglob('*')))
        notices = json.loads((self.output / 'notices.json').read_text())
        self.assertEqual(len(notices['packages']), 3)
        self.assertNotIn(str(self.directory), (self.output / 'notices.json').read_text())
        moved = self.directory / 'installed elsewhere'
        shutil.move(self.output, moved)
        self.assertEqual(runtime.verify_prefix(moved)['files'], manifest['files'])
        self.assertEqual(json.loads((moved / 'share/zenoh.json5').read_text())['listen']['endpoints'],
                         ['tcp/127.0.0.1:7449'])
        with tarfile.open(archive, 'r:gz') as packed:
            members = packed.getmembers()
            self.assertTrue(members)
            for member in members:
                self.assertTrue(member.name.startswith('preview-v1/'))
                self.assertNotIn('..', Path(member.name).parts)
                self.assertFalse(member.issym() or member.islnk())
                self.assertEqual((member.uid, member.gid, member.mtime), (0, 0, 0))
            self.assertEqual(packed.extractfile('preview-v1/manifest.json').read(),
                             (moved / 'manifest.json').read_bytes())

    def test_native_backend_or_wrong_elf_and_transcript_drift_are_refused(self):
        self.artifacts[0]['features'] = ['native-backends']
        self.refresh_transcript()
        self.refuse('native-free')
        self.artifacts[0]['features'] = []
        self.refresh_transcript()
        binary = self.binaries['gz-sim-rs']
        binary.write_bytes(b'not an ELF')
        self.spec['simulator']['sha256'] = sha(binary)
        self.refuse('ELF')
        self.spec['drive_command']['cargo_messages_sha256'] = '0' * 64
        with self.assertRaisesRegex(ValueError, 'hash'):
            package.binary_input(self.spec['drive_command'], 'drive_command', 'gz-sim-server')

    def test_failed_missing_and_duplicate_build_completion_are_refused(self):
        success = {'reason': 'build-finished', 'success': True}
        for endings in ([], [{'reason': 'build-finished', 'success': False}], [success, success]):
            self.transcript.write_text(''.join(json.dumps(event) + '\n' for event in [*self.artifacts, *endings]))
            for key in ('simulator', 'drive_command'):
                self.spec[key]['cargo_messages_sha256'] = sha(self.transcript)
            self.refuse('terminal successful build-finished')

    def test_missing_viewer_flavor_hash_and_escaping_input_are_refused(self):
        self.spec['viewer_files'] = dict(self.viewer_hashes)
        removed = self.spec['viewer_files'].pop('startup.mjs')
        self.refuse('six')
        self.spec['viewer_files']['startup.mjs'] = '0' * 64
        self.refuse('hash')
        self.spec['viewer_files']['startup.mjs'] = removed
        selected = self.viewer / 'startup.mjs'
        external = self.directory / 'outside.mjs'
        selected.rename(external)
        selected.symlink_to(external)
        self.refuse('regular')

    def test_all_three_metadata_graphs_and_native_notices_are_required(self):
        self.spec['cargo_metadata'].pop()
        self.refuse('both accepted viewer')
        self.spec['cargo_metadata'] = []
        self.refuse('metadata inputs')

    def test_missing_notice_never_publishes_a_partial_prefix(self):
        del self.spec['extra_notices']['Geist-OFL.txt']
        self.refuse('notices are required')
        self.assertEqual(list(self.directory.glob('.gz-preview-assembly-*')), [])

    def test_per_flavor_budget_and_missing_compiled_package_notices_are_refused(self):
        path = self.viewer / 'target/wasm32-unknown-unknown/release/webgl2/gzweb_bg.wasm'
        path.write_bytes(b'\0asm\x01\0\0\0' + os.urandom(14_100_000))
        self.spec['viewer_files'] = dict(self.viewer_hashes)
        self.spec['viewer_files'][path.relative_to(self.viewer).as_posix()] = sha(path)
        self.refuse('gzip budget')
        path.write_bytes(b'\0asm\x01\0\0\0')
        self.spec['viewer_files'][path.relative_to(self.viewer).as_posix()] = sha(path)
        self.spec['cargo_metadata'][0]['selected_package_ids'].pop()
        self.refuse('compiler artifact packages')

    def test_unresolved_license_text_does_not_leave_a_published_prefix(self):
        record = self.spec['cargo_metadata'][1]
        metadata = Path(record['path'])
        document = json.loads(metadata.read_text())
        document['packages'][0]['license'] = 'MIT OR Apache-2.0'
        metadata.write_text(json.dumps(document))
        for graph in self.spec['cargo_metadata'][1:]:
            graph['sha256'] = sha(metadata)
        self.refuse('missing actual text')
        self.assertEqual(list(self.directory.glob('.gz-preview-assembly-*')), [])

    def test_existing_outputs_and_archive_are_never_replaced(self):
        self.output.mkdir()
        sentinel = self.output / 'owner-data'
        sentinel.write_bytes(b'preserve')
        with self.assertRaisesRegex(ValueError, 'existing output'):
            package.assemble(self.spec, self.output)
        self.assertEqual(sentinel.read_bytes(), b'preserve')
        sentinel.unlink()
        self.output.rmdir()
        archive = self.directory / 'existing.tar.gz'
        archive.write_bytes(b'keep archive')
        with self.assertRaisesRegex(ValueError, 'existing output'):
            package.assemble(self.spec, self.output, archive)
        self.assertFalse(self.output.exists())
        self.assertEqual(archive.read_bytes(), b'keep archive')


if __name__ == '__main__':
    unittest.main()
