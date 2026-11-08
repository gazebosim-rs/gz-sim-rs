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
                     'examples/m19_robot.sdf', 'examples/meshes/m16-asymmetric.obj',
                     'docs/M12_INSTALL.md', 'docs/M12_CAPABILITIES.md',
                     'docs/M19_INSTALL.md', 'docs/M19_CAPABILITIES.md',
                     'docs/M20_INSTALL.md', 'docs/M20_CAPABILITIES.md',
                     'docs/M25_INSTALL.md', 'docs/M25_CAPABILITIES.md', 'docs/M25_PREVIEW.md',
                     *['examples/' + name for name in package.NOISY_EXAMPLES], 'LICENSE', 'NOTICE']:
            path = self.source / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('source fixture ' + name)
        (self.source / 'examples/meshes/m16-asymmetric.obj').write_bytes(
            b'o package_mesh\nv 0 0 0\nv 1 0 0\nv 0 2 0\nf 1 2 3\n')
        (self.source / 'examples/m19_robot.sdf').write_text(
            '<sdf version="1.12"><world name="demo"><model name="robot"><link name="body">'
            '<visual name="cargo"><geometry><mesh><uri>meshes/m16-asymmetric.obj</uri>'
            '</mesh></geometry></visual></link></model></world></sdf>')
        shutil.copyfile(ROOT / 'scripts/preview_runtime.py', self.source / 'scripts/preview_runtime.py')
        for name in package.NOISY_EXAMPLES:
            shutil.copyfile(ROOT / 'examples' / name, self.source / 'examples' / name)
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
        selected = {'examples/m19_robot.sdf': 'share/robot.sdf',
                    'examples/meshes/m16-asymmetric.obj': 'share/meshes/m16-asymmetric.obj',
                    'docs/M25_INSTALL.md': 'INSTALL.md',
                    'docs/M25_CAPABILITIES.md': 'CAPABILITIES.md',
                    'docs/M25_PREVIEW.md': 'PREVIEW.md',
                    **{'examples/' + name: 'share/examples/' + name for name in package.NOISY_EXAMPLES}}
        for source, installed in selected.items():
            with self.subTest(installed=installed):
                raw = (self.source / source).read_bytes()
                self.assertEqual((self.output / installed).read_bytes(), raw)
                self.assertEqual(manifest['files'][installed],
                                 {'bytes': len(raw), 'sha256': hashlib.sha256(raw).hexdigest(), 'mode': 0o644})
        self.assertNotIn('M12_INSTALL.md', manifest['files'])
        self.assertNotIn('M12_CAPABILITIES.md', manifest['files'])
        self.assertNotEqual((self.output / 'share/robot.sdf').read_bytes(),
                            (self.source / 'examples/m8_diff_drive.sdf').read_bytes())
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
            self.assertEqual(packed.extractfile('preview-v1/share/meshes/m16-asymmetric.obj').read(),
                             (self.source / 'examples/meshes/m16-asymmetric.obj').read_bytes())
            for name in package.NOISY_EXAMPLES:
                member = packed.getmember('preview-v1/share/examples/' + name)
                self.assertTrue(member.isfile())
                self.assertEqual(member.mode, 0o644)
                self.assertEqual(packed.extractfile(member).read(), (self.source / 'examples' / name).read_bytes())
        for source, installed in selected.items():
            self.assertEqual((moved / installed).read_bytes(), (self.source / source).read_bytes())

    def test_missing_linked_and_oversized_fixed_mesh_never_publish_outputs(self):
        mesh = self.source / 'examples/meshes/m16-asymmetric.obj'
        original = mesh.read_bytes()
        archive = self.directory / 'missing-mesh.tar.gz'
        for variant in ('missing', 'linked-file', 'linked-directory', 'oversized'):
            with self.subTest(variant=variant):
                outside = self.directory / 'external-mesh.obj'
                outside_directory = self.directory / 'external-meshes'
                if variant == 'linked-directory':
                    mesh.parent.rename(outside_directory)
                    mesh.parent.symlink_to(outside_directory, target_is_directory=True)
                else:
                    mesh.unlink()
                    if variant == 'linked-file':
                        outside.write_bytes(original)
                        mesh.symlink_to(outside)
                    elif variant == 'oversized':
                        mesh.write_bytes(b'x' * (16 * 1024 * 1024 + 1))
                try:
                    with self.assertRaisesRegex(ValueError, 'regular|linked directories|file bound'):
                        package.assemble(self.spec, self.output, archive)
                    self.assertFalse(self.output.exists())
                    self.assertFalse(archive.exists())
                    self.assertEqual(list(self.directory.glob('.gz-preview-assembly-*')), [])
                finally:
                    if variant == 'linked-directory':
                        mesh.parent.unlink()
                        outside_directory.rename(mesh.parent)
                    else:
                        if mesh.exists() or mesh.is_symlink():
                            mesh.unlink()
                        mesh.write_bytes(original)
                    if outside.exists():
                        self.assertEqual(outside.read_bytes(), original)
                        outside.unlink()
                self.assertEqual(mesh.read_bytes(), original)

    def test_dedicated_guides_are_required_even_when_legacy_sources_exist(self):
        for name in ('M25_INSTALL.md', 'M25_CAPABILITIES.md', 'M25_PREVIEW.md'):
            with self.subTest(guide=name):
                guide = self.source / 'docs' / name
                original = guide.read_bytes()
                guide.unlink()
                self.assertTrue((self.source / 'docs/M12_INSTALL.md').is_file())
                self.assertTrue((self.source / 'docs/M19_INSTALL.md').is_file())
                self.assertTrue((self.source / 'docs/M19_CAPABILITIES.md').is_file())
                self.assertTrue((self.source / 'docs/M20_INSTALL.md').is_file())
                self.assertTrue((self.source / 'docs/M20_CAPABILITIES.md').is_file())
                self.refuse('regular')
                self.assertEqual(list(self.directory.glob('.gz-preview-assembly-*')), [])
                guide.write_bytes(original)
        output = package.assemble(self.spec, self.output)
        self.assertIn('INSTALL.md', output['files'])
        self.assertIn('CAPABILITIES.md', output['files'])
        self.assertIn('PREVIEW.md', output['files'])
        self.assertEqual((self.output / 'INSTALL.md').read_bytes(),
                         (self.source / 'docs/M25_INSTALL.md').read_bytes())
        for phase in ('M12', 'M19', 'M20'):
            self.assertNotEqual((self.output / 'INSTALL.md').read_bytes(),
                                (self.source / ('docs/' + phase + '_INSTALL.md')).read_bytes())

    def test_m25_documented_archive_root_examples_and_installed_launcher_survive_relocation(self):
        for name in ('INSTALL', 'CAPABILITIES', 'PREVIEW'):
            shutil.copyfile(ROOT / ('docs/M25_' + name + '.md'),
                            self.source / ('docs/M25_' + name + '.md'))
        output = self.directory / 'gz-preview-m25-linux-x86_64'
        archive = self.directory / (output.name + '.tar.gz')
        manifest = package.assemble(self.spec, output, archive)
        self.assertEqual((output / 'bin/gz-preview').read_bytes(),
                         (self.source / 'scripts/preview_runtime.py').read_bytes())
        self.assertEqual(manifest['files']['bin/gz-preview']['mode'], 0o755)
        with tarfile.open(archive, 'r:gz') as packed:
            self.assertTrue(all(member.name.startswith(output.name + '/') for member in packed.getmembers()))
            self.assertEqual(packed.extractfile(output.name + '/INSTALL.md').read(),
                             (ROOT / 'docs/M25_INSTALL.md').read_bytes())
            self.assertEqual(packed.extractfile(output.name + '/bin/gz-preview').read(),
                             (self.source / 'scripts/preview_runtime.py').read_bytes())
        moved = self.directory / 'relocated-m25'
        shutil.move(output, moved)
        self.assertEqual(runtime.verify_prefix(moved)['files'], manifest['files'])
        self.assertIn(archive.name, (moved / 'INSTALL.md').read_text())
        self.assertIn('gz-preview-m25-linux-x86_64', (moved / 'INSTALL.md').read_text())
        with patch.object(runtime, 'serve', return_value=0) as serve:
            self.assertEqual(runtime.main(['run', '--lifecycle', '--http-port', '8082'], moved), 0)
            serve.assert_called_once_with(moved, 8082, 9014, 'gz-preview', lifecycle=True)

    def test_fixed_noisy_examples_are_bounded_ordinary_inputs_and_missing_sources_are_atomic(self):
        archive = self.directory / 'examples.tar.gz'
        for name in package.NOISY_EXAMPLES:
            source = self.source / 'examples' / name
            original = source.read_bytes()
            original_mode = source.stat().st_mode & 0o777
            for variant in ('missing', 'linked', 'oversized'):
                with self.subTest(example=name, variant=variant):
                    outside = self.directory / 'external-example.sdf'
                    source.unlink()
                    if variant == 'linked':
                        outside.write_bytes(original)
                        source.symlink_to(outside)
                    elif variant == 'oversized':
                        source.write_bytes(b'x' * (package.MAX_EXAMPLE + 1))
                    try:
                        with self.assertRaisesRegex(ValueError, 'regular|file bound'):
                            package.assemble(self.spec, self.output, archive)
                        self.assertFalse(self.output.exists())
                        self.assertFalse(archive.exists())
                        self.assertEqual(list(self.directory.glob('.gz-preview-assembly-*')), [])
                    finally:
                        if source.exists() or source.is_symlink():
                            source.unlink()
                        source.write_bytes(original)
                        source.chmod(original_mode)
                        if outside.exists():
                            self.assertEqual(outside.read_bytes(), original)
                            outside.unlink()
            self.assertEqual(source.read_bytes(), original)

    def test_each_noisy_example_stays_byte_bound_after_relocation_and_refuses_tamper_before_run(self):
        before = {name: (self.source / 'examples' / name).read_bytes() for name in package.NOISY_EXAMPLES}
        manifest = package.assemble(self.spec, self.output)
        moved = self.directory / 'relocated noisy examples'
        shutil.move(self.output, moved)
        for name, original in before.items():
            with self.subTest(example=name):
                source = self.source / 'examples' / name
                installed = moved / 'share/examples' / name
                self.assertEqual(source.read_bytes(), original)
                self.assertEqual(installed.read_bytes(), original)
                self.assertEqual(installed.stat().st_mode & 0o777, 0o644)
                self.assertEqual(manifest['files']['share/examples/' + name],
                                 {'bytes': len(original), 'sha256': hashlib.sha256(original).hexdigest(), 'mode': 0o644})
                self.assertFalse((moved / 'web' / name).exists())
                installed.unlink()
                with self.assertRaisesRegex(ValueError, 'inventory'):
                    runtime.verify_prefix(moved)
                installed.write_bytes(original + b'<!-- edited -->')
                installed.chmod(0o644)
                with patch.object(runtime, 'serve', return_value=0) as serve:
                    self.assertEqual(runtime.main(['run'], moved), 1)
                    serve.assert_not_called()
                installed.write_bytes(original)
                installed.chmod(0o600)
                with self.assertRaisesRegex(ValueError, 'differs'):
                    runtime.verify_prefix(moved)
                installed.chmod(0o644)
                self.assertEqual(runtime.verify_prefix(moved)['files'], manifest['files'])
        self.assertEqual({name: (self.source / 'examples' / name).read_bytes() for name in before}, before)

    def test_real_m25_preview_guides_name_all_four_finite_installed_examples(self):
        preview = (ROOT / 'docs/M25_PREVIEW.md').read_text()
        for name in package.NOISY_EXAMPLES:
            self.assertIn('"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/' + name + '"', preview)
        self.assertEqual(preview.count('--iterations 2 --step-ns 1000000'), 4)
        self.assertIn('PREVIEW.md', (ROOT / 'docs/M25_INSTALL.md').read_text())
        self.assertIn('Gaussian', (ROOT / 'docs/M25_CAPABILITIES.md').read_text())

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
