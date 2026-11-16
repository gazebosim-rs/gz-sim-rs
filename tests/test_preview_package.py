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
                     'docs/M29_INSTALL.md', 'docs/M29_CAPABILITIES.md', 'docs/M29_PREVIEW.md',
                     'docs/M32_INSTALL.md', 'docs/M32_CAPABILITIES.md', 'docs/M32_PREVIEW.md',
                     'docs/M37_INSTALL.md', 'docs/M37_CAPABILITIES.md', 'docs/M37_PREVIEW.md',
                     'docs/M40_INSTALL.md', 'docs/M40_CAPABILITIES.md', 'docs/M40_PREVIEW.md',
                     'docs/M44_INSTALL.md', 'docs/M44_CAPABILITIES.md', 'docs/M44_PREVIEW.md',
                     'docs/M60_INSTALL.md', 'docs/M60_CAPABILITIES.md', 'docs/M60_PREVIEW.md',
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
        # Final integration must include Root's exact reviewed M59 fixture.
        # Missing inputs remain hard failures; never fabricate or skip a copy.
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
        for target, crate in [('gz-sim-rs', 'gz-sim-cli'), ('drive_command', 'gz-sim-server'),
                              ('joint_command', 'gz-sim-server')]:
            path = self.directory / target
            header = bytearray(64)
            header[:6] = b'\x7fELF\x02\x01'
            struct.pack_into('<H', header, 18, 62)
            path.write_bytes(header)
            path.chmod(0o755)
            crate_root = self.source / 'crates' / crate
            crate_root.mkdir(parents=True, exist_ok=True)
            (crate_root / 'Cargo.toml').write_text('fixture')
            identity = 'path+file://' + str(crate_root) + '#0.1.0'
            artifacts.append({'reason': 'compiler-artifact', 'executable': str(path),
                              'package_id': identity, 'target': {'name': target}, 'features': [],
                              'profile': {'test': False, 'opt_level': '3'}})
            if not any(entry['id'] == identity for entry in packages):
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
                     'joint_command': self.binary_record('joint_command'),
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
        for key in ['simulator', 'drive_command', 'joint_command']:
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
                    'docs/M44_INSTALL.md': 'INSTALL.md',
                    'docs/M44_CAPABILITIES.md': 'CAPABILITIES.md',
                    'docs/M44_PREVIEW.md': 'PREVIEW.md',
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
        for name in ('M44_INSTALL.md', 'M44_CAPABILITIES.md', 'M44_PREVIEW.md'):
            with self.subTest(guide=name):
                guide = self.source / 'docs' / name
                original = guide.read_bytes()
                guide.unlink()
                self.assertTrue((self.source / 'docs/M12_INSTALL.md').is_file())
                self.assertTrue((self.source / 'docs/M19_INSTALL.md').is_file())
                self.assertTrue((self.source / 'docs/M19_CAPABILITIES.md').is_file())
                self.assertTrue((self.source / 'docs/M20_INSTALL.md').is_file())
                self.assertTrue((self.source / 'docs/M20_CAPABILITIES.md').is_file())
                self.assertTrue((self.source / 'docs/M25_INSTALL.md').is_file())
                self.assertTrue((self.source / 'docs/M29_INSTALL.md').is_file())
                self.assertTrue((self.source / 'docs/M32_INSTALL.md').is_file())
                self.assertTrue((self.source / 'docs/M37_INSTALL.md').is_file())
                self.assertTrue((self.source / 'docs/M40_INSTALL.md').is_file())
                self.refuse('regular')
                self.assertEqual(list(self.directory.glob('.gz-preview-assembly-*')), [])
                guide.write_bytes(original)
        output = package.assemble(self.spec, self.output)
        self.assertIn('INSTALL.md', output['files'])
        self.assertIn('CAPABILITIES.md', output['files'])
        self.assertIn('PREVIEW.md', output['files'])
        self.assertEqual((self.output / 'INSTALL.md').read_bytes(),
                         (self.source / 'docs/M44_INSTALL.md').read_bytes())
        for phase in ('M12', 'M19', 'M20', 'M25', 'M29', 'M32', 'M37', 'M40'):
            self.assertNotEqual((self.output / 'INSTALL.md').read_bytes(),
                                (self.source / ('docs/' + phase + '_INSTALL.md')).read_bytes())

    def test_m44_documented_archive_root_examples_and_installed_launcher_survive_relocation(self):
        for name in ('INSTALL', 'CAPABILITIES', 'PREVIEW'):
            shutil.copyfile(ROOT / ('docs/M44_' + name + '.md'),
                            self.source / ('docs/M44_' + name + '.md'))
        output = self.directory / 'gz-preview-m44-linux-x86_64'
        archive = self.directory / (output.name + '.tar.gz')
        manifest = package.assemble(self.spec, output, archive)
        self.assertEqual((output / 'bin/gz-preview').read_bytes(),
                         (self.source / 'scripts/preview_runtime.py').read_bytes())
        self.assertEqual(manifest['files']['bin/gz-preview']['mode'], 0o755)
        with tarfile.open(archive, 'r:gz') as packed:
            self.assertTrue(all(member.name.startswith(output.name + '/') for member in packed.getmembers()))
            self.assertEqual(packed.extractfile(output.name + '/INSTALL.md').read(),
                             (ROOT / 'docs/M44_INSTALL.md').read_bytes())
            self.assertEqual(packed.extractfile(output.name + '/bin/gz-preview').read(),
                             (self.source / 'scripts/preview_runtime.py').read_bytes())
        moved = self.directory / 'relocated-m44'
        shutil.move(output, moved)
        self.assertEqual(runtime.verify_prefix(moved)['files'], manifest['files'])
        self.assertIn(archive.name, (moved / 'INSTALL.md').read_text())
        self.assertIn('gz-preview-m44-linux-x86_64', (moved / 'INSTALL.md').read_text())
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
        for name in package.NOISY_EXAMPLES[:4]:
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


    def test_real_m29_guides_name_seven_finite_examples_and_preserve_the_default_robot(self):
        preview = (ROOT / 'docs/M29_PREVIEW.md').read_text()
        expected = ('m21_altimeter_noise.sdf', 'm22_air_pressure_noise.sdf',
                    'm23_magnetometer_noise.sdf', 'm24_cpu_lidar_noise.sdf',
                    'm26_air_speed.sdf', 'm27_pose_publisher.sdf', 'm28_air_speed_noise.sdf')
        self.assertEqual(package.NOISY_EXAMPLES[:7], expected)
        for name in expected:
            self.assertIn('"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/' + name + '"', preview)
        self.assertEqual(preview.count('--iterations 2 --step-ns 1000000'), 7)
        self.assertIn('10 ms', preview)
        self.assertIn('PREVIEW.md', (ROOT / 'docs/M29_INSTALL.md').read_text())
        capabilities = (ROOT / 'docs/M29_CAPABILITIES.md').read_text()
        self.assertIn('PosePublisher', capabilities)
        self.assertIn('pre-100', capabilities)
        self.assertIn('default robot', capabilities)

    def test_real_m32_guides_name_nine_examples_and_keep_old_topics_and_runtime(self):
        old = ('m21_altimeter_noise.sdf', 'm22_air_pressure_noise.sdf',
               'm23_magnetometer_noise.sdf', 'm24_cpu_lidar_noise.sdf',
               'm26_air_speed.sdf', 'm27_pose_publisher.sdf', 'm28_air_speed_noise.sdf')
        expected = (*old, 'm30_logical_camera.sdf', 'm31_pose_static_publisher.sdf')
        self.assertEqual(package.NOISY_EXAMPLES[:9], expected)
        preview = (ROOT / 'docs/M32_PREVIEW.md').read_text()
        for name in expected:
            self.assertIn('"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/' + name + '"', preview)
        self.assertEqual(preview.count('--iterations 2 --step-ns 1000000'), 9)
        for topic in ('/m30/logical_camera', '/m30/logical_camera/frustum',
                      '/m31/pose', '/m31/pose_static'):
            self.assertIn(topic, preview)
        self.assertIn('gz-preview-m32-linux-x86_64.tar.gz', (ROOT / 'docs/M32_INSTALL.md').read_text())
        capabilities = (ROOT / 'docs/M32_CAPABILITIES.md').read_text()
        for contract in ('model origins', 'no visibility epsilon', 'PosePublisher',
                         'pre-100', 'default robot', '14,000,000'):
            self.assertIn(contract, capabilities)

    def test_m33_relative_mesh_is_complete_in_archive_and_relocated_verifier(self):
        archive = self.directory / 'm33-mesh.tar.gz'
        manifest = package.assemble(self.spec, self.output, archive)
        moved = self.directory / 'm33 relocated'
        shutil.move(self.output, moved)
        raw = (self.source / 'examples/meshes/m16-asymmetric.obj').read_bytes()
        relative = 'share/examples/meshes/m16-asymmetric.obj'
        self.assertEqual(manifest['files'][relative],
                         {'bytes': len(raw), 'sha256': hashlib.sha256(raw).hexdigest(), 'mode': 0o644})
        self.assertEqual(manifest['files'][relative], manifest['files']['share/meshes/m16-asymmetric.obj'])
        with tarfile.open(archive, 'r:gz') as packed:
            member = packed.getmember(self.output.name + '/' + relative)
            self.assertTrue(member.isfile())
            self.assertEqual(member.mode, 0o644)
            self.assertEqual(packed.extractfile(member).read(), raw)
        example = moved / 'share/examples/m33_robot_sensors.sdf'
        self.assertEqual(example.read_bytes(), (ROOT / 'examples/m33_robot_sensors.sdf').read_bytes())
        self.assertEqual(example.parent / 'meshes/m16-asymmetric.obj', moved / relative)
        mesh = moved / relative
        self.assertEqual(mesh.read_bytes(), raw)
        mesh.unlink()
        with self.assertRaisesRegex(ValueError, 'inventory'):
            runtime.verify_prefix(moved)
        # The default robot's separate copy cannot conceal the missing URI target.
        self.assertEqual((moved / 'share/meshes/m16-asymmetric.obj').read_bytes(), raw)
        mesh.symlink_to(moved / 'share/meshes/m16-asymmetric.obj')
        with self.assertRaises(ValueError):
            runtime.verify_prefix(moved)
        mesh.unlink()
        for changed, mode in [(raw + b'# tampered\n', 0o644), (raw, 0o600)]:
            mesh.write_bytes(changed)
            mesh.chmod(mode)
            with patch.object(runtime, 'serve', return_value=0) as serve:
                self.assertEqual(runtime.main(['run'], moved), 1)
                serve.assert_not_called()
        mesh.write_bytes(raw)
        mesh.chmod(0o644)
        self.assertEqual(runtime.verify_prefix(moved)['files'], manifest['files'])

    def test_real_m40_guides_name_fifteen_examples_and_distinguish_finite_loads(self):
        old = ('m21_altimeter_noise.sdf', 'm22_air_pressure_noise.sdf',
               'm23_magnetometer_noise.sdf', 'm24_cpu_lidar_noise.sdf',
               'm26_air_speed.sdf', 'm27_pose_publisher.sdf', 'm28_air_speed_noise.sdf',
               'm30_logical_camera.sdf', 'm31_pose_static_publisher.sdf')
        expected = (*old, 'm33_robot_sensors.sdf', 'm34_odometry_publisher.sdf',
                    'm35_apply_link_wrench.sdf', 'm36_wrench_odometry.sdf',
                    'm38_odometry_3d.sdf', 'm39_odometry_translation_offsets.sdf')
        self.assertEqual(package.NOISY_EXAMPLES[:len(expected)], expected)
        preview = (ROOT / 'docs/M40_PREVIEW.md').read_text()
        for name in expected:
            self.assertIn('"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/' + name + '"', preview)
        self.assertEqual(preview.count('--iterations 2 --step-ns 1000000'), 15)
        for topic in ('/m33/imu', '/m33/scan', '/m34/odometry',
                      '/world/default/wrench', '/m36/odometry', '/m38/odometry', '/m39/odometry'):
            self.assertIn(topic, preview)
        for limit in ('finite coordinates', '20 ms', 'filtered', 'share/examples/meshes'):
            self.assertIn(limit, preview)
        install = (ROOT / 'docs/M40_INSTALL.md').read_text()
        self.assertIn('gz-preview-m40-linux-x86_64.tar.gz', install)
        self.assertIn('share/examples/meshes', install)
        capabilities = (ROOT / 'docs/M40_CAPABILITIES.md').read_text()
        for contract in ('c40418cd', '14,000,000', 'default robot',
                         'ApplyLinkWrench', 'ground-truth', 'No native Physics'):
            self.assertIn(contract, capabilities)

    def test_relative_mesh_cannot_diverge_from_default_robot_copy_during_assembly(self):
        mesh = self.source / 'examples/meshes/m16-asymmetric.obj'
        original = mesh.read_bytes()
        copy = package.copy_file

        def change_between_mesh_copies(source, destination, mode=0o644, expected=None):
            copy(source, destination, mode, expected)
            if destination.parts[-3:] == ('share', 'meshes', 'm16-asymmetric.obj'):
                mesh.write_bytes(original + b'# changed between copies\n')

        archive = self.directory / 'changed-mesh.tar.gz'
        with patch.object(package, 'copy_file', side_effect=change_between_mesh_copies):
            with self.assertRaisesRegex(ValueError, 'input changed'):
                package.assemble(self.spec, self.output, archive)
        self.assertFalse(self.output.exists())
        self.assertFalse(archive.exists())
        self.assertEqual(list(self.directory.glob('.gz-preview-assembly-*')), [])


    def test_m40_new_odometry_examples_remain_required_after_relocation(self):
        manifest = package.assemble(self.spec, self.output)
        moved = self.directory / 'fifteen-example installation'
        shutil.move(self.output, moved)
        self.assertEqual(runtime.verify_prefix(moved)['files'], manifest['files'])
        for name in ('m38_odometry_3d.sdf', 'm39_odometry_translation_offsets.sdf'):
            target = moved / 'share/examples' / name
            original = (self.source / 'examples' / name).read_bytes()
            self.assertEqual(target.read_bytes(), original)
            self.assertEqual(manifest['files']['share/examples/' + name],
                             {'bytes': len(original), 'sha256': hashlib.sha256(original).hexdigest(), 'mode': 0o644})
            for altered, mode in ((None, 0o644), (original + b'<!-- changed -->', 0o644), (original, 0o600)):
                with self.subTest(example=name, mode=mode, missing=altered is None):
                    if altered is None:
                        target.unlink()
                    else:
                        target.write_bytes(altered)
                        target.chmod(mode)
                    with self.assertRaises(ValueError):
                        runtime.verify_prefix(moved)
                    target.write_bytes(original)
                    target.chmod(0o644)
                    self.assertEqual(runtime.verify_prefix(moved)['files'], manifest['files'])
            # New odometry payloads cannot replace any of the thirteen retained scenes.
            for old in package.NOISY_EXAMPLES[:13]:
                self.assertEqual((moved / 'share/examples' / old).read_bytes(),
                                 (self.source / 'examples' / old).read_bytes())


    def test_joint_helper_requires_its_own_release_artifact_and_notice_closure(self):
        record = self.spec.pop('joint_command')
        self.refuse('joint-command release artifact')
        self.spec['joint_command'] = record
        own = self.artifacts[2]
        for field, value in (('target', {'name': 'drive_command'}),
                             ('package_id', 'registry+https://example.invalid#foreign@1.0.0'),
                             ('features', ['native-backends'])):
            previous = own[field]
            own[field] = value
            self.refresh_transcript()
            self.refuse('exactly one Cargo artifact|native-free')
            own[field] = previous
        self.refresh_transcript()
        # A dependency present only in the joint helper's actual transcript
        # cannot be omitted by reusing the drive helper's notice selection.
        joint_transcript = self.directory / 'joint-cargo.jsonl'
        events = [own, {'reason': 'compiler-artifact', 'executable': None,
                        'package_id': 'registry+https://example.invalid#joint-only@1.0.0'},
                  {'reason': 'build-finished', 'success': True}]
        joint_transcript.write_text(''.join(json.dumps(event) + '\n' for event in events))
        record.update(cargo_messages=str(joint_transcript), cargo_messages_sha256=sha(joint_transcript))
        self.refuse('notices omit native compiler artifact packages')
        self.assertEqual(list(self.directory.glob('.gz-preview-assembly-*')), [])

    def test_joint_helper_is_source_bound_in_archive_and_relocated_prefix(self):
        archive = self.directory / 'three-executables.tar.gz'
        manifest = package.assemble(self.spec, self.output, archive)
        original = self.binaries['joint_command'].read_bytes()
        name = 'bin/gz-joint-command'
        self.assertEqual(manifest['files'][name],
                         {'bytes': len(original), 'sha256': sha(self.binaries['joint_command']), 'mode': 0o755})
        with tarfile.open(archive, 'r:gz') as packed:
            member = packed.getmember(self.output.name + '/' + name)
            self.assertTrue(member.isfile())
            self.assertEqual(member.mode, 0o755)
            self.assertEqual(packed.extractfile(member).read(), original)
        moved = self.directory / 'three executable relocated prefix'
        shutil.move(self.output, moved)
        self.assertEqual(runtime.verify_prefix(moved)['files'], manifest['files'])
        target = moved / name
        for raw, mode in ((original + b'changed', 0o755), (original, 0o644)):
            target.write_bytes(raw)
            target.chmod(mode)
            with self.assertRaisesRegex(ValueError, 'differs'):
                runtime.verify_prefix(moved)
        target.write_bytes(original)
        target.chmod(0o755)
        self.assertEqual(runtime.verify_prefix(moved)['files'], manifest['files'])

    def test_m44_guides_preserve_fifteen_examples_and_add_three_finite_commands(self):
        self.assertEqual(len(package.NOISY_EXAMPLES[:18]), 18)
        self.assertEqual(package.NOISY_EXAMPLES[15:18],
                         ('m41_air_speed_wind.sdf', 'm42_joint_force_feedback.sdf',
                          'm43_imu_reference_frames.sdf'))
        preview = (ROOT / 'docs/M44_PREVIEW.md').read_text()
        for name in package.NOISY_EXAMPLES[:18]:
            self.assertIn('"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/' + name + '"', preview)
        self.assertEqual(preview.count('--iterations 2 --step-ns 1000000'), 18)
        for topic in ('/m42/joint_state', '/m43/imu'):
            self.assertIn(topic, preview)
        install = (ROOT / 'docs/M44_INSTALL.md').read_text()
        self.assertIn('gz-preview-m44-linux-x86_64.tar.gz', install)
        self.assertIn('"$preview_prefix/bin/gz-joint-command" tcp/127.0.0.1:7449 force 1.5 1', install)
        self.assertIn('Zero coasts', install)
        self.assertIn('matching alone does not prove', install)
        for name in ('M40_INSTALL.md', 'M40_CAPABILITIES.md', 'M40_PREVIEW.md'):
            self.assertTrue((ROOT / 'docs' / name).is_file())


    def test_m60_registry_appends_exact_fifteen_without_replacing_the_eighteen_prefix(self):
        prefix = ('m21_altimeter_noise.sdf', 'm22_air_pressure_noise.sdf',
                  'm23_magnetometer_noise.sdf', 'm24_cpu_lidar_noise.sdf',
                  'm26_air_speed.sdf', 'm27_pose_publisher.sdf', 'm28_air_speed_noise.sdf',
                  'm30_logical_camera.sdf', 'm31_pose_static_publisher.sdf',
                  'm33_robot_sensors.sdf', 'm34_odometry_publisher.sdf',
                  'm35_apply_link_wrench.sdf', 'm36_wrench_odometry.sdf',
                  'm38_odometry_3d.sdf', 'm39_odometry_translation_offsets.sdf',
                  'm41_air_speed_wind.sdf', 'm42_joint_force_feedback.sdf',
                  'm43_imu_reference_frames.sdf')
        suffix = ('m45_imu_custom_world.sdf', 'm46_odometry_rotation_offsets.sdf', 'm47_odometry_noise.sdf', 'm48_imu_world_heading.sdf', 'm49_multiple_altimeters.sdf', 'm50_multiple_air_pressures.sdf', 'm51_multiple_air_speeds.sdf', 'm52_multiple_magnetometers.sdf', 'm53_multiple_logical_cameras.sdf', 'm54_multiple_lidars.sdf', 'm55_multiple_pose_publishers.sdf', 'm56_multiple_odometry_publishers.sdf', 'm57_multiple_joint_state_publishers.sdf', 'm58_multiple_joint_controllers.sdf', 'm59_multiple_diff_drives.sdf')
        self.assertEqual(package.NOISY_EXAMPLES, prefix + suffix)
        self.assertEqual(len(package.NOISY_EXAMPLES), 33)
        self.assertEqual(len(set(package.NOISY_EXAMPLES)), 33)
        self.assertTrue(all(Path(name).name == name and name.endswith('.sdf')
                            for name in package.NOISY_EXAMPLES))

    def test_m60_explicit_guides_bind_archive_relocation_and_keep_default_robot_and_meshes(self):
        self.spec['guide_generation'] = 'M60'
        for name in ('INSTALL', 'CAPABILITIES', 'PREVIEW'):
            shutil.copyfile(ROOT / ('docs/M60_' + name + '.md'),
                            self.source / ('docs/M60_' + name + '.md'))
        output = self.directory / 'gz-preview-m60-linux-x86_64'
        archive = self.directory / (output.name + '.tar.gz')
        manifest = package.assemble(self.spec, output, archive)
        self.assertEqual(manifest['guide_generation'], 'M60')
        self.assertEqual((output / 'share/robot.sdf').read_bytes(),
                         (self.source / 'examples/m19_robot.sdf').read_bytes())
        self.assertEqual((output / 'share/meshes/m16-asymmetric.obj').read_bytes(),
                         (output / 'share/examples/meshes/m16-asymmetric.obj').read_bytes())
        for name in ('INSTALL', 'CAPABILITIES', 'PREVIEW'):
            self.assertEqual((output / (name + '.md')).read_bytes(),
                             (ROOT / ('docs/M60_' + name + '.md')).read_bytes())
            self.assertNotEqual((output / (name + '.md')).read_bytes(),
                                (self.source / ('docs/M44_' + name + '.md')).read_bytes())
        with tarfile.open(archive, 'r:gz') as packed:
            for name in package.NOISY_EXAMPLES:
                path = 'share/examples/' + name
                self.assertEqual(packed.extractfile(output.name + '/' + path).read(),
                                 (self.source / 'examples' / name).read_bytes())
                self.assertEqual(manifest['files'][path]['mode'], 0o644)
            self.assertEqual(packed.extractfile(output.name + '/INSTALL.md').read(),
                             (ROOT / 'docs/M60_INSTALL.md').read_bytes())
        moved = self.directory / 'relocated M60 prefix'
        shutil.move(output, moved)
        self.assertEqual(runtime.verify_prefix(moved)['files'], manifest['files'])
        for name in package.NOISY_EXAMPLES:
            self.assertEqual((moved / 'share/examples' / name).read_bytes(),
                             (self.source / 'examples' / name).read_bytes())
        self.assertIn('gz-preview-m60-linux-x86_64.tar.gz', (moved / 'INSTALL.md').read_text())
        self.assertEqual(len(manifest['viewer_gzip_bytes']), 2)
        self.assertTrue(all(value <= 14_000_000 for value in manifest['viewer_gzip_bytes'].values()))

    def test_m60_guide_selection_never_falls_back_and_bad_inputs_publish_nothing(self):
        self.spec['guide_generation'] = 'M60'
        for name in ('M60_INSTALL.md', 'M60_CAPABILITIES.md', 'M60_PREVIEW.md'):
            path = self.source / 'docs' / name
            original = path.read_bytes()
            for fault in ('missing', 'linked', 'oversized'):
                with self.subTest(guide=name, fault=fault):
                    path.unlink()
                    outside = self.directory / 'outside-guide'
                    if fault == 'linked':
                        outside.write_bytes(original)
                        path.symlink_to(outside)
                    elif fault == 'oversized':
                        path.write_bytes(b'x' * (package.MAX_EXAMPLE + 1))
                    self.assertTrue((self.source / 'docs/M44_INSTALL.md').is_file())
                    self.refuse('regular|bound')
                    self.assertEqual(list(self.directory.glob('.gz-preview-assembly-*')), [])
                    if path.exists() or path.is_symlink():
                        path.unlink()
                    if outside.exists():
                        self.assertEqual(outside.read_bytes(), original)
                        outside.unlink()
                    path.write_bytes(original)
        docs = self.source / 'docs'
        saved = self.source / 'real-docs'
        docs.rename(saved)
        docs.symlink_to(saved, target_is_directory=True)
        try:
            self.refuse('linked directories')
            self.assertEqual(list(self.directory.glob('.gz-preview-assembly-*')), [])
        finally:
            docs.unlink()
            saved.rename(docs)

    def test_guide_generation_is_closed_and_legacy_default_equals_explicit_m44(self):
        for generation in (None, '', 'M59', 'm60', '../M60', '/tmp/M60', False, 60, [], {}):
            with self.subTest(generation=generation):
                self.spec['guide_generation'] = generation
                self.refuse('guide_generation')
        self.spec.pop('guide_generation')
        default = package.assemble(self.spec, self.output)
        self.spec['guide_generation'] = 'M44'
        explicit = package.assemble(self.spec, self.directory / 'explicit-m44')
        self.assertEqual(default, explicit)
        self.assertEqual(default['guide_generation'], 'M44')
        self.assertEqual((self.output / 'INSTALL.md').read_bytes(),
                         (self.source / 'docs/M44_INSTALL.md').read_bytes())

    def test_m60_guides_document_all_thirty_three_and_explicit_commands_without_pass_claims(self):
        preview = (ROOT / 'docs/M60_PREVIEW.md').read_text()
        for name in package.NOISY_EXAMPLES:
            self.assertIn('"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/' + name + '"', preview)
        self.assertEqual(preview.count('--iterations 2 --step-ns 1000000'), 33)
        install = (ROOT / 'docs/M60_INSTALL.md').read_text()
        self.assertIn('run --example m59_multiple_diff_drives.sdf', install)
        self.assertIn('drive --topic /m59/left/cmd_vel', install)
        self.assertIn('drive --topic /m59/right/cmd_vel', install)
        self.assertIn('M40 remains Latest', install)
        self.assertIn('No M60 installed or browser result is claimed', install)
        for phase in ('M40', 'M44'):
            for kind in ('INSTALL', 'CAPABILITIES', 'PREVIEW'):
                self.assertTrue((ROOT / ('docs/' + phase + '_' + kind + '.md')).is_file())

    def m65_inputs(self):
        self.spec['guide_generation'] = 'M65'
        for kind in ('INSTALL', 'CAPABILITIES', 'PREVIEW'):
            shutil.copyfile(ROOT / ('docs/M65_' + kind + '.md'),
                            self.source / ('docs/M65_' + kind + '.md'))
        for name in package.M65_FLEET_EXAMPLES:
            shutil.copyfile(ROOT / 'examples' / name, self.source / 'examples' / name)
        return package.NOISY_EXAMPLES + package.M65_FLEET_EXAMPLES

    def test_m65_generation_alone_selects_the_exact_thirty_five_example_prefix(self):
        self.assertEqual(package.M65_FLEET_EXAMPLES,
                         ('m61_eight_diff_drives.sdf', 'm62_eight_joint_controllers.sdf'))
        self.assertEqual(len(package.NOISY_EXAMPLES), 33)
        selected = package.NOISY_EXAMPLES + package.M65_FLEET_EXAMPLES
        self.assertEqual(len(selected), len(set(selected)))
        self.assertEqual(len(selected), 35)
        for generation in ('M44', 'M60'):
            self.spec['guide_generation'] = generation
            output = self.directory / ('legacy-' + generation)
            manifest = package.assemble(self.spec, output)
            sdf = {name for name in manifest['files']
                   if name.startswith('share/examples/') and name.endswith('.sdf')}
            self.assertEqual(sdf, {'share/examples/' + name for name in package.NOISY_EXAMPLES})
            for name in package.M65_FLEET_EXAMPLES:
                self.assertFalse((output / 'share/examples' / name).exists())
        # Old inputs and guides cannot stand in for the explicit fleet generation.
        self.spec['guide_generation'] = 'M65'
        self.refuse('regular')
        self.assertEqual(list(self.directory.glob('.gz-preview-assembly-*')), [])

    def test_m65_real_fleet_copies_guides_archive_and_relocation_are_byte_bound(self):
        selected = self.m65_inputs()
        archive = self.directory / 'm65.tar.gz'
        originals = {name: (ROOT / 'examples' / name).read_bytes() for name in selected}
        manifest = package.assemble(self.spec, self.output, archive)
        self.assertEqual(manifest['guide_generation'], 'M65')
        expected = {'share/examples/' + name for name in selected}
        self.assertEqual({name for name in manifest['files']
                          if name.startswith('share/examples/') and name.endswith('.sdf')}, expected)
        moved = self.directory / 'relocated fleet prefix'
        shutil.move(self.output, moved)
        self.assertEqual(runtime.verify_prefix(moved), manifest)
        with tarfile.open(archive, 'r:gz') as stream:
            for name, original in originals.items():
                relative = 'share/examples/' + name
                self.assertEqual((moved / relative).read_bytes(), original)
                self.assertEqual(manifest['files'][relative],
                                 {'bytes': len(original), 'sha256': hashlib.sha256(original).hexdigest(),
                                  'mode': 0o644})
                member = stream.getmember(self.output.name + '/' + relative)
                self.assertTrue(member.isfile())
                self.assertEqual(member.mode, 0o644)
                self.assertEqual(stream.extractfile(member).read(), original)
            for kind in ('INSTALL', 'CAPABILITIES', 'PREVIEW'):
                self.assertEqual((moved / (kind + '.md')).read_bytes(),
                                 (ROOT / ('docs/M65_' + kind + '.md')).read_bytes())
            for name in ('share/meshes/m16-asymmetric.obj',
                         'share/examples/meshes/m16-asymmetric.obj'):
                self.assertEqual((moved / name).read_bytes(),
                                 (self.source / 'examples/meshes/m16-asymmetric.obj').read_bytes())
        self.assertEqual((moved / 'share/robot.sdf').read_bytes(),
                         (self.source / 'examples/m19_robot.sdf').read_bytes())
        for name in package.M65_FLEET_EXAMPLES:
            with patch.object(runtime, 'serve', return_value=0) as serve:
                self.assertEqual(runtime.main(['run', '--example', name], moved), 0)
                self.assertEqual(serve.call_args.kwargs['example'], name)

    def test_m65_fleet_sources_are_required_ordinary_and_bounded_before_publication(self):
        self.m65_inputs()
        archive = self.directory / 'invalid-fleet.tar.gz'
        for name in package.M65_FLEET_EXAMPLES:
            source = self.source / 'examples' / name
            original = source.read_bytes()
            for fault in ('missing', 'linked', 'oversized'):
                with self.subTest(example=name, fault=fault):
                    outside = self.directory / 'outside-fleet.sdf'
                    source.unlink()
                    if fault == 'linked':
                        outside.write_bytes(original)
                        source.symlink_to(outside)
                    elif fault == 'oversized':
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
                        if outside.exists():
                            self.assertEqual(outside.read_bytes(), original)
                            outside.unlink()
        examples = self.source / 'examples'
        saved = self.source / 'real-examples'
        examples.rename(saved)
        examples.symlink_to(saved, target_is_directory=True)
        try:
            self.refuse('linked directories')
        finally:
            examples.unlink()
            saved.rename(examples)

    def test_m65_installed_fleet_tampering_never_reaches_the_launcher(self):
        self.m65_inputs()
        manifest = package.assemble(self.spec, self.output)
        moved = self.directory / 'fleet relocated'
        shutil.move(self.output, moved)
        for name in package.M65_FLEET_EXAMPLES:
            path = moved / 'share/examples' / name
            original = path.read_bytes()
            for fault in ('missing', 'edited', 'mode', 'linked'):
                with self.subTest(example=name, fault=fault):
                    path.unlink()
                    outside = self.directory / 'external-installed-fleet.sdf'
                    if fault == 'linked':
                        outside.write_bytes(original)
                        path.symlink_to(outside)
                    elif fault != 'missing':
                        path.write_bytes(original + (b'<!-- changed -->' if fault == 'edited' else b''))
                        path.chmod(0o600 if fault == 'mode' else 0o644)
                    try:
                        with self.assertRaises(ValueError):
                            runtime.verify_prefix(moved)
                        with patch.object(runtime, 'serve', return_value=0) as serve:
                            self.assertEqual(runtime.main(['run', '--example', name], moved), 1)
                            serve.assert_not_called()
                    finally:
                        if path.exists() or path.is_symlink():
                            path.unlink()
                        path.write_bytes(original)
                        path.chmod(0o644)
                        if outside.exists():
                            self.assertEqual(outside.read_bytes(), original)
                            outside.unlink()
            self.assertEqual(runtime.verify_prefix(moved), manifest)
        with patch.object(runtime, 'serve', return_value=0) as serve:
            self.assertEqual(runtime.main(['run', '--example', '../m61_eight_diff_drives.sdf'], moved), 1)
            self.assertEqual(runtime.main(['run', '--example', 'm63_two_rgb_cameras.sdf'], moved), 1)
            serve.assert_not_called()

    def test_m65_guides_are_closed_required_bounded_and_never_fall_back(self):
        self.m65_inputs()
        for kind in ('INSTALL', 'CAPABILITIES', 'PREVIEW'):
            path = self.source / ('docs/M65_' + kind + '.md')
            original = path.read_bytes()
            for fault in ('missing', 'linked', 'oversized'):
                with self.subTest(guide=kind, fault=fault):
                    path.unlink()
                    outside = self.directory / 'external-guide.md'
                    if fault == 'linked':
                        outside.write_bytes(original)
                        path.symlink_to(outside)
                    elif fault == 'oversized':
                        path.write_bytes(b'x' * (package.MAX_EXAMPLE + 1))
                    try:
                        self.refuse('regular|bound')
                        self.assertTrue((self.source / ('docs/M60_' + kind + '.md')).is_file())
                        self.assertTrue((self.source / ('docs/M44_' + kind + '.md')).is_file())
                        self.assertEqual(list(self.directory.glob('.gz-preview-assembly-*')), [])
                    finally:
                        if path.exists() or path.is_symlink():
                            path.unlink()
                        path.write_bytes(original)
                        if outside.exists():
                            self.assertEqual(outside.read_bytes(), original)
                            outside.unlink()
        docs = self.source / 'docs'
        saved = self.source / 'real-docs'
        docs.rename(saved)
        docs.symlink_to(saved, target_is_directory=True)
        try:
            self.refuse('linked directories')
        finally:
            docs.unlink()
            saved.rename(docs)
        for value in ('m65', '../M65', '/tmp/M65', 'M65_INSTALL.md'):
            with self.subTest(generation=value):
                self.spec['guide_generation'] = value
                self.refuse('guide_generation')

    def test_m65_guides_separate_finite_loads_direct_helper_and_unexecuted_release(self):
        preview = (ROOT / 'docs/M65_PREVIEW.md').read_text()
        selected = package.NOISY_EXAMPLES + package.M65_FLEET_EXAMPLES
        for name in selected:
            self.assertIn('"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/' + name + '"', preview)
        self.assertEqual(preview.count('--iterations 2 --step-ns 1000000'), 35)
        install = (ROOT / 'docs/M65_INSTALL.md').read_text()
        for text in ('No M65 installed or', 'browser result is claimed',
                     'export GZ_PARTITION=gz-preview-fleet', '--partition "$GZ_PARTITION"',
                     'tcp/127.0.0.1:7449 position 0.4 0.2 --topic',
                     'tcp/127.0.0.1:7449 velocity 0.3 0.2 --topic',
                     '/model/arm_${owner}/joint/base_hinge/0/cmd_pos',
                     '/model/arm_${owner}/joint/tip_hinge/cmd_vel'):
            self.assertIn(text, install)
        for name in ('m61_eight_diff_drives.sdf', 'm62_eight_joint_controllers.sdf'):
            self.assertIn(
                '"$preview_prefix/bin/gz-preview" --partition "$GZ_PARTITION" run --example '
                + name + ' --http-port 8082 --ws-port 9014 --lifecycle', install)
        capabilities = (ROOT / 'docs/M65_CAPABILITIES.md').read_text()
        self.assertIn('M44/M60 package generations keep their original guide selection', capabilities)
        self.assertIn('There is no RGB/depth worker', capabilities)
        self.assertIn('35 installed', capabilities)
        self.assertIn('37 examples', capabilities)



if __name__ == '__main__':
    unittest.main()
