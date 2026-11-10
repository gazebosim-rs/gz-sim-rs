#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Assemble an explicitly source-bound M40 Linux preview, without building inputs."""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import struct
import tarfile
import tempfile

from preview_licenses import collect_notices

ROOT = Path(__file__).resolve().parents[1]
VIEWER_FILES = ('index.html', 'startup.mjs', *(
    'target/wasm32-unknown-unknown/release/' + flavor + '/' + name
    for flavor in ('webgpu', 'webgl2') for name in ('gzweb.js', 'gzweb_bg.wasm')))
COMMIT = re.compile(r'^[0-9a-f]{40}$')
DIGEST = re.compile(r'^[0-9a-f]{64}$')
MAX_FILE = 256 * 1024 * 1024
MAX_EXAMPLE = 1024 * 1024
NOISY_EXAMPLES = (
    'm21_altimeter_noise.sdf', 'm22_air_pressure_noise.sdf',
    'm23_magnetometer_noise.sdf', 'm24_cpu_lidar_noise.sdf',
    'm26_air_speed.sdf', 'm27_pose_publisher.sdf', 'm28_air_speed_noise.sdf',
    'm30_logical_camera.sdf', 'm31_pose_static_publisher.sdf',
    'm33_robot_sensors.sdf', 'm34_odometry_publisher.sdf',
    'm35_apply_link_wrench.sdf', 'm36_wrench_odometry.sdf',
    'm38_odometry_3d.sdf', 'm39_odometry_translation_offsets.sdf')


def regular(path, maximum=MAX_FILE):
    path = Path(path)
    if not path.is_absolute() or path.is_symlink() or not path.is_file():
        raise ValueError('an absolute regular input file is required: ' + str(path))
    if path.stat().st_size > maximum:
        raise ValueError('input exceeds the package file bound: ' + str(path))
    return path


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def bound_file(path, expected, maximum=MAX_FILE):
    path = regular(path, maximum)
    if not isinstance(expected, str) or not DIGEST.fullmatch(expected) or digest(path) != expected:
        raise ValueError('input hash does not match its build record: ' + str(path))
    return path


def json_file(path):
    path = regular(path, 64 * 1024 * 1024)
    return json.loads(path.read_text())


def binary_input(record, target, package):
    path = bound_file(record['path'], record['sha256'])
    with path.open('rb') as stream:
        header = stream.read(64)
    if (len(header) < 64 or header[:6] != b'\x7fELF\x02\x01'
            or struct.unpack_from('<H', header, 18)[0] != 62
            or not path.stat().st_mode & 0o111):
        raise ValueError('an executable Linux x86_64 ELF input is required')
    transcript = bound_file(record['cargo_messages'], record['cargo_messages_sha256'], 64 * 1024 * 1024)
    artifacts = []
    messages = []
    for line in transcript.read_text().splitlines():
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            continue
        messages.append(event)
        if event.get('reason') == 'compiler-artifact':
            artifacts.append(event)
    finished = [event for event in messages if event.get('reason') == 'build-finished']
    if (len(finished) != 1 or finished[0].get('success') is not True
            or not messages or messages[-1] is not finished[0]):
        raise ValueError('exactly one terminal successful build-finished record is required')
    matching = [event for event in artifacts if event.get('executable') is not None
                and Path(event['executable']).resolve() == path.resolve()
                and event.get('target', {}).get('name') == target]
    if len(matching) != 1:
        raise ValueError('the selected binary must have exactly one Cargo artifact record')
    event = matching[0]
    if (not ('#' + package + '@' in event['package_id'] or '/' + package + '#' in event['package_id'])
            or event.get('features') != []
            or event.get('profile', {}).get('test') is not False
            or event.get('profile', {}).get('opt_level') != '3'):
        raise ValueError('a native-free, GPU-free release artifact is required')
    return path, {event['package_id'] for event in artifacts}


def output_path(value):
    path = Path(value).absolute()
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,127}', path.name):
        raise ValueError('package output needs a simple directory or archive name')
    if path.exists() or path.is_symlink():
        raise ValueError('refusing to replace existing output: ' + str(path))
    if not path.parent.is_dir() or path.parent.is_symlink():
        raise ValueError('output parent must be an existing real directory')
    return path


def copy_file(source, destination, mode=0o644, expected=None):
    expected = digest(source) if expected is None else expected
    destination.parent.mkdir(parents=True, exist_ok=True)
    with Path(source).open('rb') as incoming, destination.open('xb') as outgoing:
        shutil.copyfileobj(incoming, outgoing)
        outgoing.flush()
        os.fsync(outgoing.fileno())
    destination.chmod(mode)
    if digest(source) != expected or digest(destination) != expected:
        raise ValueError('input changed while copying: ' + str(source))


def archive_prefix(prefix, destination):
    # No absolute names, links, host identity or development timestamps in the archive.
    with destination.open('xb') as output:
        with gzip.GzipFile(fileobj=output, mode='wb', filename='', mtime=0) as compressed:
            with tarfile.open(fileobj=compressed, mode='w|', format=tarfile.PAX_FORMAT) as archive:
                for path in sorted(prefix.rglob('*')):
                    if path.is_symlink() or not (path.is_dir() or path.is_file()):
                        raise ValueError('only ordinary package files can be archived')
                    member = archive.gettarinfo(str(path), arcname=prefix.name + '/' + path.relative_to(prefix).as_posix())
                    member.uid = member.gid = 0
                    member.uname = member.gname = ''
                    member.mtime = 0
                    if path.is_file():
                        with path.open('rb') as stream:
                            archive.addfile(member, stream)
                    else:
                        archive.addfile(member)
        output.flush()
        os.fsync(output.fileno())


def assemble(spec, output, archive=None):
    output = output_path(output)
    archive = output_path(archive) if archive is not None else None
    if archive is not None and (archive == output or archive.is_relative_to(output)):
        raise ValueError('the archive must be outside the installed prefix')
    if spec.get('schema_version') != 1 or spec.get('target') != 'ubuntu-24.04-x86_64':
        raise ValueError('only the admitted Ubuntu 24.04 x86_64 package is supported')
    repositories = spec['repositories']
    if set(repositories) != {'gz-sim-rs', 'gzweb-rs'} or not all(
            isinstance(value, str) and COMMIT.fullmatch(value) for value in repositories.values()):
        raise ValueError('exact simulator and viewer commit identities are required')
    simulator, native_ids = binary_input(spec['simulator'], 'gz-sim-rs', 'gz-sim-cli')
    sender, helper_ids = binary_input(spec['drive_command'], 'drive_command', 'gz-sim-server')
    viewer_root = Path(spec['viewer_root'])
    if not viewer_root.is_absolute() or not viewer_root.is_dir() or viewer_root.is_symlink():
        raise ValueError('an absolute accepted viewer bundle directory is required')
    if set(spec['viewer_files']) != set(VIEWER_FILES):
        raise ValueError('exactly the six maintained viewer files are required')
    viewer = {}
    budgets = {}
    for name in VIEWER_FILES:
        path = bound_file(viewer_root / name, spec['viewer_files'][name], 64 * 1024 * 1024)
        if not path.resolve().is_relative_to(viewer_root.resolve()):
            raise ValueError('viewer input escapes its bundle')
        if name.endswith('.wasm'):
            with path.open('rb') as stream:
                if stream.read(8) != b'\0asm\x01\0\0\0':
                    raise ValueError('viewer input is not an admitted Wasm module')
            size = len(gzip.compress(path.read_bytes(), mtime=0))
            if size > 14_000_000:
                raise ValueError('viewer flavor exceeds its unchanged gzip budget')
            budgets[name] = size
        viewer[name] = path
    metadata = spec['cargo_metadata']
    if not isinstance(metadata, list) or not 1 <= len(metadata) <= 4:
        raise ValueError('bounded native/helper and viewer Cargo metadata inputs are required')
    documents = []
    admitted_ids = set()
    configurations = set()
    for record in metadata:
        document = json_file(bound_file(record['path'], record['sha256'], 64 * 1024 * 1024))
        configuration = record.get('configuration')
        if configuration not in {'native', 'viewer-webgpu', 'viewer-webgl2'} or configuration in configurations:
            raise ValueError('unique native and both viewer metadata configurations are required')
        configurations.add(configuration)
        if (record.get('repository') == 'gz-sim-rs') != (configuration == 'native'):
            raise ValueError('metadata configuration and repository disagree')
        if record.get('repository') not in repositories:
            raise ValueError('unknown metadata repository')
        if record.get('commit') != repositories[record['repository']]:
            raise ValueError('metadata commit does not match the packaged repository')
        selected = record['selected_package_ids']
        if not isinstance(selected, list) or not selected or len(selected) != len(set(selected)):
            raise ValueError('unique selected Cargo package identities are required')
        by_id = {package['id']: package for package in document['packages']}
        if not set(selected) <= by_id.keys():
            raise ValueError('selected package is missing from Cargo metadata')
        if record['repository'] == 'gz-sim-rs':
            admitted_ids.update(selected)
        documents.append({'label': configuration, 'workspace_root': document['workspace_root'],
                          'packages': [by_id[identity] for identity in selected]})
    if configurations != {'native', 'viewer-webgpu', 'viewer-webgl2'}:
        raise ValueError('native and both accepted viewer feature graphs are required')
    if not (native_ids | helper_ids) <= admitted_ids:
        raise ValueError('notices omit native compiler artifact packages')
    extras = {name: regular(path, 1024 * 1024) for name, path in spec['extra_notices'].items()}
    if not {'licenses/Apache-2.0.txt', 'OSRF-NOTICE.txt', 'Geist-OFL.txt',
            'Rust-COPYRIGHT.html', 'Rust-LICENSE-APACHE.txt', 'Rust-LICENSE-MIT.txt',
            'Viewer-mesh-loader-LICENSE.txt', 'Viewer-proto-LICENSE.txt'} <= extras.keys():
        raise ValueError('project, embedded font and Rust standard-library notices are required')
    # This fixed fixture's only local mesh must be an ordinary bounded source
    # file. Reject linked parent directories too, not just a symlink at the leaf.
    mesh_name = Path('examples/meshes/m16-asymmetric.obj')
    mesh = regular(ROOT / mesh_name, 16 * 1024 * 1024)
    if mesh.resolve() != ROOT.resolve() / mesh_name:
        raise ValueError('the fixed mesh source may not traverse linked directories')
    examples = {}
    for name in NOISY_EXAMPLES:
        relative = Path('examples') / name
        source = regular(ROOT / relative, MAX_EXAMPLE)
        if source.resolve() != ROOT.resolve() / relative:
            raise ValueError('fixed example sources may not traverse linked directories')
        examples[name] = source
    mesh_sha = digest(mesh)
    with tempfile.TemporaryDirectory(prefix='.gz-preview-assembly-', dir=output.parent) as temporary:
        prefix = Path(temporary) / 'prefix'
        prefix.mkdir()
        copy_file(simulator, prefix / 'bin/gz-sim-rs', 0o755, spec['simulator']['sha256'])
        copy_file(sender, prefix / 'bin/gz-drive-command', 0o755, spec['drive_command']['sha256'])
        copy_file(regular(ROOT / 'scripts/preview_runtime.py'), prefix / 'bin/gz-preview', 0o755)
        copy_file(regular(ROOT / 'examples/m19_robot.sdf'), prefix / 'share/robot.sdf')
        copy_file(mesh, prefix / 'share/meshes/m16-asymmetric.obj', expected=mesh_sha)
        # M33's unchanged relative URI resolves from share/examples, while the
        # default robot retains its original share/meshes location.
        copy_file(mesh, prefix / 'share/examples/meshes/m16-asymmetric.obj', expected=mesh_sha)
        for name, source in examples.items():
            copy_file(source, prefix / 'share/examples' / name)
        config = prefix / 'share/zenoh.json5'
        config.write_text(json.dumps({'listen': {'endpoints': ['tcp/127.0.0.1:7449']},
                                      'scouting': {'multicast': {'enabled': False}}}, indent=2) + '\n')
        for name, path in viewer.items():
            copy_file(path, prefix / 'web' / name, expected=spec['viewer_files'][name])
        for source_name, installed_name in (('M40_INSTALL.md', 'INSTALL.md'),
                                            ('M40_CAPABILITIES.md', 'CAPABILITIES.md'),
                                            ('M40_PREVIEW.md', 'PREVIEW.md')):
            copy_file(regular(ROOT / 'docs' / source_name), prefix / installed_name)
        copy_file(regular(ROOT / 'LICENSE'), prefix / 'LICENSE')
        copy_file(regular(ROOT / 'NOTICE'), prefix / 'NOTICE')
        notices = collect_notices(documents, extras, prefix / 'notices')
        # Build-input paths belong in qualification evidence, never installed metadata.
        for entry in notices['files']:
            entry.pop('source_path', None)
        for entry in notices['packages']:
            if entry['source'] is None:
                entry['id'] = 'workspace:' + entry['name'] + '@' + entry['version']
            entry.pop('license_file', None)
        (prefix / 'notices.json').write_text(json.dumps(notices, indent=2) + '\n')
        files = {}
        for path in sorted(prefix.rglob('*')):
            if path.is_file():
                with path.open('rb') as stream:
                    os.fsync(stream.fileno())
                files[path.relative_to(prefix).as_posix()] = {
                    'bytes': path.stat().st_size, 'sha256': digest(path),
                    'mode': stat.S_IMODE(path.stat().st_mode)}
        manifest = {'schema_version': 1, 'target': spec['target'], 'repositories': repositories,
                    'profile': 'release; no-default-features; no rgb-camera',
                    'viewer_gzip_bytes': budgets, 'files': files}
        with (prefix / 'manifest.json').open('x') as stream:
            stream.write(json.dumps(manifest, indent=2) + '\n')
            stream.flush()
            os.fsync(stream.fileno())
        for directory in [*sorted((path for path in prefix.rglob('*') if path.is_dir()), reverse=True), prefix]:
            descriptor = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
        # Exclusive mkdir claims a new destination; replacing our empty claim is atomic.
        output.mkdir()
        try:
            os.replace(prefix, output)
            descriptor = os.open(output.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
        except BaseException:
            if prefix.exists():
                try:
                    output.rmdir()
                except OSError:
                    pass  # Never remove unexpected content or a published prefix.
            raise
    if archive is not None:
        archive_prefix(output, archive)
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spec', type=Path, required=True, help='source/build-bound package input JSON')
    parser.add_argument('--output', type=Path, required=True, help='new relocatable installation prefix')
    parser.add_argument('--archive', type=Path, help='optional new tar.gz outside the prefix')
    args = parser.parse_args(argv)
    try:
        manifest = assemble(json_file(args.spec), args.output, args.archive)
        print(json.dumps({'prefix': str(args.output), 'files': len(manifest['files']),
                          'viewer_gzip_bytes': manifest['viewer_gzip_bytes']}))
        return 0
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, 'Package assembly failed: ' + str(error) + '\n')


if __name__ == '__main__':
    raise SystemExit(main())
