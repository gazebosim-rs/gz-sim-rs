#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Build the Zenoh TPE application reference inside the immutable headless SDK.

The default action only prints a Docker argv. The explicit --inside-sdk action
mutates a disposable container's /reference build/install trees, never the host
SDK. It leaves the original manifest/build artifacts intact in the base image. No packages are
installed, native assertions changed, or performance measurements performed.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def module(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


selected = module('headless_selected_client', 'scripts/bootstrap-zenoh-native.py')
pipeline = module('headless_pipeline_contract', 'scripts/benchmark-headless-pipeline.py')
BASE_PATH = ROOT / 'planning/reference/headless-sdk-environment.json'
REGISTRY_PATH = ROOT / 'planning/reference/headless-sdk-build-libraries.json'
APP_NAMES = ('gz-physics', 'gz-sensors', 'gz-sim')


def base_image():
    registry = json.loads(REGISTRY_PATH.read_text())
    if registry['sdk_environment_sha256'] != selected.digest(BASE_PATH):
        raise ValueError('base image registry belongs to a different SDK')
    image = registry['image_id']
    if not re.fullmatch(r'sha256:[0-9a-f]{64}', image):
        raise ValueError('base SDK must have an immutable image ID')
    return image


def container_command(host_client, jobs, name):
    if not 1 <= jobs <= 4 or not re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9_.-]*', name):
        raise ValueError('invalid container build arguments')
    host_client = host_client.resolve(strict=True)
    metadata = selected.validate_reference(host_client)
    archive = host_client / 'downloads' / metadata['lock']['vendor']['url'].rsplit('/', 1)[1]
    # Docker is given an argv, not a shell command. Reject ambiguous mount syntax.
    for path in [ROOT, host_client / 'sources', archive]:
        if ':' in str(path) or ',' in str(path):
            raise ValueError('unsupported Docker bind path')
    return ['docker', 'run', '--pull=never', '--network=none', '--name', name,
            '-v', f'{ROOT}:{ROOT}:ro', '-v', f'{host_client}/sources:/client-sources:ro',
            '-v', f'{archive}:/zenoh-vendor.deb:ro', '-w', str(ROOT), base_image(),
            'python3', str(Path(__file__).resolve()), '--inside-sdk', '--root', '/reference',
            '--client-reference', '/zenoh-client', '--client-sources', '/client-sources',
            '--vendor-archive', '/zenoh-vendor.deb', '--jobs', str(jobs)]


def tools(env):
    return {name: selected.digest(Path(shutil.which(name, path=env['PATH'])).resolve())
            for name in ['c++', 'cmake', 'git', 'python3']}


def packages(env):
    return selected.builder.output(
        ['/usr/bin/dpkg-query', '-W', '-f=${binary:Package}\t${Version}\t${Architecture}\n'], env)


def preflight_base(root):
    if root != Path('/reference') or root.is_symlink():
        raise ValueError('build only in a disposable SDK container at /reference')
    metadata = root / 'environment.json'
    if metadata.is_symlink() or selected.digest(metadata) != selected.digest(BASE_PATH):
        raise ValueError('original immutable SDK manifest differs')
    fresh_paths = ['base-environment.json', 'zenoh-headless-smoke',
                   pipeline.APP_BUILD_DIRECTORIES['gz-sensors'], pipeline.APP_BUILD_DIRECTORIES['gz-sim']]
    if any((root / name).exists() or (root / name).is_symlink() for name in fresh_paths):
        raise ValueError('app build requires a fresh disposable base SDK')
    base = json.loads(BASE_PATH.read_text())
    prefix = root / 'install'
    env = selected.build_environment(prefix, prefix)
    if packages(env) != base['packages'] or tools(env) != base['tools_sha256']:
        raise ValueError('app container package/tool identity differs from immutable SDK')
    pipeline.client_reference.validate_reference(prefix, base, {}, base, pipeline.client_reference.SDK)
    builds = {name.removeprefix('gz-'): root / 'builds' / name for name in APP_NAMES}
    pipeline.validate_sdk_builds(builds, prefix, base)
    return base


def environment(root, client, metadata):
    prefix = client / 'install'
    vendor = client / 'vendor' / metadata['lock']['vendor']['prefix']
    env = selected.build_environment(prefix, vendor)
    env.update(CMAKE_PREFIX_PATH=os.pathsep.join(map(str, [prefix, vendor, root / 'install'])),
               PKG_CONFIG_PATH=os.pathsep.join(str(path / 'lib/pkgconfig') for path in [prefix, vendor, root / 'install']),
               LD_LIBRARY_PATH=os.pathsep.join(str(path / 'lib') for path in
                                              [*(root / pipeline.APP_BUILD_DIRECTORIES[name] for name in APP_NAMES), prefix, root / 'install', vendor]))
    return env


def configure_command(root, client, name, flags):
    extra = {key: '' for key in selected.LAUNCHERS}
    extra.update(CMAKE_FIND_USE_PACKAGE_REGISTRY='OFF', CMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY='OFF')
    for package in ['gz-utils', 'gz-math', 'gz-msgs', 'gz-transport']:
        directory = client / 'install/lib/cmake' / package
        if not directory.is_dir():
            raise ValueError('selected client CMake package is missing: ' + package)
        extra[package + '_DIR'] = str(directory)
    return ['/usr/bin/cmake', '-S', str(root / 'sources' / name), '-B', str(root / pipeline.APP_BUILD_DIRECTORIES[name]),
            '-G', 'Unix Makefiles', *(f'-D{key}={value}' for key, value in (flags | extra).items())]


def fresh_build_directory(root, name):
    if name not in {'gz-sensors', 'gz-sim'}:
        raise ValueError('only affected app modules get new build directories')
    path = root / pipeline.APP_BUILD_DIRECTORIES[name]
    if path.exists() or path.is_symlink():
        raise ValueError('app build directory must be fresh')
    # New upper-layer paths avoid EXDEV renames or copying old overlay contents.
    path.mkdir()
    return path


def owned_source_cache(source_root, destination, env):
    if destination.exists() or destination.is_symlink():
        raise ValueError('owned client source cache must be fresh')
    lock = json.loads((ROOT / 'planning/zenoh-native-lock.json').read_text())
    selected.validate_lock(lock, json.loads((ROOT / 'planning/baseline.json').read_text()))
    destination.mkdir()
    for row in lock['repositories']:
        source = source_root / row['name']
        if source.is_symlink() or not source.is_dir():
            raise ValueError('offline client source must be a real directory')
        copied = destination / row['name']
        # copytree preserves source symlinks but gives files the container user's
        # ownership. The read-only host cache and Git safe-directory policy stay intact.
        shutil.copytree(source, copied, symlinks=True)
        selected.builder.verify_source(copied, row, env)
    return destination


def compiler_records(root, client):
    headers = {str(Path('include/gz/transport16/gz/transport') / name):
               selected.digest(client / 'install/include/gz/transport16/gz/transport' / name)
               for name in pipeline.TRANSPORT_HEADERS}
    records = {}
    for source, repository in pipeline.TRANSPORT_CONSUMERS.items():
        basename = Path(source.split(':', 1)[1]).name
        matches = list((root / pipeline.APP_BUILD_DIRECTORIES[repository]).rglob(basename + '.o.d'))
        if len(matches) != 1:
            raise ValueError('expected exactly one affected compiler dependency: ' + source)
        depfile = matches[0]
        paths = {'depfile': depfile, 'object': depfile.with_suffix(''), 'flags': depfile.parent / 'flags.make'}
        records[source] = {name: str(path.relative_to(root)) for name, path in paths.items()}
        records[source]['files'] = {str(path.relative_to(root)): selected.digest(path) for path in paths.values()}
    pipeline.validate_transport_compilation(root, client / 'install', records, headers)
    return records, headers


def smoke_command(root, compiler, source, binary):
    target = root / pipeline.APP_BUILD_DIRECTORIES['gz-sim'] / 'src/CMakeFiles/UNIT_ServerConfig_TEST.dir'
    flags = (target / 'flags.make').read_text()
    includes = shlex.split(next(line.split('=', 1)[1] for line in flags.splitlines()
                                if line.startswith('CXX_INCLUDES =')))
    link = shlex.split((target / 'link.txt').read_text())
    libraries = [str((root / pipeline.APP_BUILD_DIRECTORIES['gz-sim'] / 'src' / value).resolve()) if value.startswith('../') else value
                 for value in link[link.index('-o') + 2:] if 'gtest' not in value]
    return [compiler, '-std=c++17', '-O3', '-DNDEBUG', '-Wall', '-Wextra', '-Werror',
            *includes, str(source), '-o', str(binary), *libraries]


def run_smoke(command, env, directory):
    child = subprocess.Popen(command, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
    try:
        stdout, stderr = child.communicate(timeout=15)
    except subprocess.TimeoutExpired:
        # This group belongs only to this fresh child session; always reap it.
        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass  # It exited between the timeout and signal; still reap it.
        stdout, stderr = child.communicate()
        (directory / 'stdout').write_bytes(stdout)
        (directory / 'stderr').write_bytes(stderr)
        raise
    (directory / 'stdout').write_bytes(stdout)
    (directory / 'stderr').write_bytes(stderr)
    if child.returncode != 0:
        raise subprocess.CalledProcessError(child.returncode, command, stdout, stderr)


def build(root, client, local_sources, archive, jobs):
    base = preflight_base(root)
    initial = {name: selected.digest(ROOT / name) for name in
               pipeline.APP_BUILD_INPUTS | {'scripts/bootstrap-zenoh-headless.py'}}
    (root / 'base-environment.json').write_bytes((root / 'environment.json').read_bytes())
    log = root / 'zenoh-headless-build.log'
    env = selected.build_environment(root / 'install', root / 'install')
    if not client.exists() or not any(client.iterdir()):
        cache = owned_source_cache(local_sources, client.with_name(client.name + '-source-cache'), env)
        selected.builder.run(['python3', ROOT / 'scripts/bootstrap-zenoh-native.py', '--root', client,
                              '--local-sources', cache, '--vendor-archive', archive, '--jobs', str(jobs)], log, env)
    client_metadata = selected.validate_reference(client)
    env = environment(root, client, client_metadata)
    if tools(env) != base['tools_sha256'] or client_metadata['tools']['c++']['sha256'] != base['tools_sha256']['c++']:
        raise ValueError('selected client was built by a different toolchain')
    pins = {row['name']: row for row in base['repositories']}
    records, registries = [], {}
    for name in APP_NAMES:
        source, build_dir = root / 'sources' / name, root / pipeline.APP_BUILD_DIRECTORIES[name]
        selected.builder.verify_source(source, pins[name], env)
        flags = pins[name]['flags'] | {'CMAKE_INSTALL_PREFIX': str(root / 'install')}
        command = None
        if name != 'gz-physics':
            fresh_build_directory(root, name)
            command = configure_command(root, client, name, flags)
            selected.builder.run(command, log, env)
            pipeline.validate_app_cache(selected.cache_values(build_dir / 'CMakeCache.txt'), flags, flags)
            selected.builder.run(['/usr/bin/cmake', '--build', build_dir, '--parallel', str(jobs)], log, env)
            selected.builder.run(['/usr/bin/cmake', '--install', build_dir], log, env)
        selected.builder.verify_source(source, pins[name], env)
        checksum = selected.digest(build_dir / 'CMakeCache.txt')
        records.append({'name': name, 'sha': pins[name]['sha'], 'flags': flags,
                        'cmake_cache_sha256': checksum, 'configure_command': command,
                        'artifact_reused_from_base': name == 'gz-physics'})
        registries[name] = {'directory': pipeline.APP_BUILD_DIRECTORIES[name], 'source': 'sources/' + name,
                            'cmake_cache_sha256': checksum, 'library_files': selected.inventory(build_dir / 'lib')}
    transport_records, headers = compiler_records(root, client)
    probe = root / 'zenoh-headless-smoke'
    probe.mkdir()
    source, binary, config, maps = (probe / name for name in ['source.cc', 'probe', 'zenoh.json', 'maps'])
    source.write_bytes((ROOT / 'scripts/zenoh_headless_probe.cc').read_bytes())
    compile_command = smoke_command(root, client_metadata['tools']['c++']['path'], source, binary)
    selected.builder.run(compile_command, log, env)
    config.write_text(json.dumps({'mode': 'peer', 'listen': {'endpoints': ['tcp/127.0.0.1:0']},
                                 'connect': {'endpoints': []}, 'scouting': {'multicast': {'enabled': False}, 'gossip': {'enabled': False}},
                                 'transport': {'shared_memory': {'enabled': False}}}) + '\n')
    libdirs = [*(root / pipeline.APP_BUILD_DIRECTORIES[name] / 'lib' for name in APP_NAMES),
               client / 'install/lib', root / 'install/lib', client / 'vendor' / client_metadata['lock']['vendor']['prefix'] / 'lib']
    child_env = pipeline.app_runtime_environment(env, libdirs, config)
    child_env.update(GZ_PARTITION=f'headless-smoke-{os.getpid()}', GZ_IP='127.0.0.1',
                     GZ_SIM_SERVER_CONFIG_PATH='', GZ_SIM_SYSTEM_PLUGIN_PATH=str(root / pipeline.APP_BUILD_DIRECTORIES['gz-sim'] / 'lib'),
                     GZ_SIM_PHYSICS_ENGINE_PATH=str(root / pipeline.APP_BUILD_DIRECTORIES['gz-physics'] / 'lib'), GZ_TRANSPORT_TOPIC_STATISTICS='0', RUST_LOG='error')
    fixture = ROOT / 'crates/gz-sim-scene/tests/fixtures/headless_imu.sdf'
    command = [str(binary), str(fixture), str(maps)]
    run_smoke(command, child_env, probe)
    paths = {'source': source, 'binary': binary, 'config': config, 'stdout': probe / 'stdout', 'stderr': probe / 'stderr', 'maps': maps}
    smoke = {'status': 0, 'timeout_seconds': 15, 'command': command, 'compile_command': compile_command,
             'files': {name: {'path': str(path.relative_to(root)), 'sha256': selected.digest(path)} for name, path in paths.items()},
             'libraries_sha256': pipeline.loaded_libraries(maps)}
    target = root / pipeline.APP_BUILD_DIRECTORIES['gz-sim'] / 'src/CMakeFiles/UNIT_ServerConfig_TEST.dir'
    metadata = {'schema_version': 1, 'profile': pipeline.FULL_APP,
                'base_sdk_environment_sha256': selected.digest(BASE_PATH),
                'client_reference': {'root': str(client), 'environment_sha256': selected.digest(client / 'environment.json')},
                'inputs_sha256': initial, 'packages': packages(env), 'tools_sha256': tools(env),
                'repositories': records, 'builds': registries, 'installed_files': selected.inventory(root / 'install'),
                'server_compiler_inputs': {name: selected.digest(target / name) for name in ['flags.make', 'link.txt']},
                'transport_compiler_inputs': transport_records, 'transport_headers': headers, 'smoke': smoke}
    builds = {name.removeprefix('gz-'): root / pipeline.APP_BUILD_DIRECTORIES[name] for name in APP_NAMES}
    publish(root, builds, client, initial, metadata)
    return metadata


def publish(root, builds, client, initial, metadata):
    pipeline.app_files(ROOT, initial)
    pipeline.validate_app_reference(root, builds, client / 'install', root / 'install', metadata=metadata)
    pending = root / 'environment.pending.json'
    if pending.exists() or pending.is_symlink():
        raise ValueError('pending app manifest already exists')
    pending.write_text(json.dumps(metadata, indent=2) + '\n')
    pipeline.app_files(ROOT, initial)
    pipeline.validate_app_reference(root, builds, client / 'install', root / 'install', metadata=metadata)
    pending.replace(root / 'environment.json')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inside-sdk', action='store_true')
    parser.add_argument('--root', type=Path, default=Path('/reference'))
    parser.add_argument('--client-reference', type=Path, default=ROOT / 'artifacts/zenoh-native-reference')
    parser.add_argument('--client-sources', type=Path)
    parser.add_argument('--vendor-archive', type=Path)
    parser.add_argument('--jobs', type=int, default=2)
    parser.add_argument('--name', default='gz-zenoh-headless-reference')
    args = parser.parse_args()
    if not 1 <= args.jobs <= 4:
        parser.error('--jobs must be 1..4')
    if not args.inside_sdk:
        print(json.dumps(container_command(args.client_reference, args.jobs, args.name), indent=2))
        return
    if args.client_sources is None or args.vendor_archive is None:
        parser.error('inside-SDK build needs offline sources and vendor archive')
    metadata = build(args.root.absolute(), args.client_reference.absolute(), args.client_sources.absolute(), args.vendor_archive.absolute(), args.jobs)
    print(json.dumps({'profile': metadata['profile'], 'reference_root': str(args.root.absolute()),
                      'scope': 'actual source build and untimed app smoke only; no performance acceptance'}, indent=2))


if __name__ == '__main__':
    main()
