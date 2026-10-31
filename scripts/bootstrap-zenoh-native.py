#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Build and validate a separate, pinned Zenoh-enabled C++ client reference.

This never installs host packages or qualifies the historical Zenoh-OFF SDK.
The resulting manifest records actual build outputs, not portable ELF hashes.
"""
import argparse
import hashlib
import importlib.util
import json
import lzma
import os
from pathlib import Path
import platform
import re
import shlex
import shutil
import subprocess
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
PROFILE = 'gazebo-zenoh-client-v1'
SOURCES = ('gz-cmake', 'gz-utils', 'gz-math', 'gz-tools', 'gz-msgs', 'gz-transport')
LAUNCHERS = ('CMAKE_C_COMPILER_LAUNCHER', 'CMAKE_CXX_COMPILER_LAUNCHER',
             'CMAKE_C_LINKER_LAUNCHER', 'CMAKE_CXX_LINKER_LAUNCHER')
INPUTS = ('scripts/bootstrap-zenoh-native.py', 'scripts/zenoh_native_probe.cc',
          'planning/zenoh-native-lock.json', 'planning/baseline.json',
          'planning/zenoh-native-packages.json',
          'reference/build-reference.py', 'LICENSE', 'NOTICE')
COMMON = {'CMAKE_BUILD_TYPE': 'Release', 'BUILD_TESTING': 'OFF',
          'CMAKE_INSTALL_LIBDIR': 'lib', 'SKIP_PYBIND11': 'ON', 'SKIP_SWIG': 'ON',
          'GZ_UTILS_VENDOR_CLI11': 'ON'} | dict.fromkeys(LAUNCHERS, '')
TRANSPORT = {'GZ_TRANSPORT_ENABLE_ZENOH': 'ON',
             'GZ_TRANSPORT_DEFAULT_IMPLEMENTATION': 'zeromq'}
spec = importlib.util.spec_from_file_location('zenoh_reference_builder', ROOT / 'reference/build-reference.py')
builder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(builder)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inputs(root=ROOT):
    return {name: digest(root / name) for name in INPUTS}


def validate_lock(lock, baseline):
    if lock['schema_version'] != 1 or lock['profile'] != PROFILE:
        raise ValueError('unknown Zenoh native profile')
    if lock['cmake'] != COMMON or lock['transport_cmake'] != TRANSPORT or lock['functional_implementation'] != 'zenoh':
        raise ValueError('Zenoh client build flags differ')
    if lock['platform'] != {'os': 'ubuntu', 'version': '24.04', 'architecture': 'amd64', 'ubuntu_snapshot': '20261024T000000Z'}:
        raise ValueError('unsupported native platform or dependency snapshot')
    pins = {row['name']: row for row in baseline['repositories']}
    if [row['name'] for row in lock['repositories']] != list(SOURCES):
        raise ValueError('native source selection differs')
    for row in lock['repositories']:
        pin = pins[row['name']]
        if row['sha'] != pin['commit'] or row['url'] != pin['upstream_url'] or not re.fullmatch('[0-9a-f]{40}', row['sha']):
            raise ValueError('native source pin differs: ' + row['name'])
    vendor = lock['vendor']
    filename = f"{vendor['package']}_{vendor['version']}_{vendor['architecture']}.deb"
    if (vendor['package'] != 'ros-jazzy-zenoh-cpp-vendor'
            or not re.fullmatch('[0-9][a-zA-Z0-9.+~_-]*', vendor['version'])
            or vendor['architecture'] != 'amd64' or vendor['zenoh_version'] != '1.8.0'
            or vendor['prefix'] != 'opt/ros/jazzy/opt/zenoh_cpp_vendor'
            or not re.fullmatch('[0-9a-f]{64}', vendor['sha256'])
            or vendor['url'] != 'https://packages.ros.org/ros2/ubuntu/pool/main/r/ros-jazzy-zenoh-cpp-vendor/' + filename):
        raise ValueError('invalid pinned Zenoh vendor archive')
    if not lock['packages'] or any(not re.fullmatch('[a-z0-9][a-z0-9+.-]*', key)
                                   or not re.fullmatch('[0-9][a-zA-Z0-9:+.~_-]*', version)
                                   for key, version in lock['packages'].items()):
        raise ValueError('invalid host package pins')
    validate_package_pins(json.loads((ROOT / 'planning/zenoh-native-packages.json').read_text()), lock)


def validate_package_pins(pins, lock):
    if (pins['schema_version'] != 1 or pins['profile'] != 'ubuntu-zenoh-native-packages-v1'
            or pins['snapshot'] != lock['platform']['ubuntu_snapshot']
            or pins['signing_key'] != 'F6ECB3762474EDA9D21B7022871920D1991BC93C'
            or [row['suite'] for row in pins['releases']] != ['noble', 'noble-updates', 'noble-security']):
        raise ValueError('Ubuntu package proof differs')
    found = set()
    for row in pins['packages']:
        if lock['packages'].get(row['Package']) != row['Version']:
            raise ValueError('package proof version differs: ' + row['Package'])
        if row['Architecture'] not in ['amd64', 'all'] or not re.fullmatch('[0-9a-f]{64}', row['SHA256']):
            raise ValueError('package proof artifact differs')
        found.add(row['Package'])
    if found != set(lock['packages']):
        raise ValueError('package proof does not cover every locked direct dependency')


def checked_download(url, path, expected):
    if path.is_symlink():
        raise ValueError('metadata cache must not be a symlink')
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        with urllib.request.urlopen(url, timeout=30) as response:
            data = response.read()
        if hashlib.sha256(data).hexdigest() != expected:
            raise ValueError('download checksum differs: ' + url)
        path.write_bytes(data)
    if digest(path) != expected:
        raise ValueError('cached metadata changed: ' + str(path))


def release_hashes(text):
    lines = text.splitlines()
    index = lines.index('SHA256:') + 1
    hashes = {}
    while index < len(lines) and lines[index].startswith(' '):
        sha, size, name = lines[index].split()
        hashes[name] = (sha, int(size))
        index += 1
    return hashes


def package_records(path):
    record = {}
    with lzma.open(path, 'rt', encoding='utf-8') as stream:
        for line in stream:
            if line == '\n':
                if 'Package' in record:
                    yield record
                record = {}
            elif not line.startswith(' '):
                key, _, value = line.rstrip('\n').partition(': ')
                record[key] = value
    if 'Package' in record:
        yield record


def verify_package_indexes(directory, lock):
    pins = json.loads((ROOT / 'planning/zenoh-native-packages.json').read_text())
    validate_package_pins(pins, lock)
    directory.mkdir(parents=True, exist_ok=True)
    if directory.is_symlink():
        raise ValueError('metadata root must be a real directory')
    observed = []
    fields = ['Package', 'Version', 'Architecture', 'Filename', 'SHA256', 'Size']
    env = git_environment()
    for release in pins['releases']:
        suite = release['suite']
        url = f"https://snapshot.ubuntu.com/ubuntu/{pins['snapshot']}/dists/{suite}/InRelease"
        if release['url'] != url or release['signature_keys'] != [pins['signing_key']]:
            raise ValueError('Ubuntu release identity differs')
        path = directory / (suite + '.InRelease')
        checked_download(url, path, release['sha256'])
        command = ['gpgv', '--keyring', '/usr/share/keyrings/ubuntu-archive-keyring.gpg', '--status-fd', '1', str(path)]
        result = subprocess.run(command, env=env, capture_output=True)
        (directory / (suite + '.gpgv.stdout')).write_bytes(result.stdout)
        (directory / (suite + '.gpgv.stderr')).write_bytes(result.stderr)
        result.check_returncode()
        keys = [line.split()[2] for line in result.stdout.decode().splitlines() if line.startswith('[GNUPG:] VALIDSIG ')]
        if keys != [pins['signing_key']]:
            raise ValueError('Ubuntu release signature key differs')
        hashes = release_hashes(path.read_text())
        if [row['component'] for row in release['indexes']] != ['main', 'universe']:
            raise ValueError('Ubuntu package index selection differs')
        for index in release['indexes']:
            name = index['component'] + '/binary-amd64/Packages.xz'
            if index['path'] != name or hashes.get(name) != (index['sha256'], index['size']):
                raise ValueError('Ubuntu index differs from signed release')
            packages = directory / f"{suite}-{index['component']}.Packages.xz"
            checked_download(url.rsplit('/', 1)[0] + '/' + name, packages, index['sha256'])
            if packages.stat().st_size != index['size']:
                raise ValueError('Ubuntu package index size differs')
            for row in package_records(packages):
                if row.get('Package') in lock['packages'] and lock['packages'][row['Package']] == row.get('Version'):
                    observed.append({'suite': suite, 'component': index['component'], **{key: row[key] for key in fields}})
    normalize = lambda rows: sorted(json.dumps(row, sort_keys=True) for row in rows)
    if normalize(observed) != normalize(pins['packages']):
        raise ValueError('locked packages differ from signed snapshot indexes')
    return {'snapshot': pins['snapshot'], 'direct_packages': len(lock['packages']),
            'matching_records': len(observed), 'signing_key': pins['signing_key']}


def build_environment(prefix, vendor):
    env = git_environment()
    for key in ['CC', 'CXX', 'CFLAGS', 'CXXFLAGS', 'CPPFLAGS', 'LDFLAGS', 'CMAKE_TOOLCHAIN_FILE',
                'CPATH', 'CPLUS_INCLUDE_PATH', 'C_INCLUDE_PATH', 'LIBRARY_PATH', 'COMPILER_PATH',
                'GCC_EXEC_PREFIX', 'PKG_CONFIG_LIBDIR', 'PKG_CONFIG_SYSROOT_DIR',
                'ROS_DISTRO', 'AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH']:
        env.pop(key, None)
    for key in LAUNCHERS:
        env.pop(key, None)
    env.update(CMAKE_PREFIX_PATH=f'{prefix}{os.pathsep}{vendor}',
               PKG_CONFIG_PATH=str(prefix / 'lib/pkgconfig'),
               LD_LIBRARY_PATH=f'{prefix}/lib:{vendor}/lib', LC_ALL='C')
    env['PATH'] = str(prefix / 'bin') + os.pathsep + env['PATH']
    return env


def git_environment():
    env = dict(os.environ)
    for key in ['LD_PRELOAD', 'LD_AUDIT']:
        env.pop(key, None)
    keys = subprocess.check_output(['git', 'rev-parse', '--local-env-vars'], env=env, text=True).splitlines()
    for key in keys:
        env.pop(key, None)
    env.update(GIT_CONFIG_NOSYSTEM='1', GIT_CONFIG_GLOBAL='/dev/null', GIT_TERMINAL_PROMPT='0',
               GIT_OPTIONAL_LOCKS='0', PYTHONDONTWRITEBYTECODE='1')
    return env


def tool_identities(env):
    result = {}
    for name, command in [('cc', '/usr/bin/gcc'), ('c++', '/usr/bin/g++'),
                          ('cmake', '/usr/bin/cmake'), ('protoc', '/usr/bin/protoc'),
                          ('pkg-config', '/usr/bin/pkg-config'), ('readelf', '/usr/bin/readelf')]:
        path = Path(command).resolve(strict=True)
        result[name] = {'path': str(path), 'sha256': digest(path),
                        'version': builder.output([path, '--version'], env)}
    return result


def verify_host(lock, env):
    os_release = dict(line.split('=', 1) for line in Path('/etc/os-release').read_text().splitlines() if '=' in line)
    if (os_release.get('ID') != 'ubuntu' or os_release.get('VERSION_ID', '').strip('"') != '24.04'
            or platform.machine() != 'x86_64'):
        raise ValueError('native reference requires Ubuntu 24.04 amd64')
    found = {}
    for name, expected in lock['packages'].items():
        actual = builder.output(['dpkg-query', '-W', '-f=${Version}', name], env)
        if actual != expected:
            raise ValueError(f'host package differs: {name}: expected {expected}, got {actual}')
        found[name] = actual
    if builder.output(['/usr/bin/protoc', '--version'], env) != 'libprotoc 3.21.12':
        raise ValueError('native protobuf compiler differs')
    return found


def inventory(root):
    if root.is_symlink() or not root.is_dir():
        raise ValueError('reference inventory root must be a real directory')
    for path in root.rglob('*'):
        if not (path.is_symlink() or path.is_file() or path.is_dir()):
            raise ValueError('unsupported reference installed entry: ' + str(path))
    return builder.files_under(root)


def vendor_versions(vendor, expected):
    for name in ['zenohc', 'zenohcxx']:
        text = (vendor / f'lib/cmake/{name}/{name}ConfigVersion.cmake').read_text()
        versions = re.findall(r'^set\(PACKAGE_VERSION "([^"]+)"\)$', text, re.MULTILINE)
        if versions != [expected]:
            raise ValueError('native vendor version differs: ' + name)
    config = (vendor / 'lib/cmake/zenohc/zenohcConfig.cmake').read_text()
    features = {name: f'set(ZENOHC_BUILD_WITH_{name} TRUE)' in config for name in ['UNSTABLE_API', 'SHARED_MEMORY']}
    if not all(features.values()):
        raise ValueError('native vendor build features differ')
    return features


def vendor_archive(reference, vendor, supplied, log, env):
    archive = reference / 'downloads' / vendor['url'].rsplit('/', 1)[1]
    archive.parent.mkdir(parents=True, exist_ok=True)
    if supplied is not None:
        if digest(supplied) != vendor['sha256']:
            raise ValueError('vendor archive checksum differs')
        if supplied.resolve() != archive.resolve():
            shutil.copyfile(supplied, archive)
    elif not archive.exists():
        temporary = archive.with_suffix('.partial')
        urllib.request.urlretrieve(vendor['url'], temporary)
        if digest(temporary) != vendor['sha256']:
            raise ValueError('vendor archive checksum differs')
        temporary.replace(archive)
    if digest(archive) != vendor['sha256']:
        raise ValueError('vendor archive checksum differs')
    for field in ['Package', 'Version', 'Architecture']:
        if builder.output(['dpkg-deb', '-f', archive, field], env) != vendor[field.lower()]:
            raise ValueError('vendor archive control differs: ' + field)
    destination = reference / 'vendor'
    if destination.exists():
        raise ValueError('vendor extraction already exists; validate the completed reference or use a fresh root')
    builder.run(['dpkg-deb', '-x', archive, destination], log, env)
    inventory(destination)
    return archive


def cache_values(path):
    result = {}
    for line in path.read_text().splitlines():
        if line and not line.startswith(('#', '//')) and ':' in line and '=' in line:
            key, value = line.split('=', 1)
            result[key.split(':', 1)[0]] = value
    return result


def validate_cache(path, source, prefix, vendor, transport=False):
    values = cache_values(path)
    expected = COMMON | {'CMAKE_INSTALL_PREFIX': str(prefix), 'CMAKE_HOME_DIRECTORY': str(source),
                         'CMAKE_C_COMPILER': '/usr/bin/gcc', 'CMAKE_CXX_COMPILER': '/usr/bin/g++'}
    if transport:
        expected |= TRANSPORT | {'HAVE_ZENOH': 'ON',
                                 'zenohc_DIR': str(vendor / 'lib/cmake/zenohc'),
                                 'zenohcxx_DIR': str(vendor / 'lib/cmake/zenohcxx')}
    for key, value in expected.items():
        if values.get(key) != value:
            raise ValueError(f'native CMake cache differs: {key}: {values.get(key)!r}')
    return values


def validate_linkage(text, prefix, vendor):
    if 'not found' in text:
        raise ValueError('native dependency is unresolved')
    selected = {}
    for line in text.splitlines():
        if '=>' not in line:
            continue
        name, target = line.strip().split('=>', 1)
        name = name.strip()
        path = Path(target.split()[0]).resolve(strict=True)
        if name.startswith('libgz-') and not path.is_relative_to(prefix.resolve()):
            raise ValueError('native Gazebo dependency escapes selected prefix: ' + name)
        if name == 'libzenohc.so' and not path.is_relative_to(vendor.resolve()):
            raise ValueError('native Zenoh dependency escapes selected vendor')
        selected[name] = {'path': str(path), 'sha256': digest(path)}
    for name in ['libgz-transport.so.16', 'libgz-msgs.so.13', 'libzenohc.so']:
        if name not in selected:
            raise ValueError('native required dependency missing: ' + name)
    return selected


def transport_dynamic(prefix, env):
    text = builder.output(['/usr/bin/readelf', '-d', prefix / 'lib/libgz-transport.so.16'], env)
    if 'Shared library: [libzenohc.so]' not in text:
        raise ValueError('transport ELF has no direct Zenoh dependency')
    return text


def probe_environment(env, config):
    child = {key: value for key, value in env.items() if not key.startswith(('GZ_', 'ZENOH_'))}
    child.update(GZ_TRANSPORT_IMPLEMENTATION='zenoh', GZ_PARTITION=f'bootstrap-{os.getpid()}', ZENOH_CONFIG=str(config))
    return child


def build_probe(reference, prefix, vendor, env, log):
    output = reference / 'probe'
    output.mkdir()
    shutil.copyfile(ROOT / 'scripts/zenoh_native_probe.cc', output / 'probe.cc')
    flags = shlex.split(builder.output(['/usr/bin/pkg-config', '--cflags', '--libs', 'gz-transport'], env))
    actual = Path(builder.output(['/usr/bin/pkg-config', '--variable=prefix', 'gz-transport'], env)).resolve()
    if actual != prefix.resolve():
        raise ValueError('pkg-config selected a foreign transport')
    command = ['/usr/bin/g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-pthread',
               output / 'probe.cc', *flags, f'-Wl,-rpath,{prefix}/lib', f'-Wl,-rpath,{vendor}/lib', '-o', output / 'probe']
    builder.run(command, log, env)
    linkage = builder.output(['ldd', output / 'probe'], env)
    (output / 'linkage.txt').write_text(linkage + '\n')
    libraries = validate_linkage(linkage, prefix, vendor)
    dynamic = transport_dynamic(prefix, env)
    (output / 'transport-dynamic.txt').write_text(dynamic + '\n')
    config = output / 'zenoh.json'
    config.write_text(json.dumps({'mode': 'peer', 'listen': {'endpoints': ['tcp/127.0.0.1:0']},
                                 'connect': {'endpoints': []}, 'scouting': {'multicast': {'enabled': False}, 'gossip': {'enabled': False}},
                                 'transport': {'shared_memory': {'enabled': False}}}) + '\n')
    try:
        result = subprocess.run([output / 'probe'], env=probe_environment(env, config), capture_output=True, timeout=15)
    except subprocess.TimeoutExpired as error:
        (output / 'stdout').write_bytes(error.stdout or b'')
        (output / 'stderr').write_bytes(error.stderr or b'')
        raise
    (output / 'stdout').write_bytes(result.stdout)
    (output / 'stderr').write_bytes(result.stderr)
    result.check_returncode()
    if result.stdout != b'zenoh client ready\n':
        raise ValueError('unexpected Zenoh native probe result')
    return {'command': list(map(str, command)), 'sha256': digest(output / 'probe'), 'libraries': libraries,
            'transport_dynamic_sha256': hashlib.sha256(dynamic.encode()).hexdigest(), 'status': 0,
            'files': {name: digest(output / name) for name in
                      ['probe.cc', 'zenoh.json', 'stdout', 'stderr', 'linkage.txt', 'transport-dynamic.txt']}}


def verify_inputs(initial):
    if inputs() != initial:
        raise ValueError('Zenoh bootstrap input changed during build')


def validate_reference(reference, lock=None, metadata=None):
    if reference.is_symlink() or not reference.is_dir():
        raise ValueError('native reference root must be a real directory')
    lock = lock or json.loads((ROOT / 'planning/zenoh-native-lock.json').read_text())
    validate_lock(lock, json.loads((ROOT / 'planning/baseline.json').read_text()))
    if metadata is None:
        metadata = json.loads((reference / 'environment.json').read_text())
    if metadata.get('profile') != PROFILE or metadata.get('lock') != lock or metadata.get('inputs_sha256') != inputs():
        raise ValueError('Zenoh reference contract differs')
    prefix = reference / 'install'
    vendor = reference / 'vendor' / lock['vendor']['prefix']
    env = build_environment(prefix, vendor)
    if verify_host(lock, env) != metadata['packages'] or tool_identities(env) != metadata['tools']:
        raise ValueError('native reference tool or package identity changed')
    if inventory(prefix) != metadata['installed_files'] or inventory(reference / 'vendor') != metadata['vendor_files']:
        raise ValueError('native installed inventory changed')
    if vendor_versions(vendor, lock['vendor']['zenoh_version']) != metadata['vendor_features']:
        raise ValueError('native vendor configuration changed')
    archive = reference / 'downloads' / lock['vendor']['url'].rsplit('/', 1)[1]
    if archive.is_symlink() or digest(archive) != lock['vendor']['sha256']:
        raise ValueError('vendor archive changed')
    for row in lock['repositories']:
        source = reference / 'sources' / row['name']
        if not source.resolve(strict=True).is_relative_to(reference.resolve()):
            raise ValueError('native source escapes reference root')
        builder.verify_source(source, row, env)
        cache = reference / 'build' / row['name'] / 'CMakeCache.txt'
        if not cache.resolve(strict=True).is_relative_to(reference.resolve()):
            raise ValueError('native build cache escapes reference root')
        validate_cache(cache, source, prefix, vendor, row['name'] == 'gz-transport')
        if digest(cache) != metadata['cache_sha256'][row['name']]:
            raise ValueError('native CMake cache changed')
    probe = reference / 'probe/probe'
    if probe.is_symlink() or digest(probe) != metadata['probe']['sha256'] or digest(reference / 'probe/probe.cc') != digest(ROOT / 'scripts/zenoh_native_probe.cc'):
        raise ValueError('native probe changed')
    required = {'probe.cc', 'zenoh.json', 'stdout', 'stderr', 'linkage.txt', 'transport-dynamic.txt'}
    if set(metadata['probe']['files']) != required or metadata['probe']['status'] != 0:
        raise ValueError('native probe evidence is incomplete')
    for name, expected in metadata['probe']['files'].items():
        path = reference / 'probe' / name
        if path.is_symlink() or digest(path) != expected:
            raise ValueError('native probe evidence changed: ' + name)
    if validate_linkage(builder.output(['ldd', probe], env), prefix, vendor) != metadata['probe']['libraries']:
        raise ValueError('native probe dependency changed')
    if hashlib.sha256(transport_dynamic(prefix, env).encode()).hexdigest() != metadata['probe']['transport_dynamic_sha256']:
        raise ValueError('transport ELF dynamic identity changed')
    return metadata


def publish_manifest(reference, lock, metadata):
    validate_reference(reference, lock, metadata)
    verify_inputs(metadata['inputs_sha256'])
    for tool in metadata['tools'].values():
        if digest(Path(tool['path'])) != tool['sha256']:
            raise ValueError('native tool changed before manifest publication')
    temporary = reference / 'environment.partial.json'
    temporary.write_text(json.dumps(metadata, indent=2) + '\n')
    temporary.replace(reference / 'environment.json')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path)
    parser.add_argument('--local-sources', type=Path)
    parser.add_argument('--vendor-archive', type=Path)
    parser.add_argument('--jobs', type=int, default=2)
    parser.add_argument('--validate-only', action='store_true')
    parser.add_argument('--verify-packages-only', action='store_true')
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error('--jobs must be positive')
    if args.validate_only and args.verify_packages_only:
        parser.error('choose one validation mode')
    root = args.root or ROOT / 'artifacts' / ('zenoh-native-package-metadata' if args.verify_packages_only else 'zenoh-native-reference')
    reference = root.resolve()
    if root.is_symlink():
        raise ValueError('native reference root must not be a symlink')
    if args.verify_packages_only:
        lock = json.loads((ROOT / 'planning/zenoh-native-lock.json').read_text())
        validate_lock(lock, json.loads((ROOT / 'planning/baseline.json').read_text()))
        print(json.dumps(verify_package_indexes(reference, lock), indent=2))
        return
    if args.validate_only:
        validate_reference(reference)
    else:
        if reference.exists() and any(reference.iterdir()):
            raise ValueError('build requires a fresh reference root; use --validate-only for completed builds')
        reference.mkdir(parents=True, exist_ok=True)
        lock = json.loads((ROOT / 'planning/zenoh-native-lock.json').read_text())
        validate_lock(lock, json.loads((ROOT / 'planning/baseline.json').read_text()))
        initial = inputs()
        prefix = reference / 'install'
        vendor = reference / 'vendor' / lock['vendor']['prefix']
        env = build_environment(prefix, vendor)
        packages = verify_host(lock, env)
        tools = tool_identities(env)
        log = reference / 'build.log'
        vendor_archive(reference, lock['vendor'], args.vendor_archive, log, env)
        vendor_files = inventory(reference / 'vendor')
        features = vendor_versions(vendor, lock['vendor']['zenoh_version'])
        for row in lock['repositories']:
            source = reference / 'sources' / row['name']
            source.mkdir(parents=True)
            origin = str(args.local_sources.resolve() / row['name']) if args.local_sources else row['url']
            builder.run(['git', 'init', '--quiet', source], log, env)
            builder.run(['git', '-C', source, 'fetch', '--depth', '1', origin, row['sha']], log, env)
            builder.run(['git', '-C', source, 'checkout', '--detach', 'FETCH_HEAD'], log, env)
            builder.verify_source(source, row, env)
            build = reference / 'build' / row['name']
            flags = lock['cmake'] | {'CMAKE_INSTALL_PREFIX': str(prefix), 'CMAKE_C_COMPILER': '/usr/bin/gcc', 'CMAKE_CXX_COMPILER': '/usr/bin/g++'}
            if row['name'] == 'gz-transport':
                flags |= lock['transport_cmake'] | {'zenohc_DIR': str(vendor / 'lib/cmake/zenohc'), 'zenohcxx_DIR': str(vendor / 'lib/cmake/zenohcxx')}
            command = ['/usr/bin/cmake', '-S', source, '-B', build, '-G', 'Unix Makefiles', *(f'-D{key}={value}' for key, value in flags.items())]
            builder.run(command, log, env)
            validate_cache(build / 'CMakeCache.txt', source, prefix, vendor, row['name'] == 'gz-transport')
            builder.run(['/usr/bin/cmake', '--build', build, '--parallel', str(args.jobs)], log, env)
            builder.run(['/usr/bin/cmake', '--install', build], log, env)
            builder.verify_source(source, row, env)
            verify_inputs(initial)
        probe = build_probe(reference, prefix, vendor, env, log)
        if tool_identities(env) != tools or verify_host(lock, env) != packages:
            raise ValueError('native build tools or packages changed during build')
        if inventory(reference / 'vendor') != vendor_files:
            raise ValueError('pinned vendor inputs changed during build')
        verify_inputs(initial)
        metadata = {'profile': PROFILE, 'lock': lock, 'inputs_sha256': initial, 'packages': packages, 'tools': tools,
                    'installed_files': inventory(prefix), 'vendor_files': vendor_files, 'vendor_features': features, 'probe': probe,
                    'cache_sha256': {row['name']: digest(reference / 'build' / row['name'] / 'CMakeCache.txt') for row in lock['repositories']}}
        for row in lock['repositories']:
            builder.verify_source(reference / 'sources' / row['name'], row, env)
        publish_manifest(reference, lock, metadata)
    print(json.dumps({'profile': PROFILE, 'prefix': str(reference / 'install'),
                      'vendor': str(reference / 'vendor' / json.loads((ROOT / 'planning/zenoh-native-lock.json').read_text())['vendor']['prefix']),
                      'source': str(reference / 'sources/gz-transport')}, indent=2))


if __name__ == '__main__':
    main()
