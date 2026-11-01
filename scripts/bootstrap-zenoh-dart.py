#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Build a separate Zenoh+DART application; print the Docker argv by default.

The explicit container action preserves the old SDK, DART and TPE installations.
It rebuilds the selected client and all three application packages, then runs one
untimed double-pendulum smoke. It does not run a benchmark or qualify Rust.
"""
import argparse
import json
import os
from pathlib import Path
import re
import shlex

from importlib.util import module_from_spec, spec_from_file_location

ROOT = Path(__file__).resolve().parents[1]


def module(name, path):
    spec = spec_from_file_location(name, ROOT / path)
    value = module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


common = module('dart_app_common', 'scripts/bootstrap-zenoh-headless.py')
dart = module('dart_app_packages', 'reference/build-dart-reference.py')
selected = common.selected
pipeline = common.pipeline
LOCK = ROOT / 'planning/zenoh-dart-app-lock.json'
PROFILE = 'gazebo-zenoh-dart-app-v1'
NAMES = ('gz-physics', 'gz-sensors', 'gz-sim')
BUILD_ROOT = Path('/reference/zenoh-dart-builds')
PREFIX = Path('/reference/zenoh-dart-install')
CLIENT = Path('/zenoh-dart-client')
MANIFEST = 'zenoh-dart-environment.json'
PROBE = 'zenoh-dart-smoke'
INPUTS = set(selected.INPUTS) | {
    'scripts/bootstrap-zenoh-dart.py', 'scripts/zenoh_dart_probe.cc',
    'scripts/bootstrap-zenoh-headless.py', 'scripts/benchmark-headless-pipeline.py',
    'scripts/native_reference.py', 'reference/build-dart-reference.py',
    'planning/zenoh-dart-app-lock.json', 'reference/headless-dart.lock.json',
    'planning/reference/headless-sdk-environment.json',
}
ISOLATED = {'mode': 'peer', 'listen': {'endpoints': ['tcp/127.0.0.1:0']},
            'connect': {'endpoints': []},
            'scouting': {'multicast': {'enabled': False}, 'gossip': {'enabled': False}},
            'transport': {'shared_memory': {'enabled': False}}}
SUCCESS = b'zenoh dart ready warmup=100 steps=100 links=7 joints=4 light=1 moving=1\n'


def lock_data():
    lock = json.loads(LOCK.read_text())
    if (lock['schema_version'] != 1 or lock['profile'] != PROFILE
            or not re.fullmatch(r'sha256:[0-9a-f]{64}', lock['base_image_id'])
            or lock['prefix'] != str(PREFIX) or lock['build_root'] != str(BUILD_ROOT)
            or lock['client_root'] != str(CLIENT) or lock['builds'] != list(NAMES)
            or lock['smoke_warmup'] != 100 or lock['smoke_steps'] != 100 or lock['smoke_timeout_seconds'] != 15):
        raise ValueError('Zenoh DART profile domain differs')
    pipeline.app_files(ROOT, lock['input_sha256'])
    pins = {row['name']: row['commit'] for row in json.loads((ROOT / 'planning/baseline.json').read_text())['repositories']}
    if lock['repositories'] != {name: pins[name] for name in NAMES}:
        raise ValueError('application source pins differ')
    if lock['fixture'] != {'repository': 'gz-sim', 'path': 'test/worlds/double_pendulum.sdf',
                           'sha256': '24e85ff944b5efe2d758fdda543e392f34d59f26297a0a87fafbb41d5a3b16aa'}:
        raise ValueError('unchanged B03 fixture is required')
    return lock


def container_command(host_client, jobs, name):
    if not 1 <= jobs <= 2 or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', name):
        raise ValueError('invalid bounded container arguments')
    lock = lock_data()
    host_client = host_client.resolve(strict=True)
    metadata = selected.validate_reference(host_client)
    archive = host_client / 'downloads' / metadata['lock']['vendor']['url'].rsplit('/', 1)[1]
    if any(':' in str(path) or ',' in str(path) for path in [ROOT, host_client, archive]):
        raise ValueError('ambiguous Docker bind path')
    return ['docker', 'run', '--pull=never', '--network=none', '--name', name,
            '-v', f'{ROOT}:{ROOT}:ro', '-v', f'{host_client}/sources:/client-sources:ro',
            '-v', f'{archive}:/zenoh-vendor.deb:ro', '-w', str(ROOT), lock['base_image_id'],
            'python3', str(Path(__file__).resolve()), '--inside-sdk', '--jobs', str(jobs)]


def base_identity(root, lock):
    if root != Path('/reference') or root.is_symlink():
        raise ValueError('requires disposable /reference container')
    manifest = root / 'dart-environment.json'
    if manifest.is_symlink() or selected.digest(manifest) != lock['base_dart_environment_sha256']:
        raise ValueError('immutable DART base manifest differs')
    base = json.loads(manifest.read_text())
    original = json.loads((ROOT / 'planning/reference/headless-sdk-environment.json').read_text())
    env = dart.build_environment(root / 'dart-install', root / 'install')
    if common.packages(env) != base['packages'] or common.tools(env) != lock['tools_sha256']:
        raise ValueError('DART base package/tool identity differs')
    dart_lock = json.loads((ROOT / 'reference/headless-dart.lock.json').read_text())
    dart.verify_metadata(ROOT, dart_lock)
    dart.verify_dart_files(Path('/'), dart_lock)
    if (base['base_environment_sha256'] != selected.digest(root / 'environment.json')
            or selected.digest(root / 'environment.json') != selected.digest(ROOT / 'planning/reference/headless-sdk-environment.json')):
        raise ValueError('original SDK identity differs')
    pipeline.client_reference.validate_reference(root / 'install', original, {}, original, pipeline.client_reference.SDK)
    pipeline.app_files(Path('/'), {name.lstrip('/'): value for name, value in base['inputs_sha256'].items()})
    if selected.inventory(root / 'dart-install') != base['installed_files']:
        raise ValueError('old DART installed inventory differs')
    for row in base['repositories']:
        source = root / 'sources' / row['name']
        selected.builder.verify_source(source, {'sha': lock['repositories'][row['name']]}, env)
        build = root / 'dart-builds' / row['name']
        if (selected.digest(build / 'CMakeCache.txt') != row['cache_sha256']
                or selected.digest(build / 'build.log') != row['build_log_sha256']):
            raise ValueError('old DART build identity differs')
        dart.check_cache((build / 'CMakeCache.txt').read_text(), source, root / 'dart-install', row['name'] == 'gz-physics')
    return base


def environment(root, metadata):
    vendor = CLIENT / 'vendor' / metadata['lock']['vendor']['prefix']
    # The signed-DART recipe clears compiler/search/loader/Python hooks.
    env = dart.build_environment(PREFIX, root / 'install')
    for key in selected.LAUNCHERS:
        env.pop(key, None)
    prefixes = [PREFIX, CLIENT / 'install', vendor, root / 'install']
    env.update(PATH=f'{CLIENT}/install/bin:/usr/bin:/bin',
               CMAKE_PREFIX_PATH=os.pathsep.join(map(str, prefixes)),
               PKG_CONFIG_PATH=os.pathsep.join(str(p / 'lib/pkgconfig') for p in prefixes),
               LD_LIBRARY_PATH=os.pathsep.join(str(p / 'lib') for p in [BUILD_ROOT / name for name in NAMES] + prefixes))
    return env


def flags_for(name, base):
    flags = next(row['flags'] for row in base['repositories'] if row['name'] == name).copy()
    flags['CMAKE_INSTALL_PREFIX'] = str(PREFIX)
    if name == 'gz-physics':
        flags.update(SKIP_dartsim='OFF', DART_DIR='/usr/share/dart/cmake')
    return flags


def configure_command(root, name, flags):
    extra = {key: '' for key in selected.LAUNCHERS}
    extra.update(CMAKE_FIND_USE_PACKAGE_REGISTRY='OFF', CMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY='OFF')
    for package in ['gz-utils', 'gz-math', 'gz-msgs', 'gz-transport']:
        directory = CLIENT / 'install/lib/cmake' / package
        if not directory.is_dir():
            raise ValueError('selected client CMake package missing')
        extra[package + '_DIR'] = str(directory)
    if name == 'gz-sim':
        for package in ['gz-physics', 'gz-sensors']:
            extra[package + '_DIR'] = str(PREFIX / 'lib/cmake' / package)
    return ['/usr/bin/cmake', '-S', str(root / 'sources' / name), '-B', str(BUILD_ROOT / name),
            '-G', 'Unix Makefiles', *(f'-D{k}={v}' for k, v in (flags | extra).items())]


def validate_cache(root, name, flags):
    path = BUILD_ROOT / name / 'CMakeCache.txt'
    cache = {}
    for line in path.read_text().splitlines():
        if line and not line.startswith(('#', '//')) and ':' in line and '=' in line:
            key, value = line.split('=', 1)
            key = key.split(':', 1)[0]
            if key in cache:
                raise ValueError('duplicate application cache key')
            cache[key] = value
    pipeline.validate_app_cache(cache, flags, flags)
    if cache.get('CMAKE_HOME_DIRECTORY') != str(root / 'sources' / name):
        raise ValueError('application build source differs')
    if '/opt/ros' in path.read_text() or any(cache.get(key, '') for key in selected.LAUNCHERS):
        raise ValueError('unqualified application search path or launcher')
    if name in {'gz-physics', 'gz-sim'}:
        dart.check_cache(path.read_text(), root / 'sources' / name, PREFIX, name == 'gz-physics')
    for package in (['gz-math'] if name == 'gz-physics' else ['gz-transport', 'gz-msgs']):
        found = [v for k, v in cache.items() if re.fullmatch(package + r'(?:\d+)?_DIR', k)]
        if not found or any(not Path(v).resolve().is_relative_to(CLIENT / 'install') for v in found):
            raise ValueError('application selected a foreign client package')
    if name == 'gz-sim':
        for package in ['gz-physics', 'gz-sensors']:
            found = [v for k, v in cache.items() if re.fullmatch(package + r'(?:\d+)?_DIR', k)]
            if not found or any(not Path(v).resolve().is_relative_to(PREFIX) for v in found):
                raise ValueError('simulator selected historical application package')


def compiler_record(repository, relative, headers):
    build = BUILD_ROOT / repository
    # DART and the retained TPE plugin have identically named feature sources.
    directory = build / 'dartsim' if repository == 'gz-physics' else build
    matches = list(directory.rglob(Path(relative).name + '.o.d'))
    if len(matches) != 1:
        raise ValueError('missing or ambiguous compiler dependency: ' + relative)
    depfile = matches[0]
    obj = depfile.with_suffix('')
    targets = [parent for parent in obj.parents if parent.name.endswith('.dir') and parent.is_relative_to(build)]
    if len(targets) != 1:
        raise ValueError('missing or ambiguous CMake object target')
    flags = targets[0] / 'flags.make'
    if any(not p.resolve().is_relative_to(build.resolve()) for p in [depfile, obj, flags]):
        raise ValueError('compiler evidence escapes its fresh build')
    target, colon, values = depfile.read_text().replace('\\\n', ' ').splitlines()[0].partition(':')
    def absolute(value):
        p = Path(value)
        return (p if p.is_absolute() else build / p).resolve()
    if not colon or len(shlex.split(target)) != 1 or absolute(shlex.split(target)[0]) != obj.resolve():
        raise ValueError('compiler dependency target differs')
    deps = {absolute(value) for value in shlex.split(values)}
    if Path('/reference/sources') / repository / relative not in deps or not set(headers).issubset(deps):
        raise ValueError('object did not use selected source/headers')
    paths = [depfile, obj, flags, *headers]
    return {str(path): selected.digest(path) for path in paths}


def compiler_records():
    transport = CLIENT / 'install/include/gz/transport16/gz/transport'
    headers = [(transport / name).resolve() for name in pipeline.TRANSPORT_HEADERS]
    if not re.search(r'^#define HAVE_ZENOH 1$', (transport / 'config.hh').read_text(), re.M):
        raise ValueError('application transport is not Zenoh ON')
    records = {key: compiler_record(repo, key.split(':', 1)[1], headers)
               for key, repo in pipeline.TRANSPORT_CONSUMERS.items()}
    records['gz-physics:dartsim/src/SimulationFeatures.cc'] = compiler_record(
        'gz-physics', 'dartsim/src/SimulationFeatures.cc',
        [Path('/usr/include/dart/config.hpp'), Path('/usr/include/dart/constraint/ContactSurface.hpp')])
    records['gz-physics:dartsim/src/SDFFeatures.cc'] = compiler_record(
        'gz-physics', 'dartsim/src/SDFFeatures.cc', [Path('/usr/include/dart/config.hpp')])
    records['gz-sim:src/systems/physics/Physics.cc'] = compiler_record(
        'gz-sim', 'src/systems/physics/Physics.cc',
        [(PREFIX / 'include/gz/physics10/gz/physics/RequestEngine.hh').resolve()])
    return records


def smoke_command(compiler, source, binary):
    target = BUILD_ROOT / 'gz-sim/src/CMakeFiles/UNIT_ServerConfig_TEST.dir'
    includes = shlex.split(next(line.split('=', 1)[1] for line in (target / 'flags.make').read_text().splitlines()
                                if line.startswith('CXX_INCLUDES =')))
    link = shlex.split((target / 'link.txt').read_text())
    libraries = [str((BUILD_ROOT / 'gz-sim/src' / v).resolve()) if v.startswith('../') else v
                 for v in link[link.index('-o') + 2:] if 'gtest' not in v]
    return [compiler, '-std=c++17', '-O3', '-DNDEBUG', '-Wall', '-Wextra', '-Werror',
            *includes, str(source), '-o', str(binary), *libraries]


def smoke_environment(root, client, env):
    libdirs = [BUILD_ROOT / name / 'lib' for name in NAMES] + [PREFIX / 'lib', CLIENT / 'install/lib',
               root / 'install/lib', CLIENT / 'vendor' / client['lock']['vendor']['prefix'] / 'lib']
    child = pipeline.app_runtime_environment(env, libdirs, root / PROBE / 'zenoh.json')
    child.update(GZ_PARTITION='zenoh-dart-smoke', GZ_IP='127.0.0.1', GZ_SIM_SERVER_CONFIG_PATH='',
                 GZ_SIM_SYSTEM_PLUGIN_PATH=str(PREFIX / 'lib/gz-sim/plugins'),
                 GZ_SIM_PHYSICS_ENGINE_PATH=str(PREFIX / 'lib/gz-physics/engine-plugins'))
    controlled = {k: v for k, v in child.items() if k.startswith(('GZ_', 'ZENOH_')) or k == 'LD_LIBRARY_PATH'}
    return child, controlled


def validate_loaded(loaded, inventories, client_metadata):
    expected = {}
    for root, inventory in inventories.items():
        expected.update(pipeline.app_library_inventory(root, inventory))
    # Old copies of app/common modules are never admissible by basename alone.
    roles = {'libgz-sim.so.': BUILD_ROOT / 'gz-sim/lib',
             'libgz-sim-physics-system.so.': PREFIX / 'lib',
             'libgz-physics.so.': BUILD_ROOT / 'gz-physics/lib',
             'libgz-physics-dartsim-plugin.so.': PREFIX / 'lib',
             'libgz-transport.so.': CLIENT / 'install/lib',
             'libgz-msgs.so.': CLIENT / 'install/lib',
             'libgz-math.so.': CLIENT / 'install/lib',
             'libgz-utils.so.': CLIENT / 'install/lib',
             'libzenohc.so': CLIENT / 'vendor' / client_metadata['lock']['vendor']['prefix'] / 'lib'}
    for role, directory in roles.items():
        rows = [Path(p) for p in loaded if Path(p).name.startswith(role)]
        if len(rows) != 1 or not rows[0].resolve().is_relative_to(directory):
            raise ValueError('missing or mixed-prefix native role: ' + role)
    for path, checksum in loaded.items():
        p = Path(path)
        if (p.name.startswith(('libgz-sim', 'libgz-physics', 'libgz-sensors'))
                and not any(p.resolve().is_relative_to(root) for root in [BUILD_ROOT, PREFIX])):
            raise ValueError('historical application module loaded')
        if (p.name.startswith(('libgz-transport', 'libgz-msgs', 'libgz-math', 'libgz-utils', 'libgz-tools'))
                and not p.resolve().is_relative_to(CLIENT / 'install/lib')):
            raise ValueError('historical common client module loaded')
        if p.name.startswith(('libgz-', 'libsdformat', 'libzenohc')):
            if checksum != expected.get(str(p.resolve())) or '/dart-install/' in str(p):
                raise ValueError('unqualified Gazebo/client library')
        if p.name.startswith(('libgz-physics-tpe', 'libgz-physics-bullet', 'libgz-physics-mujoco')):
            raise ValueError('unexpected physics backend')
    dart_files = json.loads((ROOT / 'reference/headless-dart.lock.json').read_text())['dart_files_sha256']
    for role in ['libdart.so.6.13.2', 'libdart-collision-ode.so.6.13.2', 'libdart-external-odelcpsolver.so.6.13.2']:
        paths = [Path(p) for p in loaded if Path(p).name == role]
        if len(paths) != 1 or paths[0].parent != Path('/usr/lib/x86_64-linux-gnu'):
            raise ValueError('missing selected DART dependency: ' + role)
        if loaded[str(paths[0])] != dart_files[str(paths[0]).lstrip('/')]:
            raise ValueError('mapped DART package bytes differ')
    if any(Path(p).name.startswith('libdart.so.') and Path(p).name != 'libdart.so.6.13.2' for p in loaded):
        raise ValueError('mixed DART versions')
    return loaded


def validate_reference(root=Path('/reference'), metadata=None):
    lock = lock_data()
    base_identity(root, lock)
    if metadata is None:
        path = root / MANIFEST
        if path.is_symlink():
            raise ValueError('app manifest cannot be a symlink')
        metadata = json.loads(path.read_text())
    expected_inputs = {name: selected.digest(ROOT / name) for name in INPUTS | set(lock['input_sha256'])}
    if (metadata.get('schema_version') != 1 or metadata.get('profile') != PROFILE
            or metadata.get('lock') != lock or metadata.get('inputs_sha256') != expected_inputs
            or metadata.get('base_image_id') != lock['base_image_id']):
        raise ValueError('Zenoh DART application contract differs')
    client = selected.validate_reference(CLIENT)
    env = environment(root, client)
    if (metadata['client_environment_sha256'] != selected.digest(CLIENT / 'environment.json')
            or metadata['packages'] != common.packages(env) or metadata['tools_sha256'] != lock['tools_sha256']
            or common.tools(env) != lock['tools_sha256']):
        raise ValueError('same-image client/app package or tool identity differs')
    sdk = json.loads((ROOT / 'planning/reference/headless-sdk-environment.json').read_text())
    if [r['name'] for r in metadata['repositories']] != list(NAMES):
        raise ValueError('complete fresh app build set is required')
    inventories = {PREFIX: selected.inventory(PREFIX), root / 'install': sdk['installed_files'],
                   CLIENT / 'install': client['installed_files'], CLIENT / 'vendor': client['vendor_files']}
    if inventories[PREFIX] != metadata['installed_files']:
        raise ValueError('new app installed inventory differs')
    for row in metadata['repositories']:
        name = row['name']
        source = root / 'sources' / name
        selected.builder.verify_source(source, {'sha': lock['repositories'][name]}, env)
        flags = flags_for(name, sdk)
        validate_cache(root, name, flags)
        if (row['sha'] != lock['repositories'][name] or row['flags'] != flags
                or row['configure_command'] != configure_command(root, name, flags)
                or row['cache_sha256'] != selected.digest(BUILD_ROOT / name / 'CMakeCache.txt')):
            raise ValueError('fresh app source/configuration differs')
        inventory = selected.inventory(BUILD_ROOT / name / 'lib')
        if inventory != row['library_files']:
            raise ValueError('fresh app build libraries differ')
        inventories[BUILD_ROOT / name / 'lib'] = inventory
    if metadata['compiler_inputs'] != compiler_records():
        raise ValueError('app compiler evidence differs')
    probe = root / PROBE
    smoke = metadata['smoke']
    paths = [probe / name for name in ['source.cc', 'probe', 'zenoh.json', 'stdout', 'stderr', 'maps']]
    if set(smoke['files']) != {str(p) for p in paths}:
        raise ValueError('complete untimed smoke evidence is required')
    for path in paths:
        if path.is_symlink() or selected.digest(path) != smoke['files'][str(path)]:
            raise ValueError('smoke artifact differs')
    fixture = root / 'sources/gz-sim' / lock['fixture']['path']
    if selected.digest(fixture) != lock['fixture']['sha256']:
        raise ValueError('B03 fixture bytes differ')
    if (paths[0].read_bytes() != (ROOT / 'scripts/zenoh_dart_probe.cc').read_bytes()
            or smoke['command'] != [str(paths[1]), str(fixture), str(paths[-1])]
            or smoke['compile_command'] != smoke_command(client['tools']['c++']['path'], paths[0], paths[1])
            or json.loads(paths[2].read_text()) != ISOLATED
            or paths[3].read_bytes() != SUCCESS or paths[4].read_bytes()):
        raise ValueError('smoke source, workload or explicit Zenoh input differs')
    if smoke['environment'] != smoke_environment(root, client, env)[1]:
        raise ValueError('smoke runtime selection differs')
    loaded = validate_loaded(pipeline.loaded_libraries(paths[-1]), inventories, client)
    if loaded != smoke['libraries_sha256']:
        raise ValueError('loaded library closure differs')
    return metadata


def build(root=Path('/reference'), jobs=2):
    lock = lock_data()
    if not 1 <= jobs <= 2:
        raise ValueError('jobs must be 1..2')
    base = base_identity(root, lock)
    if any(p.exists() or p.is_symlink() for p in [BUILD_ROOT, PREFIX, CLIENT, root / PROBE, root / MANIFEST]):
        raise ValueError('Zenoh DART output paths must be fresh')
    initial = {name: selected.digest(ROOT / name) for name in INPUTS | set(lock['input_sha256'])}
    env = dart.build_environment(PREFIX, root / 'install')
    cache = common.owned_source_cache(Path('/client-sources'), Path(str(CLIENT) + '-source-cache'), env)
    log = root / 'zenoh-dart-build.log'
    selected.builder.run(['python3', ROOT / 'scripts/bootstrap-zenoh-native.py', '--root', CLIENT,
                          '--local-sources', cache, '--vendor-archive', '/zenoh-vendor.deb', '--jobs', str(jobs)], log, env)
    client = selected.validate_reference(CLIENT)
    env = environment(root, client)
    BUILD_ROOT.mkdir()
    sdk = json.loads((ROOT / 'planning/reference/headless-sdk-environment.json').read_text())
    records = []
    for name in NAMES:
        selected.builder.verify_source(root / 'sources' / name, {'sha': lock['repositories'][name]}, env)
        flags = flags_for(name, sdk)
        command = configure_command(root, name, flags)
        selected.builder.run(command, log, env)
        validate_cache(root, name, flags)
        selected.builder.run(['/usr/bin/cmake', '--build', BUILD_ROOT / name, '--parallel', str(jobs)], log, env)
        selected.builder.run(['/usr/bin/cmake', '--install', BUILD_ROOT / name], log, env)
        records.append({'name': name, 'sha': lock['repositories'][name], 'flags': flags, 'configure_command': command,
                        'cache_sha256': selected.digest(BUILD_ROOT / name / 'CMakeCache.txt'),
                        'library_files': selected.inventory(BUILD_ROOT / name / 'lib')})
    compilation = compiler_records()
    fixture = root / 'sources/gz-sim' / lock['fixture']['path']
    if selected.digest(fixture) != lock['fixture']['sha256']:
        raise ValueError('B03 fixture bytes differ')
    probe = root / PROBE
    probe.mkdir()
    source, binary, config, maps = [probe / name for name in ['source.cc', 'probe', 'zenoh.json', 'maps']]
    source.write_bytes((ROOT / 'scripts/zenoh_dart_probe.cc').read_bytes())
    command = smoke_command(client['tools']['c++']['path'], source, binary)
    selected.builder.run(command, log, env)
    config.write_text(json.dumps(ISOLATED) + '\n')
    child, controlled = smoke_environment(root, client, env)
    run = [str(binary), str(fixture), str(maps)]
    common.run_smoke(run, child, probe)
    if (probe / 'stdout').read_bytes() != SUCCESS or (probe / 'stderr').read_bytes():
        raise ValueError('untimed smoke did not verify its full workload')
    inventories = {PREFIX: selected.inventory(PREFIX), root / 'install': sdk['installed_files'],
                   CLIENT / 'install': client['installed_files'], CLIENT / 'vendor': client['vendor_files']}
    inventories.update({BUILD_ROOT / r['name'] / 'lib': r['library_files'] for r in records})
    loaded = validate_loaded(pipeline.loaded_libraries(maps), inventories, client)
    metadata = {'schema_version': 1, 'profile': PROFILE, 'lock': lock, 'inputs_sha256': initial,
                'base_image_id': lock['base_image_id'], 'packages': common.packages(env), 'tools_sha256': common.tools(env),
                'client_environment_sha256': selected.digest(CLIENT / 'environment.json'), 'repositories': records,
                'compiler_inputs': compilation, 'installed_files': inventories[PREFIX],
                'smoke': {'command': run, 'compile_command': command, 'environment': controlled,
                          'files': {str(p): selected.digest(p) for p in probe.iterdir()}, 'libraries_sha256': loaded}}
    if base_identity(root, lock) != base:
        raise ValueError('DART build base changed')
    pipeline.app_files(ROOT, initial)
    selected.validate_reference(CLIENT)
    for name in NAMES:
        selected.builder.verify_source(root / 'sources' / name, {'sha': lock['repositories'][name]}, env)
    validate_reference(root, metadata)
    pending = root / (MANIFEST + '.pending')
    if pending.exists() or pending.is_symlink():
        raise ValueError('pending app manifest already exists')
    pending.write_text(json.dumps(metadata, indent=2) + '\n')
    pending.replace(root / MANIFEST)
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inside-sdk', action='store_true')
    parser.add_argument('--validate-only', action='store_true')
    parser.add_argument('--client-reference', type=Path, default=ROOT / 'artifacts/zenoh-native-reference')
    parser.add_argument('--jobs', type=int, choices=[1, 2], default=2)
    parser.add_argument('--name', default='gz-zenoh-dart-reference')
    args = parser.parse_args()
    if args.inside_sdk or args.validate_only:
        result = validate_reference() if args.validate_only else build(jobs=args.jobs)
        print(json.dumps({'profile': result['profile'], 'scope': 'untimed native articulation smoke only'}))
    else:
        print(json.dumps(container_command(args.client_reference, args.jobs, args.name), indent=2))


if __name__ == '__main__':
    main()
