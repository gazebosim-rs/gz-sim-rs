#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Measure matched TPE/IMU/process-local delivery after validating every sample."""
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import platform
import re
import shlex
import shutil
import statistics
import subprocess
import tomllib

ROOT = Path(__file__).resolve().parents[1]
reference_spec = importlib.util.spec_from_file_location('client_reference', ROOT / 'scripts/native_reference.py')
client_reference = importlib.util.module_from_spec(reference_spec)
reference_spec.loader.exec_module(client_reference)

SDK_BUILD_INPUTS = {
    'Cargo.toml', 'Cargo.lock', 'rust-toolchain.toml', 'planning/baseline.json',
    'planning/transport-reference-lock.json', 'planning/reference/headless-sdk-environment.json',
    'planning/reference/headless-sdk-build-libraries.json',
    'scripts/benchmark-headless-pipeline.py', 'scripts/native_reference.py',
    'scripts/probe-headless-imu.py', 'reference/build-reference.py',
    'crates/gz-sim-scene/tests/fixtures/headless_imu.sdf',
    *('crates/gz-sim-server/tests/' + name for name in [
        'native_pipeline_benchmark.cc', 'native_lockstep_benchmark.cc', 'native_stream_benchmark.cc']),
}

FULL_APP = 'gazebo-zenoh-headless-v1'
APP_STATIC_ROLES = ('libgz-sim.so.', 'libgz-msgs.so.', 'libgz-transport.so.', 'libzenohc.so')
APP_BUILD_DIRECTORIES = {'gz-physics': 'builds/gz-physics',
                         'gz-sensors': 'builds/zenoh-gz-sensors',
                         'gz-sim': 'builds/zenoh-gz-sim'}
APP_BUILD_INPUTS = {
    'scripts/benchmark-headless-pipeline.py', 'scripts/bootstrap-zenoh-native.py',
    'scripts/zenoh_headless_probe.cc', 'scripts/bootstrap-zenoh-headless.py',
    'planning/zenoh-native-lock.json',
    'planning/zenoh-native-packages.json', 'planning/baseline.json',
    'reference/build-reference.py', 'reference/headless-tpe.lock.json',
    'planning/reference/headless-sdk-environment.json',
    'planning/reference/headless-sdk-build-libraries.json', 'scripts/native_reference.py',
    'crates/gz-sim-scene/tests/fixtures/headless_imu.sdf',
    *('crates/gz-sim-server/tests/' + name for name in [
        'native_pipeline_benchmark.cc', 'native_lockstep_benchmark.cc', 'native_stream_benchmark.cc']),
}
TRANSPORT_HEADERS = ('config.hh', 'Node.hh', 'detail/Node.hh',
                     'SubscriptionHandler.hh', 'RepHandler.hh', 'ReqHandler.hh')
TRANSPORT_CONSUMERS = {
    'gz-sensors:src/Sensor.cc': 'gz-sensors',
    'gz-sim:src/SimulationRunner.cc': 'gz-sim',
    'gz-sim:src/systems/scene_broadcaster/SceneBroadcaster.cc': 'gz-sim',
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def cache_value(build, key):
    prefix = key + ':'
    return next(line.split('=', 1)[1] for line in (build / 'CMakeCache.txt').read_text().splitlines()
                if line.startswith(prefix))


def benchmark_source_roots(profile, rust=None, app=None):
    if profile != FULL_APP and (rust is not None or app is not None):
        raise ValueError('explicit source contexts require the Zenoh full-app profile')
    roots = []
    for path in [ROOT if rust is None else rust, ROOT if app is None else app]:
        if path.is_symlink() or not path.is_dir():
            raise ValueError('benchmark source root must be a real directory')
        roots.append(path.resolve())
    return tuple(roots)


def check_benchmark_context_inputs(rust, app):
    # Both programs must still consume the same fixture and native driver bytes.
    for name in ['crates/gz-sim-scene/tests/fixtures/headless_imu.sdf',
                 *('crates/gz-sim-server/tests/' + value for value in [
                     'native_pipeline_benchmark.cc', 'native_lockstep_benchmark.cc',
                     'native_stream_benchmark.cc'])]:
        left = client_reference.contained_file(rust, name)
        right = client_reference.contained_file(app, name)
        if digest(left) != digest(right):
            raise ValueError('benchmark source contexts differ: ' + name)


def measurement_tool_inputs():
    return {name: digest(ROOT / name) for name in [
        'scripts/benchmark-headless-pipeline.py', 'scripts/native_reference.py',
        'scripts/probe-headless-imu.py', 'reference/build-reference.py']}


def validate_rust_build(record_path, build_json, binary, profile=None, *, source_root=None):
    """Check the retained host build record before executing in the compiler-free SDK."""
    root = ROOT if source_root is None else source_root
    record = json.loads(record_path.read_text())
    for key in ['source_tree', 'git_base']:
        if not re.fullmatch('[0-9a-f]{40}', record[key]):
            raise ValueError('invalid Rust source identity')
    if record['build_json_sha256'] != digest(build_json) or record['binary_sha256'] != digest(binary):
        raise ValueError('Rust build or executable identity differs')
    with binary.open('rb') as executable:
        if executable.read(4) != b'\x7fELF':
            raise ValueError('Rust SDK benchmark requires an ELF executable')
    messages = [json.loads(line) for line in build_json.read_text().splitlines()]
    if [row.get('success') for row in messages if row.get('reason') == 'build-finished'] != [True]:
        raise ValueError('Rust build did not complete successfully')
    channel = tomllib.loads((root / 'rust-toolchain.toml').read_text())['toolchain']['channel']
    if not record['rustc_vv'].startswith('rustc ' + channel + ' '):
        raise ValueError('Rust compiler version differs from the pinned toolchain')
    required = (APP_BUILD_INPUTS if profile == FULL_APP else SDK_BUILD_INPUTS).copy()
    if profile == FULL_APP:
        required |= {'.cargo/config.toml', 'Cargo.toml', 'Cargo.lock', 'rust-toolchain.toml'}
        required |= {str(path.relative_to(root)) for path in (root / 'tests/support').rglob('*.rs')}
        manifest = tomllib.loads((root / 'Cargo.toml').read_text())
        patch_path = Path(manifest['patch']['crates-io']['prost-derive']['path'])
        if patch_path.is_absolute() or '..' in patch_path.parts:
            raise ValueError('Rust derive patch escapes source root')
        for directory in [root / 'crates/gz-sim-proto', root / patch_path]:
            if directory.is_symlink() or not directory.is_dir() or not directory.resolve().is_relative_to(root.resolve()):
                raise ValueError('Rust generation/derive input must be a real directory')
            required |= {str(path.relative_to(root)) for path in directory.rglob('*')
                         if path.is_file() and (path.suffix in {'.proto', '.rs', '.toml', '.json'}
                                                or path.name in {'LICENSE', 'NOTICE'})}
    required |= {str(path.relative_to(root)) for path in (root / 'crates').rglob('*.rs')}
    required |= {str(path.relative_to(root)) for path in (root / 'crates').rglob('Cargo.toml')}
    sources = record['source_sha256']
    if not required.issubset(sources):
        raise ValueError('Rust build record omits required source inputs')
    for name, expected in sources.items():
        path = client_reference.contained_file(root, name)
        if path.is_symlink() or digest(path) != expected:
            raise ValueError('Rust build source changed: ' + name)
    return record


def validate_sdk_builds(builds, prefix, sdk):
    registry = json.loads((ROOT/'planning/reference/headless-sdk-build-libraries.json').read_text())
    if registry['sdk_environment_sha256'] != digest(ROOT/'planning/reference/headless-sdk-environment.json'):
        raise ValueError('native build registry belongs to a different SDK')
    if set(registry['builds']) != {'gz-'+name for name in builds}:
        raise ValueError('native SDK build set differs')
    root = prefix.parent
    pins = {row['name']: row for row in sdk['repositories']}
    libraries = {str((prefix/name).resolve()): entry['sha256']
                 for name,entry in sdk['installed_files'].items() if 'sha256' in entry and '.so' in Path(name).name}
    for name,directory in builds.items():
        key = 'gz-'+name
        expected = registry['builds'][key]
        if expected['directory'] != 'builds/'+key or expected['source'] != 'sources/'+key:
            raise ValueError('unexpected native SDK layout')
        if directory != (root/expected['directory']).resolve() or Path(cache_value(directory,'CMAKE_HOME_DIRECTORY')).resolve() != (root/expected['source']).resolve():
            raise ValueError('native build or source root differs from SDK')
        cache_hash = digest(directory/'CMakeCache.txt')
        if cache_hash != expected['cmake_cache_sha256'] or cache_hash != pins[key]['cmake_cache_sha256']:
            raise ValueError('native CMake cache differs from SDK')
        files = client_reference.builder.files_under(directory/'lib')
        if files != expected['library_files']:
            raise ValueError('native build library inventory differs from SDK')
        libraries.update({str((directory/'lib'/path).resolve()): entry['sha256']
                          for path,entry in files.items() if 'sha256' in entry and '.so' in Path(path).name})
    target = builds['sim']/'src/CMakeFiles/UNIT_ServerConfig_TEST.dir'
    if registry['server_compiler_inputs'] != {name: digest(target/name) for name in ['flags.make','link.txt']}:
        raise ValueError('native compiler inputs differ from SDK')
    return libraries


def validate_sdk_loaded_libraries(loaded, expected):
    for path,actual in loaded.items():
        if Path(path).name.startswith(('libgz-', 'libsdformat')):
            canonical = str(Path(path).resolve())
            if canonical not in expected or expected[canonical] != actual:
                raise ValueError('unqualified loaded SDK library: '+path)


def require_native_libraries(loaded, required, qualified=None):
    for name,directory in required.items():
        found = [Path(path) for path in loaded if Path(path).name.startswith(name)]
        if len(found) != 1:
            raise ValueError('missing or duplicate loaded library: '+name)
        path = found[0]
        if qualified is None:
            valid = path.resolve() == (directory/path.name).resolve()
        else:
            canonical = str(path.resolve())
            valid = canonical in qualified and qualified[canonical] == loaded[str(path)]
        if not valid:
            raise ValueError('unqualified loaded library location: '+name)


def app_directory(root, name):
    relative = Path(name)
    if relative.is_absolute() or '..' in relative.parts:
        raise ValueError('app directory escapes reference')
    path = root / relative
    if path.is_symlink() or not path.is_dir() or not path.resolve().is_relative_to(root.resolve()):
        raise ValueError('app directory is missing or escapes reference')
    return path


def app_files(root, expected):
    if not expected:
        raise ValueError('empty app file identity')
    for name, checksum in expected.items():
        path = client_reference.contained_file(root, name)
        if path.is_symlink() or digest(path) != checksum:
            raise ValueError('app file changed: ' + name)


def validate_transport_compilation(app, client_prefix, records, header_hashes):
    headers = {str(Path('include/gz/transport16/gz/transport') / name)
               for name in TRANSPORT_HEADERS}
    if set(header_hashes) != headers or set(records) != set(TRANSPORT_CONSUMERS):
        raise ValueError('app transport compiler evidence is incomplete')
    app_files(client_prefix, header_hashes)
    header_paths = {str((client_prefix / name).resolve()) for name in headers}
    config = client_prefix / 'include/gz/transport16/gz/transport/config.hh'
    if not re.search(r'^#define HAVE_ZENOH 1$', config.read_text(), re.MULTILINE):
        raise ValueError('app transport headers are not Zenoh enabled')
    for name, record in records.items():
        app_files(app, record['files'])
        if len(record['files']) != 3 or set(record['files']) != {record['object'], record['depfile'], record['flags']}:
            raise ValueError('app compiler inputs differ')
        build = (app / APP_BUILD_DIRECTORIES[TRANSPORT_CONSUMERS[name]]).resolve()
        for field in ['object', 'depfile', 'flags']:
            if not (app / record[field]).resolve().is_relative_to(build):
                raise ValueError('app compiler input is outside its build')
        # GCC emits one dependency rule, possibly continued, followed by optional
        # phony header rules. Refuse a missing rule instead of guessing includes.
        first = (app / record['depfile']).read_text().replace(chr(92) + chr(10), ' ').splitlines()[0]
        target, colon, dependencies = first.partition(':')
        if not colon or len(shlex.split(target)) != 1:
            raise ValueError('unsupported app compiler dependency rule')
        def absolute(value):
            path = Path(value)
            return (path if path.is_absolute() else build / path).resolve()
        if absolute(shlex.split(target)[0]) != (app / record['object']).resolve():
            raise ValueError('app compiler dependency target differs')
        dependencies = {str(absolute(value)) for value in shlex.split(dependencies)}
        source = app / 'sources' / name.replace(':', '/', 1)
        if str(source.resolve()) not in dependencies or not header_paths.issubset(dependencies):
            raise ValueError('app objects did not use selected source/Zenoh headers')


def validate_app_cache(cache, recorded_flags, required_flags):
    if recorded_flags != required_flags or any(cache.get(key) != value for key, value in required_flags.items()):
        raise ValueError('app build flags differ')
    for key in ['CMAKE_C_FLAGS', 'CMAKE_CXX_FLAGS', 'CMAKE_EXE_LINKER_FLAGS', 'CMAKE_SHARED_LINKER_FLAGS']:
        if cache.get(key, ''):
            raise ValueError('app compiler/linker flags are not qualified')
    for key in ['CMAKE_C_FLAGS_RELEASE', 'CMAKE_CXX_FLAGS_RELEASE']:
        if cache.get(key) != '-O3 -DNDEBUG':
            raise ValueError('app Release optimization differs')


def app_library_inventory(root, files):
    return {str((root / name).resolve()): row['sha256'] for name, row in files.items()
            if 'sha256' in row and '.so' in Path(name).name}


def validate_app_loaded_libraries(loaded, expected, *, all_libraries=False):
    for path, checksum in loaded.items():
        if all_libraries or Path(path).name.startswith(('libgz-', 'libsdformat', 'libzenohc')):
            canonical = str(Path(path).resolve())
            if canonical not in expected or expected[canonical] != checksum:
                raise ValueError('unqualified loaded app library: ' + path)


def validate_app_linkage(text, expected, required):
    if 'not found' in text:
        raise ValueError('unresolved app benchmark dependency')
    loaded = {}
    for line in text.splitlines():
        if '=>' in line:
            path = Path(line.split('=>', 1)[1].split()[0]).resolve(strict=True)
            loaded[str(path)] = digest(path)
    validate_app_loaded_libraries(loaded, expected, all_libraries=True)
    require_native_libraries(loaded, required, expected)
    return loaded


def app_runtime_environment(environment, libdirs, config):
    env = {key: value for key, value in environment.items()
           if not key.startswith(('GZ_', 'ZENOH_')) and key not in {'LD_PRELOAD', 'LD_AUDIT'}}
    env.update(LD_LIBRARY_PATH=os.pathsep.join(map(str, libdirs)),
               GZ_TRANSPORT_IMPLEMENTATION='zenoh', ZENOH_CONFIG=str(config))
    return env


def validate_app_smoke(app, smoke, expected_libraries, required, *, source_root=None):
    root = ROOT if source_root is None else source_root
    names = {'source', 'binary', 'config', 'stdout', 'stderr', 'maps'}
    if set(smoke['files']) != names or smoke['status'] != 0:
        raise ValueError('app smoke evidence is incomplete')
    paths = {}
    for name, record in smoke['files'].items():
        app_files(app, {record['path']: record['sha256']})
        paths[name] = app / record['path']
    if paths['source'].read_bytes() != (root / 'scripts/zenoh_headless_probe.cc').read_bytes():
        raise ValueError('app smoke source differs')
    command = [str(paths['binary']), str(root / 'crates/gz-sim-scene/tests/fixtures/headless_imu.sdf'), str(paths['maps'])]
    if smoke.get('command') != command or smoke.get('timeout_seconds') != 15:
        raise ValueError('app smoke launch is not bounded or input-bound')
    isolated = {'mode': 'peer', 'listen': {'endpoints': ['tcp/127.0.0.1:0']},
                'connect': {'endpoints': []},
                'scouting': {'multicast': {'enabled': False}, 'gossip': {'enabled': False}},
                'transport': {'shared_memory': {'enabled': False}}}
    if json.loads(paths['config'].read_text()) != isolated:
        raise ValueError('app smoke configuration differs')
    # These are verification flags for one valid common timestamp, not counts of
    # startup traffic or a benchmark duration. The probe itself owns validation.
    if (paths['stdout'].read_bytes() != b'zenoh headless ready clock=1 pose=1 imu=1 sim_ns=1000000\n'
            or paths['stderr'].read_bytes()):
        raise ValueError('app smoke did not verify all committed streams')
    loaded = loaded_libraries(paths['maps'])
    if loaded != smoke['libraries_sha256']:
        raise ValueError('app smoke loaded library identity changed')
    validate_app_loaded_libraries(loaded, expected_libraries)
    require_native_libraries(loaded, required, expected_libraries)
    return loaded


def validate_app_reference(app, builds, prefix, dependency_prefix, metadata=None, *, source_root=None):
    root = ROOT if source_root is None else source_root
    if app.is_symlink() or not app.is_dir():
        raise ValueError('app reference root must be a real directory')
    if metadata is None:
        environment = client_reference.contained_file(app, 'environment.json')
        if environment.is_symlink():
            raise ValueError('app manifest must not be a symlink')
        metadata = json.loads(environment.read_text())
    if metadata.get('schema_version') != 1 or metadata.get('profile') != FULL_APP:
        raise ValueError('a Zenoh full-app profile is required, not a client SDK')
    base_path = root / 'planning/reference/headless-sdk-environment.json'
    if metadata['base_sdk_environment_sha256'] != digest(base_path):
        raise ValueError('app base SDK identity differs')
    base = json.loads(base_path.read_text())
    client = metadata['client_reference']
    client_root = Path(client['root'])
    if not client_root.is_absolute() or digest(client_root / 'environment.json') != client['environment_sha256']:
        raise ValueError('app selected-client identity differs')
    if not APP_BUILD_INPUTS.issubset(metadata['inputs_sha256']):
        raise ValueError('app build omits required source inputs')
    app_files(root, metadata['inputs_sha256'])
    spec = importlib.util.spec_from_file_location('app_zenoh_reference', root / 'scripts/bootstrap-zenoh-native.py')
    selected = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(selected)
    client_metadata = selected.validate_reference(client_root)
    client_prefix = client_root / 'install'
    vendor = client_root / 'vendor' / client_metadata['lock']['vendor']['prefix']
    if prefix != client_prefix.resolve() or dependency_prefix != (app / 'install').resolve():
        raise ValueError('app command selected different dependency roots')
    env = selected.build_environment(client_prefix, vendor)
    packages = selected.builder.output(
        ['/usr/bin/dpkg-query', '-W', '-f=${binary:Package}\t${Version}\t${Architecture}\n'], env)
    if metadata['packages'] != base['packages'] or packages != metadata['packages']:
        raise ValueError('app packages differ from the immutable SDK base')
    tools = {name: digest(Path(shutil.which(name, path=env['PATH'])).resolve())
             for name in ['c++', 'cmake', 'git', 'python3']}
    if tools != metadata['tools_sha256'] or tools != base['tools_sha256']:
        raise ValueError('app tool identity differs')
    if tools['c++'] != client_metadata['tools']['c++']['sha256']:
        raise ValueError('app compiler differs from selected Zenoh compiler')
    installed = app_directory(app, 'install')
    if selected.inventory(installed) != metadata['installed_files']:
        raise ValueError('app installed inventory changed')
    pins = {row['name']: row for row in base['repositories']}
    records = {row['name']: row for row in metadata['repositories']}
    if len(metadata['repositories']) != 3 or set(records) != {'gz-sim', 'gz-sensors', 'gz-physics'} or set(metadata['builds']) != set(records):
        raise ValueError('app build set differs')
    libraries = app_library_inventory(installed, metadata['installed_files'])
    for name, record in records.items():
        source = app_directory(app, 'sources/' + name)
        build = app_directory(app, APP_BUILD_DIRECTORIES[name])
        if builds[name.removeprefix('gz-')] != build.resolve() or record['sha'] != pins[name]['sha']:
            raise ValueError('app source/build selection differs')
        selected.builder.verify_source(source, record, env)
        registry = metadata['builds'][name]
        if registry['source'] != 'sources/' + name or registry['directory'] != APP_BUILD_DIRECTORIES[name]:
            raise ValueError('app build registry layout differs')
        checksum = digest(build / 'CMakeCache.txt')
        if checksum != record['cmake_cache_sha256'] or checksum != registry['cmake_cache_sha256']:
            raise ValueError('app CMake cache changed')
        cache = selected.cache_values(build / 'CMakeCache.txt')
        flags = pins[name]['flags'] | {'CMAKE_INSTALL_PREFIX': str(installed)}
        validate_app_cache(cache, record['flags'], flags)
        if cache.get('CMAKE_HOME_DIRECTORY') != str(source):
            raise ValueError('app source cache differs')
        for key in selected.LAUNCHERS:
            if cache.get(key, ''):
                raise ValueError('app compiler/linker launcher is not qualified')
        if name != 'gz-physics':
            dirs = [value for key, value in cache.items() if re.fullmatch(r'gz-transport(?:16)?_DIR', key)]
            if not dirs or any(not Path(value).resolve().is_relative_to(client_prefix.resolve()) for value in dirs):
                raise ValueError('app CMake selected a foreign transport')
        files = selected.inventory(build / 'lib')
        if files != registry['library_files']:
            raise ValueError('app build library inventory changed')
        libraries.update(app_library_inventory(build / 'lib', files))
    target = builds['sim'] / 'src/CMakeFiles/UNIT_ServerConfig_TEST.dir'
    if metadata['server_compiler_inputs'] != {name: digest(target / name) for name in ['flags.make', 'link.txt']}:
        raise ValueError('app benchmark compiler inputs changed')
    validate_transport_compilation(app, client_prefix, metadata['transport_compiler_inputs'], metadata['transport_headers'])
    # The selected client owns the six common Gazebo libraries. Do not allow an
    # old installed Transport/Msgs copy merely because it appears in app inventory.
    common = ('libgz-transport', 'libgz-msgs', 'libgz-math', 'libgz-utils', 'libgz-tools')
    libraries = {name: value for name, value in libraries.items() if not Path(name).name.startswith(common)}
    libraries.update(app_library_inventory(client_prefix, client_metadata['installed_files']))
    libraries.update(app_library_inventory(client_root / 'vendor', client_metadata['vendor_files']))
    required = {
        'libgz-sim.so.': builds['sim'] / 'lib',
        'libgz-sim-physics-system.so.': builds['sim'] / 'lib',
        'libgz-sim-imu-system.so.': builds['sim'] / 'lib',
        'libgz-sim-scene-broadcaster-system.so.': builds['sim'] / 'lib',
        'libgz-physics-tpe-plugin.so.': builds['physics'] / 'lib',
        'libgz-physics-tpelib.so.': builds['physics'] / 'lib',
        'libgz-sensors.so.': builds['sensors'] / 'lib',
        'libgz-sensors-imu.so.': builds['sensors'] / 'lib',
        'libgz-transport.so.': client_prefix / 'lib', 'libgz-msgs.so.': client_prefix / 'lib',
        'libzenohc.so': vendor / 'lib',
    }
    smoke_libraries = validate_app_smoke(app, metadata['smoke'], libraries, required, source_root=root)
    libraries.update(smoke_libraries)
    return {'metadata': metadata, 'client_metadata': client_metadata,
            'compiler': client_metadata['tools']['c++']['path'],
            'environment': env, 'libraries': libraries, 'vendor': vendor,
            'client_root': client_root}


def decode(output, steps, fixture_sha256, *, warmup_force_x=None, streams=False, request_period_us=None):
    rows = [json.loads(line.split('PIPELINE ', 1)[1]) for line in output.splitlines()
            if 'PIPELINE ' in line]
    if len(rows) != 1:
        raise ValueError('expected exactly one validated pipeline result')
    row = rows[0]
    if any(type(row[key]) is not int for key in ['steps', 'nanos', 'logical_bytes', 'time_sum']) or row['steps'] != steps or row['nanos'] <= 0 or row['logical_bytes'] <= 0:
        raise ValueError('invalid measured work or duration')
    if row['time_sum'] != steps * (steps + 3) // 2 * 1_000_000:
        raise ValueError('sample timestamps/checksum differ')
    if not math.isclose(row['force_sum'], 9.81 * steps, rel_tol=1e-12, abs_tol=1e-8):
        raise ValueError('physical checksum differs')
    if not math.isfinite(row['final_x']) or not abs(row['final_x'] - (steps + 1) * 0.001) < 1e-9:
        raise ValueError('physical motion checksum differs')
    if hashlib.sha256(bytes.fromhex(row['fixture_hex'])).hexdigest() != fixture_sha256:
        raise ValueError('application consumed a different SDF fixture')
    if warmup_force_x is not None and (not math.isfinite(row['warmup_force_x']) or abs(row['warmup_force_x'] - warmup_force_x) >= 1e-9):
        raise ValueError('velocity replacement warmup differs from native')
    if streams:
        for key in ['wall_nanos', 'pacing_wait_nanos', 'request_period_us', 'clock_count', 'pose_count', 'clock_time_sum', 'pose_time_sum',
                    'clock_bytes', 'pose_bytes', 'pose_bytes_without_id_values']:
            if type(row[key]) is not int or row[key] <= 0:
                raise ValueError('invalid stream work: ' + key)
        period = row['request_period_us']
        if request_period_us is not None and period != request_period_us:
            raise ValueError('application ignored requested cadence')
        if not 20000 <= period <= 1000000 or row['wall_nanos'] < steps * period * 1000:
            raise ValueError('wrong request cadence or elapsed wall time')
        if row['wall_nanos'] < row['nanos'] + row['pacing_wait_nanos']:
            raise ValueError('latency/pacing intervals exceed elapsed wall time')
        if row['clock_count'] != steps or row['pose_count'] != steps:
            raise ValueError('incomplete clock/pose observations')
        if row['clock_time_sum'] != row['time_sum'] or row['pose_time_sum'] != row['time_sum']:
            raise ValueError('stream timestamps differ')
        ids = row['pose_ids']
        if not isinstance(ids, dict) or set(ids) != {'ball', 'ground', 'body', 'base'}:
            raise ValueError('wrong pose identity roles')
        if any(type(value) is not int or not 0 < value <= 0xffffffff for value in ids.values()) or len(set(ids.values())) != 4:
            raise ValueError('invalid pose identities')
        id_bytes = steps * sum((value.bit_length() + 6) // 7 for value in ids.values())
        if row['pose_bytes'] - row['pose_bytes_without_id_values'] != id_bytes:
            raise ValueError('pose normalization removed more than opaque ID values')
    return row


def loaded_libraries(maps):
    paths = {line.split(maxsplit=5)[5] for line in maps.read_text().splitlines()
             if len(line.split(maxsplit=5)) == 6 and line.split(maxsplit=5)[5].startswith('/')}
    return {path: digest(Path(path)) for path in sorted(paths)
            if Path(path).is_file() and '.so' in Path(path).name}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['sim-build', 'physics-build', 'sensors-build', 'dependency-prefix',
                 'reference-prefix', 'rust-build-json', 'work-dir', 'output']:
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--steps', type=int, default=1000)
    parser.add_argument('--samples', type=int, default=7)
    parser.add_argument('--cpu', type=int)
    parser.add_argument('--path', choices=['direct', 'server', 'streams'], default='direct')
    parser.add_argument('--request-period-us', type=int, default=20000)
    parser.add_argument('--executor', choices=['default', 'single', 'parallel'], default='default')
    parser.add_argument('--reference-profile', choices=[client_reference.LEGACY, client_reference.SDK, FULL_APP], default=client_reference.LEGACY)
    parser.add_argument('--rust-build-info', type=Path)
    parser.add_argument('--app-reference', type=Path)
    parser.add_argument('--app-source-root', type=Path, help='source view bound to the immutable native app inputs and smoke argv')
    parser.add_argument('--rust-source-root', type=Path, help='actual source export recorded by the Rust Cargo build')
    args = parser.parse_args()
    if args.reference_profile == client_reference.SDK and args.rust_build_info is None:
        raise ValueError('the SDK profile requires the retained Rust build record')
    if args.reference_profile == FULL_APP and args.rust_build_info is None:
        raise ValueError('the Zenoh app profile requires the retained Rust build record')
    if args.reference_profile == FULL_APP and args.app_reference is None:
        raise ValueError('the Zenoh profile requires a full-app reference')
    if args.reference_profile != FULL_APP and args.app_reference is not None:
        raise ValueError('an app reference requires the explicit Zenoh full-app profile')
    rust_root, app_source_root = benchmark_source_roots(
        args.reference_profile, args.rust_source_root, args.app_source_root)
    tool_inputs = measurement_tool_inputs() if args.reference_profile == FULL_APP else None
    if args.path == 'streams' and not 20000 <= args.request_period_us <= 1000000:
        raise ValueError('native pose/info requires declared request period of20ms or more')
    if not 1 <= args.steps <= 1000 or args.samples < 7:
        raise ValueError('require 1..1000 steps and at least seven measured pairs')
    allowed = os.sched_getaffinity(0)
    cpu = min(allowed) if args.cpu is None else args.cpu
    if cpu not in allowed:
        raise ValueError('CPU is outside this process affinity')
    work = args.work_dir.resolve()
    work.mkdir(parents=True, exist_ok=True)
    if args.reference_profile in {client_reference.SDK, FULL_APP} and any(work.iterdir()):
        raise ValueError('the SDK profile requires a new measurement directory')
    builds = {name: getattr(args, name.replace('-', '_') + '_build').resolve()
              for name in ['sim', 'physics', 'sensors']}
    # Reuse the already reviewed raw-source checker; no native source is modified.
    spec = importlib.util.spec_from_file_location('native_reference', ROOT / 'scripts/probe-headless-imu.py')
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    native_inputs = app_source_root if args.reference_profile == FULL_APP else ROOT
    lock = {item['name']: item['commit'] for item in
            json.loads((native_inputs / 'planning/baseline.json').read_text())['repositories']}
    source_files = {
        'sim': ['src/Server.cc', 'src/ServerPrivate.cc', 'src/SimulationRunner.cc', 'src/systems/physics/Physics.cc', 'src/systems/imu/Imu.cc'],
        'physics': ['tpe/lib/src/Engine.cc', 'tpe/lib/src/CollisionDetector.cc', 'tpe/plugin/src/SimulationFeatures.cc'],
        'sensors': ['src/Sensor.cc', 'src/ImuSensor.cc'],
    }
    if args.path == 'streams':
        source_files['sim'].append('src/systems/scene_broadcaster/SceneBroadcaster.cc')
    native_sources, build_settings = {}, {}
    for name, build in builds.items():
        if cache_value(build, 'CMAKE_BUILD_TYPE') != 'Release':
            raise ValueError(f'{name} must be a Release build')
        source = Path(cache_value(build, 'CMAKE_HOME_DIRECTORY')).resolve()
        native_sources[name] = helper.checked_sources(source, lock['gz-' + name], source_files[name])
        build_settings[name] = dict(source=str(source), cache_sha256=digest(build / 'CMakeCache.txt'),
                                    release_flags=cache_value(build, 'CMAKE_CXX_FLAGS_RELEASE'))
    prefix = args.reference_prefix.resolve()
    app_root = args.app_reference.absolute() if args.app_reference is not None else None
    app = validate_app_reference(app_root, builds, prefix, args.dependency_prefix.resolve(), source_root=app_source_root) if app_root is not None else None
    if app is not None:
        check_benchmark_context_inputs(rust_root, app_source_root)
        metadata_path = app_root / 'environment.json'
        metadata_bytes = metadata_path.read_bytes()
        reference_identity = {'profile': FULL_APP, 'reference_root': str(app_root), 'environment_sha256': digest(metadata_path),
                              'base_sdk_environment_sha256': app['metadata']['base_sdk_environment_sha256'],
                              'client_reference': app['metadata']['client_reference']}
        sdk = None
        compile_env = app['environment']
        sdk_libraries = app['libraries']
        sdk_registry_sha256 = None
    else:
        metadata_path = prefix.parent / 'environment.json'
        metadata_bytes = metadata_path.read_bytes()
        reference = json.loads(metadata_bytes)
        expected_lock = json.loads((ROOT / 'planning/transport-reference-lock.json').read_text())
        sdk = json.loads((ROOT / 'planning/reference/headless-sdk-environment.json').read_text()) if args.reference_profile == client_reference.SDK else None
        reference_identity = client_reference.validate_reference(prefix, reference, expected_lock, sdk, args.reference_profile)
        compile_env = client_reference.builder.build_environment(prefix) if sdk is not None else os.environ.copy()
        sdk_libraries = validate_sdk_builds(builds,prefix,sdk) if sdk is not None else None
        sdk_registry_sha256 = digest(ROOT/'planning/reference/headless-sdk-build-libraries.json') if sdk is not None else None
        if sdk is not None and args.dependency_prefix.resolve() != prefix:
            raise ValueError('the SDK profile requires its qualified dependency prefix')
    rust_candidates = []
    for line in args.rust_build_json.read_text().splitlines():
        try:
            item = json.loads(line)
        except json.JSONDecodeError:
            continue
        if item.get('reason') == 'compiler-artifact' and item.get('executable'):
            if item['target']['name'] == 'gz_sim_server' and item['profile']['test']:
                if item['profile']['opt_level'] != '3' or item['profile']['debug_assertions']:
                    raise ValueError('Rust test executable is not the release profile')
                if (sdk is not None or app is not None) and Path(item['target']['src_path']).resolve() != rust_root / 'crates/gz-sim-server/src/lib.rs':
                    raise ValueError('Rust executable belongs to a different source export')
                rust_candidates.append(Path(item['executable']).resolve())
    if len(set(rust_candidates)) != 1:
        raise ValueError('expected exactly one release server test executable')
    rust = rust_candidates[0]
    rust_sha256 = digest(rust)
    rust_build = validate_rust_build(args.rust_build_info, args.rust_build_json, rust, args.reference_profile, source_root=rust_root) if sdk is not None or app is not None else None
    rust_build_info_sha256 = digest(args.rust_build_info) if rust_build is not None else None
    fixture = rust_root / 'crates/gz-sim-scene/tests/fixtures/headless_imu.sdf'
    fixture_sha256 = digest(fixture)
    target = builds['sim'] / 'src/CMakeFiles/UNIT_ServerConfig_TEST.dir'
    flags = (target / 'flags.make').read_text()
    includes = shlex.split(next(line.split('=', 1)[1] for line in flags.splitlines()
                                if line.startswith('CXX_INCLUDES =')))
    link = shlex.split((target / 'link.txt').read_text())
    libraries = [str((builds['sim'] / 'src' / item).resolve()) if item.startswith('../') else item
                 for item in link[link.index('-o') + 2:] if 'gtest' not in item]
    source = rust_root / ('crates/gz-sim-server/tests/native_stream_benchmark.cc' if args.path == 'streams' else 'crates/gz-sim-server/tests/native_lockstep_benchmark.cc' if args.path == 'server' else 'crates/gz-sim-server/tests/native_pipeline_benchmark.cc')
    source_bytes = source.read_bytes()
    captured_source = work / 'native-pipeline-source.cc'
    captured_source.write_bytes(source_bytes)
    compiler = Path(app['compiler'] if app is not None else shutil.which(link[0], path=compile_env.get('PATH', '')) or link[0]).resolve(strict=True)
    compiler_sha256 = digest(compiler)
    if sdk is not None and compiler_sha256 != reference_identity['expected_compiler_sha256']:
        raise ValueError('native benchmark compiler differs from the SDK')
    compiler_version = subprocess.check_output([str(compiler), '--version'], env=compile_env, text=True)
    native = work / 'native-pipeline'
    command = [str(compiler), '-std=c++17', '-O3', '-DNDEBUG', '-Wall', '-Wextra', '-Werror',
               *includes, str(captured_source), '-o', str(native), *libraries]
    subprocess.run(command, cwd=builds['sim'] / 'src', env=compile_env, check=True)
    binary_sha256 = dict(native=digest(native), rust=rust_sha256)

    config_path = None
    zenoh_config_sha256 = None

    def recheck_inputs():
        if config_path is not None and digest(config_path) != zenoh_config_sha256:
            raise ValueError('benchmark Zenoh configuration changed')
        if metadata_path.read_bytes() != metadata_bytes or source.read_bytes() != source_bytes or captured_source.read_bytes() != source_bytes:
            raise ValueError('native benchmark input changed')
        if app is not None:
            check_benchmark_context_inputs(rust_root, app_source_root)
            if measurement_tool_inputs() != tool_inputs:
                raise ValueError('measurement tool source changed')
            if validate_app_reference(app_root, builds, prefix, args.dependency_prefix.resolve(), source_root=app_source_root) != app:
                raise ValueError('native Zenoh app reference changed')
        elif client_reference.validate_reference(prefix, reference, expected_lock, sdk, args.reference_profile) != reference_identity:
            raise ValueError('native benchmark reference changed')
        if digest(compiler) != compiler_sha256:
            raise ValueError('native benchmark compiler changed')
        if sdk is not None and (digest(ROOT/'planning/reference/headless-sdk-build-libraries.json') != sdk_registry_sha256 or validate_sdk_builds(builds,prefix,sdk) != sdk_libraries):
            raise ValueError('native SDK build identity changed')
        if dict(native=digest(native), rust=digest(rust)) != binary_sha256 or digest(fixture) != fixture_sha256:
            raise ValueError('benchmark executable or fixture changed')
        if rust_build is not None and (digest(args.rust_build_info) != rust_build_info_sha256 or validate_rust_build(args.rust_build_info, args.rust_build_json, rust, args.reference_profile, source_root=rust_root) != rust_build):
            raise ValueError('Rust benchmark build record changed')

    recheck_inputs()
    libdirs = [builds['physics'] / 'lib', builds['sensors'] / 'lib', builds['sim'] / 'lib',
               prefix / 'lib', args.dependency_prefix.resolve() / 'lib']
    base_env = os.environ.copy()
    for key in ['GZ_TRANSPORT_USERNAME', 'GZ_TRANSPORT_PASSWORD', 'GZ_RELAY']:
        base_env.pop(key, None)
    base_env.update(LD_LIBRARY_PATH=os.pathsep.join(map(str, libdirs)),
                    GZ_IP='127.0.0.1', GZ_SIM_SERVER_CONFIG_PATH='',
                    GZ_SIM_SYSTEM_PLUGIN_PATH=str(builds['sim'] / 'lib'),
                    GZ_SIM_PHYSICS_ENGINE_PATH=str(builds['physics'] / 'lib'),
                    GZ_TRANSPORT_IMPLEMENTATION='zeromq', GZ_TRANSPORT_TOPIC_STATISTICS='0',
                    GZ_TRANSPORT_RCVHWM='1000', GZ_TRANSPORT_SNDHWM='1000',
                    GZ_DISCOVERY_MSG_PORT='10317', GZ_DISCOVERY_SRV_PORT='10318',
                    GZ_STREAM_PERIOD_US=str(args.request_period_us), GZ_PIPELINE_STEPS=str(args.steps), GZ_PIPELINE_EXECUTOR=args.executor)
    if app is not None:
        libdirs.append(app['vendor'] / 'lib')
        config_path = work / 'zenoh-peer.json'
        config_path.write_text(json.dumps({
            'mode': 'peer', 'listen': {'endpoints': ['tcp/127.0.0.1:0']},
            'connect': {'endpoints': []},
            'scouting': {'multicast': {'enabled': False}, 'gossip': {'enabled': False}},
            'transport': {'shared_memory': {'enabled': False}},
        }) + '\n')
        zenoh_config_sha256 = digest(config_path)
        base_env = app_runtime_environment(compile_env, libdirs, config_path)
        base_env.update(GZ_IP='127.0.0.1', GZ_SIM_SERVER_CONFIG_PATH='',
                        GZ_SIM_SYSTEM_PLUGIN_PATH=str(builds['sim'] / 'lib'),
                        GZ_SIM_PHYSICS_ENGINE_PATH=str(builds['physics'] / 'lib'),
                        GZ_TRANSPORT_TOPIC_STATISTICS='0',
                        GZ_STREAM_PERIOD_US=str(args.request_period_us),
                        GZ_PIPELINE_STEPS=str(args.steps), GZ_PIPELINE_EXECUTOR=args.executor)
    required = {
        'libgz-sim.so.': builds['sim'] / 'lib',
        'libgz-sim-physics-system.so.': builds['sim'] / 'lib',
        'libgz-sim-imu-system.so.': builds['sim'] / 'lib',
        'libgz-physics-tpe-plugin.so.': builds['physics'] / 'lib',
        'libgz-physics-tpelib.so.': builds['physics'] / 'lib',
        'libgz-sensors.so.': builds['sensors'] / 'lib',
        'libgz-sensors-imu.so.': builds['sensors'] / 'lib',
        'libgz-msgs.so.': prefix / 'lib',
        'libgz-transport.so.': prefix / 'lib',
    }
    if app is not None:
        required['libzenohc.so'] = app['vendor'] / 'lib'
    if args.path == 'streams':
        required['libgz-sim-scene-broadcaster-system.so.'] = builds['sim'] / 'lib'
    if app is not None:
        linkage = subprocess.check_output(['/usr/bin/ldd', str(native)], env=base_env, text=True)
        validate_app_linkage(linkage, sdk_libraries, {name: required[name] for name in APP_STATIC_ROLES})
        (work / 'native-linkage.txt').write_text(linkage)
        recheck_inputs()
    observations, native_libs = [], None
    for index in range(args.samples + 1):
        pair = {}
        for kind in (['native', 'rust'] if index % 2 == 0 else ['rust', 'native']):
            maps = work / f'native-{index}.maps'
            run_command = ([str(native), str(fixture), str(maps), str(args.steps)] if kind == 'native' else
                           [str(rust), ('streams_benchmark::tpe_clock_pose_imu_complete_lockstep' if args.path == 'streams' else 'pipeline_benchmark::tpe_imu_full_server_pipeline' if args.path == 'server' else 'pipeline_benchmark::tpe_imu_process_local_pipeline'),
                            '--exact', '--nocapture', '--test-threads=1'])
            env = dict(base_env, GZ_PARTITION=f'pipeline-{os.getpid()}-{index}-{kind}')
            result = subprocess.run(['taskset', '-c', str(cpu), *run_command],
                                    env=env, text=True, capture_output=True,
                                    timeout=max(30, args.steps * args.request_period_us / 1_000_000 * 1.25 + 10) if args.path == 'streams' else 30)
            (work / f'{kind}-{index}.stdout').write_text(result.stdout)
            (work / f'{kind}-{index}.stderr').write_text(result.stderr)
            if result.returncode or result.stderr:
                raise ValueError(f'{kind} sample {index} failed: {result.returncode} {result.stderr}')
            pair[kind] = decode(result.stdout, args.steps, fixture_sha256, warmup_force_x=0.0 if args.path != 'direct' else None, streams=args.path == 'streams', request_period_us=args.request_period_us)
            if kind == 'native':
                loaded = loaded_libraries(maps)
                if app is not None:
                    validate_app_loaded_libraries(loaded, sdk_libraries, all_libraries=True)
                elif sdk_libraries is not None:
                    validate_sdk_loaded_libraries(loaded,sdk_libraries)
                require_native_libraries(loaded,required,sdk_libraries)
                if native_libs is not None and loaded != native_libs:
                    raise ValueError('native library set changed between samples')
                native_libs = loaded
        fields = ['steps', 'logical_bytes', 'time_sum']
        if args.path == 'streams':
            fields += ['request_period_us', 'clock_count', 'pose_count', 'clock_time_sum', 'pose_time_sum', 'pose_bytes_without_id_values']
        for field in fields:
            if pair['native'][field] != pair['rust'][field]:
                raise ValueError('native/Rust work differs: ' + field)
        if not math.isclose(pair['native']['force_sum'], pair['rust']['force_sum'], abs_tol=1e-8):
            raise ValueError('native/Rust physical checksum differs')
        observations.append(pair)
    recheck_inputs()
    values = {name: [pair[name]['nanos'] / args.steps for pair in observations[1:]]
              for name in ['native', 'rust']}
    medians = {name: statistics.median(value) for name, value in values.items()}
    model = next((line.split(':', 1)[1].strip() for line in Path('/proc/cpuinfo').read_text().splitlines()
                  if line.startswith('model name')), 'unknown')
    report = dict(
        scope={
            'direct': 'Historical Rust direct publisher versus native full Server; native ratio is contextual only. ',
            'server': 'Actual Rust Server including unobserved pose/clock work versus native Server+Physics+Imu; native ratio is contextual. ',
            'streams': 'Actual Rust Server versus native Server+Physics+Imu+SceneBroadcaster, all three requested clock/pose/IMU streams with prospective20ms-or-greater request pacing and per-step complete receipt; summed work latency excludes declared pacing, walltime recorded separately. ',
        }[args.path] + 'Two-model TPE with one constant1m/s free body, one1000Hz noise-free IMU, sharedSDF, internallyunpaced1ms physics and complete decoded local observations. One received warmup excluded; N measured steps. Not network, joints, rendering, balanced robotics or whole-engine speedup.',
        measurement_boundary=('Sum of per-request servercall+complete-receipt/check latency, declared host pacing excluded and recorded separately' if args.path == 'streams' else 'Elapsed complete observed pipeline interval'),
        pose_byte_contract=('Record raw bytes and opaque stable roleIDs; cross-run size comparison subtracts only ID varint values, guarded by every small pose record encodedlen<128' if args.path == 'streams' else None),
        qualification='Shared host CPU-pinned alternating samples. Native local transport may bypass serialization; Rust currently uses encoded queues. Requested delivered messages/settings match, not internal instructions. Other installed dependencies/build provenance remain qualified; no P2 acceptance inferred.',
        hardware=dict(cpu_model=model, machine=platform.machine(), cpu_affinity=cpu),
        observation_protocol=('Rust receives/checks each requested observation after Server::tick; native calls Server.Run(true,1,false), checks its callbacks and waits for all requested observations. Neither admits another step before complete receipt. Both include complete receipt and checking inside timing.' if args.path != 'direct' else 'Historical direct publisher batch pipeline'),
        queue_settings=(dict(rust_sample_queue_capacity=1000, native_local_queue='copied typed pubQueue; no ZMQ HWM applies') if app is not None else dict(rust_capacity=1000, rust_send_hwm=1000, rust_receive_hwm=1000, native_explicit_hwm=1000)),
        request_period_us=args.request_period_us if args.path == 'streams' else 0, steps=args.steps, rust_path=args.path, rust_executor=args.executor, warmup_pairs=1, measured_pairs=args.samples, samples=observations,
        ns_per_step=dict(medians=medians, min={k:min(v) for k,v in values.items()},
                         max={k:max(v) for k,v in values.items()}, native_over_rust=medians['native']/medians['rust']),
        builds=build_settings, native_revisions={name:lock['gz-'+name] for name in builds}, native_sources=native_sources, native_command=command,
        compiler=compiler_version, compiler_sha256=compiler_sha256,
        reference_identity=reference_identity, rust_build_info_sha256=rust_build_info_sha256,
        sdk_build_registry_sha256=sdk_registry_sha256,
        source_tree=rust_build['source_tree'] if rust_build is not None else None,
        rustc=rust_build['rustc_vv'] if rust_build is not None else subprocess.check_output(['rustc', '-vV'], text=True),
        fixture_sha256=fixture_sha256, rust_production_sources={str(p.relative_to(rust_root)):digest(p) for p in (rust_root/'crates').rglob('*.rs') if '/src/' in str(p)}, cargo_lock_sha256=digest(rust_root/'Cargo.lock'), git_base=rust_build['git_base'] if rust_build is not None else subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(), source_sha256={str(p.relative_to(rust_root)):digest(p) for p in
            [source, rust_root/'crates/gz-sim-server/tests/private/pipeline_benchmark.rs',
             *([rust_root/'crates/gz-sim-server/tests/private/streams_benchmark.rs'] if args.path == 'streams' else []),
             rust_root/'crates/gz-sim-server/src/lib.rs', rust_root/'crates/gz-sim-core/src/lib.rs', rust_root/'scripts/benchmark-headless-pipeline.py']},
        binary_sha256=binary_sha256, native_libraries=native_libs)
    if app is not None:
        report['zenoh_config_sha256'] = zenoh_config_sha256
        report['source_contexts'] = {'rust': str(rust_root), 'native_app': str(app_source_root), 'measurement_tool': str(ROOT)}
        report['measurement_tool_inputs_sha256'] = tool_inputs
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report['ns_per_step']))


if __name__ == '__main__':
    main()
