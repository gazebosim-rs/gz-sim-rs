#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Replay the historical DART oracle independently of the candidate adapter."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import stat
import subprocess

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / 'crates/gz-physics-dart/tests/fixtures/three_body_static_contacts.json'
PROBE = 'crates/gz-physics-dart/tests/fixtures/three_body_contact_literals.cc'
IDENTITY = 'crates/gz-physics-dart/tests/fixtures/body_revolute_identity.hh'
HEADER = 'crates/gz-physics-dart/src/joint_state.hh'
HISTORICAL_HEADER = 'tests/fixtures/three_body_static_contacts_joint_state.hh'
HISTORICAL_REVISION = 'e829be51d42fed27268606528ba758a93efc37a2'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def project_sources(root, output, fixture):
    """Retain logical include paths, with the oracle's original header bytes."""
    inputs = {PROBE: root / PROBE, IDENTITY: root / IDENTITY,
              HEADER: root / HISTORICAL_HEADER}
    expected = fixture['native_sources']
    assert set(expected) == set(inputs), 'Historical owned source set changed'
    assert expected[PROBE] == fixture['probe_source_sha256']
    contents = {name: path.read_bytes() for name, path in inputs.items()}
    assert all(hashlib.sha256(data).hexdigest() == expected[name]
               for name, data in contents.items()), 'Historical source hash changed'
    directory = output / 'historical-sources'
    directory.mkdir(exist_ok=False)
    records = []
    for name, data in contents.items():
        projected = directory / name
        projected.parent.mkdir(parents=True, exist_ok=True)
        projected.write_bytes(data)
        assert digest(projected) == expected[name]
        records.append({'logical_path': name, 'input_path': str(inputs[name]),
                        'projected_path': str(projected), 'sha256': expected[name]})
    return directory / PROBE, records


def check_compiler_inputs(dependencies, projection, expected_system):
    owned = {str(Path(row['projected_path']).resolve()): row['sha256']
             for row in projection}
    assert len(owned) == 3
    actual_owned = {name: value for name, value in dependencies.items() if name in owned}
    assert actual_owned == owned, 'Historical projected compiler inputs changed'
    system = {name: value for name, value in dependencies.items() if name not in owned}
    assert system == expected_system, 'Native compiler input closure changed'


# This explicit selection is a metadata projection, not fresh native evidence.
DEPENDENCY_VARIABLE = 'GZ_NATIVE_DEPENDENCY_SNAPSHOT'
FIXTURE_IDENTITY = {'sha256': '66bd14f834a1405947c4769a6668c34c53410c2a4b9e1efc300b03bbede659c0', 'bytes': 168164}
DEPENDENCY_DIRECTORY = Path('/home/nkoenig/work/gazebo/rust/gz-sim-rs/artifacts/m58-delivery')
DEPENDENCY_PINS = {
    'snapshot': {'path': str(DEPENDENCY_DIRECTORY / 'native-sdk-current-dependency-closure.json'), 'sha256': '37ec87f58c22d033aa1addff2c3f256635aa34858f400ed2c9f745b2b1dfbd94', 'bytes': 830235},
    'inventory': {'path': str(DEPENDENCY_DIRECTORY / 'sdk-dependency-drift-inventory.json'), 'sha256': '770820c1971a778977e0ca385f056ac8c67d1c62a694989c0a46e3ca609a75a6', 'bytes': 476390},
    'actual_exit': {'path': str(DEPENDENCY_DIRECTORY / 'sdk-dependency-drift-inventory-actual-exit.json'), 'sha256': '82b30aeb444875ebdf1aa9f8cbf4387f742f506162a06b952920b2e5b7fa25da', 'bytes': 1852},
}
APPROVED_DEPENDENCY_CHANGES = {
    '/usr/lib/x86_64-linux-gnu/libc.so.6': ('3a15d66867d83762c7f2f1e37359cb8f6c5743edb369c65285cb0b1c4f7498bf', '755a371e86cc9f564ab54f66140747700fb0a103f698f659f2060f707897e09a'),
    '/usr/lib/x86_64-linux-gnu/libm.so.6': ('fce00b6f25f459cf4ae0b7fae4257909a1a8a86f9149e7c029a5d617baf1ccd0', '5dcc4d25466e29ca390ed6fabb371c69b65dff6fa4e91c34561057a020d6994a'),
    '/usr/lib/x86_64-linux-gnu/ld-linux-x86-64.so.2': ('c20a2dc8917c755f02b94049356320fe1f62ac7d9f8994731f807d9df39302da', '5fc0940888e9d2e1a00e66d6b1a559c36364ee867683652e1074fa576952cc3f'),
}


def verified_bytes(path, expected=None, cap=2 * 1024**2):
    """Hash and consume the same bounded, ordinary, stable no-follow FD bytes."""
    path = Path(path)
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    try:
        before = os.fstat(fd)
        assert stat.S_ISREG(before.st_mode) and before.st_size <= cap, ('metadata type/size', str(path), before.st_size, cap)
        with os.fdopen(fd, 'rb', closefd=False) as stream:
            raw = stream.read(cap + 1)
        after = os.fstat(fd)
        stamp = lambda value: (value.st_dev, value.st_ino, value.st_mode, value.st_size, value.st_mtime_ns, value.st_ctime_ns)
        assert len(raw) == before.st_size and stamp(before) == stamp(after) == stamp(path.lstat()), ('metadata changed during read', str(path))
        observed = {'path': str(path), 'sha256': hashlib.sha256(raw).hexdigest(), 'bytes': len(raw)}
        if expected is not None:
            assert observed == expected, ('metadata identity mismatch', {'expected': expected, 'observed': observed})
        return raw, observed
    finally:
        os.close(fd)


def metadata_json(raw):
    def pairs(values):
        result = {}
        for key, value in values:
            assert key not in result, ('duplicate metadata key', key)
            result[key] = value
        return result
    def invalid_constant(value):
        raise ValueError('nonfinite metadata number: ' + value)
    return json.loads(raw, object_pairs_hook=pairs, parse_constant=invalid_constant)


def project_dependency_snapshot(fixture, snapshot, inventory, receipt):
    """Exactly three historical library SHA leaves; no current body reads."""
    historical = fixture['native_libraries']
    assert len(historical) == 18, ('historical dependency count', len(historical))
    update = snapshot['environment_update_M58']
    assert update['retained_complete_inventory'] == DEPENDENCY_PINS['inventory']
    assert update['retained_inventory_actual_Root_receipt'] == DEPENDENCY_PINS['actual_exit']
    assert receipt['report'] == DEPENDENCY_PINS['inventory']
    assert type(receipt['actual_root_exit_code']) is int and receipt['actual_root_exit_code'] == 0, 'dependency inventory lacks genuine Root0'
    assert type(receipt['tool_exit_basis']['actual_exit_code']) is int and receipt['tool_exit_basis']['actual_exit_code'] == 0
    assert receipt['tool_exit_basis']['completion_chunk'], 'missing dependency inventory completion'
    assert inventory['state'] == 'SAVED351_CURRENT_IDENTITY_DIAGNOSTIC_ONLY_NOT_ADMISSION'
    assert type(inventory['internal_exit_code']) is int and inventory['internal_exit_code'] == 0 and inventory['errors'] == []
    assert inventory['observed_count'] == 351 and inventory['matching_count'] == 347 and inventory['delta_count'] == 4
    assert len(inventory['rows']) == 351 and len(snapshot['nodes']) == 349
    assert update['same_349_node_paths_and_needed_edges'] is True
    assert update['changed_current349_node_hashes'] == 4 and update['unchanged_current349_node_hashes'] == 345
    changed = {row['path']: row for row in update['changed_files']}
    expected_delta_paths = set(APPROVED_DEPENDENCY_CHANGES) | {'/usr/lib/x86_64-linux-gnu/libresolv.so.2'}
    assert len(update['changed_files']) == 4 and set(changed) == set(inventory['deltas']) == expected_delta_paths
    observed = {row['path']: row for row in inventory['rows']}
    assert len(observed) == 351, 'duplicate inventory node'
    effective = dict(historical)
    projection = []
    for path, (old, current) in APPROVED_DEPENDENCY_CHANGES.items():
        delta = changed[path]
        actual = observed[path]['actual']
        assert historical.get(path) == old, ('historical dependency mismatch', path, old, historical.get(path))
        assert delta['previous_sha256'] == old and delta['current_sha256'] == current
        assert snapshot['nodes'][path]['sha256'] == actual['sha256'] == current
        assert snapshot['nodes'][path]['bytes'] == delta['bytes'] == actual['bytes']
        assert delta['dynamic_text_byte_identical'] is True and delta['resolved_needed_unchanged'] is True
        assert delta['package_md5_comparison']['matches_package_md5'] is True
        effective[path] = current
        projection.append({'path': path, 'historical_sha256': old, 'effective_sha256': current, 'bytes': actual['bytes']})
    restored = dict(effective)
    for path, (old, _) in APPROVED_DEPENDENCY_CHANGES.items():
        restored[path] = old
    assert restored == historical and set(effective) == set(historical)
    assert {path for path in historical if historical[path] != effective[path]} == set(APPROVED_DEPENDENCY_CHANGES)
    return effective, projection


def dependency_environment(fixture, fixture_card, environ=None):
    environ = os.environ if environ is None else environ
    historical = fixture['native_libraries']
    if DEPENDENCY_VARIABLE not in environ:
        return dict(historical), {'state': 'HISTORICAL_STRICT_DEFAULT', 'fixture': fixture_card,
                                  'selected_metadata': {}, 'projection': [],
                                  'historical_libraries': dict(historical), 'effective_libraries': dict(historical)}
    selected = environ[DEPENDENCY_VARIABLE]
    assert selected and selected == DEPENDENCY_PINS['snapshot']['path'], ('invalid explicit dependency snapshot', selected)
    assert {key: fixture_card[key] for key in ('sha256', 'bytes')} == FIXTURE_IDENTITY, ('selected historical fixture mismatch', fixture_card, FIXTURE_IDENTITY)
    values = {}
    cards = {}
    for role, expected in DEPENDENCY_PINS.items():
        raw, cards[role] = verified_bytes(expected['path'], expected)
        values[role] = metadata_json(raw)
    libraries, projection = project_dependency_snapshot(fixture, values['snapshot'], values['inventory'], values['actual_exit'])
    return libraries, {'state': 'EXPLICIT_RETAINED_THREE_LIBC_DEPENDENCY_PROJECTION',
                       'fixture': fixture_card, 'selected_metadata': cards, 'projection': projection,
                       'historical_libraries': dict(historical), 'effective_libraries': libraries,
                       'native_replay_pass_claimed_by_selection': False}


def check_dependency_environment(record):
    # Re-read the same fixed metadata cards; a computed map is not lasting custody.
    verified_bytes(record['fixture']['path'], record['fixture'])
    for expected in record['selected_metadata'].values():
        verified_bytes(expected['path'], expected)


def check_library(name, expected, context='native dependency hash mismatch'):
    path = Path(name)
    try:
        observed = digest(path)
        size = path.stat().st_size
    except OSError as error:
        raise AssertionError((context, {'path': str(path), 'expected_sha256': expected,
                                        'observed_sha256': None, 'observed_bytes': None,
                                        'error': str(error)})) from error
    assert observed == expected, (context, {'path': str(path), 'expected_sha256': expected,
                                           'observed_sha256': observed, 'observed_bytes': size})


def check_libraries(libraries):
    observations = []
    mismatches = []
    for name, expected in libraries.items():
        path = Path(name)
        row = {'path': str(path), 'expected_sha256': expected, 'observed_sha256': None,
               'observed_bytes': None, 'read_error': None}
        try:
            row['observed_sha256'] = digest(path)
            row['observed_bytes'] = path.stat().st_size
        except OSError as error:
            row['read_error'] = str(error)
        observations.append(row)
        if row['read_error'] is not None or row['observed_sha256'] != expected:
            mismatches.append(row)
    assert not mismatches, ('native dependency hash mismatch', {
        'observed_count': len(observations), 'observations': observations, 'mismatches': mismatches})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    assert shutil.disk_usage(output).free > 1280 * 1024**2
    fixture_raw, fixture_card = verified_bytes(FIXTURE)
    fixture = metadata_json(fixture_raw)
    libraries, environment = dependency_environment(fixture, fixture_card)
    check_dependency_environment(environment)
    source, projection = project_sources(ROOT, output, fixture)
    assert all(digest(Path(name)) == value for name, value in fixture['compiler_system_inputs'].items())
    env = dict(os.environ)
    for key in ['LD_PRELOAD', 'LD_AUDIT', 'LD_LIBRARY_PATH', 'CXXFLAGS',
                'CPPFLAGS', 'LDFLAGS', 'CPATH', 'CPLUS_INCLUDE_PATH', 'LIBRARY_PATH',
                'PKG_CONFIG_SYSROOT_DIR']:
        env.pop(key, None)
    env['LC_ALL'] = 'C'
    compiler = Path('/usr/bin/g++-13').resolve(strict=True)
    assert digest(compiler) == fixture['compiler_sha256']
    def pkg(*arguments):
        return subprocess.check_output(['pkg-config', *arguments], env=env, text=True).strip()
    assert pkg('--modversion', 'dart') == fixture['dart_version']
    libdir = Path(pkg('--variable=libdir', 'dart')).resolve()
    ode_dir = Path(pkg('--variable=libdir', 'ode')).resolve()
    assert libraries
    check_libraries(libraries)
    inputs = [FIXTURE, Path(__file__)] + [Path(row[key]) for row in projection
                                       for key in ('input_path', 'projected_path')]
    before = {str(path): digest(path) for path in inputs}
    assert before[str(FIXTURE)] == fixture_card['sha256'], ('fixture changed after verified parse', fixture_card, before[str(FIXTURE)])
    binary = output / 'reference'
    command = [str(compiler), '-O0', '-g0', '-Wall', '-Wextra', '-Werror',
               *shlex.split(pkg('--cflags', 'dart')), str(source), '-MD', '-MF',
               str(output / 'reference.d'), '-o', str(binary),
               *shlex.split(pkg('--libs', 'dart')), '-L' + str(libdir),
               '-ldart-collision-ode', '-ldart-external-odelcpsolver',
               '-L' + str(ode_dir), '-lode', '-UNDEBUG', '-std=c++17',
               '-Wl,--disable-new-dtags', '-Wl,-rpath,' + str(libdir),
               '-Wl,-rpath,' + str(ode_dir)]
    result = {'scope': 'Historical direct native fixture replay; candidate adapter is tested by Rust',
              'historical_header_revision': HISTORICAL_REVISION,
              'logical_native_sources': fixture['native_sources'],
              'source_projection': projection, 'fixture_sha256': fixture_card['sha256'],
              'compile_command': command, 'source_sha256': before, 'runs': [],
              'dependency_environment': environment}
    check_dependency_environment(environment)
    def save():
        (output / 'native-result.json').write_text(json.dumps(result, indent=2) + '\n')
    save()
    with (output / 'compile.stdout').open('wb') as stdout, (output / 'compile.stderr').open('wb') as stderr:
        compiled = subprocess.run(command, env=env, stdin=subprocess.DEVNULL,
                                  stdout=stdout, stderr=stderr, timeout=180)
    result['compile_exit'] = compiled.returncode
    save()
    assert compiled.returncode == 0
    result['elf_sha256'] = digest(binary)
    dependencies = shlex.split((output / 'reference.d').read_text().replace('\\\n', ' ').split(':', 1)[1])
    result['compiler_inputs'] = {str(Path(path).resolve()): digest(Path(path)) for path in dependencies}
    check_compiler_inputs(result['compiler_inputs'], projection, fixture['compiler_system_inputs'])
    run_env = dict(env, GZ_DART_PROBE_LIBDIR=str(libdir), GZ_DART_PROBE_ODE_LIBDIR=str(ode_dir))
    for mode in ['attach-live', 'attach-rollback', 'approach-live', 'approach-remove',
                 'footprint-sphere', 'footprint-box', 'above-condition']:
        argv = [str(binary), mode, '0', str(output / (mode + '.before.maps')),
                str(output / (mode + '.after.maps'))]
        record = {'mode': mode, 'argv': argv}
        with (output / (mode + '.stdout')).open('wb') as stdout, (output / (mode + '.stderr')).open('wb') as stderr:
            try:
                run = subprocess.run(argv, env=run_env, stdin=subprocess.DEVNULL,
                                     stdout=stdout, stderr=stderr, timeout=30)
                record.update(exit_code=run.returncode, timeout=False)
            except subprocess.TimeoutExpired:
                record.update(exit_code=None, timeout=True)
        result['runs'].append(record)
        save()
        assert record['exit_code'] == 0 and not record['timeout']
        lines = (output / (mode + '.stdout')).read_text().splitlines()
        rows = [json.loads(line) for line in lines if line.startswith('{')]
        record['native_messages'] = [line for line in lines if not line.startswith('{')]
        if mode in fixture['cases']:
            expected = fixture['cases'][mode]
            stages = {row['stage'] for row in expected['rows']}
            measured = [row for row in rows if row['kind'] == 'state' and row['stage'] in stages]
            assert measured == expected['rows'], ('native selected records differ', mode)
            if 'conditions' in expected:
                assert [row for row in rows if row['kind'] == 'matrix_condition'] == expected['conditions']
        for suffix in ['before', 'after']:
            for line in (output / (mode + '.' + suffix + '.maps')).read_text().splitlines():
                fields = line.split(maxsplit=5)
                if len(fields) != 6 or not fields[5].startswith('/'):
                    continue
                path = Path(fields[5]).resolve()
                if path == binary:
                    continue
                check_library(path, libraries.get(str(path)), 'unqualified mapping')
        save()
    assert before == {path: digest(Path(path)) for path in before}
    check_libraries(libraries)
    check_dependency_environment(environment)
    result['source_unchanged'] = True
    save()
    print('Seven source-bound native static-contact controls and selected records replayed')


if __name__ == '__main__':
    main()
