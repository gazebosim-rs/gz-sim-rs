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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    assert shutil.disk_usage(output).free > 1280 * 1024**2
    fixture = json.loads(FIXTURE.read_text())
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
    libraries = fixture['native_libraries']
    assert libraries and all(digest(Path(path)) == value for path, value in libraries.items())
    inputs = [FIXTURE, Path(__file__)] + [Path(row[key]) for row in projection
                                       for key in ('input_path', 'projected_path')]
    before = {str(path): digest(path) for path in inputs}
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
              'source_projection': projection, 'fixture_sha256': digest(FIXTURE),
              'compile_command': command, 'source_sha256': before, 'runs': []}
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
                assert libraries.get(str(path)) == digest(path), ('unqualified mapping', path)
        save()
    assert before == {path: digest(Path(path)) for path in before}
    assert all(digest(Path(path)) == value for path, value in libraries.items())
    result['source_unchanged'] = True
    save()
    print('Seven source-bound native static-contact controls and selected records replayed')


if __name__ == '__main__':
    main()
