#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Replay selected direct DART static-contact rows against their qualified inputs."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'crates/gz-physics-dart/tests/fixtures/three_body_contact_literals.cc'
FIXTURE = ROOT / 'crates/gz-physics-dart/tests/fixtures/three_body_static_contacts.json'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    assert shutil.disk_usage(output).free > 1280 * 1024**2
    fixture = json.loads(FIXTURE.read_text())
    assert digest(SOURCE) == fixture['probe_source_sha256']
    assert all(digest(ROOT / name) == value for name, value in fixture['native_sources'].items())
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
    inputs = [SOURCE, ROOT / 'crates/gz-physics-dart/tests/fixtures/body_revolute_identity.hh',
              ROOT / 'crates/gz-physics-dart/src/joint_state.hh', FIXTURE, Path(__file__)]
    before = {str(path): digest(path) for path in inputs}
    binary = output / 'reference'
    command = [str(compiler), '-O0', '-g0', '-Wall', '-Wextra', '-Werror',
               *shlex.split(pkg('--cflags', 'dart')), str(SOURCE), '-MD', '-MF',
               str(output / 'reference.d'), '-o', str(binary),
               *shlex.split(pkg('--libs', 'dart')), '-L' + str(libdir),
               '-ldart-collision-ode', '-ldart-external-odelcpsolver',
               '-L' + str(ode_dir), '-lode', '-UNDEBUG', '-std=c++17',
               '-Wl,--disable-new-dtags', '-Wl,-rpath,' + str(libdir),
               '-Wl,-rpath,' + str(ode_dir)]
    result = {'scope': 'Direct native fixture replay, no adapter admission',
              'compile_command': command, 'source_sha256': before, 'runs': []}
    def save():
        (output / 'native-result.json').write_text(json.dumps(result, indent=2) + '\n')
    with (output / 'compile.stdout').open('wb') as stdout, (output / 'compile.stderr').open('wb') as stderr:
        compiled = subprocess.run(command, env=env, stdin=subprocess.DEVNULL,
                                  stdout=stdout, stderr=stderr, timeout=180)
    result['compile_exit'] = compiled.returncode
    save()
    assert compiled.returncode == 0
    result['elf_sha256'] = digest(binary)
    dependencies = shlex.split((output / 'reference.d').read_text().replace('\\\n', ' ').split(':', 1)[1])
    result['compiler_inputs'] = {str(Path(path).resolve()): digest(Path(path)) for path in dependencies}
    owned = {(ROOT / name).resolve() for name in fixture['native_sources']}
    system = {name: value for name, value in result['compiler_inputs'].items() if Path(name) not in owned}
    assert system == fixture['compiler_system_inputs'], 'Native compiler input closure changed'
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
