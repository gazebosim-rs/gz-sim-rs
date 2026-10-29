#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Replay DART generalized torque through the pinned C++ Gazebo physics plugin."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / 'crates/gz-physics-dart/tests/fixtures'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['physics-source', 'build', 'prefix', 'work-dir', 'report']:
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--write-fixture', action='store_true')
    args = parser.parse_args()
    source, build, prefix, work = [p.resolve() for p in
        [args.physics_source, args.build, args.prefix, args.work_dir]]
    lock = next(row['commit'] for row in json.loads((ROOT / 'planning/baseline.json').read_text())['repositories']
                if row['name'] == 'gz-physics')
    assert subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip() == lock
    subprocess.run(['git', '-C', str(source), 'diff', '--quiet', lock, '--'], check=True)
    sources = {}
    for name in ['dartsim/src/JointFeatures.cc', 'dartsim/src/SimulationFeatures.cc', 'dartsim/src/SDFFeatures.cc']:
        assert (source / name).read_bytes() == subprocess.check_output(['git', '-C', str(source), 'show', lock + ':' + name])
        sources[name] = digest(source / name)
    cache = (build / 'CMakeCache.txt').read_text()
    home = next(line.split('=', 1)[1] for line in cache.splitlines() if line.startswith('CMAKE_HOME_DIRECTORY:'))
    assert Path(home).resolve() == source
    install_prefix = next(line.split('=', 1)[1] for line in cache.splitlines()
                          if line.startswith('CMAKE_INSTALL_PREFIX:'))
    assert Path(install_prefix).resolve() == prefix
    build_command = ['cmake', '--build', str(build), '--target', 'gz-physics-dartsim-plugin', '--parallel', '2']
    subprocess.run(build_command, check=True, stdout=subprocess.DEVNULL)
    work.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, PKG_CONFIG_PATH=str(prefix / 'lib/pkgconfig'))
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs',
        'gz-physics', 'gz-physics-sdf', 'gz-plugin-loader', 'sdformat', 'gz-math-eigen3'], env=env, text=True))
    cpp, binary, maps = FIXTURES / 'joint_effort.cc', work / 'joint-effort', work / 'loaded.maps'
    command = ['c++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', str(cpp), '-o', str(binary),
               *flags, '-Wl,-rpath,' + str(prefix / 'lib')]
    subprocess.run(command, check=True)
    system = dict(os.environ, PKG_CONFIG_PATH='')
    system.pop('PKG_CONFIG_LIBDIR', None)
    assert subprocess.check_output(['pkg-config', '--modversion', 'dart'], env=system, text=True).strip() == '6.13.2'
    dart = Path(subprocess.check_output(['pkg-config', '--variable=libdir', 'dart'], env=system, text=True).strip()).resolve()
    ode = Path(subprocess.check_output(['pkg-config', '--variable=libdir', 'ode'], env=system, text=True).strip()).resolve()
    env['LD_LIBRARY_PATH'] = str(dart) + ':' + str(ode) + ':' + str(build / 'lib') + ':' + str(prefix / 'lib')
    plugin = (build / 'lib/libgz-physics-dartsim-plugin.so').resolve(strict=True)
    result = subprocess.run([str(binary), str(plugin), str(maps)], env=env,
                            capture_output=True, text=True, check=True, timeout=30)
    assert result.stderr == '', result.stderr
    observations = [json.loads(line) for line in result.stdout.splitlines()]
    assert len(observations) == 6
    paths = {Path(line.split(maxsplit=5)[5]) for line in maps.read_text().splitlines()
             if len(line.split(maxsplit=5)) == 6 and line.split(maxsplit=5)[5].startswith('/')}
    libraries = {}
    for name, directory in [('libdart.so.', dart), ('libdart-collision-ode.so.', dart),
                            ('libdart-external-odelcpsolver.so.', dart), ('libode.so.', ode),
                            ('libgz-physics-dartsim-plugin.so.', build / 'lib')]:
        matches = [path for path in paths if path.name.startswith(name)]
        assert len(matches) == 1, (name, matches)
        actual = matches[0]
        assert actual.parent.resolve() == directory.resolve(), str(actual)
        libraries[actual.name] = digest(actual)
    fixture = {'source_sha256': digest(cpp), 'physics_commit': lock,
               'dart_version': '6.13.2', 'observations': observations}
    path = FIXTURES / 'joint_effort.json'
    if args.write_fixture:
        path.write_text(json.dumps(fixture, indent=2) + '\n')
    else:
        assert json.loads(path.read_text()) == fixture
    report = dict(scope='Fresh C++ client and rebuilt pinned native plugin target with installed dependencies; not a fresh SDK configuration or performance comparison.',
                  fixture=fixture, sources=sources, loaded_libraries=libraries,
                  cmake_cache_sha256=digest(build / 'CMakeCache.txt'), build_command=build_command, command=command,
                  compiler=subprocess.check_output(['c++', '--version'], text=True),
                  binary_sha256=digest(binary), stderr=result.stderr)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + '\n')
    print('6 native joint-effort observations passed')


if __name__ == '__main__':
    main()
