#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Rebuild actual native Server world-velocity/wrench controls and verify engine identity."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def checked_sources(source, revision, names):
    actual = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    if actual != revision:
        raise ValueError('native revision mismatch: ' + str(source))
    hashes = {}
    for name in names:
        original = subprocess.check_output(['git', '-C', str(source), 'show', revision + ':' + name])
        if original != (source / name).read_bytes():
            raise ValueError('native source changed: ' + name)
        hashes[name] = digest(source / name)
    return hashes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['sim-source', 'physics-source', 'sim-build', 'dependency-prefix', 'work-dir', 'output']:
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    upstream, physics, build = args.sim_source.resolve(), args.physics_source.resolve(), args.sim_build.resolve()
    work = args.work_dir.resolve()
    work.mkdir(parents=True, exist_ok=True)
    lock = {item['name']: item['commit'] for item in json.loads((ROOT / 'planning/baseline.json').read_text())['repositories']}
    sources = {
        'gz-sim': checked_sources(upstream, lock['gz-sim'], ['src/systems/physics/Physics.cc', 'src/Link.cc', 'include/gz/sim/Link.hh', 'include/gz/sim/components/LinearVelocityReset.hh', 'include/gz/sim/components/AngularVelocityReset.hh']),
        'gz-physics': checked_sources(physics, lock['gz-physics'], ['tpe/plugin/src/FreeGroupFeatures.cc', 'tpe/plugin/src/KinematicsFeatures.cc', 'dartsim/src/FreeGroupFeatures.cc', 'dartsim/src/LinkFeatures.cc']),
    }
    cache = (build / 'CMakeCache.txt').read_text()
    home = next(line.split('=', 1)[1] for line in cache.splitlines() if line.startswith('CMAKE_HOME_DIRECTORY:'))
    if Path(home).resolve() != upstream:
        raise ValueError('native build belongs to another source directory')
    target = build / 'src/CMakeFiles/UNIT_ServerConfig_TEST.dir'
    flags = (target / 'flags.make').read_text()
    includes = shlex.split(next(line.split('=', 1)[1] for line in flags.splitlines() if line.startswith('CXX_INCLUDES =')))
    link = shlex.split((target / 'link.txt').read_text())
    libraries = [str((build / 'src' / item).resolve()) if item.startswith('../') else item
                 for item in link[link.index('-o') + 2:] if 'gtest' not in item]
    source = ROOT / 'crates/gz-sim-scene/tests/native_physics_inputs.cc'
    binary = work / 'native-inputs'
    command = [link[0], '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', *includes, str(source), '-o', str(binary), *libraries]
    subprocess.run(command, cwd=build / 'src', check=True)
    prefix = args.dependency_prefix.resolve()
    env = dict(os.environ, GZ_SIM_SERVER_CONFIG_PATH='', GZ_IP='127.0.0.1', GZ_PARTITION=f'gz-rust-inputs-{os.getpid()}', GZ_SIM_SYSTEM_PLUGIN_PATH=str(build / 'lib'), GZ_PHYSICS_ENGINE_PATH=str(prefix / 'lib/gz-physics/engine-plugins'))
    env['LD_LIBRARY_PATH'] = str(build / 'lib') + ':' + str(prefix / 'lib') + ':' + env.get('LD_LIBRARY_PATH', '')
    fixture = ROOT / 'crates/gz-sim-scene/tests/fixtures/physics_inputs.json'
    observations, library_sets = [], {}
    for index, row in enumerate(json.loads(fixture.read_text())):
        maps = work / f'loaded-{index}.maps'
        result = subprocess.run([str(binary), row['mode'], str(maps)], input=row['xml'], env=env, text=True, capture_output=True, timeout=20, check=True)
        actual = [json.loads(line) for line in result.stdout.splitlines() if line.startswith('{')]
        if actual != row['native']:
            raise ValueError(f"{row['engine']}/{row['frame']}/{row['mode']}: {actual}")
        paths = {line.split(maxsplit=5)[5] for line in maps.read_text().splitlines() if len(line.split(maxsplit=5)) == 6 and line.split(maxsplit=5)[5].startswith('/')}
        loaded = {path: digest(Path(path)) for path in sorted(paths) if Path(path).is_file() and '.so' in Path(path).name}
        required = [f"libgz-physics-{row['engine']}-plugin.so.", 'libgz-sim-physics-system.so.', 'libgz-sim.so.']
        for name in required:
            matches = [path for path in loaded if Path(path).name.startswith(name)]
            if len(matches) != 1:
                raise ValueError('missing or duplicate loaded native library: ' + name)
            expected_root = prefix / 'lib' if name.startswith('libgz-physics-') else build / 'lib'
            if loaded[matches[0]] != digest(expected_root / Path(matches[0]).name):
                raise ValueError('native library mismatch: ' + name)
        previous = library_sets.setdefault(row['engine'], loaded)
        if loaded != previous:
            raise ValueError('native library set changed between controls')
        observations.append(dict(engine=row['engine'], frame=row['frame'], mode=row['mode'], native=actual, stderr=result.stderr))
    report = dict(scope='Actual native world-velocity-reset and COM wrench controls. Installed dependencies/existing build provenance qualified; no Rust-engine trajectory/performance parity claimed.', revisions={name: lock[name] for name in sources}, sources=sources, cmake_cache_sha256=digest(build / 'CMakeCache.txt'), source_sha256=digest(source), fixture_sha256=digest(fixture), binary_sha256=digest(binary), command=command, compiler=subprocess.check_output([link[0], '--version'], text=True), loaded_libraries=library_sets, observations=observations)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(f'{len(observations)} native world input controls passed')


if __name__ == '__main__':
    main()
