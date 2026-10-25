#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Reproduce pinned native Server physics-profile clock selection without plugins."""
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sim-source', type=Path, required=True)
    parser.add_argument('--sim-build', type=Path, required=True)
    parser.add_argument('--dependency-prefix', type=Path, required=True)
    parser.add_argument('--work-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    upstream, build = args.sim_source.resolve(), args.sim_build.resolve()
    work = args.work_dir.resolve()
    work.mkdir(parents=True, exist_ok=True)
    revision = subprocess.check_output(['git', '-C', str(upstream), 'rev-parse', 'HEAD'], text=True).strip()
    if revision != '71c60db31fddd5c98d2aa689e9a2cccb3548a0a3':
        raise ValueError('native simulator revision mismatch')
    sources = {}
    for name in ['src/SimulationRunner.cc', 'src/SdfEntityCreator.cc', 'src/ServerConfig.cc', 'include/gz/sim/Server.hh', 'include/gz/sim/ServerConfig.hh', 'include/gz/sim/Types.hh']:
        original = subprocess.check_output(['git', '-C', str(upstream), 'show', revision + ':' + name])
        if original != (upstream / name).read_bytes():
            raise ValueError('native source changed: ' + name)
        sources[name] = digest(upstream / name)
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
    source = ROOT / 'crates/gz-sim-scene/tests/native_world_physics.cc'
    binary = work / 'native-profile'
    command = [link[0], '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', *includes, str(source), '-o', str(binary), *libraries]
    subprocess.run(command, cwd=build / 'src', check=True)
    env = dict(os.environ, GZ_SIM_SERVER_CONFIG_PATH='', GZ_IP='127.0.0.1', GZ_PARTITION=f'gz-rust-profile-{os.getpid()}')
    env['LD_LIBRARY_PATH'] = str(build / 'lib') + ':' + str(args.dependency_prefix.resolve() / 'lib') + ':' + env.get('LD_LIBRARY_PATH', '')
    fixture = ROOT / 'crates/gz-sim-scene/tests/fixtures/world_physics.json'
    observations = []
    for row in json.loads(fixture.read_text()):
        result = subprocess.run([str(binary)], input=row['xml'], env=env, text=True, capture_output=True, timeout=15, check=True)
        actual = json.loads(result.stdout)
        if actual != row['native']:
            raise ValueError(f"{row['name']}: {actual}")
        observations.append({'name': row['name'], 'native': actual, 'stderr': result.stderr})
    linkage = subprocess.check_output(['ldd', str(binary)], env=env, text=True)
    loaded = {item: digest(Path(item)) for item in linkage.split() if item.startswith('/') and Path(item).is_file()}
    sim_lib = [path for path in loaded if Path(path).name.startswith('libgz-sim.so.')]
    if len(sim_lib) != 1 or loaded[sim_lib[0]] != digest(build / 'lib' / Path(sim_lib[0]).name):
        raise ValueError('native simulator library mismatch')
    report = dict(scope='Actual native Server clock/configuration; plugins explicitly disabled, no physics/sensor/performance comparison. Installed dependencies and existing build provenance qualified.', revision=revision, sources=sources, cmake_cache_sha256=digest(build / 'CMakeCache.txt'), command=command, compiler=subprocess.check_output([link[0], '--version'], text=True), source_sha256=digest(source), fixture_sha256=digest(fixture), binary_sha256=digest(binary), libraries=loaded, observations=observations)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(f'{len(observations)} native Server clock controls passed')


if __name__ == '__main__':
    main()
