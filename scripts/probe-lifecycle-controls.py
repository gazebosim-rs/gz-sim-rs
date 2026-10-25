#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Rebuild selected native reset/removal controls on the shared headless SDF."""
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['sim-source', 'physics-source', 'sim-build', 'dependency-prefix', 'work-dir', 'output']:
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    work = args.work_dir.resolve()
    work.mkdir(parents=True, exist_ok=True)
    build = args.sim_build.resolve()
    spec = importlib.util.spec_from_file_location('reference', ROOT / 'scripts/probe-headless-imu.py')
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    lock = {row['name']: row['commit'] for row in json.loads((ROOT / 'planning/baseline.json').read_text())['repositories']}
    sources = {
        'gz-sim': helper.checked_sources(args.sim_source.resolve(), lock['gz-sim'], [
            'src/Server.cc', 'src/ServerPrivate.cc', 'src/TestFixture.cc',
            'src/SimulationRunner.cc', 'src/systems/user_commands/UserCommands.cc',
            'src/systems/physics/Physics.cc']),
        'gz-physics': helper.checked_sources(args.physics_source.resolve(), lock['gz-physics'], [
            'dartsim/src/SimulationFeatures.cc', 'tpe/plugin/src/SimulationFeatures.cc']),
    }
    cache = (build / 'CMakeCache.txt').read_text()
    home = next(line.split('=', 1)[1] for line in cache.splitlines() if line.startswith('CMAKE_HOME_DIRECTORY:'))
    if Path(home).resolve() != args.sim_source.resolve():
        raise ValueError('native build belongs to another source directory')
    target = build / 'src/CMakeFiles/UNIT_ServerConfig_TEST.dir'
    flags = (target / 'flags.make').read_text()
    includes = shlex.split(next(line.split('=', 1)[1] for line in flags.splitlines() if line.startswith('CXX_INCLUDES =')))
    link = shlex.split((target / 'link.txt').read_text())
    libraries = [str((build / 'src' / item).resolve()) if item.startswith('../') else item
                 for item in link[link.index('-o') + 2:] if 'gtest' not in item]
    source = ROOT / 'crates/gz-sim-server/tests/native_lifecycle_controls.cc'
    binary = work / 'native-lifecycle-controls'
    command = [link[0], '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', *includes, str(source), '-o', str(binary), *libraries]
    subprocess.run(command, cwd=build / 'src', check=True)
    prefix = args.dependency_prefix.resolve()
    env = dict(os.environ, GZ_SIM_SERVER_CONFIG_PATH='', GZ_IP='127.0.0.1',
               GZ_SIM_SYSTEM_PLUGIN_PATH=str(build / 'lib'),
               GZ_SIM_PHYSICS_ENGINE_PATH=str(prefix / 'lib/gz-physics/engine-plugins'))
    env['LD_LIBRARY_PATH'] = str(build / 'lib') + ':' + str(prefix / 'lib') + ':' + env.get('LD_LIBRARY_PATH', '')
    fixture = ROOT / 'crates/gz-sim-scene/tests/fixtures/headless_imu.sdf'
    rows, libraries = [], {}
    for engine in ['dartsim', 'tpe']:
        for mode in ['ordinary', 'overwrite', 'model-only', 'time-only', 'step-reset']:
            env['GZ_PARTITION'] = f'lifecycle-{os.getpid()}-{engine}-{mode}'
            maps = work / f'{engine}-{mode}.maps'
            result = subprocess.run([str(binary), mode, engine, str(maps)], input=fixture.read_text(),
                                    env=env, text=True, capture_output=True, timeout=25, check=True)
            if result.stderr:
                raise ValueError('unexpected native stderr: ' + result.stderr)
            row = json.loads(result.stdout)
            if row['mode'] != mode or row['engine'] != engine or row['original'] <= 0:
                raise ValueError('wrong native control or missing original identity')
            expected_resets = int(mode not in ['overwrite', 'model-only'])
            expected_ball = 0 if mode == 'model-only' else row['original']
            expected_time = 2_000_000 if mode == 'step-reset' else 0
            if (row['resets'], row['ball'], row['time_ns']) != (expected_resets, expected_ball, expected_time):
                raise ValueError('native lifecycle observation differs: ' + str(row))
            expected_pose = None if mode == 'model-only' else [0, 0, 5 - (0.00002943 if engine == 'dartsim' and mode == 'step-reset' else 0), 0, 0, 0, 1]
            if expected_pose is None:
                if row['pose'] is not None:
                    raise ValueError('removed model still has a pose')
            elif not isinstance(row['pose'], list) or len(row['pose']) != 7 or any(
                    not math.isfinite(actual) or abs(actual - expected) >= 1e-12
                    for actual, expected in zip(row['pose'], expected_pose, strict=True)):
                raise ValueError('native model pose differs')
            loaded = {line.split(maxsplit=5)[5] for line in maps.read_text().splitlines()
                      if len(line.split(maxsplit=5)) == 6 and line.split(maxsplit=5)[5].startswith('/')}
            required = {
                'libgz-sim.so.': build / 'lib',
                'libgz-sim-physics-system.so.': build / 'lib',
                'libgz-sim-user-commands-system.so.': build / 'lib',
                f'libgz-physics-{engine}-plugin.so.': prefix / 'lib/gz-physics/engine-plugins',
            }
            for name, directory in required.items():
                found = [Path(path) for path in loaded if Path(path).name.startswith(name)]
                if len(found) != 1 or found[0].resolve() != (directory / found[0].name).resolve():
                    raise ValueError('wrong or duplicate loaded native library: ' + name)
            libraries[engine + ':' + mode] = {path: digest(Path(path)) for path in sorted(loaded)
                                            if Path(path).is_file() and '.so' in Path(path).name}
            if not any(f'gz-physics-{engine}-plugin' in path for path in loaded):
                raise ValueError('requested native backend not loaded')
            rows.append(row)
    proof = dict(scope='Selected shared-SDF native control precedence, queued removal and reset identity/iteration observations; no performance or full P2 acceptance. Installed dependency/build provenance qualified.',
                 revisions={name: lock[name] for name in sources}, sources=sources,
                 cmake_cache_sha256=digest(build / 'CMakeCache.txt'), source_sha256=digest(source),
                 fixture_sha256=digest(fixture), binary_sha256=digest(binary), command=command,
                 compiler=subprocess.check_output([link[0], '--version'], text=True),
                 runner_sha256=digest(Path(__file__).resolve()), helper_sha256=digest(ROOT / 'scripts/probe-headless-imu.py'),
                 libraries=libraries, observations=rows)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(proof, indent=2) + '\n')
    print(f'{len(rows)} native lifecycle controls passed')


if __name__ == '__main__':
    main()
