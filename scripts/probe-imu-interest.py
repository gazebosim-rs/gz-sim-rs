#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Reproduce native IMU late/shared interest and inactive-gap sequences."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['sim-source', 'sensor-source', 'transport-source', 'physics-source',
                 'sim-build', 'sensors-build', 'physics-build', 'dependency-prefix',
                 'reference-prefix', 'work-dir', 'output']:
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location('reference', ROOT / 'scripts/probe-headless-imu.py')
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    lock = {row['name']: row['commit'] for row in json.loads((ROOT / 'planning/baseline.json').read_text())['repositories']}
    sources = {
        'gz-sim': helper.checked_sources(args.sim_source.resolve(), lock['gz-sim'], ['src/systems/imu/Imu.cc']),
        'gz-sensors': helper.checked_sources(args.sensor_source.resolve(), lock['gz-sensors'], ['src/Sensor.cc', 'src/ImuSensor.cc']),
        'gz-transport': helper.checked_sources(args.transport_source.resolve(), lock['gz-transport'], ['src/Node.cc']),
        'gz-physics': helper.checked_sources(args.physics_source.resolve(), lock['gz-physics'], ['tpe/plugin/src/SimulationFeatures.cc']),
    }
    build, work = args.sim_build.resolve(), args.work_dir.resolve()
    work.mkdir(parents=True, exist_ok=True)
    cache = build / 'CMakeCache.txt'
    home = next(s.split('=', 1)[1] for s in cache.read_text().splitlines() if s.startswith('CMAKE_HOME_DIRECTORY:'))
    if Path(home).resolve() != args.sim_source.resolve():
        raise ValueError('native build belongs to another source directory')
    target = build / 'src/CMakeFiles/UNIT_ServerConfig_TEST.dir'
    flags = (target / 'flags.make').read_text()
    includes = shlex.split(next(s.split('=', 1)[1] for s in flags.splitlines() if s.startswith('CXX_INCLUDES =')))
    link = shlex.split((target / 'link.txt').read_text())
    libraries = [str((build / 'src' / s).resolve()) if s.startswith('../') else s
                 for s in link[link.index('-o') + 2:] if 'gtest' not in s]
    source = ROOT / 'crates/gz-sim-server/tests/native_imu_interest.cc'
    binary = work / 'native-imu-interest'
    command = [link[0], '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', *includes,
               str(source), '-o', str(binary), *libraries]
    subprocess.run(command, cwd=build / 'src', check=True)
    roots = [args.physics_build.resolve() / 'lib', args.sensors_build.resolve() / 'lib',
             build / 'lib', args.reference_prefix.resolve() / 'lib', args.dependency_prefix.resolve() / 'lib']
    env = dict(os.environ, LD_LIBRARY_PATH=':'.join(map(str, roots)), GZ_IP='127.0.0.1',
               GZ_SIM_SERVER_CONFIG_PATH='', GZ_SIM_SYSTEM_PLUGIN_PATH=str(build / 'lib'),
               GZ_SIM_PHYSICS_ENGINE_PATH=str(args.physics_build.resolve() / 'lib'),
               GZ_TRANSPORT_IMPLEMENTATION='zeromq', GZ_TRANSPORT_TOPIC_STATISTICS='0')
    for key in ['GZ_TRANSPORT_USERNAME', 'GZ_TRANSPORT_PASSWORD', 'GZ_RELAY']:
        env.pop(key, None)
    fixture = ROOT / 'crates/gz-sim-scene/tests/fixtures/imu_interest.sdf'
    observations, loaded = {}, {}
    for mode, initial, sequence, slow in [('late', 50, 0, 0), ('shared', 100, 2, 2)]:
        env['GZ_PARTITION'] = f'imu-interest-{os.getpid()}-{mode}'
        maps = work / (mode + '.maps')
        result = subprocess.run([str(binary), mode, str(maps)], env=env, input=fixture.read_text(),
                                capture_output=True, text=True, timeout=20)
        (work / (mode + '.stdout')).write_text(result.stdout)
        (work / (mode + '.stderr')).write_text(result.stderr)
        result.check_returncode()
        if result.stderr:
            raise ValueError('unexpected native stderr: ' + result.stderr)
        actual = json.loads(result.stdout)
        expected = dict(mode=mode, slow_count=slow, fast=[
            dict(stamp_ns=(initial + 1) * 1_000_000, sequence=sequence),
            dict(stamp_ns=(initial + 12) * 1_000_000, sequence=sequence + 1)])
        if actual != expected:
            raise ValueError('native acquisition/sequence contract differs')
        paths = {line.split(maxsplit=5)[5] for line in maps.read_text().splitlines()
                 if len(line.split(maxsplit=5)) == 6 and line.split(maxsplit=5)[5].startswith('/')}
        libs = {p: helper.digest(Path(p)) for p in sorted(paths) if Path(p).is_file() and '.so' in Path(p).name}
        for name, directory in [('libgz-sim.so.', build / 'lib'),
                                ('libgz-sim-imu-system.so.', build / 'lib'),
                                ('libgz-sensors-imu.so.', args.sensors_build.resolve() / 'lib'),
                                ('libgz-physics-tpe-plugin.so.', args.physics_build.resolve() / 'lib')]:
            matches = [p for p in libs if Path(p).name.startswith(name)]
            if len(matches) != 1 or libs[matches[0]] != helper.digest(directory / Path(matches[0]).name):
                raise ValueError('native loaded library differs: ' + name)
        observations[mode], loaded[mode] = actual, libs
    report = dict(scope='Actual headless TPE Server+Physics+Imu and same-process native subscribers. Late activation, global any-due-connected gate updating other due sensors, inactive gap without sequence consumption. Not remote readiness, runtime SetActive, renderer or broad SENSOR01 acceptance. Installed dependency/build provenance qualified.',
                  revisions={name: lock[name] for name in sources}, sources=sources,
                  source_sha256=helper.digest(source), runner_sha256=helper.digest(Path(__file__)),
                  fixture_sha256=helper.digest(fixture), binary_sha256=helper.digest(binary),
                  cmake_cache_sha256=helper.digest(cache), command=command,
                  compiler=subprocess.check_output([link[0], '--version'], text=True),
                  loaded_libraries=loaded, observations=observations)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print('Two native late/shared IMU acquisition controls passed')


if __name__ == '__main__':
    main()
