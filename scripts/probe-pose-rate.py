#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Check pinned native 60 wall-Hz publisher semantics used by SceneBroadcaster."""
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
    for name in ['sim-source', 'transport-source', 'sim-build', 'dependency-prefix', 'work-dir', 'output']:
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location('reference', ROOT / 'scripts/probe-headless-imu.py')
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    lock = {row['name']: row['commit'] for row in json.loads((ROOT / 'planning/baseline.json').read_text())['repositories']}
    sources = {
        'gz-sim': helper.checked_sources(args.sim_source.resolve(), lock['gz-sim'], [
            'src/systems/scene_broadcaster/SceneBroadcaster.cc']),
        'gz-transport': helper.checked_sources(args.transport_source.resolve(), lock['gz-transport'], [
            'src/Node.cc', 'src/Node_TEST.cc']),
    }
    build, prefix, work = args.sim_build.resolve(), args.dependency_prefix.resolve(), args.work_dir.resolve()
    work.mkdir(parents=True, exist_ok=True)
    cache = build / 'CMakeCache.txt'
    home = next(line.split('=', 1)[1] for line in cache.read_text().splitlines() if line.startswith('CMAKE_HOME_DIRECTORY:'))
    if Path(home).resolve() != args.sim_source.resolve():
        raise ValueError('native build belongs to another source directory')
    target = build / 'src/CMakeFiles/UNIT_ServerConfig_TEST.dir'
    flags = (target / 'flags.make').read_text()
    includes = shlex.split(next(line.split('=', 1)[1] for line in flags.splitlines() if line.startswith('CXX_INCLUDES =')))
    compiler = shlex.split((target / 'link.txt').read_text())[0]
    source = ROOT / 'crates/gz-sim-server/tests/native_pose_rate.cc'
    binary = work / 'native-pose-rate'
    command = [compiler, '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', *includes,
               str(source), '-o', str(binary), '-L' + str(prefix / 'lib'),
               '-lgz-transport', '-lgz-msgs', '-lprotobuf', '-lpthread']
    subprocess.run(command, check=True)
    observations, loaded = [], None
    for index in range(5):
        maps = work / f'control-{index}.maps'
        env = dict(os.environ, GZ_IP='127.0.0.1', GZ_PARTITION=f'pose-rate-{os.getpid()}-{index}',
                   GZ_TRANSPORT_IMPLEMENTATION='zeromq', GZ_TRANSPORT_TOPIC_STATISTICS='0')
        for name in ['GZ_RELAY', 'GZ_TRANSPORT_USERNAME', 'GZ_TRANSPORT_PASSWORD']:
            env.pop(name, None)
        env['LD_LIBRARY_PATH'] = str(prefix / 'lib') + ':' + env.get('LD_LIBRARY_PATH', '')
        result = subprocess.run([str(binary), str(maps)], env=env, text=True, capture_output=True, timeout=10)
        (work / f'control-{index}.stdout').write_text(result.stdout)
        (work / f'control-{index}.stderr').write_text(result.stderr)
        result.check_returncode()
        if result.stderr:
            raise ValueError('unexpected native stderr: ' + result.stderr)
        row = json.loads(result.stdout)
        if row['received'] != [0, 99] or not 0 <= row['burst_ns'] < 16_666_667 or row['next_ready_ns'] < 16_666_667:
            raise ValueError('native rate control differs')
        paths = {line.split(maxsplit=5)[5] for line in maps.read_text().splitlines()
                 if len(line.split(maxsplit=5)) == 6 and line.split(maxsplit=5)[5].startswith('/')}
        libraries = {path: helper.digest(Path(path)) for path in sorted(paths)
                     if Path(path).is_file() and '.so' in Path(path).name}
        for name in ['libgz-transport.so.', 'libgz-msgs.so.']:
            matches = [path for path in libraries if Path(path).name.startswith(name)]
            if len(matches) != 1 or libraries[matches[0]] != helper.digest(prefix / 'lib' / Path(matches[0]).name):
                raise ValueError('missing or different native library: ' + name)
        if loaded is not None and loaded != libraries:
            raise ValueError('native libraries changed between controls')
        loaded = libraries
        observations.append(row)
    report = dict(
        scope='Actual pinned native Transport publisher at the unchanged SceneBroadcaster fixed60 setting; source-backed reset timer reuse. Five independent processes, first-frame and shared-copy suppression, success-return and next eligibility controls. Not a full Server reset/cadence test or performance measurement. Installed dependency/build provenance qualified.',
        revisions={name: lock[name] for name in sources}, sources=sources,
        source_sha256=helper.digest(source), runner_sha256=helper.digest(Path(__file__)),
        transport_environment=dict(implementation='zeromq', statistics=False, relay=False, authentication=False),
        binary_sha256=helper.digest(binary),
        cmake_cache_sha256=helper.digest(cache), command=command,
        compiler=subprocess.check_output([compiler, '--version'], text=True),
        loaded_libraries=loaded, observations=observations)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print('Five native 60 wall-Hz publisher controls passed')


if __name__ == '__main__':
    main()
