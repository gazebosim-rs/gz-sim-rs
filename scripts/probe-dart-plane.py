#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Replay fixed-plane trajectories through the pinned native dartsim engine."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import resource
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[1]

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('physics-source', 'prefix', 'work-dir'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--write-fixture', action='store_true')
    args = parser.parse_args()
    source, prefix, work = args.physics_source.resolve(), args.prefix.resolve(), args.work_dir.resolve()
    work.mkdir(parents=True, exist_ok=True)
    helper_spec = importlib.util.spec_from_file_location('reference', ROOT / 'scripts/probe-headless-imu.py')
    helper = importlib.util.module_from_spec(helper_spec)
    helper_spec.loader.exec_module(helper)
    revision = next(row['commit'] for row in json.loads((ROOT / 'planning/baseline.json').read_bytes())['repositories'] if row['name'] == 'gz-physics')
    hashes = helper.checked_sources(source, revision, [
        'dartsim/src/SDFFeatures.cc', 'dartsim/src/ShapeFeatures.cc',
        'dartsim/src/SimulationFeatures.cc', 'dartsim/src/WorldFeatures.cc',
        'include/gz/physics/ForwardStep.hh', 'include/gz/physics/FrameSemantics.hh',
    ])
    env = dict(os.environ, PKG_CONFIG_PATH=str(prefix / 'lib/pkgconfig'))
    includedir = Path(subprocess.check_output(['pkg-config', '--variable=includedir', 'gz-physics'], env=env, text=True).strip())
    for name in ('ForwardStep.hh', 'FrameSemantics.hh'):
        if digest(includedir / 'gz/physics' / name) != hashes['include/gz/physics/' + name]:
            raise ValueError('installed native physics header mismatch')
    dart = Path(subprocess.check_output(['pkg-config', '--variable=libdir', 'dart'], env=env, text=True).strip()).resolve()
    ode = Path(subprocess.check_output(['pkg-config', '--variable=libdir', 'ode'], env=env, text=True).strip()).resolve()
    version = subprocess.check_output(['pkg-config', '--modversion', 'dart'], env=env, text=True).strip()
    if version != '6.13.2':
        raise ValueError('native DART version mismatch')
    env['LD_LIBRARY_PATH'] = os.pathsep.join(map(str, (dart, ode, prefix / 'lib')))
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'gz-physics', 'gz-physics-sdf', 'gz-plugin-loader', 'sdformat', 'gz-math-eigen3'], env=env, text=True))
    probe = ROOT / 'crates/gz-physics-dart/tests/fixtures/plane_reference.cc'
    binary = work / 'reference'
    command = ['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', str(probe), *flags, '-Wl,-rpath,' + str(prefix / 'lib'), '-o', str(binary)]
    subprocess.run(command, env=env, check=True)
    plugin = (prefix / 'lib/libgz-physics-dartsim-plugin.so').resolve()
    maps = work / 'loaded.maps'
    def no_core():
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    result = subprocess.run([str(binary), str(plugin), str(maps)], env=env, capture_output=True, check=True, timeout=60, preexec_fn=no_core)
    (work / 'stdout.json').write_bytes(result.stdout)
    (work / 'stderr').write_bytes(result.stderr)
    if result.stderr:
        raise ValueError('unexpected native output')
    rows = json.loads(result.stdout)
    cases = {row['case'] for row in rows}
    expected_cases = {'center', 'small_size', 'large_size', 'outside_cube', 'footprint_edge', 'tilted', 'translated'}
    if len(rows) != 49 or cases != expected_cases:
        raise ValueError('native plane cases missing')
    for case in cases:
        if [row['step'] for row in rows if row['case'] == case] != [0, 1, 10, 100, 500, 1000, 1500]:
            raise ValueError('native trajectory samples missing')
    loaded = set()
    for line in maps.read_bytes().split(b'\n'):
        fields = line.split(maxsplit=5)
        if len(fields) == 6 and fields[5].startswith(b'/'):
            path = Path(os.fsdecode(fields[5]))
            if path.is_file():
                loaded.add(path.resolve())
    if plugin not in loaded:
        raise ValueError('native plugin library mismatch')
    for name, directory in [('libdart.so.6.13.2', dart), ('libdart-collision-ode.so.6.13.2', dart), ('libdart-external-odelcpsolver.so.6.13.2', dart), ('libode.so', ode)]:
        expected = (directory / name).resolve()
        found = {path for path in loaded if path.name.startswith(name.split('.so')[0] + '.so')}
        if found != {expected}:
            raise ValueError('native runtime library mismatch: ' + name)
    libraries = {path.name: digest(path) for path in loaded if '.so' in path.name}
    if len(libraries) != sum('.so' in path.name for path in loaded):
        raise ValueError('ambiguous native library names')
    fixture = ROOT / 'crates/gz-physics-dart/tests/fixtures/planes.json'
    report = dict(scope='Installed pinned native dartsim engine; fixed local +Z SDF planes, link frames and trajectories. Selected source/headers/runtime libraries verified; no fresh engine build or other-normal/backend/performance claim.', source_revision=revision, sources_sha256=hashes, probe_sha256=digest(probe), runner_sha256=digest(Path(__file__)), dart_version=version, libraries_sha256=libraries, rows=rows)
    if args.write_fixture:
        fixture.write_text(json.dumps(report, indent=2) + '\n')
    elif json.loads(fixture.read_bytes()) != report:
        raise ValueError('native plane fixture differs')
    (work / 'provenance.json').write_text(json.dumps(dict(command=command, compiler=subprocess.check_output(['g++', '--version'], text=True), binary_sha256=digest(binary), libraries={str(path): digest(path) for path in loaded if '.so' in path.name}), indent=2) + '\n')
    print('49 native plane samples across seven physical/frame cases passed')

if __name__ == '__main__':
    main()
