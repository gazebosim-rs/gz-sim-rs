#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Replay generalized force through the pinned MuJoCo C++ API, without actuators."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / 'crates/gz-physics-mujoco/tests/fixtures'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['sdk-root', 'work-dir', 'report']:
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--write-fixture', action='store_true')
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location('physics_bootstrap', ROOT / 'scripts/bootstrap-physics.py')
    bootstrap = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bootstrap)
    sdk, work = args.sdk_root.resolve(), args.work_dir.resolve()
    source, build, prefix = [sdk / name for name in ['mujoco-source', 'mujoco-build', 'mujoco']]
    revision = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    assert revision == bootstrap.MUJOCO_COMMIT
    subprocess.run(['git', '-C', str(source), 'diff', '--quiet', revision, '--'], check=True)
    cache = (build / 'CMakeCache.txt').read_text()
    for key, expected in [('CMAKE_HOME_DIRECTORY:', source), ('CMAKE_INSTALL_PREFIX:', prefix)]:
        actual = next(line.split('=', 1)[1] for line in cache.splitlines() if line.startswith(key))
        assert Path(actual).resolve() == expected
    headers = {}
    for header in (source / 'include/mujoco').glob('*.h'):
        assert header.read_bytes() == (prefix / 'include/mujoco' / header.name).read_bytes()
        headers[header.name] = digest(header)
    assert headers
    build_command = ['cmake', '--build', str(build), '--target', 'mujoco', '--parallel', '2']
    subprocess.run(build_command, check=True, stdout=subprocess.DEVNULL)
    work.mkdir(parents=True, exist_ok=True)
    cpp, binary, maps = FIXTURES / 'joint_effort.cc', work / 'joint-effort', work / 'loaded.maps'
    libdir = build / 'lib'
    library = libdir / ('libmujoco.so.' + bootstrap.MUJOCO_VERSION)
    command = ['c++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
               '-I', str(prefix / 'include'), str(cpp), '-L', str(libdir),
               '-Wl,-rpath,' + str(libdir), '-l:' + library.name, '-o', str(binary)]
    subprocess.run(command, check=True)
    result = subprocess.run([str(binary), str(maps)], env=dict(os.environ, LD_LIBRARY_PATH=str(libdir)),
                            capture_output=True, text=True, check=True, timeout=30)
    assert result.stderr == '', result.stderr
    paths = {Path(line.split(maxsplit=5)[5]) for line in maps.read_text().splitlines()
             if len(line.split(maxsplit=5)) == 6 and line.split(maxsplit=5)[5].startswith('/')}
    loaded = [p for p in paths if p.name.startswith('libmujoco.so.')]
    assert len(loaded) == 1 and loaded[0].resolve() == library.resolve(), loaded
    observations = [json.loads(line) for line in result.stdout.splitlines()]
    assert len(observations) == 6
    fixture = {'source_sha256': digest(cpp), 'mujoco_commit': revision,
               'version': bootstrap.MUJOCO_VERSION, 'observations': observations}
    path = FIXTURES / 'joint_effort.json'
    if args.write_fixture:
        path.write_text(json.dumps(fixture, indent=2) + '\n')
    else:
        assert json.loads(path.read_text()) == fixture
    report = dict(scope='Fresh C++ client and rebuilt pinned MuJoCo target, no actuators; not a fresh SDK configuration or performance comparison.',
                  fixture=fixture, header_hashes=headers, library_sha256=digest(library),
                  cmake_cache_sha256=digest(build / 'CMakeCache.txt'), build_command=build_command,
                  command=command, binary_sha256=digest(binary), stderr=result.stderr,
                  compiler=subprocess.check_output(['c++', '--version'], text=True))
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + '\n')
    print('6 native MuJoCo joint-effort observations passed')


if __name__ == '__main__':
    main()
