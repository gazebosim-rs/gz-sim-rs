#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Reproduce pinned native capsule bounds, model contacts and original shape test."""
import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['physics-source', 'physics-build', 'dependency-prefix', 'work-dir', 'output']:
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location('native_reference', ROOT / 'scripts/probe-headless-imu.py')
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    source, build, prefix, work = (getattr(args, name).resolve() for name in ['physics_source', 'physics_build', 'dependency_prefix', 'work_dir'])
    lock = {row['name']: row['commit'] for row in json.loads((ROOT / 'planning/baseline.json').read_text())['repositories']}
    files = ['tpe/lib/src/Shape.cc', 'tpe/lib/src/Shape.hh', 'tpe/lib/src/Shape_TEST.cc', 'tpe/lib/src/World.cc', 'tpe/lib/src/Model.cc', 'tpe/lib/src/Link.cc', 'tpe/lib/src/Collision.cc', 'tpe/lib/src/CollisionDetector.cc']
    hashes = helper.checked_sources(source, lock['gz-physics'], files)
    compiler = Path(shutil.which('c++')).resolve()
    compiler_hash = helper.digest(compiler)
    cache_hash = helper.digest(build / 'CMakeCache.txt')
    cache = (build / 'CMakeCache.txt').read_text()
    home = next(s.split('=', 1)[1] for s in cache.splitlines() if s.startswith('CMAKE_HOME_DIRECTORY:'))
    if Path(home).resolve() != source:
        raise ValueError('native build belongs to another source')
    work.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, LD_LIBRARY_PATH=os.pathsep.join(map(str, [build / 'lib', prefix / 'lib'])), PKG_CONFIG_PATH=str(prefix / 'lib/pkgconfig'))
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'gz-math', 'gz-utils', 'gz-common-graphics'], env=env, text=True))
    probe = ROOT / 'crates/gz-sim-scene/tests/native_capsule.cc'
    binary = work / 'native-capsule'
    command = [str(compiler), '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', '-I' + str(source / 'tpe/lib/src'), '-I' + str(build / 'include'), str(probe), '-L' + str(build / 'lib'), '-lgz-physics-tpelib', *flags, '-o', str(binary)]
    subprocess.run(command, env=env, check=True)
    result = subprocess.run([str(binary)], env=env, capture_output=True, text=True)
    (work / 'controls.stdout').write_text(result.stdout)
    (work / 'controls.stderr').write_text(result.stderr)
    result.check_returncode()
    if result.stderr:
        raise ValueError('unexpected native control stderr')
    controls = json.loads(result.stdout)
    if controls['radius'] != 0.6 or controls['length'] != 2.8 or controls['min'] != [-0.6, -0.6, -2.0] or controls['max'] != [0.6, 0.6, 2.0]:
        raise ValueError('native capsule axis convention differs')
    if [row['mode'] for row in controls['rows']] != list(range(8)):
        raise ValueError('native contact control modes missing')
    expected_counts = [1, 1, 0, 1, 1, 1, 0, 1]
    for row, count in zip(controls['rows'], expected_counts):
        if len(row['contacts']) != count or any(
                len(point) != 3 or not all(math.isfinite(value) for value in point)
                for point in row['contacts']):
            raise ValueError('native capsule contact count or point differs')
    original = build / 'bin/UNIT_tpelib_Shape_TEST'
    xml = work / 'original.xml'
    result = subprocess.run([str(original), '--gtest_filter=Shape.CapsuleShape', '--gtest_output=xml:' + str(xml)], env=env, capture_output=True, text=True)
    (work / 'original.stdout').write_text(result.stdout)
    (work / 'original.stderr').write_text(result.stderr)
    result.check_returncode()
    original_result = ET.parse(xml).getroot()
    cases = list(original_result.iter('testcase'))
    if (original_result.attrib['tests'] != '1'
            or any(original_result.attrib.get(key, '0') != '0'
                   for key in ('failures', 'errors', 'disabled', 'skipped'))
            or len(cases) != 1 or cases[0].attrib.get('classname') != 'Shape'
            or cases[0].attrib.get('name') != 'CapsuleShape'
            or cases[0].attrib.get('status') != 'run'
            or cases[0].attrib.get('result') != 'completed'
            or any(list(case) for case in cases)):
        raise ValueError('original capsule test did not complete successfully')
    linkage = subprocess.check_output(['ldd', str(binary)], env=env, text=True)
    libraries = {path: helper.digest(Path(path)) for path in linkage.split() if path.startswith('/') and Path(path).is_file()}
    native = [path for path in libraries if Path(path).name.startswith('libgz-physics-tpelib.so.')]
    if len(native) != 1 or libraries[native[0]] != helper.digest(build / 'lib' / Path(native[0]).name):
        raise ValueError('native TPE library differs')
    if (helper.digest(compiler) != compiler_hash
            or helper.digest(build / 'CMakeCache.txt') != cache_hash
            or helper.checked_sources(source, lock['gz-physics'], files) != hashes):
        raise ValueError('native compiler, source or build cache changed')
    report = dict(compiler_path=str(compiler), compiler_sha256=compiler_hash, scope='Pinned native capsule dimensions/bounds, eight TPE model contact controls and one unchanged original. Rust physical construction requires positive representable dimensions; no complete mutable Shape constructor/setter port, solver equivalence or P2 claim. Installed dependency/build provenance remains qualified.', upstream=lock['gz-physics'], source_sha256=hashes, probe_sha256=helper.digest(probe), runner_sha256=helper.digest(Path(__file__)), compiler=subprocess.check_output([str(compiler), '--version'], text=True), command=command, cmake_cache_sha256=helper.digest(build / 'CMakeCache.txt'), binary_sha256=helper.digest(binary), original_binary_sha256=helper.digest(original), original_xml_sha256=helper.digest(xml), original_tests=1, libraries=libraries, controls=controls)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print('Native capsule bounds, eight contact modes and unchanged original passed')


if __name__ == '__main__':
    main()
