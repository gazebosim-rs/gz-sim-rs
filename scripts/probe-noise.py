#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Reproduce native Gaussian controls and seven unchanged original noise tests."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import shlex
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['sensor-source', 'sensor-build', 'math-source', 'dependency-prefix', 'work-dir', 'output']:
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--runtime-library-dir', type=Path, action='append', default=[])
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location('reference', ROOT / 'scripts/probe-headless-imu.py')
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    lock = {row['name']: row['commit'] for row in json.loads((ROOT / 'planning/baseline.json').read_text())['repositories']}
    sources = {
        'gz-sensors': helper.checked_sources(args.sensor_source.resolve(), lock['gz-sensors'], [
            'src/Noise.cc', 'src/GaussianNoiseModel.cc', 'src/Noise_TEST.cc', 'src/ImuSensor.cc',
            'include/gz/sensors/Noise.hh', 'include/gz/sensors/GaussianNoiseModel.hh']),
        'gz-math': helper.checked_sources(args.math_source.resolve(), lock['gz-math'], [
            'src/Rand.cc', 'include/gz/math/Rand.hh']),
    }
    build, prefix, work = args.sensor_build.resolve(), args.dependency_prefix.resolve(), args.work_dir.resolve()
    work.mkdir(parents=True, exist_ok=True)
    cache = build / 'CMakeCache.txt'
    home = next(s.split('=', 1)[1] for s in cache.read_text().splitlines() if s.startswith('CMAKE_HOME_DIRECTORY:'))
    if Path(home).resolve() != args.sensor_source.resolve():
        raise ValueError('native build belongs to another source directory')
    compiler = next(s.split('=', 1)[1] for s in cache.read_text().splitlines() if s.startswith('CMAKE_CXX_COMPILER:'))
    flags = (build / 'src/CMakeFiles/gz-sensors.dir/flags.make').read_text()
    includes = shlex.split(next(s.split('=', 1)[1] for s in flags.splitlines() if s.startswith('CXX_INCLUDES =')))
    source = ROOT / 'crates/gz-sensors/tests/native_noise.cc'
    binary = work / 'native-noise'
    command = [compiler, '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', *includes,
               str(source), '-L' + str(build / 'lib'), '-L' + str(prefix / 'lib'),
               '-lgz-sensors', '-lgz-math', '-lsdformat', '-o', str(binary)]
    env = dict(os.environ, LD_LIBRARY_PATH=':'.join(map(str, [build / 'lib', prefix / 'lib',
                   *[p.resolve() for p in args.runtime_library_dir]])))
    subprocess.run(command, env=env, check=True)
    result = subprocess.run([str(binary)], env=env, text=True, capture_output=True)
    (work / 'controls.stdout').write_text(result.stdout)
    (work / 'controls.stderr').write_text(result.stderr)
    result.check_returncode()
    if result.stderr:
        raise ValueError('unexpected native control stderr: ' + result.stderr)
    rows = [json.loads(line) for line in result.stdout.splitlines()]
    if [r['mode'] for r in rows] != ['none', 'mean', 'precision', 'white', 'bias', 'dynamic']:
        raise ValueError('missing or extra native control modes')
    for row in rows:
        if len(row['samples']) != 16:
            raise ValueError('wrong native sample count')
        for sample in row['samples']:
            value, before = sample['value'], sample['input']
            expected = {'none': before, 'mean': before + 10, 'bias': before + row['initial_bias'],
                        'dynamic': before + sample['bias']}.get(row['mode'])
            if expected is not None and value != expected:
                raise ValueError('native deterministic control differs')
            if row['mode'] == 'precision' and abs(value / 0.3 - round(value / 0.3)) > 1e-12:
                raise ValueError('native quantization differs')
    subprocess.run(['cmake', '--build', str(build), '--target', 'UNIT_Noise_TEST', '-j2'], env=env, check=True)
    original = build / 'bin/UNIT_Noise_TEST'
    xml = work / 'original.xml'
    original_run = subprocess.run([str(original), '--gtest_output=xml:' + str(xml)], env=env, text=True, capture_output=True)
    (work / 'original.stdout').write_text(original_run.stdout)
    (work / 'original.stderr').write_text(original_run.stderr)
    original_run.check_returncode()
    tree = ET.parse(xml).getroot()
    if tree.attrib['tests'] != '7' or tree.attrib['failures'] != '0' or tree.attrib['disabled'] != '0':
        raise ValueError('original noise tests did not all pass')
    linkage = subprocess.check_output(['ldd', str(binary)], env=env, text=True)
    libraries = {path: helper.digest(Path(path)) for path in linkage.split() if path.startswith('/') and Path(path).is_file()}
    native = [p for p in libraries if Path(p).name.startswith('libgz-sensors.so.')]
    if len(native) != 1 or libraries[native[0]] != helper.digest(build / 'lib' / Path(native[0]).name):
        raise ValueError('native noise library mismatch')
    report = dict(scope='Native scalar NoiseModel controls and seven unchanged NoiseTest cases; expected negative-case original stderr retained. RNG outputs characterize native only, not cross-language seeded byte identity. IMU acquisition float-dt/covariance are source-backed here, not a complete noisy native Server stream. Installed dependency/build provenance qualified.',
                  revisions={name: lock[name] for name in sources}, sources=sources,
                  source_sha256=helper.digest(source), runner_sha256=helper.digest(Path(__file__)),
                  binary_sha256=helper.digest(binary), cmake_cache_sha256=helper.digest(cache),
                  compiler=subprocess.check_output([compiler, '--version'], text=True), command=command,
                  libraries=libraries, observations=rows, original_tests=7, original_failures=0,
                  original_binary_sha256=helper.digest(original),
                  original_xml_sha256=helper.digest(xml))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print('Six native controls and seven unchanged noise tests passed')


if __name__ == '__main__':
    main()
