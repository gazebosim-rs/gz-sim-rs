#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Reproduce pinned native Sensor sampling clock controls with SDK dependencies."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dependency-prefix', type=Path, required=True)
    parser.add_argument('--sensor-source', type=Path, required=True)
    parser.add_argument('--work-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    work = args.work_dir.resolve()
    work.mkdir(parents=True, exist_ok=True)
    upstream = args.sensor_source.resolve()
    revision = subprocess.check_output(['git', '-C', str(upstream), 'rev-parse', 'HEAD'], text=True).strip()
    if revision != '8ddf4c17be8a434c51b51d5be7c866fb69aaa54f':
        raise ValueError('native sensor revision mismatch')
    sources = {}
    for name in ['src/Sensor.cc', 'src/Sensor_TEST.cc', 'include/gz/sensors/Sensor.hh']:
        original = subprocess.check_output(['git', '-C', str(upstream), 'show', revision + ':' + name])
        if original != (upstream / name).read_bytes():
            raise ValueError('native source changed: ' + name)
        sources[name] = digest(upstream / name)
    prefix = args.dependency_prefix.resolve()
    env = dict(os.environ, PKG_CONFIG_PATH=str(prefix / 'lib/pkgconfig'), GZ_IP='127.0.0.1', GZ_PARTITION=f'gz-rust-clock-{os.getpid()}')
    env['LD_LIBRARY_PATH'] = str(upstream / 'build/lib') + ':' + str(prefix / 'lib') + ':' + env.get('LD_LIBRARY_PATH', '')
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'gz-sensors'], env=env, text=True))
    include = Path(subprocess.check_output(['pkg-config', '--variable=includedir', 'gz-sensors'], env=env, text=True).strip())
    if digest(include / 'gz/sensors/Sensor.hh') != sources['include/gz/sensors/Sensor.hh']:
        raise ValueError('native SDK header mismatch')
    source = ROOT / 'crates/gz-sim-scene/tests/native_sensor_schedule.cc'
    binary = work / 'schedule'
    command = ['c++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', str(source), *flags, '-o', str(binary)]
    subprocess.run(command, env=env, check=True)
    fixture = ROOT / 'crates/gz-sim-scene/tests/fixtures/sensor_schedule.json'
    rows = json.loads(fixture.read_text())
    observations = []
    for row in rows:
        result = subprocess.run([str(binary), str(row['rate_hz']), str(row['start_ns']), str(row['now_ns']), str(int(row['forced']))], env=env, text=True, capture_output=True)
        if result.returncode:
            raise ValueError(row['name'] + ': ' + result.stderr)
        actual = json.loads(result.stdout)
        if actual != {key: row[key] for key in ['sampled', 'next_ns']}:
            raise ValueError(f"{row['name']}: {actual}")
        observations.append({'name': row['name'], 'native': actual})
    linkage = subprocess.check_output(['ldd', str(binary)], env=env, text=True)
    libraries = {item: digest(Path(item)) for item in linkage.split() if item.startswith('/') and Path(item).is_file()}
    sensor_lib = [path for path in libraries if Path(path).name.startswith('libgz-sensors.so.')]
    if len(sensor_lib) != 1 or libraries[sensor_lib[0]] != digest(upstream / 'build/lib' / Path(sensor_lib[0]).name):
        raise ValueError('native scheduler library mismatch')
    subprocess.run(['cmake', '--build', str(upstream / 'build'), '--target', 'UNIT_Sensor_TEST', '-j2'], env=env, check=True)
    original = upstream / 'build/bin/UNIT_Sensor_TEST'
    xml_report = work / 'original.xml'
    original_run = subprocess.run([str(original), '--gtest_filter=SensorUpdate.NextDataUpdateTime', '--gtest_output=xml:' + str(xml_report)], env=env, text=True, capture_output=True, check=True)
    (work / 'original.log').write_text(original_run.stdout + original_run.stderr)
    result = ET.parse(xml_report).getroot()
    if result.attrib['tests'] != '1' or result.attrib['failures'] != '0':
        raise ValueError('original native scheduling test failed or did not run')
    report = dict(scope='Sampling clock only; installed dependencies qualified, not full sensor-stream/simulator parity.', revision=revision, native_sources=sources, command=command, source_sha256=digest(source), fixture_sha256=digest(fixture), binary_sha256=digest(binary), libraries=libraries, observations=observations, original_binary_sha256=digest(original), original_tests=1, original_failures=0)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(f'{len(rows)} native schedule controls passed')


if __name__ == '__main__':
    main()
