#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Capture native activation observations without declaring Rust parity."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import traceback
import uuid
import xml.etree.ElementTree as ET


def validate_observations(actual):
    assert type(actual) is dict and set(actual) == {'groups'}
    assert type(actual['groups']) is list and len(actual['groups']) == 2
    labels = ['active_before_deadline', 'inactive_before_deadline',
              'forced_inactive_before_deadline', 'reenabled_before_deadline',
              'inactive_at_deadline', 'reenabled_at_deadline', 'inactive_overdue',
              'forced_inactive_overdue', 'reenabled_overdue']
    times = [0, 50, 50, 50, 100, 100, 250, 250, 250]
    forces = [False, False, True, False, False, False, False, True, False]
    fields = {'active', 'rate_hz', 'next_ns', 'count'}

    def state(value):
        assert type(value['active']) is bool
        assert type(value['rate_hz']) in (int, float) and math.isfinite(value['rate_hz'])
        assert type(value['next_ns']) is int and type(value['count']) is int

    for group in actual['groups']:
        assert type(group) is dict and set(group) == {'initial', 'rows'}
        assert type(group['initial']) is dict and set(group['initial']) == fields
        state(group['initial'])
        assert type(group['rows']) is list and len(group['rows']) == 9
        for row, label, now, force in zip(group['rows'], labels, times, forces):
            assert type(row) is dict and set(row) == fields | {'label', 'now_ms', 'force', 'sampled'}
            assert row['label'] == label and type(row['label']) is str
            assert type(row['now_ms']) is int and row['now_ms'] == now
            assert type(row['force']) is bool and row['force'] is force
            assert type(row['sampled']) is bool
            state(row)


def validate_original(path, source):
    xml = ET.parse(path).getroot()
    assert xml.tag == 'testsuites'
    suites = xml.findall('testsuite')
    assert len(suites) == 1 and suites[0].get('name') == 'Sensor_TEST'
    for level in (xml, suites[0]):
        assert level.get('tests') == '1'
        for key in ('failures', 'disabled', 'errors'):
            assert level.get(key) == '0'
        assert level.get('skipped', '0') == '0'
    assert suites[0].get('skipped') == '0'
    cases = xml.findall('.//testcase')
    assert len(cases) == 1
    case = cases[0]
    assert (case.get('classname'), case.get('name'), case.get('status'), case.get('result')) == (
        'Sensor_TEST', 'Sensor', 'run', 'completed')
    assert Path(case.get('file', '')).resolve() == source / 'src/Sensor_TEST.cc'
    assert all(xml.find('.//' + name) is None for name in ('failure', 'error', 'skipped'))


def dependencies(path):
    target, prerequisites = path.read_text().replace('\\\n', '').split(':', 1)
    assert target.strip() == 'activation-probe'
    paths = {Path(name).resolve(strict=True) for name in shlex.split(prerequisites)}
    assert paths
    return paths


ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, required=True)
    parser.add_argument('--sensor-source', type=Path, default=ROOT.parent / 'gz-sensors')
    parser.add_argument('--sensor-build', type=Path)
    parser.add_argument('--compiler', default='c++')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    source = args.sensor_source.resolve()
    build = (args.sensor_build or source / 'build').resolve()
    prefix = args.prefix.resolve()
    cpp = ROOT / 'crates/gz-sim-scene/tests/native_imu_activation.cc'
    sha = lambda path: hashlib.sha256(Path(path).read_bytes()).hexdigest()
    inputs = {}
    libraries = {}
    report = {'state': 'started', 'stage': 'inputs', 'scope': 'Base Sensor activation/getters/update ordering only. No actual IMU stream, simulator reset, noise, forced Rust API or P2 claim.'}
    env = dict(os.environ)

    def run(stage, command, timeout=60):
        report['stage'] = stage
        (out / (stage + '-command.json')).write_text(json.dumps(command) + '\n')
        try:
            result = subprocess.run(command, env=env, capture_output=True, timeout=timeout)
        except subprocess.TimeoutExpired as error:
            (out / (stage + '.stdout')).write_bytes(error.stdout or b'')
            (out / (stage + '.stderr')).write_bytes(error.stderr or b'')
            raise
        (out / (stage + '.stdout')).write_bytes(result.stdout)
        (out / (stage + '.stderr')).write_bytes(result.stderr)
        (out / (stage + '-exit.json')).write_text(json.dumps({'returncode': result.returncode}) + '\n')
        result.check_returncode()
        return result.stdout

    try:
        baseline_path = ROOT / 'planning/baseline.json'
        prior_path = ROOT / 'planning/reference/sensor-schedule.json'
        pins = {row['name']: row['commit'] for row in json.loads(baseline_path.read_text())['repositories']}
        prior = json.loads(prior_path.read_text())
        assert prior['revision'] == pins['gz-sensors']
        assert subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip() == pins['gz-sensors']
        for name in ('src/Sensor.cc', 'src/Sensor_TEST.cc', 'include/gz/sensors/Sensor.hh'):
            path = source / name
            assert path.read_bytes() == subprocess.check_output(['git', '-C', str(source), 'show', pins['gz-sensors'] + ':' + name])
            assert sha(path) == prior['native_sources'][name]
            inputs[str(path)] = sha(path)
        cache = build / 'CMakeCache.txt'
        home = next(line.split('=', 1)[1] for line in cache.read_text().splitlines() if line.startswith('CMAKE_HOME_DIRECTORY:'))
        assert Path(home).resolve() == source
        driver = shutil.which(args.compiler)
        if not driver:
            raise ValueError('C++ compiler is unavailable')
        # Keep the invoked C++ driver name even when its executable is a symlink.
        compiler = Path(driver)
        assert sha(compiler) == '02ba98cc5feefe173cfb8c28c98089817737800537dc7189138ed66b07cf56ec'
        inputs.update({str(path): sha(path) for path in (baseline_path, prior_path, cache, cpp, compiler.resolve(), Path(__file__).resolve())})
        env['PKG_CONFIG_PATH'] = str(prefix / 'lib/pkgconfig') + os.pathsep + env.get('PKG_CONFIG_PATH', '')
        dirs = [str(build / 'lib'), str(prefix / 'lib')]
        dirs += list(dict.fromkeys(str(Path(path).parent) for path in prior['libraries'] if path.startswith('/opt/ros/')))
        dirs += env.get('LD_LIBRARY_PATH', '').split(os.pathsep) if env.get('LD_LIBRARY_PATH') else []
        env.update(LD_LIBRARY_PATH=os.pathsep.join(dict.fromkeys(dirs)),
                   GZ_PARTITION='gz-activation-' + uuid.uuid4().hex,
                   GZ_IP='127.0.0.1', GZ_LOG_PATH=str(out / 'logs'))
        selected_prefix = subprocess.check_output(['pkg-config', '--variable=prefix', 'gz-sensors'], env=env, text=True).strip()
        assert Path(selected_prefix).resolve() == prefix
        include = Path(subprocess.check_output(['pkg-config', '--variable=includedir', 'gz-sensors'], env=env, text=True).strip()) / 'gz/sensors/Sensor.hh'
        assert sha(include) == inputs[str(source / 'include/gz/sensors/Sensor.hh')]
        inputs[str(include)] = sha(include)
        cflags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', 'gz-sensors'], env=env, text=True))
        link_flags = shlex.split(subprocess.check_output(['pkg-config', '--libs', 'gz-sensors'], env=env, text=True))
        binary = out / 'activation-probe'
        report['inputs_sha256'] = inputs
        report['compiler_version'] = run('compiler', [str(compiler), '--version']).decode()
        common = [str(compiler), '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', *cflags]
        run('dependencies', [*common, '-M', '-MF', str(out / 'headers.d'), '-MT', 'activation-probe', str(cpp)])
        headers = dependencies(out / 'headers.d')
        assert cpp.resolve() in headers
        inputs.update({str(path): sha(path) for path in headers})
        run('compile', [*common, '-MD', '-MF', str(out / 'compiled-headers.d'), '-MT', 'activation-probe', str(cpp), *link_flags, '-o', str(binary)])
        assert dependencies(out / 'compiled-headers.d') == headers
        assert all(sha(path) == value for path, value in inputs.items())
        report['compiler_prerequisites_sha256'] = {str(path): inputs[str(path)] for path in headers}
        linkage = run('ldd', ['ldd', str(binary)]).decode()
        assert 'not found' not in linkage
        libraries = {path: sha(path) for path in linkage.split() if path.startswith('/') and Path(path).is_file()}
        sensor_libs = [path for path in libraries if Path(path).name.startswith('libgz-sensors.so.')]
        assert len(sensor_libs) == 1
        assert libraries[sensor_libs[0]] == sha(build / 'lib' / Path(sensor_libs[0]).name)
        report.update(libraries_sha256=libraries, binary_sha256=sha(binary))
        actual = json.loads(run('runtime', [str(binary)], 30))
        validate_observations(actual)
        original = build / 'bin/UNIT_Sensor_TEST'
        assert sha(original) == prior['original_binary_sha256']
        inputs[str(original)] = sha(original)
        original_linkage = run('original-ldd', ['ldd', str(original)]).decode()
        assert 'not found' not in original_linkage
        original_libraries = {path: sha(path) for path in original_linkage.split() if path.startswith('/') and Path(path).is_file()}
        original_sensor = [path for path in original_libraries if Path(path).name.startswith('libgz-sensors.so.')]
        assert len(original_sensor) == 1 and original_libraries[original_sensor[0]] == libraries[sensor_libs[0]]
        assert all(path not in libraries or libraries[path] == value for path, value in original_libraries.items())
        libraries.update(original_libraries)
        report['original_libraries_sha256'] = original_libraries
        xml_path = out / 'original.xml'
        run('original', [str(original), '--gtest_filter=Sensor_TEST.Sensor', '--gtest_output=xml:' + str(xml_path)])
        validate_original(xml_path, source)
        assert subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip() == pins['gz-sensors']
        for path, expected in {**inputs, **libraries, str(binary): report['binary_sha256']}.items():
            assert sha(path) == expected
        report.update(state='captured_pending_independent_review', stage='complete',
                      observations=actual, original_xml_sha256=sha(xml_path),
                      original_tests=1, original_failures=0,
                      stdout_sha256=sha(out / 'runtime.stdout'), stderr_sha256=sha(out / 'runtime.stderr'))
    except BaseException as error:
        report.update(state='failed', error=repr(error), traceback=traceback.format_exc(),
                      inputs_sha256=inputs, libraries_sha256=libraries)
        raise
    finally:
        (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
