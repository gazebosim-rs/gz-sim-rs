#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Capture pinned native Server IMU rate/reset and global-deadline behavior."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import traceback
import uuid

ROOT = Path(__file__).resolve().parents[1]
SOURCES = {
    'gz-sim': ['src/Server.cc', 'src/ServerPrivate.cc', 'src/TestFixture.cc',
               'src/SimulationRunner.cc', 'src/SystemManager.cc', 'src/ServerConfig.cc',
               'src/systems/imu/Imu.cc', 'src/systems/physics/Physics.cc',
               'include/gz/sim/TestFixture.hh', 'include/gz/sim/Server.hh',
               'include/gz/sim/ServerConfig.hh'],
    'gz-sensors': ['src/Sensor.cc', 'src/Sensor_TEST.cc', 'src/ImuSensor.cc',
                   'include/gz/sensors/Sensor.hh', 'include/gz/sensors/ImuSensor.hh'],
    'gz-physics': ['tpe/plugin/src/SimulationFeatures.cc', 'dartsim/src/SimulationFeatures.cc'],
    'gz-transport': ['src/Node.cc', 'src/NodeShared.cc', 'src/NodeSharedPrivate.hh',
                     'include/gz/transport/Node.hh', 'include/gz/transport/detail/Node.hh'],
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def capture(out, label, command, env, timeout, *, cwd=ROOT, stdin=None):
    def text(value):
        return value.decode(errors='replace') if isinstance(value, bytes) else value or ''
    try:
        result = subprocess.run(command, cwd=cwd, env=env, input=stdin,
                                capture_output=True, text=True, timeout=timeout)
        state = {'state': 'exited', 'returncode': result.returncode}
        stdout, stderr = result.stdout, result.stderr
    except subprocess.TimeoutExpired as error:
        state = {'state': 'timeout', 'timeout': timeout}
        stdout, stderr = text(error.stdout), text(error.stderr)
    except OSError as error:
        state = {'state': 'launch_failed', 'error': str(error)}
        stdout, stderr = '', str(error)
    state['command'] = command
    (out / (label + '.stdout')).write_text(stdout)
    (out / (label + '.stderr')).write_text(stderr)
    (out / (label + '.exit.json')).write_text(json.dumps(state, indent=2) + '\n')
    if state.get('returncode') != 0:
        raise RuntimeError(f'{label}: {state}')
    return stdout


def validate(row, kind):
    assert isinstance(row['resets'], int) and (row['resets'] >= 1 if kind == 'reset' else row['resets'] == 0)
    assert 0 < len(row['samples']) <= 256
    for sample in row['samples']:
        assert sample['phase'] in range(5) and isinstance(sample['time_ns'], int) and sample['time_ns'] >= 0
        assert bytes.fromhex(sample['wire_hex'])
        if kind == 'future':
            assert sample['channel'] in ['target', 'peer']
    labels = (['bootstrap_end', 'initial_start', 'initial_end', 'rate200_start', 'rate200_end',
               'reset_start', 'reset_end', 'rate500_start', 'rate500_end'] if kind == 'reset' else
              ['bootstrap_end', 'initial_start', 'initial_end', 'changed_start', 'changed_end',
               'before_deadline_start', 'before_deadline_end', 'deadline_start', 'deadline_end'])
    assert [v['label'] for v in row['boundaries']] == labels
    assert {v['phase'] for v in row['ticks']} >= {1, 2, 3, 4}
    for tick in row['ticks']:
        assert isinstance(tick['time_ns'], int) and tick['time_ns'] >= 0
        assert isinstance(tick['iteration'], int) and isinstance(tick['paused'], bool)
    samples = ticks = 0
    for boundary in row['boundaries']:
        assert samples <= boundary['samples'] <= len(row['samples'])
        assert ticks <= boundary['ticks'] <= len(row['ticks'])
        assert all(v['phase'] == boundary['phase'] for v in row['samples'][samples:boundary['samples']])
        assert all(v['phase'] == boundary['phase'] for v in row['ticks'][ticks:boundary['ticks']])
        samples, ticks = boundary['samples'], boundary['ticks']
    assert (samples, ticks) == (len(row['samples']), len(row['ticks']))


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kind', choices=['reset', 'future'], required=True)
    parser.add_argument('--engine', choices=['tpe', 'dartsim'], default='tpe')
    for name in ['upstream-root', 'sim-build', 'dependency-prefix', 'work-dir']:
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--headless-reference', type=Path, default=ROOT / 'planning/reference/headless-imu.json')
    parser.add_argument('--schedule-reference', type=Path, default=ROOT / 'planning/reference/sensor-schedule.json')
    return parser.parse_args()


def library_hash(libraries, role):
    matches = [digest for path, digest in libraries.items() if Path(path).name.startswith(role)]
    if len(matches) != 1:
        raise ValueError(f'expected one canonical library for {role}, found {len(matches)}')
    return matches[0]


def run(args, out, state):
    build, prefix = args.sim_build.resolve(), args.dependency_prefix.resolve()
    baseline = ROOT / 'planning/baseline.json'
    lock = {row['name']: row['commit'] for row in json.loads(baseline.read_text())['repositories']}
    cpp = ROOT / f'crates/gz-sim-scene/tests/native_imu_rate_{args.kind}.cc'
    fixture = ROOT / 'crates/gz-sim-scene/tests/fixtures' / (
        'headless_imu.sdf' if args.kind == 'reset' else 'runtime_imu_future_gate.sdf')
    inputs = state['inputs_sha256']
    for path in [baseline, cpp, fixture, Path(__file__), args.headless_reference, args.schedule_reference]:
        inputs[str(path)] = sha(path)
    state['stage'] = 'source_inputs'
    sources = {name: args.upstream_root.resolve() / name for name in SOURCES}
    for name, files in SOURCES.items():
        directory = sources[name]
        assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=directory, text=True, stderr=subprocess.PIPE).strip() == lock[name]
        for relative in files:
            path = directory / relative
            assert path.read_bytes() == subprocess.check_output(['git', 'show', lock[name] + ':' + relative], cwd=directory, stderr=subprocess.PIPE)
            inputs[str(path)] = sha(path)
    headless, schedule = [json.loads(path.read_text()) for path in [args.headless_reference, args.schedule_reference]]
    assert all(lock[name] == revision for name, revision in headless['revisions'].items())
    assert schedule['revision'] == lock['gz-sensors']
    for name, entries in headless['sources'].items():
        for relative, digest in entries.items():
            assert inputs[str(sources[name] / relative)] == digest
    for relative, digest in schedule['native_sources'].items():
        assert inputs[str(sources['gz-sensors'] / relative)] == digest
    state['stage'] = 'build_inputs'
    target = build / 'src/CMakeFiles/UNIT_ServerConfig_TEST.dir'
    flags, link, cache = target / 'flags.make', target / 'link.txt', build / 'CMakeCache.txt'
    for path in [flags, link, cache]:
        inputs[str(path)] = sha(path)
    home = next(v.split('=', 1)[1] for v in cache.read_text().splitlines() if v.startswith('CMAKE_HOME_DIRECTORY:'))
    assert Path(home).resolve() == sources['gz-sim']
    includes = shlex.split(next(v.split('=', 1)[1] for v in flags.read_text().splitlines() if v.startswith('CXX_INCLUDES =')))
    words = shlex.split(link.read_text())
    compiler = Path(shutil.which(words[0]))  # Keep clang++'s C++ driver basename.
    inputs[str(compiler)] = sha(compiler)
    libraries = [str((build / 'src' / v).resolve()) if v.startswith('../') else v
                 for v in words[words.index('-o') + 2:] if 'gtest' not in v]
    include_dirs = []
    for index, value in enumerate(includes):
        if value.startswith('-I'):
            include_dirs.append(Path(value[2:]))
        elif value == '-isystem':
            include_dirs.append(Path(includes[index + 1]))
    state['stage'] = 'headers'
    for repo, names in [('gz-sim', ['TestFixture.hh', 'Server.hh', 'ServerConfig.hh']),
                        ('gz-transport', ['Node.hh', 'detail/Node.hh'])]:
        for name in names:
            relative = Path('gz') / repo.removeprefix('gz-') / name
            actual = next(directory / relative for directory in include_dirs if (directory / relative).is_file())
            expected = sources[repo] / 'include' / relative
            assert actual.read_bytes() == expected.read_bytes()
            if repo == 'gz-sim':
                assert actual.resolve() == expected.resolve()
            else:
                assert actual.is_relative_to(prefix)
            inputs[str(actual)] = sha(actual)
    # Dependency lineage only; the Server probe does not include Sensor headers.
    for name in ['Sensor.hh', 'ImuSensor.hh']:
        source = sources['gz-sensors'] / 'include/gz/sensors' / name
        installed = prefix / 'include/gz/sensors11/gz/sensors' / name
        assert source.read_bytes() == installed.read_bytes()
        inputs[str(installed)] = sha(installed)
    plugins = [build / f'lib/libgz-sim-{name}-system.so' for name in ['physics', 'imu']]
    plugins.append(prefix / f'lib/gz-physics/engine-plugins/libgz-physics-{args.engine}-plugin.so')
    for path in plugins:
        inputs[str(path.resolve())] = sha(path)
    binary = out / 'native-imu-rate'
    command = [str(compiler), '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', *includes, str(cpp), '-o', str(binary), *libraries]
    env = dict(os.environ, GZ_SIM_SERVER_CONFIG_PATH='', GZ_IP='127.0.0.1',
               GZ_SIM_SYSTEM_PLUGIN_PATH=str(build / 'lib'),
               GZ_SIM_PHYSICS_ENGINE_PATH=str(prefix / 'lib/gz-physics/engine-plugins'), GZ_LOG_PATH=str(out / 'logs'))
    env['LD_LIBRARY_PATH'] = os.pathsep.join(p for p in [str(build / 'lib'), str(prefix / 'lib'), env.get('LD_LIBRARY_PATH', '')] if p)
    (out / 'inputs.json').write_text(json.dumps({'sha256': inputs, 'command': command,
        'compiler_version': subprocess.check_output([str(compiler), '--version'], text=True, stderr=subprocess.PIPE)}, indent=2) + '\n')
    state['stage'] = 'compile'
    capture(out, 'compile', command, env, 90, cwd=build / 'src')
    binary_hash = sha(binary)
    state['stage'] = 'startup_linkage'
    linkage = subprocess.check_output(['ldd', str(binary)], env=env, text=True, stderr=subprocess.PIPE)
    (out / 'ldd.txt').write_text(linkage)
    assert 'not found' not in linkage
    loaded = {p: sha(p) for p in linkage.split() if p.startswith('/') and Path(p).is_file()}
    startup = loaded.copy()
    observations = []
    modes = ['all', 'time-only'] if args.kind == 'reset' else ['zero', 'nan', 'zero-peer', 'nan-peer']
    for mode in modes:
        state['active_mode'] = mode
        state['stage'] = 'native:' + mode
        env['GZ_PARTITION'] = 'imu-rate-' + uuid.uuid4().hex
        maps = out / (mode + '.maps')
        stdout = capture(out, mode, [str(binary), mode, args.engine, str(maps)], env, 60, stdin=fixture.read_text())
        state['stage'] = 'validate:' + mode
        row = json.loads(stdout)
        assert row['mode'] == mode and row['engine'] == args.engine
        validate(row, args.kind)
        paths = {v.split(maxsplit=5)[5] for v in maps.read_text().splitlines()
                 if len(v.split(maxsplit=5)) == 6 and v.split(maxsplit=5)[5].startswith('/')}
        actual = {p: sha(p) for p in paths if Path(p).is_file()}
        for plugin in plugins:
            assert actual[str(plugin.resolve())] == inputs[str(plugin.resolve())]
        known = headless['loaded_libraries'][args.engine]
        roles = ['libgz-sim.so.', 'libgz-sim-physics-system.so', 'libgz-sim-imu-system.so',
                 'libgz-sensors.so.', 'libgz-transport.so.', f'libgz-physics-{args.engine}-plugin.so']
        for role in roles:
            current = [p for p in actual if Path(p).name.startswith(role)]
            previous = [p for p in known if Path(p).name.startswith(role)]
            assert len(current) == len(previous) == 1 and actual[current[0]] == known[previous[0]]
        sensor_hash = library_hash(schedule['libraries'], 'libgz-sensors.so.11')
        assert next(actual[p] for p in actual if Path(p).name.startswith('libgz-sensors.so.')) == sensor_hash
        for path, digest in actual.items():
            assert path not in loaded or loaded[path] == digest
            loaded[path] = digest
        observations.append({'observation': row, 'loaded_libraries_sha256': actual})
        state['completed_modes'].append(mode)
    state['stage'] = 'final_identity'
    for name, directory in sources.items():
        assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=directory, text=True, stderr=subprocess.PIPE).strip() == lock[name]
    for path, digest in {**inputs, **loaded}.items():
        assert sha(path) == digest
    assert sha(binary) == binary_hash
    raw = {str(path.relative_to(out)): sha(path) for path in out.iterdir() if path.is_file()}
    report = {'state': 'captured_native_imu_rate_pending_independent_review', 'kind': args.kind,
              'engine': args.engine, 'inputs_sha256': inputs, 'binary_sha256': binary_hash,
              'startup_libraries_sha256': startup, 'observations': observations, 'raw_sha256': raw,
              'scope': 'Qualified native Server functional packet/tick capture with FIFO drain boundaries. No Rust parity, clean-image, performance, full sensor or P2 acceptance implied.'}
    (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    state['report_sha256'] = sha(out / 'report.json')
    print(f'Captured native {args.kind}/{args.engine}; independent interpretation required.')


def execute(args, out):
    state = {'state': 'running', 'stage': 'preflight', 'kind': args.kind, 'engine': args.engine,
             'inputs_sha256': {}, 'completed_modes': []}
    try:
        run(args, out, state)
    except BaseException as error:
        state.update(state='failed', error_type=type(error).__name__, error=str(error),
                     traceback=traceback.format_exc())
        for name in ['stdout', 'stderr', 'output']:
            value = getattr(error, name, None)
            if value is not None:
                state['error_' + name] = value.decode(errors='replace') if isinstance(value, bytes) else value
        try:
            state['raw_sha256'] = {path.name: sha(path) for path in out.iterdir() if path.is_file()}
            (out / 'workflow.json').write_text(json.dumps(state, indent=2) + '\n')
        except Exception as retention_error:
            print(f'Could not retain workflow failure: {retention_error}', file=sys.stderr)
        raise
    state.update(state='captured_pending_independent_review', stage='complete')
    (out / 'workflow.json').write_text(json.dumps(state, indent=2) + '\n')


def main():
    args = arguments()
    out = args.work_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    execute(args, out)


if __name__ == '__main__':
    main()
