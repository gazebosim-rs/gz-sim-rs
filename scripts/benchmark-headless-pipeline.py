#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Measure matched TPE/IMU/process-local delivery after validating every sample."""
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import platform
import shlex
import statistics
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def cache_value(build, key):
    prefix = key + ':'
    return next(line.split('=', 1)[1] for line in (build / 'CMakeCache.txt').read_text().splitlines()
                if line.startswith(prefix))


def decode(output, steps, fixture_sha256, *, warmup_force_x=None, streams=False, request_period_us=None):
    rows = [json.loads(line.split('PIPELINE ', 1)[1]) for line in output.splitlines()
            if 'PIPELINE ' in line]
    if len(rows) != 1:
        raise ValueError('expected exactly one validated pipeline result')
    row = rows[0]
    if any(type(row[key]) is not int for key in ['steps', 'nanos', 'logical_bytes', 'time_sum']) or row['steps'] != steps or row['nanos'] <= 0 or row['logical_bytes'] <= 0:
        raise ValueError('invalid measured work or duration')
    if row['time_sum'] != steps * (steps + 3) // 2 * 1_000_000:
        raise ValueError('sample timestamps/checksum differ')
    if not math.isclose(row['force_sum'], 9.81 * steps, rel_tol=1e-12, abs_tol=1e-8):
        raise ValueError('physical checksum differs')
    if not math.isfinite(row['final_x']) or not abs(row['final_x'] - (steps + 1) * 0.001) < 1e-9:
        raise ValueError('physical motion checksum differs')
    if hashlib.sha256(bytes.fromhex(row['fixture_hex'])).hexdigest() != fixture_sha256:
        raise ValueError('application consumed a different SDF fixture')
    if warmup_force_x is not None and (not math.isfinite(row['warmup_force_x']) or abs(row['warmup_force_x'] - warmup_force_x) >= 1e-9):
        raise ValueError('velocity replacement warmup differs from native')
    if streams:
        for key in ['wall_nanos', 'pacing_wait_nanos', 'request_period_us', 'clock_count', 'pose_count', 'clock_time_sum', 'pose_time_sum',
                    'clock_bytes', 'pose_bytes', 'pose_bytes_without_id_values']:
            if type(row[key]) is not int or row[key] <= 0:
                raise ValueError('invalid stream work: ' + key)
        period = row['request_period_us']
        if request_period_us is not None and period != request_period_us:
            raise ValueError('application ignored requested cadence')
        if not 20000 <= period <= 1000000 or row['wall_nanos'] < steps * period * 1000:
            raise ValueError('wrong request cadence or elapsed wall time')
        if row['wall_nanos'] < row['nanos'] + row['pacing_wait_nanos']:
            raise ValueError('latency/pacing intervals exceed elapsed wall time')
        if row['clock_count'] != steps or row['pose_count'] != steps:
            raise ValueError('incomplete clock/pose observations')
        if row['clock_time_sum'] != row['time_sum'] or row['pose_time_sum'] != row['time_sum']:
            raise ValueError('stream timestamps differ')
        ids = row['pose_ids']
        if not isinstance(ids, dict) or set(ids) != {'ball', 'ground', 'body', 'base'}:
            raise ValueError('wrong pose identity roles')
        if any(type(value) is not int or not 0 < value <= 0xffffffff for value in ids.values()) or len(set(ids.values())) != 4:
            raise ValueError('invalid pose identities')
        id_bytes = steps * sum((value.bit_length() + 6) // 7 for value in ids.values())
        if row['pose_bytes'] - row['pose_bytes_without_id_values'] != id_bytes:
            raise ValueError('pose normalization removed more than opaque ID values')
    return row


def loaded_libraries(maps):
    paths = {line.split(maxsplit=5)[5] for line in maps.read_text().splitlines()
             if len(line.split(maxsplit=5)) == 6 and line.split(maxsplit=5)[5].startswith('/')}
    return {path: digest(Path(path)) for path in sorted(paths)
            if Path(path).is_file() and '.so' in Path(path).name}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['sim-build', 'physics-build', 'sensors-build', 'dependency-prefix',
                 'reference-prefix', 'rust-build-json', 'work-dir', 'output']:
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--steps', type=int, default=1000)
    parser.add_argument('--samples', type=int, default=7)
    parser.add_argument('--cpu', type=int)
    parser.add_argument('--path', choices=['direct', 'server', 'streams'], default='direct')
    parser.add_argument('--request-period-us', type=int, default=20000)
    parser.add_argument('--executor', choices=['default', 'single', 'parallel'], default='default')
    args = parser.parse_args()
    if args.path == 'streams' and not 20000 <= args.request_period_us <= 1000000:
        raise ValueError('native pose/info requires declared request period of20ms or more')
    if not 1 <= args.steps <= 1000 or args.samples < 7:
        raise ValueError('require 1..1000 steps and at least seven measured pairs')
    allowed = os.sched_getaffinity(0)
    cpu = min(allowed) if args.cpu is None else args.cpu
    if cpu not in allowed:
        raise ValueError('CPU is outside this process affinity')
    work = args.work_dir.resolve()
    work.mkdir(parents=True, exist_ok=True)
    builds = {name: getattr(args, name.replace('-', '_') + '_build').resolve()
              for name in ['sim', 'physics', 'sensors']}
    # Reuse the already reviewed raw-source checker; no native source is modified.
    spec = importlib.util.spec_from_file_location('native_reference', ROOT / 'scripts/probe-headless-imu.py')
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    lock = {item['name']: item['commit'] for item in
            json.loads((ROOT / 'planning/baseline.json').read_text())['repositories']}
    source_files = {
        'sim': ['src/Server.cc', 'src/ServerPrivate.cc', 'src/SimulationRunner.cc', 'src/systems/physics/Physics.cc', 'src/systems/imu/Imu.cc'],
        'physics': ['tpe/lib/src/Engine.cc', 'tpe/lib/src/CollisionDetector.cc', 'tpe/plugin/src/SimulationFeatures.cc'],
        'sensors': ['src/Sensor.cc', 'src/ImuSensor.cc'],
    }
    if args.path == 'streams':
        source_files['sim'].append('src/systems/scene_broadcaster/SceneBroadcaster.cc')
    native_sources, build_settings = {}, {}
    for name, build in builds.items():
        if cache_value(build, 'CMAKE_BUILD_TYPE') != 'Release':
            raise ValueError(f'{name} must be a Release build')
        source = Path(cache_value(build, 'CMAKE_HOME_DIRECTORY')).resolve()
        native_sources[name] = helper.checked_sources(source, lock['gz-' + name], source_files[name])
        build_settings[name] = dict(source=str(source), cache_sha256=digest(build / 'CMakeCache.txt'),
                                    release_flags=cache_value(build, 'CMAKE_CXX_FLAGS_RELEASE'))
    prefix = args.reference_prefix.resolve()
    reference = json.loads((prefix.parent / 'environment.json').read_text())
    expected_lock = json.loads((ROOT / 'planning/transport-reference-lock.json').read_text())
    if reference['source_lock'] != expected_lock:
        raise ValueError('transport reference lock differs')
    for name, expected in reference['libraries_sha256'].items():
        if digest(prefix / name) != expected:
            raise ValueError('transport reference library changed: ' + name)
    target = builds['sim'] / 'src/CMakeFiles/UNIT_ServerConfig_TEST.dir'
    flags = (target / 'flags.make').read_text()
    includes = shlex.split(next(line.split('=', 1)[1] for line in flags.splitlines()
                                if line.startswith('CXX_INCLUDES =')))
    link = shlex.split((target / 'link.txt').read_text())
    libraries = [str((builds['sim'] / 'src' / item).resolve()) if item.startswith('../') else item
                 for item in link[link.index('-o') + 2:] if 'gtest' not in item]
    source = ROOT / ('crates/gz-sim-server/tests/native_stream_benchmark.cc' if args.path == 'streams' else 'crates/gz-sim-server/tests/native_lockstep_benchmark.cc' if args.path == 'server' else 'crates/gz-sim-server/tests/native_pipeline_benchmark.cc')
    native = work / 'native-pipeline'
    command = [link[0], '-std=c++17', '-O3', '-DNDEBUG', '-Wall', '-Wextra', '-Werror',
               *includes, str(source), '-o', str(native), *libraries]
    subprocess.run(command, cwd=builds['sim'] / 'src', check=True)
    rust_candidates = []
    for line in args.rust_build_json.read_text().splitlines():
        try:
            item = json.loads(line)
        except json.JSONDecodeError:
            continue
        if item.get('reason') == 'compiler-artifact' and item.get('executable'):
            if item['target']['name'] == 'gz_sim_server' and item['profile']['test']:
                if item['profile']['opt_level'] != '3' or item['profile']['debug_assertions']:
                    raise ValueError('Rust test executable is not the release profile')
                rust_candidates.append(Path(item['executable']).resolve())
    if len(set(rust_candidates)) != 1:
        raise ValueError('expected exactly one release server test executable')
    rust = rust_candidates[0]
    fixture = ROOT / 'crates/gz-sim-scene/tests/fixtures/headless_imu.sdf'
    libdirs = [builds['physics'] / 'lib', builds['sensors'] / 'lib', builds['sim'] / 'lib',
               prefix / 'lib', args.dependency_prefix.resolve() / 'lib']
    base_env = os.environ.copy()
    for key in ['GZ_TRANSPORT_USERNAME', 'GZ_TRANSPORT_PASSWORD', 'GZ_RELAY']:
        base_env.pop(key, None)
    base_env.update(LD_LIBRARY_PATH=os.pathsep.join(map(str, libdirs)),
                    GZ_IP='127.0.0.1', GZ_SIM_SERVER_CONFIG_PATH='',
                    GZ_SIM_SYSTEM_PLUGIN_PATH=str(builds['sim'] / 'lib'),
                    GZ_SIM_PHYSICS_ENGINE_PATH=str(builds['physics'] / 'lib'),
                    GZ_TRANSPORT_IMPLEMENTATION='zeromq', GZ_TRANSPORT_TOPIC_STATISTICS='0',
                    GZ_TRANSPORT_RCVHWM='1000', GZ_TRANSPORT_SNDHWM='1000',
                    GZ_DISCOVERY_MSG_PORT='10317', GZ_DISCOVERY_SRV_PORT='10318',
                    GZ_STREAM_PERIOD_US=str(args.request_period_us), GZ_PIPELINE_STEPS=str(args.steps), GZ_PIPELINE_EXECUTOR=args.executor)
    required = {
        'libgz-sim.so.': builds['sim'] / 'lib',
        'libgz-sim-physics-system.so.': builds['sim'] / 'lib',
        'libgz-sim-imu-system.so.': builds['sim'] / 'lib',
        'libgz-physics-tpe-plugin.so.': builds['physics'] / 'lib',
        'libgz-physics-tpelib.so.': builds['physics'] / 'lib',
        'libgz-sensors.so.': builds['sensors'] / 'lib',
        'libgz-sensors-imu.so.': builds['sensors'] / 'lib',
        'libgz-msgs.so.': prefix / 'lib',
        'libgz-transport.so.': prefix / 'lib',
    }
    if args.path == 'streams':
        required['libgz-sim-scene-broadcaster-system.so.'] = builds['sim'] / 'lib'
    observations, native_libs = [], None
    for index in range(args.samples + 1):
        pair = {}
        for kind in (['native', 'rust'] if index % 2 == 0 else ['rust', 'native']):
            maps = work / f'native-{index}.maps'
            run_command = ([str(native), str(fixture), str(maps), str(args.steps)] if kind == 'native' else
                           [str(rust), ('streams_benchmark::tpe_clock_pose_imu_complete_lockstep' if args.path == 'streams' else 'pipeline_benchmark::tpe_imu_full_server_pipeline' if args.path == 'server' else 'pipeline_benchmark::tpe_imu_process_local_pipeline'),
                            '--exact', '--nocapture', '--test-threads=1'])
            env = dict(base_env, GZ_PARTITION=f'pipeline-{os.getpid()}-{index}-{kind}')
            result = subprocess.run(['taskset', '-c', str(cpu), *run_command],
                                    env=env, text=True, capture_output=True,
                                    timeout=max(30, args.steps * args.request_period_us / 1_000_000 * 1.25 + 10) if args.path == 'streams' else 30)
            (work / f'{kind}-{index}.stdout').write_text(result.stdout)
            (work / f'{kind}-{index}.stderr').write_text(result.stderr)
            if result.returncode or result.stderr:
                raise ValueError(f'{kind} sample {index} failed: {result.returncode} {result.stderr}')
            pair[kind] = decode(result.stdout, args.steps, digest(fixture), warmup_force_x=0.0 if args.path != 'direct' else None, streams=args.path == 'streams', request_period_us=args.request_period_us)
            if kind == 'native':
                loaded = loaded_libraries(maps)
                for name, directory in required.items():
                    found = [Path(path) for path in loaded if Path(path).name.startswith(name)]
                    if len(found) != 1 or found[0].resolve() != (directory / found[0].name).resolve():
                        raise ValueError('wrong or duplicate loaded library: ' + name)
                if native_libs is not None and loaded != native_libs:
                    raise ValueError('native library set changed between samples')
                native_libs = loaded
        fields = ['steps', 'logical_bytes', 'time_sum']
        if args.path == 'streams':
            fields += ['request_period_us', 'clock_count', 'pose_count', 'clock_time_sum', 'pose_time_sum', 'pose_bytes_without_id_values']
        for field in fields:
            if pair['native'][field] != pair['rust'][field]:
                raise ValueError('native/Rust work differs: ' + field)
        if not math.isclose(pair['native']['force_sum'], pair['rust']['force_sum'], abs_tol=1e-8):
            raise ValueError('native/Rust physical checksum differs')
        observations.append(pair)
    values = {name: [pair[name]['nanos'] / args.steps for pair in observations[1:]]
              for name in ['native', 'rust']}
    medians = {name: statistics.median(value) for name, value in values.items()}
    model = next((line.split(':', 1)[1].strip() for line in Path('/proc/cpuinfo').read_text().splitlines()
                  if line.startswith('model name')), 'unknown')
    report = dict(
        scope={
            'direct': 'Historical Rust direct publisher versus native full Server; native ratio is contextual only. ',
            'server': 'Actual Rust Server including unobserved pose/clock work versus native Server+Physics+Imu; native ratio is contextual. ',
            'streams': 'Actual Rust Server versus native Server+Physics+Imu+SceneBroadcaster, all three requested clock/pose/IMU streams with prospective20ms-or-greater request pacing and per-step complete receipt; summed work latency excludes declared pacing, walltime recorded separately. ',
        }[args.path] + 'Two-model TPE with one constant1m/s free body, one1000Hz noise-free IMU, sharedSDF, internallyunpaced1ms physics and complete decoded local observations. One received warmup excluded; N measured steps. Not network, joints, rendering, balanced robotics or whole-engine speedup.',
        measurement_boundary=('Sum of per-request servercall+complete-receipt/check latency, declared host pacing excluded and recorded separately' if args.path == 'streams' else 'Elapsed complete observed pipeline interval'),
        pose_byte_contract=('Record raw bytes and opaque stable roleIDs; cross-run size comparison subtracts only ID varint values, guarded by every small pose record encodedlen<128' if args.path == 'streams' else None),
        qualification='Shared host CPU-pinned alternating samples. Native local transport may bypass serialization; Rust currently uses encoded queues. Requested delivered messages/settings match, not internal instructions. Other installed dependencies/build provenance remain qualified; no P2 acceptance inferred.',
        hardware=dict(cpu_model=model, machine=platform.machine(), cpu_affinity=cpu),
        observation_protocol=('Rust receives/checks each requested observation after Server::tick; native calls Server.Run(true,1,false), checks its callbacks and waits for all requested observations. Neither admits another step before complete receipt. Both include complete receipt and checking inside timing.' if args.path != 'direct' else 'Historical direct publisher batch pipeline'),
        queue_settings=dict(rust_capacity=1000, rust_send_hwm=1000, rust_receive_hwm=1000, native_explicit_hwm=1000),
        request_period_us=args.request_period_us if args.path == 'streams' else 0, steps=args.steps, rust_path=args.path, rust_executor=args.executor, warmup_pairs=1, measured_pairs=args.samples, samples=observations,
        ns_per_step=dict(medians=medians, min={k:min(v) for k,v in values.items()},
                         max={k:max(v) for k,v in values.items()}, native_over_rust=medians['native']/medians['rust']),
        builds=build_settings, native_revisions={name:lock['gz-'+name] for name in builds}, native_sources=native_sources, native_command=command,
        compiler=subprocess.check_output([link[0], '--version'], text=True),
        rustc=subprocess.check_output(['rustc', '-vV'], text=True),
        fixture_sha256=digest(fixture), rust_production_sources={str(p.relative_to(ROOT)):digest(p) for p in (ROOT/'crates').rglob('*.rs') if '/src/' in str(p)}, cargo_lock_sha256=digest(ROOT/'Cargo.lock'), git_base=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(), source_sha256={str(p.relative_to(ROOT)):digest(p) for p in
            [source, ROOT/'crates/gz-sim-server/tests/private/pipeline_benchmark.rs',
             *([ROOT/'crates/gz-sim-server/tests/private/streams_benchmark.rs'] if args.path == 'streams' else []),
             ROOT/'crates/gz-sim-server/src/lib.rs', ROOT/'crates/gz-sim-core/src/lib.rs', Path(__file__).resolve()]},
        binary_sha256=dict(native=digest(native), rust=digest(rust)), native_libraries=native_libs)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report['ns_per_step']))


if __name__ == '__main__':
    main()
