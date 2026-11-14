#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Attest the unchanged Zenoh probe against one reviewed libc transition.

The CLI writes an internal result, never a selection or a Root exit receipt.
Without an explicit selection manifest the original bootstrap stays strict.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import signal
import stat
import subprocess
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
ENVIRONMENT = 'GZ_ZENOH_RUNTIME_PROBE_MANIFEST'
META_CAP = 2 * 1024 * 1024
STREAM_CAP = 1024 * 1024
STOPPED_MARKER = 'runtime-probe-stopped.json'
_CLI_INTERRUPTION = None
PROOF_ROLES = (
    'selection.json', 'result.json', 'actual-exit.json', 'config.json',
    'transition.json', 'dependency-diagnostic.json', 'dependency-diagnostic-actual-exit.json',
    'linkage-before.stdout', 'linkage-before.stderr', 'dynamic-before.stdout', 'dynamic-before.stderr',
    'probe.stdout', 'probe.stderr', 'linkage-after.stdout', 'linkage-after.stderr',
    'dynamic-after.stdout', 'dynamic-after.stderr',
)
BASE_SHA = 'b5af8405acf3851764bf6ca598da9156cf9f04421b00e2379fbc137f5389f5dc'
TRANSITION_SHA = '37ec87f58c22d033aa1addff2c3f256635aa34858f400ed2c9f745b2b1dfbd94'
PROBE_SHA = '9e05c560766f2f5060ebf5fe5869052cfe775d54cbf9967f088f3af2d1c6bbd8'
LOADER = '/usr/lib/x86_64-linux-gnu/ld-linux-x86-64.so.2'
LOADER_SHA = '5fc0940888e9d2e1a00e66d6b1a559c36364ee867683652e1074fa576952cc3f'
DELTAS = {
    'libc.so.6': ('/usr/lib/x86_64-linux-gnu/libc.so.6',
                  '3a15d66867d83762c7f2f1e37359cb8f6c5743edb369c65285cb0b1c4f7498bf',
                  '755a371e86cc9f564ab54f66140747700fb0a103f698f659f2060f707897e09a'),
    'libm.so.6': ('/usr/lib/x86_64-linux-gnu/libm.so.6',
                 'fce00b6f25f459cf4ae0b7fae4257909a1a8a86f9149e7c029a5d617baf1ccd0',
                 '5dcc4d25466e29ca390ed6fabb371c69b65dff6fa4e91c34561057a020d6994a'),
    'libresolv.so.2': ('/usr/lib/x86_64-linux-gnu/libresolv.so.2',
                      '1d730ea434da5a5595bc6a961d7a9ace83e03703205df9644044b3dca79c362f',
                      '4dcb01645dbeea5d17f28fd96a730430a808d828b3b7d6a097cfbc311501f69f'),
}
NOTICES = {'copyright': 'Copyright (C) 2026 Open Source Robotics Foundation',
           'SPDX-License-Identifier': 'Apache-2.0'}
spec = importlib.util.spec_from_file_location('zenoh_runtime_bootstrap', ROOT / 'scripts/bootstrap-zenoh-native.py')
bootstrap = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bootstrap)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def check_interruption():
    if _CLI_INTERRUPTION is not None:
        raise RuntimeError('runtime probe interrupted by signal ' + str(_CLI_INTERRUPTION))


def read_file(path, cap=META_CAP):
    """Read bounded ordinary bytes through the same verified descriptor."""
    path = Path(path)
    require(path.is_absolute(), 'artifact path must be absolute')
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        before = os.fstat(descriptor)
        require(stat.S_ISREG(before.st_mode) and before.st_size <= cap, 'ordinary file/cap: ' + str(path))
        chunks, size = [], 0
        while True:
            chunk = os.read(descriptor, min(65536, cap + 1 - size))
            if not chunk:
                break
            chunks.append(chunk)
            size += len(chunk)
            require(size <= cap, 'file grew beyond cap: ' + str(path))
        after = os.fstat(descriptor)
        current = path.lstat()
        identity = lambda value: (value.st_dev, value.st_ino, value.st_mode,
                                  value.st_size, value.st_mtime_ns, value.st_ctime_ns)
        require(identity(before) == identity(after) == identity(current), 'file changed: ' + str(path))
        data = b''.join(chunks)
        require(len(data) == before.st_size, 'short file read')
        return data
    finally:
        os.close(descriptor)


def card(path, cap=META_CAP):
    data = read_file(path, cap)
    return {'path': str(path), 'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data)}


def checked_card(value, cap=META_CAP):
    require(isinstance(value, dict) and set(value) == {'path', 'sha256', 'bytes'}, 'invalid artifact card')
    require(isinstance(value['path'], str) and isinstance(value['sha256'], str)
            and re.fullmatch('[0-9a-f]{64}', value['sha256']) is not None
            and type(value['bytes']) is int and 0 <= value['bytes'] <= cap, 'invalid artifact identity')
    data = read_file(value['path'], cap)
    require(len(data) == value['bytes'] and hashlib.sha256(data).hexdigest() == value['sha256'],
            'artifact identity differs: ' + value['path'])
    return data


def write_json(path, value):
    raw = (json.dumps(value, indent=2) + '\n').encode()
    require(len(raw) <= META_CAP, 'result exceeds metadata cap')
    with Path(path).open('xb') as stream:
        stream.write(raw)
        stream.flush()
        os.fsync(stream.fileno())
    descriptor = os.open(Path(path).parent, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def projection(metadata):
    """Only three approved SHA leaves differ; all old evidence remains old."""
    projected = copy.deepcopy(metadata)
    require(len(projected['probe']['libraries']) == 24, 'expected exact 24 probe dependencies')
    for soname, (path, old, current) in DELTAS.items():
        require(projected['probe']['libraries'][soname] == {'path': path, 'sha256': old},
                'old probe dependency differs: ' + soname)
        projected['probe']['libraries'][soname]['sha256'] = current
    inverse = copy.deepcopy(projected)
    for soname, (_, old, _) in DELTAS.items():
        inverse['probe']['libraries'][soname]['sha256'] = old
    require(inverse == metadata, 'probe projection changed other evidence')
    return projected


def transition_proofs(value):
    raw = checked_card(value)
    require(value['sha256'] == TRANSITION_SHA, 'unreviewed dependency transition')
    metadata = json.loads(raw)
    update = metadata['environment_update_M58']
    expected = {(path, old, current) for path, old, current in DELTAS.values()}
    expected.add((LOADER, 'c20a2dc8917c755f02b94049356320fe1f62ac7d9f8994731f807d9df39302da', LOADER_SHA))
    actual = {(row['path'], row['previous_sha256'], row['current_sha256']) for row in update['changed_files']}
    require(actual == expected and len(update['changed_files']) == 4, 'transition must contain exactly four deltas')
    diagnostic = update['retained_complete_inventory']
    diagnostic_data = json.loads(checked_card(diagnostic))
    receipt = update['retained_inventory_actual_Root_receipt']
    receipt_data = json.loads(checked_card(receipt))
    require(diagnostic_data['internal_exit_code'] == 0 and diagnostic_data['errors'] == []
            and diagnostic_data['observed_count'] == 351 and diagnostic_data['matching_count'] == 347
            and diagnostic_data['delta_count'] == 4 and diagnostic_data['child_present_after_cleanup'] is False,
            'dependency diagnostic was not complete')
    require(receipt_data['actual_root_exit_code'] == 0
            and receipt_data['tool_exit_basis']['actual_exit_code'] == 0
            and receipt_data['report'] == diagnostic and receipt_data['all_children_reaped'] is True,
            'dependency diagnostic lacks genuine Root completion')
    return {'transition.json': value, 'dependency-diagnostic.json': diagnostic,
            'dependency-diagnostic-actual-exit.json': receipt}


def guard_reference(reference, base, transition):
    """Use the unchanged bootstrap for every existing SDK/current-body guard."""
    transition_proofs(transition)
    require(card(reference / 'environment.json') == base and base['sha256'] == BASE_SHA,
            'selected original reference differs')
    metadata = json.loads(checked_card(base))
    projected = projection(metadata)
    require(bootstrap.validate_reference(reference, metadata=projected) == projected,
            'current reference validation differs')
    require(card(reference / 'probe/probe', 16 * META_CAP)['sha256'] == PROBE_SHA, 'original probe differs')
    require(card(Path(LOADER))['sha256'] == LOADER_SHA, 'current loader differs')
    return metadata


def validate_stage(row, metadata, reference):
    require(set(row) == {'role', 'argv', 'actual_exit_code', 'reaped', 'error',
                         'elapsed_seconds', 'stdout', 'stderr'}, 'runtime stage schema differs')
    require(type(row['actual_exit_code']) is int and row['actual_exit_code'] == 0
            and row['reaped'] is True and row['error'] is None
            and type(row['elapsed_seconds']) in (int, float)
            and 0 <= row['elapsed_seconds'] <= 15, 'runtime stage failed')
    stdout, stderr = checked_card(row['stdout'], STREAM_CAP), checked_card(row['stderr'], STREAM_CAP)
    require(stderr == b'', 'runtime stage stderr is not empty')
    if row['role'].startswith('linkage-'):
        require(row['argv'] == ['/usr/bin/ldd', str(reference / 'probe/probe')], 'linkage argv differs')
        require(bootstrap.validate_linkage(stdout.decode().strip(), reference / 'install',
                reference / 'vendor' / metadata['lock']['vendor']['prefix'])
                == projection(metadata)['probe']['libraries'], 'runtime dependency evidence differs')
    elif row['role'].startswith('dynamic-'):
        require(row['argv'] == ['/usr/bin/readelf', '-d', str(reference / 'install/lib/libgz-transport.so.16')],
                'dynamic argv differs')
        require(hashlib.sha256(stdout.decode().strip().encode()).hexdigest()
                == metadata['probe']['transport_dynamic_sha256'], 'runtime transport dynamic differs')
    else:
        require(row['role'] == 'probe' and row['argv'] == [str(reference / 'probe/probe')]
                and stdout == b'zenoh client ready\n', 'actual runtime probe did not return the required result')


def validate_result(result, reference):
    require(set(result) == set(NOTICES) | {'schema_version', 'state', 'reference_root',
            'base_environment', 'helper_source', 'transition', 'probe_binary', 'probe_source',
            'loader', 'config', 'partition', 'stages', 'resource_minima', 'errors',
            'probe_environment', 'all_children_reaped', 'internal_exit_code'}, 'runtime result schema differs')
    require(all(result.get(k) == v for k, v in NOTICES.items()), 'runtime result notices differ')
    require(type(result['schema_version']) is int and result['schema_version'] == 1
            and result['state'] == 'ZENOH_RUNTIME_PROBE_COMPLETED_NOT_SELECTED',
            'unknown runtime result')
    require(result['reference_root'] == str(reference) and type(result['internal_exit_code']) is int
            and result['internal_exit_code'] == 0
            and result['errors'] == [] and result['all_children_reaped'] is True, 'runtime probe did not pass')
    require(isinstance(result['helper_source']['path'], str)
            and Path(result['helper_source']['path']).is_absolute()
            and result['helper_source']['sha256'] == card(Path(__file__).absolute())['sha256']
            and result['helper_source']['bytes'] == card(Path(__file__).absolute())['bytes'],
            'runtime helper Source differs')
    require(result['base_environment']['sha256'] == BASE_SHA, 'wrong runtime base')
    metadata = guard_reference(reference, result['base_environment'], result['transition'])
    require(card(reference / 'probe/probe', 16 * META_CAP) == result['probe_binary']
            and card(reference / 'probe/probe.cc') == result['probe_source'], 'runtime probe identity differs')
    require(result['probe_binary']['sha256'] == PROBE_SHA and result['loader']['sha256'] == LOADER_SHA
            and card(Path(LOADER)) == result['loader'], 'runtime binary/loader identity differs')
    config = json.loads(checked_card(result['config']))
    require(config == loopback_config(), 'runtime probe config differs')
    require(re.fullmatch('runtime-probe-[0-9a-f]{32}', result['partition']) is not None, 'invalid probe partition')
    require(result['probe_environment'] == {
        'GZ_TRANSPORT_IMPLEMENTATION': 'zenoh', 'GZ_PARTITION': result['partition'],
        'ZENOH_CONFIG': result['config']['path'],
        'LD_LIBRARY_PATH': str(reference / 'install/lib') + os.pathsep
            + str(reference / 'vendor' / metadata['lock']['vendor']['prefix'] / 'lib')},
        'recorded probe environment differs')
    minima = result['resource_minima']
    packet = Path(result['config']['path']).parent
    require(not os.path.lexists(packet / STOPPED_MARKER), 'runtime probe has a stopped marker')
    for key, floor in ((str(packet), 1024 ** 3), ('/dev/shm', 1024 ** 3),
                       ('/run/user/1000', 1024 ** 3 + 2 * 1024 ** 2), ('MemAvailable', 8 * 1024 ** 3)):
        require(type(minima.get(key)) is int and minima[key] >= floor, 'runtime resource evidence differs')
    stages = result['stages']
    require([row['role'] for row in stages] == ['linkage-before', 'dynamic-before', 'probe',
                                              'linkage-after', 'dynamic-after'], 'runtime stage order differs')
    for row in stages:
        validate_stage(row, metadata, reference)
    return metadata


def validate_selection(reference, manifest=None):
    reference = Path(reference).absolute()
    if manifest is None and ENVIRONMENT in os.environ:
        require(bool(os.environ[ENVIRONMENT]), 'explicit runtime probe manifest is empty')
        manifest = Path(os.environ[ENVIRONMENT])
    helper = card(Path(__file__).absolute())
    if manifest is None:
        metadata = bootstrap.validate_reference(reference)
        return {'metadata': metadata, 'base_environment': card(reference / 'environment.json'),
                'runtime_probe': None, 'helper_source': helper}
    selected_card = card(Path(manifest))
    selected = json.loads(checked_card(selected_card))
    require(set(selected) == set(NOTICES) | {'schema_version', 'state', 'reference_root',
            'base_environment', 'result', 'actual_exit'}, 'selection schema differs')
    require(all(selected.get(k) == v for k, v in NOTICES.items())
            and type(selected['schema_version']) is int and selected['schema_version'] == 1
            and selected['state'] == 'ZENOH_RUNTIME_PROBE_SELECTED'
            and selected['reference_root'] == str(reference), 'unreviewed runtime selection')
    result = json.loads(checked_card(selected['result']))
    require(not os.path.lexists(Path(selected['result']['path']).parent / STOPPED_MARKER),
            'runtime probe has a stopped marker')
    receipt = json.loads(checked_card(selected['actual_exit']))
    require(receipt['state'] == 'ZENOH_RUNTIME_PROBE_ROOT_ACTUAL0'
            and type(receipt['actual_root_exit_code']) is int and receipt['actual_root_exit_code'] == 0
            and receipt['reference_root'] == str(reference)
            and receipt['base_environment'] == selected['base_environment'] == result['base_environment']
            and receipt['method_path'] == result['helper_source']['path']
            and receipt['method_sha256'] == result['helper_source']['sha256'] == helper['sha256']
            and receipt['result_path'] == selected['result']['path']
            and receipt['result_sha256'] == selected['result']['sha256']
            and receipt['result_bytes'] == selected['result']['bytes'], 'runtime Root receipt binding differs')
    basis = receipt['tool_exit_basis']
    require((type(basis['session']) is int and basis['session'] > 0 or basis['session'] is None)
            and isinstance(basis['completion_chunk'], str) and bool(basis['completion_chunk'])
            and type(basis['actual_exit_code']) is int and basis['actual_exit_code'] == 0,
            'runtime external Root completion is missing')
    metadata = validate_result(result, reference)
    require(card(Path(manifest)) == selected_card and card(Path(__file__).absolute()) == helper,
            'selection changed during validation')
    return {'metadata': metadata, 'base_environment': selected['base_environment'],
            'runtime_probe': selected_card, 'helper_source': helper}


def proof_files(selection):
    """One fixed retained bundle per run; clients bind its selection SHA only."""
    if selection['runtime_probe'] is None:
        return {}
    selected = json.loads(checked_card(selection['runtime_probe']))
    result = json.loads(checked_card(selected['result']))
    require(not os.path.lexists(Path(selected['result']['path']).parent / STOPPED_MARKER),
            'runtime probe has a stopped marker')
    require(selected['base_environment'] == selection['base_environment']
            and result['helper_source']['sha256'] == selection['helper_source']['sha256']
            and result['helper_source']['bytes'] == selection['helper_source']['bytes'],
            'retained proof selection differs')
    files = {'selection.json': selection['runtime_probe'], 'result.json': selected['result'],
             'actual-exit.json': selected['actual_exit'], 'config.json': result['config']}
    files.update(transition_proofs(result['transition']))
    for stage in result['stages']:
        for stream in ('stdout', 'stderr'):
            files[stage['role'] + '.' + stream] = stage[stream]
    require(set(files) == set(PROOF_ROLES), 'runtime proof roles differ')
    for value in files.values():
        checked_card(value)
    return files


def loopback_config():
    return {'mode': 'peer', 'listen': {'endpoints': ['tcp/127.0.0.1:0']},
            'connect': {'endpoints': []}, 'scouting': {'multicast': {'enabled': False},
            'gossip': {'enabled': False}}, 'transport': {'shared_memory': {'enabled': False}}}


def floors(path, minima=None):
    requirements = {path: 1024 ** 3, Path('/dev/shm'): 1024 ** 3,
                    Path('/run/user/1000'): 1024 ** 3 + 2 * 1024 ** 2}
    values = {str(p): shutil.disk_usage(p).free for p in requirements}
    values['MemAvailable'] = int(next(line.split()[1] for line in Path('/proc/meminfo').read_text().splitlines()
                                     if line.startswith('MemAvailable:'))) * 1024
    if minima is not None:
        for key, value in values.items():
            minima[key] = min(minima.get(key, value), value)
    require(all(values[str(p)] >= minimum for p, minimum in requirements.items())
            and values['MemAvailable'] >= 8 * 1024 ** 3, 'runtime probe resource floor')
    return values


def run_stage(role, argv, env, directory, stages, minima):
    row = {'role': role, 'argv': list(map(str, argv)), 'actual_exit_code': None,
           'reaped': False, 'error': None, 'elapsed_seconds': None, 'stdout': None, 'stderr': None}
    stages.append(row)
    start, child = time.monotonic(), None
    paths = {stream: directory / (role + '.' + stream) for stream in ('stdout', 'stderr')}
    try:
        check_interruption()
        with paths['stdout'].open('xb') as stdout, paths['stderr'].open('xb') as stderr:
            child = subprocess.Popen(argv, env=env, stdin=subprocess.DEVNULL,
                                     stdout=stdout, stderr=stderr, start_new_session=True)
            # The handler only records a flag: ownership is established first,
            # including when a signal arrived inside the Popen constructor.
            check_interruption()
            while child.poll() is None:
                check_interruption()
                require(time.monotonic() - start < 15, 'stage deadline')
                require(all(path.stat().st_size <= STREAM_CAP for path in paths.values()), 'stage stream cap')
                floors(directory, minima)
                time.sleep(0.02)
            row['actual_exit_code'] = child.wait()
            row['reaped'] = True
            check_interruption()
            # A fast exit can bypass the polling loop entirely.
            floors(directory, minima)
        require(all(path.stat().st_size <= STREAM_CAP for path in paths.values()), 'stage post-exit stream cap')
        require(time.monotonic() - start <= 15 and row['actual_exit_code'] == 0, 'stage failed or exceeded deadline')
    except BaseException as error:
        row['error'] = type(error).__name__ + ': ' + str(error)[:1024]
        raise
    finally:
        if child is not None and not row['reaped']:
            try:
                if child.poll() is None:
                    try:
                        os.killpg(child.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    try:
                        status = child.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        try:
                            os.killpg(child.pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                        status = child.wait(timeout=2)
                else:
                    status = child.wait(timeout=2)
                row['actual_exit_code'] = status
                row['reaped'] = True
            except BaseException as error:
                row['error'] = (row['error'] or '') + '; cleanup: ' + str(error)[:512]
        row['elapsed_seconds'] = time.monotonic() - start
        for stream, path in paths.items():
            if path.exists():
                try:
                    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
                    try:
                        os.fsync(descriptor)
                    finally:
                        os.close(descriptor)
                    row[stream] = card(path, 16 * META_CAP)
                except BaseException as error:
                    row[stream] = {'path': str(path), 'bytes': path.stat().st_size,
                                   'sha256': None, 'error': str(error)[:512]}


def attest(reference, directory, transition):
    reference, directory, transition = Path(reference).absolute(), Path(directory).absolute(), Path(transition).absolute()
    require(reference.resolve(strict=True) == reference and reference.is_dir()
            and directory.resolve(strict=False) == directory
            and directory.parent.resolve(strict=True) == directory.parent
            and not directory.is_relative_to(reference) and not reference.is_relative_to(directory),
            'runtime paths must be canonical and physically separate from the preserved reference')
    require(not directory.exists() and not directory.is_symlink(), 'runtime packet must be fresh')
    directory.mkdir()
    result = {**NOTICES, 'schema_version': 1, 'state': 'ZENOH_RUNTIME_PROBE_COMPLETED_NOT_SELECTED',
              'reference_root': str(reference), 'base_environment': None, 'helper_source': None,
              'transition': None, 'probe_binary': None, 'probe_source': None, 'loader': None,
              'config': None, 'partition': None, 'probe_environment': None, 'stages': [], 'resource_minima': {},
              'errors': [], 'all_children_reaped': False, 'internal_exit_code': 1}
    try:
        check_interruption()
        floors(directory, result['resource_minima'])
        result['base_environment'] = card(reference / 'environment.json')
        result['helper_source'] = card(Path(__file__).absolute())
        result['transition'] = card(transition)
        metadata = guard_reference(reference, result['base_environment'], result['transition'])
        check_interruption()
        result['probe_binary'] = card(reference / 'probe/probe', 16 * META_CAP)
        result['probe_source'] = card(reference / 'probe/probe.cc')
        result['loader'] = card(Path(LOADER))
        config_path = directory / 'zenoh.json'
        write_json(config_path, loopback_config())
        result['config'] = card(config_path)
        result['partition'] = 'runtime-probe-' + uuid.uuid4().hex
        env = bootstrap.build_environment(reference / 'install', reference / 'vendor' / metadata['lock']['vendor']['prefix'])
        probe_env = bootstrap.probe_environment(env, config_path)
        probe_env['GZ_PARTITION'] = result['partition']
        result['probe_environment'] = {key: probe_env[key] for key in
            ('GZ_TRANSPORT_IMPLEMENTATION', 'GZ_PARTITION', 'ZENOH_CONFIG', 'LD_LIBRARY_PATH')}
        for phase in ('before', 'after'):
            run_stage('linkage-' + phase, ['/usr/bin/ldd', str(reference / 'probe/probe')],
                      env, directory, result['stages'], result['resource_minima'])
            validate_stage(result['stages'][-1], metadata, reference)
            run_stage('dynamic-' + phase, ['/usr/bin/readelf', '-d', str(reference / 'install/lib/libgz-transport.so.16')],
                      env, directory, result['stages'], result['resource_minima'])
            validate_stage(result['stages'][-1], metadata, reference)
            if phase == 'before':
                run_stage('probe', [str(reference / 'probe/probe')], probe_env,
                          directory, result['stages'], result['resource_minima'])
                validate_stage(result['stages'][-1], metadata, reference)
        result['all_children_reaped'] = all(row['reaped'] for row in result['stages'])
        result['internal_exit_code'] = 0
        validate_result(result, reference)
        require(card(Path(__file__).absolute()) == result['helper_source'], 'runtime helper changed')
    except BaseException as error:
        result['internal_exit_code'] = 1
        result['errors'].append(type(error).__name__ + ': ' + str(error)[:2048])
    finally:
        result['all_children_reaped'] = all(row['reaped'] for row in result['stages'])
        if result['base_environment'] is not None and result['transition'] is not None:
            try:
                guard_reference(reference, result['base_environment'], result['transition'])
                require(card(Path(__file__).absolute()) == result['helper_source'], 'runtime helper changed')
                check_interruption()
            except BaseException as error:
                result['internal_exit_code'] = 1
                result['errors'].append('final guard: ' + type(error).__name__ + ': ' + str(error)[:2048])
        try:
            floors(directory, result['resource_minima'])
            write_json(directory / 'result.json', result)
            check_interruption()
            # No recursive result rewriting: a late failure is a veto marker.
            floors(directory, result['resource_minima'])
            check_interruption()
        except BaseException as error:
            result['internal_exit_code'] = 1
            result['errors'].append('finalization: ' + type(error).__name__ + ': ' + str(error)[:2048])
            stopped = {**NOTICES, 'state': 'ZENOH_RUNTIME_PROBE_FINALIZATION_STOPPED',
                       'internal_exit_code': 1, 'errors': result['errors'],
                       'resource_minima': result['resource_minima']}
            try:
                write_json(directory / STOPPED_MARKER, stopped)
            except BaseException as marker_error:
                # Root still sees a genuine nonzero even if persistence failed.
                message = ('runtime probe finalization failed: ' + str(error)[:512]
                           + '; stopped marker failed: ' + str(marker_error)[:512])
                os.write(2, (message.encode('ascii', 'backslashreplace')[:4095] + b'\n'))
    return result['internal_exit_code']


def main():
    global _CLI_INTERRUPTION
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--transition', type=Path, required=True)
    args = parser.parse_args()
    previous_flag = _CLI_INTERRUPTION
    _CLI_INTERRUPTION = None
    previous = {number: signal.getsignal(number) for number in
                (signal.SIGTERM, signal.SIGHUP, signal.SIGINT)}

    def interrupted(number, _frame):
        global _CLI_INTERRUPTION
        if _CLI_INTERRUPTION is None:
            _CLI_INTERRUPTION = number

    status = 1
    try:
        for number in previous:
            signal.signal(number, interrupted)
        status = attest(args.reference, args.output, args.transition)
    finally:
        try:
            for number, handler in previous.items():
                signal.signal(number, handler)
            # Read the last flag only after no CLI handler can set it again.
            if _CLI_INTERRUPTION is not None:
                completed_success = type(status) is int and status == 0
                status = 1
                marker = args.output / STOPPED_MARKER
                if completed_success and not os.path.lexists(marker):
                    try:
                        write_json(marker, {**NOTICES, 'state': 'ZENOH_RUNTIME_PROBE_FINALIZATION_STOPPED',
                            'internal_exit_code': 1, 'errors': ['runtime probe interrupted by signal '
                                + str(_CLI_INTERRUPTION)], 'resource_minima': {}})
                    except BaseException:
                        os.write(2, b'Runtime probe interrupted; no selection is eligible.\n')
        finally:
            _CLI_INTERRUPTION = previous_flag
    return status


if __name__ == '__main__':
    raise SystemExit(main())
