#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""One configured physics-only B03 batch; never a full Server speedup."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import statistics
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
XML = ROOT / 'crates/gz-sim-scene/tests/fixtures/b03_double_pendulum.sdf'
XML_SHA = '24e85ff944b5efe2d758fdda543e392f34d59f26297a0a87fafbb41d5a3b16aa'
CONTRACT = 'b03-physics-unlimited-pair-v1'
FIELDS = ['px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw', 'origin_vx', 'origin_vy',
          'origin_vz', 'com_vx', 'com_vy', 'com_vz', 'wx', 'wy', 'wz', 'com_px',
          'com_py', 'com_pz', 'local_com_x', 'local_com_y', 'local_com_z']
ROLES = {'native': 'native-public-physics', 'rust': 'rust-scene-physics'}


def require(ok, message):
    if not ok:
        raise RuntimeError(message)


def parse(raw):
    def pairs(items):
        value = {}
        for key, item in items:
            require(key not in value, f'duplicate JSON key: {key}')
            value[key] = item
        return value
    def invalid(value):
        raise RuntimeError(f'nonfinite JSON literal: {value}')
    return json.loads(raw, object_pairs_hook=pairs, parse_constant=invalid)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def elf_file(path):
    require(path.is_file(), f'mapped/build file is unavailable: {path}')
    with path.open('rb') as stream:
        require(stream.read(4) == b'\x7fELF', f'unverified non-ELF mapping: {path}')


def known_role_closure(path, role, executable):
    """Read the explicit known selection for this one measured role/build."""
    value = parse(path.read_bytes())
    require(isinstance(value, dict) and set(value) == {'role', 'executable_sha256', 'files'},
            'role closure schema differs')
    require(value['role'] == role, 'role closure belongs to another workload')
    hash_pattern = r'[0-9a-f]{64}'
    require(isinstance(value['executable_sha256'], str)
            and re.fullmatch(hash_pattern, value['executable_sha256']), 'invalid executable hash')
    elf_file(executable)
    require(value['executable_sha256'] == digest(executable), 'role closure belongs to another executable')
    files = value['files']
    require(isinstance(files, dict) and files, 'missing known role DSO closure')
    for name, expected in files.items():
        require(isinstance(name, str) and Path(name).is_absolute(), 'relative closure path')
        selected = Path(name).resolve(strict=True)
        require(str(selected) == name and not selected.samefile(executable), 'closure path is not canonical or contains executable')
        require(isinstance(expected, str) and re.fullmatch(hash_pattern, expected), 'invalid DSO hash')
        elf_file(selected)
        require(digest(selected) == expected, f'known role DSO changed: {name}')
    return value


def verify_role_maps(path, closure, executable, observation=None):
    """Hash the actor's actual post-window file-backed ELF selection."""
    raw = path.read_bytes()
    require(raw, 'actual maps output is empty')
    files, observed_paths, binary_paths = {}, {}, []
    binary_hash = digest(executable)
    observed = {'maps_sha256': hashlib.sha256(raw).hexdigest(),
                'executable_sha256': binary_hash, 'mapped_binary_paths': binary_paths,
                'observed_paths': observed_paths, 'files_sha256': files}
    if observation is not None:
        observation.update(observed)
    for line in raw.decode().splitlines():
        fields = line.split(maxsplit=5)
        require(len(fields) in (5, 6), 'malformed maps record')
        require(re.fullmatch(r'[0-9a-fA-F]+-[0-9a-fA-F]+', fields[0])
                and re.fullmatch(r'[r-][w-][x-][ps]', fields[1])
                and re.fullmatch(r'[0-9a-fA-F]+', fields[2])
                and re.fullmatch(r'[0-9a-fA-F]+:[0-9a-fA-F]+', fields[3])
                and fields[4].isdigit(), 'invalid maps columns')
        low, high = [int(v, 16) for v in fields[0].split('-')]
        require(low < high, 'invalid maps range')
        if len(fields) == 5:
            continue
        name = fields[5]
        if not name.startswith('/'):
            require(name.startswith('[') and name.endswith(']'), 'unverified nonabsolute map path')
            continue
        require(not name.endswith(' (deleted)'), 'deleted mapping cannot be qualified')
        selected = Path(name).resolve(strict=True)
        elf_file(selected)
        observed_paths[name] = str(selected)
        if selected.samefile(executable):
            binary_paths.append(name)
        elif str(selected) not in files:
            files[str(selected)] = digest(selected)
    require(binary_paths, 'measured executable is absent from its actual maps')
    require(binary_hash == closure['executable_sha256'], 'mapped executable changed')
    expected = closure['files']
    require(set(files) == set(expected),
            f'role DSO set differs: missing={sorted(set(expected)-set(files))}, extra={sorted(set(files)-set(expected))}')
    require(files == expected, 'actual mapped DSO hash differs from the known role selection')
    observed['mapped_binary_paths'] = sorted(set(binary_paths))
    if observation is not None:
        observation.update(observed)
    return observed


def expected_entities():
    require(digest(XML) == XML_SHA, 'unchanged B03 XML differs')
    worlds = ET.fromstring(XML.read_bytes()).findall('world')
    require(len(worlds) == 1 and worlds[0].attrib['name'] == 'default', 'world differs')
    world = worlds[0]
    require([light.attrib['name'] for light in world.findall('light')] == ['sun'], 'sun input lost')
    names = {'default': 'WORLD'}
    def add(name, family):
        require(name not in names, 'duplicate full scoped name')
        names[name] = family
    for model in world.findall('model'):
        mn = 'default::' + model.attrib['name']
        add(mn, 'MODEL')
        for link in model.findall('link'):
            ln = mn + '::' + link.attrib['name']
            add(ln, 'BODY')
            for collision in link.findall('collision'):
                add(ln + '::' + collision.attrib['name'], 'SHAPE')
        for joint in model.findall('joint'):
            add(mn + '::' + joint.attrib['name'], 'JOINT')
    require({family: list(names.values()).count(family) for family in set(names.values())}
            == {'WORLD': 1, 'MODEL': 3, 'BODY': 7, 'JOINT': 4, 'SHAPE': 15}, 'physical inventory differs')
    return names


def normalized(state, role):
    require(type(state['schema_version']) is int and state['schema_version'] == 1 and state['contract'] == CONTRACT
            and state['role'] == ROLES[role] and state['declared_uri'] == 'b03.sdf'
            and state['body_fields'] == FIELDS, 'state schema/role differs')
    names = expected_entities()
    require(len(state['entities']) == len(names), 'entity count differs')
    by_id, by_name = {}, {}
    for entity in state['entities']:
        name, identity = entity['name'], entity['id']
        require(name in names and name not in by_name and type(identity) is int
                and identity >= 0 and identity not in by_id and entity['family'] == names[name], 'identity differs')
        by_name[name] = entity
        by_id[identity] = name
    require(set(by_name) == set(names), 'physical full name set differs')
    hierarchy = {}
    for name, entity in by_name.items():
        parent_id = entity['parent_id']
        require(parent_id is None or type(parent_id) is int, 'invalid parent identity')
        parent = by_id.get(parent_id)
        require(parent == (name.rsplit('::', 1)[0] if '::' in name else None)
                and (parent_id is None) == (name == 'default'), 'parent differs')
        hierarchy[name] = [entity['family'], parent]
    frames = state['frames']
    require(len(frames) == 100 and [f['tick'] for f in frames] == list(range(1, 101)), 'full100 schedule differs')
    output = []
    for frame in frames:
        require(type(frame['tick']) is int and type(frame['time_ns']) is int
                and frame['time_ns'] == frame['tick'] * 1000000, 'requested step time differs')
        rows = {}
        for family, plural, width in [('BODY', 'bodies', 22), ('JOINT', 'joints', 2)]:
            values = {}
            require(len(frame[plural]) == sum(f == family for f in names.values()), 'query cardinality differs')
            for row in frame[plural]:
                name, v = row['name'], row['values']
                require(name in names and names[name] == family and name not in values
                        and type(row['id']) is int and row['id'] == by_name[name]['id'], 'row ownership differs')
                require(len(v) == width and all(type(n) in (int, float) and math.isfinite(n) for n in v),
                        'nonfinite/incomplete row')
                values[name] = v
            rows[plural] = values
        output.append({'tick': frame['tick'], 'time_ns': frame['time_ns'], **rows})
    return {'hierarchy': hierarchy, 'frames': output}


def comparison(actual, expected):
    require(actual['hierarchy'] == expected['hierarchy'], 'hierarchy mismatch')
    require(len(actual['frames']) == len(expected['frames']), 'frame width mismatch')
    maximum, outside, count, first = 0.0, 0, 0, None
    for a, b in zip(actual['frames'], expected['frames']):
        require((a['tick'], a['time_ns']) == (b['tick'], b['time_ns']), 'time mismatch')
        for plural in ['bodies', 'joints']:
            require(set(a[plural]) == set(b[plural]), 'state name mismatch')
            for name, values in a[plural].items():
                target = b[plural][name]
                require(len(values) == len(target), 'state width mismatch')
                for field, (x, y) in enumerate(zip(values, target)):
                    require(math.isfinite(x) and math.isfinite(y), 'nonfinite comparison')
                    delta = abs(x-y)
                    count += 1
                    maximum = max(maximum, delta)
                    if delta > 1e-12:
                        outside += 1
                        if first is None:
                            first = {'tick': a['tick'], 'name': name, 'field': field, 'actual': x, 'expected': y}
    return {'values': count, 'maximum_delta': maximum, 'outside_1e_12': outside, 'first_outside': first}


def settings(value, role):
    expected = {'gravity': [0, 0, -9.8], 'detector': 'ode', 'solver': 'DantzigBoxedLcpSolver',
                'pair_max_contacts': 'unlimited', 'global_max_contacts': 10000,
                'dt_ns': 1000000, 'solver_iterations': None}
    require(set(value) == set(expected), 'settings fields differ')
    native = {'gravity': 'public-readback', 'detector': 'public-readback', 'solver': 'public-readback',
              'pair_max_contacts': 'public-readback', 'global_max_contacts': 'source-established',
              'dt_ns': 'forward-step-input', 'solver_iterations': 'unsupported'}
    rust = {'gravity': 'sdf-input', 'detector': 'source-established', 'solver': 'source-established',
            'pair_max_contacts': 'source-established', 'global_max_contacts': 'source-established',
            'dt_ns': 'scene-readback', 'solver_iterations': 'unsupported'}
    evidence = native if role == 'native' else rust
    for key, target in expected.items():
        item = value[key]
        require(type(item['value']) is type(target) and item['value'] == target
                and item['evidence'] == evidence[key], f'settings/capability differs: {key}')
        allowed = {'value', 'evidence'}
        if role == 'native' and key == 'pair_max_contacts':
            allowed.add('raw')
            raw = item['raw']
            require(type(raw) is str and raw == '18446744073709551615', 'raw readback differs')
        require(set(item) == allowed, 'unverified settings evidence fields')
    return expected


def metrics(value, role, mode):
    target = {'schema_version': 1, 'contract': CONTRACT, 'role': ROLES[role], 'mode': mode,
              'steps': 100, 'dt_ns': 1000000, 'body_queries': 700, 'joint_queries': 400,
              'worlds': 1, 'bodies': 7, 'joints': 4, 'colliders': 15, 'sensors': 0,
              'rendering': False, 'network': False}
    require(set(value) == set(target) | {'window_wall_ns'}, 'metrics fields differ')
    for key, expected in target.items():
        require(type(value[key]) is type(expected) and value[key] == expected, f'work vector differs: {key}')
    require(type(value['window_wall_ns']) is int and value['window_wall_ns'] > 0, 'invalid window wall time')


def build_record(path, role, executable, actor):
    record = parse(path.read_bytes())
    require(record['role'] == ROLES[role] and record['executable_sha256'] == digest(executable), 'build role/binary differs')
    argv, files = record['compile_argv'], record['inputs_sha256']
    require(isinstance(argv, list) and argv and all(isinstance(a, str) for a in argv), 'actual compiler argv missing')
    require(isinstance(files, dict) and files, 'actual compiler/source inputs missing')
    for name, sha in files.items():
        p = Path(name)
        require(p.is_absolute() and str(p.resolve(strict=True)) == name and digest(p) == sha, 'build input changed')
    require(str(actor.resolve()) in files and str(Path(argv[0]).resolve(strict=True)) in files, 'actor/compiler not bound')
    require(not any('fast-math' in a or a == '-Ofast' for a in argv), 'unreviewed floating point optimization')
    if role == 'native':
        require('-O3' in argv and '-UNDEBUG' in argv
                and all(a == '-O3' for a in argv if re.fullmatch(r'-O(?:[0-9sgz]|fast)?', a))
                and not any(a.startswith('-DNDEBUG') or (a == '-D' and i+1 < len(argv)
                            and argv[i+1].split('=', 1)[0] == 'NDEBUG') for i, a in enumerate(argv)),
                'native optimized/assertion settings differ')
    else:
        options = {}
        for i, arg in enumerate(argv):
            setting = argv[i+1] if arg == '-C' and i+1 < len(argv) else arg[2:] if arg.startswith('-C') else ''
            if '=' not in setting:
                continue
            key, value = setting.split('=', 1)
            if key in {'opt-level', 'debug-assertions', 'overflow-checks'}:
                require(key not in options, 'repeated Rust optimization/assertion setting')
                options[key] = value
        require(set(options) == {'opt-level', 'debug-assertions', 'overflow-checks'}
                and options['opt-level'] == '3'
                and all(options[key] in {'yes', 'on', 'true', 'y'}
                        for key in ['debug-assertions', 'overflow-checks']),
                'Rust release/assertion settings differ')
    return record


def default_comparison(native, fixture):
    require(fixture['schema_version'] == 1 and fixture['xml_sha256'] == XML_SHA
            and fixture['body_fields'] == FIELDS, 'accepted default fixture differs')
    require([f['tick'] for f in fixture['frames']] == [1, 10, 100], 'default selected schedule differs')
    by_tick = {f['tick']: f for f in native['frames']}
    a, b = [], []
    for frame in fixture['frames']:
        rows = {}
        require(type(frame['time_ns']) is int and frame['time_ns'] == frame['tick']*1000000, 'default time differs')
        for plural, width in [('bodies', 22), ('joints', 2)]:
            items = frame[plural]
            require(len(items) == (7 if plural == 'bodies' else 4), 'default cardinality differs')
            rows[plural] = {}
            for row in items:
                require(row['name'] not in rows[plural] and len(row['values']) == width
                        and all(type(v) in (int, float) and math.isfinite(v) for v in row['values']), 'default row differs')
                rows[plural][row['name']] = row['values']
        a.append(by_tick[frame['tick']])
        b.append({'tick': frame['tick'], 'time_ns': frame['time_ns'], **rows})
    return comparison({'hierarchy': native['hierarchy'], 'frames': a},
                      {'hierarchy': native['hierarchy'], 'frames': b})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['native', 'rust', 'plugin', 'native-build-record', 'rust-build-record',
                 'native-closure', 'rust-closure', 'output']:
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--cpu', type=int, required=True)
    args = parser.parse_args()
    require(args.cpu in os.sched_getaffinity(0), 'CPU outside actual allowed affinity')
    require(not args.output.exists(), 'never overwrite a batch')
    args.output.mkdir(parents=True)
    actors = {'native': ROOT / 'reference/b03_physics.cc',
              'rust': ROOT / 'crates/gz-sim-scene/examples/b03_physics_query_benchmark.rs'}
    binaries = {'native': args.native.resolve(strict=True), 'rust': args.rust.resolve(strict=True)}
    closures, builds = {}, {}
    for role in ROLES:
        closures[role] = known_role_closure(getattr(args, role+'_closure'), ROLES[role], binaries[role])
    for role in ROLES:
        builds[role] = build_record(getattr(args, role+'_build_record'), role, binaries[role], actors[role])
    plugin = args.plugin.resolve(strict=True)
    require(str(plugin) in closures['native']['files'], 'selected plugin absent from explicit native DSO closure')
    reference = parse((ROOT / 'reference/b03_physics.json').read_bytes())
    require(reference['contract'] == CONTRACT and reference['xml_sha256'] == XML_SHA, 'source contract differs')
    source_files = reference['source_established_sha256']
    require(source_files and all(digest(ROOT / name) == sha for name, sha in source_files.items()), 'product source binding differs')
    # Selected implementation sources are retained source evidence, not getters
    # or fictitious dependencies of compiling this observer.
    for suffix, sha in reference['native_source_established_sha256'].items():
        matches = [p for p, h in builds['native']['inputs_sha256'].items() if p.endswith('/'+suffix) and h == sha]
        require(len(matches) == 1, 'native source-established setting is not bound to reviewed source')
    manifest_matches = [p for p, h in builds['native']['inputs_sha256'].items()
                        if p.endswith('/zenoh-dart-environment.json')
                        and h == reference['native_sdk']['manifest_sha256']]
    require(len(manifest_matches) == 1, 'qualified native SDK manifest is not bound')
    require(digest(plugin) == reference['native_plugin_sha256'], 'selected native plugin differs')
    for role in ROLES:
        for name, sha in reference['required_common_libraries_sha256'].items():
            matches = [p for p, h in closures[role]['files'].items() if Path(p).name == name and h == sha]
            require(len(matches) == 1, f'qualified common library missing/changed: {role}/{name}')
    fixture_path = ROOT / 'crates/gz-sim-scene/tests/fixtures/b03_world_fresh_native.json'
    require(digest(fixture_path) == reference['accepted_default_fixture_sha256'], 'accepted default reference changed')
    fixture = parse(fixture_path.read_bytes())
    paths = {XML.resolve(), Path(__file__).resolve(), (ROOT / 'reference/b03_physics.json').resolve(), fixture_path.resolve(),
             plugin, *binaries.values(), *(p.resolve() for p in actors.values()),
             args.native_build_record.resolve(), args.rust_build_record.resolve(), args.native_closure.resolve(), args.rust_closure.resolve()}
    paths.update((ROOT / p).resolve() for p in source_files)
    for role in ROLES:
        paths.update(Path(p) for p in closures[role]['files'])
        paths.update(Path(p) for p in builds[role]['inputs_sha256'])
    before = {str(p): digest(p) for p in sorted(paths)}
    result = {'contract': CONTRACT, 'status': 'incomplete', 'equal_work_eligible': False,
              'scope': 'explicitly configured physics-only; no Server/network/IO/renderer/sensors',
              'input_sha256': before, 'commands': [], 'runtime_closures': [], 'pairs': [],
              'cpu': args.cpu, 'affinity_before': sorted(os.sched_getaffinity(0)),
              'process_metric_scope': 'whole process including setup/output/destruction; GNU time resolution retained',
              'window_scope': '100 steps +700 fresh22-field body rows +400 two-scalar joint rows; preallocated frame storage',
              'clock_scope': 'requested/count-derived1ms step timestamps; no native public clock getter',
              'settings_scope': 'per-field readback or source-established as labeled; not all solver internals independently readable',
              'full_server_equal_work': False, 'product_default_change': False}
    def save():
        (args.output / 'result.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    save()
    try:
        baseline = None
        for pair in range(8):
            order = ['native', 'rust'] if pair % 2 == 0 else ['rust', 'native']
            mode = 'correctness' if pair == 0 else 'measure'
            states, values, config = {}, {}, {}
            for role in order:
                prefix = (args.output / f'{pair}-{role}').resolve()
                argv = ['/usr/bin/time', '-f', '%U %S %e %M', '-o', str(prefix)+'.process-time',
                        '/usr/bin/taskset', '--cpu-list', str(args.cpu), str(binaries[role])]
                if role == 'native':
                    argv.append(str(plugin))
                argv += [str(XML), str(prefix), mode]
                result['commands'].append({'pair': pair, 'role': role, 'argv': argv})
                save()
                with open(str(prefix)+'.stdout', 'xb') as stdout, open(str(prefix)+'.stderr', 'xb') as stderr:
                    child = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=stdout, stderr=stderr, start_new_session=True)
                    try:
                        status = child.wait(timeout=120)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL)
                        child.wait()
                        raise RuntimeError(f'owned actor timed out: pair{pair}/{role}')
                require(status == 0, f'actor failed: pair{pair}/{role}: {status}')
                require(not Path(str(prefix)+'.stderr').read_bytes(), 'unexpected stderr retained')
                require(Path(str(prefix)+'.input.sdf').read_bytes() == XML.read_bytes(), 'actor input differs')
                runtime = {'pair': pair, 'role': role}
                result['runtime_closures'].append(runtime)
                verify_role_maps(Path(str(prefix)+'.maps'), closures[role], binaries[role], runtime)
                payload = parse(Path(str(prefix)+'.state.json').read_bytes())
                states[role] = normalized(payload, role)
                config[role] = settings(payload['settings'], role)
                work = parse(Path(str(prefix)+'.metrics.json').read_bytes())
                metrics(work, role, mode)
                process_time = Path(str(prefix)+'.process-time').read_text().split()
                require(len(process_time) == 4, 'GNU time record differs')
                user, system, elapsed = map(float, process_time[:3])
                rss = int(process_time[3])
                require(all(math.isfinite(v) and v >= 0 for v in [user, system, elapsed]) and rss > 0, 'invalid process time/RSS')
                values[role] = {'window_wall_ns': work['window_wall_ns'], 'process_user_seconds': user,
                                'process_system_seconds': system, 'process_wall_seconds': elapsed, 'peak_rss_kib': rss}
            require(config['native'] == config['rust'], 'common configured values differ')
            paired = comparison(states['rust'], states['native'])
            result['pairs'].append({'pair': pair, 'excluded_correctness_warmup': pair == 0, 'order': order,
                                    'comparison': paired, **values})
            save()
            require(paired['outside_1e_12'] == 0, 'full100 configured paired correctness failed; no timing eligibility')
            if pair == 0:
                compatibility = default_comparison(states['native'], fixture)
                result['configured_native_vs_default_selected'] = compatibility
                if compatibility['outside_1e_12'] == 0:
                    rust_default = default_comparison(states['rust'], fixture)
                    result['rust_vs_compatible_default_selected'] = rust_default
                    require(rust_default['outside_1e_12'] == 0, 'Rust default-compatible selected reference failed')
                baseline = states
                result['paired_configuration_and_correctness'] = 'passed before measured pair1'
            else:
                for role in ROLES:
                    deterministic = comparison(states[role], baseline[role])
                    require(deterministic['outside_1e_12'] == 0, 'role differs from excluded correctness capture')
            require({str(p): digest(p) for p in sorted(paths)} == before, 'source/build/runtime input changed')
        result['summary'] = {role: {metric: {'median': statistics.median(v), 'min': min(v), 'max': max(v)}
                                    for metric in result['pairs'][1][role]
                                    for v in [[pair[role][metric] for pair in result['pairs'][1:]]]}
                             for role in ROLES}
        result['equal_work_eligible'] = True
        result['physics_only_native_over_rust_window_median'] = (
            result['summary']['native']['window_wall_ns']['median'] / result['summary']['rust']['window_wall_ns']['median'])
        result['status'] = 'completed configured physics-only batch; not original-default/full-Server/rewrite acceptance'
    except BaseException as error:
        result['status'] = 'failed; no retry or performance eligibility'
        result['equal_work_eligible'] = False
        result['failure'] = repr(error)
        raise
    finally:
        result['output_sha256'] = {p.name: digest(p) for p in sorted(args.output.iterdir()) if p.is_file() and p.name != 'result.json'}
        save()


if __name__ == '__main__':
    main()
