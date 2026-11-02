#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Materialize only an actual reviewed Gazebo Server B03 observation."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shlex
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'reference/b03_world.cc'
XML = ROOT / 'crates/gz-sim-scene/tests/fixtures/b03_double_pendulum.sdf'
OUTPUT = ROOT / 'crates/gz-sim-scene/tests/fixtures/b03_world_native.json'
XML_SHA256 = '24e85ff944b5efe2d758fdda543e392f34d59f26297a0a87fafbb41d5a3b16aa'
FIELDS = ['px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw', 'origin_vx', 'origin_vy', 'origin_vz',
          'com_vx', 'com_vy', 'com_vz', 'wx', 'wy', 'wz', 'com_px', 'com_py', 'com_pz',
          'local_com_x', 'local_com_y', 'local_com_z']


def require(ok, why):
    if not ok:
        raise ValueError(why)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def unique(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, f'duplicate key: {key}')
        result[key] = value
    return result


def parse(data):
    def reject(value):
        raise ValueError(f'nonfinite JSON: {value}')
    return json.loads(data, object_pairs_hook=unique, parse_constant=reject)


def expected_names(xml):
    world, = ET.fromstring(xml).findall('world')
    names = {world.attrib['name']: 'WORLD'}
    prefix = world.attrib['name'] + '::'
    for light in world.findall('light'):
        names[prefix + light.attrib['name']] = 'LIGHT'
    for model in world.findall('model'):
        model_name = prefix + model.attrib['name']
        names[model_name] = 'MODEL'
        for link in model.findall('link'):
            link_name = model_name + '::' + link.attrib['name']
            names[link_name] = 'BODY'
            for collision in link.findall('collision'):
                names[link_name + '::' + collision.attrib['name']] = 'COLLISION'
        for joint in model.findall('joint'):
            names[model_name + '::' + joint.attrib['name']] = 'JOINT'
    require({family: sum(v == family for v in names.values()) for family in set(names.values())}
            == {'WORLD': 1, 'MODEL': 3, 'LIGHT': 1, 'BODY': 7, 'JOINT': 4, 'COLLISION': 15},
            'B03 identity domain changed')
    return names


def decode_state(text, xml):
    expected = expected_names(xml)
    lines = [shlex.split(line) for line in text.splitlines()]
    require(len(lines) == 2 + 31 + 3*(1+7+4), 'state record count changed')
    first, gravity, *records = lines
    require(len(first) == 5 and first[:2] == ['INPUT', 'b03.sdf'], 'input tag changed')
    require(first[2] and int(first[4]) >= -1, 'native input/source context missing')
    require(len(gravity) == 4 and gravity[0] == 'GRAVITY', 'gravity record missing')
    gravity = [float(v) for v in gravity[1:]]
    require(all(math.isfinite(v) for v in gravity), 'nonfinite gravity')
    entities, by_id = {}, {}
    for record in records[:31]:
        require(len(record) == 5 and record[0] == 'ENTITY', 'identity record schema')
        _, family, name, identity, parent = record
        identity, parent = int(identity), int(parent)
        require(expected.get(name) == family and name not in entities and identity > 0
                and identity not in by_id, 'identity family/name/ID differs')
        entities[name] = {'family': family, 'name': name, 'id': identity,
                          'parent_id': None if parent == 0 else parent}
        by_id[identity] = name
    require(set(entities) == set(expected), 'incomplete entity set')
    for name, entity in entities.items():
        parent = name.rsplit('::', 1)[0] if '::' in name else None
        require((by_id.get(entity['parent_id']) if entity['parent_id'] else None) == parent,
                f'wrong native ownership: {name}')
    frames = []
    remaining = records[31:]
    for index, tick in enumerate([1, 10, 100]):
        marker, *rows = remaining[index*12:index*12+12]
        require(marker == ['FRAME', str(tick), str(tick*1000000)], 'actual native step clock differs')
        bodies, joints = {}, {}
        for row in rows:
            require(row[0] in ['BODY', 'JOINT'] and int(row[1]) == tick, 'row family/tick differs')
            family, _, name, identity, *numbers = row
            count = 22 if family == 'BODY' else 2
            require(len(numbers) == count and entities.get(name, {}).get('family') == family
                    and int(identity) == entities[name]['id'], 'state identity differs')
            values = [float(v) for v in numbers]
            require(all(math.isfinite(v) for v in values), 'nonfinite native row')
            if family == 'BODY':
                require(abs(sum(v*v for v in values[3:7])-1) <= 1e-12, 'nonunit body orientation')
            target = bodies if family == 'BODY' else joints
            require(name not in target, 'duplicate state row')
            target[name] = {'name': name, 'id': int(identity), 'values': values}
        require(set(bodies) == {n for n, f in expected.items() if f == 'BODY'}
                and set(joints) == {n for n, f in expected.items() if f == 'JOINT'}, 'query set changed')
        frames.append({'tick': tick, 'time_ns': tick*1000000,
                       'bodies': list(bodies.values()), 'joints': list(joints.values())})
    return {'declared_uri': first[1], 'native_source': {'set_sdf_file': first[2],
            'world_file_path': first[3], 'world_line': int(first[4])},
            'gravity': gravity, 'entities': list(entities.values()), 'frames': frames}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prefix', type=Path, help='actual observer output prefix')
    parser.add_argument('provenance', type=Path, help='successful actual capture result.json')
    parser.add_argument('--output', type=Path, default=OUTPUT)
    args = parser.parse_args()
    paths = [SOURCE, XML, args.provenance] + [Path(str(args.prefix)+suffix)
            for suffix in ['.state', '.input.sdf', '.maps', '.metrics.json']]
    require(all(p.is_file() and not p.is_symlink() for p in paths), 'nonregular/missing capture input')
    before = {p: p.read_bytes() for p in paths}
    require(digest(before[XML]) == XML_SHA256, 'canonical B03 fixture changed')
    require(before[Path(str(args.prefix)+'.input.sdf')] == before[XML], 'actual native input differs')
    provenance = parse(before[args.provenance])
    require(provenance.get('state') == 'Actual B03 Server observations collected; no Rust/native comparison claimed'
            and 'failure' not in provenance and 'traceback' not in provenance, 'unsuccessful capture')
    require(provenance['generator_sha256'] == digest(before[SOURCE]), 'generator binding changed')
    metrics = parse(before[Path(str(args.prefix)+'.metrics.json')])
    for key, expected in {'role': 'gazebo-server', 'mode': 'correctness', 'steps': 100, 'dt_ns': 1000000,
                          'body_queries': 700, 'joint_queries': 400, 'worlds': 1, 'bodies': 7,
                          'joints': 4, 'colliders': 15, 'sensors': 0, 'rendering': False,
                          'paused_preparation_calls': 1,
                          'scope': 'contextual-loaded-world-physics-query',
                          'intrinsic_server_network': 'present-unmatched'}.items():
        require(type(metrics.get(key)) is type(expected) and metrics[key] == expected, f'work vector differs: {key}')
    require(type(metrics['window_wall_ns']) is int and metrics['window_wall_ns'] > 0, 'missing measured window')
    state = before[Path(str(args.prefix)+'.state')]
    observed = decode_state(state.decode(), before[XML])
    result = {'schema_version': 1, 'copyright': 'Copyright (C) 2026 Open Source Robotics Foundation',
              'license': 'Apache-2.0', 'scope': 'Actual Gazebo Server B03 observations, no Rust parity/performance acceptance',
              'xml': before[XML].decode(), 'xml_sha256': XML_SHA256,
              'generator_sha256': digest(before[SOURCE]), 'provenance_sha256': digest(before[args.provenance]),
              'body_fields': FIELDS, 'raw_state': state.decode(), 'metrics': metrics,
              'capture_sha256': {suffix: digest(before[Path(str(args.prefix)+suffix)])
                                  for suffix in ['.state', '.input.sdf', '.maps', '.metrics.json']},
              **observed}
    require(all(p.read_bytes() == data for p, data in before.items()), 'capture changed')
    with args.output.open('x') as output:
        output.write(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(f'Materialized actual B03 world observations: {args.output}')


if __name__ == '__main__':
    main()
