#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Repackage one reviewed native capture; never generate expected physics values."""
import argparse
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'crates/gz-physics-dart/tests/fixtures/compound_collisions.cc'
OUTPUT = ROOT / 'crates/gz-physics-dart/tests/fixtures/compound_collisions.json'
TICKS = (0, 1, 10, 100, 600)
FIELDS = {'scenario', 'count', 'contact', 'tick', 'position', 'orientation',
          'velocity', 'angular_velocity'}


def require(valid, message):
    if not valid:
        raise ValueError(message)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, f'duplicate JSON key: {key}')
        result[key] = value
    return result


def rejected_constant(value):
    raise ValueError(f'nonfinite JSON constant: {value}')


def parse(text):
    return json.loads(text, object_pairs_hook=unique_object,
                      parse_constant=rejected_constant)


def expected_xml(count, contact):
    def collision(index, pose, geometry):
        return (f"<collision name='c{index}'><pose>{pose}</pose><geometry>"
                f'{geometry}</geometry></collision>')
    sphere = '<sphere><radius>0.2</radius></sphere>'
    box = '<box><size>0.6 0.4 0.8</size></box>'
    cylinder = '<cylinder><radius>0.2</radius><length>1</length></cylinder>'
    capsule = '<capsule><radius>0.2</radius><length>0.8</length></capsule>'
    shapes = [collision(0, '0 0 0 0 0 0', sphere),
              collision(0, '-0.4 0 0 0 0 0', box)
              + collision(1, '0.4 0 0 0 1.5708 0.3', cylinder),
              collision(0, '-0.4 0 0 0 0 0', box)
              + collision(1, '0 0 0 0 1.5708 0.3', cylinder)
              + collision(2, '0.4 0 0 0.2 0.4 -0.1', capsule)]
    floor = ("<model name='floor'><static>true</static><link name='body'>"
             "<collision name='plane'><geometry><plane><normal>0 0 1</normal>"
             '<size>100 100</size></plane></geometry></collision></link></model>') if contact else ''
    return ("<sdf version='1.11'><world name='w'><gravity>0 0 "
            + ('-9.81' if contact else '0') + '</gravity>' + floor
            + "<model name='compound'><link name='body'><pose>0 0 "
            + ('1' if contact else '2') + ' 0.2 0.1 -0.3</pose>'
            + '<inertial><pose>0 0 0 0.1 0.2 0.3</pose><mass>2</mass><inertia>'
            + '<ixx>1</ixx><iyy>1.5</iyy><izz>2</izz></inertia></inertial>'
            + shapes[count - 1] + '</link></model></world></sdf>')


def validate_row(row, scenario, count, contact, tick):
    require(isinstance(row, dict) and set(row) == FIELDS, 'native row schema changed')
    for key, value in [('scenario', scenario), ('count', count),
                       ('contact', contact), ('tick', tick)]:
        require(type(row[key]) is int and row[key] == value, f'wrong native {key}')
    for key, length in [('position', 3), ('orientation', 4),
                        ('velocity', 3), ('angular_velocity', 3)]:
        values = row[key]
        require(isinstance(values, list) and len(values) == length
                and all(type(v) in (int, float) and math.isfinite(v) for v in values),
                f'invalid native {key}')
    require(abs(sum(v * v for v in row['orientation']) - 1) <= 1e-12,
            'native quaternion is not unit length')
    require(all(abs(v) <= 5e8 for v in row['position'])
            and all(abs(v) <= 1e60 for key in ['velocity', 'angular_velocity'] for v in row[key]),
            'native state leaves the admitted numeric domain')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('generated', type=Path)
    parser.add_argument('stdout', type=Path)
    parser.add_argument('stderr', type=Path)
    parser.add_argument('provenance', type=Path)
    parser.add_argument('--output', type=Path, default=OUTPUT)
    args = parser.parse_args()
    paths = [SOURCE, args.stdout, args.stderr, args.provenance]
    require(args.generated.is_dir() and not args.generated.is_symlink(), 'invalid capture directory')
    names = {f'{i}.sdf' for i in range(6)} | {'before.maps', 'after.maps'}
    require({p.name for p in args.generated.iterdir()} == names, 'capture files differ')
    paths += [args.generated / name for name in sorted(names)]
    require(all(p.is_file() and not p.is_symlink() for p in paths), 'capture contains non-regular files')
    before = {p: p.read_bytes() for p in paths}
    require(not before[args.stderr], 'native stderr is not empty')
    provenance = parse(before[args.provenance].decode())
    require(isinstance(provenance, dict)
            and provenance.get('state') == 'Actual plugin observations collected; no Rust/native comparison claimed'
            and 'failure' not in provenance and 'traceback' not in provenance,
            'native provenance does not record a successful capture')
    generator = digest(before[SOURCE])
    require(provenance['generator_sha256'] == generator, 'compiled generator source binding differs')
    rows = [parse(line) for line in before[args.stdout].decode().splitlines()]
    require(len(rows) == 30, 'expected exactly30 native rows')
    cases = []
    for scenario in range(6):
        count, contact = scenario // 2 + 1, scenario % 2
        xml = before[args.generated / f'{scenario}.sdf']
        require(xml == expected_xml(count, contact).encode(), f'XML input differs: {scenario}')
        selected = rows[scenario * 5:scenario * 5 + 5]
        for row, tick in zip(selected, TICKS):
            validate_row(row, scenario, count, contact, tick)
        if contact:
            # Semi-implicit free fall has v=-g*N*dt and z=z0-g*dt²*N*(N+1)/2.
            # A captured floor case must actually depart from that motion.
            ballistic_z = 1 - 9.81 * 0.001**2 * 600 * 601 / 2
            require(selected[-1]['position'][2] > ballistic_z + 0.01
                    and selected[-1]['velocity'][2] > -9.81 * 0.6 + 0.01,
                    f'no actual floor-contact response captured: {scenario}')
        cases.append({'scenario': scenario, 'count': count, 'contact': contact,
                      'xml': xml.decode(), 'xml_sha256': digest(xml), 'rows': selected})
    result = {
        'schema_version': 1,
        'copyright': 'Copyright (C) 2026 Open Source Robotics Foundation',
        'license': 'Apache-2.0',
        'scope': 'Native DART compound-link reference; no B03 acceptance.',
        'generator_sha256': generator,
        'capture': {'provenance_sha256': digest(before[args.provenance]),
                    'stdout_sha256': digest(before[args.stdout]),
                    'stderr_sha256': digest(before[args.stderr]),
                    'maps_sha256': {name: digest(before[args.generated / f'{name}.maps'])
                                    for name in ['before', 'after']}},
        'cases': cases,
    }
    require(all(p.read_bytes() == data for p, data in before.items()), 'capture changed during materialization')
    with args.output.open('x') as output:
        output.write(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(f'Materialized six actual native inputs and30 rows: {args.output}')


if __name__ == '__main__':
    main()
