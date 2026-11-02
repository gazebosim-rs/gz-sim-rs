#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Repackage the actual two-representation diagnostic; never infer state values."""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'compound_capture', ROOT / 'scripts/materialize-compound-collisions.py')
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)
SOURCE = ROOT / 'crates/gz-physics-dart/tests/fixtures/compound_com_gauge.cc'
OUTPUT = ROOT / 'crates/gz-physics-dart/tests/fixtures/compound_com_gauge.json'
FIELDS = {'scenario', 'tick', 'time_ns', 'wrench_this_step', 'position',
          'orientation', 'velocity', 'angular_velocity', 'origin_velocity',
          'com_position', 'local_com', 'world_shapes'}
VECTORS = {'position': 3, 'orientation': 4, 'velocity': 3,
           'angular_velocity': 3, 'origin_velocity': 3,
           'com_position': 3, 'local_com': 3}


def vector(value, length):
    base.require(isinstance(value, list) and len(value) == length
                 and all(type(v) in (int, float) and math.isfinite(v) for v in value),
                 'invalid finite vector')


def xml_input(xml, inputs, case):
    root = ET.fromstring(xml)
    base.require(root.tag == 'sdf' and root.attrib == {'version': '1.11'}, 'SDF version')
    world, = list(root)
    base.require(world.tag == 'world' and world.attrib == {'name': 'w'}, 'world identity')
    base.require([e.tag for e in world] == ['gravity', 'model'], 'world payload changed')
    base.require(world.findtext('gravity') == '0 0 0', 'gravity changed')
    model = world.find('model')
    base.require(model.attrib == {'name': 'compound'} and len(model) == 1, 'model changed')
    link = model.find('link')
    base.require(link is not None and link.attrib == {'name': 'body'}, 'link identity')
    base.require([e.tag for e in link] == ['pose', 'inertial', 'collision', 'collision', 'collision'],
                 'link payload changed')

    def pose(element, position, quaternion):
        base.require(element.attrib == {'rotation_format': 'quat_xyzw'}, 'pose representation')
        values = [float(v) for v in element.text.split()]
        vector(values, 7)
        base.require(values == position + quaternion, 'pose input changed')

    pose(link.find('pose'), case['position'], inputs['body_quaternion'])
    inertial = link.find('inertial')
    base.require([e.tag for e in inertial] == ['pose', 'mass', 'inertia'], 'inertial payload')
    pose(inertial.find('pose'), case['local_com'], [0, 0, 0, 1])
    base.require(inertial.findtext('mass') == '1', 'mass changed')
    tensor = inertial.find('inertia')
    base.require([e.tag for e in tensor] == ['ixx', 'iyy', 'izz']
                 and [e.text for e in tensor] == ['1', '1', '1'], 'inertia changed')
    center = [0.1, -0.05, 0.2]
    for index, collision in enumerate(link.findall('collision')):
        base.require(collision.attrib == {'name': f'c{index}'}
                     and [e.tag for e in collision] == ['pose', 'geometry'], 'collision changed')
        position = [[-0.4, 0, 0], [0, 0, 0], [0.4, 0, 0]][index]
        if case['scenario'] == 1:
            position = [v - c for v, c in zip(position, center)]
        pose(collision.find('pose'), position,
             inputs['box_quaternion'] if index == 1 else [0, 0, 0, 1])
        geometry = collision.find('geometry')
        shape, = list(geometry)
        base.require(shape.tag == ('box' if index == 1 else 'sphere'), 'geometry kind')
        base.require([e.tag for e in shape] == (['size'] if index == 1 else ['radius'])
                     and [e.text for e in shape] == (['0.6 0.4 0.8'] if index == 1 else ['0.2']),
                     'geometry dimensions')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['generated', 'stdout', 'stderr', 'provenance']:
        parser.add_argument(name, type=Path)
    parser.add_argument('--output', type=Path, default=OUTPUT)
    args = parser.parse_args()
    names = {'0.sdf', '1.sdf', 'inputs.json', 'before.maps', 'after.maps'}
    base.require(args.generated.is_dir() and not args.generated.is_symlink()
                 and {p.name for p in args.generated.iterdir()} == names, 'capture files differ')
    paths = [SOURCE, args.stdout, args.stderr, args.provenance]
    paths += [args.generated / name for name in sorted(names)]
    base.require(all(p.is_file() and not p.is_symlink() for p in paths), 'nonregular capture input')
    before = {p: p.read_bytes() for p in paths}
    base.require(not before[args.stderr], 'unexpected native stderr')
    provenance = base.parse(before[args.provenance].decode())
    base.require(provenance.get('state') == 'Actual plugin observations collected; no Rust/native comparison claimed'
                 and 'failure' not in provenance and 'traceback' not in provenance,
                 'unsuccessful native capture')
    base.require(provenance.get('time_scope') == 'Row time_ns is requested dt times counted steps, not queried native clock',
                 'diagnostic clock scope changed')
    generator = base.digest(before[SOURCE])
    base.require(provenance['generator_sha256'] == generator, 'generator source changed')
    inputs = base.parse(before[args.generated / 'inputs.json'].decode())
    expected = {'source_test': 'changing_body_origin_preserves_world_shapes_com_motion_and_pending_wrenches',
                'dt_ns': 1000000, 'steps': 10, 'gravity': [0, 0, 0], 'mass': 1,
                'inertia': [1, 1, 1, 0, 0, 0], 'inertial_orientation': [0, 0, 0, 1],
                'com_velocity': [0.2, -0.1, 0.3], 'angular_velocity': [0.1, 0.2, -0.3],
                'first_step_world_force': [1, 2, 3], 'first_step_world_torque': [0.2, -0.1, 0.3]}
    base.require(set(inputs) == set(expected) | {'raw_body_quaternion', 'body_quaternion', 'box_quaternion', 'cases'},
                 'input transcript schema changed')
    base.require(all(inputs[k] == v for k, v in expected.items()), 'declared input changed')
    for key in ['raw_body_quaternion', 'body_quaternion', 'box_quaternion']:
        vector(inputs[key], 4)
        base.require(abs(sum(v*v for v in inputs[key])-1) <= 1e-12, 'nonunit input')
    base.require(len(inputs['cases']) == 2, 'representation count changed')
    rows = [base.parse(line) for line in before[args.stdout].decode().splitlines()]
    base.require(len(rows) == 22, 'expected exactly22 rows')
    cases = []
    for scenario, case in enumerate(inputs['cases']):
        base.require(set(case) == {'scenario', 'position', 'local_com', 'commanded_origin_velocity'}
                     and type(case['scenario']) is int and case['scenario'] == scenario, 'case input changed')
        for key in ['position', 'local_com', 'commanded_origin_velocity']:
            vector(case[key], 3)
        base.require(case['local_com'] == ([0.1, -0.05, 0.2] if scenario == 0 else [0, 0, 0]),
                     'case COM changed')
        if scenario == 0:
            base.require(case['position'] == [1, 2, 3], 'first origin changed')
        xml = before[args.generated / f'{scenario}.sdf']
        xml_input(xml, inputs, case)
        selected = rows[scenario*11:scenario*11+11]
        for tick, row in enumerate(selected):
            base.require(set(row) == FIELDS, 'row schema changed')
            base.require(type(row['scenario']) is int and row['scenario'] == scenario
                         and type(row['tick']) is int and row['tick'] == tick
                         and type(row['time_ns']) is int and row['time_ns'] == tick*1000000
                         and row['wrench_this_step'] is (tick == 1), 'row coordinate changed')
            for key, length in VECTORS.items():
                vector(row[key], length)
            base.require(row['local_com'] == case['local_com'], 'observed COM changed')
            base.require(abs(sum(v*v for v in row['orientation'])-1) <= 1e-12, 'nonunit observation')
            base.require(len(row['world_shapes']) == 3, 'shape observation count')
            for shape in row['world_shapes']:
                base.require(set(shape) == {'position', 'orientation'}, 'shape schema')
                vector(shape['position'], 3)
                vector(shape['orientation'], 4)
        cases.append({'scenario': scenario, 'xml': xml.decode(),
                      'xml_sha256': base.digest(xml), 'rows': selected})
    result = {'schema_version': 1,
              'copyright': 'Copyright (C) 2026 Open Source Robotics Foundation', 'license': 'Apache-2.0',
              'scope': 'Native plugin origin-change diagnostic; no gauge invariance or B03 motion claim.',
              'time_scope': provenance['time_scope'], 'generator_sha256': generator,
              'capture': {'provenance_sha256': base.digest(before[args.provenance]),
                          'stdout_sha256': base.digest(before[args.stdout]),
                          'stderr_sha256': base.digest(before[args.stderr]),
                          'inputs_sha256': base.digest(before[args.generated / 'inputs.json']),
                          'maps_sha256': {name: base.digest(before[args.generated / f'{name}.maps'])
                                          for name in ['before', 'after']}},
              'inputs_raw': before[args.generated / 'inputs.json'].decode(),
              'inputs': inputs, 'cases': cases}
    base.require(all(p.read_bytes() == data for p, data in before.items()), 'capture changed')
    with args.output.open('x') as output:
        output.write(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(f'Materialized actual two-representation22-row diagnostic: {args.output}')


if __name__ == '__main__':
    main()
