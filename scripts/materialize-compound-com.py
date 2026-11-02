#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Materialize actual offset-COM observations without synthesizing state values."""
import argparse
import importlib.util
import math
from pathlib import Path
import json

ROOT = Path(__file__).resolve().parents[1]
helper_spec = importlib.util.spec_from_file_location(
    'compound_capture', ROOT / 'scripts/materialize-compound-collisions.py')
base = importlib.util.module_from_spec(helper_spec)
helper_spec.loader.exec_module(base)
SOURCE = ROOT / 'crates/gz-physics-dart/tests/fixtures/compound_com.cc'
OUTPUT = ROOT / 'crates/gz-physics-dart/tests/fixtures/compound_com.json'
EXTRA = {'origin_velocity', 'com_position', 'local_com'}
CENTERS = ([0.03, -0.02, 0.05], [0.1, -0.05, 0.2], [0.0, 0.0, 0.5])
CENTER_TEXT = ('0.03 -0.02 0.05', '0.1 -0.05 0.2', '0 0 0.5')


def expected_xml(count, contact):
    original = '<inertial><pose>0 0 0 0.1 0.2 0.3</pose>'
    xml = base.expected_xml(count, contact)
    base.require(xml.count(original) == 1, 'centered reference XML changed')
    return xml.replace(original, f'<inertial><pose>{CENTER_TEXT[count-1]} 0.1 0.2 0.3</pose>')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['generated', 'stdout', 'stderr', 'provenance']:
        parser.add_argument(name, type=Path)
    parser.add_argument('--output', type=Path, default=OUTPUT)
    args = parser.parse_args()
    require = base.require
    require(args.generated.is_dir() and not args.generated.is_symlink(), 'invalid capture directory')
    names = {f'{i}.sdf' for i in range(6)} | {'before.maps', 'after.maps'}
    require({p.name for p in args.generated.iterdir()} == names, 'capture files differ')
    paths = [SOURCE, args.stdout, args.stderr, args.provenance]
    paths += [args.generated / name for name in sorted(names)]
    require(all(p.is_file() and not p.is_symlink() for p in paths), 'capture contains nonregular files')
    before = {p: p.read_bytes() for p in paths}
    require(not before[args.stderr], 'native stderr is not empty')
    provenance = base.parse(before[args.provenance].decode())
    require(isinstance(provenance, dict)
            and provenance.get('state') == 'Actual plugin observations collected; no Rust/native comparison claimed'
            and 'failure' not in provenance and 'traceback' not in provenance,
            'native provenance does not record a successful capture')
    generator = base.digest(before[SOURCE])
    require(provenance['generator_sha256'] == generator, 'generator source binding differs')
    rows = [base.parse(line) for line in before[args.stdout].decode().splitlines()]
    require(len(rows) == 30, 'expected exactly30 native rows')
    cases = []
    for scenario in range(6):
        count, contact = scenario // 2 + 1, scenario % 2
        xml = before[args.generated / f'{scenario}.sdf']
        require(xml == expected_xml(count, contact).encode(), f'XML input differs: {scenario}')
        selected = rows[scenario*5:scenario*5+5]
        for row, tick in zip(selected, base.TICKS):
            require(isinstance(row, dict) and set(row) == base.FIELDS | EXTRA, 'COM row schema differs')
            base.validate_row({key: row[key] for key in base.FIELDS}, scenario, count, contact, tick)
            for key in EXTRA:
                values = row[key]
                require(isinstance(values, list) and len(values) == 3
                        and all(type(v) in (int, float) and math.isfinite(v) for v in values),
                        f'invalid {key}')
            require(row['local_com'] == CENTERS[count-1], 'actual native DOM COM differs')
        if contact:
            initial = selected[0]['com_position'][2]
            ballistic = initial - 9.81 * 0.001**2 * 600 * 601 / 2
            require(selected[-1]['com_position'][2] > ballistic + 0.01
                    and selected[-1]['velocity'][2] > -9.81 * 0.6 + 0.01,
                    f'no actual COM contact response: {scenario}')
        cases.append({'scenario': scenario, 'count': count, 'contact': contact,
                      'xml': xml.decode(), 'xml_sha256': base.digest(xml), 'rows': selected})
    result = {
        'schema_version': 1,
        'copyright': 'Copyright (C) 2026 Open Source Robotics Foundation',
        'license': 'Apache-2.0',
        'scope': 'Native DART offset-COM compound-link reference; no B03 acceptance.',
        'generator_sha256': generator,
        'capture': {'provenance_sha256': base.digest(before[args.provenance]),
                    'stdout_sha256': base.digest(before[args.stdout]),
                    'stderr_sha256': base.digest(before[args.stderr]),
                    'maps_sha256': {name: base.digest(before[args.generated / f'{name}.maps'])
                                    for name in ['before', 'after']}},
        'cases': cases,
    }
    require(all(p.read_bytes() == data for p, data in before.items()), 'capture changed')
    with args.output.open('x') as output:
        output.write(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(f'Materialized six actual offset-COM inputs and30 rows: {args.output}')


if __name__ == '__main__':
    main()
