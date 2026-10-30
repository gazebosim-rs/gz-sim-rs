#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Validate captured native math results against the pinned case inventory."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

IMAGE = 'sha256:11b7378c7cca2844a4b7b2c9f52525e983b19d6651d33c83375d220672462d1d'
ENVIRONMENT = '0775304639aa56e106de8f49802246bb54a16816185cafdc433b0006ddd1f59f'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def passing_xml(path):
    if path.is_symlink() or not path.is_file():
        raise ValueError('missing or linked result XML')
    root = ET.parse(path).getroot()
    if root.tag not in ('testsuite', 'testsuites'):
        raise ValueError('unexpected XML root')
    cases = list(root.iter('testcase'))
    if int(root.attrib['tests']) != len(cases) or not cases:
        raise ValueError('XML case count mismatch')
    for node in root.iter():
        if node.tag in ('failure', 'error', 'skipped'):
            raise ValueError('native failure or skip')
        for key in ('failures', 'errors', 'disabled', 'skipped'):
            if int(node.get(key, '0')) != 0:
                raise ValueError('native failure or skip total')
    for case in cases:
        if case.get('status') != 'run' or case.get('result', 'completed') != 'completed':
            raise ValueError('native case did not complete')
    return cases


def summarize(inventory, capture):
    run = json.loads((capture / 'run.json').read_bytes())
    expected_command = ['ctest', '--test-dir', '/reference/builds/gz-math', '-j2',
                        '--output-on-failure', '--output-junit', '/out/ctest-junit.xml']
    if (run['image_id'] != IMAGE or run['environment_sha256'] != ENVIRONMENT
            or type(run['returncode']) is not int or run['returncode'] != 0
            or run['command'] != expected_command):
        raise ValueError('native run identity or exit differs')
    rows = [row for row in inventory['entries'] if row['repository'] == 'gz-math']
    registered = {row['ctest'] for row in rows}
    if len(registered) != len(rows) or not rows:
        raise ValueError('duplicate or missing inventory registrations')
    ctest = passing_xml(capture / 'ctest-junit.xml')
    names = [case.attrib['name'] for case in ctest]
    if len(set(names)) != len(names) or set(names) != registered:
        raise ValueError('CTest registration mismatch')
    configurations = []
    expected_files = set()
    for row in rows:
        if row['state'] != 'listed':
            continue
        filename = row['ctest'] + '.xml'
        expected_files.add(filename)
        path = capture / 'gtest-results' / filename
        cases = passing_xml(path)
        actual = [case.attrib['classname'] + '.' + case.attrib['name'] for case in cases]
        expected = [case['id'] for case in row['cases']]
        if (any(case['disabled'] for case in row['cases']) or len(set(actual)) != len(actual)
                or len(set(expected)) != len(expected) or set(actual) != set(expected)):
            raise ValueError('gtest case identity or disabled status mismatch')
        configurations.append({'ctest': row['ctest'], 'configuration': row['configuration'],
                               'binary_sha256': row['binary_sha256'], 'xml_sha256': digest(path),
                               'cases': [dict(case.attrib) for case in cases]})
    if {path.name for path in (capture / 'gtest-results').iterdir()} != expected_files:
        raise ValueError('unexpected native result files')
    return {'scope': 'Actual native math results only; no Rust port credit or complete REF-02 acceptance.',
            'state': 'validated_native_capture', 'image_id': IMAGE,
            'environment_sha256': ENVIRONMENT, 'ctest_passed': len(names),
            'gtest_configurations': len(configurations),
            'gtest_cases': sum(len(row['cases']) for row in configurations),
            'ctest_xml_sha256': digest(capture / 'ctest-junit.xml'),
            'run_record_sha256': digest(capture / 'run.json'),
            'capture_program_sha256': digest(capture / 'run.py'),
            'stdout_sha256': digest(capture / 'stdout'), 'stderr_sha256': digest(capture / 'stderr'),
            'registrations': sorted(names), 'configurations': configurations}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inventory', type=Path, required=True)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = summarize(json.loads(args.inventory.read_bytes()), args.capture)
    report['inventory_sha256'] = digest(args.inventory)
    report['summarizer_sha256'] = digest(Path(__file__))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(f"{report['ctest_passed']} native registrations; {report['gtest_cases']} cases; review pending")


if __name__ == '__main__':
    main()
