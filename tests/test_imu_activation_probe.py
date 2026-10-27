# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Reject incomplete native activation evidence without predicting its results."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('imu_activation_probe', ROOT / 'scripts/probe-imu-activation.py')
PROBE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROBE)


class ObservationTests(unittest.TestCase):
    def setUp(self):
        self.value = json.loads((ROOT / 'crates/gz-sim-scene/tests/fixtures/imu_activation.json').read_text())['observations']

    def test_complete_observations_allow_unpredicted_results(self):
        for group in self.value['groups']:
            for state in [group['initial'], *group['rows']]:
                state.update(active=False, rate_hz=123.456, next_ns=-17, count=9876)
            for row in group['rows']:
                row['sampled'] = False
        PROBE.validate_observations(self.value)

    def test_missing_or_extra_groups_rows_and_fields_are_rejected(self):
        edits = [lambda v: v['groups'].pop(),
                 lambda v: v['groups'][0].pop('initial'),
                 lambda v: v['groups'][0]['rows'].pop(),
                 lambda v: v['groups'][0]['initial'].update(extra=1)]
        edits += [lambda v, k=key: v['groups'][0]['rows'][0].pop(k)
                  for key in self.value['groups'][0]['rows'][0]]
        for edit in edits:
            with self.subTest(edit=edits.index(edit)):
                value = copy.deepcopy(self.value)
                edit(value)
                with self.assertRaises((AssertionError, KeyError)):
                    PROBE.validate_observations(value)

    def test_wrong_input_echoes_and_mistyped_results_are_rejected(self):
        for key, bad in [('label', 'other'), ('now_ms', 1), ('now_ms', False),
                         ('force', True), ('force', 0), ('sampled', 0), ('active', 0),
                         ('next_ns', 0.0), ('count', 0.0), ('rate_hz', False),
                         ('rate_hz', float('nan')), ('rate_hz', float('inf'))]:
            with self.subTest(key=key, value=bad):
                value = copy.deepcopy(self.value)
                value['groups'][0]['rows'][0][key] = bad
                with self.assertRaises(AssertionError):
                    PROBE.validate_observations(value)


class NativeXmlTests(unittest.TestCase):
    def test_only_the_complete_original_test_is_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)
            path = source / 'result.xml'
            def document():
                root = ET.Element('testsuites', tests='1', failures='0', disabled='0', errors='0')
                suite = ET.SubElement(root, 'testsuite', name='Sensor_TEST', tests='1', failures='0', disabled='0', errors='0', skipped='0')
                ET.SubElement(suite, 'testcase', classname='Sensor_TEST', name='Sensor', status='run', result='completed', file=str(source / 'src/Sensor_TEST.cc'))
                return root
            def validate(root):
                ET.ElementTree(root).write(path)
                PROBE.validate_original(path, source)
            validate(document())
            edits = [lambda r: r[0].remove(r[0][0]),
                     lambda r: ET.SubElement(r[0], 'testcase', name='Other')]
            edits += [lambda r, k=key, v=value: r[0][0].set(k, v) for key, value in
                      [('classname', 'Other'), ('name', 'Other'), ('status', 'notrun'),
                       ('result', 'suppressed'), ('file', str(source / 'other.cc'))]]
            edits += [lambda r, i=level, k=key: (r if i == 0 else r[0]).set(k, '1')
                      for level in [0, 1] for key in ['failures', 'disabled', 'errors', 'skipped']]
            edits += [lambda r, k=key: ET.SubElement(r[0][0], k)
                      for key in ['failure', 'error', 'skipped']]
            for index, edit in enumerate(edits):
                with self.subTest(edit=index):
                    root = document()
                    edit(root)
                    with self.assertRaises(AssertionError):
                        validate(root)

    def test_dependency_paths_require_exact_target_and_existing_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            header = root / 'space header.hh'
            header.touch()
            dep = root / 'headers.d'
            dep.write_text('activation-probe: ' + str(header).replace(' ', '\\ ') + '\n')
            self.assertEqual(PROBE.dependencies(dep), {header})
            dep.write_text('other: ' + str(header).replace(' ', '\\ ') + '\n')
            with self.assertRaises(AssertionError):
                PROBE.dependencies(dep)
            dep.write_text('activation-probe: ' + str(root / 'missing.hh') + '\n')
            with self.assertRaises(FileNotFoundError):
                PROBE.dependencies(dep)
