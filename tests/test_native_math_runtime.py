# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import zipfile

spec = importlib.util.spec_from_file_location('math_runtime', Path(__file__).resolve().parents[1] / 'scripts/summarize-native-math-runtime.py')
runtime = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runtime)


class NativeMathRuntimeTests(unittest.TestCase):
    def test_retained_real_capture_matches_every_recorded_original_case(self):
        root = Path(__file__).resolve().parents[1]
        reference = root / 'planning/reference'
        expected = json.loads((reference / 'math-runtime-baseline.json').read_bytes())
        with tempfile.TemporaryDirectory() as directory:
            capture = Path(directory)
            with zipfile.ZipFile(reference / 'math-runtime-capture.zip') as archive:
                self.assertEqual(len(archive.infolist()), 72)
                for member in archive.infolist():
                    name = Path(member.filename)
                    self.assertFalse(name.is_absolute())
                    self.assertNotIn('..', name.parts)
                    self.assertLessEqual(member.file_size, 256 * 1024)
                    path = capture / name
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.write_bytes(archive.read(member))
            inventory = reference / 'headless-test-inventory.json'
            actual = runtime.summarize(json.loads(inventory.read_bytes()), capture)
            actual['inventory_sha256'] = runtime.digest(inventory)
            actual['summarizer_sha256'] = runtime.digest(Path(runtime.__file__))
            self.assertEqual(actual, expected)
            self.assertEqual(actual['capture_program_sha256'], runtime.digest(root / 'reference/capture_math_runtime.py'))
            self.assertEqual((actual['ctest_passed'], actual['gtest_configurations'], actual['gtest_cases']), (135, 67, 658))

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.results = self.root / 'gtest-results'
        self.results.mkdir()
        self.run = {'image_id': runtime.IMAGE, 'environment_sha256': runtime.ENVIRONMENT,
                    'returncode': 0, 'command': ['ctest', '--test-dir', '/reference/builds/gz-math', '-j2',
                    '--output-on-failure', '--output-junit', '/out/ctest-junit.xml']}
        (self.root / 'run.json').write_text(json.dumps(self.run))
        for name in ('run.py', 'stdout', 'stderr'):
            (self.root / name).write_text('fixture')
        self.ctest = '<testsuite tests="3" failures="0"><testcase name="UNIT_Test" status="run"/><testcase name="check_UNIT_Test" status="run"/><testcase name="abi" status="run"/></testsuite>'
        self.gtest = '<testsuites tests="2" failures="0" errors="0" disabled="0"><testsuite tests="2"><testcase classname="Suite" name="A" status="run" result="completed"/><testcase classname="Suite" name="B" status="run" result="completed"/></testsuite></testsuites>'
        (self.root / 'ctest-junit.xml').write_text(self.ctest)
        self.xml = self.results / 'UNIT_Test.xml'
        self.xml.write_text(self.gtest)
        self.inventory = {'entries': [
            {'repository': 'gz-math', 'ctest': 'UNIT_Test', 'state': 'listed',
             'configuration': 'fixture', 'binary_sha256': 'fixture-binary',
             'cases': [{'id': 'Suite.A', 'disabled': False}, {'id': 'Suite.B', 'disabled': False}]},
            {'repository': 'gz-math', 'ctest': 'check_UNIT_Test', 'state': 'registered_result_guard'},
            {'repository': 'gz-math', 'ctest': 'abi', 'state': 'registered_native_wrapper'}]}

    def summarize(self):
        return runtime.summarize(self.inventory, self.root)

    def test_exact_native_results_remain_separate_from_rust_credit(self):
        report = self.summarize()
        self.assertEqual((report['ctest_passed'], report['gtest_configurations'], report['gtest_cases']), (3, 1, 2))
        self.assertEqual(report['state'], 'validated_native_capture')
        self.assertNotIn('rust_tests', report)

    def test_failed_or_different_capture_identity_rejects(self):
        for change in ({'returncode': 8}, {'returncode': False}, {'image_id': 'other'},
                       {'environment_sha256': 'other'}, {'command': ['true']}):
            with self.subTest(change=change):
                (self.root / 'run.json').write_text(json.dumps(self.run | change))
                with self.assertRaises(ValueError):
                    self.summarize()

    def test_xml_failure_skip_noncompletion_count_and_identity_reject(self):
        for original, path in ((self.gtest, self.xml), (self.ctest, self.root / 'ctest-junit.xml')):
            variants = [original.replace('failures="0"', 'failures="1"'),
                        original.replace('status="run"', 'status="notrun"'),
                        original.replace('/>', '><skipped/></testcase>', 1),
                        original.replace('tests="2"', 'tests="3"', 1) if path == self.xml
                        else original.replace('tests="3"', 'tests="4"', 1),
                        original.replace('name="A"', 'name="B"') if path == self.xml
                        else original.replace('name="abi"', 'name="other"')]
            for bad in variants:
                with self.subTest(path=path.name, xml=bad), self.assertRaises(ValueError):
                    path.write_text(bad)
                    self.summarize()
                path.write_text(original)

    def test_disabled_missing_extra_and_linked_results_reject(self):
        self.inventory['entries'][0]['cases'][0]['disabled'] = True
        with self.assertRaises(ValueError):
            self.summarize()
        self.inventory['entries'][0]['cases'][0]['disabled'] = False
        self.xml.unlink()
        with self.assertRaises(ValueError):
            self.summarize()
        outside = self.root / 'outside.xml'
        outside.write_text(self.gtest)
        self.xml.symlink_to(outside)
        with self.assertRaises(ValueError):
            self.summarize()
        self.xml.unlink()
        self.xml.write_text(self.gtest)
        (self.results / 'extra.xml').write_text(self.gtest)
        with self.assertRaises(ValueError):
            self.summarize()


if __name__ == '__main__':
    unittest.main()
