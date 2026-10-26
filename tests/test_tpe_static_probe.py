# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

spec = importlib.util.spec_from_file_location("tpe_static_probe", Path(__file__).resolve().parents[1] / "scripts/probe-tpe-static.py")
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


def completed():
    return ET.fromstring('''<testsuites tests="1" failures="0" disabled="0" errors="0">
      <testsuite name="Model" tests="1" failures="0" disabled="0" errors="0" skipped="0">
        <testcase classname="Model" name="BasicAPI" status="run" result="completed"/>
      </testsuite>
    </testsuites>''')


class OriginalCompletionTests(unittest.TestCase):
    def test_selected_completed_original_passes(self):
        probe.check_original(completed())

    def test_failure_error_disabled_and_skip_counters_fail_at_both_levels(self):
        for level in ["root", "suite"]:
            for key in ["failures", "errors", "disabled", "skipped"]:
                with self.subTest(level=level, counter=key):
                    root = completed()
                    node = root if level == "root" else root.find("testsuite")
                    node.set(key, "1")
                    with self.assertRaises(ValueError):
                        probe.check_original(root)

    def test_skipped_or_error_children_fail_even_with_zero_summary_counts(self):
        for tag in ["skipped", "error", "failure"]:
            with self.subTest(tag=tag):
                root = completed()
                ET.SubElement(root.find(".//testcase"), tag)
                with self.assertRaises(ValueError):
                    probe.check_original(root)

    def test_notrun_incomplete_and_missing_execution_attributes_fail(self):
        for key, value in [("status", "notrun"), ("result", "suppressed"), ("status", None), ("result", None)]:
            with self.subTest(key=key, value=value):
                root = completed()
                case = root.find(".//testcase")
                if value is None:
                    del case.attrib[key]
                else:
                    case.set(key, value)
                with self.assertRaises(ValueError):
                    probe.check_original(root)

    def test_wrong_identity_empty_and_duplicate_selection_fail(self):
        for target, key, value in [("testsuite", "name", "Other"), (".//testcase", "classname", "Other"), (".//testcase", "name", "Other")]:
            with self.subTest(target=target, key=key):
                root = completed()
                root.find(target).set(key, value)
                with self.assertRaises(ValueError):
                    probe.check_original(root)
        for extra in [False, True]:
            root = completed()
            suite = root.find("testsuite")
            if extra:
                suite.append(ET.fromstring(ET.tostring(suite[0])))
            else:
                suite.remove(suite[0])
            with self.subTest(extra=extra), self.assertRaises(ValueError):
                probe.check_original(root)

    def test_wrong_totals_or_root_and_missing_suite_fail(self):
        for target in ["root", "suite"]:
            root = completed()
            (root if target == "root" else root.find("testsuite")).set("tests", "0")
            with self.subTest(target=target), self.assertRaises(ValueError):
                probe.check_original(root)
        for root in [ET.Element("testsuites"), ET.Element("testsuite")]:
            with self.assertRaises(ValueError):
                probe.check_original(root)
