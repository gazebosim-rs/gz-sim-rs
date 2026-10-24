# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("verify", Path(__file__).resolve().parents[1] / "scripts/verify.py")
verify = importlib.util.module_from_spec(spec)
spec.loader.exec_module(verify)


class CoverageGate(unittest.TestCase):
    def report(self, values):
        return {"data": [{"files": [
            {"filename": name, "summary": {
                metric: {"count": count, "covered": covered}
                for metric in ("lines", "regions")}}
            for name, count, covered in values]}]}

    def test_strict_threshold_and_nonempty_denominator(self):
        source = Path("/a/lib.rs")
        for count, covered in [(100, 95), (100, 94), (0, 0)]:
            with self.subTest(covered=covered), self.assertRaises(ValueError):
                verify.check_coverage(self.report([(str(source), count, covered)]), {"a": [source]})
        verify.check_coverage(self.report([(str(source), 100, 96)]), {"a": [source]})

    def test_missing_files_crates_and_regions_fail(self):
        with self.assertRaises(ValueError):
            verify.check_coverage(self.report([]), {})
        with self.assertRaises(ValueError):
            verify.check_coverage(self.report([]), {"a": []})
        with self.assertRaises(ValueError):
            verify.check_coverage(self.report([]), {"a": [Path("/missing.rs")]})
        report = self.report([("/a.rs", 100, 100)])
        report["data"][0]["files"][0]["summary"]["regions"]["covered"] = 95
        with self.assertRaises(ValueError):
            verify.check_coverage(report, {"a": [Path("/a.rs")]})

    def test_large_crate_cannot_mask_small_uncovered_crate(self):
        report = self.report([("/a.rs", 10000, 10000), ("/b.rs", 10, 5)])
        with self.assertRaises(ValueError):
            verify.check_coverage(report, {"a": [Path("/a.rs")], "b": [Path("/b.rs")]})

    def test_duplicate_instrumentation_fails(self):
        report = self.report([("/a.rs", 100, 100), ("/a.rs", 100, 100)])
        with self.assertRaises(ValueError):
            verify.check_coverage(report, {"a": [Path("/a.rs")]})

    def test_changed_hunks_use_new_lines_and_preserve_multiple_files(self):
        diff = """--- a/src/one.rs
+++ b/src/one.rs
@@ -3,2 +3,3 @@
@@ -8 +9 @@
--- a/src/old.rs
+++ /dev/null
@@ -1,8 +0,0 @@
--- /dev/null
+++ b/src/new.rs
@@ -0,0 +1,2 @@
"""
        self.assertEqual(verify.changed_lines(diff), {
            "src/one.rs": {3, 4, 5, 9}, "src/new.rs": {1, 2}})
        with self.assertRaises(ValueError):
            verify.changed_lines('+++ "unsupported path"\n@@ -0,0 +1 @@')

    def test_changed_coverage_is_strict_and_independent_per_crate(self):
        a, b = verify.ROOT / "a.rs", verify.ROOT / "b.rs"
        lcov = f"SF:{a}\nDA:1,3\nend_of_record\nSF:{b}\n"
        lcov += "\n".join(f"DA:{i},{int(i != 20)}" for i in range(1, 21))
        changed = {"a.rs": {1}, "b.rs": set(range(1, 21))}
        with self.assertRaisesRegex(ValueError, "b: changed executable lines 19/20"):
            verify.check_changed_coverage(lcov, changed, {"a": [a], "b": [b]})
        changed["b.rs"].remove(20)
        self.assertEqual(verify.check_changed_coverage(lcov, changed, {"a": [a], "b": [b]}), {"a": [1, 1], "b": [19, 19]})

    def test_comments_and_deletions_do_not_invent_executable_coverage(self):
        source = verify.ROOT / "a.rs"
        lcov = f"SF:{source}\nDA:3,0\nend_of_record\n"
        self.assertEqual(verify.check_changed_coverage(lcov, {"a.rs": {1, 2}}, {"a": [source]}), {"a": [0, 0]})
        with self.assertRaises(ValueError):
            verify.check_changed_coverage(lcov, {"a.rs": {3}}, {"a": [source]})

    def test_missing_or_duplicate_lcov_source_fails(self):
        source = verify.ROOT / "a.rs"
        with self.assertRaises(ValueError):
            verify.check_changed_coverage("", {}, {"a": [source]})
        with self.assertRaises(ValueError):
            verify.check_changed_coverage(f"SF:{source}\nSF:{source}\n", {}, {"a": [source]})

    def test_empty_or_duplicate_lcov_lines_cannot_hide_changed_code(self):
        source = verify.ROOT / "a.rs"
        for data in ["", "DA:1,0\nDA:1,1\n", "DA:0,1\n", "DA:1,-1\n"]:
            with self.subTest(data=data), self.assertRaises(ValueError):
                verify.check_changed_coverage(f"SF:{source}\n{data}", {"a.rs": {1}}, {"a": [source]}, {source: 1})
        self.assertEqual(verify.check_changed_coverage(f"SF:{source}\n", {}, {"a": [source]}, {source: 0}), {"a": [0, 0]})


if __name__ == "__main__":
    unittest.main()
