# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import importlib.util
import subprocess
import tempfile
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
-old
-old
+new
+new
+new
@@ -8 +9 @@
-old
+new
--- a/src/old.rs
+++ /dev/null
@@ -1,8 +0,0 @@
-a
-b
-c
-d
-e
-f
-g
-h
--- /dev/null
+++ b/src/new.rs
@@ -0,0 +1,2 @@
+first
+second
"""
        self.assertEqual(verify.changed_lines(diff), {
            "src/one.rs": {3, 4, 5, 9}, "src/new.rs": {1, 2}})
        with self.assertRaises(ValueError):
            verify.changed_lines('+++ "unsupported path"\n@@ -0,0 +1 @@')
        with self.assertRaisesRegex(ValueError, "unsupported quoted diff path"):
            verify.changed_lines('--- /dev/null\n+++ "unsupported path"\n@@ -0,0 +1 @@\n+x\n')

    def test_hunk_data_cannot_impersonate_headers(self):
        diff = "--- a/real.rs\n+++ b/real.rs\n@@ -1 +1 @@\n--- a/fake.rs\n+++ b/fake.rs\n@@ -8 +8 @@\n-old\n+new\n"
        self.assertEqual(verify.changed_lines(diff), {"real.rs": {1, 8}})
        context = "--- a/a.rs\n+++ b/a.rs\n@@ -1,2 +1,2 @@\n same\n-old\n+new\n\\ No newline at end of file\n"
        self.assertEqual(verify.changed_lines(context), {"a.rs": {1, 2}})

    def test_malformed_hunks_fail_closed(self):
        header = "--- a/a.rs\n+++ b/a.rs\n"
        for body in ("@@ -1 +1 @@\n", "@@ -1 +1 @@\n+x\n",
                     "@@ -0,0 +1 @@\n+x\n+extra\n",
                     "@@ -1,0 +1 @@\n-old\n+new\n",
                     "@@ nonsense\n", "@@ -1 +1 @@\n-old\n@@ -2 +2 @@\n"):
            with self.subTest(body=body), self.assertRaises(ValueError):
                verify.changed_lines(header + body)
        with self.assertRaises(ValueError):
            verify.changed_lines("@@ -0,0 +1 @@\n+x\n")

    def test_real_git_raw_string_preserves_later_hunk_path(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            def git(*args):
                return subprocess.check_output(["git", *args], cwd=root, text=True)
            git("init", "-q")
            path = root / "real.rs"
            original = ['const TEXT: &str = r#"', "before", '"#;'] + ["// gap"] * 12 + ["fn f() { old(); }"]
            path.write_text("\n".join(original) + "\n")
            git("add", "real.rs")
            modified = original.copy()
            modified[1] = "++ b/fake.rs"
            modified[-1] = "fn f() { changed(); }"
            path.write_text("\n".join(modified) + "\n")
            diff = git("diff", "--no-ext-diff", "--unified=0")
            self.assertEqual(verify.changed_lines(diff), {"real.rs": {2, 16}})

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
