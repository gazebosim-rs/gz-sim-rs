# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import importlib.util
import subprocess
import tempfile
from pathlib import Path
import unittest
from unittest import mock

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

    def test_hunk_text_separators_remain_one_git_line(self):
        for separator in ("\v", "\f", "\r", "\x85", "\u2028", "\u2029"):
            with self.subTest(separator=repr(separator)):
                diff = ("--- a/real.rs\n+++ b/real.rs\n@@ -1 +1 @@\n-old\n"
                        "+data" + separator + "diff --git a/fake.rs b/fake.rs" + separator
                        + "--- a/fake.rs" + separator + "+++ b/fake.rs\n"
                        "@@ -8 +8 @@\n-old\n+new\n")
                self.assertEqual(verify.changed_lines(diff), {"real.rs": {1, 8}})

    def test_separator_data_cannot_hide_later_uncovered_executable_line(self):
        diff = ("--- a/real.rs\n+++ b/real.rs\n@@ -1 +1 @@\n-old\n"
                "+data\u2028diff --git a/fake.rs b/fake.rs\u2028--- a/fake.rs\u2028+++ b/fake.rs\n"
                "@@ -8 +8 @@\n-old\n+new\n")
        source = verify.ROOT / "real.rs"
        lcov = f"SF:{source}\nDA:1,1\nDA:8,0\nend_of_record\n"
        with self.assertRaisesRegex(ValueError, "changed executable lines 1/2"):
            verify.check_changed_coverage(lcov, verify.changed_lines(diff), {"real": [source]})

    def test_source_line_numbers_use_only_lf_and_preserve_exact_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "a.rs"
            for data, expected in [(b"", set()), (b"x", {1}), (b"x\n", {1}),
                                   (b"x\r\ny\r", {1, 2}),
                                   ("// a\u2028b\nfn f() {}\n".encode(), {1, 2})]:
                path.write_bytes(data)
                self.assertEqual(verify.source_line_numbers(path), expected)

    def test_changed_sources_reads_git_bytes_without_newline_conversion(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            def git(*args):
                return subprocess.check_output(["git", *args], cwd=root)
            git("init", "-q")
            source = root / "real.rs"
            original = ['const TEXT: &str = r#"', 'before', '"#;'] + ['// gap'] * 12 + ['fn f() { old(); }']
            source.write_text("\n".join(original) + "\n")
            git("add", "real.rs")
            git("-c", "user.name=Test", "-c", "user.email=test@example.invalid", "commit", "-qm", "baseline")
            modified = original.copy()
            modified[1] = "before\u2028diff --git a/fake.rs b/fake.rs\u2028--- a/fake.rs\u2028+++ b/fake.rs"
            modified[-1] = 'fn f() { changed(); }'
            source.write_bytes(("\r\n".join(modified) + "\r\n").encode())
            # Disable Git newline conversion as well as Python's text-mode conversion.
            git("config", "core.autocrlf", "false")
            git("add", "real.rs")
            expected = set(range(1, 17))
            with mock.patch.object(verify, "ROOT", root), mock.patch.dict("os.environ", {}, clear=False):
                with mock.patch.object(verify, "changed_lines", wraps=verify.changed_lines) as parsed:
                    self.assertEqual(verify.changed_sources({}, index=True), {"real.rs": expected})
                    self.assertIn("\r\n", parsed.call_args.args[0])
                git("-c", "user.name=Test", "-c", "user.email=test@example.invalid", "commit", "-qm", "candidate")
                with mock.patch.dict("os.environ", {"GZ_COVERAGE_BASE": git("rev-parse", "HEAD^").decode().strip()}):
                    self.assertEqual(verify.changed_sources({}), {"real.rs": expected})

    def test_untracked_source_path_bytes_are_not_newline_normalized(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            def git(*args):
                return subprocess.check_output(["git", *args], cwd=root)
            git("init", "-q")
            for value in ("first", "second"):
                (root / "readme").write_text(value)
                git("add", "readme")
                git("-c", "user.name=Test", "-c", "user.email=test@example.invalid", "commit", "-qm", value)
            source = root / "src/a\rb.rs"
            source.parent.mkdir()
            source.write_bytes(b"// header\nfn extra() {}\n")
            with mock.patch.object(verify, "ROOT", root), mock.patch.dict("os.environ", {"GZ_COVERAGE_BASE": ""}):
                self.assertEqual(verify.changed_sources({"crate": [source]}), {"src/a\rb.rs": {1, 2}})

    def test_lcov_source_names_preserve_non_lf_separators(self):
        for separator in ("\r", "\u2028", "\u2029"):
            source = verify.ROOT / f"a{separator}b.rs"
            lcov = f"SF:{source}\nDA:1,1\nend_of_record\n"
            changed = {str(source.relative_to(verify.ROOT)): {1}}
            self.assertEqual(verify.check_changed_coverage(lcov, changed, {"crate": [source]}), {"crate": [1, 1]})

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
