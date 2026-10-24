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


if __name__ == "__main__":
    unittest.main()
