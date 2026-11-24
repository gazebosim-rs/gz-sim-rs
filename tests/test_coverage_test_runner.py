# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Exercise the headless Cargo runner with real libtest executables."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class CoverageTestRunner(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        scratch = tempfile.TemporaryDirectory(prefix="robot coverage ")
        cls.addClassCleanup(scratch.cleanup)
        cls.root = Path(scratch.name)
        cls.runner = cls.root / "coverage-test-runner"
        cls.target = cls.root / "target"
        cls.binary = cls.target / "debug/examples/local_robot_foundations-0123456789abcdef"
        cls.binary.parent.mkdir(parents=True)
        for source, flags, output in [
            (ROOT / "scripts/coverage-test-runner.rs", [], cls.runner),
            (ROOT / "tests/support/coverage_runner_fixture.rs", ["--test"], cls.binary),
        ]:
            subprocess.run(["rustc", "--edition=2024", "-Dwarnings", *flags, str(source),
                            "-o", str(output)], check=True)

    def run_fixture(self, *args, executable=None, failing=False):
        with tempfile.TemporaryDirectory(dir=self.root) as directory:
            result = subprocess.run([self.runner, executable or self.binary, *args], env={
                **os.environ, "CARGO_TARGET_DIR": str(self.target), "RUST_TEST_THREADS": "2",
                "GZ_RUNNER_FIXTURE_DIR": directory, "GZ_RUNNER_FIXTURE_FAIL": str(int(failing)),
            }, capture_output=True, text=True)
            pids = {p.name: p.read_text() for p in Path(directory).iterdir()}
            return result, pids

    def test_isolates_each_default_case_and_keeps_ignored_body_dormant(self):
        result, pids = self.run_fixture()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(set(pids), {"first", "second"})
        self.assertEqual(len(set(pids.values())), 2)
        self.assertIn("1 ignored", result.stdout)

    def test_failed_case_fails_suite_and_other_cases_still_run(self):
        result, pids = self.run_fixture(failing=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("second", result.stderr)
        self.assertEqual(set(pids), {"first", "second"})

    def test_user_selection_and_other_cargo_harnesses_pass_through(self):
        result, pids = self.run_fixture("--ignored")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(set(pids), {"dormant"})
        unrelated = self.binary.with_name("local_robot_foundations_helper-0123456789abcdef")
        shutil.copyfile(self.binary, unrelated)
        unrelated.chmod(0o755)
        result, pids = self.run_fixture(executable=unrelated)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(set(pids), {"first", "second"})
        self.assertEqual(len(set(pids.values())), 1)

    def test_native_cargo_uses_the_real_runner_for_its_hashed_example(self):
        package = self.root / "cargo fixture"
        source = package / "examples/local_robot_foundations.rs"
        source.parent.mkdir(parents=True)
        shutil.copyfile(ROOT / "tests/support/coverage_runner_fixture.rs", source)
        (package / "Cargo.toml").write_text(
            '[package]\nname="fixture"\nversion="0.1.0"\nedition="2024"\n')
        host = next(row[6:] for row in subprocess.check_output(["rustc", "-vV"], text=True)
                    .splitlines() if row.startswith("host: "))
        with tempfile.TemporaryDirectory(dir=self.root) as directory:
            result = subprocess.run([
                "cargo", "test", "--offline", "--all-targets", "--message-format=json",
                "--config", f"target.{host}.runner=" + json.dumps([str(self.runner)]),
            ], cwd=package, env={
                **os.environ, "CARGO_TARGET_DIR": str(package / "target"), "RUSTC_WRAPPER": "",
                "RUST_TEST_THREADS": "2", "GZ_RUNNER_FIXTURE_DIR": directory,
                "GZ_RUNNER_FIXTURE_FAIL": "0",
            }, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("3 exact libtest cases", result.stderr)
            self.assertEqual(len({p.read_text() for p in Path(directory).iterdir()}), 2)

    def test_invalid_or_empty_discovery_and_invalid_thread_count_fail_closed(self):
        target = self.root / "invalid-target"
        exe = target / "debug/examples" / self.binary.name
        exe.parent.mkdir(parents=True)
        fixture = self.root / "invalid_listing.rs"
        fixture.write_text("""
            fn main() {
                assert_eq!(std::env::args().nth(1).as_deref(), Some("--list"));
                match std::env::var("GZ_BAD_LIST").unwrap().as_str() {
                    "empty" => (),
                    "duplicate" => println!("a: test\\na: test"),
                    "noise" => println!("unknown descriptor"),
                    _ => std::process::exit(3),
                }
            }
        """)
        subprocess.run(["rustc", str(fixture), "-o", str(exe)], check=True)
        for mode, message in [
            ("empty", "has no tests"), ("duplicate", "duplicate test"),
            ("noise", "unexpected libtest listing"), ("nonzero", "listing"),
        ]:
            with self.subTest(mode=mode):
                result = subprocess.run([self.runner, exe], env={
                    **os.environ, "CARGO_TARGET_DIR": str(target), "GZ_BAD_LIST": mode,
                }, capture_output=True, text=True)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(message, result.stderr)
        result = subprocess.run([self.runner, self.binary], env={
            **os.environ, "CARGO_TARGET_DIR": str(self.target), "RUST_TEST_THREADS": "0",
        }, capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("headless test runner", result.stderr)
        result = subprocess.run([self.runner], capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("missing Cargo executable", result.stderr)
        result, pids = self.run_fixture(executable=target / "absent-executable")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("headless test runner", result.stderr)
        self.assertFalse(pids)
