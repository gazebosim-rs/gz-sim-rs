# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location("native_inventory", Path(__file__).resolve().parents[1] / "scripts/inventory-native-tests.py")
inventory = importlib.util.module_from_spec(spec)
spec.loader.exec_module(inventory)


class NativeInventoryTests(unittest.TestCase):
    def test_parameterized_and_disabled_cases_keep_native_identities(self):
        cases = inventory.parse_cases("""Running main() from gtest_main.cc
Typed/0.  # TypeParam = double
  Value/first  # GetParam() = 4
  DISABLED_Second/0
DISABLED_Group.
  Other
""")
        self.assertEqual(cases, [
            {"id": "Typed/0.Value/first", "disabled": False},
            {"id": "Typed/0.DISABLED_Second/0", "disabled": True},
            {"id": "DISABLED_Group.Other", "disabled": True},
        ])

    def test_empty_malformed_and_duplicate_lists_fail_closed(self):
        for output in ["", "Suite.\n", "  Orphan\n", "Suite.\n  Case\n  Case\n", "Suite.\nOther.\n  Case\n", "Suite.\n  Case\nunexpected diagnostic\n"]:
            with self.subTest(output=output), self.assertRaises(ValueError):
                inventory.parse_cases(output)

    def test_configuration_identity_includes_directory_and_ordered_environment(self):
        command = ["/native/test", "--gtest_list_tests"]
        base = {"WORKING_DIRECTORY": "/a", "ENVIRONMENT": ["A=one", "A=two"], "ENVIRONMENT_MODIFICATION": ["B=set:first", "B=string_append:last"]}
        key = inventory.configuration_key(command, base)
        self.assertEqual(key, inventory.configuration_key(command, dict(reversed(list(base.items())))))
        for properties in [base | {"WORKING_DIRECTORY": "/b"}, base | {"ENVIRONMENT": list(reversed(base["ENVIRONMENT"]))}, base | {"ENVIRONMENT_MODIFICATION": list(reversed(base["ENVIRONMENT_MODIFICATION"]))}, base | {"DISABLED": True}]:
            self.assertNotEqual(key, inventory.configuration_key(command, properties))

    def job(self, code, properties=None, configuration="a" * 64):
        row = {"repository": "gz-math", "ctest": "../../escape", "configuration": configuration}
        return row, [sys.executable, "-c", code], properties or {}, self.directory

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)

    def test_failed_child_keeps_output_without_claiming_listed_cases(self):
        row = inventory.enumerate_job(self.job("import sys; print('Suite.\\n  Case'); print('diagnostic', file=sys.stderr); sys.exit(7)"), self.directory, os.environ.copy())
        self.assertEqual(row["state"], "enumeration_failed")
        self.assertEqual(row["exit_code"], 7)
        self.assertNotIn("cases", row)
        self.assertEqual(next(self.directory.glob("*.stdout")).read_bytes(), b"Suite.\n  Case\n")
        self.assertEqual(next(self.directory.glob("*.stderr")).read_bytes(), b"diagnostic\n")

    def test_configuration_hashes_avoid_ctest_name_path_and_output_collisions(self):
        for identity in ["a" * 64, "b" * 64]:
            row = inventory.enumerate_job(self.job("print('Suite.\\n  Case')", configuration=identity), self.directory, os.environ.copy())
            self.assertEqual(row["state"], "listed")
        self.assertEqual(len(list(self.directory.glob("*.stdout"))), 2)
        self.assertFalse((self.directory.parent / "escape.stdout").exists())

    def test_ctest_environment_then_native_cmake_modification_and_reset(self):
        properties = {"ENVIRONMENT": ["TASK_INVENTORY_VALUE=first"], "ENVIRONMENT_MODIFICATION": ["TASK_INVENTORY_VALUE=string_append:discard", "TASK_INVENTORY_VALUE=reset:", "TASK_INVENTORY_VALUE=string_append:_last"]}
        code = "import os; assert os.environ['TASK_INVENTORY_VALUE']=='first_last'; print('Suite.\\n  Case')"
        row = inventory.enumerate_job(self.job(code, properties), self.directory, os.environ.copy())
        self.assertEqual(row["state"], "listed")
        self.assertEqual(row["cases"], [{"id": "Suite.Case", "disabled": False}])

    def test_timeout_retains_partial_stdout_and_stderr(self):
        error = subprocess.TimeoutExpired(["native"], 20, output=b"partial", stderr=b"reason")
        with patch.object(inventory.subprocess, "run", side_effect=error):
            row = inventory.enumerate_job(self.job("unused"), self.directory, {})
        self.assertEqual(row["state"], "enumeration_failed")
        self.assertEqual(row["reason"], "TimeoutExpired")
        self.assertEqual(next(self.directory.glob("*.stdout")).read_bytes(), b"partial")
        self.assertEqual(next(self.directory.glob("*.stderr")).read_bytes(), b"reason")

    def test_unsupported_environment_shape_is_not_silently_ignored(self):
        with patch.object(inventory.subprocess, "run") as child:
            row = inventory.enumerate_job(self.job("unused", {"ENVIRONMENT": "A=x;B=y"}), self.directory, {})
        child.assert_not_called()
        self.assertEqual(row["state"], "enumeration_failed")

    def test_changed_executable_cannot_claim_an_old_binary_hash(self):
        executable = self.directory / "native-fixture"
        executable.write_text("#!" + sys.executable + "\nfrom pathlib import Path\nPath(__file__).write_text('changed')\nprint('Suite.\\n  Case')\n")
        executable.chmod(0o755)
        row = {"repository": "gz-math", "configuration": "c" * 64, "binary_sha256": inventory.digest(executable.read_bytes())}
        result = inventory.enumerate_job((row, [str(executable)], {}, self.directory), self.directory, os.environ.copy())
        self.assertEqual(result["state"], "enumeration_failed")
        self.assertEqual(result["reason"], "binary_changed")
        self.assertNotIn("cases", result)
        self.assertEqual(next(self.directory.glob("*.stdout")).read_bytes(), b"Suite.\n  Case\n")

    def source_fixture(self, name, contents):
        source = self.directory / name
        git_env = os.environ.copy()
        for key in subprocess.check_output(["git", "rev-parse", "--local-env-vars"], text=True).splitlines():
            git_env.pop(key, None)
        git_env.update(GIT_AUTHOR_NAME="Inventory Fixture", GIT_COMMITTER_NAME="Inventory Fixture",
                       GIT_AUTHOR_EMAIL="inventory@example.invalid", GIT_COMMITTER_EMAIL="inventory@example.invalid")
        subprocess.run(["git", "init", str(source)], env=git_env, check=True, capture_output=True)
        (source / "NOTICE").write_text(contents)
        subprocess.run(["git", "-C", str(source), "add", "NOTICE"], env=git_env, check=True, capture_output=True)
        subprocess.run(["git", "-C", str(source), "commit", "-m", "Native reference fixture"], env=git_env, check=True, capture_output=True)
        baseline = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], env=git_env, text=True).strip()
        return source, baseline

    def build_fixture(self, source):
        build = self.directory / "build"
        build.mkdir()
        (build / "CMakeCache.txt").write_text("CMAKE_HOME_DIRECTORY:INTERNAL=" + str(source) + "\n")
        binary_dir = self.directory / "bin"
        binary_dir.mkdir()
        marker = self.directory / "ctest-executed"
        fake = binary_dir / "ctest"
        fake.write_text("#!" + sys.executable + "\nfrom pathlib import Path\nPath(" + repr(str(marker)) + ").touch()\nprint('{\"tests\": []}')\n")
        fake.chmod(0o755)
        return build, binary_dir, marker

    def test_wrong_revision_or_dirty_source_cannot_execute_ctest(self):
        source, baseline = self.source_fixture("source", "original tracked fixture\n")
        build, binary_dir, marker = self.build_fixture(source)
        with patch.dict(os.environ, {"PATH": str(binary_dir) + os.pathsep + os.environ["PATH"]}):
            with self.assertRaisesRegex(ValueError, "source_revision_differs"):
                inventory.collect("gz-math", build, "0" * 40, self.directory)
            (source / "NOTICE").write_text("modified tracked source fixture\n")
            with self.assertRaises(subprocess.CalledProcessError):
                inventory.collect("gz-math", build, baseline, self.directory)
        self.assertFalse(marker.exists())


    def test_parent_git_environment_cannot_substitute_a_decoy_checkout(self):
        source, baseline = self.source_fixture("source", "native source\n")
        decoy, decoy_revision = self.source_fixture("decoy", "different source\n")
        self.assertNotEqual(baseline, decoy_revision)
        build, binary_dir, marker = self.build_fixture(source)
        hostile = {"PATH": str(binary_dir) + os.pathsep + os.environ["PATH"],
                   "GIT_DIR": str(decoy / ".git"), "GIT_WORK_TREE": str(decoy)}
        with patch.dict(os.environ, hostile):
            with self.assertRaisesRegex(ValueError, "source_revision_differs"):
                inventory.collect("gz-math", build, decoy_revision, self.directory)
            self.assertFalse(marker.exists())
            reference, _, _ = inventory.collect("gz-math", build, baseline, self.directory)
            self.assertEqual(reference["source_revision"], baseline)
            self.assertTrue(marker.exists())
            marker.unlink()
            (source / "NOTICE").write_text("dirty native source\n")
            with self.assertRaises(subprocess.CalledProcessError):
                inventory.collect("gz-math", build, baseline, self.directory)
            self.assertFalse(marker.exists())


if __name__ == "__main__":
    unittest.main()
