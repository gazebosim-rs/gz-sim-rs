# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import importlib.util
import copy
import json
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

    def guard_fixture(self):
        helper = self.directory / "check_test_ran.py"
        helper.write_text("# Nonexecuted registration fixture.\n")
        self.guard_digest = inventory.digest(helper.read_bytes())
        parent = {"name": "UNIT_fixture_TEST", "command": ["/native/gtest", "--gtest_output=xml:" + str(self.directory / "test_results/UNIT_fixture_TEST.xml")], "properties": [{"name": "WORKING_DIRECTORY", "value": str(self.directory)}]}
        guard = {"name": "check_" + parent["name"], "command": ["/usr/bin/python3", str(helper), parent["command"][1].removeprefix("--gtest_output=xml:")], "properties": parent["properties"]}
        return parent, guard

    def test_exact_result_guard_is_a_registration_without_execution_or_cases(self):
        parent, guard = self.guard_fixture()
        with patch.object(inventory, "RESULT_GUARD_SHA256", self.guard_digest), patch.object(inventory.subprocess, "run") as child:
            row = inventory.result_guard(guard, [parent, guard], {parent["name"]}, self.directory)
        child.assert_not_called()
        self.assertEqual(row["state"], "registered_result_guard")
        self.assertEqual(row["scope"], "registration_only_not_executed")
        self.assertEqual(row["guarded_ctest"], parent["name"])
        self.assertNotIn("cases", row)
        self.assertFalse(Path(guard["command"][2]).exists())

    def test_unknown_helper_interpreter_and_missing_or_ambiguous_parent_remain_gaps(self):
        parent, guard = self.guard_fixture()
        with patch.object(inventory, "RESULT_GUARD_SHA256", self.guard_digest):
            self.assertIsNone(inventory.result_guard(guard, [parent, guard], set(), self.directory))
            self.assertIsNone(inventory.result_guard(guard, [parent, parent, guard], {parent["name"]}, self.directory))
            self.assertIsNone(inventory.result_guard(guard, [parent, guard, guard], {parent["name"]}, self.directory))
            self.assertIsNone(inventory.result_guard(guard, [guard], {parent["name"]}, self.directory))
            for interpreter in ["/usr/bin/python2", "/tmp/python3"]:
                changed = guard | {"command": [interpreter, *guard["command"][1:]]}
                self.assertIsNone(inventory.result_guard(changed, [parent, changed], {parent["name"]}, self.directory))
            Path(guard["command"][1]).write_text("# changed helper\n")
            self.assertIsNone(inventory.result_guard(guard, [parent, guard], {parent["name"]}, self.directory))

    def test_changed_guard_arguments_environment_and_xml_endpoint_remain_gaps(self):
        parent, guard = self.guard_fixture()
        variants = [guard | {"command": [*guard["command"], "extra"]},
                    guard | {"command": [*guard["command"][:2], str(self.directory / "other.xml")]},
                    guard | {"properties": [*guard["properties"], {"name": "ENVIRONMENT", "value": ["PYTHONPATH=/decoy"]}]},
                    guard | {"properties": [{"name": "WORKING_DIRECTORY", "value": "/other"}]},
                    guard | {"properties": [*guard["properties"], *guard["properties"]]}]
        with patch.object(inventory, "RESULT_GUARD_SHA256", self.guard_digest):
            for changed in variants:
                with self.subTest(changed=changed):
                    self.assertIsNone(inventory.result_guard(changed, [parent, changed], {parent["name"]}, self.directory))
            for endpoint in [str(self.directory.parent / "escaped.xml"), "test_results/UNIT_fixture_TEST.xml"]:
                changed_parent = parent | {"command": [parent["command"][0], "--gtest_output=xml:" + endpoint]}
                changed_guard = guard | {"command": [*guard["command"][:2], endpoint]}
                self.assertIsNone(inventory.result_guard(changed_guard, [changed_parent, changed_guard], {parent["name"]}, self.directory))

    def compile_failure_fixture(self, repository, source=None, build=None):
        source = source or self.directory / repository
        build = build or self.directory / (repository + "-build")
        build.mkdir(exist_ok=True)
        contract = copy.deepcopy(inventory.COMPILE_FAILURES[repository])
        directory = source / contract["directory"]
        directory.mkdir(parents=True)
        helper, cmake = directory / "testrunner.bash", directory / "CMakeLists.txt"
        helper.write_text("# Nonexecuted compiler registration fixture.\n")
        cmake.write_text("# Controlled CTest registration source.\n")
        contract.update(helper_sha256=inventory.digest(helper.read_bytes()),
                        cmake_sha256=inventory.digest(cmake.read_bytes()))
        tests = []
        for target, diagnostic in contract["targets"].items():
            (directory / (target + ".cc")).write_text("// Synthetic source identity: " + target + "\n")
            tests.append({"name": "STATIC_ASSERT_" + target,
                          "command": [str(helper), "/usr/bin/cmake", "--build", ".", "--target", target, "--config", "Release"],
                          "properties": [{"name": "PASS_REGULAR_EXPRESSION", "value": [diagnostic]},
                                         {"name": "WORKING_DIRECTORY", "value": str(build)}]})
        return source, build, contract, tests

    def test_compile_failure_registrations_preserve_all_seven_diagnostics_without_execution(self):
        count = 0
        for repository in inventory.COMPILE_FAILURES:
            source, build, contract, tests = self.compile_failure_fixture(repository)
            with patch.dict(inventory.COMPILE_FAILURES, {repository: contract}), patch.object(inventory.subprocess, "run") as child:
                for test in tests:
                    row = inventory.compile_failure_registration(repository, test, tests, build, source)
                    self.assertEqual(row["state"], "registered_compile_failure")
                    self.assertEqual(row["scope"], "registration_only_not_executed")
                    self.assertEqual(row["expected_diagnostic"], test["properties"][0]["value"][0])
                    self.assertEqual(row["source_sha256"]["testrunner.bash"], contract["helper_sha256"])
                    self.assertIn(row["target"] + ".cc", row["source_sha256"])
                    self.assertNotIn("cases", row)
                    self.assertNotIn("exit_code", row)
                    count += 1
            child.assert_not_called()
        self.assertEqual(count, 7)

    def test_compile_failure_changed_commands_properties_and_duplicate_names_remain_gaps(self):
        repository = "gz-physics"
        source, build, contract, tests = self.compile_failure_fixture(repository)
        test = tests[0]
        variants = [test | {"name": "OTHER_" + test["name"]},
                    test | {"command": ["/decoy/testrunner.bash", *test["command"][1:]]},
                    test | {"command": [test["command"][0], "/decoy/cmake", *test["command"][2:]]},
                    test | {"command": [*test["command"], "--verbose"]},
                    test | {"command": [*test["command"][:-1], "Debug"]},
                    test | {"properties": []},
                    test | {"properties": [*test["properties"], {"name": "ENVIRONMENT", "value": ["CXX=/decoy"]}]},
                    test | {"properties": [{"name": "PASS_REGULAR_EXPRESSION", "value": ["different"]}, *test["properties"][1:]]}]
        with patch.dict(inventory.COMPILE_FAILURES, {repository: contract}):
            for changed in variants:
                with self.subTest(changed=changed):
                    self.assertIsNone(inventory.compile_failure_registration(repository, changed, [changed], build, source))
            self.assertIsNone(inventory.compile_failure_registration(repository, test, [test, test], build, source))
            self.assertIsNone(inventory.compile_failure_registration("unknown", test, [test], build, source))

    def test_compile_failure_changed_helper_cmake_missing_source_and_escaping_source_remain_gaps(self):
        repository = "gz-plugin"
        source, build, contract, tests = self.compile_failure_fixture(repository)
        test = tests[0]
        directory = source / contract["directory"]
        with patch.dict(inventory.COMPILE_FAILURES, {repository: contract}):
            for name in ["testrunner.bash", "CMakeLists.txt"]:
                path = directory / name
                original = path.read_bytes()
                path.write_bytes(original + b"changed")
                self.assertIsNone(inventory.compile_failure_registration(repository, test, tests, build, source))
                path.write_bytes(original)
            unit = directory / (next(iter(contract["targets"])) + ".cc")
            data = unit.read_bytes()
            unit.unlink()
            self.assertIsNone(inventory.compile_failure_registration(repository, test, tests, build, source))
            outside = self.directory / "outside.cc"
            outside.write_bytes(data)
            unit.symlink_to(outside)
            self.assertIsNone(inventory.compile_failure_registration(repository, test, tests, build, source))

    def test_collect_keeps_compile_failure_registrations_outside_listing_jobs(self):
        source, baseline = self.source_fixture("compiler-source", "source fixture\n")
        build, _, _ = self.build_fixture(source)
        repository = "gz-physics"
        source, build, contract, tests = self.compile_failure_fixture(repository, source, build)
        original_run = subprocess.run
        def child(command, **kwargs):
            if command[:1] == ["ctest"]:
                return subprocess.CompletedProcess(command, 0, stdout=json.dumps({"tests": tests}).encode(), stderr=b"")
            return original_run(command, **kwargs)
        with patch.dict(inventory.COMPILE_FAILURES, {repository: contract}), patch.object(inventory.subprocess, "run", side_effect=child):
            reference, rows, jobs = inventory.collect(repository, build, baseline, self.directory)
        self.assertEqual(reference["registered_entries"], 3)
        self.assertEqual([row["state"] for row in rows], ["registered_compile_failure"] * 3)
        self.assertEqual(jobs, [])
        self.assertTrue(all("cases" not in row for row in rows))

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

    def test_collect_keeps_result_guards_separate_from_native_listing_jobs(self):
        import json
        source, baseline = self.source_fixture("source", "registration fixture\n")
        build, binary_dir, _ = self.build_fixture(source)
        helper = self.directory / "check_test_ran.py"
        helper.write_text("# Nonexecuted guard fixture\n")
        executable = self.directory / "UNIT_fixture_TEST"
        executable.write_bytes(b"\x7fELF controlled gtest_list_tests marker")
        endpoint = str(build / "test_results/UNIT_fixture_TEST.xml")
        properties = [{"name": "WORKING_DIRECTORY", "value": str(build)}]
        parent = {"name": "UNIT_fixture_TEST", "command": [str(executable), "--gtest_output=xml:" + endpoint], "properties": properties}
        guard = {"name": "check_UNIT_fixture_TEST", "command": ["/usr/bin/python3", str(helper), endpoint], "properties": properties}
        data = json.dumps({"tests": [guard, parent]})
        (binary_dir / "ctest").write_text("#!" + sys.executable + "\nprint(" + repr(data) + ")\n")
        with patch.dict(os.environ, {"PATH": str(binary_dir) + os.pathsep + os.environ["PATH"]}), patch.object(inventory, "RESULT_GUARD_SHA256", inventory.digest(helper.read_bytes())):
            reference, rows, jobs = inventory.collect("gz-math", build, baseline, self.directory)
        self.assertEqual(reference["registered_entries"], 2)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["state"], "registered_result_guard")
        self.assertEqual(len(jobs), 1)
        self.assertEqual(jobs[0][0]["ctest"], parent["name"])
        self.assertNotIn("cases", rows[0])

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
