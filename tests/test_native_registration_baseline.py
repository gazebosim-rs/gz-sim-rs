# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location("native_registration_runtime", Path(__file__).resolve().parents[1] / "scripts/run-native-registration-baseline.py")
runtime = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runtime)
inventory = runtime.inventory


class NativeRegistrationBaselineTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source, self.build, self.install, self.raw = [self.root / name for name in ["source", "build", "install", "raw"]]
        for directory in [self.source, self.build, self.install, self.raw]:
            directory.mkdir()
        (self.build / "CMakeCache.txt").write_text("CMAKE_INSTALL_PREFIX:PATH=" + str(self.install) + "\nGZ_TRANSPORT_ENABLE_ZENOH:BOOL=OFF\n")
        self.helper = self.source / "guard.bash"
        self.helper.write_text("printf 'No unversioned symbols found (num versioned symbols:17)\\n'\n")
        self.test = {"name": "exact_guard", "command": ["/usr/bin/bash", str(self.helper)],
                     "properties": [{"name": "WORKING_DIRECTORY", "value": str(self.build)}]}
        self.contract = {"framework": "abi_symbols", "command": ["/usr/bin/bash", "{source}/guard.bash"],
                         "properties": [{"name": "WORKING_DIRECTORY", "value": "{build}"}],
                         "interpreter_sha256": inventory.digest(Path("/usr/bin/bash").read_bytes()),
                         "files": [{"anchor": "source", "path": "guard.bash", "sha256": inventory.digest(self.helper.read_bytes())}]}
        self.contracts = {("fixture", "exact_guard"): self.contract}
        self.contract_patch = patch.object(inventory, "registration_contracts", return_value=("fixture-contract-digest", self.contracts))
        self.contract_patch.start()
        self.addCleanup(self.contract_patch.stop)

    def recognize(self, test=None, tests=None):
        test = test or self.test
        return inventory.audited_registration("fixture", test, tests or [test], self.build, self.source)

    def execute(self):
        return runtime.execute_registration("fixture", self.test, [self.test], self.build, self.source, self.raw, {})

    def test_exact_contract_is_registration_only_then_real_command_executes(self):
        with patch.object(inventory.subprocess, "run") as child:
            row = self.recognize()
        child.assert_not_called()
        self.assertEqual(row["state"], "registered_native_wrapper")
        self.assertEqual(row["scope"], "registration_only_not_executed")
        self.assertNotIn("cases", row)
        executed = self.execute()
        self.assertEqual(executed["state"], "executed_pass")
        self.assertEqual(executed["versioned_symbols"], 17)
        self.assertEqual(executed["exit_code"], 0)
        self.assertTrue(list(self.raw.glob("*.stdout")))

    def test_altered_wrapper_properties_interpreter_and_duplicate_identity_reject(self):
        variants = [self.test | {"command": self.test["command"] + ["--extra"]},
                    self.test | {"command": ["/bin/sh", str(self.helper)]},
                    self.test | {"properties": self.test["properties"] + [{"name": "DISABLED", "value": True}]},
                    self.test | {"properties": [{"name": "WORKING_DIRECTORY", "value": str(self.source)}]}]
        for test in variants:
            self.assertIsNone(self.recognize(test))
        self.assertIsNone(self.recognize(tests=[self.test, self.test]))
        self.contract["interpreter_sha256"] = "0" * 64
        self.assertIsNone(self.recognize())

    def test_changed_missing_and_symlinked_source_reject(self):
        original = self.helper.read_bytes()
        self.helper.write_bytes(original + b"# changed\n")
        self.assertIsNone(self.recognize())
        self.helper.unlink()
        self.assertIsNone(self.recognize())
        other = self.root / "outside"
        other.write_bytes(original)
        self.helper.symlink_to(other)
        self.assertIsNone(self.recognize())

    def test_execution_does_not_count_nonzero_or_false_abi_output(self):
        for code, out, err in [(1, b"No unversioned symbols found (num versioned symbols:17)\n", b""),
                               (0, b"No unversioned symbols found (num versioned symbols:0)\n", b""),
                               (0, b"", b""), (0, b"No unversioned symbols found (num versioned symbols:17)\n", b"warning")]:
            with patch.object(runtime.subprocess, "run", return_value=subprocess.CompletedProcess([], code, out, err)):
                self.assertEqual(self.execute()["state"], "execution_failed")

    def xml_contract(self):
        self.contract.update(framework="cmake_policy_xml", result_xml="{build}/result.xml",
                             case_class_suffix="native", case_name="original")
        return self.build / "result.xml"

    def test_xml_requires_exact_case_and_no_failure_skip_or_missing_results(self):
        self.xml_contract()
        good = b'<testsuite tests="1" failures="0" errors="0"><testcase classname="native" name="original"/></testsuite>'
        self.assertEqual(runtime.validate_xml(good, self.contract)["tests"], 1)
        for bad in [good.replace(b'failures="0"', b'failures="1"'),
                    good.replace(b'name="original"', b'name="changed"'),
                    good.replace(b'/>', b'><skipped/></testcase>'),
                    good.replace(b'/>', b' result="suppressed"/>'),
                    b'<testsuite tests="0"/>', b'<not-xml>']:
            with self.assertRaises((ValueError, runtime.ET.ParseError)):
                runtime.validate_xml(bad, self.contract)
        with patch.object(runtime.subprocess, "run", return_value=subprocess.CompletedProcess([], 0, b"", b"")):
            self.assertEqual(self.execute()["state"], "execution_failed")

    def test_stale_xml_is_not_deleted_and_command_never_runs(self):
        path = self.xml_contract()
        path.write_text('<testsuite tests="1"><testcase classname="native" name="original"/></testsuite>')
        old = path.read_bytes()
        with patch.object(runtime.subprocess, "run") as child:
            result = self.execute()
        child.assert_not_called()
        self.assertEqual(result["reason"], "stale XML endpoint")
        self.assertEqual(path.read_bytes(), old)

    def test_fresh_xml_and_post_execution_input_mutation(self):
        path = self.xml_contract()
        def run(command, **kwargs):
            self.assertEqual(command, self.test["command"])
            self.assertEqual(kwargs["cwd"], str(self.build))
            path.write_text('<testsuite tests="1"><testcase classname="native" name="original"/></testsuite>')
            return subprocess.CompletedProcess(command, 0, b"native output\n", b"")
        with patch.object(runtime.subprocess, "run", side_effect=run):
            self.assertEqual(self.execute()["state"], "executed_pass")
        path.unlink()
        def mutate(command, **kwargs):
            result = run(command, **kwargs)
            self.helper.write_text("changed during execution")
            return result
        with patch.object(runtime.subprocess, "run", side_effect=mutate):
            self.assertEqual(self.execute()["reason"], "registration inputs changed during execution")

    def test_only_exact_source_flags_cache_binary_and_empty_output_gate_zenoh(self):
        binary = self.build / "node"
        binary.write_bytes(b"fixture binary")
        self.test["command"] = [str(binary)]
        self.contract.pop("interpreter_sha256")
        self.contract.update(framework="profile_blocked_zenoh", command=["{build}/node"],
                             binary_sha256=inventory.digest(binary.read_bytes()),
                             cache_flag="GZ_TRANSPORT_ENABLE_ZENOH:BOOL=OFF", guarded_cases=13,
                             empty_stdout_sha256=inventory.digest(b"Running main() from gtest_main.cc\n"))
        for filename, contents in [("flags.make", "CXX_DEFINES = -Dordinary\n"), ("config.hh", "/* #undef HAVE_ZENOH */\n")]:
            path = self.build / filename
            path.write_text(contents)
            self.contract["files"].append({"anchor": "build", "path": filename, "sha256": inventory.digest(path.read_bytes())})
        for item in self.contract["files"]:
            path = (self.source if item["anchor"] == "source" else self.build) / item["path"]
            original = path.read_bytes()
            path.write_bytes(original + b"#define HAVE_ZENOH\n")
            self.assertIsNone(self.recognize())
            path.write_bytes(original)
        profile = self.recognize()
        self.assertIn("_known_profile_gate", profile)
        row = {"repository": "fixture", "ctest": "exact_guard", "configuration": "key",
               "binary_sha256": self.contract["binary_sha256"], **profile,
               "_profile_context": ("fixture", self.test, [self.test], self.build, self.source)}
        command = [str(binary), "--gtest_list_tests"]
        job = (row, command, {}, str(self.build))
        with patch.object(inventory.subprocess, "run", return_value=subprocess.CompletedProcess(command, 0, b"Running main() from gtest_main.cc\n", b"")):
            blocked = inventory.enumerate_job(job, self.raw, {})
            self.assertEqual(blocked["state"], "profile_blocked")
            ordinary = (row | {"_known_profile_gate": None}, command, {}, str(self.build))
            self.assertEqual(inventory.enumerate_job(ordinary, self.raw, {})["state"], "enumeration_failed")
        for output, error in [(b"", b""), (b"Running main() from gtest_main.cc\n", b"warning")]:
            with patch.object(inventory.subprocess, "run", return_value=subprocess.CompletedProcess(command, 0, output, error)):
                self.assertEqual(inventory.enumerate_job(job, self.raw, {})["state"], "enumeration_failed")
        def change_flags(command, **kwargs):
            (self.build / "flags.make").write_text("-DHAVE_ZENOH")
            return subprocess.CompletedProcess(command, 0, b"Running main() from gtest_main.cc\n", b"")
        original_flags = (self.build / "flags.make").read_bytes()
        with patch.object(inventory.subprocess, "run", side_effect=change_flags):
            self.assertEqual(inventory.enumerate_job(job, self.raw, {})["state"], "enumeration_failed")
        (self.build / "flags.make").write_bytes(original_flags)
        cache = self.build / "CMakeCache.txt"
        old = cache.read_text()
        cache.write_text(old.replace("ZENOH:BOOL=OFF", "ZENOH:BOOL=ON"))
        self.assertIsNone(self.recognize())
        cache.write_text(old)
        binary.write_bytes(b"different binary")
        self.assertIsNone(self.recognize())


if __name__ == "__main__":
    unittest.main()
