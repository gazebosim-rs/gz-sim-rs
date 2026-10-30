# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import copy
from contextlib import redirect_stdout
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("reference_build", ROOT / "reference/build-reference.py")
reference = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reference)


class ReferenceBuildTests(unittest.TestCase):
    def setUp(self):
        self.lock = json.loads((ROOT / "reference/headless-tpe.lock.json").read_text())
        self.baseline = json.loads((ROOT / "planning/baseline.json").read_text())

    def test_float_sources_test_disable_and_debug_profiles_are_rejected(self):
        reference.validate_lock(self.lock, self.baseline)
        for key, value in [("sha", "main"), ("sha", "0" * 40), ("url", "https://example.com/decoy")]:
            candidate = copy.deepcopy(self.lock)
            candidate["repositories"][0][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                reference.validate_lock(candidate, self.baseline)
        for key, value in [("BUILD_TESTING", "OFF"), ("CMAKE_BUILD_TYPE", "Debug"), ("CMAKE_CXX_COMPILER", "/tmp/decoy-compiler")]:
            candidate = copy.deepcopy(self.lock)
            candidate["repositories"][0]["cmake"][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                reference.validate_lock(candidate, self.baseline)

    def test_missing_duplicate_and_gui_sources_are_rejected(self):
        for change in ["missing", "duplicate", "gui"]:
            candidate = copy.deepcopy(self.lock)
            if change == "missing":
                candidate["repositories"].pop()
            elif change == "duplicate":
                candidate["repositories"].append(candidate["repositories"][0])
            else:
                candidate["repositories"][0]["name"] = "gz-gui"
            with self.subTest(change=change), self.assertRaises(ValueError):
                reference.validate_lock(candidate, self.baseline)

    def test_base_snapshot_and_package_selection_fail_closed(self):
        for key, value in [("base_image", "ubuntu:24.04"), ("ubuntu_snapshot", "latest"), ("apt_dependencies", []), ("apt_dependencies", ["--allow-unauthenticated"])]:
            candidate = copy.deepcopy(self.lock)
            candidate[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                reference.validate_lock(candidate, self.baseline)

    def test_recipe_cannot_misrepresent_its_base_snapshot_or_bootstrap(self):
        recipe = (ROOT / "reference/Dockerfile").read_text()
        bootstrap = json.loads((ROOT / "reference/ubuntu-bootstrap.json").read_text())
        reference.validate_recipe(self.lock, bootstrap, recipe)
        for altered in [recipe.replace(self.lock["base_image"], "ubuntu:24.04"), recipe.replace(self.lock["ubuntu_snapshot"], "20261025T000000Z"), recipe.replace(bootstrap["package"]["SHA256"], "0" * 64), recipe.replace(bootstrap["package"]["Filename"], "pool/decoy.deb")]:
            with self.subTest(recipe=altered[:80]), self.assertRaises(ValueError):
                reference.validate_recipe(self.lock, bootstrap, altered)

    def test_failed_command_retains_output_and_never_reports_success(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "build.log"
            shown = io.StringIO()
            with redirect_stdout(shown), self.assertRaises(subprocess.CalledProcessError) as failed:
                reference.run([sys.executable, "-c", "import sys; print('stdout evidence'); print('stderr evidence', file=sys.stderr); sys.exit(7)"], log, reference.git_environment())
            self.assertEqual(failed.exception.returncode, 7)
            for text in ["stdout evidence", "stderr evidence"]:
                self.assertIn(text, log.read_text())
                self.assertIn(text, shown.getvalue())

    def test_extra_recipe_instruction_changes_recorded_provenance(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in reference.provenance_inputs(ROOT):
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes((ROOT / name).read_bytes())
            before = reference.provenance_inputs(root)
            recipe = root / "reference/Dockerfile"
            recipe.write_text(recipe.read_text() + "\nRUN true\n")
            after = reference.provenance_inputs(root)
            self.assertNotEqual(before["reference/Dockerfile"], after["reference/Dockerfile"])
            self.assertEqual({k: v for k, v in before.items() if k != "reference/Dockerfile"}, {k: v for k, v in after.items() if k != "reference/Dockerfile"})

    def test_compiler_flag_toolchain_and_include_overrides_are_removed(self):
        overrides = {k: "decoy" for k in ["CC", "CXX", "CFLAGS", "CXXFLAGS", "CPPFLAGS", "LDFLAGS", "CMAKE_TOOLCHAIN_FILE", "CPATH", "CPLUS_INCLUDE_PATH", "LIBRARY_PATH", "COMPILER_PATH", "GCC_EXEC_PREFIX", "PKG_CONFIG_LIBDIR", "PKG_CONFIG_SYSROOT_DIR"]}
        with patch.dict(os.environ, overrides):
            env = reference.build_environment(Path("/reference/install"))
        for key in overrides:
            self.assertNotIn(key, env)
        self.assertEqual(env["CMAKE_PREFIX_PATH"], "/reference/install")

    def test_changed_inputs_cannot_be_qualified_by_a_post_build_hash(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in reference.provenance_inputs(ROOT):
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes((ROOT / name).read_bytes())
            initial = reference.provenance_inputs(root)
            reference.verify_inputs(root, initial)
            for name in initial:
                path = root / name
                saved = path.read_bytes()
                path.write_bytes(saved + b"\nchanged by build step\n")
                with self.subTest(path=name), self.assertRaises(ValueError):
                    reference.verify_inputs(root, initial)
                path.write_bytes(saved)
            self.assertEqual(reference.provenance_inputs(root), initial)

    def test_embedded_collector_requires_its_contract_and_rejects_mutation(self):
        name = "planning/reference/non-gtest-registration-contracts.json"
        initial = reference.provenance_inputs(ROOT)
        self.assertEqual(initial[name], reference.digest(ROOT / name))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for entry in initial:
                path = root / entry
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes((ROOT / entry).read_bytes())
            manifest = root / name
            saved = manifest.read_bytes()
            manifest.unlink()
            with self.assertRaises(FileNotFoundError):
                reference.provenance_inputs(root)
            manifest.write_bytes(saved)
            reference.verify_inputs(root, initial)
            changed = json.loads(saved)
            changed["contracts"][0]["command"].append("--changed")
            manifest.write_text(json.dumps(changed))
            with self.assertRaisesRegex(ValueError, "reference input changed"):
                reference.verify_inputs(root, initial)

    def test_failing_and_malformed_discovery_keep_both_raw_streams(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "ctest.json"
            for code in [3, 0]:
                command = [sys.executable, "-c", f"import sys; print('partial JSON'); print('discovery diagnostic', file=sys.stderr); sys.exit({code})"]
                error = subprocess.CalledProcessError if code else json.JSONDecodeError
                with self.subTest(exit=code), self.assertRaises(error):
                    reference.retained_listing(command, path, reference.git_environment())
                self.assertEqual(path.read_bytes(), b"partial JSON\n")
                self.assertEqual(path.with_suffix(".stderr").read_bytes(), b"discovery diagnostic\n")

    def test_apt_configuration_rejects_inherited_live_sources_and_links(self):
        script = ROOT / "reference/apt-sources.sh"
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            entries = root / "sources.list.d"
            entries.mkdir()
            (entries / "ubuntu.sources").write_text("old base source")
            (root / "sources.list").write_text("# inactive legacy comment\n")
            result = subprocess.run(["sh", str(script), str(root), "20261024T000000Z"], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            configured = (entries / "ubuntu.sources").read_text()
            self.assertIn("URIs: https://snapshot.ubuntu.com/ubuntu/20261024T000000Z/", configured)
            self.assertNotIn("archive.ubuntu.com", configured)
            for name in ["extra.list", "extra.sources"]:
                path = entries / name
                path.write_text("deb http://archive.ubuntu.com/ubuntu noble main\n")
                failed = subprocess.run(["sh", str(script), str(root), "20261024T000000Z"], capture_output=True)
                self.assertNotEqual(failed.returncode, 0)
                self.assertEqual((entries / "ubuntu.sources").read_text(), configured)
                path.unlink()
            legacy = root / "sources.list"
            legacy.write_text("deb http://archive.ubuntu.com/ubuntu noble main\n")
            self.assertNotEqual(subprocess.run(["sh", str(script), str(root), "20261024T000000Z"], capture_output=True).returncode, 0)
            legacy.unlink()
            legacy.symlink_to(entries / "ubuntu.sources")
            self.assertNotEqual(subprocess.run(["sh", str(script), str(root), "20261024T000000Z"], capture_output=True).returncode, 0)

    def test_install_manifest_hashes_files_and_rejects_escape_or_broken_links(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "lib").write_bytes(b"native")
            (root / "alias").symlink_to("lib")
            result = reference.files_under(root)
            self.assertEqual(result["alias"], {"symlink": "lib"})
            self.assertEqual(result["lib"], {"sha256": reference.digest(root / "lib")})
            (root / "alias").unlink()
            (root / "alias").symlink_to("/etc/passwd")
            with self.assertRaises(ValueError):
                reference.files_under(root)
            (root / "alias").unlink()
            (root / "alias").symlink_to("missing")
            with self.assertRaises(FileNotFoundError):
                reference.files_under(root)

    def test_real_git_source_revision_root_and_dirty_state_are_checked(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "source"
            env = reference.git_environment()
            subprocess.run(["git", "init", "-q", str(source)], env=env, check=True)
            (source / "file").write_text("pinned")
            subprocess.run(["git", "-C", str(source), "add", "file"], env=env, check=True)
            subprocess.run(["git", "-C", str(source), "-c", "user.name=Reference Test", "-c", "user.email=reference@example.invalid", "-c", "core.hooksPath=/dev/null", "commit", "-qm", "pin"], env=env, check=True)
            env = reference.git_environment()
            sha = reference.output(["git", "-C", str(source), "rev-parse", "HEAD"], env)
            reference.verify_source(source, {"sha": sha}, env)
            with self.assertRaises(ValueError):
                reference.verify_source(source, {"sha": "0" * 40}, env)
            nested = source / "nested"
            nested.mkdir()
            with self.assertRaises(ValueError):
                reference.verify_source(nested, {"sha": sha}, env)
            (source / "file").write_text("modified")
            with self.assertRaises(ValueError):
                reference.verify_source(source, {"sha": sha}, env)
            (source / "file").write_text("pinned")
            (source / "untracked").touch()
            with self.assertRaises(ValueError):
                reference.verify_source(source, {"sha": sha}, env)

            (source / "untracked").unlink()
            (source / ".git/info/exclude").write_text("ignored-payload\n")
            (source / "ignored-payload").write_text("untracked build input")
            with self.assertRaises(ValueError):
                reference.verify_source(source, {"sha": sha}, env)
