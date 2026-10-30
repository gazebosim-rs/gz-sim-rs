# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

spec = importlib.util.spec_from_file_location("feature_verify", Path(__file__).resolve().parents[1] / "scripts/verify.py")
verify = importlib.util.module_from_spec(spec)
spec.loader.exec_module(verify)


def report(crates, covered=100):
    return {"data": [{"files": [
        {"filename": str(path), "summary": {
            metric: {"count": 100, "covered": covered} for metric in ("lines", "regions")}}
        for paths in crates.values() for path in paths]}]}


def sources():
    return {name: [Path("/virtual") / name / "src/lib.rs"]
            for name in verify.RENDER_PACKAGES | {"gz-sim-core", "gz-sim-cli"}}


class FeatureProfiles(unittest.TestCase):
    def test_explicit_test_support_ownership_cannot_hide_production(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            support_root = root / "tests/support/render-fixtures"
            (support_root / "src/bin").mkdir(parents=True)
            (support_root / "tests").mkdir()
            helper = support_root / "src/bin/helper.rs"
            test = support_root / "tests/behavior.rs"
            helper.write_text("// helper")
            test.write_text("// test")
            prod_root = root / "crates/prod"
            (prod_root / "src").mkdir(parents=True)
            lib = prod_root / "src/lib.rs"
            lib.write_text("// production")
            support = {"id": "support", "name": verify.TEST_SUPPORT_PACKAGE,
                       "manifest_path": str(support_root / "Cargo.toml"), "publish": [],
                       "metadata": {"gz-sim": {"role": "test-support"}}, "dependencies": [],
                       "targets": [{"kind": ["bin"], "src_path": str(helper)},
                                   {"kind": ["test"], "src_path": str(test)}]}
            prod = {"id": "prod", "name": "prod", "manifest_path": str(prod_root / "Cargo.toml"),
                    "dependencies": [], "targets": [{"kind": ["lib"], "src_path": str(lib)}]}
            data = {"workspace_members": ["support", "prod"], "packages": [support, prod]}
            with patch.object(verify, "ROOT", root), patch.object(verify, "run", return_value=SimpleNamespace(stdout=json.dumps(data))):
                verify.validate_test_support(data)
                self.assertEqual(verify.production_sources(), {"prod": [lib]})
                prod["dependencies"] = [{"name": verify.TEST_SUPPORT_PACKAGE}]
                with self.assertRaisesRegex(ValueError, "depends on test-support"):
                    verify.validate_test_support(data)
                prod["dependencies"] = []
                support["publish"] = None
                with self.assertRaisesRegex(ValueError, "ownership"):
                    verify.validate_test_support(data)
                support["publish"] = []
                support["metadata"]["gz-sim"]["role"] = "production"
                with self.assertRaisesRegex(ValueError, "ownership"):
                    verify.validate_test_support(data)
                support["metadata"]["gz-sim"]["role"] = "test-support"
                support["manifest_path"] = str(root / "elsewhere/Cargo.toml")
                with self.assertRaisesRegex(ValueError, "ownership"):
                    verify.validate_test_support(data)
                support["manifest_path"] = str(support_root / "Cargo.toml")
                support["targets"].append({"kind": ["lib"], "src_path": str(helper)})
                with self.assertRaisesRegex(ValueError, "targets"):
                    verify.validate_test_support(data)
                support["targets"].pop()
                support["targets"][0]["src_path"] = str(root / "elsewhere/helper.rs")
                with self.assertRaisesRegex(ValueError, "targets"):
                    verify.validate_test_support(data)
                with self.assertRaisesRegex(ValueError, "missing or duplicate"):
                    verify.validate_test_support({"workspace_members": ["prod"], "packages": [prod]})

    def test_production_target_outside_src_is_measured_even_under_tests(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "src").mkdir()
            (root / "tests").mkdir()
            lib = root / "src/lib.rs"
            binary = root / "tests/helper.rs"
            test = root / "tests/behavior.rs"
            for path in (lib, binary, test):
                path.write_text("// fixture")
            package = {"id": "one", "name": "crate", "manifest_path": str(root / "Cargo.toml"),
                       "targets": [{"kind": ["lib"], "src_path": str(lib)},
                                   {"kind": ["bin"], "src_path": str(binary)},
                                   {"kind": ["test"], "src_path": str(test)}]}
            data = {"workspace_members": ["one"], "packages": [package]}
            with patch.object(verify, "run", return_value=SimpleNamespace(stdout=json.dumps(data))):
                found = verify.production_sources()
            self.assertEqual(set(found["crate"]), {lib, binary})
            package["targets"][1]["src_path"] = str(root.parent / "outside.rs")
            with patch.object(verify, "run", return_value=SimpleNamespace(stdout=json.dumps(data))):
                with self.assertRaisesRegex(ValueError, "outside package root"):
                    verify.production_sources()

    def test_checked_in_profile_contract_rejects_missing_or_weakened_profiles(self):
        contract = verify.load_profile_contract()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            with patch.object(verify, "ROOT", root):
                with self.assertRaisesRegex(ValueError, "missing or unsafe"):
                    verify.load_profile_contract()
                path = root / "planning/coverage-profiles.json"
                path.parent.mkdir()
                path.write_text(json.dumps(contract))
                self.assertEqual(verify.load_profile_contract(), contract)
                weakened = json.loads(json.dumps(contract))
                weakened["excluded_sources"] = ["crates/gz-renderer/src/gpu.rs"]
                path.write_text(json.dumps(weakened))
                with self.assertRaisesRegex(ValueError, "invalid coverage"):
                    verify.load_profile_contract()
                weakened = json.loads(json.dumps(contract))
                del weakened["profiles"]["renderer"]
                path.write_text(json.dumps(weakened))
                with self.assertRaisesRegex(ValueError, "invalid coverage"):
                    verify.load_profile_contract()

    def test_profiles_cover_exact_members_and_preserve_unified_core(self):
        crates = sources()
        plan = verify.profile_plan(crates)
        self.assertEqual(set(plan), {"headless", "renderer"})
        self.assertEqual(set(plan["headless"][0]), {"gz-sim-core", "gz-sim-cli"})
        self.assertEqual(set(plan["renderer"][0]), verify.RENDER_PACKAGES | {"gz-sim-core"})
        self.assertEqual(set(plan["headless"][0]) | set(plan["renderer"][0]), set(crates))
        self.assertIn("gz-renderer/gpu", plan["renderer"][1])
        self.assertEqual(plan["headless"][1].count("--exclude"), len(verify.RENDER_PACKAGES) + 1)
        self.assertIn(verify.TEST_SUPPORT_PACKAGE, plan["renderer"][1])

    def test_missing_package_empty_source_and_duplicate_ownership_fail(self):
        crates = sources()
        del crates["gz-render-worker"]
        with self.assertRaisesRegex(ValueError, "packages missing"):
            verify.profile_plan(crates)
        crates = sources()
        crates["gz-render-unlisted"] = [Path("/virtual/unlisted/src/lib.rs")]
        with self.assertRaisesRegex(ValueError, "undeclared renderer"):
            verify.profile_plan(crates)
        crates = sources()
        crates["gz-renderer"] = []
        with self.assertRaisesRegex(ValueError, "empty production"):
            verify.profile_plan(crates)
        crates = sources()
        crates["gz-renderer"] = crates["gz-sim-core"]
        with self.assertRaisesRegex(ValueError, "multiple owners"):
            verify.profile_plan(crates)

    def test_missing_profile_and_inactive_or_exactly_95_crate_fail(self):
        plan = verify.profile_plan(sources())
        reports = {name: report(crates) for name, (crates, _) in plan.items()}
        verify.check_profile_reports(plan, reports)
        with self.assertRaisesRegex(ValueError, "coverage profiles"):
            verify.check_profile_reports(plan, {"headless": reports["headless"]})
        for metric in ("lines", "regions"):
            bad = {name: report(crates) for name, (crates, _) in plan.items()}
            file = bad["renderer"]["data"][0]["files"][0]
            file["summary"][metric]["covered"] = 95
            with self.subTest(metric=metric), self.assertRaises(ValueError):
                verify.check_profile_reports(plan, bad)
        bad = {name: report(crates) for name, (crates, _) in plan.items()}
        bad["renderer"]["data"][0]["files"].clear()
        with self.assertRaisesRegex(ValueError, "uninstrumented"):
            verify.check_profile_reports(plan, bad)

    def test_zero_executable_classification_is_narrow_and_requires_gpu(self):
        facade = (verify.ROOT / "crates/gz-renderer/src/lib.rs").resolve()
        gpu = (verify.ROOT / "crates/gz-renderer/src/gpu.rs").resolve()
        crates = {"gz-renderer": [facade, gpu]}
        gpu_report = report({"gz-renderer": [gpu]})
        with self.assertRaisesRegex(ValueError, "uninstrumented"):
            verify.check_coverage(gpu_report, crates)
        self.assertEqual(verify.check_coverage(gpu_report, crates, {facade})["gz-renderer"]["lines"], [100, 100])
        with self.assertRaisesRegex(ValueError, "unrecognized zero-executable"):
            verify.check_coverage(gpu_report, crates, {gpu})
        with self.assertRaisesRegex(ValueError, "inactive"):
            verify.check_coverage(report({}), crates, {facade})
        contradictory = report({"gz-renderer": [facade, gpu]})
        with self.assertRaisesRegex(ValueError, "has executable regions"):
            verify.check_coverage(contradictory, crates, {facade})
        lcov = f"SF:{gpu}\nDA:1,1\nend_of_record\n"
        changed = {str(facade.relative_to(verify.ROOT)): [1]}
        self.assertEqual(verify.check_changed_coverage(lcov, changed, crates, {gpu: 100}, {facade}), {"gz-renderer": [0, 0]})
        with self.assertRaisesRegex(ValueError, "missing LCOV"):
            verify.check_changed_coverage(lcov, changed, crates, {gpu: 100})
        with self.assertRaisesRegex(ValueError, "has LCOV lines"):
            verify.check_changed_coverage(lcov + f"SF:{facade}\nDA:1,1\nend_of_record\n", changed, crates, {gpu: 100}, {facade})

    def test_gpu_implementation_must_exist_and_have_executable_regions(self):
        gpu = (verify.ROOT / "crates/gz-renderer/src/gpu.rs").resolve()
        good = report({"renderer": [gpu]})
        verify.check_gpu_source(good)
        with self.assertRaisesRegex(ValueError, "inactive"):
            verify.check_gpu_source(report({}))
        for metric in ("lines", "regions"):
            bad = report({"renderer": [gpu]})
            bad["data"][0]["files"][0]["summary"][metric]["count"] = 0
            with self.subTest(metric=metric), self.assertRaisesRegex(ValueError, "inactive"):
                verify.check_gpu_source(bad)

    def test_toolchain_override_and_wrong_release_are_rejected(self):
        verify.require_toolchain("rustc 1.97.1\nrelease: 1.97.1\n", {})
        with self.assertRaisesRegex(ValueError, "Rust 1.97.1"):
            verify.require_toolchain("rustc 1.95.0\nrelease: 1.95.0\n", {})
        for key in ("RUSTC", "CARGO_BUILD_RUSTC", "RUSTC_WRAPPER"):
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, "compiler override"):
                verify.require_toolchain("release: 1.97.1\n", {key: "/other/compiler"})

    def test_default_workspace_and_cli_gpu_dependencies_fail(self):
        def metadata(default):
            return json.dumps({"workspace_default_members": default,
                               "packages": [{"name": "gz-renderer", "id": "renderer"},
                                            {"name": "gz-sim-cli", "id": "cli"}]})
        with patch.object(verify, "run", side_effect=[SimpleNamespace(stdout=metadata(["renderer"]))]):
            with self.assertRaisesRegex(ValueError, "default workspace"):
                verify.check_headless_dependencies()
        with patch.object(verify, "run", side_effect=[SimpleNamespace(stdout=metadata(["cli"])),
                                                       SimpleNamespace(stdout="gz-sim-cli v0.1.0\nwgpu v30\n")]):
            with self.assertRaisesRegex(ValueError, "rendering dependencies"):
                verify.check_headless_dependencies()
        with patch.object(verify, "run", side_effect=[SimpleNamespace(stdout=metadata(["cli"])),
                                                       SimpleNamespace(stdout="gz-sim-cli v0.1.0\n"),
                                                       SimpleNamespace(stdout="gz-sim-server v0.1.0\nwgpu-hal v30\n")]):
            with self.assertRaisesRegex(ValueError, "headless workspace"):
                verify.check_headless_dependencies()
        with patch.object(verify, "run", side_effect=[SimpleNamespace(stdout=metadata(["cli"])),
                                                       SimpleNamespace(stdout="gz-sim-cli v0.1.0\n"),
                                                       SimpleNamespace(stdout="gz-sim-server v0.1.0\nbevy_render v0.20\n")]):
            with self.assertRaisesRegex(ValueError, "headless workspace"):
                verify.check_headless_dependencies()
        with patch.object(verify, "run", side_effect=[SimpleNamespace(stdout=metadata(["cli"])),
                                                       SimpleNamespace(stdout="gz-sim-cli v0.1.0\nbevy_ecs v0.20\n"),
                                                       SimpleNamespace(stdout="gz-sim-server v0.1.0\nbevy_ecs v0.20\nbevy_asset v0.20\n")]):
            verify.check_headless_dependencies()

    def test_profile_without_raw_evidence_fails_and_preserves_log(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / "src/lib.rs"
            source.parent.mkdir()
            source.write_text("pub fn answer() -> i32 { 42 }\n")
            output = root / "profile"
            result = SimpleNamespace(returncode=0, stdout=json.dumps({"reason": "compiler-artifact", "executable": "/tmp/probe"}), stderr="", args=["cargo"])
            with patch.object(verify, "ROOT", root), patch.object(verify.subprocess, "run", return_value=result), patch.object(verify, "run", return_value=SimpleNamespace(stdout="rustc")):
                with self.assertRaisesRegex(ValueError, "no raw coverage profiles"):
                    verify.profile_coverage("headless", {"crate": [source]}, ["cargo"], output, {}, root / "wrapper", root, "rustc")
            self.assertTrue((output / "test-output.log").is_file())

    def test_renderer_requires_worker_binary_and_its_own_raw_profile(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / "src/lib.rs"
            source.parent.mkdir()
            source.write_text("original")
            binary = root / "target/coverage/debug/gz-render-worker"
            binary.parent.mkdir(parents=True)
            binary.write_bytes(b"binary")
            def fake_cargo(*args, **kwargs):
                env = kwargs["env"]
                raw = Path(env["LLVM_PROFILE_FILE"]).parent
                (raw / "test.profraw").write_bytes(b"raw")
                item = {"reason": "compiler-artifact", "executable": str(binary),
                        "target": {"name": "gz-render-worker", "kind": ["bin"], "src_path": str(root / "crates/gz-render-worker/src/main.rs")},
                        "profile": {"test": False}}
                return SimpleNamespace(returncode=0, stdout=json.dumps(item), stderr="", args=["cargo"])
            with patch.object(verify, "ROOT", root), patch.object(verify.subprocess, "run", side_effect=fake_cargo), patch.object(verify, "run", return_value=SimpleNamespace(stdout="rustc")):
                with self.assertRaisesRegex(ValueError, "worker emitted no raw profile"):
                    verify.profile_coverage("renderer", {"crate": [source]}, ["cargo"], root / "profile", {}, root / "wrapper", root, "rustc")

    def test_renderer_requires_worker_artifact_from_its_feature_graph(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / "src/lib.rs"
            source.parent.mkdir()
            source.write_text("original")
            result = SimpleNamespace(returncode=0, stdout=json.dumps({"reason": "compiler-artifact", "executable": "/tmp/not-the-worker", "target": {"name": "other", "kind": ["bin"]}, "profile": {"test": False}}), stderr="", args=["cargo"])
            with patch.object(verify, "ROOT", root), patch.object(verify.subprocess, "run", return_value=result):
                with self.assertRaisesRegex(ValueError, "worker binary absent"):
                    verify.profile_coverage("renderer", {"crate": [source]}, ["cargo"], root / "profile", {}, root / "wrapper", root, "rustc")

    def test_auditor_source_change_during_renderer_profile_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / "src/lib.rs"
            source.parent.mkdir()
            source.write_text("production")
            audit = root / "tests/support/render-fixtures/src/bin/source_audit.rs"
            audit.parent.mkdir(parents=True)
            audit.write_text("original auditor")
            binary = root / "target/coverage/debug/gz-render-worker"
            binary.parent.mkdir(parents=True)
            binary.write_bytes(b"worker")
            def fake_cargo(*args, **kwargs):
                audit.write_text("changed auditor")
                item = {"reason": "compiler-artifact", "executable": str(binary),
                        "target": {"name": "gz-render-worker", "kind": ["bin"],
                                   "src_path": str(root / "crates/gz-render-worker/src/main.rs")},
                        "profile": {"test": False}}
                return SimpleNamespace(returncode=0, stdout=json.dumps(item), stderr="", args=["cargo"])
            with patch.object(verify, "ROOT", root), patch.object(verify.subprocess, "run", side_effect=fake_cargo), patch.object(verify, "run", return_value=SimpleNamespace(stdout="rustc")):
                with self.assertRaisesRegex(ValueError, "auditor changed"):
                    verify.profile_coverage("renderer", {"crate": [source]}, ["cargo"], root / "profile", {}, root / "wrapper", root, "rustc")

    def test_compiler_change_during_profile_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / "src/lib.rs"
            source.parent.mkdir()
            source.write_text("original")
            result = SimpleNamespace(returncode=0, stdout=json.dumps({"reason": "compiler-artifact", "executable": "/tmp/probe"}), stderr="", args=["cargo"])
            with patch.object(verify, "ROOT", root), patch.object(verify.subprocess, "run", return_value=result), patch.object(verify, "run", return_value=SimpleNamespace(stdout="changed rustc")):
                with self.assertRaisesRegex(ValueError, "compiler changed"):
                    verify.profile_coverage("headless", {"crate": [source]}, ["cargo"], root / "profile", {}, root / "wrapper", root, "rustc")

    def test_source_mutation_during_profile_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / "src/lib.rs"
            source.parent.mkdir()
            source.write_text("original")
            output = root / "profile"
            def fake_cargo(*args, **kwargs):
                source.write_text("changed")
                (output / "raw" / "one.profraw").write_bytes(b"raw")
                return SimpleNamespace(returncode=0, stdout=json.dumps({"reason": "compiler-artifact", "executable": "/tmp/probe"}), stderr="", args=["cargo"])
            with patch.object(verify, "ROOT", root), patch.object(verify.subprocess, "run", side_effect=fake_cargo), patch.object(verify, "run", return_value=SimpleNamespace(stdout="rustc")):
                with self.assertRaisesRegex(ValueError, "sources changed"):
                    verify.profile_coverage("headless", {"crate": [source]}, ["cargo"], output, {}, root / "wrapper", root, "rustc")

    def test_raw_profiles_are_retained_independently_and_missing_is_partial(self):
        with tempfile.TemporaryDirectory() as tmp:
            candidate = Path(tmp) / "candidate"
            coverage = candidate / "artifacts/coverage"
            output = Path(tmp) / "out"
            output.mkdir()
            coverage.mkdir(parents=True)
            (coverage / "profile-index.json").write_text(json.dumps({"run": "profiles-test", "profiles": ["headless", "renderer"]}))
            for profile in ("headless", "renderer"):
                base = coverage / "profiles-test" / profile
                (base / "raw").mkdir(parents=True)
                for name in ("test-output.log", "coverage.json", "coverage.lcov", "summary.json", "evidence.json", "merged.profdata"):
                    (base / name).write_text(profile)
                (base / "raw" / "one.profraw").write_text(profile)
                if profile == "renderer":
                    (base / "facade-audit.json").write_text("proof")
            hashes, missing = {}, []
            verify.retain_profile_evidence(candidate, output, hashes, missing)
            self.assertEqual(missing, [])
            self.assertEqual((output / "profiles-test/headless/raw/one.profraw").read_text(), "headless")
            self.assertEqual((output / "profiles-test/renderer/raw/one.profraw").read_text(), "renderer")
            (coverage / "profiles-test/renderer/raw/one.profraw").unlink()
            hashes, missing = {}, []
            verify.retain_profile_evidence(candidate, output, hashes, missing)
            self.assertIn("profiles-test/renderer/raw/*.profraw", missing)

    def test_successful_stage_requires_profile_index_when_contract_exists(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            candidate = root / "candidate"
            coverage = candidate / "artifacts/coverage"
            coverage.mkdir(parents=True)
            planning = candidate / "planning"
            planning.mkdir()
            (planning / "coverage-profiles.json").write_text("{}")
            for name in ("test-output.log", "summary.json", "evidence.json"):
                (coverage / name).write_text("legacy")
            with patch.object(verify, "ROOT", root):
                with self.assertRaisesRegex(ValueError, "profile-index.json"):
                    verify.retain_staged_evidence("a" * 40, candidate, "passed")
            output = next((root / "artifacts/staged").iterdir())
            self.assertEqual(json.loads((output / "stage-result.json").read_text())["missing_artifacts"], ["profile-index.json"])

    def test_successful_stage_cannot_pass_with_missing_profile_evidence(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            candidate = root / "candidate"
            coverage = candidate / "artifacts/coverage"
            coverage.mkdir(parents=True)
            (coverage / "profile-index.json").write_text(json.dumps({"run": "profiles-test", "profiles": ["headless", "renderer"]}))
            for name in ("test-output.log", "summary.json", "evidence.json"):
                (coverage / name).write_text("legacy")
            with patch.object(verify, "ROOT", root):
                with self.assertRaisesRegex(ValueError, "missing retained profile evidence"):
                    verify.retain_staged_evidence("a" * 40, candidate, "passed")
            output = next((root / "artifacts/staged").iterdir())
            self.assertEqual(json.loads((output / "stage-result.json").read_text())["evidence_state"], "partial")

    def test_staged_raw_symlink_is_not_copied(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            candidate = root / "candidate"
            coverage = candidate / "artifacts/coverage"
            output = root / "out"
            output.mkdir()
            coverage.mkdir(parents=True)
            (coverage / "profile-index.json").write_text(json.dumps({"run": "profiles-test", "profiles": ["headless", "renderer"]}))
            outside = root / "outside.profraw"
            outside.write_text("outside")
            for profile in ("headless", "renderer"):
                base = coverage / "profiles-test" / profile
                (base / "raw").mkdir(parents=True)
                for name in ("test-output.log", "coverage.json", "coverage.lcov", "summary.json", "evidence.json", "merged.profdata"):
                    (base / name).write_text(profile)
                if profile == "renderer":
                    (base / "facade-audit.json").write_text("proof")
                    (base / "raw/one.profraw").symlink_to(outside)
                else:
                    (base / "raw/one.profraw").write_text("headless")
            hashes, missing = {}, []
            verify.retain_profile_evidence(candidate, output, hashes, missing)
            self.assertEqual(missing, ["profiles-test/renderer/raw/one.profraw"])
            self.assertFalse((output / "profiles-test/renderer/raw/one.profraw").exists())

    def test_staged_profile_parent_symlink_cannot_escape_export(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            candidate = root / "candidate"
            coverage = candidate / "artifacts/coverage"
            output = root / "out"
            output.mkdir()
            coverage.mkdir(parents=True)
            (coverage / "profile-index.json").write_text(json.dumps({"run": "profiles-test", "profiles": ["headless", "renderer"]}))
            outside = root / "outside"
            (outside / "raw").mkdir(parents=True)
            (outside / "raw/one.profraw").write_text("outside")
            run = coverage / "profiles-test"
            run.mkdir()
            (run / "headless").symlink_to(outside, target_is_directory=True)
            hashes, missing = {}, []
            verify.retain_profile_evidence(candidate, output, hashes, missing)
            self.assertIn("profiles-test/headless/raw", missing)
            self.assertFalse((output / "profiles-test/headless/raw/one.profraw").exists())

    def test_profile_index_cannot_escape_export(self):
        with tempfile.TemporaryDirectory() as tmp:
            candidate = Path(tmp) / "candidate"
            coverage = candidate / "artifacts/coverage"
            coverage.mkdir(parents=True)
            output = Path(tmp) / "out"
            output.mkdir()
            (coverage / "profile-index.json").write_text(json.dumps({"run": "../outside", "profiles": ["headless", "renderer"]}))
            with self.assertRaisesRegex(ValueError, "invalid staged"):
                verify.retain_profile_evidence(candidate, output, {}, [])

class RealInstrumentation(unittest.TestCase):
    def test_disabled_feature_cannot_borrow_enabled_profile_coverage(self):
        # A tiny Rust fixture exercises the gate itself; it is not a rendering test.
        import os
        import shutil
        import subprocess
        with tempfile.TemporaryDirectory(prefix="gz-feature-profile-test-") as tmp:
            root = Path(tmp)
            (root / "src").mkdir()
            (root / "Cargo.toml").write_text('[package]\nname="profile-probe"\nversion="0.1.0"\nedition="2024"\n[features]\ngpu=[]\n')
            lib = root / "src/lib.rs"
            gpu = root / "src/gpu.rs"
            lib.write_text('#[cfg(feature="gpu")] pub mod gpu;\npub fn base() -> u8 { 1 }\n#[cfg(test)] mod tests { #[test] fn base_works() { assert_eq!(super::base(), 1); } }\n')
            gpu.write_text('pub fn active() -> u8 { 2 }\n#[cfg(test)] mod tests { #[test] fn gpu_works() { assert_eq!(super::active(), 2); } }\n')
            wrapper = root / "wrapper"
            shutil.copyfile(Path(__file__).resolve().parents[1] / "scripts/rustc-coverage.py", wrapper)
            wrapper.chmod(0o700)
            environment = dict(os.environ, RUSTUP_TOOLCHAIN="1.97.1")
            subprocess.run(["cargo", "generate-lockfile", "--offline"], cwd=root, env=environment, check=True, capture_output=True)
            version = subprocess.check_output(["rustc", "-vV"], env=environment, text=True)
            sysroot = Path(subprocess.check_output(["rustc", "--print", "sysroot"], env=environment, text=True).strip())
            host = next(line.split(": ", 1)[1] for line in version.splitlines() if line.startswith("host: "))
            llvm = sysroot / "lib/rustlib" / host / "bin"
            crates = {"profile-probe": [lib, gpu]}
            with patch.dict(os.environ, environment, clear=True), patch.object(verify, "ROOT", root):
                good, zero = verify.profile_coverage("headless", crates,
                    ["cargo", "test", "--locked", "--all-targets", "--features", "gpu"],
                    root / "active", {}, wrapper, llvm, version)
                self.assertFalse(zero)
                verify.check_coverage(good, crates)
                evidence = json.loads((root / "active/evidence.json").read_text())
                self.assertTrue(evidence["objects"])
                for artifact in evidence["objects"]:
                    self.assertEqual(len(artifact["sha256"]), 64)
                    self.assertTrue(Path(artifact["path"]).is_file())
                with self.assertRaisesRegex(ValueError, "uninstrumented"):
                    verify.profile_coverage("headless", crates,
                        ["cargo", "test", "--locked", "--all-targets"],
                        root / "inactive", {}, wrapper, llvm, version)
            self.assertTrue(list((root / "active/raw").glob("*.profraw")))
            self.assertTrue(list((root / "inactive/raw").glob("*.profraw")))
            self.assertNotEqual((root / "active/raw").resolve(), (root / "inactive/raw").resolve())

    def test_worker_raw_profile_comes_from_same_cargo_test_graph(self):
        # This synthetic worker tests evidence plumbing only, not rendering behavior.
        import os
        import shutil
        import subprocess
        with tempfile.TemporaryDirectory(prefix="gz-worker-profile-test-") as tmp:
            root = Path(tmp)
            (root / "Cargo.toml").write_text('[workspace]\nmembers=["crates/gz-renderer","crates/gz-render-worker"]\nresolver="3"\n')
            renderer = root / "crates/gz-renderer"
            worker = root / "crates/gz-render-worker"
            (renderer / "src").mkdir(parents=True)
            (worker / "src").mkdir(parents=True)
            (worker / "tests").mkdir()
            (renderer / "Cargo.toml").write_text('[package]\nname="gz-renderer"\nversion="0.1.0"\nedition="2024"\n[features]\ngpu=[]\n')
            lib = renderer / "src/lib.rs"
            gpu = renderer / "src/gpu.rs"
            lib.write_text('#[cfg(feature="gpu")] pub mod gpu;\npub fn base()->u8 {1}\n#[cfg(test)] mod tests { #[test] fn base_works(){assert_eq!(super::base(),1);} }\n')
            gpu.write_text('pub fn active()->u8 {2}\n#[cfg(test)] mod tests { #[test] fn gpu_works(){assert_eq!(super::active(),2);} }\n')
            (worker / "Cargo.toml").write_text('[package]\nname="gz-render-worker"\nversion="0.1.0"\nedition="2024"\n')
            main = worker / "src/main.rs"
            main.write_text('fn main(){println!("synthetic worker exited");}\n')
            (worker / "tests/child.rs").write_text('''#[test]
fn emits_child_profile() {
    let status = std::process::Command::new(env!("CARGO_BIN_EXE_gz-render-worker"))
        .env("LLVM_PROFILE_FILE", std::env::var("GZ_RENDER_WORKER_PROFILE").unwrap())
        .status().unwrap();
    assert!(status.success());
}
''')
            wrapper = root / "wrapper"
            shutil.copyfile(Path(__file__).resolve().parents[1] / "scripts/rustc-coverage.py", wrapper)
            wrapper.chmod(0o700)
            environment = dict(os.environ, RUSTUP_TOOLCHAIN="1.97.1")
            subprocess.run(["cargo", "generate-lockfile", "--offline"], cwd=root, env=environment, check=True, capture_output=True)
            version = subprocess.check_output(["rustc", "-vV"], env=environment, text=True)
            sysroot = Path(subprocess.check_output(["rustc", "--print", "sysroot"], env=environment, text=True).strip())
            host = next(line.split(": ", 1)[1] for line in version.splitlines() if line.startswith("host: "))
            crates = {"gz-renderer": [lib, gpu], "gz-render-worker": [main]}
            command = ["cargo", "test", "--locked", "--all-targets", "--package", "gz-renderer", "--package", "gz-render-worker", "--features", "gz-renderer/gpu"]
            with patch.dict(os.environ, environment, clear=True), patch.object(verify, "ROOT", root):
                report_data, zero = verify.profile_coverage("renderer", crates, command, root / "profile", {}, wrapper, sysroot / "lib/rustlib" / host / "bin", version)
                self.assertFalse(zero)
                verify.check_coverage(report_data, crates)
            self.assertTrue(list((root / "profile/raw").glob("worker-*.profraw")))
