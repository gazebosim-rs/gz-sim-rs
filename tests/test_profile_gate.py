# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import importlib.util
import hashlib
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
            for name in verify.RENDER_PACKAGES | {"gz-sim-core", "gz-sim-scene", "gz-sim-cli", "gz-sim-proto"}}


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
                for profile in ("headless", "renderer"):
                    weakened = json.loads(json.dumps(contract))
                    weakened["profiles"][profile]["features"].remove("gz-sim-cli/rgb-camera")
                    path.write_text(json.dumps(weakened))
                    with self.subTest(profile=profile), self.assertRaisesRegex(ValueError, "invalid coverage"):
                        verify.load_profile_contract()
                weakened = json.loads(json.dumps(contract))
                weakened["profiles"]["renderer"]["packages"].remove("gz-sim-cli")
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
        self.assertEqual(set(plan["headless"][0]), {"gz-sim-core", "gz-sim-scene", "gz-sim-cli", "gz-sim-proto"})
        self.assertEqual(set(plan["renderer"][0]), verify.RENDER_PACKAGES | {"gz-sim-core", "gz-sim-scene", "gz-sim-cli", "gz-sim-proto"})
        self.assertEqual(set(plan["headless"][0]) | set(plan["renderer"][0]), set(crates))
        for profile in ("headless", "renderer"):
            command = plan[profile][1]
            self.assertEqual(command.count("--no-fail-fast"), 1)
            self.assertIn("gz-sim-cli/rgb-camera", command[command.index("--features") + 1].split(","))
            self.assertIn("gz-sim-cli", plan[profile][0])
        self.assertIn("gz-renderer/gpu", plan["renderer"][1][plan["renderer"][1].index("--features") + 1].split(","))
        self.assertNotIn("gz-sim-cli", verify.RENDER_PACKAGES)
        self.assertEqual(plan["headless"][1].count("--exclude"), len(verify.RENDER_PACKAGES) + 1)
        self.assertIn(verify.TEST_SUPPORT_PACKAGE, plan["renderer"][1])

    def test_missing_package_empty_source_and_duplicate_ownership_fail(self):
        crates = sources()
        del crates["gz-sim-cli"]
        with self.assertRaisesRegex(ValueError, "packages missing"):
            verify.profile_plan(crates)
        crates = sources()
        del crates["gz-camera-output"]
        with self.assertRaisesRegex(ValueError, "packages missing"):
            verify.profile_plan(crates)
        crates = sources()
        del crates["gz-render-scene"]
        with self.assertRaisesRegex(ValueError, "packages missing"):
            verify.profile_plan(crates)
        crates = sources()
        del crates["gz-sim-scene"]
        with self.assertRaisesRegex(ValueError, "packages missing"):
            verify.profile_plan(crates)
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

    def test_cli_whole_and_changed_coverage_fail_independently_in_each_profile(self):
        plan = verify.profile_plan(sources())
        for profile in ("headless", "renderer"):
            for metric in ("lines", "regions"):
                reports = {name: report(crates) for name, (crates, _) in plan.items()}
                cli = plan[profile][0]["gz-sim-cli"][0]
                entry = next(file for file in reports[profile]["data"][0]["files"]
                             if file["filename"] == str(cli))
                entry["summary"][metric]["covered"] = 95
                with self.subTest(profile=profile, metric=metric), self.assertRaisesRegex(ValueError, "gz-sim-cli"):
                    verify.check_profile_reports(plan, reports)
            with patch.object(verify, "ROOT", Path("/virtual")):
                lcov, changed = [], {}
                for crate, paths in plan[profile][0].items():
                    for path in paths:
                        changed[str(path.relative_to(verify.ROOT))] = list(range(1, 21))
                        lcov.append(f"SF:{path}\n" + "\n".join(
                            f"DA:{line},{int(crate != 'gz-sim-cli' or line != 20)}" for line in range(1, 21))
                                    + "\nend_of_record\n")
                with self.subTest(profile=profile), self.assertRaisesRegex(ValueError, "gz-sim-cli: changed executable lines 19/20"):
                    verify.check_changed_coverage("".join(lcov), changed, plan[profile][0])

    def renderer_artifact(self, root, name):
        package = "gz-sim-cli" if name == "gz-sim-rs" else "gz-render-worker"
        source = root / "crates" / package / "src/main.rs"
        source.parent.mkdir(parents=True, exist_ok=True)
        source.write_text("fn main() {}\n")
        manifest = source.parent.parent / "Cargo.toml"
        manifest.write_text(f'[package]\nname="{package}"\nversion="0.1.0"\n')
        binary = root / "target/coverage/debug" / name
        binary.parent.mkdir(parents=True, exist_ok=True)
        binary.write_bytes(b"\x7fELF" + name.encode())
        binary.chmod(0o700)
        return {"reason": "compiler-artifact", "package_id": "path+" + manifest.parent.as_uri() + "#0.1.0",
                "manifest_path": str(manifest), "target": {"kind": ["bin"], "crate_types": ["bin"],
                    "name": name, "src_path": str(source)}, "profile": {"test": False},
                "features": ["rgb-camera"] if name == "gz-sim-rs" else [],
                "executable": str(binary), "filenames": [str(binary)], "fresh": False}

    def test_renderer_executable_bindings_preserve_cargo_identity_and_detect_byte_drift(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp).resolve()
            artifacts = [self.renderer_artifact(root, name) for name in ("gz-sim-rs", "gz-render-worker")]
            with patch.object(verify, "ROOT", root):
                stdout = "\n".join(map(json.dumps, artifacts))
                bound = verify.renderer_executables(stdout, root / "target/coverage")
                self.assertEqual(set(bound), {"gz-sim-rs", "gz-render-worker"})
                self.assertEqual(bound["gz-sim-rs"]["cargo_artifact"], artifacts[0])
                cached = json.loads(json.dumps(artifacts))
                cached[0]["fresh"] = True
                verify.same_renderer_executables(bound, verify.renderer_executables(
                    "\n".join(map(json.dumps, cached)), root / "target/coverage"))
                binary = Path(artifacts[0]["executable"])
                binary.write_bytes(binary.read_bytes() + b"changed")
                with self.assertRaisesRegex(ValueError, "changed between"):
                    verify.same_renderer_executables(bound, verify.renderer_executables(stdout, root / "target/coverage"))

    def test_renderer_executable_bindings_reject_unowned_or_wrong_feature_artifacts(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp).resolve()
            artifacts = [self.renderer_artifact(root, name) for name in ("gz-sim-rs", "gz-render-worker")]
            variants = []
            for key, value in [("features", []), ("manifest_path", str(root / "other/Cargo.toml")),
                               ("package_id", "registry+https://example.invalid#gz-sim-cli@0.1.0"),
                               ("filenames", []), ("executable", "target/coverage/debug/gz-sim-rs")]:
                bad = json.loads(json.dumps(artifacts))
                bad[0][key] = value
                variants.append(bad)
            for key, value in [("src_path", str(root / "other/main.rs")), ("kind", ["test"])]:
                bad = json.loads(json.dumps(artifacts))
                bad[0]["target"][key] = value
                variants.append(bad)
            bad = json.loads(json.dumps(artifacts))
            bad[0]["profile"]["test"] = True
            variants.extend([bad, artifacts[:1], artifacts + [artifacts[0]]])
            with patch.object(verify, "ROOT", root):
                for index, items in enumerate(variants):
                    with self.subTest(index=index), self.assertRaisesRegex(ValueError, "renderer.*binding"):
                        verify.renderer_executables("\n".join(map(json.dumps, items)), root / "target/coverage")
                with self.assertRaisesRegex(ValueError, "binding"):
                    verify.renderer_executables("\n".join(map(json.dumps, artifacts)), root / "different-target")
                original = Path(artifacts[0]["executable"])
                link = original.with_name("linked-cli")
                link.symlink_to(original)
                bad = json.loads(json.dumps(artifacts))
                bad[0]["executable"] = str(link)
                bad[0]["filenames"] = [str(link)]
                with self.assertRaisesRegex(ValueError, "binding"):
                    verify.renderer_executables("\n".join(map(json.dumps, bad)), root / "target/coverage")
                original.write_bytes(b"not an executable")
                with self.assertRaisesRegex(ValueError, "not ELF"):
                    verify.renderer_executables("\n".join(map(json.dumps, artifacts)), root / "target/coverage")

    def test_renderer_build_only_handoff_precedes_exactly_one_test_execution(self):
        import subprocess
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp).resolve()
            artifacts = [self.renderer_artifact(root, name) for name in ("gz-sim-rs", "gz-render-worker")]
            prepared = SimpleNamespace(returncode=0, stdout="\n".join(map(json.dumps, artifacts)), stderr="build only", args=["cargo", "test", "--no-run"])
            failed = SimpleNamespace(returncode=1, stdout="actual test failure", stderr="", args=["cargo", "test"])
            crates = {package: [root / "crates" / package / "src/main.rs"] for package in ("gz-sim-cli", "gz-render-worker")}
            with patch.object(verify, "ROOT", root), patch.object(verify.profile_process, "run", side_effect=[prepared, failed]) as run:
                with self.assertRaises(subprocess.CalledProcessError):
                    verify.profile_coverage("renderer", crates, ["cargo", "test", "--features", "gz-sim-cli/rgb-camera"],
                        root / "profile", {}, root / "wrapper", root, "rustc")
            self.assertEqual(run.call_count, 2)
            self.assertIn("--no-run", run.call_args_list[0].args[0])
            self.assertNotIn("--no-run", run.call_args_list[1].args[0])
            env = run.call_args_list[1].kwargs["env"]
            self.assertEqual(env["GZ_RENDER_CLI_PATH"], artifacts[0]["executable"])
            self.assertEqual(env["GZ_RENDER_WORKER_PATH"], artifacts[1]["executable"])
            self.assertEqual(json.loads(env["GZ_RENDER_EXECUTABLES_JSON"])["gz-sim-rs"]["cargo_artifact"], artifacts[0])
            self.assertTrue((root / "profile/renderer-build-output.log").is_file())
            self.assertEqual((root / "profile/test-output.log").read_text(), "actual test failure")

    def test_failed_renderer_preparation_preserves_diagnostics_without_running_tests(self):
        import subprocess
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp).resolve()
            artifact = self.renderer_artifact(root, "gz-sim-rs")
            failed = SimpleNamespace(returncode=1, stdout="build stopped", stderr="compiler failure",
                                     args=["cargo", "test", "--no-run"])
            with patch.object(verify, "ROOT", root), patch.object(verify.profile_process, "run", return_value=failed) as run:
                with self.assertRaises(subprocess.CalledProcessError):
                    verify.profile_coverage("renderer", {"gz-sim-cli": [Path(artifact["target"]["src_path"])]},
                        ["cargo", "test"], root / "profile", {}, root / "wrapper", root, "rustc")
            self.assertEqual(run.call_count, 1)
            self.assertIn("--no-run", run.call_args.args[0])
            self.assertEqual((root / "profile/renderer-build-output.log").read_text(), "build stoppedcompiler failure")
            self.assertFalse((root / "profile/test-output.log").exists())

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
            with patch.object(verify, "ROOT", root), patch.object(verify.profile_process, "run", return_value=result), patch.object(verify, "run", return_value=SimpleNamespace(stdout="rustc")):
                with self.assertRaisesRegex(ValueError, "no raw coverage profiles"):
                    verify.profile_coverage("headless", {"crate": [source]}, ["cargo"], output, {}, root / "wrapper", root, "rustc")
            self.assertTrue((output / "test-output.log").is_file())

    def test_profile_export_keeps_all_owned_sources_and_strict_instrumentation_checks(self):
        # Model LLVM's source selection, not a successful Cargo or LLVM execution.
        # Shared test support has multiple mappings; every owned mapping, including
        # a completely uncovered source and build.rs, must still reach the checks.
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            library = root / "src/lib.rs"
            build = root / "build.rs"
            cold = root / "src/cold.rs"
            shared = root / "tests/support/shared.rs"
            for path in (library, build, cold, shared):
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("// nonexecuted coverage fixture\n")
            binary = root / "test-elf"
            binary.write_bytes(b"nonexecuted test executable fixture")
            facade = root / verify.proto_inputs.FACADE
            proto_build = root / verify.proto_inputs.BUILD
            auditor_source = root / verify.proto_inputs.AUDITOR
            auditor = root / "target/coverage/source-auditor"
            generated = root / "target/coverage/build/proto/out"
            generated.mkdir(parents=True)
            binding = generated / "gz.msgs.rs"
            for path in (facade, proto_build, auditor_source, auditor, binding):
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(b"nonexecuted generator/auditor fixture")
            (generated / "schema-descriptor.pb").write_bytes(b"descriptor fixture")
            proto_evidence = {"generated": {"path": str(binding)}}

            def mapping(path, covered, count):
                return {"filename": str(path), "summary": {
                    metric: {"count": count, "covered": covered}
                    for metric in ("lines", "regions")}}

            for scenario in ("shared_uncovered", "shared_covered", "missing_owned", "duplicate_owned", "uncovered_build",
                             "lexical_build", "duplicate_owned_alias",
                             "proto_valid", "proto_missing", "proto_extra"):
                with self.subTest(scenario=scenario):
                    output = root / scenario
                    crates = {"owned": [library, build, cold]}
                    records = [mapping(library, 20, 20), mapping(build, 20, 20), mapping(cold, 0, 1)]
                    support = mapping(shared, int(scenario == "shared_covered"), 1)
                    records += [support, support]
                    if scenario == "missing_owned":
                        records = [row for row in records if row["filename"] != str(cold)]
                    elif scenario == "duplicate_owned":
                        records.append(mapping(build, 20, 20))
                    elif scenario == "uncovered_build":
                        records[1] = mapping(build, 0, 20)
                    elif scenario == "lexical_build":
                        records[1] = mapping(root / "tests/../build.rs", 20, 20)
                    elif scenario == "duplicate_owned_alias":
                        records.append(mapping(root / "tests/../build.rs", 20, 20))
                    if scenario.startswith("proto_"):
                        crates[verify.proto_inputs.PACKAGE] = [facade, proto_build]
                        records.append(mapping(proto_build, 20, 20))
                        if scenario != "proto_missing":
                            records.append(mapping(binding, 1, 1))
                        if scenario == "proto_extra":
                            records.append(mapping(generated / "unexpected.rs", 1, 1))

                    def cargo(*args, **kwargs):
                        if args[0][0] == str(auditor):
                            proof = {"proof": verify.proto_inputs.PROOF, "source": str(facade),
                                     "source_sha256": verify.proto_inputs.digest(facade)}
                            Path(kwargs["env"]["GZ_PROTO_FACADE_PROOF"]).write_text(json.dumps(proof))
                            return SimpleNamespace(returncode=0, stdout="", stderr="")
                        raw = Path(kwargs["env"]["LLVM_PROFILE_FILE"]).parent
                        (raw / "fixture.profraw").write_bytes(b"nonexecuted raw fixture")
                        artifact = {"reason": "compiler-artifact", "executable": str(binary)}
                        events = [artifact]
                        if scenario.startswith("proto_"):
                            events += [
                                {"reason": "compiler-artifact", "package_id": "proto-fixture",
                                 "target": {"src_path": str(proto_build), "kind": ["custom-build"]}},
                                {"reason": "build-script-executed", "package_id": "proto-fixture", "out_dir": str(generated)},
                                {"reason": "compiler-artifact", "executable": str(auditor),
                                 "target": {"src_path": str(auditor_source), "kind": ["test"]}},
                            ]
                        return SimpleNamespace(returncode=0, stdout="\n".join(map(json.dumps, events)), stderr="", args=["cargo"])

                    def llvm(command, **kwargs):
                        if command[:2] == ["rustc", "-vV"]:
                            return SimpleNamespace(stdout="fixture rustc")
                        if command[1] == "merge":
                            Path(command[command.index("-o") + 1]).write_bytes(b"nonexecuted merged fixture")
                            return SimpleNamespace()
                        self.assertEqual(command[1], "export")
                        selected = records
                        if "--sources" in command:
                            # Real LLVM normalizes requested paths but can retain
                            # lexical tests/../build.rs spellings in its mapping.
                            paths = {str(Path(value).resolve()) for value in command[command.index("--sources") + 1:]
                                     if not value.startswith("-")}
                            selected = [row for row in records if row["filename"] in paths]
                        if "-format=lcov" in command:
                            lcov = "".join(f"SF:{row['filename']}\nDA:1,{int(row['summary']['lines']['covered'] > 0)}\nend_of_record\n"
                                           for row in selected)
                            return SimpleNamespace(stdout=lcov.encode())
                        json.dump({"data": [{"files": selected}]}, kwargs["stdout"])
                        return SimpleNamespace()

                    with patch.object(verify, "ROOT", root), \
                         patch.object(verify.profile_process, "run", side_effect=cargo), \
                         patch.object(verify.subprocess, "run", side_effect=cargo), \
                         patch.object(verify, "run", side_effect=llvm), \
                         patch.object(verify.proto_inputs, "generated_inputs", return_value=proto_evidence):
                        def profile():
                            return verify.profile_coverage(
                                "headless", crates, ["cargo"], output, {"src/lib.rs": [1]},
                                root / "wrapper", root, "fixture rustc")

                        if scenario == "missing_owned":
                            with self.assertRaisesRegex(ValueError, "uninstrumented source"):
                                profile()
                        elif scenario in ("duplicate_owned", "duplicate_owned_alias"):
                            with self.assertRaisesRegex(ValueError, "duplicate coverage"):
                                profile()
                        elif scenario == "uncovered_build":
                            with self.assertRaisesRegex(ValueError, "must exceed 95%"):
                                profile()
                        elif scenario in ("proto_missing", "proto_extra"):
                            with self.assertRaisesRegex(ValueError, "generated source is absent or unexpected"):
                                profile()
                            self.assertTrue((output / "proto-coverage.json").is_file())
                            self.assertFalse((output / "coverage.json").exists())
                        else:
                            profile()
                            summary = json.loads((output / "summary.json").read_text())
                            self.assertEqual(summary["owned"], {"lines": [40, 41], "regions": [40, 41]})
                            exported = json.loads((output / "coverage.json").read_text())
                            expected = {str(path) for path in crates["owned"]}
                            if scenario == "proto_valid":
                                expected.add(str(proto_build))
                                self.assertEqual(summary[verify.proto_inputs.PACKAGE],
                                                 {"lines": [20, 20], "regions": [20, 20]})
                                unfiltered = json.loads((output / "proto-coverage.json").read_text())
                                self.assertIn(str(binding), {row["filename"] for row in unfiltered["data"][0]["files"]})
                            self.assertEqual({str(Path(row["filename"]).resolve()) for row in exported["data"][0]["files"]}, expected)
                            if scenario == "lexical_build":
                                self.assertIn(str(root / "tests/../build.rs"),
                                              {row["filename"] for row in exported["data"][0]["files"]})

    def test_owned_view_preserves_whole_alias_records_and_refuses_malformed_lcov(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            build = root / "build.rs"
            alias = root / "tests/../build.rs"
            support = root / "tests/helper.rs"
            owned = {"crate": [build]}
            mapping = {"filename": str(alias), "segments": [[1, 1, 0, True, True, False]],
                       "summary": {metric: {"count": 1, "covered": 0} for metric in ("lines", "regions")}}
            shared = {"filename": str(support), "summary": {}}
            raw = {"data": [{"files": [shared, mapping, shared]}]}
            before = json.dumps(raw, sort_keys=True)
            block = f"TN:fixture\nSF:{alias}\nFN:1,uncovered\nFNDA:0,uncovered\nDA:1,0\nBRDA:1,0,0,-\nend_of_record\n"
            external = f"SF:{support}\nDA:1,1\nend_of_record\n"
            view, lcov = verify.owned_coverage(raw, external + block + external, owned)
            self.assertEqual(view, {"data": [{"files": [mapping]}]})
            self.assertEqual(lcov, block)
            self.assertEqual(json.dumps(raw, sort_keys=True), before)
            for invalid in ["DA:1,0\nend_of_record\n",
                            f"SF:{build}\nSF:{support}\nend_of_record\n",
                            f"SF:{build}\nDA:1,0\n",
                            block.replace("\n", "\r\n"),
                            block + "DA:2,0\n"]:
                with self.subTest(lcov=invalid), self.assertRaises(ValueError):
                    verify.owned_coverage(raw, invalid, owned)

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
            with patch.object(verify, "ROOT", root), patch.object(verify.profile_process, "run", side_effect=fake_cargo), patch.object(verify, "run", return_value=SimpleNamespace(stdout="rustc")):
                with self.assertRaisesRegex(ValueError, "worker emitted no raw profile"):
                    verify.profile_coverage("renderer", {"crate": [source]}, ["cargo"], root / "profile", {}, root / "wrapper", root, "rustc")

    def test_renderer_requires_worker_artifact_from_its_feature_graph(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / "src/lib.rs"
            source.parent.mkdir()
            source.write_text("original")
            result = SimpleNamespace(returncode=0, stdout=json.dumps({"reason": "compiler-artifact", "executable": "/tmp/not-the-worker", "target": {"name": "other", "kind": ["bin"]}, "profile": {"test": False}}), stderr="", args=["cargo"])
            with patch.object(verify, "ROOT", root), patch.object(verify.profile_process, "run", return_value=result):
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
            with patch.object(verify, "ROOT", root), patch.object(verify.profile_process, "run", side_effect=fake_cargo), patch.object(verify, "run", return_value=SimpleNamespace(stdout="rustc")):
                with self.assertRaisesRegex(ValueError, "auditor changed"):
                    verify.profile_coverage("renderer", {"crate": [source]}, ["cargo"], root / "profile", {}, root / "wrapper", root, "rustc")

    def test_compiler_change_during_profile_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / "src/lib.rs"
            source.parent.mkdir()
            source.write_text("original")
            result = SimpleNamespace(returncode=0, stdout=json.dumps({"reason": "compiler-artifact", "executable": "/tmp/probe"}), stderr="", args=["cargo"])
            with patch.object(verify, "ROOT", root), patch.object(verify.profile_process, "run", return_value=result), patch.object(verify, "run", return_value=SimpleNamespace(stdout="changed rustc")):
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
            with patch.object(verify, "ROOT", root), patch.object(verify.profile_process, "run", side_effect=fake_cargo), patch.object(verify, "run", return_value=SimpleNamespace(stdout="rustc")):
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
                for name in ("test-output.log", "coverage.json", "coverage.lcov", "summary.json", "evidence.json", "merged.profdata", "proto-generated.json", "proto-facade-audit.json", "proto-facade-audit-output.log", "proto-generated.rs", "proto-descriptor.pb", "proto-coverage.json", "coverage-unfiltered.lcov"):
                    (base / name).write_text(profile)
                (base / "raw" / "one.profraw").write_text(profile)
                for channel in ("stdout", "stderr"):
                    (base / ("tests." + channel)).write_text(profile + channel)
                if profile == "renderer":
                    (base / "facade-audit.json").write_text("proof")
            hashes, missing = {}, []
            verify.retain_profile_evidence(candidate, output, hashes, missing)
            self.assertEqual(missing, [])
            self.assertEqual((output / "profiles-test/headless/raw/one.profraw").read_text(), "headless")
            self.assertEqual((output / "profiles-test/renderer/raw/one.profraw").read_text(), "renderer")
            for profile in ("headless", "renderer"):
                for channel in ("stdout", "stderr"):
                    relative = f"profiles-test/{profile}/tests.{channel}"
                    raw = (profile + channel).encode()
                    self.assertEqual((output / relative).read_bytes(), raw)
                    self.assertEqual(hashes[relative], hashlib.sha256(raw).hexdigest())
            # A cancelled profile may never finalize test-output.log. Partial
            # channels must still be retained without granting a passing stage.
            final_log = coverage / "profiles-test/headless/test-output.log"
            final_log.unlink()
            partial = Path(tmp) / "partial"
            partial.mkdir()
            partial_hashes, partial_missing = {}, []
            verify.retain_profile_evidence(candidate, partial, partial_hashes, partial_missing)
            self.assertIn("profiles-test/headless/test-output.log", partial_missing)
            self.assertEqual((partial / "profiles-test/headless/tests.stdout").read_text(), "headlessstdout")
            final_log.write_text("headless")
            for name in ("proto-coverage.json", "coverage-unfiltered.lcov"):
                relative = "profiles-test/headless/" + name
                self.assertEqual((output / relative).read_bytes(), b"headless")
                self.assertEqual(hashes[relative], hashlib.sha256(b"headless").hexdigest())
            (coverage / "profiles-test/renderer/raw/one.profraw").unlink()
            hashes, missing = {}, []
            verify.retain_profile_evidence(candidate, output, hashes, missing)
            self.assertIn("profiles-test/renderer/raw/*.profraw", missing)
            (coverage / "profiles-test/renderer/raw/one.profraw").write_text("renderer")
            for name in ("proto-coverage.json", "coverage-unfiltered.lcov"):
                with self.subTest(artifact=name):
                    relative = "profiles-test/headless/" + name
                    proof = coverage / relative
                    proof.unlink()
                    incomplete = Path(tmp) / ("missing-" + name)
                    incomplete.mkdir()
                    hashes, missing = {}, []
                    verify.retain_profile_evidence(candidate, incomplete, hashes, missing)
                    self.assertEqual(missing, [relative])
                    foreign = Path(tmp) / ("foreign-" + name)
                    foreign.write_text("unqualified unfiltered report")
                    proof.symlink_to(foreign)
                    unsafe = Path(tmp) / ("linked-" + name)
                    unsafe.mkdir()
                    hashes, missing = {}, []
                    verify.retain_profile_evidence(candidate, unsafe, hashes, missing)
                    self.assertEqual(missing, [relative])
                    self.assertFalse((unsafe / relative).exists())
                    proof.unlink()
                    proof.write_text("headless")

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
                for name in ("test-output.log", "coverage.json", "coverage.lcov", "summary.json", "evidence.json", "merged.profdata", "proto-generated.json", "proto-facade-audit.json", "proto-facade-audit-output.log", "proto-generated.rs", "proto-descriptor.pb", "proto-coverage.json", "coverage-unfiltered.lcov"):
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

    def test_selected_native_metadata_is_retained_and_cannot_be_substituted(self):
        import hashlib
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            candidate = root / "candidate"
            planning = candidate / "planning"
            planning.mkdir(parents=True)
            # Retention sees the same complete RGB profile contract as a real
            # staged export, while the metadata below remains synthetic.
            (planning / "coverage-profiles.json").write_bytes(
                (verify.ROOT / "planning/coverage-profiles.json").read_bytes())
            coverage = candidate / "artifacts/coverage"
            run = coverage / "profiles-test"
            run.mkdir(parents=True)
            output = root / "out"
            output.mkdir()
            blob = b'{"profile":"gazebo-zenoh-client-v1"}\n'
            expected = hashlib.sha256(blob).hexdigest()
            (coverage / "profile-index.json").write_text(json.dumps({
                "run": "profiles-test", "profiles": ["headless", "renderer"],
                "zenoh_reference": {"environment_sha256": expected},
            }))
            metadata = run / "zenoh-native-reference.json"
            metadata.write_bytes(blob)
            hashes, missing = {}, []
            verify.retain_profile_evidence(candidate, output, hashes, missing)
            self.assertEqual((output / metadata.name).read_bytes(), blob)
            self.assertEqual(hashes[metadata.name], expected)
            metadata.write_bytes(blob + b" ")
            with self.assertRaisesRegex(ValueError, "reference metadata differs"):
                verify.retain_profile_evidence(candidate, output, {}, [])
            metadata.unlink()
            missing = []
            verify.retain_profile_evidence(candidate, output, {}, missing)
            self.assertIn(metadata.name, missing)
            foreign = root / "foreign.json"
            foreign.write_bytes(blob)
            metadata.symlink_to(foreign)
            missing = []
            verify.retain_profile_evidence(candidate, output, {}, missing)
            self.assertIn(metadata.name, missing)

            metadata.unlink()
            (coverage / "profile-index.json").write_text(json.dumps({
                "run": "profiles-test", "profiles": ["headless", "renderer"],
            }))
            missing = []
            verify.retain_profile_evidence(candidate, output, {}, missing)
            self.assertIn("zenoh_reference", missing)


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
