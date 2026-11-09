# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("rapier_retention_qualifier", ROOT / "scripts/qualify-rapier-runtime.py")
QUALIFIER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(QUALIFIER)
SPEC = importlib.util.spec_from_file_location("rapier_retention_verify", ROOT / "scripts/verify.py")
VERIFY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VERIFY)


class AuxiliaryRetention(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name) / "source"
        self.directory = self.root / "artifacts/coverage/rapier-runtime"
        self.directory.mkdir(parents=True)
        for name in ["Cargo.toml", "Cargo.lock", "rust-toolchain.toml", ".cargo/config.toml",
                     "scripts/rustc-coverage.py", "scripts/verify.py", "scripts/qualify-rapier-runtime.py",
                     "scripts/check-rapier-runtime.py", "crates/gz-sim-cli/tests/empty.sdf",
                     "crates/gz-sim-cli/src/live.rs",
                     "crates/gz-camera-output/tests/fixtures/m5-rgb.sdf",
                     "crates/gz-camera-output/tests/fixtures/m6-depth.sdf", *QUALIFIER.NOISY_EXAMPLES,
                     "crates/gz-sim-proto/schema-source.json", "crates/gz-sim-proto/schemas/manifest.json",
                     "third_party/prost-derive-0.14.4/Cargo.toml", "third_party/prost-derive-0.14.4/LICENSE",
                     "third_party/prost-derive-0.14.4/src/lib.rs",
                     *QUALIFIER.EXECUTABLE_SOURCES.values(), *QUALIFIER.RGB_SOURCES.values()]:
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("fixture input\n")
        (self.root / "scripts/qualify-rapier-runtime.py").write_bytes(
            (ROOT / "scripts/qualify-rapier-runtime.py").read_bytes())
        (self.root / "scripts/check-rapier-runtime.py").write_bytes(
            (ROOT / "scripts/check-rapier-runtime.py").read_bytes())
        source = '\n#[cfg(not(feature = "native-backends"))]\nPhysicsEngine::Dart | PhysicsEngine::Mujoco => {\nreturn Err(LoadError::BackendUnavailable(engine));\n}\n'
        (self.root / QUALIFIER.SCENE).parent.mkdir(parents=True, exist_ok=True)
        (self.root / QUALIFIER.SCENE).write_text(source)
        for name in ["commands.json", "rustc-coverage.py", "closure.stdout", "closure.stderr", "merged.profdata",
                     "raw/case.profraw", *[test + "." + stream for test, _ in QUALIFIER.TARGETS.values()
                                         for stream in ["stdout", "stderr"]]]:
            self.write(name, b"fixture evidence\n")
        events, objects = [], []
        for name, path in QUALIFIER.EXECUTABLE_SOURCES.items():
            original = self.root / "target/rapier-runtime-fixture" / name
            artifact = {"reason": "compiler-artifact", "executable": str(original), "features": [],
                        "profile": {"test": name != "gz-sim-rs"},
                        "target": {"name": name, "src_path": str(self.root / path),
                                   "kind": ["bin" if name == "gz-sim-rs" else "test"]}}
            events.append(artifact)
            saved = "objects/" + name
            self.write(saved, ("synthetic object for metadata control: " + name).encode())
            if name == "gz-sim-rs":
                (self.directory / saved).chmod(0o755)
            objects.append({"original_path": str(original), "retained_path": saved,
                            "sha256": QUALIFIER.digest(self.directory / saved), "cargo_artifact": artifact})
        self.write_json("cargo-events.json", events)
        self.write_json("objects.json", objects)
        report = {"data": [{"functions": [{"filenames": [str(self.root / QUALIFIER.SCENE)],
                                            "regions": [[3, 1, 5, 2, 2, 0, 0, 0]]}]}]}
        lcov = f"SF:{self.root / QUALIFIER.SCENE}\nDA:4,2\nend_of_record\n"
        self.write_json("coverage.json", report)
        self.write("coverage.lcov", lcov.encode())
        inputs = QUALIFIER.inputs(self.root)
        self.write_json("inputs.json", inputs)
        self.report = {"state": "passed", "private_target_removed": True,
                       "source_root": str(self.root), "target_root": str(self.root / "target/rapier-runtime-fixture"),
                       "sources_and_inputs": inputs, "objects_bound_after_execution": objects,
                       "disabled_backend": QUALIFIER.disabled_region(report, lcov, self.root, source)}
        # Synthetic metadata only: these records exercise the independent RGB
        # validator, not a compiler, executable, renderer, or coverage workload.
        for name in ["stderr", "merged.profdata", "raw/rgb.profraw", "closure.stderr"]:
            self.write("rgb-camera/" + name, b"synthetic RGB evidence\n")
        self.write_json("rgb-camera/inputs.json", inputs)
        self.write_json("rgb-camera/command.json", QUALIFIER.rgb_command())
        self.write("rgb-camera/stdout", ("\n".join("test " + name + " ... ok" for name in QUALIFIER.RGB_TESTS)
                   + f"\ntest result: ok. {len(QUALIFIER.RGB_TESTS)} passed; 0 failed; 0 ignored;\n").encode())
        rgb_sources = [str(self.root / name) for name in ["crates/gz-sim-cli/src/main.rs", "crates/gz-sim-cli/src/live.rs"]]
        self.write_json("rgb-camera/coverage.json", {"data": [{"files": [{"filename": name} for name in rgb_sources]}]})
        self.write("rgb-camera/coverage.lcov", "".join(f"SF:{name}\nend_of_record\n" for name in rgb_sources).encode())
        rgb_events, rgb_objects = [], []
        for name, path in QUALIFIER.RGB_SOURCES.items():
            original = self.root / "target/rapier-rgb-fixture" / name
            event = {"reason": "compiler-artifact", "executable": str(original), "features": ["rgb-camera"],
                     "profile": {"test": name != "gz-sim-rs"},
                     "target": {"name": name, "src_path": str(self.root / path),
                                "kind": ["bin" if name == "gz-sim-rs" else "test"]}}
            rgb_events.append(event)
            saved = "rgb-camera/objects/" + name
            self.write(saved, ("synthetic RGB object: " + name).encode())
            rgb_objects.append({"original_path": str(original), "retained_path": saved,
                                "sha256": QUALIFIER.digest(self.directory / saved), "cargo_artifact": event})
        self.write_json("rgb-camera/cargo-events.json", rgb_events)
        self.write_json("rgb-camera/objects.json", rgb_objects)
        packages = ["gz-sim-cli", "gz-sim-server", "gz-sim-scene", "gz-physics-rapier", "gz-physics-tpe",
                    "gz-camera-output", "gz-render-client", "gz-render-protocol", "gz-render-scene", "gz-render-frame"]
        self.write_json("rgb-camera/closure.stdout", {"binary": str(self.root / "target/rapier-rgb-fixture/gz-sim-rs"),
                        "packages": packages, "libraries": "libc.so.6 => /lib/libc.so.6 (0x1)\n",
                        "cargo_command": ["cargo", "tree", "--locked", "--offline", "-p", "gz-sim-cli", "--no-default-features",
                                          "--edges", "normal,build", "--prefix", "none", "--format", "{p}", "--features", "rgb-camera"]})
        self.report["rgb_camera"] = {"state": "passed", "features": ["rgb-camera"],
                                     "target_root": str(self.root / "target/rapier-rgb-fixture"),
                                     "private_target_removed": True, "objects_bound_after_execution": rgb_objects}
        # Synthetic receipts exercise validation only, never claim actual CLI runs.
        cli = next(item for item in objects if item["cargo_artifact"]["target"]["name"] == "gz-sim-rs")
        noisy = {"schema_version": 1, "source_directory": str(self.directory),
                 "binary": {"retained_path": cli["retained_path"], "sha256": cli["sha256"], "mode": 0o755}, "runs": []}
        for source in QUALIFIER.NOISY_EXAMPLES:
            saved = "examples/" + Path(source).name
            self.write(saved, (self.root / source).read_bytes())
            noisy["runs"].append({"source": source, "retained_fixture": saved,
                                  "fixture_sha256": QUALIFIER.digest(self.root / source),
                                  "fixture_mode": (self.root / source).stat().st_mode & 0o777,
                                  "argv": QUALIFIER.noisy_command(self.directory / cli["retained_path"], self.directory / saved),
                                  "actual_exit_code": 0, "stdout": (QUALIFIER.NOISY_OUTPUT + "".join(
                                      name + " " + " ".join(map(str, xyz)) + "\n"
                                      for name, xyz in sorted(QUALIFIER.NOISY_LINKS[source].items()))), "stderr": ""})
        self.report["noisy_examples"] = noisy
        self.write_json("noisy-examples.json", noisy)
        self.finish_manifest()

    def write(self, name, data):
        path = self.directory / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)

    def write_json(self, name, value):
        self.write(name, (json.dumps(value) + "\n").encode())

    def finish_manifest(self):
        self.report["artifacts_sha256"] = {str(p.relative_to(self.directory)): QUALIFIER.digest(p)
                                           for p in self.directory.rglob("*")
                                           if p.is_file() and p.name != "result.json"}
        self.write_json("result.json", self.report)

    def test_retains_objects_reports_and_raw_after_only_private_target_is_gone(self):
        output = Path(self.temporary.name) / "retained"
        output.mkdir()
        hashes, missing = {}, []
        VERIFY.retain_rapier_evidence(self.root, output, hashes, missing)
        self.assertEqual(missing, [])
        self.assertFalse((self.directory / "target").exists())
        self.assertTrue("rapier-runtime/result.json" in hashes)
        for name, digest in hashes.items():
            self.assertEqual(QUALIFIER.digest(output / name), digest)
        self.assertEqual(len(list((output / "rapier-runtime/objects").iterdir())), 4)
        self.assertEqual(len(list((output / "rapier-runtime/rgb-camera/objects").iterdir())), 2)

    def test_original_features_off_proof_does_not_replace_the_rgb_proof(self):
        del self.report["rgb_camera"]
        self.finish_manifest()
        with self.assertRaisesRegex(ValueError, "incomplete native-free RGB"):
            QUALIFIER.validate_evidence(self.root, self.directory)

    def test_rgb_raw_and_object_bytes_are_required_independently(self):
        for name in ["rgb-camera/raw/rgb.profraw", "rgb-camera/objects/rgb_camera"]:
            path = self.directory / name
            original = path.read_bytes()
            path.unlink()
            self.finish_manifest()
            with self.assertRaises(ValueError):
                QUALIFIER.validate_evidence(self.root, self.directory)
            path.write_bytes(original)
        self.finish_manifest()
        self.write("rgb-camera/objects/gz-sim-rs", b"tampered RGB binary")
        with self.assertRaisesRegex(ValueError, "tampered native-off artifact"):
            QUALIFIER.validate_evidence(self.root, self.directory)

    def test_rehashed_rgb_feature_or_gpu_closure_claim_is_not_accepted(self):
        path = self.directory / "rgb-camera/cargo-events.json"
        baseline = path.read_bytes()
        for features in [[], ["native-backends", "rgb-camera"]]:
            events = json.loads(baseline)
            events[0]["features"] = features
            path.write_text(json.dumps(events))
            self.finish_manifest()
            with self.assertRaises(ValueError):
                QUALIFIER.validate_evidence(self.root, self.directory)
        path.write_bytes(baseline)
        closure = json.loads((self.directory / "rgb-camera/closure.stdout").read_text())
        closure["packages"].append("wgpu-core")
        self.write_json("rgb-camera/closure.stdout", closure)
        self.finish_manifest()
        with self.assertRaisesRegex(ValueError, "RGB closure"):
            QUALIFIER.validate_evidence(self.root, self.directory)

    def test_rehashed_rgb_command_and_report_attribution_cannot_change(self):
        command = QUALIFIER.rgb_command()
        command.remove("--no-default-features")
        self.write_json("rgb-camera/command.json", command)
        self.finish_manifest()
        with self.assertRaisesRegex(ValueError, "RGB command differs"):
            QUALIFIER.validate_evidence(self.root, self.directory)
        self.write_json("rgb-camera/command.json", QUALIFIER.rgb_command())
        self.write_json("rgb-camera/coverage.json", {"data": [{"files": [{"filename": "/different/source.rs"}]}]})
        self.finish_manifest()
        with self.assertRaisesRegex(ValueError, "reports lack their actual CLI source"):
            QUALIFIER.validate_evidence(self.root, self.directory)

    def test_missing_raw_or_result_prevents_complete_retention(self):
        (self.directory / "raw/case.profraw").unlink()
        with self.assertRaises(ValueError):
            QUALIFIER.validate_evidence(self.root, self.directory)
        output = Path(self.temporary.name) / "retained"
        output.mkdir()
        missing = []
        VERIFY.retain_rapier_evidence(self.root, output, {}, missing)
        self.assertTrue(missing)
        (self.directory / "result.json").unlink()
        with self.assertRaises(FileNotFoundError):
            QUALIFIER.validate_evidence(self.root, self.directory)

    def test_passed_staged_checks_cannot_accept_missing_auxiliary_proof(self):
        (self.directory / "raw/case.profraw").unlink()
        with patch.object(VERIFY, "ROOT", Path(self.temporary.name) / "outer"):
            with self.assertRaises(ValueError):
                VERIFY.retain_staged_evidence("a" * 40, self.root, "passed")
            result = next((VERIFY.ROOT / "artifacts/staged").glob("*/stage-result.json"))
            report = json.loads(result.read_text())
            self.assertEqual(report["evidence_state"], "partial")
            self.assertTrue(any("rapier-runtime" in name for name in report["missing_artifacts"]))

    def test_tampered_retained_object_is_not_a_valid_proof(self):
        self.write("objects/gz-sim-rs", b"different bytes")
        with self.assertRaises(ValueError):
            QUALIFIER.validate_evidence(self.root, self.directory)

    def test_rehashed_report_cannot_omit_the_server_build_test_object(self):
        objects = [item for item in self.report["objects_bound_after_execution"]
                   if item["cargo_artifact"]["target"]["name"] != "no_native_build"]
        self.report["objects_bound_after_execution"] = objects
        self.write_json("objects.json", objects)
        self.write_json("cargo-events.json", [item["cargo_artifact"] for item in objects])
        (self.directory / "objects/no_native_build").unlink()
        self.finish_manifest()
        with self.assertRaisesRegex(ValueError, "missing retained CLI or targeted test object"):
            QUALIFIER.validate_evidence(self.root, self.directory)

    def test_source_or_build_input_change_invalidates_evidence(self):
        for name in ["Cargo.lock", QUALIFIER.SCENE, "third_party/prost-derive-0.14.4/Cargo.toml",
                     "third_party/prost-derive-0.14.4/LICENSE", "third_party/prost-derive-0.14.4/src/lib.rs",
                     *QUALIFIER.NOISY_EXAMPLES]:
            path = self.root / name
            old = path.read_bytes()
            path.write_bytes(old + b"changed\n")
            with self.assertRaises(ValueError):
                QUALIFIER.validate_evidence(self.root, self.directory)
            path.write_bytes(old)

    def test_retainer_uses_the_exported_validator_source(self):
        marker = self.root / "scripts/qualify-rapier-runtime.py"
        marker.write_text(marker.read_text() +
                          '\ndef validate_evidence(root, directory, report=None):\n'
                          '    raise ValueError("candidate-validator-witness")\n')
        output = Path(self.temporary.name) / "retained"
        output.mkdir()
        missing = []
        VERIFY.retain_rapier_evidence(self.root, output, {}, missing)
        self.assertTrue(any("candidate-validator-witness" in item for item in missing))

    def test_rehashed_original_object_paths_cannot_escape_or_use_noncanonical_spelling(self):
        baseline = (self.directory / "objects.json").read_text()
        target = self.report["target_root"]
        for original in [target + "/../outside", "target/rapier-runtime-fixture/gz-sim-rs",
                         target + "/./gz-sim-rs"]:
            objects = json.loads(baseline)
            objects[0]["original_path"] = original
            objects[0]["cargo_artifact"]["executable"] = str(Path(original))
            self.report["objects_bound_after_execution"] = objects
            self.write_json("objects.json", objects)
            self.write_json("cargo-events.json", [item["cargo_artifact"] for item in objects])
            self.finish_manifest()
            with self.assertRaises(ValueError):
                QUALIFIER.validate_evidence(self.root, self.directory)

    def test_symlink_artifact_or_parent_is_rejected_before_copying(self):
        outside = Path(self.temporary.name) / "outside"
        outside.write_bytes(b"not owned evidence")
        path = self.directory / "objects/gz-sim-rs"
        path.unlink()
        path.symlink_to(outside)
        with self.assertRaises(ValueError):
            QUALIFIER.validate_evidence(self.root, self.directory)
        self.assertEqual(outside.read_bytes(), b"not owned evidence")
        path.unlink()
        moved = Path(self.temporary.name) / "moved-objects"
        (self.directory / "objects").rename(moved)
        (self.directory / "objects").symlink_to(moved, target_is_directory=True)
        with self.assertRaises(ValueError):
            QUALIFIER.validate_evidence(self.root, self.directory)

    def test_manifest_escape_and_reintroduced_target_cache_are_rejected(self):
        self.report["artifacts_sha256"]["../escape"] = "0" * 64
        self.write_json("result.json", self.report)
        with self.assertRaises(ValueError):
            QUALIFIER.validate_evidence(self.root, self.directory)
        self.finish_manifest()
        (self.directory / "target").mkdir()
        with self.assertRaises(ValueError):
            QUALIFIER.validate_evidence(self.root, self.directory)

    def test_rehashed_but_wrong_cargo_source_or_enabled_feature_is_refused(self):
        events = json.loads((self.directory / "cargo-events.json").read_text())
        events.append({"reason": "compiler-artifact", "features": ["native-backends"],
                       "target": {"name": "gz_sim_scene"}})
        self.write_json("cargo-events.json", events)
        self.finish_manifest()
        with self.assertRaises(ValueError):
            QUALIFIER.validate_evidence(self.root, self.directory)
        events.pop()
        objects = json.loads((self.directory / "objects.json").read_text())
        objects[0]["cargo_artifact"]["target"]["src_path"] = str(self.root / "crates/wrong/src/main.rs")
        events[0] = objects[0]["cargo_artifact"]
        self.report["objects_bound_after_execution"] = objects
        self.write_json("objects.json", objects)
        self.write_json("cargo-events.json", events)
        self.finish_manifest()
        with self.assertRaises(ValueError):
            QUALIFIER.validate_evidence(self.root, self.directory)

    def test_each_noisy_example_requires_its_real_command_output_and_zero_exit(self):
        baseline = json.dumps(self.report["noisy_examples"])
        for field, wrong in [("actual_exit_code", 1), ("stdout", "simulation_ns=1000000 iterations=1\n"),
                             ("stderr", "load failed\n"), ("stdout", QUALIFIER.NOISY_OUTPUT),
                             ("stdout", QUALIFIER.NOISY_OUTPUT + "default::rig::body 0 0 4\n"),
                             ("stdout", QUALIFIER.NOISY_OUTPUT + "default::rig::body 0 0 3\ndefault::extra::body 0 0 0\n"),
                             ("argv", ["/different/binary"]),
                             ("source", QUALIFIER.NOISY_EXAMPLES[-1])]:
            with self.subTest(field=field):
                record = json.loads(baseline)
                record["runs"][0][field] = wrong
                self.report["noisy_examples"] = record
                self.write_json("noisy-examples.json", record)
                self.finish_manifest()
                with self.assertRaisesRegex(ValueError, "noisy-example"):
                    QUALIFIER.validate_evidence(self.root, self.directory)
        record = json.loads(baseline)
        record["runs"].pop()
        self.report["noisy_examples"] = record
        self.write_json("noisy-examples.json", record)
        self.finish_manifest()
        with self.assertRaisesRegex(ValueError, "noisy-example executions"):
            QUALIFIER.validate_evidence(self.root, self.directory)

    def test_rehashed_noisy_fixture_bytes_modes_and_binary_claim_cannot_be_substituted(self):
        copied = self.directory / "examples" / Path(QUALIFIER.NOISY_EXAMPLES[0]).name
        original = copied.read_bytes()
        copied.write_bytes(b"different but rehashed SDF")
        self.finish_manifest()
        with self.assertRaisesRegex(ValueError, "noisy-example fixture identity"):
            QUALIFIER.validate_evidence(self.root, self.directory)
        copied.write_bytes(original)
        copied.chmod(0o600)
        self.finish_manifest()
        with self.assertRaisesRegex(ValueError, "noisy-example fixture identity"):
            QUALIFIER.validate_evidence(self.root, self.directory)
        copied.chmod((self.root / QUALIFIER.NOISY_EXAMPLES[0]).stat().st_mode & 0o777)
        self.report["noisy_examples"]["binary"]["sha256"] = "0" * 64
        self.write_json("noisy-examples.json", self.report["noisy_examples"])
        self.finish_manifest()
        with self.assertRaisesRegex(ValueError, "noisy-example copied CLI identity"):
            QUALIFIER.validate_evidence(self.root, self.directory)

    def test_missing_noisy_receipt_or_copied_fixture_cannot_be_retained(self):
        for name in ["noisy-examples.json", "examples/" + Path(QUALIFIER.NOISY_EXAMPLES[0]).name]:
            path = self.directory / name
            original = path.read_bytes()
            path.unlink()
            self.finish_manifest()
            with self.assertRaises(ValueError):
                QUALIFIER.validate_evidence(self.root, self.directory)
            path.write_bytes(original)

    def test_new_preview_examples_cannot_reuse_another_run_or_omit_evidence(self):
        baseline = json.dumps(self.report["noisy_examples"])
        for index in range(4, 7):
            for field, wrong in [("actual_exit_code", 1), ("stderr", "failed\n"),
                                 ("stdout", QUALIFIER.NOISY_OUTPUT),
                                 ("argv", self.report["noisy_examples"]["runs"][0]["argv"]),
                                 ("source", QUALIFIER.NOISY_EXAMPLES[0]),
                                 ("fixture_sha256", "0" * 64), ("fixture_mode", 0o600)]:
                with self.subTest(source=QUALIFIER.NOISY_EXAMPLES[index], field=field):
                    record = json.loads(baseline)
                    record["runs"][index][field] = wrong
                    self.report["noisy_examples"] = record
                    self.write_json("noisy-examples.json", record)
                    self.finish_manifest()
                    with self.assertRaisesRegex(ValueError, "noisy-example"):
                        QUALIFIER.validate_evidence(self.root, self.directory)
            record = json.loads(baseline)
            record["runs"].pop(index)
            self.report["noisy_examples"] = record
            self.write_json("noisy-examples.json", record)
            self.finish_manifest()
            with self.assertRaisesRegex(ValueError, "noisy-example executions"):
                QUALIFIER.validate_evidence(self.root, self.directory)
        self.report["noisy_examples"] = json.loads(baseline)
        self.write_json("noisy-examples.json", self.report["noisy_examples"])
        self.finish_manifest()

    def test_pose_output_tolerance_cannot_admit_a_rehashed_wrong_retained_pose(self):
        record = self.report["noisy_examples"]
        run = next(row for row in record["runs"] if row["source"] == "examples/m27_pose_publisher.sdf")
        run["stdout"] = run["stdout"].replace("default::rig::base 1 2 3", "default::rig::base 1.000000000002 2 3")
        self.write_json("noisy-examples.json", record)
        self.finish_manifest()
        with self.assertRaisesRegex(ValueError, "noisy-example execution"):
            QUALIFIER.validate_evidence(self.root, self.directory)
