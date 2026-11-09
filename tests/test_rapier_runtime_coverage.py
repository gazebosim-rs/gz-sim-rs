# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import importlib.util
import json
import subprocess
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location("qualify_rapier", Path(__file__).resolve().parents[1] / "scripts/qualify-rapier-runtime.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class DisabledBackendProof(unittest.TestCase):
    def setUp(self):
        self.root = Path("/source")
        self.path = str(self.root / MODULE.SCENE)
        self.source = '\n#[cfg(not(feature = "native-backends"))]\nPhysicsEngine::Dart | PhysicsEngine::Mujoco => {\nreturn Err(LoadError::BackendUnavailable(engine));\n}\n'
        self.lcov = f"SF:{self.path}\nDA:4,2\nend_of_record\n"

    def report(self, region):
        return {"data": [{"functions": [{"filenames": [self.path], "regions": [region]}]}]}

    def test_actual_line_and_bounded_code_region_are_required_together(self):
        result = MODULE.disabled_region(self.report([3, 1, 5, 2, 2, 0, 0, 0]), self.lcov, self.root, self.source)
        self.assertEqual((result["line"], result["line_hits"]), (4, 2))

    def test_live_outer_function_cannot_stand_in_for_cold_return(self):
        for region in [[1, 1, 9, 1, 12, 0, 0, 0], [3, 1, 5, 2, 0, 0, 0, 0]]:
            with self.assertRaises(ValueError):
                MODULE.disabled_region(self.report(region), self.lcov, self.root, self.source)

    def test_gap_expansion_or_other_file_is_not_the_return_code_region(self):
        for region in [[3, 1, 5, 2, 2, 0, 0, 1], [3, 1, 5, 2, 2, 0, 0, 3]]:
            with self.assertRaises(ValueError):
                MODULE.disabled_region(self.report(region), self.lcov, self.root, self.source)
        self.path = "/other/lib.rs"
        with self.assertRaises(ValueError):
            MODULE.disabled_region(self.report([3, 1, 5, 2, 2, 0, 0, 0]), self.lcov, self.root, self.source)

    def test_cold_missing_or_duplicate_line_cannot_be_reported_as_execution(self):
        for lcov in [self.lcov.replace("4,2", "4,0"), self.lcov.replace("DA:4,2\n", ""),
                     self.lcov.replace("DA:4,2", "DA:4,2\nDA:4,2")]:
            with self.assertRaises(ValueError):
                MODULE.disabled_region(self.report([3, 1, 5, 2, 2, 0, 0, 0]), lcov, self.root, self.source)

    def test_native_feature_or_package_cannot_masquerade_as_a_reduced_build(self):
        for name, features in [("gz_sim_scene", ["native-backends"]),
                               ("gz_physics_dart", []), ("gz_physics_mujoco", [])]:
            event = {"reason": "compiler-artifact", "target": {"name": name}, "features": features}
            with self.assertRaises(ValueError):
                MODULE.checked_artifacts([event], Path("/target"))

    def test_missing_actual_test_objects_are_not_configuration_evidence(self):
        with self.assertRaises(ValueError):
            MODULE.checked_artifacts([], Path("/target"))

    def test_each_owning_crate_test_executable_is_required(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / "target"
            target.mkdir()
            events = []
            for name, source in MODULE.EXECUTABLE_SOURCES.items():
                binary = target / name
                binary.write_bytes(b"metadata fixture, not executable evidence")
                events.append({"reason": "compiler-artifact", "features": [],
                               "executable": str(binary),
                               "target": {"name": name, "src_path": str(root / source),
                                          "kind": ["bin" if name == "gz-sim-rs" else "test"]}})
            with patch.object(MODULE, "ROOT", root):
                self.assertEqual(len(MODULE.checked_artifacts(events, target)), 4)
                for missing in MODULE.EXECUTABLE_SOURCES:
                    with self.subTest(missing=missing), self.assertRaises(ValueError):
                        MODULE.checked_artifacts(
                            [event for event in events if event["target"]["name"] != missing], target)

    def test_rgb_requires_every_named_case_without_failure_or_skip(self):
        lines = "\n".join("test " + name + " ... ok" for name in MODULE.RGB_TESTS)
        summary = f"\ntest result: ok. {len(MODULE.RGB_TESTS)} passed; 0 failed; 0 ignored;"
        MODULE.require_rgb_tests(lines + summary)
        for wrong in [lines.replace("test " + MODULE.RGB_TESTS[0] + " ... ok", ""),
                      (lines + summary).replace("0 ignored", "1 ignored")]:
            with self.assertRaises(ValueError):
                MODULE.require_rgb_tests(wrong if "test result:" in wrong else wrong + summary)

    def test_rgb_executables_bind_feature_source_and_both_actual_targets(self):
        target = Path("/source/target/rapier-rgb-fixture")
        events = [{"reason": "compiler-artifact", "features": ["rgb-camera"],
                   "executable": str(target / name), "profile": {"test": name != "gz-sim-rs"},
                   "target": {"name": name, "src_path": str(self.root / source),
                              "kind": ["bin" if name == "gz-sim-rs" else "test"]}}
                  for name, source in MODULE.RGB_SOURCES.items()]
        self.assertEqual(set(MODULE.rgb_artifacts(events, self.root, target)), set(MODULE.RGB_SOURCES))
        for missing in MODULE.RGB_SOURCES:
            with self.assertRaises(ValueError):
                MODULE.rgb_artifacts([event for event in events if event["target"]["name"] != missing], self.root, target)
        for features in [[], ["native-backends", "rgb-camera"]]:
            with self.assertRaises(ValueError):
                MODULE.rgb_artifacts([{**events[0], "features": features}, events[1]], self.root, target)
        wrong = {**events[0], "target": {**events[0]["target"], "src_path": "/other/main.rs"}}
        with self.assertRaises(ValueError):
            MODULE.rgb_artifacts([wrong, events[1]], self.root, target)

    def test_native_or_gpu_artifacts_are_not_cpu_rgb_evidence(self):
        for name in ["gz_physics_dart", "gz_physics_mujoco", "gz_renderer", "bevy_render", "wgpu_core", "ash"]:
            event = {"reason": "compiler-artifact", "features": [], "target": {"name": name}}
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, "native/GPU dependency"):
                MODULE.rgb_artifacts([event], self.root, self.root / "target/rapier-rgb-fixture")

    def test_copied_noisy_examples_execute_once_each_and_bind_the_actual_results(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            directory = root / "artifacts/noisy-control"
            binary = directory / "objects/gz-sim-rs"
            binary.parent.mkdir(parents=True)
            binary.write_bytes(b"mocked executable bytes; this test does not launch it")
            binary.chmod(0o755)
            for source in MODULE.NOISY_EXAMPLES:
                fixture = root / source
                fixture.parent.mkdir(exist_ok=True)
                fixture.write_text("fixture Source: " + source)
            objects = [{"retained_path": "objects/gz-sim-rs", "sha256": MODULE.digest(binary),
                        "cargo_artifact": {"target": {"name": "gz-sim-rs"}}}]
            results = [subprocess.CompletedProcess([], 0, MODULE.NOISY_OUTPUT + "".join(
                name + " " + " ".join(map(str, xyz)) + "\n"
                for name, xyz in sorted(MODULE.NOISY_LINKS[source].items())), "")
                for source in MODULE.NOISY_EXAMPLES]
            with patch.object(MODULE.subprocess, "run", side_effect=results) as run:
                record = MODULE.run_noisy_examples(root, directory, objects, {"LLVM_PROFILE_FILE": "/owned/raw"})
            self.assertEqual(run.call_count, 9)
            self.assertEqual([row["source"] for row in record["runs"]], list(MODULE.NOISY_EXAMPLES))
            for call, source in zip(run.call_args_list, MODULE.NOISY_EXAMPLES):
                copied = directory / "examples" / Path(source).name
                self.assertEqual(copied.read_bytes(), (root / source).read_bytes())
                self.assertEqual(call.args[0], [str(binary), str(copied), "--physics-engine", "rapier",
                                               "--iterations", "2", "--step-ns", "1000000"])
                self.assertEqual(call.kwargs["env"], {"LLVM_PROFILE_FILE": "/owned/raw"})
                self.assertEqual(call.kwargs["timeout"], 30)
            self.assertEqual(json.loads((directory / "noisy-examples.json").read_text()), record)

    def test_failed_copied_noisy_example_retains_actual_failure_and_stops_without_later_runs(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            directory = root / "artifacts/noisy-failed"
            binary = directory / "objects/gz-sim-rs"
            binary.parent.mkdir(parents=True)
            binary.write_bytes(b"mocked executable bytes")
            binary.chmod(0o755)
            fixture = root / MODULE.NOISY_EXAMPLES[0]
            fixture.parent.mkdir()
            fixture.write_text("malformed SDF test input")
            objects = [{"retained_path": "objects/gz-sim-rs", "sha256": MODULE.digest(binary),
                        "cargo_artifact": {"target": {"name": "gz-sim-rs"}}}]
            result = subprocess.CompletedProcess([], 1, "", "SDF parse failed\n")
            with patch.object(MODULE.subprocess, "run", return_value=result) as run:
                with self.assertRaisesRegex(ValueError, "did not complete two exact steps"):
                    MODULE.run_noisy_examples(root, directory, objects, {})
            self.assertEqual(run.call_count, 1)
            record = json.loads((directory / "noisy-examples.json").read_text())
            self.assertEqual(len(record["runs"]), 1)
            self.assertEqual(record["runs"][0]["actual_exit_code"], 1)
            self.assertEqual(record["runs"][0]["stderr"], "SDF parse failed\n")

    def test_noisy_outputs_require_exact_headline_and_complete_sorted_link_positions(self):
        source = MODULE.NOISY_EXAMPLES[3]
        valid = MODULE.NOISY_OUTPUT + "default::scanner::body 0 0 0\ndefault::target::body 5 0 0\n"
        self.assertTrue(MODULE.noisy_output_matches(source, valid))
        for wrong in [MODULE.NOISY_OUTPUT, valid.replace("iterations=2", "iterations=1"),
                      valid.replace("default::target::body 5 0 0\n", ""),
                      valid.replace("default::target::body 5", "default::target::body NaN"),
                      valid.replace("default::target::body 5", "default::target::body 6"),
                      valid + "default::extra::body 0 0 0\n",
                      MODULE.NOISY_OUTPUT + "default::target::body 5 0 0\ndefault::scanner::body 0 0 0\n"]:
            with self.subTest(output=wrong):
                self.assertFalse(MODULE.noisy_output_matches(source, wrong))

    def test_pose_example_requires_full_sorted_finite_world_poses_with_only_its_reviewed_bound(self):
        source = "examples/m27_pose_publisher.sdf"
        headline = "simulation_ns=2000000 iterations=2\n"
        base = "default::rig::base 1 2 3\n"
        tip = "default::rig::tip 1.6602684919277426 2.4517139787160285 3\n"
        valid = headline + base + tip
        self.assertTrue(MODULE.noisy_output_matches(source, valid))
        self.assertTrue(MODULE.noisy_output_matches(source, valid.replace("base 1 2", "base 1.0000000000005 2")))
        for wrong in [valid.replace("simulation_ns=2000000", "simulation_ns=1000000"),
                      valid.replace("iterations=2", "iterations=1"), headline,
                      headline + tip + base, headline + base, headline + tip,
                      valid.replace("rig::tip", "rig::wrong"),
                      valid + "default::extra::body 0 0 0\n", valid.rstrip("\n"),
                      valid + "\n", valid.replace("base 1 2", "base 1  2")]:
            with self.subTest(output=wrong):
                self.assertFalse(MODULE.noisy_output_matches(source, wrong))
        rows = [base.split(), tip.split()]
        for row_index in range(2):
            for coordinate in range(1, 4):
                for value in ["nan", "inf", "-inf", "bad",
                              str(float(rows[row_index][coordinate]) + 2e-12),
                              str(float(rows[row_index][coordinate]) - 2e-12), "1e20"]:
                    with self.subTest(row=row_index, coordinate=coordinate, value=value):
                        changed = [list(row) for row in rows]
                        changed[row_index][coordinate] = value
                        wrong = headline + "".join(" ".join(row) + "\n" for row in changed)
                        self.assertFalse(MODULE.noisy_output_matches(source, wrong))

    def test_new_air_speed_examples_keep_exact_integer_pose_checks(self):
        valid = "simulation_ns=2000000 iterations=2\ndefault::rig::body 0 0 3\n"
        for source in ["examples/m26_air_speed.sdf", "examples/m28_air_speed_noise.sdf"]:
            with self.subTest(source=source):
                self.assertTrue(MODULE.noisy_output_matches(source, valid))
                for wrong in [valid.replace("body 0 0 3", "body 0 0 3.0000000000001"),
                              valid.replace("body 0 0 3", "body 0 0 nan"),
                              valid.replace("rig::body", "rig::base"), MODULE.NOISY_OUTPUT]:
                    self.assertFalse(MODULE.noisy_output_matches(source, wrong))

    def test_m32_appends_both_existing_scenes_without_replacing_any_control(self):
        self.assertEqual(MODULE.NOISY_EXAMPLES, (
            "examples/m21_altimeter_noise.sdf", "examples/m22_air_pressure_noise.sdf",
            "examples/m23_magnetometer_noise.sdf", "examples/m24_cpu_lidar_noise.sdf",
            "examples/m26_air_speed.sdf", "examples/m27_pose_publisher.sdf",
            "examples/m28_air_speed_noise.sdf", "examples/m30_logical_camera.sdf",
            "examples/m31_pose_static_publisher.sdf",
        ))

    def test_logical_camera_output_requires_all_three_sorted_exact_model_origin_poses(self):
        source = "examples/m30_logical_camera.sdf"
        headline = "simulation_ns=2000000 iterations=2\n"
        poses = ["default::behind::body -3 0 3\n", "default::rig::body 0 0 3\n",
                 "default::target::body 4 0 3\n"]
        valid = headline + "".join(poses)
        self.assertTrue(MODULE.noisy_output_matches(source, valid))
        for wrong in [headline, valid.replace("simulation_ns=2000000", "simulation_ns=1000000"),
                      valid.replace("iterations=2", "iterations=1"),
                      headline + "".join(reversed(poses)),
                      valid.replace("target::body", "target::wrong"),
                      valid + "default::extra::body 0 0 0\n", valid.rstrip("\n"),
                      valid + "\n", valid.replace("rig::body 0 0", "rig::body 0  0")]:
            with self.subTest(output=wrong):
                self.assertFalse(MODULE.noisy_output_matches(source, wrong))
        for missing in poses:
            with self.subTest(missing=missing):
                self.assertFalse(MODULE.noisy_output_matches(source, valid.replace(missing, "")))
        rows = [row.split() for row in poses]
        for row_index in range(3):
            for coordinate in range(1, 4):
                for value in ["nan", "inf", "-inf", "bad",
                              str(float(rows[row_index][coordinate]) + 1e-13)]:
                    with self.subTest(row=row_index, coordinate=coordinate, value=value):
                        changed = [list(row) for row in rows]
                        changed[row_index][coordinate] = value
                        wrong = headline + "".join(" ".join(row) + "\n" for row in changed)
                        self.assertFalse(MODULE.noisy_output_matches(source, wrong))

    def test_static_pose_output_uses_only_the_existing_composed_geometry_bound(self):
        source = "examples/m31_pose_static_publisher.sdf"
        headline = "simulation_ns=2000000 iterations=2\n"
        base = "default::rig::base 1 2 3\n"
        tip = "default::rig::tip 1.6602684919277426 2.4517139787160285 3\n"
        valid = headline + base + tip
        self.assertTrue(MODULE.noisy_output_matches(source, valid))
        self.assertTrue(MODULE.noisy_output_matches(source, valid.replace("base 1 2", "base 1.0000000000005 2")))
        for wrong in [valid.replace("simulation_ns=2000000", "simulation_ns=1000000"),
                      valid.replace("iterations=2", "iterations=1"), headline,
                      headline + tip + base, headline + base, headline + tip,
                      valid.replace("rig::tip", "rig::wrong"),
                      valid + "default::extra::body 0 0 0\n", valid.rstrip("\n"),
                      valid + "\n", valid.replace("base 1 2", "base 1  2")]:
            with self.subTest(output=wrong):
                self.assertFalse(MODULE.noisy_output_matches(source, wrong))
        rows = [base.split(), tip.split()]
        for row_index in range(2):
            for coordinate in range(1, 4):
                for value in ["nan", "inf", "-inf", "bad",
                              str(float(rows[row_index][coordinate]) + 2e-12),
                              str(float(rows[row_index][coordinate]) - 2e-12), "1e20"]:
                    with self.subTest(row=row_index, coordinate=coordinate, value=value):
                        changed = [list(row) for row in rows]
                        changed[row_index][coordinate] = value
                        wrong = headline + "".join(" ".join(row) + "\n" for row in changed)
                        self.assertFalse(MODULE.noisy_output_matches(source, wrong))
