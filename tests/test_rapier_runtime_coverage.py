# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import importlib.util
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
