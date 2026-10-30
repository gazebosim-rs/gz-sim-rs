# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import copy
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("dart_reference", ROOT / "reference/build-dart-reference.py")
dart = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dart)


class DartReferenceBuildTests(unittest.TestCase):
    def setUp(self):
        self.lock = json.loads((ROOT / "reference/headless-dart.lock.json").read_bytes())

    def copy_metadata(self, root):
        for name in self.lock["metadata_sha256"]:
            path = root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / name, path)

    def test_actual_signed_binary_and_source_indexes_bind_selected_dependency(self):
        result = dart.verify_metadata(ROOT, self.lock)
        self.assertEqual(result["packages"], 10)
        self.assertEqual(result["source"]["Package"], "dart")
        self.assertEqual(result["source"]["Version"], dart.VERSION)
        self.assertEqual(result["signing_fingerprint"], "D2486D2DD83DB69272AFE98867170598AF249743")
        self.assertIn("dart_6.13.2+ds1.orig.tar.xz", result["source"]["Checksums-Sha256"])

    def test_changed_metadata_bytes_or_signature_are_not_qualified(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.copy_metadata(root)
            name = "reference/osrf-dart/InRelease"
            path = root / name
            path.write_bytes(path.read_bytes().replace(b"Origin: OSRF", b"Origin: FAKE"))
            with self.assertRaisesRegex(ValueError, "metadata bytes"):
                dart.verify_metadata(root, self.lock)
            changed = copy.deepcopy(self.lock)
            changed["metadata_sha256"][name] = dart.reference.digest(path)
            with self.assertRaises(subprocess.CalledProcessError):
                dart.verify_metadata(root, changed)

    def test_mutated_package_hash_version_source_or_selection_fails_signed_binding(self):
        for key, value in [("SHA256", "0" * 64), ("Version", "6.14.0"), ("Source", "other"),
                           ("Filename", "pool/main/d/dart/../../outside.deb")]:
            changed = copy.deepcopy(self.lock)
            changed["packages"][0][key] = value
            with self.subTest(field=key), self.assertRaises(ValueError):
                dart.verify_metadata(ROOT, changed)
        for key, value in [("builds", []), ("builds", ["gz-sim"]), ("DART_DIR", "/opt/ros/dart"),
                           ("prefix", "/reference/install"), ("osrf_url", "http://example.invalid/")]:
            changed = copy.deepcopy(self.lock)
            changed[key] = value
            with self.subTest(field=key), self.assertRaises(ValueError):
                dart.validate_lock(changed)
        changed = copy.deepcopy(self.lock)
        changed["source_package"]["Version"] = "6.14.0"
        with self.assertRaisesRegex(ValueError, "source-package"):
            dart.verify_metadata(ROOT, changed)

    def test_recipe_must_use_exact_base_and_payload_checksums(self):
        recipe = (ROOT / "reference/Dockerfile.dart").read_text()
        dart.verify_recipe(recipe, self.lock)
        for changed in [recipe.replace(self.lock["base_image_tag"], "ubuntu:latest"),
                        recipe.replace(self.lock["packages"][0]["SHA256"], "0" * 64),
                        recipe + "ADD https://example.invalid/unqualified.deb /tmp/extra.deb\n"]:
            with self.assertRaisesRegex(ValueError, "recipe"):
                dart.verify_recipe(changed, self.lock)

    def test_wrong_missing_or_linked_deb_fails_before_installation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            package = self.lock["packages"][0]
            path = root / Path(package["Filename"]).name
            with patch.object(dart.subprocess, "check_output") as command:
                with self.assertRaisesRegex(ValueError, "payload"):
                    dart.verify_debs(root, self.lock)
                path.write_bytes(b"not the signed payload")
                with self.assertRaisesRegex(ValueError, "payload"):
                    dart.verify_debs(root, self.lock)
                path.unlink()
                path.symlink_to(ROOT / "reference/headless-dart.lock.json")
                with self.assertRaisesRegex(ValueError, "payload"):
                    dart.verify_debs(root, self.lock)
            command.assert_not_called()

    def test_no_ros_compiler_or_search_overrides_survive(self):
        overrides = {key: "/opt/ros/decoy" for key in ["CMAKE_PREFIX_PATH", "PKG_CONFIG_PATH", "LD_LIBRARY_PATH",
            "CPLUS_INCLUDE_PATH", "CMAKE_TOOLCHAIN_FILE", "CMAKE_FIND_ROOT_PATH", "ROS_DISTRO", "AMENT_PREFIX_PATH",
            "COLCON_PREFIX_PATH", "PYTHONPATH", "PYTHONHOME", "GZ_PHYSICS_ENGINE_PATH", "LD_PRELOAD", "LD_AUDIT"]}
        with patch.dict(os.environ, overrides):
            env = dart.build_environment(Path("/reference/dart-install"), Path("/reference/install"))
        for key in overrides:
            self.assertNotIn("/opt/ros", env.get(key, ""), key)
        self.assertEqual(env["CMAKE_PREFIX_PATH"], "/reference/dart-install:/reference/install")
        self.assertNotIn("CMAKE_TOOLCHAIN_FILE", env)

    def physics_cache_values(self):
        return {"CMAKE_HOME_DIRECTORY": "/reference/sources/gz-physics", "CMAKE_INSTALL_PREFIX": "/reference/dart-install",
                  "CMAKE_BUILD_TYPE": "Release", "BUILD_TESTING": "ON", "SKIP_dartsim": "OFF", "SKIP_bullet": "ON",
                  "SKIP_bullet-featherstone": "ON", "SKIP_mujoco": "ON", "DART_DIR": "/usr/share/dart/cmake",
                  "DART_INCLUDE_DIRS": "/usr/include", "DART_HAS_CONTACT_SURFACE_HEADER": "1",
                  "CMAKE_C_COMPILER": "/usr/bin/gcc", "CMAKE_CXX_COMPILER": "/usr/bin/g++"}

    def test_dart_cache_cannot_disable_tests_or_use_ros_headers(self):
        values = self.physics_cache_values()
        def cache(items):
            return "\n".join(key + ":STRING=" + value for key, value in items.items())
        source, prefix = Path(values["CMAKE_HOME_DIRECTORY"]), Path(values["CMAKE_INSTALL_PREFIX"])
        dart.check_cache(cache(values), source, prefix, True)
        for key, value in [("BUILD_TESTING", "OFF"), ("SKIP_dartsim", "ON"), ("CMAKE_BUILD_TYPE", "Debug"),
                           ("DART_DIR", "/opt/ros/jazzy/share/dart/cmake"), ("DART_INCLUDE_DIRS", "/opt/ros/include"),
                           ("DART_HAS_CONTACT_SURFACE_HEADER", ""), ("CMAKE_CXX_COMPILER", "/opt/ros/c++")]:
            with self.subTest(field=key), self.assertRaises(ValueError):
                dart.check_cache(cache(values | {key: value}), source, prefix, True)

    def test_duplicate_raw_cache_key_cannot_hide_an_earlier_value(self):
        complete = "\n".join(key + ":STRING=" + value for key, value in self.physics_cache_values().items())
        for earlier in ["Release", "Debug"]:
            cache = "CMAKE_BUILD_TYPE:STRING=" + earlier + "\n" + complete
            with self.subTest(earlier=earlier), self.assertRaisesRegex(ValueError, "duplicate raw DART cache key"):
                dart.check_cache(cache, Path("/reference/sources/gz-physics"), Path("/reference/dart-install"), True)


if __name__ == "__main__":
    unittest.main()
