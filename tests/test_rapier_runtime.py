# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import importlib.util
from pathlib import Path
import unittest

PATH = Path(__file__).resolve().parents[1] / "scripts/check-rapier-runtime.py"
SPEC = importlib.util.spec_from_file_location("rapier_runtime_closure", PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class RapierRuntimeClosure(unittest.TestCase):
    def setUp(self):
        self.tree = "\n".join(name + " v0.1.0" for name in MODULE.REQUIRED_PACKAGES)

    def test_complete_runtime_closure_requires_both_available_engines(self):
        self.assertEqual(MODULE.validate_closure(self.tree, "libc.so.6 => /lib/libc.so.6"),
                         sorted(MODULE.REQUIRED_PACKAGES))
        with self.assertRaises(ValueError):
            MODULE.validate_closure(self.tree.replace("gz-physics-tpe", "wrong-backend"), "")

    def test_either_native_package_in_transitive_closure_is_refused(self):
        for package in MODULE.NATIVE_PACKAGES:
            with self.assertRaises(ValueError):
                MODULE.validate_closure(self.tree + "\n" + package + " v0.1.0", "")

    def test_native_shared_objects_or_sdk_libraries_are_refused(self):
        for library in (*MODULE.NATIVE_LIBRARIES, "libgz-transport", "libgz-msgs", "libgz-math"):
            with self.assertRaises(ValueError):
                MODULE.validate_closure(self.tree, library + ".so => /opt/sdk/lib.so")

    def test_unresolved_dynamic_libraries_are_not_a_closure_pass(self):
        with self.assertRaises(ValueError):
            MODULE.validate_closure(self.tree, "libsomething.so => not found")
