# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Reject stale source or baseline identities in the native effort oracle."""
import hashlib
import importlib.util
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class JointEffortReference(unittest.TestCase):
    def test_native_fixture_binds_peer_source_and_physics_baseline(self):
        directory = ROOT / 'crates/gz-physics-dart/tests/fixtures'
        fixture = json.loads((directory / 'joint_effort.json').read_text())
        self.assertEqual(fixture['source_sha256'], hashlib.sha256((directory / 'joint_effort.cc').read_bytes()).hexdigest())
        baseline = json.loads((ROOT / 'planning/baseline.json').read_text())
        physics = next(row['commit'] for row in baseline['repositories'] if row['name'] == 'gz-physics')
        self.assertEqual(fixture['physics_commit'], physics)
        self.assertEqual(fixture['dart_version'], '6.13.2')
        self.assertEqual([row['effort'] for row in fixture['observations']], [1.5, 0, -3, 0, 2, 0])

    def test_mujoco_fixture_binds_peer_source_and_sdk_revision(self):
        directory = ROOT / 'crates/gz-physics-mujoco/tests/fixtures'
        fixture = json.loads((directory / 'joint_effort.json').read_text())
        self.assertEqual(fixture['source_sha256'], hashlib.sha256((directory / 'joint_effort.cc').read_bytes()).hexdigest())
        spec = importlib.util.spec_from_file_location('physics_bootstrap', ROOT / 'scripts/bootstrap-physics.py')
        bootstrap = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(bootstrap)
        self.assertEqual(fixture['mujoco_commit'], bootstrap.MUJOCO_COMMIT)
        self.assertEqual(fixture['version'], bootstrap.MUJOCO_VERSION)
        self.assertEqual([row['effort'] for row in fixture['observations']], [1.5, 0, -3, 0, 2, 0])
