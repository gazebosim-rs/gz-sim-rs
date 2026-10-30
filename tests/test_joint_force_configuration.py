# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import hashlib
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class JointForceTextFixture(unittest.TestCase):
    def test_native_inputs_are_bound_to_source_and_pinned_repositories(self):
        fixture = json.loads((ROOT / 'crates/gz-sim-scene/tests/fixtures/joint_force_text.json').read_text())
        baseline = {v['name']: v['commit'] for v in
                    json.loads((ROOT / 'planning/baseline.json').read_text())['repositories']}
        for key, file in [('source_sha256', 'crates/gz-sim-scene/tests/fixtures/joint_force_text.cc'),
                          ('script_sha256', 'scripts/probe-joint-force-text.py')]:
            self.assertEqual(fixture[key], hashlib.sha256((ROOT / file).read_bytes()).hexdigest())
        self.assertEqual(fixture['sdformat_commit'], baseline['sdformat'])
        self.assertEqual(fixture['sim_commit'], baseline['gz-sim'])
        rows = fixture['observations']
        self.assertEqual(len(rows), 14)
        self.assertEqual(len({r['name'] for r in rows}), len(rows))
        self.assertTrue(all(r['stdout'] for r in rows))
