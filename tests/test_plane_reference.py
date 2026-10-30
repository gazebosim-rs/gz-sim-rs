# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Keep the plane oracle tied to its generator and pinned physics source."""
import hashlib
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class PlaneReference(unittest.TestCase):
    def test_generator_and_baseline_identity(self):
        directory = ROOT / 'crates/gz-physics-dart/tests/fixtures'
        fixture = json.loads((directory / 'planes.json').read_bytes())
        for key, path in (
            ('probe_sha256', directory / 'plane_reference.cc'),
            ('runner_sha256', ROOT / 'scripts/probe-dart-plane.py'),
        ):
            self.assertEqual(fixture[key], hashlib.sha256(path.read_bytes()).hexdigest())
        baseline = json.loads((ROOT / 'planning/baseline.json').read_bytes())
        revision = next(row['commit'] for row in baseline['repositories'] if row['name'] == 'gz-physics')
        self.assertEqual(fixture['source_revision'], revision)
        self.assertEqual(fixture['dart_version'], '6.13.2')
        self.assertIn('dartsim/src/SDFFeatures.cc', fixture['sources_sha256'])
        self.assertIn('libdart.so.6.13.2', fixture['libraries_sha256'])

    def test_complete_physical_and_frame_trajectories(self):
        fixture = json.loads((ROOT / 'crates/gz-physics-dart/tests/fixtures/planes.json').read_bytes())
        rows = fixture['rows']
        cases = {'center', 'small_size', 'large_size', 'outside_cube',
                 'footprint_edge', 'tilted', 'translated'}
        self.assertEqual({row['case'] for row in rows}, cases)
        self.assertEqual(len(rows), 49)
        for case in cases:
            selected = [row for row in rows if row['case'] == case]
            self.assertEqual([row['step'] for row in selected], [0, 1, 10, 100, 500, 1000, 1500])
            for row in selected:
                for name in ('ground', 'ball'):
                    self.assertEqual(set(row[name]), {'position', 'orientation', 'velocity', 'angular_velocity'})
